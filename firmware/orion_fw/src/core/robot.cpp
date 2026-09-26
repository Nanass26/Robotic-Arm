// ORION-6 firmware — implémentation du cœur portable (voir robot.h).
#include "robot.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace orion {

static constexpr uint32_t CFG_MAGIC = 0x4F52494FUL;  // « ORIO »
static constexpr uint16_t CFG_VERSION = 2;

const char* stateName(State s) {
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::READY: return "READY";
    case State::HOMING: return "HOMING";
    case State::RUN: return "RUN";
    case State::HOLD: return "HOLD";
    case State::FAULT: return "FAULT";
    case State::ESTOP: return "ESTOP";
  }
  return "?";
}

uint32_t crc32(const uint8_t* d, uint32_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
  }
  return ~c;
}

void defaultConfig(Config& c) {
  memset(&c, 0, sizeof(c));
  c.magic = CFG_MAGIC;
  c.version = CFG_VERSION;
  c.size = sizeof(Config);
  for (int a = 0; a < NA; a++) {
    c.stepsPerDeg[a] = orion_cfg::STEPS_PER_DEG[a];
    c.minDeg[a] = orion_cfg::MIN_DEG[a];
    c.maxDeg[a] = orion_cfg::MAX_DEG[a];
    c.vmax[a] = orion_cfg::VMAX_DEG_S[a];
    c.amax[a] = orion_cfg::AMAX_DEG_S2[a];
    c.invert[a] = orion_cfg::INVERT_DIR[a] ? 1 : 0;
    c.homeDir[a] = orion_cfg::HOMING_DIR[a] < 0 ? -1 : 1;
    c.homePos[a] = orion_cfg::HOMING_SWITCH_POS[a];
    c.homeSpeed[a] = orion_cfg::HOMING_SPEED[a];
    c.homeSlow[a] = orion_cfg::HOMING_SLOW[a];
    c.homeBackoff[a] = orion_cfg::HOMING_BACKOFF[a];
    c.homePose[a] = orion_cfg::HOME_POSE_DEG[a];
    c.homeOrder[a] = orion_cfg::HOMING_ORDER[a];
    c.homeEnabled[a] = orion_cfg::HOMING_ENABLED[a] ? 1 : 0;
  }
  c.watchdogMs = orion_cfg::WATCHDOG_MS;
  c.streamDelayMs = orion_cfg::STREAM_DELAY_MS;
  c.gripClosedUs = orion_cfg::GRIPPER_US_CLOSED;
  c.gripOpenUs = orion_cfg::GRIPPER_US_OPEN;
  c.crc = crc32(reinterpret_cast<const uint8_t*>(&c), offsetof(Config, crc));
}

bool configValid(const Config& c) {
  if (c.magic != CFG_MAGIC || c.version != CFG_VERSION || c.size != sizeof(Config)) return false;
  if (c.crc != crc32(reinterpret_cast<const uint8_t*>(&c), offsetof(Config, crc))) return false;
  for (int a = 0; a < NA; a++)
    if (!(c.stepsPerDeg[a] > 0) || !(c.vmax[a] > 0) || !(c.amax[a] > 0) || !(c.minDeg[a] < c.maxDeg[a])) return false;
  return true;
}

static uint8_t xorSum(const char* s, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= (uint8_t)s[i];
  return c;
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

Robot::Robot(Hal& hal) : hal_(hal) {
  defaultConfig(cfg_);
  memset(sp_, 0, sizeof(sp_));
}

void Robot::begin() {
  Config tmp;
  if (hal_.loadBlob(&tmp, sizeof(tmp)) && configValid(tmp)) cfg_ = tmp;
  enable(false);
  setState(State::IDLE);
  hal_.setGripperUs(cfg_.gripOpenUs);
  reply("EVT BOOT %s fw%s axes=%d", ORION_ROBOT_NAME, FW_VERSION, NA);
}

// ------------------------------------------------------------------ sorties
void Robot::reply(const char* fmt, ...) {
  char buf[208];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf) - 5, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(buf) - 6) n = (int)sizeof(buf) - 6;
  snprintf(buf + n, 5, "*%02X\n", xorSum(buf, (size_t)n));
  hal_.write(buf);
}

uint32_t Robot::flags() const {
  uint32_t f = 0;
  if (enabled_) f |= F_ENABLED;
  if (homed_) f |= F_HOMED;
  if (motion_ != Motion::NONE) f |= F_MOVING;
  if (state_ == State::FAULT) f |= F_FAULT;
  if (hal_.estopActive()) f |= F_ESTOP_IN;
  if (watchdogTrip_) f |= F_WATCHDOG;
  for (int a = 0; a < NA; a++)
    if (hal_.limitActive((uint8_t)a)) { f |= F_LIMIT; break; }
  if (underrun_) f |= F_UNDERRUN;
  if (limited_) f |= F_LIMITED;
  return f;
}

void Robot::sendStatus() {
  char buf[190];
  int n = snprintf(buf, sizeof(buf), "ST %s %lu", stateName(state_), (unsigned long)now_);
  for (int a = 0; a < NA && n < (int)sizeof(buf) - 16; a++) n += snprintf(buf + n, sizeof(buf) - n, " %.3f", (double)posDeg(a));
  unsigned di = 0;
  for (int k = 0; k < 8; k++)
    if (hal_.readInput((uint8_t)k)) di |= (1u << k);
  snprintf(buf + n, sizeof(buf) - n, " %02lX %02X", (unsigned long)flags(), di);
  reply("%s", buf);
}

void Robot::setState(State s) { state_ = s; }

void Robot::fault(const char* why) {
  motion_ = Motion::NONE;
  for (int a = 0; a < NA; a++) { pDes_[a] = (float)pos_[a] + phase_[a]; vDes_[a] = 0; }
  strncpy(faultMsg_, why, sizeof(faultMsg_) - 1);
  setState(State::FAULT);
  reply("EVT FAULT %s", why);
}

void Robot::enable(bool on) {
  enabled_ = on;
  hal_.enableDrivers(on);
  if (!on) {
    // Moteurs libres : la position n’est plus garantie → nouvelle prise d’origine requise
    homed_ = false;
    motion_ = Motion::NONE;
    for (int a = 0; a < NA; a++) { rate_[a] = 0; vDes_[a] = 0; pDes_[a] = (float)pos_[a]; }
  }
}

void Robot::setPosition(int a, int32_t steps) {
  hal_.irqLock(true);
  pos_[a] = steps;
  phase_[a] = 0;
  rate_[a] = 0;
  hal_.irqLock(false);
  pDes_[a] = lastTarget_[a] = (float)steps;
  vDes_[a] = 0;
}

void Robot::beginMotion(Motion m) {
  for (int a = 0; a < NA; a++) lastTarget_[a] = pDes_[a];
  motion_ = m;
  limited_ = false;
  setState(State::RUN);
}

// ------------------------------------------------------------------ réception
void Robot::onChar(char c) {
  if (c == '\n' || c == '\r') {
    if (lineOverflow_) {
      lineOverflow_ = false;
      reply("ERR 3 ligne trop longue");
    } else if (lineLen_) {
      line_[lineLen_] = 0;
      handleLine(line_);
    }
    lineLen_ = 0;
    return;
  }
  if (lineLen_ < sizeof(line_) - 1) line_[lineLen_++] = c;
  else lineOverflow_ = true;
}

bool Robot::parseFloats(char** tok, int n, float* out) {
  for (int i = 0; i < n; i++) {
    if (!tok[i]) return false;
    char* end = nullptr;
    out[i] = strtof(tok[i], &end);
    if (end == tok[i] || *end || !isfinite(out[i])) return false;
  }
  return true;
}

void Robot::handleLine(char* line) {
  // Somme de contrôle optionnelle « *HH » (XOR des caractères précédents)
  char* star = strrchr(line, '*');
  if (star && strlen(star) == 3 && isxdigit((unsigned char)star[1]) && isxdigit((unsigned char)star[2])) {
    const unsigned v = (unsigned)strtoul(star + 1, nullptr, 16);
    if (xorSum(line, (size_t)(star - line)) != v) { reply("ERR 1 somme de controle"); return; }
    *star = 0;
  }
  char* tok[24] = {nullptr};
  int nt = 0;
  for (char* p = strtok(line, " \t"); p && nt < 24; p = strtok(nullptr, " \t")) tok[nt++] = p;
  if (!nt) return;
  for (char* p = tok[0]; *p; p++) *p = (char)toupper((unsigned char)*p);
  const char* c = tok[0];
  auto is = [&](const char* s) { return strcmp(c, s) == 0; };

  if (is("PING")) { reply("OK PONG %s fw%s axes=%d", ORION_ROBOT_NAME, FW_VERSION, NA); return; }
  if (is("VER")) { reply("OK VER %s", FW_VERSION); return; }
  if (is("STAT")) { sendStatus(); return; }
  if (is("FLT")) { reply("OK FLT %s", faultMsg_[0] ? faultMsg_ : "aucun"); return; }
  if (is("STREAM")) {
    const int hz = nt > 1 ? atoi(tok[1]) : 0;
    statusHz_ = (uint16_t)(hz < 0 ? 0 : hz > 500 ? 500 : hz);
    nextStatus_ = now_;
    reply("OK STREAM %u", statusHz_);
    return;
  }
  if (is("ESTOP")) {
    enable(false);
    setState(State::ESTOP);
    reply("OK ESTOP");
    return;
  }
  if (is("CLR")) {
    if (hal_.estopActive()) { reply("ERR 7 arret d'urgence actif"); return; }
    watchdogTrip_ = underrun_ = limited_ = false;
    faultMsg_[0] = 0;
    if (state_ == State::FAULT || state_ == State::ESTOP) setState(enabled_ ? (homed_ ? State::HOLD : State::READY) : State::IDLE);
    reply("OK CLR");
    return;
  }
  if (is("EN")) {
    const bool on = nt > 1 && atoi(tok[1]) != 0;
    if (on && hal_.estopActive()) { reply("ERR 7 arret d'urgence actif"); return; }
    if (on && (state_ == State::FAULT || state_ == State::ESTOP)) { reply("ERR 4 defaut actif (CLR)"); return; }
    if (on != enabled_) {
      enable(on);
      setState(on ? State::READY : State::IDLE);
    }
    reply("OK EN %d", on ? 1 : 0);
    return;
  }
  if (is("GRIP")) {
    float pct;
    if (nt < 2 || !parseFloats(tok + 1, 1, &pct)) { reply("ERR 3 GRIP pct(0..100)"); return; }
    pct = clampf(pct, 0, 100);
    hal_.setGripperUs((uint16_t)lroundf(cfg_.gripClosedUs + ((float)cfg_.gripOpenUs - (float)cfg_.gripClosedUs) * pct / 100.0f));
    reply("OK GRIP %.0f", (double)pct);
    return;
  }
  if (is("DO")) {
    if (nt < 3) { reply("ERR 3 DO n(0..7) v(0/1)"); return; }
    const int n = atoi(tok[1]);
    if (n < 0 || n > 7) { reply("ERR 3 sortie 0..7"); return; }
    out_[n] = atoi(tok[2]) != 0;
    hal_.setOutput((uint8_t)n, out_[n]);
    reply("OK DO %d %d", n, out_[n] ? 1 : 0);
    return;
  }
  if (is("DI")) {
    unsigned di = 0;
    for (int k = 0; k < 8; k++)
      if (hal_.readInput((uint8_t)k)) di |= (1u << k);
    reply("OK DI %02X", di);
    return;
  }
  if (is("SET") || is("GET")) {
    const bool set = is("SET");
    if (nt < 3 || (set && nt < 4)) { reply("ERR 3 SET cle axe valeur | GET cle axe"); return; }
    for (char* p = tok[1]; *p; p++) *p = (char)tolower((unsigned char)*p);
    if (!set) { cmdGet(tok[1], tok[2]); return; }
    float v;
    if (!parseFloats(tok + 3, 1, &v)) { reply("ERR 3 valeur invalide"); return; }
    cmdSet(tok[1], tok[2], v);
    return;
  }
  if (is("SAVE")) {
    cfg_.crc = crc32(reinterpret_cast<const uint8_t*>(&cfg_), offsetof(Config, crc));
    reply(hal_.saveBlob(&cfg_, sizeof(cfg_)) ? "OK SAVE" : "ERR 8 EEPROM indisponible");
    return;
  }
  const bool moving = motion_ != Motion::NONE;
  if (is("LOAD") || is("DEFAULTS")) {
    const bool defaults = is("DEFAULTS");
    if (moving) { reply("ERR 4 robot en mouvement"); return; }
    Config tmp;
    if (defaults) defaultConfig(tmp);
    else if (!hal_.loadBlob(&tmp, sizeof(tmp)) || !configValid(tmp)) { reply("ERR 8 configuration EEPROM invalide"); return; }
    const bool scaleChanged = memcmp(tmp.stepsPerDeg, cfg_.stepsPerDeg, sizeof(cfg_.stepsPerDeg)) != 0;
    cfg_ = tmp;
    if (scaleChanged && homed_) {
      homed_ = false;
      if (state_ == State::HOLD) setState(State::READY);
    }
    reply(defaults ? "OK DEFAULTS" : "OK LOAD");
    return;
  }

  // --- commandes de mouvement
  if (is("STOP")) { cmdStop(); reply("OK STOP"); return; }
  const bool canMove = enabled_ && state_ != State::FAULT && state_ != State::ESTOP && state_ != State::HOMING;
  if (is("HOME")) {
    if (!canMove) { reply("ERR 4 activer les moteurs (EN 1) / etat %s", stateName(state_)); return; }
    const uint32_t mask = nt > 1 ? (uint32_t)strtoul(tok[1], nullptr, 0) : ((1u << NA) - 1);
    cmdHome(mask);
    return;
  }
  if (is("ZERO")) {
    // Déclare la position actuelle comme pose Home (sans capteurs de prise d’origine)
    if (!enabled_ || moving || state_ == State::FAULT || state_ == State::ESTOP) { reply("ERR 4 moteurs actifs et robot immobile requis"); return; }
    for (int a = 0; a < NA; a++) setPosition(a, (int32_t)lroundf(degToSteps(a, cfg_.homePose[a])));
    homed_ = true;
    if (state_ == State::READY) setState(State::HOLD);
    reply("OK ZERO");
    return;
  }
  if (is("JG")) {  // déplacement relatif d’un axe, possible sans prise d’origine : JG axe delta_deg
    float d;
    const int a = nt > 1 ? atoi(tok[1]) - 1 : -1;
    if (!canMove) { reply("ERR 4 etat %s", stateName(state_)); return; }
    if (nt < 3 || a < 0 || a >= NA || !parseFloats(tok + 2, 1, &d) || fabsf(d) > 30.f) { reply("ERR 3 JG axe(1..%d) delta(|d|<=30)", NA); return; }
    float q[NA];
    for (int k = 0; k < NA; k++) q[k] = desDeg(k);
    q[a] += d;
    cmdMoveJ(0, q, homed_);
    return;
  }
  if (is("MJ")) {
    if (!canMove) { reply("ERR 4 etat %s", stateName(state_)); return; }
    if (!homed_) { reply("ERR 4 prise d'origine requise (HOME ou ZERO)"); return; }
    float v[NA + 1];
    if (nt != NA + 2 || !parseFloats(tok + 1, NA + 1, v)) { reply("ERR 3 MJ T q1..q%d", NA); return; }
    cmdMoveJ(v[0], v + 1, true);
    return;
  }
  if (is("SP")) {
    if (!canMove) { reply("ERR 4 etat %s", stateName(state_)); return; }
    if (!homed_) { reply("ERR 4 prise d'origine requise"); return; }
    float v[2 * NA + 2];
    const bool hasVel = nt == 2 * NA + 3;
    if ((nt != NA + 3 && !hasVel) || !parseFloats(tok + 1, nt - 1, v)) { reply("ERR 3 SP seq t_ms q1..q%d [v1..v%d]", NA, NA); return; }
    cmdStream((uint32_t)v[0], (uint32_t)v[1], v + 2, hasVel ? v + 2 + NA : nullptr);
    return;
  }
  reply("ERR 2 commande inconnue %s", c);
}

// ------------------------------------------------------------------ paramètres
float* Robot::cfgField(const char* key, uint8_t*& u8, int8_t*& i8) {
  u8 = nullptr;
  i8 = nullptr;
  static const char* const keys[] = {"steps_per_deg", "min", "max", "vmax", "amax", "home_pos",
                                     "home_speed", "home_slow", "home_backoff", "home_pose"};
  float* const fields[] = {cfg_.stepsPerDeg, cfg_.minDeg, cfg_.maxDeg, cfg_.vmax, cfg_.amax, cfg_.homePos,
                           cfg_.homeSpeed, cfg_.homeSlow, cfg_.homeBackoff, cfg_.homePose};
  for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); i++)
    if (!strcmp(key, keys[i])) return fields[i];
  if (!strcmp(key, "invert")) u8 = cfg_.invert;
  else if (!strcmp(key, "home_en")) u8 = cfg_.homeEnabled;
  else if (!strcmp(key, "home_dir")) i8 = cfg_.homeDir;
  return nullptr;
}

void Robot::cmdSet(const char* key, const char* axis, float v) {
  auto scalar = [&](const char* k, uint16_t& field, float lo, float hi) {
    if (strcmp(key, k)) return false;
    field = (uint16_t)clampf(v, lo, hi);
    reply("OK SET %s %u", k, field);
    return true;
  };
  if (scalar("watchdog", cfg_.watchdogMs, 20, 5000) || scalar("stream_delay", cfg_.streamDelayMs, 0, 400) ||
      scalar("grip_closed", cfg_.gripClosedUs, 500, 2500) || scalar("grip_open", cfg_.gripOpenUs, 500, 2500))
    return;
  uint8_t* u8;
  int8_t* i8;
  float* f = cfgField(key, u8, i8);
  if (!f && !u8 && !i8) { reply("ERR 3 cle inconnue %s", key); return; }
  int a0 = 0, a1 = NA - 1;
  if (strcmp(axis, "*")) {
    a0 = a1 = atoi(axis) - 1;
    if (a0 < 0 || a0 >= NA) { reply("ERR 3 axe 1..%d ou *", NA); return; }
  }
  const bool positive = !strcmp(key, "steps_per_deg") || !strcmp(key, "vmax") || !strcmp(key, "amax") ||
                        !strcmp(key, "home_speed") || !strcmp(key, "home_slow");
  if (positive && !(v > 0)) { reply("ERR 3 %s doit etre > 0", key); return; }
  if (!strcmp(key, "steps_per_deg") && motion_ != Motion::NONE) { reply("ERR 4 robot en mouvement"); return; }
  for (int a = a0; a <= a1; a++) {
    if (!strcmp(key, "min") && !(v < cfg_.maxDeg[a])) { reply("ERR 3 min >= max (J%d)", a + 1); return; }
    if (!strcmp(key, "max") && !(v > cfg_.minDeg[a])) { reply("ERR 3 max <= min (J%d)", a + 1); return; }
  }
  for (int a = a0; a <= a1; a++) {
    if (f) {
      if (f == cfg_.stepsPerDeg && f[a] != v) {
        // Conserve la position angulaire : conversion du compteur de pas à la nouvelle échelle
        setPosition(a, (int32_t)lroundf((float)pos_[a] * (v / f[a])));
      }
      f[a] = v;
    } else if (u8) {
      u8[a] = v != 0 ? 1 : 0;
    } else {
      i8[a] = v < 0 ? -1 : 1;
    }
  }
  reply("OK SET %s %s %.5g", key, axis, (double)v);
}

void Robot::cmdGet(const char* key, const char* axis) {
  if (!strcmp(key, "watchdog")) { reply("OK %u", cfg_.watchdogMs); return; }
  if (!strcmp(key, "stream_delay")) { reply("OK %u", cfg_.streamDelayMs); return; }
  if (!strcmp(key, "grip_closed")) { reply("OK %u", cfg_.gripClosedUs); return; }
  if (!strcmp(key, "grip_open")) { reply("OK %u", cfg_.gripOpenUs); return; }
  uint8_t* u8;
  int8_t* i8;
  float* f = cfgField(key, u8, i8);
  const int a = atoi(axis) - 1;
  if ((!f && !u8 && !i8) || a < 0 || a >= NA) { reply("ERR 3 GET cle axe(1..%d)", NA); return; }
  if (f) reply("OK %.5g", (double)f[a]);
  else if (u8) reply("OK %u", u8[a]);
  else reply("OK %d", i8[a]);
}

// ------------------------------------------------------------------ mouvements
void Robot::clampToLimits(float* p) const {
  for (int a = 0; a < NA; a++) p[a] = clampf(p[a], degToSteps(a, cfg_.minDeg[a]), degToSteps(a, cfg_.maxDeg[a]));
}

void Robot::cmdMoveJ(float T, const float* qDeg, bool checkLimits) {
  if (checkLimits) {
    for (int a = 0; a < NA; a++) {
      if (qDeg[a] < cfg_.minDeg[a] - 1e-3f || qDeg[a] > cfg_.maxDeg[a] + 1e-3f) {
        reply("ERR 5 J%d hors butees (%.2f)", a + 1, (double)qDeg[a]);
        return;
      }
    }
  }
  // Durée commune : la plus contraignante des articulations (profil trapézoïdal 25/50/25)
  float dur = T > 0 ? T : 0;
  for (int a = 0; a < NA; a++) {
    const float d = fabsf(qDeg[a] - desDeg(a));
    if (d < 1e-6f) continue;
    const float tv = d / (0.75f * cfg_.vmax[a]);
    const float ta = sqrtf(d / (0.1875f * cfg_.amax[a]));
    if (tv > dur) dur = tv;
    if (ta > dur) dur = ta;
  }
  for (int a = 0; a < NA; a++) {
    mjStart_[a] = pDes_[a];
    mjDelta_[a] = degToSteps(a, qDeg[a]) - pDes_[a];
  }
  mjT0_ = now_;
  mjDur_ = (uint32_t)ceilf(dur * 1000.f);
  if (mjDur_ < 1) mjDur_ = 1;
  beginMotion(Motion::MOVEJ);
  reply("OK MJ %.3f", (double)(mjDur_ / 1000.f));
}

void Robot::cmdStream(uint32_t seq16, uint32_t t16, const float* qDeg, const float* vDeg) {
  seq16 &= 0xFFFF;
  t16 &= 0xFFFF;
  SpPoint p;
  for (int a = 0; a < NA; a++) {
    p.q[a] = degToSteps(a, qDeg[a]);
    p.v[a] = vDeg ? degToSteps(a, vDeg[a]) / 1000.f : 0;
  }
  p.hasVel = vDeg != nullptr;
  clampToLimits(p.q);
  if (motion_ != Motion::STREAM) {
    // Démarrage : la première consigne doit être proche de la consigne actuelle
    for (int a = 0; a < NA; a++) {
      if (fabsf(p.q[a] - pDes_[a]) > degToSteps(a, 5.0f)) {
        reply("ERR 5 flux : J%d trop loin de la position actuelle (MJ d'abord)", a + 1);
        return;
      }
    }
    spW_ = spR_ = 0;
    p.t = 0;
    spClock_ = 0;
    spStarted_ = false;
    watchdogTrip_ = underrun_ = false;
    beginMotion(Motion::STREAM);
  } else {
    const int16_t dseq = (int16_t)(uint16_t)(seq16 - spLastSeq_);
    if (dseq <= 0) return;  // doublon ou consigne périmée
    const int16_t dt = (int16_t)(uint16_t)(t16 - spLastT16_);
    p.t = sp_[(spW_ - 1) % SPN].t + (dt > 0 ? dt : 0);
    if (spW_ - spR_ >= (uint32_t)SPN) {  // tampon plein : l’émetteur est trop en avance
      spR_++;
      const float t0 = sp_[spR_ % SPN].t;
      if (spClock_ < t0) spClock_ = t0;
    }
  }
  spLastSeq_ = seq16;
  spLastT16_ = (uint16_t)t16;
  sp_[spW_ % SPN] = p;
  spW_++;
  spLastArrival_ = now_;
}

void Robot::cmdStop() {
  if (motion_ == Motion::NONE || motion_ == Motion::HOMING) {
    motion_ = Motion::NONE;
    for (int a = 0; a < NA; a++) vDes_[a] = 0;
    if (state_ == State::RUN || state_ == State::HOMING) setState(homed_ ? State::HOLD : State::READY);
    return;
  }
  motion_ = Motion::STOPPING;
}

void Robot::cmdHome(uint32_t mask) {
  homeMask_ = mask;
  homeIdx_ = 0;
  hstep_ = HStep::START;
  homed_ = false;
  motion_ = Motion::HOMING;
  for (int a = 0; a < NA; a++) vDes_[a] = 0;
  setState(State::HOMING);
  reply("OK HOME 0x%02lX", (unsigned long)mask);
}

// ------------------------------------------------------------------ boucle 1 kHz
void Robot::tick1ms() {
  now_++;
  // Arrêt d’urgence matériel : coupure immédiate de la puissance (catégorie 0)
  if (hal_.estopActive() && state_ != State::ESTOP) {
    enable(false);
    setState(State::ESTOP);
    reply("EVT ESTOP");
  }
  // Capteur de fin de course atteint hors prise d’origine : défaut
  if (state_ == State::RUN || state_ == State::HOLD) {
    for (int a = 0; a < NA; a++) {
      if (cfg_.homeEnabled[a] && hal_.limitActive((uint8_t)a)) {
        char m[40];
        snprintf(m, sizeof(m), "fin de course J%d", a + 1);
        fault(m);
        break;
      }
    }
  }
  if (enabled_) {
    if (motion_ == Motion::HOMING) homingTick();
    else motionTick();
  }
  computeRates();
  if (statusHz_ && (int32_t)(now_ - nextStatus_) >= 0) {
    nextStatus_ = now_ + 1000 / statusHz_;
    sendStatus();
  }
}

bool Robot::streamTarget(float* target) {
  if (spW_ == 0) return false;
  const SpPoint& last = sp_[(spW_ - 1) % SPN];
  const float D = (float)cfg_.streamDelayMs;
  const float lag = last.t - spClock_;
  if (!spStarted_ && (lag >= D || spW_ - spR_ >= (uint32_t)(SPN / 2))) {
    spStarted_ = true;
    spLagF_ = lag;
  }
  if (spStarted_) {
    // Horloge de lecture : 1 ms par tick, corrigée lentement pour garder ≈ D ms d’avance
    spLagF_ += (lag - spLagF_) * 0.005f;
    spClock_ += clampf(1.f + 0.002f * (spLagF_ - D), 0.95f, 1.05f);
  }
  if (spClock_ >= last.t) {
    underrun_ = spStarted_ && spClock_ > last.t;
    spClock_ = last.t;
  } else {
    underrun_ = false;
  }
  while (spR_ + 1 < spW_ && sp_[(spR_ + 1) % SPN].t <= spClock_) spR_++;
  const SpPoint& p0 = sp_[spR_ % SPN];
  if (spR_ + 1 >= spW_) {
    for (int a = 0; a < NA; a++) target[a] = p0.q[a];
    return true;
  }
  const SpPoint& p1 = sp_[(spR_ + 1) % SPN];
  const float dt = p1.t - p0.t;
  const float s = dt > 0 ? clampf((spClock_ - p0.t) / dt, 0, 1) : 1;
  if (p0.hasVel && p1.hasVel && dt > 0 && dt < 250) {
    // Interpolation d’Hermite cubique (positions + vitesses)
    const float s2 = s * s, s3 = s2 * s;
    const float h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s, h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;
    for (int a = 0; a < NA; a++) target[a] = h00 * p0.q[a] + h10 * dt * p0.v[a] + h01 * p1.q[a] + h11 * dt * p1.v[a];
  } else {
    for (int a = 0; a < NA; a++) target[a] = p0.q[a] + (p1.q[a] - p0.q[a]) * s;
  }
  return true;
}

void Robot::motionTick() {
  float target[NA];
  bool finished = false;
  switch (motion_) {
    case Motion::MOVEJ: {
      const float x = (float)(now_ - mjT0_) / (float)mjDur_;
      // Profil trapézoïdal normalisé : accélération 25 %, vitesse constante 50 %, décélération 25 %
      const float f = 0.25f, acc = 1.f / (f * (1.f - f)), vp = 1.f / (1.f - f);
      float s;
      if (x <= 0) s = 0;
      else if (x >= 1) s = 1;
      else if (x < f) s = 0.5f * acc * x * x;
      else if (x <= 1 - f) s = 0.5f * acc * f * f + vp * (x - f);
      else s = 1.f - 0.5f * acc * (1 - x) * (1 - x);
      for (int a = 0; a < NA; a++) target[a] = mjStart_[a] + mjDelta_[a] * s;
      finished = x >= 1.f;
      break;
    }
    case Motion::STREAM: {
      if (!streamTarget(target)) return;
      // Chien de garde : plus de consignes → arrêt contrôlé
      if (now_ - spLastArrival_ > cfg_.watchdogMs) {
        watchdogTrip_ = true;
        motion_ = Motion::STOPPING;
        reply("EVT WATCHDOG");
        return;
      }
      break;
    }
    case Motion::STOPPING: {
      bool still = true;
      for (int a = 0; a < NA; a++) {
        const float am = degToSteps(a, cfg_.amax[a]) / 1e6f;  // pas/ms²
        float v = vDes_[a];
        if (v > am) v -= am;
        else if (v < -am) v += am;
        else v = 0;
        vDes_[a] = v;
        pDes_[a] += v;
        if (v != 0) still = false;
      }
      if (still) {
        motion_ = Motion::NONE;
        setState(homed_ ? State::HOLD : State::READY);
        reply("EVT STOPPED");
      }
      return;
    }
    default:
      for (int a = 0; a < NA; a++) vDes_[a] = 0;
      return;
  }
  if (homed_) clampToLimits(target);
  float maxErr = 0;
  bool lim = false;
  for (int a = 0; a < NA; a++) {
    const float spd = cfg_.stepsPerDeg[a];
    const float vm = cfg_.vmax[a] * spd / 1000.f * 1.05f;  // pas/ms
    const float ac = cfg_.amax[a] * spd / 1e6f;            // pas/ms²
    const float am = ac * 1.5f;
    const float vff = target[a] - lastTarget_[a];
    const float e0 = lastTarget_[a] - pDes_[a];  // retard accumulé avant ce tick
    lastTarget_[a] = target[a];
    // Rattrapage proportionnel, borné pour pouvoir s’arrêter sans dépasser la cible
    const float vb = sqrtf(2.f * ac * fabsf(e0));
    const float vc = clampf(0.2f * e0, -vb, vb);
    const float v0 = vff + vc;
    float v = clampf(v0, vDes_[a] - am, vDes_[a] + am);
    v = clampf(v, -vm, vm);
    if (fabsf(v - v0) > 1e-3f) lim = true;
    vDes_[a] = v;
    pDes_[a] += v;
    const float err = fabsf(target[a] - pDes_[a]);
    if (err > maxErr) maxErr = err;
  }
  if (lim) limited_ = true;
  if (finished && maxErr < 0.5f) {
    for (int a = 0; a < NA; a++) { pDes_[a] = target[a]; vDes_[a] = 0; }
    motion_ = Motion::NONE;
    setState(State::HOLD);
    reply("EVT DONE");
  }
}

void Robot::homingTick() {
  // Axe courant (ordre configuré) ; axes non demandés ou sans capteur ignorés
  while (homeIdx_ < NA) {
    const int a = cfg_.homeOrder[homeIdx_];
    if (a < NA && (homeMask_ & (1u << a)) && cfg_.homeEnabled[a]) break;
    homeIdx_++;
  }
  if (homeIdx_ >= NA) {
    homed_ = true;
    motion_ = Motion::NONE;
    reply("EVT HOMED");
    float pose[NA];
    for (int a = 0; a < NA; a++) pose[a] = clampf(cfg_.homePose[a], cfg_.minDeg[a], cfg_.maxDeg[a]);
    cmdMoveJ(0, pose, true);
    return;
  }
  const int a = cfg_.homeOrder[homeIdx_];
  const float spd = cfg_.stepsPerDeg[a];
  const float fast = cfg_.homeSpeed[a] * spd / 1000.f, slow = cfg_.homeSlow[a] * spd / 1000.f;
  const float backoff = cfg_.homeBackoff[a] * spd;
  const float dir = cfg_.homeDir[a] < 0 ? -1.f : 1.f;
  const bool sw = hal_.limitActive((uint8_t)a);
  char m[48];
  switch (hstep_) {
    case HStep::START:
      htravel_ = 0;
      hstep_ = sw ? HStep::CLEAR : HStep::FAST;
      break;
    case HStep::CLEAR:  // déjà sur le capteur : on s’en dégage d’abord
      pDes_[a] -= dir * slow * 3;
      htravel_ += slow * 3;
      if (!sw && htravel_ >= backoff) { hstep_ = HStep::FAST; htravel_ = 0; }
      else if (htravel_ > 45 * spd) {
        snprintf(m, sizeof(m), "prise d'origine J%d : capteur bloque", a + 1);
        fault(m);
      }
      break;
    case HStep::FAST:
      pDes_[a] += dir * fast;
      htravel_ += fast;
      if (sw) { hstep_ = HStep::BACK; htravel_ = 0; }
      else if (htravel_ > (cfg_.maxDeg[a] - cfg_.minDeg[a] + 20.f) * spd) {
        snprintf(m, sizeof(m), "prise d'origine J%d : capteur introuvable", a + 1);
        fault(m);
      }
      break;
    case HStep::BACK:
      pDes_[a] -= dir * slow * 3;
      htravel_ += slow * 3;
      if (htravel_ >= backoff && !sw) { hstep_ = HStep::SLOW; htravel_ = 0; }
      else if (htravel_ > 45 * spd) {
        snprintf(m, sizeof(m), "prise d'origine J%d : capteur bloque", a + 1);
        fault(m);
      }
      break;
    case HStep::SLOW:
      pDes_[a] += dir * slow;
      htravel_ += slow;
      if (sw) {
        // Front du capteur à vitesse lente = position calibrée
        setPosition(a, (int32_t)lroundf(cfg_.homePos[a] * spd));
        hstep_ = HStep::RELEASE;
        htravel_ = 0;
      } else if (htravel_ > 2 * backoff + 2 * spd) {
        snprintf(m, sizeof(m), "prise d'origine J%d : capteur perdu", a + 1);
        fault(m);
      }
      break;
    case HStep::RELEASE:  // dégagement : l’axe quitte le capteur et revient dans ses butées
      pDes_[a] -= dir * slow * 3;
      htravel_ += slow * 3;
      if (htravel_ >= backoff && !sw) {
        reply("EVT HOMED_AXIS %d", a + 1);
        hstep_ = HStep::START;
        homeIdx_++;
      } else if (htravel_ > 45 * spd) {
        snprintf(m, sizeof(m), "prise d'origine J%d : capteur bloque", a + 1);
        fault(m);
      }
      break;
  }
}

void Robot::computeRates() {
  for (int a = 0; a < NA; a++) {
    if (!enabled_) { rate_[a] = 0; continue; }
    const float err = pDes_[a] - ((float)pos_[a] + phase_[a]);
    const float r = clampf(err / (float)ISR_PER_MS, -0.45f, 0.45f);  // ≤ 45 kpas/s
    // Le sens demandé est appliqué par l’interruption, entre deux impulsions
    if (r > 1e-6f) dirReq_[a] = 1;
    else if (r < -1e-6f) dirReq_[a] = -1;
    rate_[a] = r;
  }
}

// ------------------------------------------------------------------ interruption 100 kHz
void Robot::stepIsr() {
  for (int a = 0; a < NA; a++) {
    float ph = phase_[a] + rate_[a];
    if (pulseHigh_[a]) {  // fin d’impulsion : niveau bas pendant au moins une période
      hal_.stepLow((uint8_t)a);
      pulseHigh_[a] = 0;
    } else if (dirReq_[a] != dir_[a] && dirReq_[a] != 0) {
      // Changement de sens ≥ 10 µs après le dernier front, ≥ 20 µs avant l’impulsion suivante
      dir_[a] = dirReq_[a];
      hal_.setDir((uint8_t)a, (dir_[a] > 0) != (cfg_.invert[a] != 0));
      dirHold_[a] = 1;
    } else if (dirHold_[a]) {
      dirHold_[a]--;
    } else if (ph >= 1.f && dir_[a] > 0) {
      ph -= 1.f;
      pos_[a]++;
      hal_.stepHigh((uint8_t)a);
      pulseHigh_[a] = 1;
    } else if (ph <= -1.f && dir_[a] < 0) {
      ph += 1.f;
      pos_[a]--;
      hal_.stepHigh((uint8_t)a);
      pulseHigh_[a] = 1;
    }
    phase_[a] = clampf(ph, -2.f, 2.f);
  }
}

}  // namespace orion
