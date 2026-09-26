// ORION-6 PRO — pont USB ↔ CAN « mode MIT » : implémentation (voir mit_core.h).
#include "mit_core.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace mit {

static constexpr uint32_t CFG_MAGIC = 0x4D495442UL;  // « MITB »
static constexpr uint16_t CFG_VERSION = 1;
static constexpr float D2R = 3.14159265358979f / 180.f;
static constexpr float R2D = 180.f / 3.14159265358979f;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ------------------------------------------------------------------ trames MIT
uint32_t floatToUint(float x, float lo, float hi, int bits) {
  const float span = hi - lo;
  const float v = clampf(x, lo, hi);
  return (uint32_t)lroundf((v - lo) * (float)((1u << bits) - 1) / span);
}

float uintToFloat(uint32_t u, float lo, float hi, int bits) {
  return (float)u * (hi - lo) / (float)((1u << bits) - 1) + lo;
}

void packCommand(const Command& c, const Limits& l, uint8_t out[8]) {
  const uint32_t p = floatToUint(c.p, -l.pMax, l.pMax, 16);
  const uint32_t v = floatToUint(c.v, -l.vMax, l.vMax, 12);
  const uint32_t kp = floatToUint(c.kp, 0, l.kpMax, 12);
  const uint32_t kd = floatToUint(c.kd, 0, l.kdMax, 12);
  const uint32_t t = floatToUint(c.t, -l.tMax, l.tMax, 12);
  out[0] = (uint8_t)(p >> 8);
  out[1] = (uint8_t)(p & 0xFF);
  out[2] = (uint8_t)(v >> 4);
  out[3] = (uint8_t)(((v & 0xF) << 4) | (kp >> 8));
  out[4] = (uint8_t)(kp & 0xFF);
  out[5] = (uint8_t)(kd >> 4);
  out[6] = (uint8_t)(((kd & 0xF) << 4) | (t >> 8));
  out[7] = (uint8_t)(t & 0xFF);
}

void unpackFeedback(const uint8_t d[8], const Limits& l, Feedback& fb) {
  const uint32_t p = ((uint32_t)d[1] << 8) | d[2];
  const uint32_t v = ((uint32_t)d[3] << 4) | (d[4] >> 4);
  const uint32_t t = ((uint32_t)(d[4] & 0xF) << 8) | d[5];
  fb.id = d[0] & 0x0F;
  fb.err = d[0] >> 4;
  fb.p = uintToFloat(p, -l.pMax, l.pMax, 16);
  fb.v = uintToFloat(v, -l.vMax, l.vMax, 12);
  fb.t = uintToFloat(t, -l.tMax, l.tMax, 12);
  fb.tMos = d[6];
  fb.tRotor = d[7];
}

void specialFrame(uint8_t code, uint8_t out[8]) {
  for (int i = 0; i < 7; i++) out[i] = 0xFF;
  out[7] = code;
}

const char* motorErrorText(uint8_t err) {
  switch (err) {
    case 0x8: return "surtension";
    case 0x9: return "sous-tension";
    case 0xA: return "surintensite";
    case 0xB: return "surchauffe MOSFET";
    case 0xC: return "surchauffe bobinage";
    case 0xD: return "perte de communication";
    case 0xE: return "surcharge";
    default: return "erreur moteur";
  }
}

const char* stateName(State s) {
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::READY: return "READY";
    case State::RUN: return "RUN";
    case State::HOLD: return "HOLD";
    case State::DAMP: return "DAMP";
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
    c.canId[a] = orion_cfg::CAN_ID[a];
    c.dir[a] = orion_cfg::MIT_DIR[a] < 0 ? -1 : 1;
    c.offsetDeg[a] = 0;
    c.minDeg[a] = orion_cfg::MIN_DEG[a];
    c.maxDeg[a] = orion_cfg::MAX_DEG[a];
    c.vmax[a] = orion_cfg::VMAX_DEG_S[a];
    c.tauMax[a] = orion_cfg::TAU_MAX[a];
    c.kp[a] = orion_cfg::MIT_KP[a];
    c.kd[a] = orion_cfg::MIT_KD[a];
    c.kdDamp[a] = orion_cfg::MIT_KD_DAMP[a];
    c.refPose[a] = orion_cfg::REF_POSE_DEG[a];
    c.lim[a] = {orion_cfg::MIT_PMAX[a], orion_cfg::MIT_VMAX[a], orion_cfg::MIT_TMAX[a], orion_cfg::MIT_KPMAX[a], orion_cfg::MIT_KDMAX[a]};
  }
  c.watchdogMs = orion_cfg::WATCHDOG_MS;
  c.streamDelayMs = orion_cfg::STREAM_DELAY_MS;
  c.ctrlHz = orion_cfg::MIT_CTRL_HZ;
  c.fbTimeoutMs = orion_cfg::MIT_FB_TIMEOUT_MS;
  c.maxTempC = orion_cfg::MIT_MAX_TEMP_C;
  c.crc = crc32(reinterpret_cast<const uint8_t*>(&c), offsetof(Config, crc));
}

bool configValid(const Config& c) {
  if (c.magic != CFG_MAGIC || c.version != CFG_VERSION || c.size != sizeof(Config)) return false;
  if (c.crc != crc32(reinterpret_cast<const uint8_t*>(&c), offsetof(Config, crc))) return false;
  if (c.ctrlHz < 50 || c.ctrlHz > 1000) return false;
  for (int a = 0; a < NA; a++)
    if (!(c.minDeg[a] < c.maxDeg[a]) || !(c.lim[a].pMax > 0) || !(c.lim[a].vMax > 0) || !(c.lim[a].tMax > 0)) return false;
  return true;
}

static uint8_t xorSum(const char* s, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) c ^= (uint8_t)s[i];
  return c;
}

// ------------------------------------------------------------------ pont
Bridge::Bridge(BridgeHal& hal) : hal_(hal) {
  defaultConfig(cfg_);
  memset(sp_, 0, sizeof(sp_));
}

void Bridge::begin() {
  Config tmp;
  if (hal_.loadBlob(&tmp, sizeof(tmp)) && configValid(tmp)) cfg_ = tmp;
  state_ = State::IDLE;
  reply("EVT BOOT %s mit-bridge%s axes=%d", ORION_ROBOT_NAME, FW_VERSION, NA);
}

void Bridge::reply(const char* fmt, ...) {
  char buf[240];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf) - 5, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(buf) - 6) n = (int)sizeof(buf) - 6;
  snprintf(buf + n, 5, "*%02X\n", xorSum(buf, (size_t)n));
  hal_.write(buf);
}

uint32_t Bridge::flags() const {
  uint32_t f = 0;
  const bool en = state_ != State::IDLE && state_ != State::ESTOP;
  if (en) f |= F_ENABLED;
  f |= F_HOMED;  // codeurs absolus : toujours référencé (après ZERO une fois)
  if (state_ == State::RUN) f |= F_MOVING;
  if (state_ == State::FAULT) f |= F_FAULT;
  if (hal_.estopActive()) f |= F_ESTOP_IN;
  if (watchdogTrip_) f |= F_WATCHDOG;
  for (int a = 0; a < NA; a++)
    if (q_[a] < cfg_.minDeg[a] || q_[a] > cfg_.maxDeg[a]) { f |= F_LIMIT; break; }
  if (underrun_) f |= F_UNDERRUN;
  if (limited_) f |= F_LIMITED;
  return f;
}

void Bridge::sendStatus() {
  char buf[200];
  int n = snprintf(buf, sizeof(buf), "ST %s %lu", stateName(state_), (unsigned long)now_);
  for (int a = 0; a < NA && n < (int)sizeof(buf) - 16; a++) n += snprintf(buf + n, sizeof(buf) - n, " %.3f", (double)q_[a]);
  snprintf(buf + n, sizeof(buf) - n, " %02lX 00", (unsigned long)flags());
  reply("%s", buf);
}

void Bridge::fault(const char* why) {
  if (state_ == State::FAULT || state_ == State::ESTOP) return;
  // Amortissement avec le dernier couple d’anticipation (≈ compensation de gravité)
  for (int a = 0; a < NA; a++) {
    holdQ_[a] = q_[a];
    if (state_ == State::RUN) holdTau_[a] = lastTau_[a];
  }
  state_ = State::FAULT;
  strncpy(faultMsg_, why, sizeof(faultMsg_) - 1);
  reply("EVT FAULT %s", why);
}

void Bridge::sendSpecial(uint8_t code) {
  uint8_t d[8];
  specialFrame(code, d);
  for (int a = 0; a < NA; a++) hal_.canSend(cfg_.canId[a], d);
}

void Bridge::sendMit(int a, float qDeg, float vDegS, float tau, float kp, float kd) {
  lastCmdDeg_[a] = qDeg;
  const float s = (float)cfg_.dir[a];
  Command c;
  c.p = s * (qDeg - cfg_.offsetDeg[a]) * D2R;
  c.v = s * vDegS * D2R;
  c.t = s * clampf(tau, -cfg_.tauMax[a], cfg_.tauMax[a]);
  c.kp = kp;
  c.kd = kd;
  uint8_t d[8];
  packCommand(c, cfg_.lim[a], d);
  hal_.canSend(cfg_.canId[a], d);
}

void Bridge::updateJointFromMotor(int a) { q_[a] = (float)cfg_.dir[a] * pm_[a] * R2D + cfg_.offsetDeg[a]; }

void Bridge::holdHere() {
  for (int a = 0; a < NA; a++) holdQ_[a] = clampf(q_[a], cfg_.minDeg[a], cfg_.maxDeg[a]);
}

void Bridge::readFeedback() {
  uint32_t id;
  uint8_t d[8];
  int guard = 0;
  while (guard++ < 64 && hal_.canReceive(id, d)) {
    const uint8_t mid = d[0] & 0x0F;
    for (int a = 0; a < NA; a++) {
      if ((cfg_.canId[a] & 0x0F) != mid) continue;
      Feedback fb;
      unpackFeedback(d, cfg_.lim[a], fb);
      const float s = (float)cfg_.dir[a];
      pm_[a] = fb.p;
      updateJointFromMotor(a);
      v_[a] = s * fb.v * R2D;
      tau_[a] = s * fb.t;
      tMos_[a] = fb.tMos;
      tRotor_[a] = fb.tRotor;
      err_[a] = fb.err;
      lastFb_[a] = now_;
      seen_[a] = true;
      break;
    }
  }
}

bool Bridge::parseFloats(char** tok, int n, float* out) {
  for (int i = 0; i < n; i++) {
    if (!tok[i]) return false;
    char* end = nullptr;
    out[i] = strtof(tok[i], &end);
    if (end == tok[i] || *end || !isfinite(out[i])) return false;
  }
  return true;
}

void Bridge::onChar(char c) {
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

void Bridge::handleLine(char* line) {
  char* star = strrchr(line, '*');
  if (star && strlen(star) == 3 && isxdigit((unsigned char)star[1]) && isxdigit((unsigned char)star[2])) {
    const unsigned v = (unsigned)strtoul(star + 1, nullptr, 16);
    if (xorSum(line, (size_t)(star - line)) != v) { reply("ERR 1 somme de controle"); return; }
    *star = 0;
  }
  char* tok[3 * NA + 4] = {nullptr};
  int nt = 0;
  for (char* p = strtok(line, " \t"); p && nt < 3 * NA + 4; p = strtok(nullptr, " \t")) tok[nt++] = p;
  if (!nt) return;
  for (char* p = tok[0]; *p; p++) *p = (char)toupper((unsigned char)*p);
  const char* c = tok[0];
  auto is = [&](const char* s) { return strcmp(c, s) == 0; };
  const bool enabled = state_ != State::IDLE && state_ != State::ESTOP;

  if (is("PING")) { reply("OK PONG %s mit-bridge%s axes=%d", ORION_ROBOT_NAME, FW_VERSION, NA); return; }
  if (is("VER")) { reply("OK VER %s", FW_VERSION); return; }
  if (is("STAT")) { sendStatus(); return; }
  if (is("FLT")) { reply("OK FLT %s", faultMsg_[0] ? faultMsg_ : "aucun"); return; }
  if (is("FB")) {  // retours détaillés : vitesses (°/s), couples (N·m), températures, erreurs
    char buf[220];
    int n = snprintf(buf, sizeof(buf), "OK FB");
    for (int a = 0; a < NA; a++)
      n += snprintf(buf + n, sizeof(buf) - n, " %.1f/%.2f/%u/%u/%X", (double)v_[a], (double)tau_[a], tMos_[a], tRotor_[a], err_[a]);
    reply("%s", buf);
    return;
  }
  if (is("STREAM")) {
    const int hz = nt > 1 ? atoi(tok[1]) : 0;
    statusHz_ = (uint16_t)(hz < 0 ? 0 : hz > 500 ? 500 : hz);
    nextStatus_ = now_;
    reply("OK STREAM %u", statusHz_);
    return;
  }
  if (is("ESTOP")) {
    sendSpecial(SP_DISABLE);
    state_ = State::ESTOP;
    reply("OK ESTOP");
    return;
  }
  if (is("CLR")) {
    if (hal_.estopActive()) { reply("ERR 7 arret d'urgence actif"); return; }
    sendSpecial(SP_CLEAR);
    watchdogTrip_ = underrun_ = limited_ = false;
    faultMsg_[0] = 0;
    if (state_ == State::FAULT) { holdHere(); state_ = State::READY; }
    else if (state_ == State::ESTOP) state_ = State::IDLE;
    reply("OK CLR");
    return;
  }
  if (is("EN")) {
    const bool on = nt > 1 && atoi(tok[1]) != 0;
    if (on) {
      if (hal_.estopActive()) { reply("ERR 7 arret d'urgence actif"); return; }
      if (state_ == State::FAULT || state_ == State::ESTOP) { reply("ERR 4 defaut actif (CLR)"); return; }
      for (int a = 0; a < NA; a++)
        if (!seen_[a] || now_ - lastFb_[a] > 200) { reply("ERR 6 moteur J%d muet (CAN, alimentation, ID)", a + 1); return; }
      if (!enabled) {
        holdHere();
        for (int a = 0; a < NA; a++) holdTau_[a] = 0;
        sendSpecial(SP_ENABLE);
        state_ = State::READY;
      }
      disableAt_ = 0;
      reply("OK EN 1");
    } else {
      if (enabled) {
        // Descente amortie pendant 1,5 s avant de couper les moteurs
        state_ = State::DAMP;
        disableAt_ = now_ + 1500;
      }
      reply("OK EN 0");
    }
    return;
  }
  if (is("DAMP")) {
    if (!enabled) { reply("ERR 4 moteurs inactifs"); return; }
    state_ = State::DAMP;
    disableAt_ = 0;
    reply("OK DAMP");
    return;
  }
  if (is("STOP")) {
    if (state_ == State::RUN) {
      for (int a = 0; a < NA; a++) { holdQ_[a] = lastCmdDeg_[a]; holdTau_[a] = lastTau_[a]; }
      state_ = State::HOLD;
    }
    reply("OK STOP");
    return;
  }
  if (is("ZERO")) {  // la position actuelle devient la pose de référence (décalages logiciels)
    if (state_ == State::RUN) { reply("ERR 4 robot en mouvement"); return; }
    for (int a = 0; a < NA; a++) {
      if (!seen_[a]) { reply("ERR 6 moteur J%d muet", a + 1); return; }
    }
    for (int a = 0; a < NA; a++) {
      cfg_.offsetDeg[a] = cfg_.refPose[a] - (float)cfg_.dir[a] * pm_[a] * R2D;
      updateJointFromMotor(a);
    }
    holdHere();
    reply("OK ZERO (SAVE pour conserver)");
    return;
  }
  if (is("MZERO")) {  // écrit le zéro dans les moteurs (une seule fois, moteurs inactifs)
    if (enabled) { reply("ERR 4 desactiver les moteurs (EN 0)"); return; }
    uint8_t d[8];
    specialFrame(SP_ZERO, d);
    const uint32_t mask = nt > 1 ? (uint32_t)strtoul(tok[1], nullptr, 0) : 0;
    for (int a = 0; a < NA; a++)
      if (mask & (1u << a)) { hal_.canSend(cfg_.canId[a], d); cfg_.offsetDeg[a] = cfg_.refPose[a]; }
    reply("OK MZERO 0x%02lX", (unsigned long)mask);
    return;
  }
  if (is("MC")) {
    if (!(state_ == State::READY || state_ == State::HOLD || state_ == State::RUN)) { reply("ERR 4 etat %s", stateName(state_)); return; }
    float v[3 * NA + 2];
    const int nv = nt - 1;
    if ((nv != NA + 2 && nv != 2 * NA + 2 && nv != 3 * NA + 2) || !parseFloats(tok + 1, nv, v)) {
      reply("ERR 3 MC seq t q1..q%d [v1..v%d [tau1..tau%d]]", NA, NA, NA);
      return;
    }
    cmdStream((uint32_t)v[0], (uint32_t)v[1], v + 2, nv >= 2 * NA + 2 ? v + 2 + NA : nullptr, nv == 3 * NA + 2 ? v + 2 + 2 * NA : nullptr);
    return;
  }
  if (is("SET") || is("GET")) {
    const bool set = is("SET");
    if (nt < 3 || (set && nt < 4)) { reply("ERR 3 SET cle axe valeur | GET cle axe"); return; }
    for (char* p = tok[1]; *p; p++) *p = (char)tolower((unsigned char)*p);
    if (!set) { cmdGet(tok[1], tok[2]); return; }
    float val;
    if (!parseFloats(tok + 3, 1, &val)) { reply("ERR 3 valeur invalide"); return; }
    cmdSet(tok[1], tok[2], val);
    return;
  }
  if (is("SAVE")) {
    cfg_.crc = crc32(reinterpret_cast<const uint8_t*>(&cfg_), offsetof(Config, crc));
    reply(hal_.saveBlob(&cfg_, sizeof(cfg_)) ? "OK SAVE" : "ERR 8 EEPROM indisponible");
    return;
  }
  if (is("LOAD") || is("DEFAULTS")) {
    const bool defaults = is("DEFAULTS");
    if (enabled) { reply("ERR 4 desactiver les moteurs (EN 0)"); return; }
    Config tmp;
    if (defaults) defaultConfig(tmp);
    else if (!hal_.loadBlob(&tmp, sizeof(tmp)) || !configValid(tmp)) { reply("ERR 8 configuration EEPROM invalide"); return; }
    cfg_ = tmp;
    reply(defaults ? "OK DEFAULTS" : "OK LOAD");
    return;
  }
  reply("ERR 2 commande inconnue %s", c);
}

float* Bridge::field(const char* key, bool& isInt) {
  isInt = false;
  static const char* const keys[] = {"kp", "kd", "kd_damp", "offset", "min", "max", "vmax", "tau_max", "ref_pose"};
  float* const f[] = {cfg_.kp, cfg_.kd, cfg_.kdDamp, cfg_.offsetDeg, cfg_.minDeg, cfg_.maxDeg, cfg_.vmax, cfg_.tauMax, cfg_.refPose};
  for (unsigned i = 0; i < sizeof(keys) / sizeof(keys[0]); i++)
    if (!strcmp(key, keys[i])) return f[i];
  if (!strcmp(key, "dir") || !strcmp(key, "can_id")) isInt = true;
  return nullptr;
}

void Bridge::cmdSet(const char* key, const char* axis, float v) {
  auto scalar = [&](const char* k, uint16_t& fld, float lo, float hi) {
    if (strcmp(key, k)) return false;
    fld = (uint16_t)clampf(v, lo, hi);
    reply("OK SET %s %u", k, fld);
    return true;
  };
  if (scalar("watchdog", cfg_.watchdogMs, 20, 5000) || scalar("stream_delay", cfg_.streamDelayMs, 0, 400) ||
      scalar("ctrl_hz", cfg_.ctrlHz, 50, 1000) || scalar("fb_timeout", cfg_.fbTimeoutMs, 10, 1000))
    return;
  if (!strcmp(key, "max_temp")) { cfg_.maxTempC = (uint8_t)clampf(v, 40, 120); reply("OK SET max_temp %u", cfg_.maxTempC); return; }
  int a0 = 0, a1 = NA - 1;
  if (strcmp(axis, "*")) {
    a0 = a1 = atoi(axis) - 1;
    if (a0 < 0 || a0 >= NA) { reply("ERR 3 axe 1..%d ou *", NA); return; }
  }
  const bool limKey = !strcmp(key, "p_max") || !strcmp(key, "v_max") || !strcmp(key, "t_max") || !strcmp(key, "kp_max") || !strcmp(key, "kd_max");
  bool isInt;
  float* f = field(key, isInt);
  if (!f && !isInt && !limKey) { reply("ERR 3 cle inconnue %s", key); return; }
  const bool enabled = state_ != State::IDLE && state_ != State::ESTOP;
  if ((limKey || isInt || !strcmp(key, "offset")) && enabled) { reply("ERR 4 desactiver les moteurs (EN 0)"); return; }
  const bool gain = !strcmp(key, "kp") || !strcmp(key, "kd") || !strcmp(key, "kd_damp");
  for (int a = a0; a <= a1; a++) {
    if (gain && (v < 0 || v > (!strcmp(key, "kp") ? cfg_.lim[a].kpMax : cfg_.lim[a].kdMax))) { reply("ERR 3 %s hors plage [0, max] (J%d)", key, a + 1); return; }
    if (!strcmp(key, "min") && !(v < cfg_.maxDeg[a])) { reply("ERR 3 min >= max (J%d)", a + 1); return; }
    if (!strcmp(key, "max") && !(v > cfg_.minDeg[a])) { reply("ERR 3 max <= min (J%d)", a + 1); return; }
    if ((limKey || !strcmp(key, "vmax") || !strcmp(key, "tau_max")) && !(v > 0)) { reply("ERR 3 %s doit etre > 0", key); return; }
  }
  for (int a = a0; a <= a1; a++) {
    if (f) f[a] = v;
    else if (!strcmp(key, "dir")) cfg_.dir[a] = v < 0 ? -1 : 1;
    else if (!strcmp(key, "can_id")) cfg_.canId[a] = (uint8_t)v;
    else if (!strcmp(key, "p_max")) cfg_.lim[a].pMax = v;
    else if (!strcmp(key, "v_max")) cfg_.lim[a].vMax = v;
    else if (!strcmp(key, "t_max")) cfg_.lim[a].tMax = v;
    else if (!strcmp(key, "kp_max")) cfg_.lim[a].kpMax = v;
    else if (!strcmp(key, "kd_max")) cfg_.lim[a].kdMax = v;
  }
  for (int a = a0; a <= a1; a++) updateJointFromMotor(a);
  reply("OK SET %s %s %.5g", key, axis, (double)v);
}

void Bridge::cmdGet(const char* key, const char* axis) {
  if (!strcmp(key, "watchdog")) { reply("OK %u", cfg_.watchdogMs); return; }
  if (!strcmp(key, "stream_delay")) { reply("OK %u", cfg_.streamDelayMs); return; }
  if (!strcmp(key, "ctrl_hz")) { reply("OK %u", cfg_.ctrlHz); return; }
  if (!strcmp(key, "fb_timeout")) { reply("OK %u", cfg_.fbTimeoutMs); return; }
  if (!strcmp(key, "max_temp")) { reply("OK %u", cfg_.maxTempC); return; }
  const int a = atoi(axis) - 1;
  if (a < 0 || a >= NA) { reply("ERR 3 GET cle axe(1..%d)", NA); return; }
  bool isInt;
  float* f = field(key, isInt);
  if (f) reply("OK %.5g", (double)f[a]);
  else if (!strcmp(key, "dir")) reply("OK %d", cfg_.dir[a]);
  else if (!strcmp(key, "can_id")) reply("OK %u", cfg_.canId[a]);
  else if (!strcmp(key, "p_max")) reply("OK %.5g", (double)cfg_.lim[a].pMax);
  else if (!strcmp(key, "v_max")) reply("OK %.5g", (double)cfg_.lim[a].vMax);
  else if (!strcmp(key, "t_max")) reply("OK %.5g", (double)cfg_.lim[a].tMax);
  else if (!strcmp(key, "kp_max")) reply("OK %.5g", (double)cfg_.lim[a].kpMax);
  else if (!strcmp(key, "kd_max")) reply("OK %.5g", (double)cfg_.lim[a].kdMax);
  else reply("ERR 3 cle inconnue %s", key);
}

void Bridge::cmdStream(uint32_t seq16, uint32_t t16, const float* q, const float* v, const float* tau) {
  seq16 &= 0xFFFF;
  t16 &= 0xFFFF;
  Pt p;
  for (int a = 0; a < NA; a++) {
    p.q[a] = clampf(q[a], cfg_.minDeg[a], cfg_.maxDeg[a]);
    p.v[a] = v ? v[a] / 1000.f : 0;
    p.tau[a] = tau ? tau[a] : 0;
  }
  p.hasVel = v != nullptr;
  if (state_ != State::RUN) {
    for (int a = 0; a < NA; a++) {
      if (fabsf(p.q[a] - q_[a]) > 5.f) {
        reply("ERR 5 flux : J%d a %.1f deg de la position mesuree", a + 1, (double)fabsf(p.q[a] - q_[a]));
        return;
      }
    }
    spW_ = spR_ = 0;
    p.t = 0;
    spClock_ = 0;
    spStarted_ = false;
    watchdogTrip_ = underrun_ = limited_ = false;
    blend_ = 0;  // raccord progressif depuis la consigne de maintien
    state_ = State::RUN;
  } else {
    const int16_t dseq = (int16_t)(uint16_t)(seq16 - spLastSeq_);
    if (dseq <= 0) return;
    const int16_t dt = (int16_t)(uint16_t)(t16 - spLastT16_);
    p.t = sp_[(spW_ - 1) % SPN].t + (dt > 0 ? dt : 0);
    if (spW_ - spR_ >= (uint32_t)SPN) {
      spR_++;
      if (spClock_ < sp_[spR_ % SPN].t) spClock_ = sp_[spR_ % SPN].t;
    }
  }
  spLastSeq_ = (uint16_t)seq16;
  spLastT16_ = (uint16_t)t16;
  sp_[spW_ % SPN] = p;
  spW_++;
  spLastArrival_ = now_;
}

bool Bridge::streamTarget(float* q, float* v, float* tau) {
  if (spW_ == 0) return false;
  const Pt& last = sp_[(spW_ - 1) % SPN];
  const float D = (float)cfg_.streamDelayMs;
  const float lag = last.t - spClock_;
  if (!spStarted_ && (lag >= D || spW_ - spR_ >= (uint32_t)(SPN / 2))) {
    spStarted_ = true;
    spLagF_ = lag;
  }
  const float dtc = ctrlDt_;  // ms écoulées depuis le pas de commande précédent
  if (spStarted_) {
    spLagF_ += (lag - spLagF_) * 0.005f * dtc;
    spClock_ += dtc * clampf(1.f + 0.002f * (spLagF_ - D), 0.95f, 1.05f);
  }
  if (spClock_ >= last.t) {
    underrun_ = spStarted_ && spClock_ > last.t;
    spClock_ = last.t;
  } else {
    underrun_ = false;
  }
  while (spR_ + 1 < spW_ && sp_[(spR_ + 1) % SPN].t <= spClock_) spR_++;
  const Pt& p0 = sp_[spR_ % SPN];
  if (spR_ + 1 >= spW_) {
    for (int a = 0; a < NA; a++) { q[a] = p0.q[a]; v[a] = 0; tau[a] = p0.tau[a]; }
    return true;
  }
  const Pt& p1 = sp_[(spR_ + 1) % SPN];
  const float dt = p1.t - p0.t;
  const float s = dt > 0 ? clampf((spClock_ - p0.t) / dt, 0, 1) : 1;
  for (int a = 0; a < NA; a++) {
    tau[a] = p0.tau[a] + (p1.tau[a] - p0.tau[a]) * s;
    if (p0.hasVel && p1.hasVel && dt > 0 && dt < 250) {
      const float s2 = s * s, s3 = s2 * s;
      q[a] = (2 * s3 - 3 * s2 + 1) * p0.q[a] + (s3 - 2 * s2 + s) * dt * p0.v[a] + (-2 * s3 + 3 * s2) * p1.q[a] + (s3 - s2) * dt * p1.v[a];
      // dérivée de l’interpolant (°/ms → °/s)
      v[a] = ((6 * s2 - 6 * s) * p0.q[a] / dt + (3 * s2 - 4 * s + 1) * p0.v[a] + (-6 * s2 + 6 * s) * p1.q[a] / dt + (3 * s2 - 2 * s) * p1.v[a]) * 1000.f;
    } else {
      q[a] = p0.q[a] + (p1.q[a] - p0.q[a]) * s;
      v[a] = dt > 0 ? (p1.q[a] - p0.q[a]) / dt * 1000.f : 0;
    }
  }
  return true;
}

void Bridge::controlStep() {
  if (state_ == State::ESTOP) return;
  if (state_ == State::IDLE) {
    // Moteurs inactifs : trames à gains nuls (couple nul) pour lire les positions à 50 Hz
    if ((int32_t)(now_ - nextPoll_) >= 0) {
      nextPoll_ = now_ + 20;
      for (int a = 0; a < NA; a++) sendMit(a, q_[a], 0, 0, 0, 0);
    }
    return;
  }
  // Surveillance des moteurs
  for (int a = 0; a < NA; a++) {
    char m[48];
    if (now_ - lastFb_[a] > cfg_.fbTimeoutMs) { snprintf(m, sizeof(m), "CAN : J%d ne repond plus", a + 1); fault(m); }
    else if (err_[a] >= 0x8) { snprintf(m, sizeof(m), "J%d : %s", a + 1, motorErrorText(err_[a])); fault(m); }
    else if (tMos_[a] >= cfg_.maxTempC || tRotor_[a] >= cfg_.maxTempC) { snprintf(m, sizeof(m), "J%d : temperature %u C", a + 1, tMos_[a] > tRotor_[a] ? tMos_[a] : tRotor_[a]); fault(m); }
    else if (q_[a] < cfg_.minDeg[a] - 5.f || q_[a] > cfg_.maxDeg[a] + 5.f) { snprintf(m, sizeof(m), "J%d hors butees (%.1f)", a + 1, (double)q_[a]); fault(m); }
  }
  float q[NA], v[NA], tau[NA];
  switch (state_) {
    case State::RUN:
      if (now_ - spLastArrival_ > cfg_.watchdogMs) {
        watchdogTrip_ = true;
        for (int a = 0; a < NA; a++) { holdQ_[a] = lastCmdDeg_[a]; holdTau_[a] = lastTau_[a]; }
        state_ = State::HOLD;
        reply("EVT WATCHDOG");
        // fallthrough vers le maintien
      } else if (streamTarget(q, v, tau)) {
        bool lim = false;
        if (blend_ < 1.f) {
          // Raccord de 300 ms : position et couple passent du maintien au flux (lissage C¹)
          blend_ = clampf(blend_ + ctrlDt_ / 300.f, 0, 1);
          const float w = blend_ * blend_ * (3 - 2 * blend_);
          for (int a = 0; a < NA; a++) {
            q[a] = holdQ_[a] + (q[a] - holdQ_[a]) * w;
            tau[a] = holdTau_[a] + (tau[a] - holdTau_[a]) * w;
            v[a] *= w;
          }
        }
        for (int a = 0; a < NA; a++) {
          if (fabsf(v[a]) > cfg_.vmax[a]) { v[a] = clampf(v[a], -cfg_.vmax[a], cfg_.vmax[a]); lim = true; }
          if (fabsf(tau[a]) > cfg_.tauMax[a]) lim = true;
          sendMit(a, q[a], v[a], tau[a], cfg_.kp[a], cfg_.kd[a]);
          lastTau_[a] = tau[a];
        }
        if (lim) limited_ = true;
        return;
      }
      [[fallthrough]];
    case State::READY:
    case State::HOLD:
      for (int a = 0; a < NA; a++) sendMit(a, holdQ_[a], 0, holdTau_[a], cfg_.kp[a], cfg_.kd[a]);
      return;
    case State::DAMP:
    case State::FAULT:
      for (int a = 0; a < NA; a++) sendMit(a, q_[a], 0, holdTau_[a], 0, cfg_.kdDamp[a]);
      return;
    default:
      return;
  }
}

void Bridge::tick1ms() {
  now_++;
  readFeedback();
  if (hal_.estopActive() && state_ != State::ESTOP) {
    sendSpecial(SP_DISABLE);
    state_ = State::ESTOP;
    reply("EVT ESTOP");
  }
  if (disableAt_ && (int32_t)(now_ - disableAt_) >= 0) {
    disableAt_ = 0;
    if (state_ == State::DAMP) {
      sendSpecial(SP_DISABLE);
      state_ = State::IDLE;
      reply("EVT DISABLED");
    }
  }
  // Cadence de commande fractionnaire (ex. 400 Hz → pas de 2 ou 3 ms, 2,5 ms en moyenne)
  ctrlAcc_ += (float)cfg_.ctrlHz / 1000.f;
  if (ctrlAcc_ >= 1.f) {
    ctrlAcc_ -= 1.f;
    if (ctrlAcc_ > 1.f) ctrlAcc_ = 0;
    ctrlDt_ = (float)(now_ - lastCtrl_);
    lastCtrl_ = now_;
    controlStep();
  }
  if (statusHz_ && (int32_t)(now_ - nextStatus_) >= 0) {
    nextStatus_ = now_ + 1000 / statusHz_;
    sendStatus();
  }
}

}  // namespace mit
