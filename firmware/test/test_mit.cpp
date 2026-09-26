// Tests natifs du pont CAN « mode MIT » (ORION-6 PRO) : encodage des trames (identique à
// studio/core/protocol.js), cycle d’activation, flux de consignes avec compensation de gravité,
// chien de garde, sécurités. Six moteurs « Damiao » sont simulés sur un faux bus CAN.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <deque>
#include <string>
#include <vector>

#include "../mit_bridge/src/mit_core.h"

using namespace mit;

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...)                                          \
  do {                                                            \
    g_checks++;                                                   \
    if (!(cond)) {                                                \
      g_fail++;                                                   \
      printf("  ÉCHEC %s:%d : %s — ", __FILE__, __LINE__, #cond); \
      printf(__VA_ARGS__);                                        \
      printf("\n");                                               \
    }                                                             \
  } while (0)

static const float PI_F = 3.14159265f;

// ------------------------------------------------------------------ moteurs simulés
struct SimMotor {
  uint8_t id = 1;
  Limits lim{12.5f, 8.f, 28.f, 500.f, 5.f};
  bool enabled = false;
  float p = 0, v = 0;            // rad, rad/s (sortie)
  float J = 0.02f;               // inertie ramenée (kg·m²)
  float gravity = 0;             // couple de gravité max (N·m) : τg = −gravity·sin(p + phase)
  float phase = 0;
  Command cmd{0, 0, 0, 0, 0};
  float tauOut = 0;
  uint8_t tMos = 35, tRotor = 38, err = 0;
  bool mute = false;
  int specials[256] = {0};

  void step(float dt) {
    const float tauG = -gravity * sinf(p + phase);
    float tau = 0;
    if (enabled) tau = cmd.kp * (cmd.p - p) + cmd.kd * (cmd.v - v) + cmd.t;
    tau = fmaxf(-lim.tMax, fminf(lim.tMax, tau));
    tauOut = tau;
    const float acc = (tau + tauG - 0.05f * v) / J;
    v += acc * dt;
    p += v * dt;
  }
  void feedback(uint8_t out[8]) const {
    const uint32_t pi = floatToUint(p, -lim.pMax, lim.pMax, 16);
    const uint32_t vi = floatToUint(v, -lim.vMax, lim.vMax, 12);
    const uint32_t ti = floatToUint(tauOut, -lim.tMax, lim.tMax, 12);
    out[0] = (uint8_t)(((enabled ? (err ? err : 1) : 0) << 4) | (id & 0x0F));
    out[1] = (uint8_t)(pi >> 8);
    out[2] = (uint8_t)(pi & 0xFF);
    out[3] = (uint8_t)(vi >> 4);
    out[4] = (uint8_t)(((vi & 0xF) << 4) | (ti >> 8));
    out[5] = (uint8_t)(ti & 0xFF);
    out[6] = tMos;
    out[7] = tRotor;
  }
};

struct FakeHal : BridgeHal {
  SimMotor m[NA];
  std::deque<std::pair<uint32_t, std::vector<uint8_t>>> rx;
  std::vector<std::string> lines;
  std::vector<uint8_t> eeprom;
  bool estop = false;
  int sent = 0, sentMit = 0;
  Command lastCmd[NA];

  FakeHal() {
    for (int a = 0; a < NA; a++) {
      m[a].id = orion_cfg::CAN_ID[a];
      m[a].lim = {orion_cfg::MIT_PMAX[a], orion_cfg::MIT_VMAX[a], orion_cfg::MIT_TMAX[a], orion_cfg::MIT_KPMAX[a], orion_cfg::MIT_KDMAX[a]};
    }
  }
  bool canSend(uint32_t id, const uint8_t d[8]) override {
    sent++;
    for (int a = 0; a < NA; a++) {
      SimMotor& s = m[a];
      if (s.id != id) continue;
      bool special = true;
      for (int k = 0; k < 7; k++) special = special && d[k] == 0xFF;
      if (special && d[7] >= 0xFB) {
        s.specials[d[7]]++;
        if (d[7] == SP_ENABLE) s.enabled = true;
        if (d[7] == SP_DISABLE) s.enabled = false;
        if (d[7] == SP_ZERO) s.p = 0;
        if (d[7] == SP_CLEAR) s.err = 0;
      } else {
        sentMit++;
        Command c;
        c.p = uintToFloat(((uint32_t)d[0] << 8) | d[1], -s.lim.pMax, s.lim.pMax, 16);
        c.v = uintToFloat(((uint32_t)d[2] << 4) | (d[3] >> 4), -s.lim.vMax, s.lim.vMax, 12);
        c.kp = uintToFloat(((uint32_t)(d[3] & 0xF) << 8) | d[4], 0, s.lim.kpMax, 12);
        c.kd = uintToFloat(((uint32_t)d[5] << 4) | (d[6] >> 4), 0, s.lim.kdMax, 12);
        c.t = uintToFloat(((uint32_t)(d[6] & 0xF) << 8) | d[7], -s.lim.tMax, s.lim.tMax, 12);
        s.cmd = c;
        lastCmd[a] = c;
      }
      if (!s.mute) {
        uint8_t fb[8];
        s.feedback(fb);
        rx.emplace_back(0x10 + s.id, std::vector<uint8_t>(fb, fb + 8));
      }
    }
    return true;
  }
  bool canReceive(uint32_t& id, uint8_t d[8]) override {
    if (rx.empty()) return false;
    id = rx.front().first;
    memcpy(d, rx.front().second.data(), 8);
    rx.pop_front();
    return true;
  }
  void write(const char* line) override { lines.emplace_back(line); }
  bool estopActive() override { return estop; }
  bool saveBlob(const void* d, uint32_t n) override {
    eeprom.assign((const uint8_t*)d, (const uint8_t*)d + n);
    return true;
  }
  bool loadBlob(void* d, uint32_t n) override {
    if (eeprom.size() != n) return false;
    memcpy(d, eeprom.data(), n);
    return true;
  }
};

struct Bench {
  FakeHal hal;
  Bridge br{hal};
  Bench() { br.begin(); }
  void send(const std::string& s) {
    uint8_t c = 0;
    for (char ch : s) c ^= (uint8_t)ch;
    char buf[8];
    snprintf(buf, sizeof(buf), "*%02X\n", c);
    for (char ch : s + buf) br.onChar(ch);
  }
  void run(int ms) {
    for (int i = 0; i < ms; i++) {
      for (int k = 0; k < 10; k++)
        for (auto& m : hal.m) m.step(1e-4f);
      br.tick1ms();
    }
  }
  bool got(const char* prefix) {
    for (size_t i = 0; i < hal.lines.size(); i++)
      if (hal.lines[i].rfind(prefix, 0) == 0) { hal.lines.erase(hal.lines.begin() + (long)i); return true; }
    return false;
  }
  std::string last() const { return hal.lines.empty() ? "" : hal.lines.back(); }
  float trueDeg(int a) const {  // position articulaire réelle, avec le repère configuré
    return br.config().dir[a] * hal.m[a].p * 180.f / PI_F + br.config().offsetDeg[a];
  }
};

static std::string hex8(const uint8_t* d) {
  char b[32];
  snprintf(b, sizeof(b), "%02X %02X %02X %02X %02X %02X %02X %02X", d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
  return b;
}

// ------------------------------------------------------------------ tests
static void testFrames() {
  printf("• trames MIT (identiques à studio/core/protocol.js)\n");
  const Limits lim{12.5f, 8.f, 28.f, 500.f, 5.f};
  struct Case { Command c; const char* hex; } cases[] = {
      {{0, 0, 0, 0, 0}, "80 00 80 00 00 00 08 00"},
      {{1.2345f, -3.5f, 120.f, 3.5f, 7.25f}, "8C A4 48 03 D7 B3 3A 12"},
      {{-12.5f, 8.f, 500.f, 5.f, -28.f}, "00 00 FF FF FF FF F0 00"},
      {{20.f, -20.f, -5.f, 9.f, 100.f}, "FF FF 00 00 00 FF FF FF"},
      {{-0.5f, 0.1f, 30.f, 1.f, -2.5f}, "7A E1 81 90 F6 33 37 49"},
  };
  // Ordre des champs de Command : p, v, kp, kd, t
  for (auto& k : cases) {
    uint8_t d[8];
    packCommand(k.c, lim, d);
    CHECK(hex8(d) == k.hex, "p=%.4f → %s (attendu %s)", k.c.p, hex8(d).c_str(), k.hex);
  }
  const uint8_t fbd[8] = {0x13, 0x8a, 0x3d, 0x7f, 0xf8, 0x12, 0x2a, 0x30};
  Feedback fb;
  unpackFeedback(fbd, lim, fb);
  CHECK(fb.id == 3 && fb.err == 1 && fabsf(fb.p - 1.0000381f) < 1e-5f && fabsf(fb.v + 0.0019536f) < 1e-5f &&
            fabsf(fb.t - 0.25299145f) < 1e-5f && fb.tMos == 42 && fb.tRotor == 48,
        "retour : id %u err %u p %.6f v %.6f t %.6f", fb.id, fb.err, fb.p, fb.v, fb.t);
  uint8_t sp[8];
  specialFrame(SP_ENABLE, sp);
  CHECK(hex8(sp) == "FF FF FF FF FF FF FF FC", "trame d’activation %s", hex8(sp).c_str());
}

// Couple de gravité appliqué au moteur simulé a (N·m), et sa compensation parfaite
static float gravityOf(const SimMotor& m, float qRad) { return m.gravity * sinf(qRad + m.phase); }

static void testEnableAndStream() {
  printf("• activation, maintien, flux MC avec compensation de gravité\n");
  Bench b;
  // Positions de départ proches de la pose de repos, gravité sur J2/J3
  for (int a = 0; a < NA; a++) b.hal.m[a].p = orion_cfg::REF_POSE_DEG[a] * PI_F / 180.f;
  b.send("EN 1");
  CHECK(b.got("ERR 6"), "EN accepté sans retour des moteurs : %s", b.last().c_str());
  b.run(60);  // interrogation à 50 Hz (gains nuls)
  for (int a = 0; a < NA; a++)
    CHECK(fabsf(b.br.qDeg(a) - orion_cfg::REF_POSE_DEG[a]) < 0.05f, "J%d lu %.3f°", a + 1, b.br.qDeg(a));
  CHECK(b.hal.lastCmd[1].kp == 0 && b.hal.lastCmd[1].kd == 0 && fabsf(b.hal.lastCmd[1].t) < 0.01f, "trames d’interrogation à couple nul");
  // Le bras reposait sur son support ; la gravité agit dès qu’on l’en retire
  b.hal.m[1].gravity = 9.f;
  b.hal.m[2].gravity = 4.f;
  b.send("EN 1");
  CHECK(b.got("OK EN 1"), "EN : %s", b.last().c_str());
  for (int a = 0; a < NA; a++) CHECK(b.hal.m[a].specials[SP_ENABLE] == 1 && b.hal.m[a].enabled, "trame d’activation J%d", a + 1);
  CHECK(b.br.state() == State::READY, "état %s", stateName(b.br.state()));
  b.run(200);
  CHECK(fabsf(b.hal.lastCmd[1].kp - orion_cfg::MIT_KP[1]) < 0.2f, "maintien avec Kp : %.2f", b.hal.lastCmd[1].kp);
  // Flux : sinusoïdes autour de la pose de repos, émises à ~60 Hz avec gigue, avec τff = gravité
  // Amplitude bornée par les butées (J3 est à 5° de sa butée haute en pose de repos)
  auto amp = [](int a) {
    return fminf(20.f, fminf(orion_cfg::MAX_DEG[a] - orion_cfg::REF_POSE_DEG[a], orion_cfg::REF_POSE_DEG[a] - orion_cfg::MIN_DEG[a]) - 1.f);
  };
  auto ref = [&](int a, float t) { return orion_cfg::REF_POSE_DEG[a] + amp(a) * sinf(2.f * PI_F * 0.3f * t + a) * (t < 1 ? t * t * (3 - 2 * t) : 1.f); };
  int nextSend = 0;
  uint32_t seq = 100;
  float maxErr = 0;
  srand(7);
  const int T = 4000;
  for (int now = 0; now < T + 300; now++) {
    if (now >= nextSend && now < T) {
      const float t = now / 1000.f;
      char buf[320];
      int n = snprintf(buf, sizeof(buf), "MC %u %u", seq++ & 0xFFFF, (unsigned)now & 0xFFFF);
      for (int a = 0; a < NA; a++) n += snprintf(buf + n, sizeof(buf) - n, " %.4f", ref(a, t));
      for (int a = 0; a < NA; a++) n += snprintf(buf + n, sizeof(buf) - n, " %.3f", (ref(a, t + 1e-3f) - ref(a, t - 1e-3f)) / 2e-3f);
      for (int a = 0; a < NA; a++) {
        const SimMotor& m = b.hal.m[a];
        n += snprintf(buf + n, sizeof(buf) - n, " %.3f", gravityOf(m, ref(a, t) * PI_F / 180.f));
      }
      b.send(buf);
      nextSend = now + 15 + rand() % 4;
    }
    b.run(1);
    if (now > 1500 && now < T) {
      const float td = (now - (int)b.br.config().streamDelayMs) / 1000.f;
      for (int a = 0; a < NA; a++) {
        const float e = fabsf(b.trueDeg(a) - ref(a, td));
        if (e > maxErr) maxErr = e;
      }
    }
  }
  int errs = 0;
  for (auto& l : b.hal.lines) errs += l.rfind("ERR", 0) == 0;
  CHECK(errs == 0, "%d erreurs pendant le flux", errs);
  CHECK(maxErr < 1.5f, "erreur de suivi max %.3f°", maxErr);
  printf("    erreur de suivi max : %.3f° (Kp/Kd + τff gravité)\n", maxErr);
  // Arrêt du flux → chien de garde → maintien
  CHECK(b.got("EVT WATCHDOG"), "chien de garde non déclenché");
  CHECK(b.br.state() == State::HOLD, "état après chien de garde : %s", stateName(b.br.state()));
  const float q2 = b.trueDeg(1);
  b.run(1000);
  CHECK(fabsf(b.trueDeg(1) - q2) < 1.0f, "dérive en maintien %.3f°", b.trueDeg(1) - q2);
  CHECK(b.br.flags() & F_WATCHDOG, "flag chien de garde");
  // Consigne au-delà de la butée haute de J3 : écrêtée par le pont
  b.send("STOP");
  b.run(10);
  char buf[200];
  float maxCmd = -1e9f;
  for (int k = 0; k < 150; k++) {
    int n = snprintf(buf, sizeof(buf), "MC %d %d", 500 + k, 20000 + 10 * k);
    for (int a = 0; a < NA; a++) n += snprintf(buf + n, sizeof(buf) - n, " %.3f", a == 2 ? b.br.qDeg(2) + fminf(k * 0.2f, 12.f) : b.br.cmdDeg(a));
    b.send(buf);
    b.run(10);
    if (b.br.cmdDeg(2) > maxCmd) maxCmd = b.br.cmdDeg(2);
  }
  CHECK(maxCmd <= orion_cfg::MAX_DEG[2] + 1e-3f, "consigne J3 %.3f° > butée %.1f°", maxCmd, orion_cfg::MAX_DEG[2]);
}

static void testSafety() {
  printf("• sécurité (arrêt d’urgence, température, silence CAN, amortissement)\n");
  Bench b;
  b.run(40);
  b.send("EN 1");
  b.run(20);
  CHECK(b.br.state() == State::READY, "état %s", stateName(b.br.state()));
  b.hal.estop = true;
  b.run(2);
  CHECK(b.got("EVT ESTOP"), "arrêt d’urgence non signalé");
  for (int a = 0; a < NA; a++) CHECK(!b.hal.m[a].enabled, "J%d toujours actif", a + 1);
  b.send("EN 1");
  CHECK(b.got("ERR 7"), "EN accepté pendant l’arrêt d’urgence");
  b.hal.estop = false;
  b.send("CLR");
  CHECK(b.got("OK CLR") && b.br.state() == State::IDLE, "acquittement → %s", stateName(b.br.state()));
  b.run(40);
  b.send("EN 1");
  CHECK(b.got("OK EN 1"), "réactivation : %s", b.last().c_str());
  // Surchauffe → défaut + amortissement (Kp = 0)
  b.hal.m[3].tMos = 95;
  b.run(10);
  CHECK(b.got("EVT FAULT J4 : temperature"), "surchauffe non détectée : %s", b.last().c_str());
  CHECK(b.br.state() == State::FAULT, "état %s", stateName(b.br.state()));
  CHECK(b.hal.lastCmd[1].kp == 0 && b.hal.lastCmd[1].kd > 0.5f, "amortissement : kp %.2f kd %.2f", b.hal.lastCmd[1].kp, b.hal.lastCmd[1].kd);
  b.send("MC 1 0 0 0 0 0 0 0");
  CHECK(b.got("ERR 4"), "flux accepté en défaut");
  b.hal.m[3].tMos = 40;
  b.send("CLR");
  CHECK(b.got("OK CLR") && b.br.state() == State::READY, "acquittement du défaut → %s", stateName(b.br.state()));
  // Moteur muet → défaut
  b.hal.m[5].mute = true;
  b.run(100);
  CHECK(b.got("EVT FAULT CAN : J6"), "silence CAN non détecté : %s", b.last().c_str());
  b.hal.m[5].mute = false;
  // Erreur moteur (surintensité)
  Bench c;
  c.run(40);
  c.send("EN 1");
  c.run(5);
  c.hal.m[0].err = 0xA;
  c.run(10);
  CHECK(c.got("EVT FAULT J1 : surintensite"), "code d’erreur moteur non détecté : %s", c.last().c_str());
  // EN 0 : descente amortie puis coupure
  Bench d;
  d.run(40);
  d.send("EN 1");
  d.run(10);
  d.send("EN 0");
  CHECK(d.br.state() == State::DAMP, "EN 0 → %s", stateName(d.br.state()));
  d.run(1600);
  CHECK(d.got("EVT DISABLED") && d.br.state() == State::IDLE, "coupure après amortissement : %s", stateName(d.br.state()));
  for (int a = 0; a < NA; a++) CHECK(!d.hal.m[a].enabled, "J%d toujours actif", a + 1);
  // Démarrage de flux trop loin de la position mesurée
  Bench e;
  e.run(40);
  e.send("EN 1");
  e.run(5);
  e.send("MC 0 0 30 0 0 0 0 0");
  CHECK(e.got("ERR 5 flux"), "démarrage éloigné accepté : %s", e.last().c_str());
}

static void testConfig() {
  printf("• repères, ZERO, paramètres\n");
  Bench b;
  for (int a = 0; a < NA; a++) b.hal.m[a].p = 0.3f * (a + 1) - 1.f;  // position moteur arbitraire
  b.run(40);
  b.send("SET dir 2 -1");
  CHECK(b.got("OK SET dir 2 -1"), "SET dir : %s", b.last().c_str());
  b.send("ZERO");
  CHECK(b.got("OK ZERO"), "ZERO : %s", b.last().c_str());
  b.run(40);
  for (int a = 0; a < NA; a++)
    CHECK(fabsf(b.br.qDeg(a) - orion_cfg::REF_POSE_DEG[a]) < 0.05f, "J%d après ZERO : %.3f° (attendu %.3f°)", a + 1, b.br.qDeg(a), orion_cfg::REF_POSE_DEG[a]);
  // Le sens inversé est respecté : +1° articulaire = −1° moteur sur J2
  b.send("EN 1");
  b.run(20);
  const float p0 = b.hal.m[1].p;
  char buf[200];
  for (int k = 0; k < 120; k++) {
    int n = snprintf(buf, sizeof(buf), "MC %d %d", k, k * 10);
    for (int a = 0; a < NA; a++) n += snprintf(buf + n, sizeof(buf) - n, " %.3f", orion_cfg::REF_POSE_DEG[a] + (a == 1 ? fminf(k * 0.1f, 5.f) : 0.f));
    b.send(buf);
    b.run(10);
  }
  b.run(300);
  const float dp = (b.hal.m[1].p - p0) * 180.f / PI_F;
  CHECK(dp < -4.0f && dp > -6.0f, "déplacement moteur J2 %.3f° (attendu ≈ −5°)", dp);
  b.send("SET kp 1 900");
  CHECK(b.got("ERR 3 kp hors plage"), "Kp > Kp_max accepté : %s", b.last().c_str());
  b.send("SET kp * 60");
  b.send("GET kp 4");
  CHECK(b.got("OK 60"), "SET kp * : %s", b.last().c_str());
  b.send("SET can_id 1 9");
  CHECK(b.got("ERR 4"), "changement d’ID CAN moteurs actifs accepté");
  b.send("SAVE");
  CHECK(b.got("OK SAVE"), "SAVE");
  Bench c;
  c.hal.eeprom = b.hal.eeprom;
  c.br.begin();
  c.send("GET kp 4");
  CHECK(c.got("OK 60"), "configuration rechargée : %s", c.last().c_str());
  c.send("GET dir 2");
  CHECK(c.got("OK -1"), "sens rechargé : %s", c.last().c_str());
  c.send("PING");
  CHECK(c.got("OK PONG ORION-6 PRO mit-bridge1.0.0 axes=6"), "PING : %s", c.last().c_str());
}

int main() {
  printf("Tests du pont CAN mode MIT (%d axes)\n", NA);
  testFrames();
  testEnableAndStream();
  testSafety();
  testConfig();
  printf("%d vérifications, %d échec(s)\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
