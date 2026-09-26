// Tests natifs (PC) du cœur du firmware ORION-6 : protocole, mouvements, prise d’origine,
// flux de consignes, chien de garde, sécurité. Un faux matériel (FakeHal) simule les
// drivers pas-à-pas (contrôle des temps DIR/STEP), les capteurs et l’EEPROM.
//
//   make -C firmware/test        (ou : npm run test:firmware)
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <functional>
#include <string>
#include <vector>

#include "../orion_fw/src/core/robot.h"

using namespace orion;

static int g_fail = 0, g_checks = 0;
#define CHECK(cond, ...)                                              \
  do {                                                                \
    g_checks++;                                                       \
    if (!(cond)) {                                                    \
      g_fail++;                                                       \
      printf("  ÉCHEC %s:%d : %s — ", __FILE__, __LINE__, #cond);     \
      printf(__VA_ARGS__);                                            \
      printf("\n");                                                   \
    }                                                                 \
  } while (0)

// ------------------------------------------------------------------ faux matériel
struct FakeHal : Hal {
  int64_t isrTick = 0;
  // État physique simulé des moteurs (pas) et contrôles de chronogramme
  int32_t motor[NA] = {0};
  bool dirPin[NA] = {false};
  bool stepPin[NA] = {false};
  int64_t lastDirChange[NA], lastStepRise[NA], lastStepFall[NA];
  int timingErrors = 0;
  bool drivers = false;
  // Capteurs : l’articulation « vraie » vaut offset + motor/spd (degrés)
  float trueOffsetDeg[NA] = {0};
  float switchDeg[NA];
  int8_t switchSide[NA];  // -1 : actif sous switchDeg, +1 : actif au-dessus
  bool switchesWired = true;
  bool estop = false;
  bool inputs[8] = {false};
  bool outputs[8] = {false};
  uint16_t gripUs = 0;
  std::vector<std::string> lines;
  std::vector<uint8_t> eeprom;
  Config* cfg = nullptr;

  FakeHal() {
    for (int a = 0; a < NA; a++) {
      lastDirChange[a] = lastStepRise[a] = lastStepFall[a] = -100;
      switchDeg[a] = orion_cfg::HOMING_SWITCH_POS[a];
      switchSide[a] = orion_cfg::HOMING_DIR[a] < 0 ? -1 : 1;
    }
  }
  float trueDeg(int a) const { return trueOffsetDeg[a] + motor[a] / orion_cfg::STEPS_PER_DEG[a]; }
  void stepHigh(uint8_t a) override {
    if (!drivers) timingErrors += 0;  // impulsions ignorées moteurs coupés
    if (stepPin[a]) timingErrors++;
    if (isrTick - lastDirChange[a] < 1) timingErrors++;  // DIR doit précéder STEP d’au moins 10 µs
    if (isrTick - lastStepFall[a] < 1) timingErrors++;   // niveau bas ≥ 10 µs
    stepPin[a] = true;
    lastStepRise[a] = isrTick;
    if (drivers) motor[a] += dirPin[a] ? 1 : -1;
  }
  void stepLow(uint8_t a) override {
    if (!stepPin[a]) timingErrors++;
    stepPin[a] = false;
    lastStepFall[a] = isrTick;
  }
  void setDir(uint8_t a, bool positive) override {
    if (stepPin[a]) timingErrors++;  // DIR ne doit pas changer pendant une impulsion
    if (dirPin[a] != positive) lastDirChange[a] = isrTick;
    dirPin[a] = positive;
  }
  void enableDrivers(bool on) override { drivers = on; }
  bool limitActive(uint8_t a) override {
    if (!switchesWired) return false;
    const float q = trueDeg(a);
    return switchSide[a] < 0 ? q <= switchDeg[a] : q >= switchDeg[a];
  }
  bool estopActive() override { return estop; }
  void setGripperUs(uint16_t us) override { gripUs = us; }
  void setOutput(uint8_t n, bool v) override { outputs[n] = v; }
  bool readInput(uint8_t n) override { return inputs[n]; }
  void write(const char* line) override { lines.emplace_back(line); }
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

// ------------------------------------------------------------------ banc de test
struct Bench {
  FakeHal hal;
  Robot robot{hal};
  std::function<void(int)> onTick;  // appelé chaque ms (avant tick1ms)
  // Enregistrement de la vitesse physique (pas par ms) pour vérifier les limites
  float maxVel[NA] = {0};
  float maxAcc[NA] = {0};
  float prevVel[NA] = {0};
  int32_t prevMotor[NA] = {0};
  std::vector<int32_t> hist[NA];

  Bench() { robot.begin(); }

  void send(const std::string& s, bool withChecksum = true) {
    std::string line = s;
    if (withChecksum) {
      uint8_t c = 0;
      for (char ch : s) c ^= (uint8_t)ch;
      char buf[8];
      snprintf(buf, sizeof(buf), "*%02X", c);
      line += buf;
    }
    line += "\n";
    for (char ch : line) robot.onChar(ch);
  }
  void run(int ms) {
    for (int i = 0; i < ms; i++) {
      if (onTick) onTick(robot.millis());
      robot.tick1ms();
      for (uint32_t k = 0; k < ISR_PER_MS; k++) {
        hal.isrTick++;
        robot.stepIsr();
      }
      for (int a = 0; a < NA; a++) {
        hist[a].push_back(hal.motor[a]);
        if (hist[a].size() > 10) hist[a].erase(hist[a].begin());
        if (hist[a].size() == 10) {
          // vitesse moyenne sur 9 ms (lisse la quantification en pas)
          const float v = fabsf((float)(hist[a].back() - hist[a].front())) / 9.f;
          if (v > maxVel[a]) maxVel[a] = v;
        }
      }
    }
  }
  bool runUntil(const std::function<bool()>& cond, int maxMs) {
    for (int i = 0; i < maxMs; i++) {
      if (cond()) return true;
      run(1);
    }
    return cond();
  }
  // Recherche d’une ligne commençant par prefix (et la retire du journal si trouvée)
  bool got(const char* prefix) {
    for (size_t i = 0; i < hal.lines.size(); i++) {
      if (hal.lines[i].rfind(prefix, 0) == 0) {
        hal.lines.erase(hal.lines.begin() + (long)i);
        return true;
      }
    }
    return false;
  }
  std::string last() const { return hal.lines.empty() ? "" : hal.lines.back(); }
  void resetVel() {
    for (int a = 0; a < NA; a++) { maxVel[a] = 0; hist[a].clear(); }
  }
  void enableAndZero() {
    // Le bras est physiquement dans la pose Home quand on envoie ZERO
    for (int a = 0; a < NA; a++) hal.trueOffsetDeg[a] = orion_cfg::HOME_POSE_DEG[a] - hal.motor[a] / orion_cfg::STEPS_PER_DEG[a];
    send("EN 1");
    send("ZERO");
    run(5);
    CHECK(robot.homed(), "ZERO devrait déclarer le robot référencé");
  }
};

static bool checksumOk(const std::string& l) {
  const size_t star = l.rfind('*');
  if (star == std::string::npos) return false;
  uint8_t c = 0;
  for (size_t i = 0; i < star; i++) c ^= (uint8_t)l[i];
  return (unsigned)strtoul(l.substr(star + 1, 2).c_str(), nullptr, 16) == c && l.back() == '\n';
}

static float vmaxSteps(int a) { return orion_cfg::VMAX_DEG_S[a] * orion_cfg::STEPS_PER_DEG[a] / 1000.f; }

// ------------------------------------------------------------------ tests
static void testProtocol() {
  printf("• protocole\n");
  Bench b;
  CHECK(b.got("EVT BOOT ORION-6"), "message de démarrage absent");
  b.send("PING");
  CHECK(!b.hal.lines.empty() && b.hal.lines.back().rfind("OK PONG ORION-6 fw1.0.0 axes=6", 0) == 0, "PONG : %s", b.last().c_str());
  CHECK(checksumOk(b.last()), "somme de contrôle de la réponse : %s", b.last().c_str());
  b.send("ping", false);
  CHECK(b.got("OK PONG"), "commande sans somme / en minuscules refusée");
  b.send("PING*00", false);
  CHECK(b.got("ERR 1"), "mauvaise somme de contrôle non détectée");
  b.send("FOO");
  CHECK(b.got("ERR 2"), "commande inconnue : %s", b.last().c_str());
  b.send(std::string(300, 'A'), false);
  CHECK(b.got("ERR 3 ligne trop longue"), "ligne trop longue : %s", b.last().c_str());
  b.send("MJ 0 0 0 0 0 90 0");
  CHECK(b.got("ERR 4"), "MJ sans moteurs actifs devrait être refusé");
  b.send("EN 1");
  CHECK(b.got("OK EN 1") && b.hal.drivers, "activation des drivers");
  b.send("MJ 0 0 0 0 0 90 0");
  CHECK(b.got("ERR 4 prise d'origine"), "MJ sans prise d’origine : %s", b.last().c_str());
  b.send("STAT");
  CHECK(b.hal.lines.back().rfind("ST READY ", 0) == 0, "STAT : %s", b.last().c_str());
  // Format ST : ST état t q1..q6 flags di
  {
    char st[16];
    unsigned long t;
    float q[6];
    unsigned flags, di;
    const int n = sscanf(b.last().c_str(), "ST %15s %lu %f %f %f %f %f %f %x %x", st, &t, q, q + 1, q + 2, q + 3, q + 4, q + 5, &flags, &di);
    CHECK(n == 10, "ST mal formé (%d champs) : %s", n, b.last().c_str());
    CHECK((flags & F_ENABLED) && !(flags & F_HOMED), "flags ST = %x", flags);
  }
  b.send("GRIP 50");
  CHECK(b.got("OK GRIP 50") && b.hal.gripUs == (orion_cfg::GRIPPER_US_CLOSED + orion_cfg::GRIPPER_US_OPEN) / 2, "pince : %u µs", b.hal.gripUs);
  b.send("DO 3 1");
  CHECK(b.got("OK DO 3 1") && b.hal.outputs[3], "sortie TOR");
  b.hal.inputs[1] = b.hal.inputs[6] = true;
  b.send("DI");
  CHECK(b.got("OK DI 42"), "entrées TOR : %s", b.last().c_str());
  b.send("STREAM 100");
  b.hal.lines.clear();
  b.run(100);
  int nst = 0;
  for (auto& l : b.hal.lines) nst += l.rfind("ST ", 0) == 0;
  CHECK(nst >= 9 && nst <= 11, "STREAM 100 Hz : %d lignes ST en 100 ms", nst);
  b.send("STREAM 0");
}

static void testMoveJ() {
  printf("• MoveJ (trapèze synchronisé, limites, précision)\n");
  Bench b;
  b.enableAndZero();
  b.hal.lines.clear();
  b.resetVel();
  const float target[6] = {45, -30, 40, 90, -60, 170};
  b.send("MJ 0 45 -30 40 90 -60 170");
  float T = 0;
  CHECK(sscanf(b.last().c_str(), "OK MJ %f", &T) == 1 && T > 0.5f, "réponse MJ : %s", b.last().c_str());
  const bool done = b.runUntil([&] { return b.got("EVT DONE"); }, (int)(T * 1000) + 200);
  CHECK(done, "EVT DONE non reçu");
  b.run(5);
  for (int a = 0; a < NA; a++) {
    CHECK(fabsf(b.hal.trueDeg(a) - target[a]) * orion_cfg::STEPS_PER_DEG[a] <= 1.0f, "J%d : %.3f°, attendu %.3f°", a + 1, b.hal.trueDeg(a), target[a]);
    CHECK(b.maxVel[a] <= vmaxSteps(a) * 1.06f, "J%d vitesse max %.2f pas/ms > %.2f", a + 1, b.maxVel[a], vmaxSteps(a));
  }
  CHECK(b.robot.state() == State::HOLD, "état après MJ");
  CHECK(b.hal.timingErrors == 0, "%d violations de chronogramme STEP/DIR", b.hal.timingErrors);
  // Retour (changement de sens sur tous les axes)
  b.send("MJ 1.5 0 0 0 0 90 0");
  CHECK(b.runUntil([&] { return b.got("EVT DONE"); }, 5000), "retour non terminé");
  b.run(3);
  for (int a = 0; a < NA; a++) {
    const float exp = a == 4 ? 90.f : 0.f;
    CHECK(fabsf(b.hal.trueDeg(a) - exp) * orion_cfg::STEPS_PER_DEG[a] <= 1.0f, "retour J%d : %.3f° (attendu %.3f°)", a + 1, b.hal.trueDeg(a), exp);
  }
  CHECK(b.hal.timingErrors == 0, "%d violations de chronogramme", b.hal.timingErrors);
  // Durée imposée respectée (1,5 s > durée minimale)
  // Butées logicielles
  b.send("MJ 0 0 150 0 0 90 0");
  CHECK(b.got("ERR 5 J2"), "butée J2 non vérifiée : %s", b.last().c_str());
  b.send("MJ 0 0 0 0 0 90");
  CHECK(b.got("ERR 3"), "MJ incomplet accepté");
}

static void testStop() {
  printf("• STOP (arrêt contrôlé)\n");
  Bench b;
  b.enableAndZero();
  b.send("MJ 0 150 0 0 0 90 0");
  b.run(900);  // en pleine vitesse
  const float v0 = b.robot.velDeg(0);
  CHECK(v0 > 50, "J1 devrait être lancé (%.1f °/s)", v0);
  b.send("STOP");
  const bool stopped = b.runUntil([&] { return b.got("EVT STOPPED"); }, 2000);
  CHECK(stopped, "EVT STOPPED absent");
  // Distance de freinage ≈ v²/(2a)
  const float t = v0 / orion_cfg::AMAX_DEG_S2[0];
  CHECK(t < 1.0f, "temps de freinage %.2f s", t);
  CHECK(b.robot.state() == State::HOLD, "état après STOP");
  CHECK(b.hal.timingErrors == 0, "%d violations de chronogramme", b.hal.timingErrors);
}

static void testHoming() {
  printf("• prise d’origine (capteurs simulés)\n");
  Bench b;
  // Position réelle inconnue au démarrage
  const float start[6] = {35.5f, -20.25f, 10.f, -95.f, 42.f, 120.f};
  for (int a = 0; a < NA; a++) b.hal.trueOffsetDeg[a] = start[a];
  b.send("EN 1");
  b.send("HOME");
  CHECK(b.got("OK HOME 0x3F"), "HOME : %s", b.last().c_str());
  const bool ok = b.runUntil([&] { return b.got("EVT HOMED*"); }, 120000);
  CHECK(ok, "prise d’origine non terminée (état %s, dernier : %s)", stateName(b.robot.state()), b.last().c_str());
  CHECK(b.runUntil([&] { return b.got("EVT DONE"); }, 10000), "retour en pose Home absent");
  for (int a = 0; a < NA; a++) {
    const float trueQ = b.hal.trueDeg(a);
    const float exp = orion_cfg::HOME_POSE_DEG[a];
    CHECK(fabsf(trueQ - exp) < 0.02f, "J%d réel %.3f° (attendu %.3f°)", a + 1, trueQ, exp);
    CHECK(fabsf(b.robot.posDeg(a) - trueQ) < 0.02f, "J%d estimé %.3f° / réel %.3f°", a + 1, b.robot.posDeg(a), trueQ);
  }
  CHECK(b.robot.homed() && b.robot.state() == State::HOLD, "état final %s", stateName(b.robot.state()));
  CHECK(b.hal.timingErrors == 0, "%d violations de chronogramme", b.hal.timingErrors);
  // Axe démarrant sur son capteur
  Bench c;
  for (int a = 0; a < NA; a++) c.hal.trueOffsetDeg[a] = 0;
  c.hal.trueOffsetDeg[0] = -173.f;  // J1 déjà sur le capteur
  c.send("EN 1");
  c.send("HOME 1");
  CHECK(c.runUntil([&] { return c.got("EVT HOMED*"); }, 60000), "prise d’origine depuis le capteur");
  CHECK(fabsf(c.robot.posDeg(0) - c.hal.trueDeg(0)) < 0.02f, "J1 estimé %.3f / réel %.3f", c.robot.posDeg(0), c.hal.trueDeg(0));
  // Capteur absent → défaut
  Bench d;
  d.hal.switchesWired = false;
  d.send("EN 1");
  d.send("HOME 1");
  CHECK(d.runUntil([&] { return d.got("EVT FAULT prise d'origine J1"); }, 400000), "capteur introuvable non détecté");
  CHECK(d.robot.state() == State::FAULT, "état %s", stateName(d.robot.state()));
}

static void testStreaming() {
  printf("• flux de consignes SP (60 Hz irrégulier, Hermite, chien de garde)\n");
  Bench b;
  b.enableAndZero();
  b.hal.lines.clear();
  b.resetVel();
  // Trajectoire de référence : sinusoïdes, émises à ~60 Hz avec gigue d’arrivée
  auto ref = [](int a, float t) { return (a == 4 ? 90.f : 0.f) + 25.f * sinf(2.f * 3.14159f * 0.4f * t + a); };
  auto dref = [](int a, float t) { return 25.f * 2.f * 3.14159f * 0.4f * cosf(2.f * 3.14159f * 0.4f * t + a); };
  // Démarrage progressif : amplitude multipliée par une rampe pour partir de la pose actuelle
  auto ramp = [](float t) { return t < 1.f ? t * t * (3 - 2 * t) : 1.f; };
  auto dramp = [](float t) { return t < 1.f ? 6 * t * (1 - t) : 0.f; };
  uint32_t seq = 65530;  // test du rebouclage 16 bits
  int nextSend = 0;
  float maxErr = 0;
  srand(42);
  const int emitMs = 5000;
  std::vector<std::pair<int, std::string>> pending;  // (heure d’arrivée, ligne)
  b.onTick = [&](int now) {
    if (now >= nextSend && now < emitMs) {
      const float t = now / 1000.f;
      char buf[200];
      int n = snprintf(buf, sizeof(buf), "SP %u %u", seq & 0xFFFF, (unsigned)(now + 60000) & 0xFFFF);
      for (int a = 0; a < NA; a++) {
        const float base = a == 4 ? 90.f : 0.f;
        n += snprintf(buf + n, sizeof(buf) - n, " %.4f", base + (ref(a, t) - base) * ramp(t));
      }
      for (int a = 0; a < NA; a++) {
        const float base = a == 4 ? 90.f : 0.f;
        n += snprintf(buf + n, sizeof(buf) - n, " %.3f", dref(a, t) * ramp(t) + (ref(a, t) - base) * dramp(t));
      }
      seq++;
      // Gigue USB : arrivée retardée de 0 à 8 ms (ordre préservé)
      int arrive = now + rand() % 9;
      if (!pending.empty() && arrive < pending.back().first) arrive = pending.back().first;
      pending.emplace_back(arrive, buf);
      nextSend = now + 15 + rand() % 4;  // 55–67 Hz
    }
    while (!pending.empty() && pending.front().first <= now) {
      b.send(pending.front().second);
      pending.erase(pending.begin());
    }
    // Erreur de suivi (position réelle vs référence retardée du tampon de lissage)
    if (now > 1500 && now < emitMs) {
      const float td = (now - (int)b.robot.config().streamDelayMs - 4) / 1000.f;  // retard nominal ≈ D + gigue moyenne
      for (int a = 0; a < NA; a++) {
        const float e = fabsf(b.hal.trueDeg(a) - ref(a, td));
        if (e > maxErr) maxErr = e;
      }
    }
  };
  b.run(emitMs + 50);
  int errs = 0;
  for (auto& l : b.hal.lines) errs += l.rfind("ERR", 0) == 0;
  CHECK(errs == 0, "%d erreurs pendant le flux (ex. %s)", errs, b.hal.lines.empty() ? "" : b.hal.lines.front().c_str());
  CHECK(maxErr < 1.5f, "erreur de suivi max %.3f°", maxErr);
  for (int a = 0; a < NA; a++) CHECK(b.maxVel[a] <= vmaxSteps(a) * 1.06f, "J%d vitesse %.2f pas/ms", a + 1, b.maxVel[a]);
  CHECK(b.hal.timingErrors == 0, "%d violations de chronogramme", b.hal.timingErrors);
  // Fin du flux : chien de garde → arrêt contrôlé
  CHECK(b.runUntil([&] { return b.got("EVT WATCHDOG"); }, 1000), "chien de garde non déclenché");
  CHECK(b.runUntil([&] { return b.got("EVT STOPPED"); }, 2000), "arrêt après chien de garde absent");
  CHECK(b.robot.flags() & F_WATCHDOG, "flag chien de garde");
  printf("    erreur de suivi max : %.3f°\n", maxErr);
}

static void testStreamJump() {
  printf("• saut de consigne (limiteur sans dépassement)\n");
  Bench b;
  b.enableAndZero();
  b.hal.lines.clear();
  b.resetVel();
  int seq = 0;
  float minQ = 1e9f, maxQ = -1e9f;
  b.onTick = [&](int now) {
    if (now % 10 == 0 && now < 4000) {
      const float q1 = now < 300 ? 0.f : 60.f;  // saut de 60° sur J1
      char buf[160];
      snprintf(buf, sizeof(buf), "SP %d %d %.3f 0 0 0 90 0", seq++, now, q1);
      b.send(buf);
    }
    const float q = b.hal.trueDeg(0);
    if (q < minQ) minQ = q;
    if (q > maxQ) maxQ = q;
  };
  b.run(3900);
  CHECK(fabsf(b.hal.trueDeg(0) - 60.f) < 0.05f, "J1 final %.3f°", b.hal.trueDeg(0));
  CHECK(maxQ < 60.1f, "dépassement : max %.3f°", maxQ);
  CHECK(minQ > -0.1f, "recul : min %.3f°", minQ);
  CHECK(b.maxVel[0] <= vmaxSteps(0) * 1.06f, "vitesse J1 %.2f > %.2f pas/ms", b.maxVel[0], vmaxSteps(0));
  CHECK(b.robot.flags() & F_LIMITED, "le flag « limité » devrait être levé");
  CHECK(b.hal.timingErrors == 0, "%d violations de chronogramme", b.hal.timingErrors);
  // Première consigne trop éloignée : refus
  Bench c;
  c.enableAndZero();
  c.send("SP 0 0 20 0 0 0 90 0");
  CHECK(c.got("ERR 5 flux"), "démarrage de flux éloigné accepté : %s", c.last().c_str());
}

static void testSafety() {
  printf("• sécurité (arrêt d’urgence, fin de course, défauts)\n");
  Bench b;
  b.enableAndZero();
  b.send("MJ 0 90 0 0 0 90 0");
  b.run(300);
  b.hal.estop = true;
  b.run(2);
  CHECK(b.got("EVT ESTOP"), "arrêt d’urgence non signalé");
  CHECK(!b.hal.drivers && !b.robot.enabled() && !b.robot.homed(), "drivers toujours actifs / référencement conservé");
  CHECK(b.robot.state() == State::ESTOP, "état %s", stateName(b.robot.state()));
  b.send("EN 1");
  CHECK(b.got("ERR 7"), "EN accepté pendant l’arrêt d’urgence");
  b.hal.estop = false;
  b.send("EN 1");
  CHECK(b.got("ERR 4"), "EN accepté sans acquittement");
  b.send("CLR");
  CHECK(b.got("OK CLR") && b.robot.state() == State::IDLE, "acquittement → %s", stateName(b.robot.state()));
  b.send("EN 1");
  CHECK(b.got("OK EN 1") && b.robot.state() == State::READY, "réactivation");
  // Fin de course pendant un mouvement → défaut
  Bench c;
  c.enableAndZero();
  c.hal.switchDeg[2] = 30.f;  // capteur J3 mal placé (dans la course)
  c.send("MJ 0 0 0 45 0 90 0");
  CHECK(c.runUntil([&] { return c.got("EVT FAULT fin de course J3"); }, 3000), "fin de course non détecté");
  CHECK(c.robot.state() == State::FAULT, "état %s", stateName(c.robot.state()));
  c.send("MJ 0 0 0 0 0 90 0");
  CHECK(c.got("ERR 4"), "mouvement accepté en défaut");
  c.send("ESTOP");
  CHECK(c.got("OK ESTOP") && !c.hal.drivers, "ESTOP logiciel");
  // JG sans prise d’origine
  Bench d;
  d.send("EN 1");
  d.send("JG 6 10");
  CHECK(d.got("OK MJ"), "JG : %s", d.last().c_str());
  CHECK(d.runUntil([&] { return d.got("EVT DONE"); }, 3000), "JG non terminé");
  CHECK(fabsf(d.hal.trueDeg(5) - 10.f) < 0.01f, "JG J6 %.3f°", d.hal.trueDeg(5));
  d.send("JG 6 45");
  CHECK(d.got("ERR 3"), "JG > 30° accepté");
}

static void testConfig() {
  printf("• paramètres (SET/GET/SAVE/LOAD)\n");
  Bench b;
  b.send("SET vmax 2 45");
  CHECK(b.got("OK SET vmax 2 45"), "SET : %s", b.last().c_str());
  b.send("GET vmax 2");
  CHECK(b.got("OK 45"), "GET : %s", b.last().c_str());
  b.send("SET amax * 250");
  b.send("GET amax 6");
  CHECK(b.got("OK 250"), "SET * : %s", b.last().c_str());
  b.send("SET vmax 1 -3");
  CHECK(b.got("ERR 3"), "vitesse négative acceptée");
  b.send("SET min 1 200");
  CHECK(b.got("ERR 3 min >= max"), "min > max accepté : %s", b.last().c_str());
  b.send("SET inconnu 1 3");
  CHECK(b.got("ERR 3 cle inconnue"), "clé inconnue");
  b.send("SET watchdog * 300");
  CHECK(b.got("OK SET watchdog 300"), "watchdog : %s", b.last().c_str());
  b.send("SET home_dir 3 -1");
  b.send("GET home_dir 3");
  CHECK(b.got("OK -1"), "home_dir : %s", b.last().c_str());
  // Changement d’échelle : la position angulaire est conservée
  b.enableAndZero();
  b.send("MJ 0 10 0 0 0 90 0");
  b.runUntil([&] { return b.got("EVT DONE"); }, 3000);
  b.send("SET steps_per_deg 1 111.11111");
  CHECK(fabsf(b.robot.posDeg(0) - 10.f) < 0.01f, "position après changement d’échelle %.3f", b.robot.posDeg(0));
  b.send("SAVE");
  CHECK(b.got("OK SAVE"), "SAVE");
  b.send("DEFAULTS");
  b.send("GET vmax 2");
  CHECK(b.got("OK 90"), "DEFAULTS : %s", b.last().c_str());
  b.send("LOAD");
  CHECK(b.got("OK LOAD"), "LOAD : %s", b.last().c_str());
  b.send("GET vmax 2");
  CHECK(b.got("OK 45"), "valeur rechargée : %s", b.last().c_str());
  // EEPROM corrompue → ignorée au démarrage
  b.hal.eeprom[12] ^= 0xFF;
  b.send("LOAD");
  CHECK(b.got("ERR 8"), "EEPROM corrompue acceptée");
  Bench c;
  c.hal.eeprom = b.hal.eeprom;
  c.robot.begin();
  c.send("GET vmax 2");
  CHECK(c.got("OK 90"), "configuration corrompue chargée au démarrage : %s", c.last().c_str());
}

int main() {
  printf("Tests du cœur firmware ORION-6 (%d axes)\n", NA);
  testProtocol();
  testMoveJ();
  testStop();
  testHoming();
  testStreaming();
  testStreamJump();
  testSafety();
  testConfig();
  printf("%d vérifications, %d échec(s)\n", g_checks, g_fail);
  return g_fail ? 1 : 0;
}
