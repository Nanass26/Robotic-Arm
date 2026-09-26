// ORION-6 firmware — cible Teensy 4.1 (Arduino/Teensyduino ou PlatformIO).
//
//  • Interruption 100 kHz (IntervalTimer) : génération des impulsions STEP (Robot::stepIsr)
//  • Boucle principale : lecture USB, anti-rebond des capteurs, tick 1 kHz (Robot::tick1ms)
//
// Le brochage et les paramètres par défaut viennent de orion_config.h, généré par
// ORION Studio (Exporter → Configuration firmware) ou par `npm run build:firmware-config`.
#include <Arduino.h>
#include <EEPROM.h>

#include "core/robot.h"

using namespace orion;

namespace {

// Entrées/sorties TOR libres (non utilisées par les axes)
constexpr uint8_t DI_PINS[8] = {16, 17, 18, 19, 20, 21, 22, 23};
constexpr uint8_t DO_PINS[8] = {31, 32, 34, 35, 36, 37, 38, 39};
constexpr uint8_t LED_PIN = 13;

class TeensyHal : public Hal {
 public:
  void begin() {
    for (int a = 0; a < NA; a++) {
      pinMode(orion_cfg::STEP_PIN[a], OUTPUT);
      pinMode(orion_cfg::DIR_PIN[a], OUTPUT);
      digitalWriteFast(orion_cfg::STEP_PIN[a], LOW);
      pinMode(orion_cfg::LIMIT_PIN[a], orion_cfg::LIMIT_ACTIVE_LOW[a] ? INPUT_PULLUP : INPUT_PULLDOWN);
    }
    pinMode(orion_cfg::ENABLE_PIN, OUTPUT);
    enableDrivers(false);
    // Arrêt d’urgence : contact NF vers GND → niveau haut = arrêt (fil coupé compris)
    pinMode(orion_cfg::ESTOP_PIN, INPUT_PULLUP);
    for (uint8_t p : DI_PINS) pinMode(p, INPUT_PULLUP);
    for (uint8_t p : DO_PINS) { pinMode(p, OUTPUT); digitalWriteFast(p, LOW); }
    pinMode(LED_PIN, OUTPUT);
    // Servo de pince : MLI matérielle 50 Hz, résolution 16 bits (≈ 0,3 µs)
    analogWriteResolution(16);
    analogWriteFrequency(orion_cfg::GRIPPER_PIN, 50);
  }

  // Anti-rebond : un capteur doit être stable 2 ms pour changer d’état
  void poll() {
    for (int a = 0; a < NA; a++) {
      const bool raw = digitalReadFast(orion_cfg::LIMIT_PIN[a]) == (orion_cfg::LIMIT_ACTIVE_LOW[a] ? LOW : HIGH);
      filt(limitCnt_[a], limit_[a], raw);
    }
    filt(estopCnt_, estop_, digitalReadFast(orion_cfg::ESTOP_PIN) == HIGH);
  }

  void stepHigh(uint8_t a) override { digitalWriteFast(orion_cfg::STEP_PIN[a], HIGH); }
  void stepLow(uint8_t a) override { digitalWriteFast(orion_cfg::STEP_PIN[a], LOW); }
  void setDir(uint8_t a, bool positive) override { digitalWriteFast(orion_cfg::DIR_PIN[a], positive ? HIGH : LOW); }
  void enableDrivers(bool on) override {
    digitalWriteFast(orion_cfg::ENABLE_PIN, (on != orion_cfg::ENABLE_ACTIVE_LOW) ? HIGH : LOW);
  }
  bool limitActive(uint8_t a) override { return limit_[a]; }
  bool estopActive() override { return estop_; }
  void setGripperUs(uint16_t us) override {
    analogWrite(orion_cfg::GRIPPER_PIN, (uint32_t)us * 65535UL / 20000UL);
  }
  void setOutput(uint8_t n, bool v) override {
    if (n < 8) digitalWriteFast(DO_PINS[n], v ? HIGH : LOW);
  }
  bool readInput(uint8_t n) override { return n < 8 && digitalReadFast(DI_PINS[n]) == LOW; }
  void write(const char* line) override {
    const size_t n = strlen(line);
    // Jamais bloquant : si l’hôte ne lit pas, la ligne est abandonnée
    if (Serial.dtr() && Serial.availableForWrite() >= (int)n) Serial.write(line, n);
  }
  bool saveBlob(const void* data, uint32_t len) override {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    for (uint32_t i = 0; i < len; i++) EEPROM.update(i, p[i]);
    return true;
  }
  bool loadBlob(void* data, uint32_t len) override {
    uint8_t* p = static_cast<uint8_t*>(data);
    for (uint32_t i = 0; i < len; i++) p[i] = EEPROM.read(i);
    return true;
  }
  void irqLock(bool lock) override {
    // noInterrupts()/interrupts() sont des macros : accolades obligatoires
    if (lock) {
      noInterrupts();
    } else {
      interrupts();
    }
  }

 private:
  static void filt(uint8_t& cnt, bool& state, bool raw) {
    if (raw == state) { cnt = 0; return; }
    if (++cnt >= 2) { state = raw; cnt = 0; }
  }
  bool limit_[NA] = {false};
  uint8_t limitCnt_[NA] = {0};
  bool estop_ = true;  // prudence au démarrage : levé tant que l’entrée n’est pas lue
  uint8_t estopCnt_ = 0;
};

TeensyHal hal;
Robot robot(hal);
IntervalTimer stepTimer;
uint32_t lastTickUs = 0;

void stepIsr() { robot.stepIsr(); }

void updateLed() {
  // Rythme de la LED : lente = prêt, rapide = mouvement, fixe = défaut / arrêt d’urgence
  const uint32_t t = robot.millis();
  bool on;
  switch (robot.state()) {
    case State::FAULT:
    case State::ESTOP: on = true; break;
    case State::RUN:
    case State::HOMING: on = (t / 100) & 1; break;
    case State::IDLE: on = (t % 2000) < 50; break;
    default: on = (t / 500) & 1; break;
  }
  digitalWriteFast(LED_PIN, on ? HIGH : LOW);
}

}  // namespace

void setup() {
  Serial.begin(115200);  // USB natif : la vitesse est ignorée
  hal.begin();
  for (int i = 0; i < 3; i++) hal.poll();
  robot.begin();
  stepTimer.priority(32);  // priorité haute (0 = maximale), au-dessus de l’USB
  stepTimer.begin(stepIsr, 1000000.0f / ISR_HZ);
  lastTickUs = micros();
}

void loop() {
  while (Serial.available() > 0) robot.onChar((char)Serial.read());
  // Tick 1 kHz cadencé sur micros() (rattrapage si la boucle a pris du retard)
  uint32_t guard = 0;
  while ((uint32_t)(micros() - lastTickUs) >= 1000 && guard++ < 10) {
    lastTickUs += 1000;
    hal.poll();
    robot.tick1ms();
  }
  if (guard >= 10) lastTickUs = micros();
  updateLed();
}
