// ORION-6 PRO — pont USB ↔ CAN « mode MIT », cible Teensy 4.1.
//
// Câblage : CAN1 (broche 22 = CTX1, 23 = CRX1) → transceiver 3,3 V (SN65HVD230, TJA1051T/3…)
// → bus CAN_H/CAN_L des moteurs (DM4340/DM4310), terminaison 120 Ω aux deux extrémités.
// Arrêt d’urgence : contact NF vers GND sur ESTOP_PIN (la coupure de puissance reste matérielle).
#include <Arduino.h>
#include <EEPROM.h>
#include <FlexCAN_T4.h>

#include "mit_core.h"

namespace {

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_64> can1;
constexpr uint8_t LED_PIN = 13;

class TeensyBridgeHal : public mit::BridgeHal {
 public:
  void begin() {
    pinMode(orion_cfg::ESTOP_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    can1.begin();
    can1.setBaudRate(orion_cfg::CAN_BITRATE);
    can1.setMaxMB(16);
    can1.enableFIFO();
  }
  void poll() {
    const bool raw = digitalReadFast(orion_cfg::ESTOP_PIN) == HIGH;
    if (raw == estop_) cnt_ = 0;
    else if (++cnt_ >= 2) { estop_ = raw; cnt_ = 0; }
  }
  bool canSend(uint32_t id, const uint8_t d[8]) override {
    CAN_message_t m;
    m.id = id;
    m.len = 8;
    m.flags.extended = 0;
    memcpy(m.buf, d, 8);
    return can1.write(m) != 0;
  }
  bool canReceive(uint32_t& id, uint8_t d[8]) override {
    CAN_message_t m;
    if (!can1.read(m)) return false;
    id = m.id;
    memcpy(d, m.buf, 8);
    return true;
  }
  void write(const char* line) override {
    const size_t n = strlen(line);
    if (Serial.dtr() && Serial.availableForWrite() >= (int)n) Serial.write(line, n);
  }
  bool estopActive() override { return estop_; }
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

 private:
  bool estop_ = true;
  uint8_t cnt_ = 0;
};

TeensyBridgeHal hal;
mit::Bridge bridge(hal);
uint32_t lastTickUs = 0;

}  // namespace

void setup() {
  Serial.begin(115200);
  hal.begin();
  for (int i = 0; i < 3; i++) hal.poll();
  bridge.begin();
  lastTickUs = micros();
}

void loop() {
  can1.events();  // vide la file d’émission CAN
  while (Serial.available() > 0) bridge.onChar((char)Serial.read());
  uint32_t guard = 0;
  while ((uint32_t)(micros() - lastTickUs) >= 1000 && guard++ < 10) {
    lastTickUs += 1000;
    hal.poll();
    bridge.tick1ms();
  }
  if (guard >= 10) lastTickUs = micros();
  const mit::State s = bridge.state();
  const uint32_t t = bridge.millis();
  const bool on = (s == mit::State::FAULT || s == mit::State::ESTOP) ? true
                  : (s == mit::State::RUN) ? ((t / 100) & 1)
                  : (s == mit::State::IDLE) ? ((t % 2000) < 50)
                  : ((t / 500) & 1);
  digitalWriteFast(LED_PIN, on ? HIGH : LOW);
}
