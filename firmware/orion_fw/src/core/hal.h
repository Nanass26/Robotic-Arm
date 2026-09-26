// ORION-6 firmware — interface matérielle abstraite.
// Le cœur (robot.cpp) n’utilise que cette interface : il se compile sur Teensy 4.1
// (main.cpp) et sur PC pour les tests unitaires (firmware/test).
#pragma once
#include <stdint.h>

namespace orion {

struct Hal {
  virtual ~Hal() = default;
  // Pas-à-pas
  virtual void stepHigh(uint8_t axis) = 0;
  virtual void stepLow(uint8_t axis) = 0;
  virtual void setDir(uint8_t axis, bool positive) = 0;  // inversion gérée par l’appelant
  virtual void enableDrivers(bool on) = 0;
  // Capteurs et sécurité
  virtual bool limitActive(uint8_t axis) = 0;  // polarité déjà appliquée
  virtual bool estopActive() = 0;
  // Périphériques
  virtual void setGripperUs(uint16_t us) = 0;
  virtual void setOutput(uint8_t n, bool v) = 0;
  virtual bool readInput(uint8_t n) = 0;
  // Communication
  virtual void write(const char* line) = 0;  // ligne complète terminée par \n
  // Persistance (EEPROM) : retourne false si indisponible
  virtual bool saveBlob(const void* data, uint32_t len) = 0;
  virtual bool loadBlob(void* data, uint32_t len) = 0;
  // Section critique vis-à-vis de l’interruption de pas (écriture de la position)
  virtual void irqLock(bool lock) { (void)lock; }
};

}  // namespace orion
