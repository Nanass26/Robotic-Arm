// ORION-6 firmware — cœur portable : protocole ORION-ASCII, génération de mouvement,
// prise d’origine, sécurité. Aucune dépendance Arduino.
//
// Deux cadences :
//   • tick1ms()  : boucle 1 kHz (consignes, prise d’origine, sécurité, calcul des vitesses de pas)
//   • stepIsr()  : interruption 100 kHz (accumulateur DDA → impulsions STEP régulières)
#pragma once
#include <stdint.h>
#include "hal.h"
#include "../orion_config.h"

namespace orion {

constexpr int NA = ORION_NUM_AXES;
constexpr uint32_t ISR_HZ = 100000;
constexpr uint32_t ISR_PER_MS = ISR_HZ / 1000;
constexpr const char* FW_VERSION = "1.0.0";

enum class State : uint8_t { IDLE, READY, HOMING, RUN, HOLD, FAULT, ESTOP };
const char* stateName(State s);

// Bits du champ « flags » de la ligne ST (identiques à studio/core/protocol.js)
enum Flag : uint32_t {
  F_ENABLED = 0x01, F_HOMED = 0x02, F_MOVING = 0x04, F_FAULT = 0x08, F_ESTOP_IN = 0x10,
  F_WATCHDOG = 0x20, F_LIMIT = 0x40, F_UNDERRUN = 0x80, F_LIMITED = 0x100,
};

// Configuration modifiable (SET/GET) et sauvegardée en EEPROM
struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  float stepsPerDeg[NA];
  float minDeg[NA], maxDeg[NA];
  float vmax[NA], amax[NA];
  uint8_t invert[NA];
  int8_t homeDir[NA];
  float homePos[NA], homeSpeed[NA], homeSlow[NA], homeBackoff[NA];
  float homePose[NA];
  uint8_t homeOrder[NA];
  uint8_t homeEnabled[NA];
  uint16_t watchdogMs;
  uint16_t streamDelayMs;  // retard du tampon de lissage des consignes SP
  uint16_t gripClosedUs, gripOpenUs;
  uint32_t crc;
};

void defaultConfig(Config& c);
uint32_t crc32(const uint8_t* d, uint32_t n);
bool configValid(const Config& c);

class Robot {
 public:
  explicit Robot(Hal& hal);
  void begin();
  void tick1ms();            // à appeler toutes les 1 ms
  void stepIsr();            // à appeler à ISR_HZ
  void onChar(char c);       // octets reçus sur l’USB
  void handleLine(char* line);

  // Accès (tests / diagnostic)
  State state() const { return state_; }
  bool homed() const { return homed_; }
  bool enabled() const { return enabled_; }
  int32_t stepPos(int a) const { return pos_[a]; }
  float posDeg(int a) const { return (float)pos_[a] / cfg_.stepsPerDeg[a]; }
  float desDeg(int a) const { return pDes_[a] / cfg_.stepsPerDeg[a]; }
  float velDeg(int a) const { return vDes_[a] * 1000.f / cfg_.stepsPerDeg[a]; }  // °/s
  Config& config() { return cfg_; }
  uint32_t millis() const { return now_; }
  uint32_t flags() const;

 private:
  enum class Motion : uint8_t { NONE, MOVEJ, STREAM, STOPPING, HOMING };
  enum class HStep : uint8_t { START, CLEAR, FAST, BACK, SLOW, RELEASE };

  struct SpPoint {
    float t;       // horodatage émetteur (ms, déroulé)
    float q[NA];   // pas
    float v[NA];   // pas/ms
    bool hasVel;
  };

  void reply(const char* fmt, ...);
  void sendStatus();
  void setState(State s);
  void fault(const char* why);
  void enable(bool on);
  void setPosition(int a, int32_t steps);
  bool parseFloats(char** tok, int n, float* out);
  void cmdMoveJ(float T, const float* qDeg, bool checkLimits);
  void cmdStream(uint32_t seq16, uint32_t t16, const float* qDeg, const float* vDeg);
  void cmdStop();
  void cmdHome(uint32_t mask);
  void cmdSet(const char* key, const char* axis, float v);
  void cmdGet(const char* key, const char* axis);
  float* cfgField(const char* key, uint8_t*& u8, int8_t*& i8);
  void motionTick();
  bool streamTarget(float* target);
  void homingTick();
  void computeRates();
  void clampToLimits(float* p) const;
  void beginMotion(Motion m);
  float degToSteps(int a, float deg) const { return deg * cfg_.stepsPerDeg[a]; }

  Hal& hal_;
  Config cfg_;
  State state_ = State::IDLE;
  Motion motion_ = Motion::NONE;
  bool homed_ = false;
  bool enabled_ = false;
  uint32_t now_ = 0;
  char faultMsg_[48] = {0};

  // Position réelle (pas émis) et consigne (pas, flottant)
  volatile int32_t pos_[NA] = {0};
  float pDes_[NA] = {0};
  float vDes_[NA] = {0};         // pas/ms
  float lastTarget_[NA] = {0};   // cible du tick précédent (anticipation de vitesse)
  volatile float rate_[NA] = {0};  // pas par tick ISR (signé)
  volatile float phase_[NA] = {0};
  volatile int8_t dir_[NA] = {0};     // sens appliqué (broche DIR)
  volatile int8_t dirReq_[NA] = {0};  // sens demandé par la boucle 1 kHz
  volatile uint8_t dirHold_[NA] = {0};
  volatile uint8_t pulseHigh_[NA] = {0};

  // MoveJ synchronisé (trapèze 25/50/25)
  float mjStart_[NA] = {0}, mjDelta_[NA] = {0};
  uint32_t mjT0_ = 0, mjDur_ = 0;

  // Flux de consignes SP : tampon de lissage horodaté
  static constexpr int SPN = 32;
  SpPoint sp_[SPN];
  uint32_t spW_ = 0, spR_ = 0;          // compteurs écriture / segment courant
  uint32_t spLastSeq_ = 0;
  uint16_t spLastT16_ = 0;              // dernier horodatage reçu (16 bits, ms)
  float spClock_ = 0;                   // horloge de lecture (temps émetteur, ms)
  float spLagF_ = 0;                    // avance du tampon, filtrée (ms)
  uint32_t spLastArrival_ = 0;
  bool spStarted_ = false;
  bool underrun_ = false, limited_ = false, watchdogTrip_ = false;

  // Prise d’origine
  uint32_t homeMask_ = 0;
  int homeIdx_ = 0;
  HStep hstep_ = HStep::START;
  float htravel_ = 0;

  // Divers
  uint16_t statusHz_ = 0;
  uint32_t nextStatus_ = 0;
  char line_[200];
  uint8_t lineLen_ = 0;
  bool lineOverflow_ = false;
  bool out_[8] = {false};
};

}  // namespace orion
