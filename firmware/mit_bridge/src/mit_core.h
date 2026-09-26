// ORION-6 PRO — pont USB ↔ CAN « mode MIT » (cœur portable, sans dépendance Arduino).
//
// L’hôte (ORION Studio, script Python…) envoie des consignes articulaires horodatées
// « MC » (position, vitesse, couple d’anticipation = compensation de gravité). Le pont les
// lisse (tampon + interpolation d’Hermite), les convertit dans le repère de chaque moteur et
// envoie à ctrlHz une trame MIT 8 octets par moteur : τ = Kp·(p* − p) + Kd·(v* − v) + τff.
//
// Sécurités : butées logicielles, couple et vitesse bornés, chien de garde (maintien),
// surveillance des retours CAN (silence, erreurs, température), amortissement en défaut.
#pragma once
#include <stdint.h>

#include "orion_config.h"

namespace mit {

constexpr int NA = ORION_NUM_AXES;
constexpr const char* FW_VERSION = "1.0.0";

// ------------------------------------------------------------------ trames MIT
struct Limits { float pMax, vMax, tMax, kpMax, kdMax; };
struct Command { float p, v, kp, kd, t; };
struct Feedback { uint8_t id, err; float p, v, t; uint8_t tMos, tRotor; };

uint32_t floatToUint(float x, float lo, float hi, int bits);
float uintToFloat(uint32_t u, float lo, float hi, int bits);
void packCommand(const Command& c, const Limits& l, uint8_t out[8]);
void unpackFeedback(const uint8_t d[8], const Limits& l, Feedback& fb);

enum Special : uint8_t { SP_CLEAR = 0xFB, SP_ENABLE = 0xFC, SP_DISABLE = 0xFD, SP_ZERO = 0xFE };
void specialFrame(uint8_t code, uint8_t out[8]);
const char* motorErrorText(uint8_t err);

// ------------------------------------------------------------------ matériel
struct BridgeHal {
  virtual ~BridgeHal() = default;
  virtual bool canSend(uint32_t id, const uint8_t d[8]) = 0;
  virtual bool canReceive(uint32_t& id, uint8_t d[8]) = 0;  // false si file vide
  virtual void write(const char* line) = 0;                 // ligne complète terminée par \n
  virtual bool estopActive() = 0;
  virtual bool saveBlob(const void* data, uint32_t len) = 0;
  virtual bool loadBlob(void* data, uint32_t len) = 0;
};

enum class State : uint8_t { IDLE, READY, RUN, HOLD, DAMP, FAULT, ESTOP };
const char* stateName(State s);

enum Flag : uint32_t {
  F_ENABLED = 0x01, F_HOMED = 0x02, F_MOVING = 0x04, F_FAULT = 0x08, F_ESTOP_IN = 0x10,
  F_WATCHDOG = 0x20, F_LIMIT = 0x40, F_UNDERRUN = 0x80, F_LIMITED = 0x100,
};

struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint8_t canId[NA];
  int8_t dir[NA];          // q = dir·p_moteur + offset
  float offsetDeg[NA];
  float minDeg[NA], maxDeg[NA];
  float vmax[NA];          // °/s (consigne)
  float tauMax[NA];        // N·m (couple d’anticipation max)
  float kp[NA], kd[NA];    // gains MIT : N·m/rad, N·m·s/rad
  float kdDamp[NA];        // amortissement en défaut / arrêt
  float refPose[NA];       // pose de référence pour ZERO (°)
  Limits lim[NA];          // plages d’encodage des trames (identiques aux réglages du moteur)
  uint16_t watchdogMs, streamDelayMs, ctrlHz, fbTimeoutMs;
  uint8_t maxTempC;
  uint8_t pad[3];
  uint32_t crc;
};

void defaultConfig(Config& c);
bool configValid(const Config& c);
uint32_t crc32(const uint8_t* d, uint32_t n);

class Bridge {
 public:
  explicit Bridge(BridgeHal& hal);
  void begin();
  void tick1ms();
  void onChar(char c);
  void handleLine(char* line);

  State state() const { return state_; }
  float qDeg(int a) const { return q_[a]; }
  float cmdDeg(int a) const { return lastCmdDeg_[a]; }
  const Config& config() const { return cfg_; }
  uint32_t millis() const { return now_; }
  uint32_t flags() const;

 private:
  struct Pt {
    float t;
    float q[NA], v[NA], tau[NA];  // °, °/ms, N·m
    bool hasVel;
  };

  void reply(const char* fmt, ...);
  void sendStatus();
  void fault(const char* why);
  void sendSpecial(uint8_t code);
  void sendMit(int a, float qDeg, float vDegS, float tau, float kp, float kd);
  void controlStep();
  void readFeedback();
  bool streamTarget(float* q, float* v, float* tau);
  void cmdStream(uint32_t seq16, uint32_t t16, const float* q, const float* v, const float* tau);
  void holdHere();
  void updateJointFromMotor(int a);
  void cmdSet(const char* key, const char* axis, float v);
  void cmdGet(const char* key, const char* axis);
  float* field(const char* key, bool& isInt);
  bool parseFloats(char** tok, int n, float* out);

  BridgeHal& hal_;
  Config cfg_;
  State state_ = State::IDLE;
  uint32_t now_ = 0, lastCtrl_ = 0, nextPoll_ = 0, disableAt_ = 0;
  float ctrlAcc_ = 0, ctrlDt_ = 1;
  bool watchdogTrip_ = false, underrun_ = false, limited_ = false;
  char faultMsg_[48] = {0};

  // Mesures : position brute du moteur (rad) et grandeurs articulaires (°, °/s, N·m)
  float pm_[NA] = {0};
  float q_[NA] = {0}, v_[NA] = {0}, tau_[NA] = {0};
  uint8_t tMos_[NA] = {0}, tRotor_[NA] = {0}, err_[NA] = {0};
  uint32_t lastFb_[NA] = {0};
  bool seen_[NA] = {false};

  // Consigne de maintien
  float holdQ_[NA] = {0}, holdTau_[NA] = {0}, lastTau_[NA] = {0};
  float lastCmdDeg_[NA] = {0};

  // Flux MC
  static constexpr int SPN = 32;
  Pt sp_[SPN];
  uint32_t spW_ = 0, spR_ = 0, spLastArrival_ = 0;
  uint16_t spLastSeq_ = 0, spLastT16_ = 0;
  float spClock_ = 0, spLagF_ = 0;
  bool spStarted_ = false;
  float blend_ = 1;  // démarrage progressif du flux (0 → 1 en 300 ms)

  uint16_t statusHz_ = 0;
  uint32_t nextStatus_ = 0;
  char line_[320];
  uint16_t lineLen_ = 0;
  bool lineOverflow_ = false;
};

}  // namespace mit
