// FICHIER GÉNÉRÉ par ORION Studio (Exporter → Configuration firmware).
// Robot : ORION-6
// Toutes les valeurs angulaires sont en DEGRÉS côté articulation (sortie de réducteur).
#pragma once
#include <stdint.h>

#define ORION_NUM_AXES 6
#define ORION_ROBOT_NAME "ORION-6"

namespace orion_cfg {
// Pas moteur par degré articulaire = (360/angle_pas) × micro-pas × réduction / 360
constexpr float STEPS_PER_DEG[ORION_NUM_AXES] = { 222.22222, 266.66667, 222.22222, 177.77778, 177.77778, 177.77778 };
constexpr float MIN_DEG[ORION_NUM_AXES]       = { -170.000, -130.000, -170.000, -170.000, -115.000, -180.000 };
constexpr float MAX_DEG[ORION_NUM_AXES]       = { 170.000, 130.000, 60.000, 170.000, 115.000, 180.000 };
constexpr float VMAX_DEG_S[ORION_NUM_AXES]    = { 120.000, 90.000, 110.000, 170.000, 170.000, 200.000 };
constexpr float AMAX_DEG_S2[ORION_NUM_AXES]   = { 300.000, 200.000, 300.000, 600.000, 600.000, 700.000 };
constexpr uint8_t STEP_PIN[ORION_NUM_AXES]    = { 2, 4, 6, 8, 10, 12 };
constexpr uint8_t DIR_PIN[ORION_NUM_AXES]     = { 3, 5, 7, 9, 11, 14 };
constexpr uint8_t LIMIT_PIN[ORION_NUM_AXES]   = { 24, 25, 26, 27, 28, 29 };
constexpr bool INVERT_DIR[ORION_NUM_AXES]     = { false, false, false, false, false, false };
constexpr bool LIMIT_ACTIVE_LOW[ORION_NUM_AXES] = { true, true, true, true, true, true };
constexpr bool HOMING_ENABLED[ORION_NUM_AXES] = { true, true, true, true, true, true };
constexpr int8_t HOMING_DIR[ORION_NUM_AXES]   = { -1, -1, 1, -1, -1, -1 };
constexpr float HOMING_SPEED[ORION_NUM_AXES]  = { 20.000, 15.000, 15.000, 30.000, 30.000, 30.000 };
constexpr float HOMING_SLOW[ORION_NUM_AXES]   = { 2.000, 2.000, 2.000, 3.000, 3.000, 3.000 };
constexpr float HOMING_BACKOFF[ORION_NUM_AXES] = { 4.000, 4.000, 4.000, 5.000, 5.000, 5.000 };
constexpr float HOMING_SWITCH_POS[ORION_NUM_AXES] = { -172.000, -133.000, 62.000, -172.000, -117.000, -182.000 };
constexpr uint8_t HOMING_ORDER[ORION_NUM_AXES] = { 4, 5, 3, 2, 1, 0 };
constexpr float HOME_POSE_DEG[ORION_NUM_AXES] = { 0.000, 0.000, 0.000, 0.000, 90.000, 0.000 };
constexpr uint8_t ENABLE_PIN = 15;
constexpr bool ENABLE_ACTIVE_LOW = true;
constexpr uint8_t ESTOP_PIN = 30;
constexpr uint8_t GRIPPER_PIN = 33;
constexpr uint16_t GRIPPER_US_CLOSED = 900;
constexpr uint16_t GRIPPER_US_OPEN = 2100;
constexpr float STEP_PULSE_US = 3.00f;
constexpr float DIR_SETUP_US = 6.00f;
constexpr uint32_t WATCHDOG_MS = 250;
constexpr uint32_t CAN_BITRATE = 1000000;
// Mode MIT (pont CAN) : identifiants et plages des moteurs
constexpr uint8_t CAN_ID[ORION_NUM_AXES] = { 1, 2, 3, 4, 5, 6 };
constexpr float MIT_PMAX[ORION_NUM_AXES] = { 12.500, 12.500, 12.500, 12.500, 12.500, 12.500 };
constexpr float MIT_VMAX[ORION_NUM_AXES] = { 8.000, 8.000, 8.000, 8.000, 8.000, 8.000 };
constexpr float MIT_TMAX[ORION_NUM_AXES] = { 28.000, 28.000, 28.000, 28.000, 28.000, 28.000 };
constexpr float MIT_KPMAX[ORION_NUM_AXES] = { 500.000, 500.000, 500.000, 500.000, 500.000, 500.000 };
constexpr float MIT_KDMAX[ORION_NUM_AXES] = { 5.000, 5.000, 5.000, 5.000, 5.000, 5.000 };
constexpr float MIT_KP[ORION_NUM_AXES] = { 90.000, 130.000, 90.000, 30.000, 30.000, 15.000 };
constexpr float MIT_KD[ORION_NUM_AXES] = { 3.0000, 4.0000, 2.5000, 1.0000, 1.0000, 0.5000 };
constexpr float MIT_KD_DAMP[ORION_NUM_AXES] = { 5.0000, 5.0000, 5.0000, 2.0000, 2.0000, 1.0000 };
constexpr int8_t MIT_DIR[ORION_NUM_AXES] = { 1, 1, 1, 1, 1, 1 };
constexpr float TAU_MAX[ORION_NUM_AXES] = { 10.320, 39.690, 10.320, 5.880, 5.880, 3.920 };  // N·m
constexpr float REF_POSE_DEG[ORION_NUM_AXES] = { 0.000, -10.000, 55.000, 0.000, 35.000, 0.000 };  // pose de référence (ZERO)
constexpr uint16_t MIT_CTRL_HZ = 400;
constexpr uint16_t MIT_FB_TIMEOUT_MS = 50;
constexpr uint8_t MIT_MAX_TEMP_C = 85;
constexpr uint16_t STREAM_DELAY_MS = 40;
}  // namespace orion_cfg
