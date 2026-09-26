// ORION-6 — Configurations par défaut et préréglages.
// Toutes les valeurs sont en SI (m, rad, kg, s, N·m, A, V).

import { MASS_PROPERTIES, COLLISION_CAPSULES } from './generated/massprops.js';

const DEG = Math.PI / 180;
const J6 = (f) => Array.from({ length: 6 }, (_, i) => f(i));
const deepClone = (o) => JSON.parse(JSON.stringify(o));

export const SCHEMA_VERSION = 1;

// ------------------------------------------------------------------ moteurs types
// Valeurs typiques de fiches techniques (à vérifier avec VOTRE moteur).
export const MOTOR_LIBRARY = {
  'NEMA17-34': { holdingTorque: 0.28, ratedCurrent: 1.3, phaseResistance: 2.4, phaseInductance: 2.8e-3, rotorInertia: 34e-7, mass: 0.23, length: 0.034 },
  'NEMA17-40': { holdingTorque: 0.42, ratedCurrent: 1.7, phaseResistance: 1.5, phaseInductance: 2.8e-3, rotorInertia: 54e-7, mass: 0.28, length: 0.040 },
  'NEMA17-48': { holdingTorque: 0.59, ratedCurrent: 2.0, phaseResistance: 1.4, phaseInductance: 3.0e-3, rotorInertia: 82e-7, mass: 0.39, length: 0.048 },
  'NEMA17-60': { holdingTorque: 0.65, ratedCurrent: 2.1, phaseResistance: 1.6, phaseInductance: 3.0e-3, rotorInertia: 102e-7, mass: 0.50, length: 0.060 },
  'NEMA23-56': { holdingTorque: 1.26, ratedCurrent: 2.8, phaseResistance: 0.9, phaseInductance: 2.5e-3, rotorInertia: 300e-7, mass: 0.70, length: 0.056 },
  'NEMA23-76': { holdingTorque: 1.89, ratedCurrent: 2.8, phaseResistance: 1.13, phaseInductance: 3.6e-3, rotorInertia: 480e-7, mass: 1.00, length: 0.076 },
};

// Actionneurs QDD « mode MIT » (valeurs typiques constructeur, à vérifier).
export const QDD_LIBRARY = {
  // rotorInertia : inertie du rotor côté moteur (estimation) ; ramenée en sortie : N²·J.
  DM4310: { peakTorque: 7, ratedTorque: 3, maxSpeed: 200 * 2 * Math.PI / 60, gearRatio: 10, torqueConstant: 0.945, rotorInertia: 6e-6, mass: 0.30, pMax: 12.5, vMax: 30, tMax: 10 },
  DM4340: { peakTorque: 27, ratedTorque: 9, maxSpeed: 52 * 2 * Math.PI / 60, gearRatio: 40, torqueConstant: 4.4, rotorInertia: 1.5e-5, mass: 0.36, pMax: 12.5, vMax: 8, tMax: 28 },
};

// ------------------------------------------------------------------ ORION-6 MAKER
const JOINT_NAMES = ['J1 Base', 'J2 Épaule', 'J3 Coude', 'J4 Avant-bras', 'J5 Poignet', 'J6 Bride'];

// Réducteurs cycloïdaux imprimés : rapport = nombre de lobes = galets − 1.
const MAKER_MOTORS = ['NEMA17-48', 'NEMA23-76', 'NEMA17-60', 'NEMA17-40', 'NEMA17-40', 'NEMA17-34'];
const MAKER_RATIOS = [25, 30, 25, 20, 20, 20];
const MAKER_MODULES = ['CY-M', 'CY-L', 'CY-M', 'CY-S', 'CY-S', 'CY-S'];

function makerActuator(i) {
  const m = MOTOR_LIBRARY[MAKER_MOTORS[i]];
  return {
    type: 'stepper',
    model: `${MAKER_MOTORS[i]} + cycloïdal ${MAKER_MODULES[i]} ${MAKER_RATIOS[i]}:1`,
    gearRatio: MAKER_RATIOS[i],
    efficiency: 0.7,
    rotorInertia: m.rotorInertia,
    backlash: [0.25, 0.2, 0.25, 0.35, 0.35, 0.4][i] * DEG,
    stiffness: [2500, 4000, 2500, 800, 800, 600][i],
    holdingTorque: m.holdingTorque,
    stepAngle: 1.8 * DEG,
    microsteps: 16,
    ratedCurrent: m.ratedCurrent,
    runCurrent: +(m.ratedCurrent * 0.9).toFixed(2),
    phaseResistance: m.phaseResistance,
    phaseInductance: m.phaseInductance,
    // Champs servo/MIT (non utilisés en pas-à-pas mais conservés pour basculer)
    peakTorque: +(m.holdingTorque * MAKER_RATIOS[i] * 0.7).toFixed(2),
    ratedTorque: +(m.holdingTorque * MAKER_RATIOS[i] * 0.7 * 0.6).toFixed(2),
    maxSpeed: (600 * 2 * Math.PI / 60) / MAKER_RATIOS[i],
    torqueConstant: 0,
    encoderCpr: 4096,
    encoderOnOutput: true,
    canId: i + 1,
    pMax: 12.5, vMax: 8, tMax: 28, kpMax: 500, kdMax: 5,
  };
}

export function makeOrion6Maker() {
  const mp = MASS_PROPERTIES.maker;
  return {
    schemaVersion: SCHEMA_VERSION,
    meta: {
      name: 'ORION-6',
      variant: 'maker',
      description: 'Bras 6 axes imprimé en 3D — NEMA17/23 + réducteurs cycloïdaux imprimés, Teensy 4.1.',
    },
    kinematics: {
      convention: 'DH',
      joints: [
        { name: JOINT_NAMES[0], type: 'revolute', a: 0, alpha: -90 * DEG, d: 0.160, thetaOffset: 0, direction: 1 },
        { name: JOINT_NAMES[1], type: 'revolute', a: 0.240, alpha: 0, d: 0, thetaOffset: -90 * DEG, direction: 1 },
        { name: JOINT_NAMES[2], type: 'revolute', a: 0, alpha: -90 * DEG, d: 0, thetaOffset: 0, direction: 1 },
        { name: JOINT_NAMES[3], type: 'revolute', a: 0, alpha: 90 * DEG, d: 0.220, thetaOffset: 0, direction: 1 },
        { name: JOINT_NAMES[4], type: 'revolute', a: 0, alpha: -90 * DEG, d: 0, thetaOffset: 0, direction: 1 },
        { name: JOINT_NAMES[5], type: 'revolute', a: 0, alpha: 0, d: mp.flangeD6, thetaOffset: 0, direction: 1 },
      ],
      base: { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 },
      tool: { name: 'Pince parallèle', x: 0, y: 0, z: mp.tcpZ, roll: 0, pitch: 0, yaw: 0 },
    },
    limits: {
      // Butées issues du balayage CAO (auto-collisions, autres axes à zéro) avec 5–10° de marge ;
      // J1, J4, J6 : limitées par les câbles.
      joints: [
        { min: -170 * DEG, max: 170 * DEG, vmax: 120 * DEG, amax: 300 * DEG, jmax: 3000 * DEG },
        { min: -130 * DEG, max: 130 * DEG, vmax: 90 * DEG, amax: 200 * DEG, jmax: 2000 * DEG },
        { min: -170 * DEG, max: 60 * DEG, vmax: 110 * DEG, amax: 300 * DEG, jmax: 3000 * DEG },
        { min: -170 * DEG, max: 170 * DEG, vmax: 170 * DEG, amax: 600 * DEG, jmax: 6000 * DEG },
        { min: -115 * DEG, max: 115 * DEG, vmax: 170 * DEG, amax: 600 * DEG, jmax: 6000 * DEG },
        { min: -180 * DEG, max: 180 * DEG, vmax: 200 * DEG, amax: 700 * DEG, jmax: 7000 * DEG },
      ],
      cartesian: { vmax: 0.25, amax: 1.0, jmax: 10, wmax: 90 * DEG, alphamax: 360 * DEG },
      workspace: { enabled: false, xmin: -0.7, xmax: 0.7, ymin: -0.7, ymax: 0.7, zmin: 0.005, zmax: 0.9 },
      floorZ: 0,
    },
    dynamics: {
      gravity: [0, 0, -9.81],
      links: deepClone(mp.links),
      tool: deepClone(mp.tool),
      payload: { mass: 0.2, com: [0, 0, 0] },
      payloadRated: 0.5,
      friction: [
        { coulomb: 0.35, viscous: 0.30, stiction: 0.45, stribeckVel: 0.02 },
        { coulomb: 0.70, viscous: 0.60, stiction: 0.90, stribeckVel: 0.02 },
        { coulomb: 0.45, viscous: 0.40, stiction: 0.60, stribeckVel: 0.02 },
        { coulomb: 0.15, viscous: 0.10, stiction: 0.20, stribeckVel: 0.02 },
        { coulomb: 0.15, viscous: 0.10, stiction: 0.20, stribeckVel: 0.02 },
        { coulomb: 0.10, viscous: 0.08, stiction: 0.14, stribeckVel: 0.02 },
      ],
    },
    actuators: {
      supplyVoltage: 36,
      joints: J6(makerActuator),
    },
    control: {
      mode: 'stepper',
      loopHz: 1000,
      feedforward: { gravity: true, friction: true, inertia: true },
      modelError: 0,
      joints: [
        { kp: 700, ki: 3500, kd: 22, iLimit: 5, dFilterHz: 150, deadband: 0, mitKp: 90, mitKd: 3.0, torqueLimit: 1 },
        { kp: 1100, ki: 5500, kd: 35, iLimit: 8, dFilterHz: 150, deadband: 0, mitKp: 130, mitKd: 4.0, torqueLimit: 1 },
        { kp: 500, ki: 2500, kd: 15, iLimit: 5, dFilterHz: 150, deadband: 0, mitKp: 90, mitKd: 2.5, torqueLimit: 1 },
        { kp: 80, ki: 400, kd: 2.5, iLimit: 2, dFilterHz: 200, deadband: 0, mitKp: 30, mitKd: 1.0, torqueLimit: 1 },
        { kp: 80, ki: 400, kd: 2.5, iLimit: 2, dFilterHz: 200, deadband: 0, mitKp: 30, mitKd: 1.0, torqueLimit: 1 },
        { kp: 40, ki: 200, kd: 1.2, iLimit: 1, dFilterHz: 200, deadband: 0, mitKp: 15, mitKd: 0.5, torqueLimit: 1 },
      ],
      impedance: { kTrans: 800, kRot: 15, zeta: 1.0, nullKd: 0.5 },
      autotune: { bandwidthHz: 8, zeta: 0.9 },
    },
    trajectory: {
      profile: 'scurve',
      speedOverride: 0.5,
      accelScale: 1,
      blendRadius: 0.005,
      sampleHz: 250,
      ik: { method: 'auto', configuration: 'closest', damping: 0.02, maxIter: 150, tolPos: 1e-5, tolRot: 1e-4, orientationWeight: 0.15 },
      jog: { linStep: 0.005, angStep: 5 * DEG, linSpeed: 0.05, jointSpeed: 30 * DEG },
    },
    gripper: {
      type: 'parallel',
      strokeMax: 0.05,
      speed: 0.04,
      force: 25,
      fingerLength: 0.055,
      servoMinUs: 900,
      servoMaxUs: 2100,
    },
    safety: {
      collisionDetection: true,
      collisionThreshold: 3.0,
      selfCollision: true,
      floorCollision: true,
      capsuleMargin: 0.004,
      capsules: deepClone(COLLISION_CAPSULES),
      reducedSpeed: 0.25,
      collabTcpSpeed: 0.25,
      watchdogMs: 250,
      softLimitMargin: 1 * DEG,
    },
    simulation: {
      dt: 0.00025,
      realtimeFactor: 1,
      enableFriction: true,
      enableBacklash: true,
      enableStepLoss: true,
      sensorNoise: 0,
      recordHz: 200,
      plotWindow: 10,
    },
    hardware: {
      controller: 'teensy41',
      stepPulse: 3e-6,
      dirSetup: 6e-6,
      enableActiveLow: true,
      streamHz: 200,
      streamDelay: 0.04,
      canBitrate: 1000000,
      mitCtrlHz: 400,
      feedbackTimeout: 0.05,
      maxMotorTemp: 85,
      pins: [
        { step: 2, dir: 3, limit: 24, invertDir: false, limitActiveLow: true },
        { step: 4, dir: 5, limit: 25, invertDir: false, limitActiveLow: true },
        { step: 6, dir: 7, limit: 26, invertDir: false, limitActiveLow: true },
        { step: 8, dir: 9, limit: 27, invertDir: false, limitActiveLow: true },
        { step: 10, dir: 11, limit: 28, invertDir: false, limitActiveLow: true },
        { step: 12, dir: 14, limit: 29, invertDir: false, limitActiveLow: true },
      ],
      enablePin: 15,
      estopPin: 30,
      gripperPin: 33,
      homing: [
        { enabled: true, direction: -1, speed: 20 * DEG, slowSpeed: 2 * DEG, backoff: 4 * DEG, switchPosition: -172 * DEG, order: 6 },
        { enabled: true, direction: -1, speed: 15 * DEG, slowSpeed: 2 * DEG, backoff: 4 * DEG, switchPosition: -133 * DEG, order: 5 },
        { enabled: true, direction: 1, speed: 15 * DEG, slowSpeed: 2 * DEG, backoff: 4 * DEG, switchPosition: 62 * DEG, order: 4 },
        { enabled: true, direction: -1, speed: 30 * DEG, slowSpeed: 3 * DEG, backoff: 5 * DEG, switchPosition: -172 * DEG, order: 3 },
        { enabled: true, direction: -1, speed: 30 * DEG, slowSpeed: 3 * DEG, backoff: 5 * DEG, switchPosition: -117 * DEG, order: 1 },
        { enabled: true, direction: -1, speed: 30 * DEG, slowSpeed: 3 * DEG, backoff: 5 * DEG, switchPosition: -182 * DEG, order: 2 },
      ],
    },
    poses: {
      home: [0, 0, 0, 0, 90 * DEG, 0],
      rest: [0, -10 * DEG, 55 * DEG, 0, 35 * DEG, 0],
      zero: [0, 0, 0, 0, 0, 0],
    },
  };
}

// ------------------------------------------------------------------ ORION-6 PRO (mode MIT)
export function makeOrion6Pro() {
  const p = makeOrion6Maker();
  const mp = MASS_PROPERTIES.pro;
  p.meta.name = 'ORION-6 PRO';
  p.meta.variant = 'pro';
  p.meta.description = 'Variante à actionneurs brushless QDD pilotés en mode MIT sur bus CAN (type Damiao DM4340 / DM4310), PD + compensation de gravité.';
  const models = ['DM4340', 'DM4340', 'DM4340', 'DM4310', 'DM4310', 'DM4310'];
  p.actuators.supplyVoltage = 24;
  p.actuators.joints = models.map((m, i) => {
    const q = QDD_LIBRARY[m];
    return {
      type: 'bldc_mit',
      model: `${m} (QDD, mode MIT)`,
      gearRatio: q.gearRatio,
      efficiency: 0.9,
      rotorInertia: q.rotorInertia,
      backlash: 0.15 * DEG,
      stiffness: [6000, 6000, 6000, 2000, 2000, 2000][i],
      holdingTorque: 0, stepAngle: 1.8 * DEG, microsteps: 1, ratedCurrent: 0, runCurrent: 0,
      phaseResistance: 1, phaseInductance: 1e-3,
      peakTorque: q.peakTorque,
      ratedTorque: q.ratedTorque,
      maxSpeed: q.maxSpeed,
      torqueConstant: q.torqueConstant,
      encoderCpr: 16384,
      encoderOnOutput: true,
      canId: i + 1,
      pMax: q.pMax, vMax: q.vMax, tMax: q.tMax, kpMax: 500, kdMax: 5,
    };
  });
  p.dynamics.links = deepClone(mp.links);
  p.dynamics.tool = deepClone(mp.tool);
  p.dynamics.friction = J6((i) => (i < 3
    ? { coulomb: 0.25, viscous: 0.08, stiction: 0.3, stribeckVel: 0.02 }
    : { coulomb: 0.08, viscous: 0.03, stiction: 0.1, stribeckVel: 0.02 }));
  p.control.mode = 'mit';
  p.control.joints = p.control.joints.map((c, i) => ({
    ...c,
    mitKp: [80, 120, 80, 30, 25, 15][i],
    mitKd: [2.5, 3.5, 2.5, 1.0, 0.8, 0.5][i],
  }));
  p.limits.joints = p.limits.joints.map((l, i) => ({
    ...l,
    vmax: Math.min(QDD_LIBRARY[models[i]].maxSpeed * 0.8, [150, 120, 150, 360, 360, 360][i] * DEG),
    amax: [400, 300, 400, 900, 900, 900][i] * DEG,
    jmax: [4000, 3000, 4000, 9000, 9000, 9000][i] * DEG,
  }));
  p.hardware.controller = 'mit_can_bridge';
  return p;
}

// ------------------------------------------------------------------ Démo : poignet décalé (type cobot)
export function makeOffsetWristCobot() {
  const p = makeOrion6Pro();
  p.meta.name = 'Cobot poignet décalé (démo)';
  p.meta.variant = 'custom';
  p.meta.description = 'Géométrie de cobot à poignet décalé (table DH publiée de type UR5) : le modèle inverse analytique ne s’applique pas, le solveur numérique prend le relais.';
  const a = [0, -0.425, -0.39225, 0, 0, 0];
  const d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823];
  const al = [90 * DEG, 0, 0, 90 * DEG, -90 * DEG, 0];
  p.kinematics.joints = p.kinematics.joints.map((j, i) => ({ ...j, a: a[i], d: d[i], alpha: al[i], thetaOffset: 0, direction: 1 }));
  p.kinematics.tool.z = 0.12;
  const masses = [3.7, 8.393, 2.275, 1.219, 1.219, 0.1879];
  p.dynamics.links = masses.map((m, i) => ({
    mass: m,
    com: [[0, -0.02561, 0.00193], [0.2125, 0, 0.11336], [0.15, 0, 0.0265], [0, -0.0018, 0.01634], [0, 0.0018, 0.01634], [0, 0, -0.001159]][i],
    inertia: [0.0102, 0.0102, 0.00666, 0, 0, 0].map((v) => v * m / 3.7),
  }));
  p.actuators.joints = p.actuators.joints.map((a2, i) => ({ ...a2, peakTorque: [150, 150, 150, 28, 28, 28][i], ratedTorque: [60, 60, 60, 12, 12, 12][i], maxSpeed: Math.PI, model: 'Servo générique', type: 'bldc_mit', tMax: [150, 150, 150, 28, 28, 28][i], vMax: 10 }));
  p.control.joints = p.control.joints.map((c, i) => ({ ...c, mitKp: [400, 600, 300, 60, 60, 30][i], mitKd: [20, 30, 15, 3, 3, 1.5][i] }));
  p.limits.joints = p.limits.joints.map((l) => ({ ...l, min: -2 * Math.PI, max: 2 * Math.PI }));
  p.poses.home = [0, -90 * DEG, 90 * DEG, -90 * DEG, -90 * DEG, 0];
  p.poses.rest = [0, -90 * DEG, 140 * DEG, -140 * DEG, -90 * DEG, 0];
  p.dynamics.payloadRated = 5;
  return p;
}

export const PRESETS = [
  { id: 'orion6-maker', label: 'ORION-6 MAKER (pas-à-pas + cycloïdal)', make: makeOrion6Maker },
  { id: 'orion6-pro', label: 'ORION-6 PRO (QDD mode MIT, CAN)', make: makeOrion6Pro },
  { id: 'cobot-offset', label: 'Démo : cobot à poignet décalé', make: makeOffsetWristCobot },
];

export const defaultParams = () => makeOrion6Maker();
