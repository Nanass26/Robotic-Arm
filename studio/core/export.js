// ORION-6 — Exports : URDF (ROS/ROS 2, MoveIt), MJCF (MuJoCo), en-tête firmware, JSON.

import {
  mat4Mul, mat4Identity, mat4Rot, mat4Pos, mat4Transl, mat4Rx, mat4Rz, mat3ToRpy, mat3ToQuat,
  mat4TransformPoint, inertiaFromVector, rotateInertia, poseToMat4, DEG,
} from './math3d.js';
import { capsulesFromDH } from './safety.js';
import { Kinematics } from './kinematics.js';
import { ActuatorModel } from './actuators.js';

const f6 = (v) => (Math.abs(v) < 1e-12 ? '0' : Number(v.toPrecision(8)).toString());
const vec = (a) => a.map(f6).join(' ');
const slug = (s) => (s || 'robot').toLowerCase().normalize('NFD').replace(/[̀-ͯ]/g, '').replace(/[^a-z0-9]+/g, '_').replace(/^_|_$/g, '');

/**
 * Décomposition de la chaîne DH en repères « articulation » (URDF/MJCF) :
 *   origin[i] : transformation constante du repère de lien i−1 vers le repère d’articulation i
 *   tail[i]   : repère DH i exprimé dans le repère de lien i (données inertielles / géométrie)
 * Lien 0 = base_link (repère de base du robot).
 */
export function jointFrames(params) {
  const k = params.kinematics;
  const J = k.joints;
  const n = J.length;
  const origin = [], tail = [];
  for (let i = 0; i < n; i++) {
    const j = J[i];
    if (k.convention === 'MDH') {
      origin.push(mat4Mul(mat4Mul(mat4Rx(j.alpha), mat4Transl(j.a, 0, 0)), mat4Mul(mat4Transl(0, 0, j.type === 'prismatic' ? j.d : j.d), mat4Rz(j.thetaOffset))));
      tail.push(mat4Identity());
    } else {
      const prevTail = i === 0 ? mat4Identity() : dhTail(J[i - 1]);
      const dConst = j.type === 'prismatic' ? mat4Transl(0, 0, j.d) : mat4Identity();
      origin.push(mat4Mul(prevTail, mat4Mul(mat4Rz(j.thetaOffset), dConst)));
      tail.push(j.type === 'prismatic' ? mat4Mul(mat4Transl(j.a, 0, 0), mat4Rx(j.alpha)) : dhTail(j));
    }
  }
  return { origin, tail, n };
}

function dhTail(j) {
  return mat4Mul(mat4Mul(mat4Transl(0, 0, j.type === 'prismatic' ? 0 : j.d), mat4Transl(j.a, 0, 0)), mat4Rx(j.alpha));
}

function originTag(T) {
  const [r, p, y] = mat3ToRpy(mat4Rot(T));
  return `<origin xyz="${vec(mat4Pos(T))}" rpy="${vec([r, p, y])}"/>`;
}

/** Inertie d’un segment exprimée dans le repère de lien (URDF/MJCF). */
function linkInertial(link, tailT) {
  const com = mat4TransformPoint(tailT, link.com);
  const I = rotateInertia(mat4Rot(tailT), inertiaFromVector(link.inertia));
  return { com, I };
}

export function toURDF(params) {
  const name = slug(params.meta.name);
  const { origin, tail, n } = jointFrames(params);
  const k = params.kinematics;
  const acts = params.actuators.joints.map((a) => new ActuatorModel(a, params.actuators.supplyVoltage));
  const kin = new Kinematics(params);
  const caps = capsulesFromDH(kin, params.safety.linkRadii).capsules;
  const L = [];
  L.push('<?xml version="1.0"?>');
  L.push(`<!-- Généré par ORION Studio — ${params.meta.name} — convention ${k.convention}. Unités SI (m, rad, kg). -->`);
  L.push(`<robot name="${name}">`);
  L.push('  <material name="orion_white"><color rgba="0.92 0.93 0.95 1"/></material>');
  L.push('  <material name="orion_dark"><color rgba="0.18 0.2 0.24 1"/></material>');
  L.push('  <material name="orion_accent"><color rgba="1.0 0.45 0.1 1"/></material>');
  L.push('  <link name="world"/>');
  L.push(`  <joint name="world_to_base" type="fixed"><parent link="world"/><child link="base_link"/>${originTag(poseToMat4(k.base))}</joint>`);
  L.push('  <link name="base_link">');
  L.push('    <visual><origin xyz="0 0 0.04" rpy="0 0 0"/><geometry><cylinder radius="0.065" length="0.08"/></geometry><material name="orion_dark"/></visual>');
  L.push('    <collision><origin xyz="0 0 0.04" rpy="0 0 0"/><geometry><cylinder radius="0.065" length="0.08"/></geometry></collision>');
  L.push('    <inertial><origin xyz="0 0 0.04" rpy="0 0 0"/><mass value="1.5"/><inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.003"/></inertial>');
  L.push('  </link>');
  for (let i = 0; i < n; i++) {
    const j = k.joints[i];
    const link = params.dynamics.links[i];
    const lname = `link${i + 1}`;
    const { com, I } = linkInertial(link, tail[i]);
    L.push(`  <link name="${lname}">`);
    L.push(`    <!-- ${j.name} -->`);
    for (const c of caps.filter((cc) => cc.link === i + 1)) {
      const p0 = mat4TransformPoint(tail[i], c.p0), p1 = mat4TransformPoint(tail[i], c.p1);
      const d = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
      const len = Math.hypot(...d);
      const mid = [(p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2];
      // Orientation du cylindre (axe z URDF) vers d
      const rpy = len > 1e-9 ? dirToRpy(d.map((v) => v / len)) : [0, 0, 0];
      const geom = len > 1e-6 ? `<cylinder radius="${f6(c.r)}" length="${f6(len)}"/>` : `<sphere radius="${f6(c.r)}"/>`;
      L.push(`    <visual><origin xyz="${vec(mid)}" rpy="${vec(rpy)}"/><geometry>${geom}</geometry><material name="${i % 2 ? 'orion_white' : 'orion_accent'}"/></visual>`);
      L.push(`    <collision><origin xyz="${vec(mid)}" rpy="${vec(rpy)}"/><geometry>${geom}</geometry></collision>`);
    }
    L.push(`    <inertial><origin xyz="${vec(com)}" rpy="0 0 0"/><mass value="${f6(link.mass)}"/>` +
      `<inertia ixx="${f6(I[0])}" ixy="${f6(I[1])}" ixz="${f6(I[2])}" iyy="${f6(I[4])}" iyz="${f6(I[5])}" izz="${f6(I[8])}"/></inertial>`);
    L.push('  </link>');
    const lim = params.limits.joints[i];
    const effort = acts[i].type === 'stepper' ? acts[i].capacity(0) : params.actuators.joints[i].peakTorque;
    const fr = params.dynamics.friction[i];
    L.push(`  <joint name="joint${i + 1}" type="${j.type === 'prismatic' ? 'prismatic' : 'revolute'}">`);
    L.push(`    <parent link="${i === 0 ? 'base_link' : `link${i}`}"/><child link="${lname}"/>`);
    L.push(`    ${originTag(origin[i])}`);
    L.push(`    <axis xyz="0 0 ${j.direction < 0 ? -1 : 1}"/>`);
    L.push(`    <limit lower="${f6(lim.min)}" upper="${f6(lim.max)}" effort="${f6(effort)}" velocity="${f6(lim.vmax)}"/>`);
    L.push(`    <dynamics damping="${f6(fr.viscous)}" friction="${f6(fr.coulomb)}"/>`);
    L.push('  </joint>');
  }
  // Bride, outil, TCP
  L.push('  <link name="flange"/>');
  L.push(`  <joint name="flange_joint" type="fixed"><parent link="link${n}"/><child link="flange"/>${originTag(tail[n - 1])}</joint>`);
  const t = params.dynamics.tool;
  L.push('  <link name="tool">');
  if (params.gripper.type !== 'none') {
    L.push(`    <visual><origin xyz="0 0 ${f6(k.tool.z / 2)}" rpy="0 0 0"/><geometry><box size="0.05 0.03 ${f6(Math.max(k.tool.z * 0.8, 0.01))}"/></geometry><material name="orion_dark"/></visual>`);
  }
  L.push(`    <inertial><origin xyz="${vec(t.com)}" rpy="0 0 0"/><mass value="${f6(t.mass)}"/><inertia ixx="${f6(t.inertiaDiag[0])}" ixy="0" ixz="0" iyy="${f6(t.inertiaDiag[1])}" iyz="0" izz="${f6(t.inertiaDiag[2])}"/></inertial>`);
  L.push('  </link>');
  L.push('  <joint name="tool_joint" type="fixed"><parent link="flange"/><child link="tool"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>');
  L.push('  <link name="tcp"/>');
  L.push(`  <joint name="tcp_joint" type="fixed"><parent link="flange"/><child link="tcp"/>${originTag(poseToMat4(k.tool))}</joint>`);
  L.push('</robot>');
  return L.join('\n') + '\n';
}

/** Angles RPY qui orientent l’axe z vers la direction unitaire d. */
function dirToRpy(d) {
  const [x, y, z] = d;
  const pitch = Math.atan2(Math.hypot(x, y), z);
  const yaw = Math.atan2(y, x);
  // R = Rz(yaw)·Ry(pitch) → rpy = (0, pitch, yaw)
  return [0, pitch, yaw];
}

export function toMJCF(params) {
  const name = slug(params.meta.name);
  const { origin, tail, n } = jointFrames(params);
  const k = params.kinematics;
  const kin = new Kinematics(params);
  const caps = capsulesFromDH(kin, params.safety.linkRadii).capsules;
  const acts = params.actuators.joints.map((a) => new ActuatorModel(a, params.actuators.supplyVoltage));
  const posQuat = (T) => `pos="${vec(mat4Pos(T))}" quat="${vec(mat3ToQuat(mat4Rot(T)))}"`;
  const L = [];
  L.push(`<!-- Généré par ORION Studio — ${params.meta.name}. Actionneurs : position (kp = Kp MIT) ou couple selon le mode. -->`);
  L.push(`<mujoco model="${name}">`);
  L.push('  <compiler angle="radian" autolimits="true"/>');
  L.push(`  <option timestep="${f6(Math.max(params.simulation.dt, 0.0005))}" gravity="${vec(params.dynamics.gravity)}" integrator="implicitfast"/>`);
  L.push('  <default><geom contype="0" conaffinity="0" rgba="0.92 0.93 0.95 1"/></default>');
  L.push('  <worldbody>');
  L.push('    <light pos="0 0 3" dir="0 0 -1"/>');
  L.push('    <geom name="floor" type="plane" size="1 1 0.05" rgba="0.8 0.82 0.85 1" contype="1" conaffinity="1"/>');
  let indent = '    ';
  L.push(`${indent}<body name="base_link" ${posQuat(poseToMat4(k.base))}>`);
  L.push(`${indent}  <geom type="cylinder" size="0.065 0.04" pos="0 0 0.04" rgba="0.18 0.2 0.24 1"/>`);
  for (let i = 0; i < n; i++) {
    indent += '  ';
    const j = k.joints[i];
    const link = params.dynamics.links[i];
    const { com, I } = linkInertial(link, tail[i]);
    const lim = params.limits.joints[i];
    const fr = params.dynamics.friction[i];
    const arm = acts[i].Ja;
    L.push(`${indent}<body name="link${i + 1}" ${posQuat(origin[i])}>`);
    L.push(`${indent}  <joint name="joint${i + 1}" type="${j.type === 'prismatic' ? 'slide' : 'hinge'}" axis="0 0 ${j.direction < 0 ? -1 : 1}" range="${f6(lim.min)} ${f6(lim.max)}" damping="${f6(fr.viscous)}" frictionloss="${f6(fr.coulomb)}" armature="${f6(arm)}"/>`);
    L.push(`${indent}  <inertial pos="${vec(com)}" mass="${f6(Math.max(link.mass, 1e-4))}" fullinertia="${f6(Math.max(I[0], 1e-7))} ${f6(Math.max(I[4], 1e-7))} ${f6(Math.max(I[8], 1e-7))} ${f6(I[1])} ${f6(I[2])} ${f6(I[5])}"/>`);
    for (const c of caps.filter((cc) => cc.link === i + 1)) {
      const p0 = mat4TransformPoint(tail[i], c.p0), p1 = mat4TransformPoint(tail[i], c.p1);
      const len = Math.hypot(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
      if (len > 1e-6) L.push(`${indent}  <geom type="capsule" fromto="${vec([...p0, ...p1])}" size="${f6(c.r)}" rgba="${i % 2 ? '0.92 0.93 0.95 1' : '1 0.45 0.1 1'}"/>`);
      else L.push(`${indent}  <geom type="sphere" pos="${vec(p0)}" size="${f6(c.r)}"/>`);
    }
  }
  // Bride et TCP dans le dernier segment
  const flange = tail[n - 1];
  const tcp = mat4Mul(flange, poseToMat4(k.tool));
  L.push(`${indent}  <site name="flange" ${posQuat(flange)} size="0.005"/>`);
  L.push(`${indent}  <site name="tcp" ${posQuat(tcp)} size="0.008" rgba="1 0 0 1"/>`);
  for (let i = n - 1; i >= 0; i--) {
    L.push(`${indent}</body>`);
    indent = indent.slice(2);
  }
  L.push(`${indent}</body>`);
  L.push('  </worldbody>');
  L.push('  <actuator>');
  for (let i = 0; i < n; i++) {
    const c = params.control.joints[i];
    const a = params.actuators.joints[i];
    const cap = a.type === 'stepper' ? acts[i].capacity(0) : a.peakTorque;
    L.push(`    <position name="act${i + 1}" joint="joint${i + 1}" kp="${f6(c.mitKp)}" kv="${f6(c.mitKd)}" forcerange="${f6(-cap)} ${f6(cap)}"/>`);
  }
  L.push('  </actuator>');
  L.push('  <keyframe>');
  for (const [key, q] of Object.entries(params.poses || {})) L.push(`    <key name="${key}" qpos="${vec(q)}" ctrl="${vec(q)}"/>`);
  L.push('  </keyframe>');
  L.push('</mujoco>');
  return L.join('\n') + '\n';
}

/** En-tête C++ pour le firmware Teensy (firmware/orion_fw/src/orion_config.h). */
export function toFirmwareConfig(params, { stamp = true } = {}) {
  const n = params.kinematics.joints.length;
  const a = params.actuators.joints;
  const lim = params.limits.joints;
  const hw = params.hardware;
  const R = 180 / Math.PI;
  const arr = (f, d = 4) => `{ ${Array.from({ length: n }, (_, i) => Number(f(i)).toFixed(d)).join(', ')} }`;
  const arrI = (f) => `{ ${Array.from({ length: n }, (_, i) => Math.round(f(i))).join(', ')} }`;
  const arrB = (f) => `{ ${Array.from({ length: n }, (_, i) => (f(i) ? 'true' : 'false')).join(', ')} }`;
  const stepsPerDeg = (i) => (a[i].type === 'stepper' ? ((360 / (a[i].stepAngle * R)) * a[i].microsteps * a[i].gearRatio) / 360 : 0);
  const homingOrder = [...Array(n).keys()].sort((x, y) => hw.homing[x].order - hw.homing[y].order);
  const L = [];
  L.push('// FICHIER GÉNÉRÉ par ORION Studio (Exporter → Configuration firmware).');
  L.push(`// Robot : ${params.meta.name}${stamp ? ` — ${new Date().toISOString().slice(0, 10)}` : ''}`);
  L.push('// Toutes les valeurs angulaires sont en DEGRÉS côté articulation (sortie de réducteur).');
  L.push('#pragma once');
  L.push('#include <stdint.h>');
  L.push('');
  L.push(`#define ORION_NUM_AXES ${n}`);
  L.push(`#define ORION_ROBOT_NAME "${params.meta.name.replace(/"/g, '')}"`);
  L.push('');
  L.push('namespace orion_cfg {');
  L.push(`// Pas moteur par degré articulaire = (360/angle_pas) × micro-pas × réduction / 360`);
  L.push(`constexpr float STEPS_PER_DEG[ORION_NUM_AXES] = ${arr(stepsPerDeg, 5)};`);
  L.push(`constexpr float MIN_DEG[ORION_NUM_AXES]       = ${arr((i) => lim[i].min * R, 3)};`);
  L.push(`constexpr float MAX_DEG[ORION_NUM_AXES]       = ${arr((i) => lim[i].max * R, 3)};`);
  L.push(`constexpr float VMAX_DEG_S[ORION_NUM_AXES]    = ${arr((i) => lim[i].vmax * R, 3)};`);
  L.push(`constexpr float AMAX_DEG_S2[ORION_NUM_AXES]   = ${arr((i) => lim[i].amax * R, 3)};`);
  L.push(`constexpr uint8_t STEP_PIN[ORION_NUM_AXES]    = ${arrI((i) => hw.pins[i].step)};`);
  L.push(`constexpr uint8_t DIR_PIN[ORION_NUM_AXES]     = ${arrI((i) => hw.pins[i].dir)};`);
  L.push(`constexpr uint8_t LIMIT_PIN[ORION_NUM_AXES]   = ${arrI((i) => hw.pins[i].limit)};`);
  L.push(`constexpr bool INVERT_DIR[ORION_NUM_AXES]     = ${arrB((i) => hw.pins[i].invertDir)};`);
  L.push(`constexpr bool LIMIT_ACTIVE_LOW[ORION_NUM_AXES] = ${arrB((i) => hw.pins[i].limitActiveLow)};`);
  L.push(`constexpr bool HOMING_ENABLED[ORION_NUM_AXES] = ${arrB((i) => hw.homing[i].enabled)};`);
  L.push(`constexpr int8_t HOMING_DIR[ORION_NUM_AXES]   = ${arrI((i) => (hw.homing[i].direction < 0 ? -1 : 1))};`);
  L.push(`constexpr float HOMING_SPEED[ORION_NUM_AXES]  = ${arr((i) => hw.homing[i].speed * R, 3)};`);
  L.push(`constexpr float HOMING_SLOW[ORION_NUM_AXES]   = ${arr((i) => hw.homing[i].slowSpeed * R, 3)};`);
  L.push(`constexpr float HOMING_BACKOFF[ORION_NUM_AXES] = ${arr((i) => hw.homing[i].backoff * R, 3)};`);
  L.push(`constexpr float HOMING_SWITCH_POS[ORION_NUM_AXES] = ${arr((i) => hw.homing[i].switchPosition * R, 3)};`);
  L.push(`constexpr uint8_t HOMING_ORDER[ORION_NUM_AXES] = { ${homingOrder.join(', ')} };`);
  L.push(`constexpr float HOME_POSE_DEG[ORION_NUM_AXES] = ${arr((i) => (params.poses.home[i] || 0) * R, 3)};`);
  L.push(`constexpr uint8_t ENABLE_PIN = ${hw.enablePin};`);
  L.push(`constexpr bool ENABLE_ACTIVE_LOW = ${hw.enableActiveLow ? 'true' : 'false'};`);
  L.push(`constexpr uint8_t ESTOP_PIN = ${hw.estopPin};`);
  L.push(`constexpr uint8_t GRIPPER_PIN = ${hw.gripperPin};`);
  L.push(`constexpr uint16_t GRIPPER_US_CLOSED = ${params.gripper.servoMinUs};`);
  L.push(`constexpr uint16_t GRIPPER_US_OPEN = ${params.gripper.servoMaxUs};`);
  L.push(`constexpr float STEP_PULSE_US = ${(hw.stepPulse * 1e6).toFixed(2)}f;`);
  L.push(`constexpr float DIR_SETUP_US = ${(hw.dirSetup * 1e6).toFixed(2)}f;`);
  L.push(`constexpr uint32_t WATCHDOG_MS = ${params.safety.watchdogMs};`);
  L.push(`constexpr uint32_t CAN_BITRATE = ${hw.canBitrate};`);
  L.push('// Mode MIT (pont CAN) : identifiants et plages des moteurs');
  L.push(`constexpr uint8_t CAN_ID[ORION_NUM_AXES] = ${arrI((i) => a[i].canId)};`);
  L.push(`constexpr float MIT_PMAX[ORION_NUM_AXES] = ${arr((i) => a[i].pMax, 3)};`);
  L.push(`constexpr float MIT_VMAX[ORION_NUM_AXES] = ${arr((i) => a[i].vMax, 3)};`);
  L.push(`constexpr float MIT_TMAX[ORION_NUM_AXES] = ${arr((i) => a[i].tMax, 3)};`);
  L.push(`constexpr float MIT_KPMAX[ORION_NUM_AXES] = ${arr((i) => a[i].kpMax, 3)};`);
  L.push(`constexpr float MIT_KDMAX[ORION_NUM_AXES] = ${arr((i) => a[i].kdMax, 3)};`);
  L.push(`constexpr float MIT_KP[ORION_NUM_AXES] = ${arr((i) => params.control.joints[i].mitKp, 3)};`);
  L.push(`constexpr float MIT_KD[ORION_NUM_AXES] = ${arr((i) => params.control.joints[i].mitKd, 4)};`);
  L.push(`constexpr float MIT_KD_DAMP[ORION_NUM_AXES] = ${arr((i) => Math.min(a[i].kdMax, Math.max(2 * params.control.joints[i].mitKd, 1)), 4)};`);
  L.push(`constexpr int8_t MIT_DIR[ORION_NUM_AXES] = ${arrI((i) => (hw.pins[i].invertDir ? -1 : 1))};`);
  L.push(`constexpr float TAU_MAX[ORION_NUM_AXES] = ${arr((i) => (a[i].peakTorque || a[i].tMax || 1) * (params.control.joints[i].torqueLimit ?? 1), 3)};  // N·m`);
  L.push(`constexpr float REF_POSE_DEG[ORION_NUM_AXES] = ${arr((i) => (params.poses.rest?.[i] || 0) * R, 3)};  // pose de référence (ZERO)`);
  L.push(`constexpr uint16_t MIT_CTRL_HZ = ${Math.round(hw.mitCtrlHz || 400)};`);
  L.push(`constexpr uint16_t MIT_FB_TIMEOUT_MS = ${Math.round((hw.feedbackTimeout ?? 0.05) * 1000)};`);
  L.push(`constexpr uint8_t MIT_MAX_TEMP_C = ${Math.round(hw.maxMotorTemp ?? 85)};`);
  L.push(`constexpr uint16_t STREAM_DELAY_MS = ${Math.round((hw.streamDelay ?? 0.04) * 1000)};`);
  L.push('}  // namespace orion_cfg');
  return L.join('\n') + '\n';
}

/** Sérialisation JSON « propre » (arrondie) d’une configuration. */
export function toJSON(params) {
  return JSON.stringify(params, (k, v) => (typeof v === 'number' ? Number(v.toPrecision(10)) : v), 2);
}

/** Commandes SET à envoyer au firmware pour appliquer la configuration sans recompiler. */
export function toFirmwareSetCommands(params) {
  const R = 180 / Math.PI;
  const a = params.actuators.joints, lim = params.limits.joints, hw = params.hardware;
  const out = [];
  if (hw.controller === 'mit_can_bridge') {
    // Pont CAN mode MIT (firmware/mit_bridge) : gains, butées, couples, plages d’encodage
    const c = params.control.joints;
    a.forEach((ac, i) => {
      const j = i + 1;
      out.push(`SET can_id ${j} ${ac.canId}`, `SET dir ${j} ${hw.pins[i].invertDir ? -1 : 1}`);
      out.push(`SET p_max ${j} ${ac.pMax}`, `SET v_max ${j} ${ac.vMax}`, `SET t_max ${j} ${ac.tMax}`, `SET kp_max ${j} ${ac.kpMax}`, `SET kd_max ${j} ${ac.kdMax}`);
      out.push(`SET kp ${j} ${+c[i].mitKp.toFixed(4)}`, `SET kd ${j} ${+c[i].mitKd.toFixed(4)}`);
      out.push(`SET kd_damp ${j} ${+Math.min(ac.kdMax, Math.max(2 * c[i].mitKd, 1)).toFixed(4)}`);
      out.push(`SET min ${j} ${(lim[i].min * R).toFixed(3)}`, `SET max ${j} ${(lim[i].max * R).toFixed(3)}`, `SET vmax ${j} ${(lim[i].vmax * R).toFixed(3)}`);
      out.push(`SET tau_max ${j} ${((ac.peakTorque || ac.tMax) * (c[i].torqueLimit ?? 1)).toFixed(3)}`);
      out.push(`SET ref_pose ${j} ${((params.poses.rest?.[i] || 0) * R).toFixed(3)}`);
    });
    out.push(`SET watchdog * ${params.safety.watchdogMs}`, `SET stream_delay * ${Math.round((hw.streamDelay ?? 0.04) * 1000)}`);
    out.push(`SET ctrl_hz * ${Math.round(hw.mitCtrlHz || 400)}`, `SET fb_timeout * ${Math.round((hw.feedbackTimeout ?? 0.05) * 1000)}`, `SET max_temp * ${Math.round(hw.maxMotorTemp ?? 85)}`);
    return out;
  }
  a.forEach((ac, i) => {
    const spd = ac.type === 'stepper' ? ((360 / (ac.stepAngle * R)) * ac.microsteps * ac.gearRatio) / 360 : 0;
    out.push(`SET steps_per_deg ${i + 1} ${spd.toFixed(5)}`);
    out.push(`SET min ${i + 1} ${(lim[i].min * R).toFixed(3)}`);
    out.push(`SET max ${i + 1} ${(lim[i].max * R).toFixed(3)}`);
    out.push(`SET vmax ${i + 1} ${(lim[i].vmax * R).toFixed(3)}`);
    out.push(`SET amax ${i + 1} ${(lim[i].amax * R).toFixed(3)}`);
    out.push(`SET invert ${i + 1} ${hw.pins[i].invertDir ? 1 : 0}`);
    out.push(`SET home_dir ${i + 1} ${hw.homing[i].direction < 0 ? -1 : 1}`);
    out.push(`SET home_pos ${i + 1} ${(hw.homing[i].switchPosition * R).toFixed(3)}`);
    out.push(`SET home_speed ${i + 1} ${(hw.homing[i].speed * R).toFixed(3)}`);
    out.push(`SET home_slow ${i + 1} ${(hw.homing[i].slowSpeed * R).toFixed(3)}`);
    out.push(`SET home_backoff ${i + 1} ${(hw.homing[i].backoff * R).toFixed(3)}`);
    out.push(`SET home_en ${i + 1} ${hw.homing[i].enabled ? 1 : 0}`);
    out.push(`SET home_pose ${i + 1} ${((params.poses.home[i] || 0) * R).toFixed(3)}`);
  });
  out.push(`SET watchdog * ${params.safety.watchdogMs}`);
  out.push(`SET stream_delay * ${Math.round((hw.streamDelay ?? 0.04) * 1000)}`);
  out.push(`SET grip_closed * ${params.gripper.servoMinUs}`, `SET grip_open * ${params.gripper.servoMaxUs}`);
  return out;
}

export { DEG };
