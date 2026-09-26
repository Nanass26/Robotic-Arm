import { test } from 'node:test';
import assert from 'node:assert/strict';
import { jointFrames, toURDF, toMJCF, toFirmwareConfig, toFirmwareSetCommands } from '../studio/core/export.js';
import { packMitCommand, unpackMitFeedback, parseLine, encodeCommand, checksum } from '../studio/core/protocol.js';
import { Kinematics } from '../studio/core/kinematics.js';
import { makeOrion6Maker, makeOffsetWristCobot } from '../studio/core/defaults.js';
import { mat4Mul, mat4Rz, mat4Transl, poseToMat4 } from '../studio/core/math3d.js';

function chainFK(params, q) {
  const { origin, tail } = jointFrames(params);
  let L = poseToMat4(params.kinematics.base);
  const frames = [L];
  params.kinematics.joints.forEach((j, i) => {
    const dir = j.direction < 0 ? -1 : 1;
    const motion = j.type === 'prismatic' ? mat4Transl(0, 0, dir * q[i]) : mat4Rz(dir * q[i]);
    L = mat4Mul(mat4Mul(L, origin[i]), motion);
    frames.push(mat4Mul(L, tail[i]));
  });
  return frames;
}

for (const [name, mk] of [['ORION-6 DH', makeOrion6Maker], ['cobot DH', makeOffsetWristCobot]]) {
  for (const conv of ['DH', 'MDH']) {
    test(`Chaîne URDF/MJCF ≡ DH (${name}, ${conv})`, () => {
      const p = mk();
      if (conv === 'MDH') {
        const J = p.kinematics.joints;
        p.kinematics.convention = 'MDH';
        p.kinematics.joints = J.map((j, i) => ({ ...j, a: i ? J[i - 1].a : 0, alpha: i ? J[i - 1].alpha : 0 }));
      }
      p.kinematics.base = { x: 0.1, y: -0.2, z: 0.05, roll: 0.1, pitch: -0.2, yaw: 0.3 };
      p.kinematics.joints[2].direction = -1;
      const k = new Kinematics(p);
      const q = [0.3, -0.5, 0.7, 1.1, -0.9, 0.4];
      const a = k.fk(q).frames, b = chainFK(p, q);
      for (let i = 0; i < a.length; i++) for (let e = 0; e < 16; e++) assert.ok(Math.abs(a[i][e] - b[i][e]) < 1e-12, `repère ${i}`);
    });
  }
}

test('Exports URDF / MJCF / firmware bien formés', () => {
  const p = makeOrion6Maker();
  const u = toURDF(p);
  assert.match(u, /<robot name="orion_6">/);
  assert.equal((u.match(/<joint name="joint\d"/g) || []).length, 6);
  const m = toMJCF(p);
  assert.equal((m.match(/<body /g) || []).length, 7);
  assert.equal((m.match(/<\/body>/g) || []).length, 7);
  const h = toFirmwareConfig(p);
  assert.match(h, /#define ORION_NUM_AXES 6/);
  // J1 : 200 pas × 16 × 25 / 360 = 222,22 pas/°
  assert.match(h, /STEPS_PER_DEG\[ORION_NUM_AXES\] = \{ 222\.22222/);
  assert.ok(toFirmwareSetCommands(p).length > 50);
});

test('Trames CAN mode MIT : aller-retour', () => {
  const lim = { pMax: 12.5, vMax: 8, tMax: 28, kpMax: 500, kdMax: 5 };
  const f = packMitCommand({ p: 1.2345, v: -2.5, kp: 80, kd: 2.5, t: 3.3 }, lim);
  assert.equal(f.length, 8);
  // Retour : on réutilise le codage (position 16 bits, vitesse 12, couple 12)
  const fb = Uint8Array.from([0x11, f[0], f[1], f[2], (f[3] & 0xf0) | (f[6] & 0x0f), f[7], 30, 35]);
  const r = unpackMitFeedback(fb, lim);
  assert.ok(Math.abs(r.p - 1.2345) < 25 / 65535 + 1e-9);
  assert.ok(Math.abs(r.v + 2.5) < 16 / 4095 + 1e-9);
  assert.ok(Math.abs(r.t - 3.3) < 56 / 4095 + 1e-9);
  assert.equal(r.id, 1); assert.equal(r.err, 1);
});

test('Protocole série : somme de contrôle et analyse', () => {
  const line = encodeCommand('SP', [12, 1.5, -2, 3, 0, 90, 0], true);
  const body = line.trim().split('*')[0];
  assert.equal(line.trim().split('*')[1], checksum(body));
  const st = parseLine('ST RUN 12345 1.5 -2 3 0 90 0 07 00');
  assert.equal(st.state, 'RUN'); assert.equal(st.q.length, 6); assert.equal(st.flags.moving, true); assert.equal(st.flags.fault, false);
  assert.equal(parseLine(`OK PONG*${checksum('OK PONG')}`).type, 'ok');
  assert.equal(parseLine('OK PONG*00').type, 'bad');
});
