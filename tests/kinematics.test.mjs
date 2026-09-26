import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Kinematics, rng } from '../studio/core/kinematics.js';
import { makeOrion6Maker, makeOffsetWristCobot } from '../studio/core/defaults.js';
import { mat4Pos, mat4Rot, rotAngleBetween, DEG } from '../studio/core/math3d.js';

const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} ≉ ${b} (tol ${tol})`);

test('FK zéro : bras supérieur vertical, avant-bras horizontal, outil vers +X', () => {
  const p = makeOrion6Maker();
  const k = new Kinematics(p);
  const f = k.fk([0, 0, 0, 0, 0, 0]);
  const d1 = p.kinematics.joints[0].d, a2 = p.kinematics.joints[1].a, d4 = p.kinematics.joints[3].d;
  const d6 = p.kinematics.joints[5].d, tz = p.kinematics.tool.z;
  // Centre du poignet (repère 4)
  const W = mat4Pos(f.frames[4]);
  close(W[0], d4, 1e-12, 'Wx'); close(W[1], 0, 1e-12, 'Wy'); close(W[2], d1 + a2, 1e-12, 'Wz');
  // TCP devant le poignet, à hauteur du poignet
  const P = mat4Pos(f.tcp);
  close(P[0], d4 + d6 + tz, 1e-12, 'TCPx'); close(P[2], d1 + a2, 1e-12, 'TCPz');
  // Axes : J1 vertical, J2/J3/J5 selon +Y, J4/J6 selon +X
  assert.deepEqual(f.axes[0].z.map((v) => Math.round(v) + 0), [0, 0, 1]);
  assert.deepEqual(f.axes[1].z.map((v) => Math.round(v) + 0), [0, 1, 0]);
  assert.deepEqual(f.axes[2].z.map((v) => Math.round(v) + 0), [0, 1, 0]);
  assert.deepEqual(f.axes[3].z.map((v) => Math.round(v) + 0), [1, 0, 0]);
  assert.deepEqual(f.axes[4].z.map((v) => Math.round(v) + 0), [0, 1, 0]);
  assert.deepEqual(f.axes[5].z.map((v) => Math.round(v) + 0), [1, 0, 0]);
});

test('Sens positifs : J2+ penche vers l’avant, J5 +90° oriente l’outil vers le bas', () => {
  const k = new Kinematics(makeOrion6Maker());
  const e0 = mat4Pos(k.fk([0, 0, 0, 0, 0, 0]).frames[2]);
  const e1 = mat4Pos(k.fk([0, 0.3, 0, 0, 0, 0]).frames[2]);
  assert.ok(e1[0] > e0[0], 'le coude avance quand J2 augmente');
  const T = k.tcp([0, 0, 0, 0, 90 * DEG, 0]);
  const z6 = [T[2], T[6], T[10]];
  close(z6[2], -1, 1e-9, 'outil vers le bas');
});

test('IK analytique : 8 solutions, aller-retour exact sur 300 poses aléatoires', () => {
  const k = new Kinematics(makeOrion6Maker());
  assert.ok(k.analyticInfo.ok, k.analyticInfo.reasons.join(','));
  const rand = rng(42);
  let found = 0;
  for (let n = 0; n < 300; n++) {
    const q = k.limits.map((l) => l.min + (l.max - l.min) * rand());
    const T = k.tcp(q);
    const sols = k.ikAnalyticAll(T, q);
    assert.ok(sols.length >= 4, 'au moins 4 solutions');
    // Toutes les solutions doivent reproduire la pose
    for (const s of sols) {
      const Ts = k.tcp(s.q);
      const ep = Math.hypot(...mat4Pos(Ts).map((v, i) => v - mat4Pos(T)[i]));
      assert.ok(ep < 1e-8, `erreur position ${ep}`);
      assert.ok(rotAngleBetween(mat4Rot(Ts), mat4Rot(T)) < 1e-7, 'erreur orientation');
    }
    // La configuration d’origine fait partie des solutions
    const hit = sols.some((s) => s.q.every((v, i) => Math.abs(v - q[i]) < 1e-6));
    if (hit) found++;
  }
  assert.ok(found >= 295, `configuration d’origine retrouvée ${found}/300`);
});

test('solveIK « closest » retrouve la configuration de départ', () => {
  const k = new Kinematics(makeOrion6Maker());
  const q0 = [0.3, -0.4, 0.5, 0.2, 1.0, -0.3];
  const r = k.solveIK(k.tcp(q0), { seed: q0.map((v) => v + 0.05) });
  assert.ok(r.ok);
  r.q.forEach((v, i) => close(v, q0[i], 1e-6, `q${i + 1}`));
});

test('IK numérique (poignet décalé, type cobot) : convergence', () => {
  const k = new Kinematics(makeOffsetWristCobot());
  assert.equal(k.analyticInfo.ok, false);
  const rand = rng(3);
  let ok = 0;
  for (let n = 0; n < 40; n++) {
    const q = [0, 0, 0, 0, 0, 0].map(() => (rand() - 0.5) * 2.5);
    const seed = q.map((v) => v + (rand() - 0.5) * 0.4);
    const r = k.solveIK(k.tcp(q), { seed });
    if (r.ok) ok++;
  }
  assert.ok(ok >= 38, `convergence ${ok}/40`);
});

test('Jacobien = différences finies du FK', () => {
  const k = new Kinematics(makeOrion6Maker());
  const q = [0.2, -0.3, 0.4, -0.5, 0.6, 0.7];
  const J = k.jacobian(q);
  const h = 1e-7;
  const p0 = mat4Pos(k.tcp(q));
  for (let i = 0; i < 6; i++) {
    const qh = q.slice(); qh[i] += h;
    const p1 = mat4Pos(k.tcp(qh));
    for (let r = 0; r < 3; r++) close((p1[r] - p0[r]) / h, J[r][i], 1e-5, `J[${r}][${i}]`);
  }
});

test('Convention DH modifiée : même robot décrit en MDH → même TCP', () => {
  const p = makeOrion6Maker();
  const k = new Kinematics(p);
  // Conversion DH → MDH : le segment i de la table MDH porte (a_{i-1}, α_{i-1}).
  const J = p.kinematics.joints;
  const pm = structuredClone(p);
  pm.kinematics.convention = 'MDH';
  pm.kinematics.joints = J.map((j, i) => ({
    ...j,
    a: i === 0 ? 0 : J[i - 1].a,
    alpha: i === 0 ? 0 : J[i - 1].alpha,
  }));
  // La dernière transformation (a6, α6) passe dans l’outil.
  const km = new Kinematics(pm);
  const tail = { a: J[5].a, alpha: J[5].alpha };
  assert.equal(tail.a, 0); assert.equal(tail.alpha, 0);
  const q = [0.1, 0.2, -0.3, 0.4, -0.5, 0.6];
  const P1 = mat4Pos(k.tcp(q)), P2 = mat4Pos(km.tcp(q));
  P1.forEach((v, i) => close(v, P2[i], 1e-12, 'TCP MDH'));
  const r = km.solveIK(k.tcp(q), { seed: q.map((v) => v + 0.1) });
  assert.ok(r.ok && r.method === 'numeric');
});
