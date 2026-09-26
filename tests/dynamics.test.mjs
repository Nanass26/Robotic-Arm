import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Kinematics } from '../studio/core/kinematics.js';
import { Dynamics } from '../studio/core/dynamics.js';
import { makeOrion6Maker } from '../studio/core/defaults.js';
import { mat4TransformPoint, mat4Pos, cross, sub } from '../studio/core/math3d.js';
import { cholesky } from '../studio/core/linalg.js';

const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} ≉ ${b} (tol ${tol})`);
const setup = () => { const p = makeOrion6Maker(); const k = new Kinematics(p); return { p, k, d: new Dynamics(k, p) }; };

test('M(q) symétrique définie positive', () => {
  const { d } = setup();
  for (const q of [[0, 0, 0, 0, 0, 0], [0.4, -0.6, 0.9, 1.2, -0.8, 2.0], [-1, 1, -1, 0.5, 1.5, -2]]) {
    const M = d.massMatrix(q);
    assert.ok(cholesky(M) !== null, 'Cholesky');
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) close(M[i][j], M[j][i], 1e-12);
  }
});

test('RNEA cohérent : τ = M·q̈ + b(q, q̇)', () => {
  const { d } = setup();
  const q = [0.3, -0.2, 0.5, -0.7, 0.9, 0.1], qd = [0.5, -0.4, 0.3, 1.0, -0.8, 0.6], qdd = [1, -2, 0.5, 3, -1, 2];
  const tau = d.rnea(q, qd, qdd, true);
  const M = d.massMatrix(q, null, false);
  const b = d.bias(q, qd);
  for (let i = 0; i < 6; i++) {
    const s = M[i].reduce((acc, m, j) => acc + m * qdd[j], 0) + b[i];
    close(tau[i], s, 1e-10, `τ${i + 1}`);
  }
});

test('Gravité : couple de J2 = moment du poids des segments 2..6 autour de l’axe J2', () => {
  const { p, k, d } = setup();
  const q = [0.2, 0.7, -0.3, 0.4, 0.5, 0.1];
  const f = k.fk(q);
  const tg = d.gravityTorque(q, f);
  const { o, z } = f.axes[1];
  let m = [0, 0, 0];
  const add = (b, frame) => {
    const c = mat4TransformPoint(frame, b.com);
    const F = [0, 0, -9.81 * b.mass];
    const mm = cross(sub(c, o), F);
    m = [m[0] + mm[0], m[1] + mm[1], m[2] + mm[2]];
  };
  for (let i = 1; i < 6; i++) for (const b of d.bodiesOf(i)) add(b, f.frames[i + 1]);
  const expected = -(m[0] * z[0] + m[1] * z[1] + m[2] * z[2]); // couple à fournir = −moment du poids
  close(tg[1], expected, 1e-10, 'τg2');
  void p;
});

test('Conservation de l’énergie sans frottement (RK4)', () => {
  const { d } = setup();
  let q = [0.1, 0.8, -0.4, 0.3, 1.1, 0.2], v = [0, 0, 0, 0, 0, 0];
  const E0 = d.energy(q, v).E;
  const f = (q1, v1) => d.forward(q1, v1, [0, 0, 0, 0, 0, 0]);
  const h = 5e-4;
  for (let k = 0; k < 1000; k++) {
    const a1 = f(q, v);
    const q2 = q.map((x, i) => x + 0.5 * h * v[i]), v2 = v.map((x, i) => x + 0.5 * h * a1[i]);
    const a2 = f(q2, v2);
    const q3 = q.map((x, i) => x + 0.5 * h * v2[i]), v3 = v.map((x, i) => x + 0.5 * h * a2[i]);
    const a3 = f(q3, v3);
    const q4 = q.map((x, i) => x + h * v3[i]), v4 = v.map((x, i) => x + h * a3[i]);
    const a4 = f(q4, v4);
    q = q.map((x, i) => x + (h / 6) * (v[i] + 2 * v2[i] + 2 * v3[i] + v4[i]));
    v = v.map((x, i) => x + (h / 6) * (a1[i] + 2 * a2[i] + 2 * a3[i] + a4[i]));
  }
  const E1 = d.energy(q, v).E;
  assert.ok(Math.abs(v[1]) > 0.1, 'le bras est bien en mouvement');
  close(E1, E0, 1e-4 * Math.max(1, Math.abs(E0)), 'énergie');
});

test('Effort extérieur au TCP : τ = −Jᵀ·F en statique', () => {
  const { k, d } = setup();
  const q = [0.3, 0.2, 0.1, -0.4, 0.8, 0.3];
  const f = k.fk(q);
  const F = [3, -2, 5];
  const P = mat4Pos(f.tcp);
  const t0 = d.rnea(q, [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], false, f, { f: F, n: [0, 0, 0], p: P });
  const J = k.jacobian(q, f);
  for (let i = 0; i < 6; i++) {
    const jt = J[0][i] * F[0] + J[1][i] * F[1] + J[2][i] * F[2];
    close(t0[i], -jt, 1e-10, `τ${i + 1}`);
  }
});
