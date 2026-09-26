// ORION-6 — Modèle géométrique : direct (FK), jacobien, inverse (IK) analytique et numérique.
import {
  dhStandard, dhModified, mat4Mul, mat4Identity, mat4InvRigid, mat4Pos, mat4Rot, mat4FromRotPos,
  poseToMat4, mat3Mul, mat3T, mat3MulVec, mat3Rx, sub, cross, norm, rotError, wrapAngle, clamp,
  mat4Axis, TAU,
} from './math3d.js';
import { dampedLeastSquares, singularValues, matMul, transpose, det } from './linalg.js';

const EPS = 1e-9;
const isZero = (v, tol = 1e-9) => Math.abs(v) < tol;

/** Générateur pseudo-aléatoire déterministe (mulberry32). */
export function rng(seed = 12345) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class Kinematics {
  constructor(params) {
    this.setParams(params);
  }

  setParams(params) {
    const k = params.kinematics;
    this.params = params;
    this.convention = k.convention === 'MDH' ? 'MDH' : 'DH';
    this.joints = k.joints.map((j) => ({ ...j }));
    this.n = this.joints.length;
    this.base = poseToMat4(k.base);
    this.baseInv = mat4InvRigid(this.base);
    this.tool = poseToMat4(k.tool);
    this.toolInv = mat4InvRigid(this.tool);
    this.limits = params.limits.joints.map((l) => ({ ...l }));
    this.ik = { ...params.trajectory.ik };
    this.analyticInfo = this.detectAnalytic();
  }

  /** Variables DH (θ, d) de l’articulation i pour la valeur articulaire q. */
  dhVars(i, q) {
    const j = this.joints[i];
    const dir = j.direction < 0 ? -1 : 1;
    if (j.type === 'prismatic') return { theta: j.thetaOffset, d: j.d + dir * q };
    return { theta: dir * q + j.thetaOffset, d: j.d };
  }

  /** Transformation élémentaire du segment i. */
  linkTransform(i, q) {
    const j = this.joints[i];
    const { theta, d } = this.dhVars(i, q);
    return this.convention === 'MDH' ? dhModified(j.a, j.alpha, d, theta) : dhStandard(j.a, j.alpha, d, theta);
  }

  /**
   * Modèle géométrique direct.
   * Retourne frames[0..n] (frames[0] = base, frames[i] = repère DH i, dans le monde),
   * flange (= frames[n]), tcp, et pour chaque articulation son axe {o, z} (monde).
   */
  fk(q) {
    const frames = [this.base];
    let T = this.base;
    for (let i = 0; i < this.n; i++) {
      T = mat4Mul(T, this.linkTransform(i, q[i]));
      frames.push(T);
    }
    const axes = new Array(this.n);
    for (let i = 0; i < this.n; i++) {
      const F = this.convention === 'MDH' ? frames[i + 1] : frames[i];
      axes[i] = { o: mat4Pos(F), z: mat4Axis(F, 2) };
    }
    const tcp = mat4Mul(T, this.tool);
    return { frames, flange: T, tcp, axes };
  }

  tcp(q) {
    return this.fk(q).tcp;
  }

  /** Jacobien géométrique 6×n au TCP, exprimé dans le repère monde (lignes : vx vy vz ωx ωy ωz). */
  jacobian(q, fkRes = null) {
    const f = fkRes || this.fk(q);
    const p = mat4Pos(f.tcp);
    const J = [[], [], [], [], [], []];
    for (let i = 0; i < this.n; i++) {
      const { o, z } = f.axes[i];
      const dir = this.joints[i].direction < 0 ? -1 : 1;
      if (this.joints[i].type === 'prismatic') {
        J[0].push(z[0] * dir); J[1].push(z[1] * dir); J[2].push(z[2] * dir);
        J[3].push(0); J[4].push(0); J[5].push(0);
      } else {
        const v = cross(z, sub(p, o));
        J[0].push(v[0] * dir); J[1].push(v[1] * dir); J[2].push(v[2] * dir);
        J[3].push(z[0] * dir); J[4].push(z[1] * dir); J[5].push(z[2] * dir);
      }
    }
    return J;
  }

  /** Indicateurs de conditionnement : manipulabilité de Yoshikawa, valeurs singulières, conditionnement. */
  manipulability(q, fkRes = null) {
    const J = this.jacobian(q, fkRes);
    const sv = singularValues(J);
    const w = this.n === 6 ? Math.abs(det(J)) : Math.sqrt(Math.max(0, det(matMul(J, transpose(J)))));
    const smin = sv[sv.length - 1], smax = sv[0];
    return { w, sv, cond: smin > 1e-12 ? smax / smin : Infinity, sigmaMin: smin };
  }

  withinLimits(q, margin = 0) {
    for (let i = 0; i < this.n; i++) {
      if (q[i] < this.limits[i].min + margin - 1e-9 || q[i] > this.limits[i].max - margin + 1e-9) return false;
    }
    return true;
  }

  clampToLimits(q) {
    return q.map((v, i) => clamp(v, this.limits[i].min, this.limits[i].max));
  }

  // ---------------------------------------------------------------- IK analytique

  /** Détecte si la structure admet la solution analytique (épaule 3R + poignet sphérique, DH standard). */
  detectAnalytic() {
    const J = this.joints;
    const reasons = [];
    if (this.convention !== 'DH') reasons.push('convention DH modifiée');
    if (this.n !== 6) reasons.push('nombre d’axes ≠ 6');
    if (J.some((j) => j.type !== 'revolute')) reasons.push('axe prismatique');
    if (reasons.length === 0) {
      const s = (a) => Math.sin(a), c = (a) => Math.cos(a);
      if (!isZero(Math.abs(s(J[0].alpha)) - 1, 1e-6)) reasons.push('α1 ≠ ±90°');
      if (!isZero(c(J[1].alpha) - 1, 1e-6)) reasons.push('α2 ≠ 0');
      if (!isZero(Math.abs(s(J[2].alpha)) - 1, 1e-6)) reasons.push('α3 ≠ ±90°');
      if (!isZero(J[3].a, 1e-9) || !isZero(J[4].a, 1e-9) || !isZero(J[4].d, 1e-9)) reasons.push('poignet non sphérique (a4, a5, d5 ≠ 0)');
      if (!isZero(Math.abs(s(J[3].alpha)) - 1, 1e-6) || !isZero(Math.abs(s(J[4].alpha)) - 1, 1e-6)) reasons.push('α4/α5 ≠ ±90°');
      const sum = wrapAngle(J[3].alpha + J[4].alpha);
      if (!isZero(sum, 1e-6) && !isZero(Math.abs(sum) - Math.PI, 1e-6)) reasons.push('α4 + α5 ∉ {0, π}');
    }
    return { ok: reasons.length === 0, reasons };
  }

  /** θ (DH) → q (articulaire), ramené dans les butées au plus près de qRef. */
  thetaToQ(i, theta, qRef) {
    const j = this.joints[i];
    const dir = j.direction < 0 ? -1 : 1;
    let q = wrapAngle((theta - j.thetaOffset) * dir);
    const { min, max } = this.limits[i];
    let best = null, bestD = Infinity;
    for (let k = -2; k <= 2; k++) {
      const cand = q + k * TAU;
      if (cand < min - 1e-9 || cand > max + 1e-9) continue;
      const d = Math.abs(cand - qRef);
      if (d < bestD) { bestD = d; best = cand; }
    }
    if (best === null) {
      // Hors butées : on garde la représentation la plus proche de la référence.
      for (let k = -2; k <= 2; k++) {
        const cand = q + k * TAU;
        const d = Math.abs(cand - qRef);
        if (d < bestD) { bestD = d; best = cand; }
      }
      return { q: best, inLimits: false };
    }
    return { q: best, inLimits: true };
  }

  /**
   * Toutes les solutions analytiques (jusqu’à 8) pour une pose TCP cible (monde).
   * Chaque solution : { q, inLimits, config: {shoulder, elbow, wrist}, label }.
   */
  ikAnalyticAll(targetTcp, qRef = null) {
    if (!this.analyticInfo.ok) return [];
    const J = this.joints;
    qRef = qRef || new Array(6).fill(0);
    // Pose de la bride dans le repère de base
    const T6 = mat4Mul(this.baseInv, mat4Mul(targetTcp, this.toolInv));
    const R6 = mat4Rot(T6), p6 = mat4Pos(T6);
    // Centre du poignet
    const zA = mat3MulVec(R6, mat3MulVec(mat3Rx(-J[5].alpha), [0, 0, 1]));
    const xA = mat3MulVec(R6, [1, 0, 0]);
    const W = [
      p6[0] - J[5].d * zA[0] - J[5].a * xA[0],
      p6[1] - J[5].d * zA[1] - J[5].a * xA[1],
      p6[2] - J[5].d * zA[2] - J[5].a * xA[2],
    ];
    const s1 = Math.sign(Math.sin(J[0].alpha));
    const s3 = Math.sign(Math.sin(J[2].alpha));
    const s4 = Math.sign(Math.sin(J[3].alpha));
    const wristSumPi = !isZero(wrapAngle(J[3].alpha + J[4].alpha), 1e-6);
    const a1 = J[0].a, d1 = J[0].d, a2 = J[1].a, a3 = J[2].a, d4 = J[3].d;
    const D = J[1].d + J[2].d;
    const L3 = Math.hypot(a3, d4);
    const phi3 = Math.atan2(-s3 * d4, a3);

    const rho = Math.hypot(W[0], W[1]);
    const phi = Math.atan2(W[1], W[0]);
    const sols = [];
    if (rho < Math.abs(D) - 1e-12) return sols;
    const kk = clamp((-D * s1) / Math.max(rho, 1e-12), -1, 1);
    const as = Math.asin(kk);
    const theta1s = rho < 1e-9 && isZero(D)
      ? [this.joints[0].thetaOffset + (this.joints[0].direction < 0 ? -1 : 1) * qRef[0]] // singularité d’épaule
      : [phi - as, phi - Math.PI + as];

    theta1s.forEach((t1, si) => {
      const c1 = Math.cos(t1), sn1 = Math.sin(t1);
      const X = c1 * W[0] + sn1 * W[1] - a1;
      const Y = s1 * (W[2] - d1);
      const r2 = X * X + Y * Y;
      let cb = (r2 - a2 * a2 - L3 * L3) / (2 * a2 * L3);
      if (cb > 1 + 1e-9 || cb < -1 - 1e-9) return; // hors d’atteinte
      cb = clamp(cb, -1, 1);
      const bAbs = Math.acos(cb);
      for (const sb of [1, -1]) {
        const beta = sb * bAbs;
        const t2 = Math.atan2(Y, X) - Math.atan2(L3 * Math.sin(beta), a2 + L3 * Math.cos(beta));
        const t3 = beta - phi3;
        // Orientation : R36 = R03ᵀ · R06
        const th = [t1, t2, t3];
        let R03 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
        for (let i = 0; i < 3; i++) {
          const A = dhStandard(J[i].a, J[i].alpha, J[i].d, th[i]);
          R03 = mat3Mul(R03, mat4Rot(A));
        }
        const R36 = mat3Mul(mat3T(R03), R6);
        let M = mat3Mul(R36, mat3Rx(-J[5].alpha));
        if (wristSumPi) M = mat3Mul(M, mat3Rx(Math.PI));
        // Extraction ZYZ : M = Rz(a)·Ry(b)·Rz(c)
        const sbv = Math.hypot(M[2], M[5]);
        const wristSols = [];
        if (sbv < 1e-7) {
          // Singularité de poignet : seule la somme (a + c) est définie → on garde θ4 courant.
          const t4ref = this.joints[3].thetaOffset + (this.joints[3].direction < 0 ? -1 : 1) * qRef[3];
          const sum = Math.atan2(M[3], M[0]);
          const b = M[8] > 0 ? 0 : Math.PI;
          wristSols.push([t4ref, b, sum - t4ref]);
        } else {
          const b = Math.atan2(sbv, M[8]);
          const a = Math.atan2(M[5], M[2]);
          const c = Math.atan2(M[7], -M[6]);
          wristSols.push([a, b, c], [a + Math.PI, -b, c + Math.PI]);
        }
        wristSols.forEach(([a, b, c], wi) => {
          const t4 = a;
          const t5 = -s4 * b;
          const t6 = wristSumPi ? -c : c;
          const thetas = [t1, t2, t3, t4, t5, t6];
          let inLimits = true;
          const q = thetas.map((t, i) => {
            const r = this.thetaToQ(i, t, qRef[i]);
            if (!r.inLimits) inLimits = false;
            return r.q;
          });
          sols.push({ q, inLimits, branch: { shoulder: si, elbow: sb > 0 ? 0 : 1, wrist: wi } });
        });
      }
    });
    // Étiquetage géométrique des configurations
    for (const s of sols) s.config = this.configOf(s.q);
    return sols;
  }

  /** Configuration géométrique (épaule avant/arrière, coude haut/bas, poignet normal/retourné). */
  configOf(q) {
    const f = this.fk(q);
    const J = this.joints;
    if (!this.analyticInfo.ok) return { shoulder: '?', elbow: '?', wrist: '?', label: '—' };
    const Tb = f.frames.map((F) => mat4Mul(this.baseInv, F));
    const W = mat4Pos(Tb[4]);
    const S = mat4Pos(Tb[1]);
    const E = mat4Pos(Tb[2]);
    // Direction du plan du bras (x du repère 1 projeté)
    const x1 = mat4Axis(Tb[1], 0);
    const shoulder = (W[0] * x1[0] + W[1] * x1[1]) >= 0 ? 'front' : 'back';
    // Coude au-dessus de la droite épaule→poignet ? (dans le plan du bras, normale = z1)
    const z1 = mat4Axis(Tb[1], 2);
    const sw = sub(W, S), se = sub(E, S);
    const side = cross(sw, se);
    const up = side[0] * z1[0] + side[1] * z1[1] + side[2] * z1[2];
    let elbow = up * Math.sign(Math.sin(J[0].alpha)) >= 0 ? 'up' : 'down';
    if (shoulder === 'back') elbow = elbow === 'up' ? 'down' : 'up';
    const t5 = this.dhVars(4, q[4]).theta;
    const wrist = Math.sin(t5) * Math.sign(Math.sin(J[3].alpha)) >= 0 ? 'noflip' : 'flip';
    const L = { front: 'Épaule avant', back: 'Épaule arrière', up: 'coude haut', down: 'coude bas', noflip: 'poignet normal', flip: 'poignet retourné' };
    return { shoulder, elbow, wrist, label: `${L[shoulder]} · ${L[elbow]} · ${L[wrist]}` };
  }

  /** Choisit une solution selon la préférence (`closest` ou `front-up-noflip`…). */
  pickSolution(sols, qRef, preference = 'closest') {
    const valid = sols.filter((s) => s.inLimits);
    if (!valid.length) return null;
    if (preference && preference !== 'closest') {
      const [sh, el, wr] = preference.split('-');
      const m = valid.filter((s) => s.config.shoulder === sh && s.config.elbow === el && s.config.wrist === wr);
      if (m.length) return m.reduce((a, b) => (this.jointDistance(a.q, qRef) <= this.jointDistance(b.q, qRef) ? a : b));
    }
    return valid.reduce((a, b) => (this.jointDistance(a.q, qRef) <= this.jointDistance(b.q, qRef) ? a : b));
  }

  jointDistance(a, b) {
    // Pondération : les axes proximaux « coûtent » plus cher à déplacer.
    const w = [1.5, 1.4, 1.2, 0.8, 0.7, 0.5];
    let s = 0;
    for (let i = 0; i < a.length; i++) s += (w[i] ?? 1) * (a[i] - b[i]) ** 2;
    return Math.sqrt(s);
  }

  // ---------------------------------------------------------------- IK numérique

  /** Erreur de pose pondérée [ex ey ez, w·rx w·ry w·rz]. */
  poseError(target, T, wOri) {
    const p = mat4Pos(T), pd = mat4Pos(target);
    const r = rotError(mat4Rot(target), mat4Rot(T));
    return {
      e: [pd[0] - p[0], pd[1] - p[1], pd[2] - p[2], r[0] * wOri, r[1] * wOri, r[2] * wOri],
      ep: Math.hypot(pd[0] - p[0], pd[1] - p[1], pd[2] - p[2]),
      er: norm(r),
    };
  }

  /** Moindres carrés amortis (Levenberg-Marquardt adaptatif) avec butées. */
  ikNumeric(target, seed, opts = {}) {
    const o = { ...this.ik, ...opts };
    const wOri = o.orientationWeight ?? 0.15;
    const maxIter = o.maxIter ?? 150;
    const tolPos = o.tolPos ?? 1e-5, tolRot = o.tolRot ?? 1e-4;
    const positionOnly = !!o.positionOnly;
    let q = this.clampToLimits(seed.slice());
    let lambda = o.damping ?? 0.02;
    let { e, ep, er } = this.poseError(target, this.tcp(q), wOri);
    if (positionOnly) { e = e.slice(0, 3); er = 0; }
    let cost = e.reduce((s, v) => s + v * v, 0);
    let it = 0;
    for (; it < maxIter; it++) {
      if (ep < tolPos && er < tolRot) break;
      const f = this.fk(q);
      let J = this.jacobian(q, f);
      if (positionOnly) J = J.slice(0, 3);
      else for (let r = 3; r < 6; r++) J[r] = J[r].map((v) => v * wOri);
      const dq = dampedLeastSquares(J, e, lambda);
      if (!dq) break;
      // Limitation du pas
      const mx = Math.max(...dq.map(Math.abs));
      const sc = mx > 0.35 ? 0.35 / mx : 1;
      const qn = this.clampToLimits(q.map((v, i) => v + dq[i] * sc));
      const res = this.poseError(target, this.tcp(qn), wOri);
      let en = res.e;
      if (positionOnly) en = en.slice(0, 3);
      const costN = en.reduce((s, v) => s + v * v, 0);
      if (costN < cost) {
        q = qn; e = en; cost = costN; ep = res.ep; er = positionOnly ? 0 : res.er;
        lambda = Math.max(lambda * 0.5, 1e-6);
      } else {
        lambda = Math.min(lambda * 3, 10);
      }
    }
    return { q, ok: ep < tolPos * 10 && er < tolRot * 10, ep, er, iterations: it };
  }

  /**
   * Cinématique inverse haut niveau.
   * options : { seed, preference, method, restarts }
   * Retourne { ok, q, method, solutions?, ep, er, message }
   */
  solveIK(target, options = {}) {
    const seed = options.seed || new Array(this.n).fill(0);
    const method = options.method || this.ik.method || 'auto';
    const preference = options.preference || this.ik.configuration || 'closest';
    if ((method === 'auto' || method === 'analytic') && this.analyticInfo.ok) {
      const sols = this.ikAnalyticAll(target, seed);
      const pick = this.pickSolution(sols, seed, preference);
      if (pick) {
        // Vérification (et affinage numérique éventuel)
        const chk = this.poseError(target, this.tcp(pick.q), 1);
        if (chk.ep < 1e-6 && chk.er < 1e-6) {
          return { ok: true, q: pick.q, method: 'analytic', solutions: sols, config: pick.config, ep: chk.ep, er: chk.er };
        }
        const ref = this.ikNumeric(target, pick.q);
        if (ref.ok) return { ok: true, q: ref.q, method: 'analytic+numeric', solutions: sols, config: pick.config, ep: ref.ep, er: ref.er };
      }
      if (method === 'analytic') {
        const why = sols.length ? 'toutes les solutions sont hors butées' : 'cible hors d’atteinte';
        return { ok: false, q: seed, method: 'analytic', solutions: sols, message: why };
      }
    }
    let res = this.ikNumeric(target, seed, options);
    if (!res.ok) {
      const rand = rng(options.randomSeed ?? 7);
      const restarts = options.restarts ?? 12;
      let best = res;
      for (let r = 0; r < restarts && !res.ok; r++) {
        const s = this.limits.map((l, i) => {
          const lo = Math.max(l.min, -Math.PI), hi = Math.min(l.max, Math.PI);
          return r < 3 ? seed[i] + (rand() - 0.5) * 0.6 : lo + rand() * (hi - lo);
        });
        res = this.ikNumeric(target, s, options);
        if (res.ep + res.er < best.ep + best.er) best = res;
      }
      if (!res.ok) res = best;
    }
    return {
      ok: res.ok, q: res.q, method: 'numeric', ep: res.ep, er: res.er, iterations: res.iterations,
      message: res.ok ? '' : `convergence impossible (erreur ${(res.ep * 1000).toFixed(2)} mm / ${(res.er * 180 / Math.PI).toFixed(2)}°)`,
    };
  }

  // ---------------------------------------------------------------- espace de travail

  /** Échantillonne l’espace de travail (Monte-Carlo). Retourne positions TCP et manipulabilité normalisée. */
  sampleWorkspace(count = 20000, seed = 1) {
    const rand = rng(seed);
    const pts = new Float32Array(count * 3);
    const manip = new Float32Array(count);
    let wMax = 1e-12;
    const stats = { reachMax: 0, zMax: -Infinity, zMin: Infinity };
    const q = new Array(this.n).fill(0);
    for (let s = 0; s < count; s++) {
      for (let i = 0; i < this.n; i++) {
        const l = this.limits[i];
        q[i] = l.min + rand() * (l.max - l.min);
      }
      const f = this.fk(q);
      const p = mat4Pos(f.tcp);
      pts[s * 3] = p[0]; pts[s * 3 + 1] = p[1]; pts[s * 3 + 2] = p[2];
      const J = this.jacobian(q, f);
      const w = Math.abs(det(J));
      manip[s] = w;
      if (w > wMax) wMax = w;
      const bp = mat4Pos(mat4Mul(this.baseInv, f.tcp));
      stats.reachMax = Math.max(stats.reachMax, Math.hypot(bp[0], bp[1]));
      stats.zMax = Math.max(stats.zMax, bp[2]);
      stats.zMin = Math.min(stats.zMin, bp[2]);
    }
    for (let s = 0; s < count; s++) manip[s] /= wMax;
    return { points: pts, manipulability: manip, stats };
  }
}

/** Pose TCP → vecteur [x, y, z, rx, ry, rz] (m, rad RPY) pratique pour l’affichage. */
export function tcpVector(T) {
  const R = mat4Rot(T);
  const sp = -R[6];
  let roll, pitch, yaw;
  if (Math.abs(sp) > 1 - 1e-10) {
    pitch = Math.sign(sp) * Math.PI / 2; roll = 0; yaw = Math.atan2(-R[1], R[4]);
  } else {
    roll = Math.atan2(R[7], R[8]); pitch = Math.asin(sp); yaw = Math.atan2(R[3], R[0]);
  }
  return [T[3], T[7], T[11], roll, pitch, yaw];
}

export { mat4FromRotPos, mat4Identity };
