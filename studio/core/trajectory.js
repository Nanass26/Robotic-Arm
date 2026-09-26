// ORION-6 — Génération de trajectoires.
//   • Profils scalaires repos→repos : trapèze, courbe en S (7 phases, Biagiotti & Melchiorri),
//     quintique (jerk minimal), cycloïdal.
//   • MoveJ : interpolation articulaire synchronisée (tous les axes arrivent ensemble).
//   • MoveL / MoveC : trajectoire cartésienne (ligne / arc de cercle) + SLERP d’orientation,
//     discrétisée par IK puis re-temporisée si une vitesse/accélération articulaire est dépassée.
//   • Lissage (« fly-by ») par superposition de segments consécutifs.

import {
  mat4Pos, mat4Rot, mat4FromRotPos, mat3ToQuat, quatToMat3, quatSlerp, rotAngleBetween,
  sub, add, scale, cross, norm, normalize, dot, dist,
} from './math3d.js';

// ------------------------------------------------------------------ profils scalaires

export class Profile {
  constructor(T) { this.T = T; }
  /** Retourne [position, vitesse, accélération] à l’instant t (bornée à [0, T]). */
  eval() { return [0, 0, 0]; }
}

class ZeroProfile extends Profile {
  constructor(L) { super(0); this.L = L; }
  eval() { return [this.L, 0, 0]; }
}

export class TrapezoidProfile extends Profile {
  constructor(L, v, a) {
    let ta, tc, vp;
    if (L <= (v * v) / a) { ta = Math.sqrt(L / a); tc = 0; vp = a * ta; } else { ta = v / a; tc = (L - v * ta) / v; vp = v; }
    super(2 * ta + tc);
    Object.assign(this, { L, a, ta, tc, vp });
    this.accelTime = ta; this.decelTime = ta;
  }
  eval(t) {
    const { L, a, ta, tc, vp, T } = this;
    if (t <= 0) return [0, 0, 0];
    if (t >= T) return [L, 0, 0];
    if (t < ta) return [0.5 * a * t * t, a * t, a];
    if (t < ta + tc) { const d = 0.5 * a * ta * ta; return [d + vp * (t - ta), vp, 0]; }
    const r = T - t;
    return [L - 0.5 * a * r * r, a * r, -a];
  }
}

export class SCurveProfile extends Profile {
  constructor(L, vmax, amax, jmax) {
    let Tj, Ta, Tv;
    // Phase d’accélération pour atteindre vmax
    if (vmax * jmax >= amax * amax) { Tj = amax / jmax; Ta = Tj + vmax / amax; } else { Tj = Math.sqrt(vmax / jmax); Ta = 2 * Tj; }
    Tv = L / vmax - Ta;
    if (Tv < 0) {
      // vmax non atteinte
      Tv = 0;
      if (L < (2 * amax ** 3) / (jmax * jmax)) {
        Tj = Math.cbrt(L / (2 * jmax)); Ta = 2 * Tj;
      } else {
        Tj = amax / jmax; Ta = (Tj + Math.sqrt(Tj * Tj + (4 * L) / amax)) / 2;
      }
    }
    const alim = jmax * Tj;
    const vlim = alim * (Ta - Tj);
    super(2 * Ta + Tv);
    Object.assign(this, { L, jmax, Tj, Ta, Tv, alim, vlim });
    this.accelTime = Ta; this.decelTime = Ta;
  }
  accPhase(t) {
    const { jmax, Tj, Ta, alim, vlim } = this;
    if (t < Tj) return [(jmax * t ** 3) / 6, (jmax * t * t) / 2, jmax * t];
    if (t < Ta - Tj) return [(alim / 6) * (3 * t * t - 3 * Tj * t + Tj * Tj), alim * (t - Tj / 2), alim];
    const r = Ta - t;
    return [(vlim * Ta) / 2 - vlim * r + (jmax * r ** 3) / 6, vlim - (jmax * r * r) / 2, jmax * r];
  }
  eval(t) {
    const { L, Ta, Tv, vlim, T } = this;
    if (t <= 0) return [0, 0, 0];
    if (t >= T) return [L, 0, 0];
    if (t < Ta) return this.accPhase(t);
    if (t < Ta + Tv) return [(vlim * Ta) / 2 + vlim * (t - Ta), vlim, 0];
    const [p, v, a] = this.accPhase(T - t);
    return [L - p, v, -a];
  }
}

export class QuinticProfile extends Profile {
  constructor(L, v, a, j) {
    super(Math.max((1.875 * L) / v, Math.sqrt((5.7735027 * L) / a), Math.cbrt((60 * L) / j)));
    this.L = L;
    this.accelTime = this.T / 2; this.decelTime = this.T / 2;
  }
  eval(t) {
    const { L, T } = this;
    if (t <= 0) return [0, 0, 0];
    if (t >= T) return [L, 0, 0];
    const s = t / T;
    return [
      L * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5),
      (L / T) * (30 * s * s - 60 * s ** 3 + 30 * s ** 4),
      (L / (T * T)) * (60 * s - 180 * s * s + 120 * s ** 3),
    ];
  }
}

export class CycloidalProfile extends Profile {
  constructor(L, v, a, j) {
    super(Math.max((2 * L) / v, Math.sqrt((2 * Math.PI * L) / a), Math.cbrt((4 * Math.PI * Math.PI * L) / j)));
    this.L = L;
    this.accelTime = this.T / 2; this.decelTime = this.T / 2;
  }
  eval(t) {
    const { L, T } = this;
    if (t <= 0) return [0, 0, 0];
    if (t >= T) return [L, 0, 0];
    const s = t / T, w = 2 * Math.PI;
    return [L * (s - Math.sin(w * s) / w), (L / T) * (1 - Math.cos(w * s)), (L / (T * T)) * w * Math.sin(w * s)];
  }
}

/** Fabrique de profil scalaire (L ≥ 0). */
export function makeProfile(type, L, v, a, j) {
  if (!(L > 1e-12)) return new ZeroProfile(Math.max(L, 0));
  v = Math.max(v, 1e-9); a = Math.max(a, 1e-9); j = Math.max(j, 1e-9);
  switch (type) {
    case 'trapezoid': return new TrapezoidProfile(L, v, a);
    case 'quintic': return new QuinticProfile(L, v, a, j);
    case 'cycloidal': return new CycloidalProfile(L, v, a, j);
    default: return new SCurveProfile(L, v, a, j);
  }
}

// ------------------------------------------------------------------ segments

let SEG_ID = 1;

/** Segment de base : sample(t) → { q, qd, qdd }. */
export class Segment {
  constructor(kind, q0, q1, T) {
    this.id = SEG_ID++;
    this.kind = kind;
    this.q0 = q0.slice();
    this.q1 = q1.slice();
    this.T = T;
    this.accelTime = T / 2;
    this.decelTime = T / 2;
    this.warnings = [];
  }
}

/** Mouvement articulaire synchronisé. */
export class JointSegment extends Segment {
  constructor(q0, q1, limits, opts = {}) {
    const dq = q1.map((v, i) => v - q0[i]);
    const vs = opts.speed ?? 1, as = opts.accel ?? 1;
    let v = Infinity, a = Infinity, j = Infinity;
    dq.forEach((d, i) => {
      const ad = Math.abs(d);
      if (ad < 1e-12) return;
      v = Math.min(v, (limits[i].vmax * vs) / ad);
      a = Math.min(a, (limits[i].amax * as) / ad);
      j = Math.min(j, (limits[i].jmax * as) / ad);
    });
    const prof = Number.isFinite(v) ? makeProfile(opts.profile || 'scurve', 1, v, a, j) : makeProfile('scurve', 0, 1, 1, 1);
    if (opts.minTime && prof.T < opts.minTime) {
      // Temps imposé (ex. synchronisation) : on étire la loi par mise à l’échelle temporelle
      prof.stretch = opts.minTime / prof.T;
    }
    const k = prof.stretch || 1;
    super('joint', q0, q1, prof.T * k);
    this.dq = dq;
    this.prof = prof;
    this.k = k;
    this.accelTime = (prof.accelTime ?? prof.T / 2) * k;
    this.decelTime = (prof.decelTime ?? prof.T / 2) * k;
  }
  sample(t) {
    const [s, sd, sdd] = this.prof.eval(t / this.k);
    const k = this.k;
    return {
      q: this.q0.map((v, i) => v + this.dq[i] * s),
      qd: this.dq.map((d) => (d * sd) / k),
      qdd: this.dq.map((d) => (d * sdd) / (k * k)),
    };
  }
}

/** Freinage indépendant de chaque axe à décélération maximale (arrêt de catégorie 1/2). */
export class BrakeSegment extends Segment {
  constructor(q0, v0, limits) {
    const tStop = v0.map((v, i) => Math.abs(v) / limits[i].amax);
    const qStop = q0.map((q, i) => q + (v0[i] * tStop[i]) / 2);
    super('brake', q0, qStop, Math.max(0, ...tStop));
    this.v0 = v0.slice();
    this.tStop = tStop;
    this.a = limits.map((l) => l.amax);
    this.accelTime = 0; this.decelTime = this.T;
  }
  sample(t) {
    const q = [], qd = [], qdd = [];
    for (let i = 0; i < this.q0.length; i++) {
      const ti = Math.min(t, this.tStop[i]);
      const s = -Math.sign(this.v0[i]) * this.a[i];
      q.push(this.q0[i] + this.v0[i] * ti + 0.5 * s * ti * ti);
      qd.push(t < this.tStop[i] ? this.v0[i] + s * t : 0);
      qdd.push(t < this.tStop[i] ? s : 0);
    }
    return { q, qd, qdd };
  }
}

/** Attente (maintien de position). */
export class HoldSegment extends Segment {
  constructor(q, T) { super('hold', q, q, T); }
  sample() { return { q: this.q0.slice(), qd: this.q0.map(() => 0), qdd: this.q0.map(() => 0) }; }
}

/**
 * Chemin cartésien générique : pose(s) pour s ∈ [0, 1] + longueurs (linéaire, angulaire).
 */
export function linePath(T0, T1) {
  const p0 = mat4Pos(T0), p1 = mat4Pos(T1);
  const q0 = mat3ToQuat(mat4Rot(T0)), q1 = mat3ToQuat(mat4Rot(T1));
  return {
    kind: 'line',
    length: dist(p0, p1),
    angle: rotAngleBetween(mat4Rot(T0), mat4Rot(T1)),
    pose(s) {
      const p = [p0[0] + (p1[0] - p0[0]) * s, p0[1] + (p1[1] - p0[1]) * s, p0[2] + (p1[2] - p0[2]) * s];
      return mat4FromRotPos(quatToMat3(quatSlerp(q0, q1, s)), p);
    },
  };
}

/** Arc de cercle passant par trois points (départ, passage, arrivée). Repli sur une ligne si alignés. */
export function arcPath(T0, Tvia, T1) {
  const A = mat4Pos(T0), B = mat4Pos(Tvia), C = mat4Pos(T1);
  const ab = sub(B, A), ac = sub(C, A);
  const n = cross(ab, ac);
  const nn = norm(n);
  if (nn < 1e-9) return linePath(T0, T1);
  // Centre du cercle circonscrit
  const abxac = n;
  const t1 = scale(cross(abxac, ab), dot(ac, ac));
  const t2 = scale(cross(ac, abxac), dot(ab, ab));
  const center = add(A, scale(add(t1, t2), 1 / (2 * nn * nn)));
  const r = dist(center, A);
  const u = normalize(sub(A, center));
  const nz = normalize(n);
  const w = cross(nz, u);
  const angleOf = (P) => {
    const d = sub(P, center);
    let a = Math.atan2(dot(d, w), dot(d, u));
    if (a < 0) a += 2 * Math.PI;
    return a;
  };
  const aC = angleOf(C);
  const q0 = mat3ToQuat(mat4Rot(T0)), q1 = mat3ToQuat(mat4Rot(T1));
  return {
    kind: 'arc',
    center, radius: r, normal: nz,
    length: r * aC,
    angle: rotAngleBetween(mat4Rot(T0), mat4Rot(T1)),
    sweep: aC,
    pose(s) {
      const phi = aC * s;
      const p = add(center, add(scale(u, r * Math.cos(phi)), scale(w, r * Math.sin(phi))));
      return mat4FromRotPos(quatToMat3(quatSlerp(q0, q1, s)), p);
    },
  };
}

/**
 * Segment cartésien : discrétisé par IK à `sampleHz`, re-temporisé si nécessaire.
 * `kin` : modèle Kinematics ; `q0` : configuration de départ (graine IK).
 */
export class CartesianSegment extends Segment {
  constructor(kin, path, q0, params, opts = {}) {
    const lim = params.limits.cartesian;
    const vs = opts.speedAbs ? 1 : (opts.speed ?? 1);
    const as = opts.accel ?? 1;
    const vlin = opts.speedAbs ?? lim.vmax * vs;
    const L = Math.max(path.length, 0), A = Math.max(path.angle, 0);
    let v = Infinity, a = Infinity, j = Infinity;
    if (L > 1e-9) { v = Math.min(v, vlin / L); a = Math.min(a, (lim.amax * as) / L); j = Math.min(j, (lim.jmax * as) / L); }
    if (A > 1e-9) {
      v = Math.min(v, (lim.wmax * (opts.speedAbs ? 1 : vs)) / A);
      a = Math.min(a, (lim.alphamax * as) / A);
      j = Math.min(j, (lim.alphamax * 10 * as) / A);
    }
    const prof = Number.isFinite(v) ? makeProfile(opts.profile || 'scurve', 1, v, a, j) : makeProfile('scurve', 0, 1, 1, 1);
    super(path.kind === 'arc' ? 'arc' : 'line', q0, q0, prof.T);
    this.path = path;
    this.prof = prof;
    this.kin = kin;
    this.accelTime = prof.accelTime ?? prof.T / 2;
    this.decelTime = prof.decelTime ?? prof.T / 2;
    this.ok = true;
    this.error = '';
    this.build(params, opts);
  }

  build(params, opts) {
    const hz = params.trajectory.sampleHz || 250;
    const kin = this.kin;
    const jl = params.limits.joints;
    let stretch = 1;
    for (let pass = 0; pass < 3; pass++) {
      const T = this.prof.T * stretch;
      const N = Math.max(2, Math.ceil(T * hz) + 1);
      const times = new Float64Array(N);
      const Q = [];
      let q = this.q0.slice();
      for (let k = 0; k < N; k++) {
        const t = (k / (N - 1)) * T;
        times[k] = t;
        const [s] = this.prof.eval(t / stretch);
        const target = this.path.pose(s);
        const r = kin.solveIK(target, { seed: q, method: kin.analyticInfo.ok ? 'auto' : 'numeric', preference: 'closest', restarts: 0 });
        if (!r.ok || kin.jointDistance(r.q, q) > 0.5) {
          // Continuité : un saut de configuration signale une singularité / sortie d’espace de travail
          const rn = kin.ikNumeric(target, q);
          if (!rn.ok) {
            this.ok = false;
            this.error = `point inatteignable à ${(s * 100).toFixed(0)} % du trajet`;
            Q.push(q.slice());
            continue;
          }
          q = rn.q;
        } else {
          q = r.q;
        }
        Q.push(q.slice());
      }
      // Vitesses / accélérations par différences finies centrées
      const n = this.q0.length;
      const QD = Q.map(() => new Array(n).fill(0));
      const QDD = Q.map(() => new Array(n).fill(0));
      const dt = times[1] - times[0];
      for (let k = 0; k < N; k++) {
        for (let i = 0; i < n; i++) {
          if (k > 0 && k < N - 1) {
            QD[k][i] = (Q[k + 1][i] - Q[k - 1][i]) / (2 * dt);
            QDD[k][i] = (Q[k + 1][i] - 2 * Q[k][i] + Q[k - 1][i]) / (dt * dt);
          }
        }
      }
      // Rapport de dépassement des limites articulaires
      let ratioV = 1, ratioA = 1;
      for (let k = 0; k < N; k++) {
        for (let i = 0; i < n; i++) {
          ratioV = Math.max(ratioV, Math.abs(QD[k][i]) / jl[i].vmax);
          ratioA = Math.max(ratioA, Math.abs(QDD[k][i]) / (jl[i].amax * 1.5));
        }
      }
      this.samples = { times, Q, QD, QDD, dt };
      this.T = T;
      this.q1 = Q[N - 1].slice();
      this.stretch = stretch;
      const need = Math.max(ratioV, Math.sqrt(ratioA));
      if (need <= 1.02 || !this.ok) break;
      stretch *= need * 1.05;
      this.warnings.push(`trajectoire ralentie ×${need.toFixed(2)} (limites articulaires, proximité de singularité)`);
    }
    this.accelTime *= this.stretch;
    this.decelTime *= this.stretch;
  }

  sample(t) {
    const { times, Q, QD, QDD, dt } = this.samples;
    const N = times.length;
    if (t <= 0) return { q: Q[0].slice(), qd: Q[0].map(() => 0), qdd: QDD[0].slice() };
    if (t >= this.T) return { q: Q[N - 1].slice(), qd: Q[0].map(() => 0), qdd: Q[0].map(() => 0) };
    const x = t / dt;
    const k = Math.min(N - 2, Math.floor(x));
    const f = x - k;
    // Interpolation d’Hermite cubique (continuité C¹)
    const h00 = 2 * f ** 3 - 3 * f * f + 1, h10 = f ** 3 - 2 * f * f + f, h01 = -2 * f ** 3 + 3 * f * f, h11 = f ** 3 - f * f;
    const q = Q[k].map((v, i) => h00 * v + h10 * dt * QD[k][i] + h01 * Q[k + 1][i] + h11 * dt * QD[k + 1][i]);
    const qd = QD[k].map((v, i) => v + (QD[k + 1][i] - v) * f);
    const qdd = QDD[k].map((v, i) => v + (QDD[k + 1][i] - v) * f);
    return { q, qd, qdd };
  }
}

// ------------------------------------------------------------------ exécution avec lissage

/**
 * File de segments exécutés dans le temps, avec lissage optionnel par superposition.
 * push(segment, blendTime) : `blendTime` = recouvrement avec le segment précédent (s).
 */
export class TrajectoryExecutor {
  constructor(n) {
    this.n = n;
    this.clear();
  }
  clear() {
    this.items = []; // { seg, start }
    this.endTime = 0;
    this.time = 0;
    this.last = null;
  }
  get busy() { return this.items.length > 0; }
  get remaining() { return Math.max(0, this.endTime - this.time); }

  push(seg, blend = 0, now = this.time) {
    let start = Math.max(this.endTime, now);
    if (blend > 0 && this.items.length) {
      const prev = this.items[this.items.length - 1];
      const ov = Math.min(blend, prev.seg.decelTime, seg.accelTime, prev.seg.T * 0.5, seg.T * 0.5);
      start = Math.max(prev.start + prev.seg.T - ov, now);
    }
    this.items.push({ seg, start });
    this.endTime = Math.max(this.endTime, start + seg.T);
    return start;
  }

  /** Échantillonne la consigne à l’instant t (absolu). Retourne null si rien à exécuter. */
  sample(t) {
    this.time = t;
    if (!this.items.length) return null;
    // Retire les segments terminés (sauf le dernier qui porte la position finale)
    while (this.items.length > 1 && t >= this.items[0].start + this.items[0].seg.T && t >= this.items[1].start) {
      this.items.shift();
    }
    const first = this.items[0];
    const s0 = first.seg.sample(t - first.start);
    const q = s0.q.slice(), qd = s0.qd.slice(), qdd = s0.qdd.slice();
    for (let k = 1; k < this.items.length; k++) {
      const it = this.items[k];
      if (t < it.start) break;
      const s = it.seg.sample(t - it.start);
      for (let i = 0; i < this.n; i++) {
        q[i] += s.q[i] - it.seg.q0[i];
        qd[i] += s.qd[i];
        qdd[i] += s.qdd[i];
      }
    }
    const done = t >= this.endTime;
    const active = this.items.find((it) => t >= it.start && t < it.start + it.seg.T) || this.items[this.items.length - 1];
    this.last = { q, qd, qdd, done, segId: active.seg.id, tag: active.seg.tag };
    if (done) this.items = [];
    return this.last;
  }
}

/** Temps de recouvrement correspondant à une zone de lissage (distance TCP) pour un segment. */
export function blendTimeForZone(kin, seg, zone) {
  if (!(zone > 0) || !seg || seg.T <= 0) return 0;
  const pEnd = mat4Pos(kin.tcp(seg.q1));
  // Recherche par dichotomie du dernier instant où la distance restante > zone
  let lo = 0, hi = seg.T;
  if (dist(mat4Pos(kin.tcp(seg.sample(0).q)), pEnd) <= zone) return seg.T * 0.5;
  for (let it = 0; it < 24; it++) {
    const mid = (lo + hi) / 2;
    const d = dist(mat4Pos(kin.tcp(seg.sample(mid).q)), pEnd);
    if (d > zone) lo = mid; else hi = mid;
  }
  return seg.T - hi;
}
