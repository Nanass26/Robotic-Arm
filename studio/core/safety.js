// ORION-6 — Modèle de collision simplifié (capsules) : auto-collision et collision avec la table.
// Chaque capsule est un segment [p0, p1] de rayon r, exprimé dans le repère DH de son segment
// (link 0 = base fixe). Si aucune capsule n’est fournie, elles sont déduites de la table DH.

import { mat4TransformPoint, mat4Pos, mat4Axis, add, scale, sub, dot } from './math3d.js';

/** Capsules déduites de la table DH (valable pour tout robot). */
export function capsulesFromDH(kin, radii) {
  const caps = [];
  const f = kin.fk(new Array(kin.n).fill(0));
  const invs = f.frames.map((F) => F);
  for (let i = 1; i <= kin.n; i++) {
    const j = kin.joints[i - 1];
    const r = radii?.[i - 1] ?? 0.04;
    if (kin.convention === 'MDH') {
      // Repère i sur l’axe i : le segment i s’étend de pᵢ vers pᵢ₊₁ (paramètres de la ligne suivante)
      const next = kin.joints[i];
      if (next) {
        // Dans le repère i : Rx(α) Tx(a) puis Tz(d) → points (0,0,0) → (a,0,0) → (a, −sα·d, cα·d)
        const a = next.a, al = next.alpha, d = next.d;
        const p1 = [a, 0, 0];
        const p2 = [a, -Math.sin(al) * d, Math.cos(al) * d];
        if (Math.abs(a) > 1e-6) caps.push({ link: i, p0: [0, 0, 0], p1, r });
        if (Math.abs(d) > 1e-6) caps.push({ link: i, p0: p1, p1: p2, r });
      }
      continue;
    }
    // DH standard, dans le repère i : origine pᵢ ; pᵢ₋₁ = −a·x − d·(z_{i−1} exprimé dans i)
    // z_{i−1} dans le repère i = Rx(−α)·ẑ = (0, sin α, cos α)
    const a = j.a, al = j.alpha, d = j.d;
    const m = [-a, 0, 0];
    const p0 = [-a, -Math.sin(al) * d, -Math.cos(al) * d];
    if (Math.abs(d) > 1e-6) caps.push({ link: i, p0, p1: m, r });
    if (Math.abs(a) > 1e-6) caps.push({ link: i, p0: m, p1: [0, 0, 0], r });
    if (Math.abs(a) < 1e-6 && Math.abs(d) < 1e-6) caps.push({ link: i, p0: [0, 0, 0], p1: [0, 0, 0], r: r * 0.8 });
  }
  void invs;
  return { capsules: caps, ignorePairs: [] };
}

/** Distance minimale entre deux segments [p1,q1] et [p2,q2] (Ericson, RTCD §5.1.9). */
export function segmentDistance(p1, q1, p2, q2) {
  const d1 = sub(q1, p1), d2 = sub(q2, p2), r = sub(p1, p2);
  const a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  let s, t;
  const EPS = 1e-12;
  if (a <= EPS && e <= EPS) return { dist: Math.sqrt(dot(r, r)), c1: p1, c2: p2 };
  if (a <= EPS) { s = 0; t = Math.min(1, Math.max(0, f / e)); } else {
    const c = dot(d1, r);
    if (e <= EPS) { t = 0; s = Math.min(1, Math.max(0, -c / a)); } else {
      const b = dot(d1, d2), den = a * e - b * b;
      s = den !== 0 ? Math.min(1, Math.max(0, (b * f - c * e) / den)) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = Math.min(1, Math.max(0, -c / a)); } else if (t > 1) { t = 1; s = Math.min(1, Math.max(0, (b - c) / a)); }
    }
  }
  const c1 = add(p1, scale(d1, s)), c2 = add(p2, scale(d2, t));
  const w = sub(c1, c2);
  return { dist: Math.sqrt(dot(w, w)), c1, c2 };
}

export class CollisionModel {
  constructor(kin, params) {
    this.setParams(kin, params);
  }

  setParams(kin, params) {
    this.kin = kin;
    const s = params.safety || {};
    const set = s.capsules && s.capsules.capsules?.length ? { ...s.capsules, explicit: true } : capsulesFromDH(kin, s.linkRadii);
    this.capsules = set.capsules.map((c) => ({ ...c }));
    this.ignore = new Set((set.ignorePairs || []).map(([a, b]) => `${Math.min(a, b)}-${Math.max(a, b)}`));
    this.margin = s.capsuleMargin ?? 0.004;
    this.floorZ = params.limits.floorZ ?? 0;
    this.selfOn = s.selfCollision !== false;
    this.floorOn = s.floorCollision !== false;
    this.toolCapsule = null;
    const tool = params.kinematics.tool;
    const toolLen = Math.hypot(tool.x, tool.y, tool.z);
    if (toolLen > 0.01 && params.gripper?.type !== 'none') {
      // Outil : de la bride au TCP (repère n)
      this.toolCapsule = { link: kin.n, p0: [0, 0, 0.01], p1: [tool.x * 0.85, tool.y * 0.85, tool.z * 0.85], r: 0.025, tool: true };
    }
    this.autoIgnore(params);
  }

  /**
   * Paires de capsules toujours en contact (approximation de la géométrie) : ignorées
   * individuellement, comme MoveIt le fait pour les paires « Adjacent/Always/Default ».
   * Critère : contact dans la pose zéro (réputée sans collision, vérifiée sur la CAO)
   * ou dans plus de 60 % des configurations tirées au hasard.
   */
  autoIgnore(params, samples = 120) {
    this.ignoreCaps = new Set();
    const selfOn = this.selfOn;
    this.selfOn = true;
    const hits = new Map();
    const collect = (q) => this.pairsInContact(q).forEach((k) => hits.set(k, (hits.get(k) || 0) + 1));
    this.pairsInContact(new Array(this.kin.n).fill(0)).forEach((k) => this.ignoreCaps.add(k));
    let seed = 99;
    const rand = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
    const lim = params.limits.joints;
    for (let k = 0; k < samples; k++) collect(lim.map((l) => l.min + rand() * (l.max - l.min)));
    for (const [k, c] of hits) if (c > 0.6 * samples) this.ignoreCaps.add(k);
    this.selfOn = selfOn;
  }

  pairsInContact(q) {
    const caps = this.worldCapsules(q);
    const out = [];
    for (let i = 0; i < caps.length; i++) {
      for (let j = i + 1; j < caps.length; j++) {
        const A = caps[i], B = caps[j];
        if (Math.abs(A.link - B.link) <= 1) continue;
        const { dist } = segmentDistance(A.w0, A.w1, B.w0, B.w1);
        if (dist - A.r - B.r < this.margin) out.push(`${i}-${j}`);
      }
    }
    return out;
  }

  /** Capsules transformées dans le monde pour la configuration q. */
  worldCapsules(q, fk = null) {
    const f = fk || this.kin.fk(q);
    const all = this.toolCapsule ? [...this.capsules, this.toolCapsule] : this.capsules;
    return all.map((c) => {
      const F = f.frames[c.link];
      return { ...c, w0: mat4TransformPoint(F, c.p0), w1: mat4TransformPoint(F, c.p1) };
    });
  }

  /**
   * Vérifie la configuration q.
   * Retourne { ok, self: [{a, b, dist}], floor: [{link, z}], minDist }.
   */
  check(q, fk = null) {
    const caps = this.worldCapsules(q, fk);
    const res = { ok: true, self: [], floor: [], minDist: Infinity, caps };
    if (this.selfOn) {
      for (let i = 0; i < caps.length; i++) {
        for (let j = i + 1; j < caps.length; j++) {
          const A = caps[i], B = caps[j];
          if (Math.abs(A.link - B.link) <= 1) continue;
          const key = `${Math.min(A.link, B.link)}-${Math.max(A.link, B.link)}`;
          if (this.ignore.has(key) || this.ignoreCaps?.has(`${i}-${j}`)) continue;
          const { dist } = segmentDistance(A.w0, A.w1, B.w0, B.w1);
          const clearance = dist - A.r - B.r;
          res.minDist = Math.min(res.minDist, clearance);
          if (clearance < this.margin) { res.ok = false; res.self.push({ a: A.link, b: B.link, dist: clearance }); }
        }
      }
    }
    if (this.floorOn) {
      for (const c of caps) {
        if (c.link <= 1 && !c.tool) continue; // la base et la tourelle reposent sur la table
        const zmin = Math.min(c.w0[2], c.w1[2]) - c.r;
        if (zmin < this.floorZ - 1e-4) { res.ok = false; res.floor.push({ link: c.link, z: zmin }); }
      }
    }
    return res;
  }

  /** Vérifie une trajectoire échantillonnée ; retourne le premier conflit trouvé. */
  checkPath(samplesQ) {
    for (let k = 0; k < samplesQ.length; k++) {
      const r = this.check(samplesQ[k]);
      if (!r.ok) return { ok: false, index: k, detail: r };
    }
    return { ok: true };
  }
}

export { mat4Pos, mat4Axis };
