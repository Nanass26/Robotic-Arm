// ORION-6 — Modèle dynamique : Newton-Euler récursif (RNEA), matrice d’inertie, dynamique directe.
// Formulation dans le repère monde, valable pour les conventions DH standard et modifiée.
//
//   τ = M(q)·q̈ + C(q, q̇)·q̇ + g(q)        (côté articulation, sans actionneurs)
//
// Les paramètres inertiels du segment i sont exprimés dans le repère DH i.

import {
  add, sub, scale, cross, dot, mat3MulVec, mat4Rot, mat4Pos, mat4TransformPoint, mat3Mul,
  inertiaFromVector, rotateInertia, mat4Mul, poseToMat4,
} from './math3d.js';
import { solveSPD } from './linalg.js';

export class Dynamics {
  /**
   * @param {Kinematics} kin  modèle géométrique (fournit fk/axes)
   * @param {object} params   configuration complète
   * @param {object} opts     { massScale: facteur appliqué aux masses (test de robustesse) }
   */
  constructor(kin, params, opts = {}) {
    this.kin = kin;
    this.setParams(params, opts);
  }

  setParams(params, opts = {}) {
    const d = params.dynamics;
    const s = opts.massScale ?? 1;
    this.n = this.kin.n;
    this.gravity = d.gravity.slice();
    this.links = d.links.map((l) => ({
      mass: l.mass * s,
      com: l.com.slice(),
      I: inertiaFromVector(l.inertia).map((v) => v * s),
    }));
    // Outil (repère bride) + charge (repère TCP) rattachés au dernier segment.
    const tool = d.tool || { mass: 0, com: [0, 0, 0], inertiaDiag: [0, 0, 0] };
    this.toolBody = {
      mass: tool.mass * s,
      com: tool.com.slice(),
      I: [tool.inertiaDiag[0], 0, 0, 0, tool.inertiaDiag[1], 0, 0, 0, tool.inertiaDiag[2]].map((v) => v * s),
    };
    const pl = d.payload || { mass: 0, com: [0, 0, 0] };
    const Ttool = poseToMat4(params.kinematics.tool);
    this.payloadBody = { mass: pl.mass * s, com: mat4TransformPoint(Ttool, pl.com), I: [0, 0, 0, 0, 0, 0, 0, 0, 0] };
    this.payloadAttached = true;
    // Inertie des rotors ramenée en sortie : N²·J (« armature »)
    this.armature = params.actuators.joints.map((a) => a.gearRatio * a.gearRatio * a.rotorInertia * s);
  }

  /** Corps rigides attachés à chaque segment (le dernier porte outil + charge). */
  bodiesOf(i) {
    const b = [this.links[i]];
    if (i === this.n - 1) {
      if (this.toolBody.mass > 0) b.push(this.toolBody);
      if (this.payloadAttached && this.payloadBody.mass > 0) b.push(this.payloadBody);
    }
    return b;
  }

  /**
   * Newton-Euler récursif.
   * @param q, qd, qdd  positions, vitesses, accélérations articulaires
   * @param gravityOn   inclure la gravité
   * @param fk          résultat de kin.fk(q) (optionnel, pour réutilisation)
   * @param fExt        effort extérieur au TCP {f:[..], n:[..], p:[..] point d’application monde}
   * @param wantWrenches retourne aussi les efforts transmis par chaque articulation
   */
  rnea(q, qd, qdd, gravityOn = true, fk = null, fExt = null, wantWrenches = false) {
    const n = this.n;
    const f = fk || this.kin.fk(q);
    const joints = this.kin.joints;
    const g = gravityOn ? this.gravity : [0, 0, 0];
    // Données par segment
    const w = new Array(n), wd = new Array(n), a = new Array(n); // a = accélération de l’origine du repère i
    const origins = f.frames.map(mat4Pos);
    let wPrev = [0, 0, 0], wdPrev = [0, 0, 0];
    let aPrev = [-g[0], -g[1], -g[2]]; // astuce : accélérer la base de −g
    let pPrev = origins[0];
    const Fc = new Array(n), Nc = new Array(n), com = new Array(n);
    for (let i = 0; i < n; i++) {
      const { o, z } = f.axes[i];
      const dir = joints[i].direction < 0 ? -1 : 1;
      const qdi = qd[i] * dir, qddi = qdd[i] * dir;
      // Accélération du point de l’axe o (solidaire du segment i-1)
      const rpo = sub(o, pPrev);
      const aO = add(aPrev, add(cross(wdPrev, rpo), cross(wPrev, cross(wPrev, rpo))));
      let wi, wdi, aOi;
      if (joints[i].type === 'prismatic') {
        wi = wPrev; wdi = wdPrev;
        aOi = add(aO, add(scale(cross(wPrev, z), 2 * qdi), scale(z, qddi)));
      } else {
        wi = add(wPrev, scale(z, qdi));
        wdi = add(wdPrev, add(scale(z, qddi), cross(wPrev, scale(z, qdi))));
        aOi = aO;
      }
      // Accélération de l’origine du repère i (solidaire du segment i)
      const pi = origins[i + 1];
      const rop = sub(pi, o);
      const ai = add(aOi, add(cross(wdi, rop), cross(wi, cross(wi, rop))));
      w[i] = wi; wd[i] = wdi; a[i] = ai;
      // Efforts d’inertie des corps du segment i (réduits au point pi)
      const R = mat4Rot(f.frames[i + 1]);
      let F = [0, 0, 0], N = [0, 0, 0], mTot = 0, mc = [0, 0, 0];
      for (const b of this.bodiesOf(i)) {
        if (b.mass <= 0 && b.I.every((v) => v === 0)) continue;
        const c = mat4TransformPoint(f.frames[i + 1], b.com);
        const rc = sub(c, pi);
        const ac = add(ai, add(cross(wdi, rc), cross(wi, cross(wi, rc))));
        const Fi = scale(ac, b.mass);
        const Iw = rotateInertia(R, b.I);
        const Ni = add(mat3MulVec(Iw, wdi), cross(wi, mat3MulVec(Iw, wi)));
        F = add(F, Fi);
        // Moment au point pi : Ni + rc × Fi
        N = add(N, add(Ni, cross(rc, Fi)));
        mTot += b.mass; mc = add(mc, scale(c, b.mass));
      }
      Fc[i] = F; Nc[i] = N; com[i] = mTot > 0 ? scale(mc, 1 / mTot) : pi;
      wPrev = wi; wdPrev = wdi; aPrev = ai; pPrev = pi;
    }
    // Passe retour
    const tau = new Array(n).fill(0);
    const wrenches = wantWrenches ? new Array(n) : null;
    let fNext = [0, 0, 0], nNext = [0, 0, 0], pNext = null; // effort exercé par le segment i+1 sur i (réduit en pNext)
    if (fExt) {
      // Effort extérieur appliqué par l’environnement SUR le robot : entre comme −fExt (on calcule le couple à fournir)
      fNext = scale(fExt.f, -1);
      nNext = scale(fExt.n || [0, 0, 0], -1);
      pNext = fExt.p;
    }
    for (let i = n - 1; i >= 0; i--) {
      const { o, z } = f.axes[i];
      const pi = origins[i + 1];
      // Effort transmis par l’articulation i (au point o) : f_i = F_i + f_{i+1}
      const fi = add(Fc[i], fNext);
      let ni = add(Nc[i], cross(sub(pi, o), Fc[i]));
      if (pNext) ni = add(ni, add(nNext, cross(sub(pNext, o), fNext)));
      const dir = this.kin.joints[i].direction < 0 ? -1 : 1;
      tau[i] = (this.kin.joints[i].type === 'prismatic' ? dot(fi, z) : dot(ni, z)) * dir;
      if (wrenches) wrenches[i] = { f: fi, n: ni, o, z };
      fNext = fi; nNext = ni; pNext = o;
    }
    return wantWrenches ? { tau, wrenches, fk: f } : tau;
  }

  /** Couple de gravité g(q). */
  gravityTorque(q, fk = null) {
    return this.rnea(q, zeros(this.n), zeros(this.n), true, fk);
  }

  /** Termes de Coriolis/centrifuges + gravité : b(q, q̇) = C(q,q̇)·q̇ + g(q). */
  bias(q, qd, fk = null, gravityOn = true) {
    return this.rnea(q, qd, zeros(this.n), gravityOn, fk);
  }

  /** Matrice d’inertie M(q) (n×n) par n appels à RNEA, + inertie des rotors ramenée. */
  massMatrix(q, fk = null, withArmature = true) {
    const n = this.n;
    const f = fk || this.kin.fk(q);
    const M = [];
    const z = zeros(n);
    for (let j = 0; j < n; j++) M.push(new Array(n).fill(0));
    for (let j = 0; j < n; j++) {
      const e = zeros(n); e[j] = 1;
      const col = this.rnea(q, z, e, false, f);
      for (let i = 0; i < n; i++) M[i][j] = col[i];
    }
    // Symétrisation numérique
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        const m = 0.5 * (M[i][j] + M[j][i]);
        M[i][j] = m; M[j][i] = m;
      }
      if (withArmature) M[i][i] += this.armature[i];
    }
    return M;
  }

  /** Dynamique directe : q̈ = M⁻¹·(τ − b). */
  forward(q, qd, tau, fk = null) {
    const f = fk || this.kin.fk(q);
    const M = this.massMatrix(q, f);
    const b = this.bias(q, qd, f);
    return solveSPD(M, tau.map((t, i) => t - b[i]));
  }

  /** Énergies cinétique (rotors rigides inclus) et potentielle (pour les vérifications). */
  energy(q, qd, fk = null, withArmature = true) {
    const f = fk || this.kin.fk(q);
    const M = this.massMatrix(q, f, withArmature);
    let T = 0;
    for (let i = 0; i < this.n; i++) for (let j = 0; j < this.n; j++) T += 0.5 * qd[i] * M[i][j] * qd[j];
    let V = 0;
    for (let i = 0; i < this.n; i++) {
      for (const b of this.bodiesOf(i)) {
        const c = mat4TransformPoint(f.frames[i + 1], b.com);
        V -= b.mass * dot(this.gravity, c);
      }
    }
    return { T, V, E: T + V };
  }

  /** Masse totale et centre de gravité global (monde). */
  massProperties(q, fk = null) {
    const f = fk || this.kin.fk(q);
    let m = 0, c = [0, 0, 0];
    for (let i = 0; i < this.n; i++) {
      for (const b of this.bodiesOf(i)) {
        const p = mat4TransformPoint(f.frames[i + 1], b.com);
        m += b.mass; c = add(c, scale(p, b.mass));
      }
    }
    return { mass: m, com: m > 0 ? scale(c, 1 / m) : [0, 0, 0] };
  }

  /** Inertie effective vue par chaque axe (diagonale de M, rotors inclus). */
  effectiveInertia(q) {
    const M = this.massMatrix(q);
    return M.map((row, i) => row[i]);
  }
}

function zeros(n) {
  return new Array(n).fill(0);
}

export { mat3Mul, mat4Mul };
