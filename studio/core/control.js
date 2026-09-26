// ORION-6 — Lois de commande.
//
// Modes (params.control.mode) :
//   stepper          consigne de position envoyée aux actionneurs (pas-à-pas en boucle ouverte ;
//                    les actionneurs brushless utilisent alors leur PD interne avec les gains MIT)
//   pid              τ = Kp·e + Ki·∫e + Kd·(q̇* − q̇) + anticipations            (échantillonné, BOZ)
//   mit              τ = Kp·(p* − p) + Kd·(v* − v) + τff   (exécuté en continu dans le driver moteur)
//   computed_torque  τ = M̂(q)·(q̈* + ω²·e + 2ζω·ė) + b̂(q, q̇)
//   impedance        τ = Jᵀ·(K·Δx + D·Δẋ) + ĝ(q) − Dnull·q̇
//   gravity_comp     τ = ĝ(q) − Dnull·q̇   (le bras « flotte », déplaçable à la main)

import { Dynamics } from './dynamics.js';
import { frictionEstimate } from './actuators.js';
import { mat4Pos, mat4Rot, rotError } from './math3d.js';
import { matMul, transpose, inverse, identity, solveSPD } from './linalg.js';

export class Controller {
  constructor(kin, params, actuators) {
    this.kin = kin;
    this.actuators = actuators;
    this.setParams(params);
    this.reset();
  }

  setParams(params) {
    this.params = params;
    this.cp = params.control;
    this.n = this.kin.n;
    // Modèle du correcteur (éventuellement faussé pour tester la robustesse)
    this.model = new Dynamics(this.kin, params, { massScale: 1 + (this.cp.modelError || 0) });
    this.friction = params.dynamics.friction;
    const w = 2 * Math.PI * (this.cp.autotune?.bandwidthHz ?? 8);
    const z = this.cp.autotune?.zeta ?? 0.9;
    this.ctKp = w * w;
    this.ctKd = 2 * z * w;
  }

  reset() {
    this.integral = new Array(this.n).fill(0);
    this.vFilt = new Array(this.n).fill(0);
    this.qPrev = null;
    this.out = null;
  }

  /** Filtre passe-bas du 1er ordre sur la vitesse mesurée (différences finies). */
  filterVelocity(qMeas, dt) {
    if (!this.qPrev) { this.qPrev = qMeas.slice(); return this.vFilt; }
    for (let i = 0; i < this.n; i++) {
      const raw = (qMeas[i] - this.qPrev[i]) / dt;
      const fc = this.cp.joints[i].dFilterHz || 200;
      const a = 1 - Math.exp(-2 * Math.PI * fc * dt);
      this.vFilt[i] += a * (raw - this.vFilt[i]);
    }
    this.qPrev = qMeas.slice();
    return this.vFilt;
  }

  /**
   * Calcule la commande.
   * @param meas  { q, v (vitesse vraie si pas de codeur) , hasEncoder[] }
   * @param des   { q, qd, qdd } consigne
   * @param dt    période de la boucle
   * @returns { kind[], posCmd[], velCmd[], tau[], mit: {qd, vd, kp, kd, tff}[] , ff[] }
   */
  compute(meas, des, dt) {
    const n = this.n, cp = this.cp, mode = cp.mode;
    const v = this.filterVelocity(meas.q, dt);
    const out = {
      kind: new Array(n), posCmd: des.q.slice(), velCmd: des.qd.slice(),
      tau: new Array(n).fill(0), mit: new Array(n).fill(null), ff: new Array(n).fill(0),
    };
    const q = meas.q;
    const vUse = meas.velocityOverride || v;
    const ffOn = cp.feedforward || {};
    let fk = null;
    const needModel = ffOn.gravity || ffOn.inertia || mode === 'computed_torque' || mode === 'impedance' || mode === 'gravity_comp' || mode === 'mit';
    if (needModel) fk = this.kin.fk(q);

    // Anticipation : gravité + inertie (+ Coriolis sur la vitesse désirée) + frottements
    const ff = new Array(n).fill(0);
    if (ffOn.gravity || ffOn.inertia) {
      const tauG = ffOn.gravity ? this.model.gravityTorque(q, fk) : new Array(n).fill(0);
      const tauI = ffOn.inertia ? this.model.rnea(q, des.qd, des.qdd, false, fk) : new Array(n).fill(0);
      for (let i = 0; i < n; i++) ff[i] = tauG[i] + tauI[i];
      if (ffOn.inertia) {
        // Inertie des rotors (non incluse dans RNEA)
        for (let i = 0; i < n; i++) ff[i] += this.actuators[i].Ja * des.qdd[i];
      }
    }
    if (ffOn.friction) for (let i = 0; i < n; i++) ff[i] += frictionEstimate(des.qd[i], this.friction[i]);
    out.ff = ff;

    if (mode === 'stepper') {
      for (let i = 0; i < n; i++) {
        const act = this.actuators[i];
        if (act.type === 'stepper') {
          out.kind[i] = 'position';
          out.posCmd[i] = act.quantize(des.q[i]);
        } else {
          // Mode position interne du driver (PD « MIT » + anticipation)
          out.kind[i] = 'mit';
          out.mit[i] = { qd: des.q[i], vd: des.qd[i], kp: cp.joints[i].mitKp, kd: cp.joints[i].mitKd, tff: ff[i] };
        }
      }
      return (this.out = out);
    }

    if (mode === 'mit') {
      for (let i = 0; i < n; i++) {
        out.kind[i] = 'mit';
        out.mit[i] = { qd: des.q[i], vd: des.qd[i], kp: cp.joints[i].mitKp, kd: cp.joints[i].mitKd, tff: ff[i] };
      }
      return (this.out = out);
    }

    for (let i = 0; i < n; i++) out.kind[i] = 'torque';

    if (mode === 'pid') {
      for (let i = 0; i < n; i++) {
        const g = cp.joints[i];
        let e = des.q[i] - q[i];
        if (Math.abs(e) < (g.deadband || 0)) e = 0;
        const ed = des.qd[i] - vUse[i];
        const iTerm = g.ki * this.integral[i];
        let u = g.kp * e + iTerm + g.kd * ed + ff[i];
        const cap = this.actuators[i].capacity(vUse[i], Math.sign(u)) * (g.torqueLimit ?? 1);
        const sat = Math.abs(u) > cap;
        // Anti-emballement : intégration conditionnelle
        if (!sat || Math.sign(e) !== Math.sign(u)) {
          this.integral[i] += e * dt;
          if (g.ki > 0) {
            const lim = (g.iLimit || 0) / g.ki;
            this.integral[i] = Math.max(-lim, Math.min(lim, this.integral[i]));
          }
        }
        u = Math.max(-cap, Math.min(cap, u));
        out.tau[i] = u;
      }
      return (this.out = out);
    }

    if (mode === 'computed_torque') {
      const aRef = des.qdd.map((a, i) => a + this.ctKp * (des.q[i] - q[i]) + this.ctKd * (des.qd[i] - vUse[i]));
      const tau = this.model.rnea(q, vUse, aRef, true, fk);
      for (let i = 0; i < n; i++) {
        let u = tau[i] + this.actuators[i].Ja * aRef[i] + (ffOn.friction ? frictionEstimate(des.qd[i], this.friction[i]) : 0);
        const cap = this.actuators[i].capacity(vUse[i], Math.sign(u)) * (cp.joints[i].torqueLimit ?? 1);
        out.tau[i] = Math.max(-cap, Math.min(cap, u));
      }
      return (this.out = out);
    }

    if (mode === 'impedance') {
      const imp = cp.impedance;
      const Td = this.kin.tcp(des.q);
      const T = fk.tcp;
      const dp = mat4Pos(Td).map((v2, k) => v2 - mat4Pos(T)[k]);
      const dr = rotError(mat4Rot(Td), mat4Rot(T));
      const J = this.kin.jacobian(q, fk);
      const xdErr = new Array(6).fill(0);
      for (let r = 0; r < 6; r++) for (let i = 0; i < n; i++) xdErr[r] += J[r][i] * (des.qd[i] - vUse[i]);
      // Inertie apparente dans l’espace opérationnel pour l’amortissement critique
      const M = this.model.massMatrix(q, fk);
      const Minv = inverse(M) || identity(n);
      const Lam = matMul(matMul(J, Minv), transpose(J));
      const k = [imp.kTrans, imp.kTrans, imp.kTrans, imp.kRot, imp.kRot, imp.kRot];
      const F = new Array(6);
      for (let r = 0; r < 6; r++) {
        // Λ⁻¹ diagonal ≈ 1/masse apparente
        const mApp = 1 / Math.max(Lam[r][r], 1e-6);
        const d = 2 * imp.zeta * Math.sqrt(k[r] * Math.min(mApp, r < 3 ? 20 : 0.5));
        F[r] = k[r] * (r < 3 ? dp[r] : dr[r - 3]) + d * xdErr[r];
      }
      const tg = this.model.gravityTorque(q, fk);
      for (let i = 0; i < n; i++) {
        let u = tg[i] - imp.nullKd * vUse[i];
        for (let r = 0; r < 6; r++) u += J[r][i] * F[r];
        const cap = this.actuators[i].capacity(vUse[i], Math.sign(u)) * (cp.joints[i].torqueLimit ?? 1);
        out.tau[i] = Math.max(-cap, Math.min(cap, u));
      }
      return (this.out = out);
    }

    // gravity_comp
    const tg = this.model.gravityTorque(q, fk);
    for (let i = 0; i < n; i++) {
      const u = tg[i] - (cp.impedance?.nullKd ?? 0.3) * vUse[i];
      const cap = this.actuators[i].capacity(vUse[i], Math.sign(u)) * (cp.joints[i].torqueLimit ?? 1);
      out.tau[i] = Math.max(-cap, Math.min(cap, u));
    }
    return (this.out = out);
  }
}

/**
 * Auto-réglage par placement de pôles autour de la configuration q :
 *   J = M_ii(q) + N²·J_rotor ;  Kp = J·ω² ; Kd = 2ζ·J·ω ; Ki = Kp·ω/10.
 * Pour le mode MIT, on applique la même règle (Kp, Kd) bornée par KP_MAX/KD_MAX.
 */
export function autoTune(kin, params, q, actuators) {
  const dyn = new Dynamics(kin, params);
  const M = dyn.massMatrix(q, null, true);
  const w = 2 * Math.PI * params.control.autotune.bandwidthHz;
  const z = params.control.autotune.zeta;
  return M.map((row, i) => {
    const J = row[i];
    const kp = J * w * w;
    const kd = 2 * z * J * w;
    const ki = (kp * w) / 10;
    const a = params.actuators.joints[i];
    const cap = actuators ? actuators[i].capacity(0) : Infinity;
    return {
      inertia: J,
      kp, kd, ki,
      iLimit: Math.min(cap * 0.3, Math.max(0.5, kp * 0.01)),
      mitKp: Math.min(kp, a.kpMax ?? 500),
      mitKd: Math.min(kd, a.kdMax ?? 5),
    };
  });
}

export { solveSPD };
