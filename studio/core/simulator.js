// ORION-6 — Simulateur physique temps réel.
//
// État par articulation i : position/vitesse de sortie (qᵢ, vᵢ) et du rotor ramené (θᵢ, uᵢ).
// Couplage rotor ↔ sortie par la transmission (raideur + jeu). Intégration d’Euler
// linéairement implicite (complément de Schur sur les rotors), stable même avec les raideurs
// magnétiques élevées des pas-à-pas :
//
//   (Ja + α + β)·Δu − (α − γ)·Δv = dt·(τm − τt)
//   [M + diag(α + δ − α(α−γ)/s)]·Δv = dt·(τt − b − τf + τext) + diag(α/s)·r
//
// avec α = dt·(dt·kt + ct) (transmission), β (raideur/amortissement propre du moteur),
// γ (PD continu du mode MIT, dépend de la sortie), δ (frottement visqueux, butées).

import { Kinematics } from './kinematics.js';
import { Dynamics } from './dynamics.js';
import { ActuatorModel, frictionTorque } from './actuators.js';
import { Controller } from './control.js';
import { CollisionModel } from './safety.js';
import {
  JointSegment, CartesianSegment, HoldSegment, BrakeSegment, TrajectoryExecutor, linePath, arcPath, blendTimeForZone,
} from './trajectory.js';
import { solveSPD } from './linalg.js';
import {
  mat4Pos, mat4Rot, mat4Mul, mat4TransformPoint, mat4FromRotPos, mat4InvRigid, sub, cross,
} from './math3d.js';

const HARD_STOP_MARGIN = 4 * Math.PI / 180; // butée mécanique au-delà de la butée logicielle
const HARD_STOP_K = 3000; // N·m/rad
const HARD_STOP_C = 30;

/** Tampon circulaire multi-canaux pour les courbes. */
export class Telemetry {
  constructor(channels, capacity = 4000) {
    this.channels = channels;
    this.capacity = capacity;
    this.data = Object.fromEntries(channels.map((c) => [c, new Float32Array(capacity)]));
    this.head = 0;
    this.size = 0;
  }
  push(rec) {
    const i = this.head;
    for (const c of this.channels) this.data[c][i] = rec[c] ?? NaN;
    this.head = (i + 1) % this.capacity;
    this.size = Math.min(this.size + 1, this.capacity);
  }
  /** Parcours dans l’ordre chronologique. */
  series(c) {
    const out = new Float32Array(this.size);
    const start = (this.head - this.size + this.capacity) % this.capacity;
    const src = this.data[c];
    for (let k = 0; k < this.size; k++) out[k] = src[(start + k) % this.capacity];
    return out;
  }
  clear() { this.head = 0; this.size = 0; }
  toCSV() {
    const cols = this.channels;
    const series = cols.map((c) => this.series(c));
    const lines = [cols.join(';')];
    for (let k = 0; k < this.size; k++) lines.push(series.map((s) => (Number.isFinite(s[k]) ? s[k].toPrecision(7) : '')).join(';'));
    return lines.join('\n');
  }
}

export class Simulator {
  constructor(params, opts = {}) {
    this.listeners = {};
    this.objects = opts.objects || [];
    this.setParams(params, { reset: true });
  }

  // ------------------------------------------------------------------ événements
  on(evt, fn) { (this.listeners[evt] ||= []).push(fn); return () => this.off(evt, fn); }
  off(evt, fn) { this.listeners[evt] = (this.listeners[evt] || []).filter((f) => f !== fn); }
  emit(evt, data) { (this.listeners[evt] || []).forEach((f) => f(data)); }

  // ------------------------------------------------------------------ configuration
  setParams(params, { reset = false } = {}) {
    const prevN = this.n;
    this.params = params;
    this.kin = new Kinematics(params);
    this.n = this.kin.n;
    this.dyn = new Dynamics(this.kin, params);
    this.actuators = params.actuators.joints.map((a) => new ActuatorModel(a, params.actuators.supplyVoltage));
    this.controller = new Controller(this.kin, params, this.actuators);
    this.collision = new CollisionModel(this.kin, params);
    const n = this.n;
    const ch = ['t'];
    for (let i = 0; i < n; i++) ch.push(`q${i}`, `qd${i}`, `v${i}`, `vd${i}`, `tau${i}`, `cap${i}`, `err${i}`, `use${i}`, `motor${i}`);
    ch.push('tcpx', 'tcpy', 'tcpz', 'tcpv', 'tcpxd', 'tcpyd', 'tcpzd');
    const cap = Math.ceil((params.simulation.plotWindow || 10) * (params.simulation.recordHz || 200) * 1.5);
    if (!this.telemetry || this.telemetry.capacity !== cap || prevN !== n) this.telemetry = new Telemetry(ch, cap);
    if (reset || prevN !== n || !this.q) this.reset(params.poses?.home || new Array(n).fill(0));
    this.dyn.payloadBody.mass = this.heldObject ? this.heldObject.mass : 0;
    this.controller.model.payloadBody.mass = this.dyn.payloadBody.mass;
  }

  /** Remet le robot à l’arrêt dans la configuration q. */
  reset(q) {
    const n = this.n;
    this.t = 0;
    this.q = q.slice(0, n);
    this.v = new Array(n).fill(0);
    this.acc = new Array(n).fill(0);
    this.th = this.q.slice();
    this.u = new Array(n).fill(0);
    this.cmdPrev = this.q.slice();
    this.lostSteps = new Array(n).fill(0);
    this.slipOffset = new Array(n).fill(0);
    this.tauT = new Array(n).fill(0);
    this.tauMotor = new Array(n).fill(0);
    this.utilization = new Array(n).fill(0);
    this.ctrlOut = null;
    this.nextCtrl = 0;
    this.Mcache = null;
    this.nextM = 0;
    this.nextRec = 0;
    this.enabled = true;
    this.fault = null;
    this.executor = new TrajectoryExecutor(n);
    this.jog = null;
    this.hold = { q: this.q.slice(), qd: new Array(n).fill(0), qdd: new Array(n).fill(0) };
    this.des = this.hold;
    this.push = null;
    this.gripper = { pos: this.params.gripper.strokeMax, target: this.params.gripper.strokeMax };
    this.heldObject = null;
    this.io = { do: new Array(8).fill(false), di: new Array(8).fill(false) };
    this.controller?.reset();
    this.telemetry?.clear();
    this.collisionState = { ok: true };
    this.gravityHold();
  }

  /**
   * Initialise un état d’équilibre sous gravité pour la consigne courante (évite un transitoire
   * au démarrage) : pas-à-pas → le rotor suit la consigne à l’angle de charge près et la sortie
   * « s’affaisse » du demi-jeu + déformation ; autres actionneurs → la sortie est sur la consigne.
   */
  gravityHold() {
    if (!this.q) return;
    const sp = this.params.simulation;
    const stepperMode = this.params.control.mode === 'stepper';
    const qCmd = this.hold.q;
    for (let it = 0; it < 2; it++) {
      const g = this.dyn.gravityTorque(this.q);
      for (let i = 0; i < this.n; i++) {
        const a = this.actuators[i];
        const h = sp.enableBacklash ? a.backlash / 2 : 0;
        const sag = Math.sign(g[i]) * h + g[i] / a.kt;
        if (stepperMode && a.type === 'stepper') {
          const amp = a.G * a.Trun;
          const e = Math.asin(Math.max(-0.95, Math.min(0.95, g[i] / amp))) / (a.Nr * a.G);
          this.th[i] = qCmd[i] - e;
          this.q[i] = this.th[i] - sag;
          this.cmdPrev[i] = a.quantize(qCmd[i]);
        } else {
          this.q[i] = qCmd[i];
          this.th[i] = this.q[i] + sag;
        }
      }
    }
  }

  // ------------------------------------------------------------------ commandes
  get tcp() { return this.kin.tcp(this.q); }
  get desiredTcp() { return this.kin.tcp(this.des.q); }

  setEnabled(on) {
    this.enabled = !!on;
    if (on) { this.fault = null; this.holdHere(); }
    this.emit('state', { enabled: this.enabled });
  }

  /**
   * Arrêt de catégorie 1 : freinage contrôlé puis maintien en position, nouveaux mouvements
   * refusés tant que le défaut n’est pas acquitté. (setEnabled(false) = coupure de puissance.)
   */
  estop(reason = 'Arrêt d’urgence') {
    this.brake();
    this.jog = null;
    this.fault = reason;
    this.emit('fault', { reason });
  }

  clearFault() {
    this.fault = null;
    this.holdHere();
    this.emit('state', { enabled: this.enabled, fault: null });
  }

  holdHere() {
    this.executor.clear();
    this.jog = null;
    this.hold = { q: this.q.slice(), qd: new Array(this.n).fill(0), qdd: new Array(this.n).fill(0) };
    this.des = this.hold;
    this.controller.reset();
  }

  /** Position de départ du prochain mouvement (fin de la file ou consigne courante). */
  plannedEnd() {
    const it = this.executor.items;
    if (it.length) return it[it.length - 1].seg.q1.slice();
    return this.des.q.slice();
  }

  motionOptions(opts = {}) {
    const tp = this.params.trajectory;
    return {
      profile: opts.profile || tp.profile,
      speed: (opts.speed ?? 1) * tp.speedOverride,
      accel: (opts.accel ?? 1) * tp.accelScale,
      speedAbs: opts.speedAbs,
    };
  }

  /** Ajoute un segment à la file (avec lissage éventuel). */
  enqueue(seg, zone = 0) {
    if (!this.enabled) throw new Error('Robot hors puissance (activez les moteurs)');
    if (this.fault) throw new Error(`Défaut actif : ${this.fault}`);
    this.jog = null;
    let blend = 0;
    const items = this.executor.items;
    if (zone > 0 && items.length) blend = blendTimeForZone(this.kin, items[items.length - 1].seg, zone);
    // Hors file, on démarre maintenant depuis la consigne courante
    this.executor.push(seg, blend, this.t);
    this.emit('queue', { size: this.executor.items.length });
    return seg;
  }

  moveJ(qTarget, opts = {}) {
    const q0 = this.plannedEnd();
    const qt = this.kin.clampToLimits(qTarget);
    const seg = new JointSegment(q0, qt, this.params.limits.joints, this.motionOptions(opts));
    seg.tag = opts.tag;
    return this.enqueue(seg, opts.zone ?? 0);
  }

  moveL(Ttarget, opts = {}) {
    const q0 = this.plannedEnd();
    const seg = new CartesianSegment(this.kin, linePath(this.kin.tcp(q0), Ttarget), q0, this.params, this.motionOptions(opts));
    if (!seg.ok) throw new Error(`MoveL impossible : ${seg.error}`);
    seg.tag = opts.tag;
    return this.enqueue(seg, opts.zone ?? 0);
  }

  moveC(Tvia, Ttarget, opts = {}) {
    const q0 = this.plannedEnd();
    const seg = new CartesianSegment(this.kin, arcPath(this.kin.tcp(q0), Tvia, Ttarget), q0, this.params, this.motionOptions(opts));
    if (!seg.ok) throw new Error(`MoveC impossible : ${seg.error}`);
    seg.tag = opts.tag;
    return this.enqueue(seg, opts.zone ?? 0);
  }

  wait(seconds, tag) {
    const seg = new HoldSegment(this.plannedEnd(), seconds);
    seg.tag = tag;
    this.executor.push(seg, 0, this.t);
    return seg;
  }

  /** Arrêt contrôlé (catégorie 2) : freinage à décélération max depuis la vitesse courante. */
  stopMotion() {
    this.brake();
  }

  brake() {
    const d = this.des;
    this.executor.clear();
    this.jog = null;
    const seg = new BrakeSegment(d.q, d.qd, this.params.limits.joints);
    this.executor.push(seg, 0, this.t);
  }

  /** Jog continu : vitesse articulaire (i, signe) ou cartésienne (axe 0..5, repère). */
  startJog(spec) {
    if (!this.enabled) return;
    if (this.executor.busy) this.executor.clear();
    this.jog = { ...spec, vel: new Array(this.n).fill(0) };
    this.hold = { q: this.des.q.slice(), qd: new Array(this.n).fill(0), qdd: new Array(this.n).fill(0) };
  }
  stopJog() { if (this.jog) this.jog.stopping = true; }

  setGripper(open01) {
    const g = this.params.gripper;
    this.gripper.target = Math.max(0, Math.min(1, open01)) * g.strokeMax;
  }
  gripperBusy() { return Math.abs(this.gripper.pos - this.gripper.target) > 1e-4 && !this.gripper.blocked; }

  /** Effort extérieur (souris) : force monde appliquée en un point solidaire du segment `link`. */
  setPush(push) { this.push = push; }

  // ------------------------------------------------------------------ consigne
  computeDesired(t, dt) {
    if (this.executor.busy) {
      const s = this.executor.sample(t);
      if (s) {
        this.des = s;
        this.hold = { q: s.q.slice(), qd: new Array(this.n).fill(0), qdd: new Array(this.n).fill(0) };
        if (s.done) this.emit('motionDone', {});
        return s;
      }
    }
    if (this.jog) return this.jogStep(dt);
    this.des = this.hold;
    return this.hold;
  }

  jogStep(dt) {
    const jg = this.jog, n = this.n;
    const lim = this.params.limits.joints;
    const tp = this.params.trajectory;
    const ovr = tp.speedOverride;
    let vTarget = new Array(n).fill(0);
    if (!jg.stopping) {
      if (jg.type === 'joint') {
        vTarget[jg.joint] = jg.dir * Math.min(tp.jog.jointSpeed, lim[jg.joint].vmax) * ovr;
      } else {
        // Contrôle en vitesse résolue : q̇ = J⁺·ẋ (moindres carrés amortis)
        const x = new Array(6).fill(0);
        const lin = tp.jog.linSpeed * ovr, ang = this.params.limits.cartesian.wmax * 0.3 * ovr;
        x[jg.axis] = jg.dir * (jg.axis < 3 ? lin : ang);
        let xw = x;
        if (jg.frame === 'tool') {
          const R = mat4Rot(this.kin.tcp(this.hold.q));
          const rot = (v) => [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]];
          xw = [...rot(x.slice(0, 3)), ...rot(x.slice(3))];
        }
        const J = this.kin.jacobian(this.hold.q);
        const JJt = J.map((r) => J.map((r2) => r.reduce((s, v, k) => s + v * r2[k], 0)));
        for (let r = 0; r < 6; r++) JJt[r][r] += 0.02 ** 2;
        const y = solveSPD(JJt, xw);
        if (y) vTarget = new Array(n).fill(0).map((_, i) => J.reduce((s, row, r) => s + row[i] * y[r], 0));
        // Mise à l’échelle si une vitesse articulaire est dépassée
        let k = 1;
        vTarget.forEach((v, i) => { k = Math.max(k, Math.abs(v) / (lim[i].vmax * ovr)); });
        vTarget = vTarget.map((v) => v / k);
      }
    }
    // Rampe d’accélération
    let moving = false;
    for (let i = 0; i < n; i++) {
      const a = lim[i].amax * 0.5 * dt;
      const dv = Math.max(-a, Math.min(a, vTarget[i] - jg.vel[i]));
      jg.vel[i] += dv;
      let qn = this.hold.q[i] + jg.vel[i] * dt;
      const m = this.params.safety.softLimitMargin || 0;
      if (qn < lim[i].min + m) { qn = lim[i].min + m; jg.vel[i] = 0; }
      if (qn > lim[i].max - m) { qn = lim[i].max - m; jg.vel[i] = 0; }
      this.hold.q[i] = qn;
      this.hold.qd[i] = jg.vel[i];
      if (Math.abs(jg.vel[i]) > 1e-6) moving = true;
    }
    if (jg.stopping && !moving) this.jog = null;
    this.des = this.hold;
    return this.hold;
  }

  // ------------------------------------------------------------------ mesure
  measure() {
    const n = this.n, q = new Array(n);
    const noise = this.params.simulation.sensorNoise || 0;
    for (let i = 0; i < n; i++) {
      const a = this.params.actuators.joints[i];
      const cpr = a.encoderCpr || 0;
      let x = a.encoderOnOutput || cpr === 0 ? this.q[i] : this.th[i];
      if (noise > 0) x += noise * gaussian();
      if (cpr > 0) {
        const res = (2 * Math.PI) / (a.encoderOnOutput ? cpr : cpr * this.actuators[i].G);
        x = Math.round(x / res) * res;
      }
      q[i] = x;
    }
    return { q };
  }

  // ------------------------------------------------------------------ intégration
  /** Avance la simulation de `dtWall` secondes (temps réel écoulé). */
  advance(dtWall) {
    const sp = this.params.simulation;
    const dtTarget = Math.min(dtWall, 0.05) * (sp.realtimeFactor || 1);
    const dt = sp.dt || 0.00025;
    const steps = Math.max(1, Math.round(dtTarget / dt));
    for (let k = 0; k < steps; k++) this.step(dt);
    return steps;
  }

  step(dt) {
    const n = this.n, p = this.params, sp = p.simulation;
    const t = this.t;
    // 1) Consigne (échantillonnée à chaque pas pour une génération de pas régulière)
    const des = this.enabled ? this.computeDesired(t, dt) : this.hold;
    // 2) Correcteur à sa propre fréquence (bloqueur d’ordre 0)
    const loopDt = 1 / (p.control.loopHz || 1000);
    if (t >= this.nextCtrl - 1e-12) {
      this.ctrlOut = this.controller.compute(this.measure(), des, loopDt);
      this.nextCtrl = t + loopDt;
      this.checkSafety(des);
    }
    const out = this.ctrlOut;
    // 3) Dynamique
    const fk = this.kin.fk(this.q);
    // M(q) varie lentement : recalculée à ~1 kHz, biais b(q, q̇) à chaque pas.
    if (!this.Mcache || this.t >= this.nextM - 1e-12) {
      this.Mcache = this.dyn.massMatrix(this.q, fk, false);
      this.nextM = this.t + 0.001;
    }
    const M = this.Mcache;
    let tauExt = null;
    if (this.push) tauExt = this.pushTorques(fk);
    const b = this.dyn.bias(this.q, this.v, fk);
    const A = M.map((r) => r.slice());
    const h = new Array(n), rr = new Array(n), s = new Array(n), coup = new Array(n);
    for (let i = 0; i < n; i++) {
      const act = this.actuators[i];
      const qi = this.q[i], vi = this.v[i], th = this.th[i], ui = this.u[i];
      // Transmission rotor → sortie
      const tr = act.transmission(th, qi, ui, vi, sp.enableBacklash);
      // Couple moteur sur le rotor
      let tauM = 0, beta = 0, gamma = 0;
      if (!this.enabled) {
        tauM = 0; // moteurs libres (pas-à-pas non alimentés : léger couple de détente négligé)
      } else if (out && out.kind[i] === 'position' && act.type === 'stepper') {
        const cmd = act.quantize(des.q[i]);
        const cmdVel = (cmd - this.cmdPrev[i]) / dt;
        this.cmdPrev[i] = cmd;
        const st = act.stepperTorque(cmd, th, ui);
        tauM = st.tau + st.c * (cmdVel - ui);
        beta = dt * (dt * st.k + st.c);
        this.utilization[i] = Math.abs(Math.sin(st.elec));
        // Décrochage : le rotor se cale une (ou plusieurs) période(s) électrique(s) plus loin
        // = 4 pas entiers perdus par période. Le modèle sin() reste valable (périodique).
        const rel = st.elec - 2 * Math.PI * this.slipOffset[i];
        const slip = Math.round(rel / (2 * Math.PI));
        if (slip !== 0) {
          this.slipOffset[i] += slip;
          this.lostSteps[i] += 4 * Math.abs(slip);
          this.emit('stepLoss', { joint: i, steps: 4 * Math.abs(slip), total: this.lostSteps[i] });
        }
      } else if (out && out.kind[i] === 'mit') {
        const m = out.mit[i];
        const raw = m.kp * (m.qd - qi) + m.kd * (m.vd - vi) + m.tff;
        const cap = act.capacity(vi, Math.sign(raw)) * (p.control.joints[i].torqueLimit ?? 1);
        tauM = Math.max(-cap, Math.min(cap, raw));
        if (Math.abs(raw) < cap) gamma = dt * (dt * m.kp + m.kd);
        this.utilization[i] = cap > 0 ? Math.abs(tauM) / cap : 1;
      } else if (out) {
        tauM = out.tau[i];
        const cap = act.capacity(vi, Math.sign(tauM));
        this.utilization[i] = cap > 0 ? Math.abs(tauM) / cap : 1;
      }
      this.tauMotor[i] = tauM;
      // Frottements (sortie)
      let tf = 0, cf = 0;
      if (sp.enableFriction) { const f = frictionTorque(vi, p.dynamics.friction[i]); tf = f.tau; cf = f.slope; }
      // Butées mécaniques
      let tStop = 0, kStop = 0, cStop = 0;
      const lim = p.limits.joints[i];
      if (qi > lim.max + HARD_STOP_MARGIN) { tStop = -HARD_STOP_K * (qi - lim.max - HARD_STOP_MARGIN) - HARD_STOP_C * vi; kStop = HARD_STOP_K; cStop = HARD_STOP_C; }
      if (qi < lim.min - HARD_STOP_MARGIN) { tStop = -HARD_STOP_K * (qi - lim.min + HARD_STOP_MARGIN) - HARD_STOP_C * vi; kStop = HARD_STOP_K; cStop = HARD_STOP_C; }
      const alpha = dt * (dt * tr.k + tr.c);
      const delta = dt * (cf + cStop) + dt * dt * kStop;
      const si = act.Ja + alpha + beta;
      rr[i] = dt * (tauM - tr.tau);
      h[i] = dt * (tr.tau - b[i] - tf + tStop + (tauExt ? tauExt[i] : 0));
      s[i] = si;
      coup[i] = alpha - gamma;
      A[i][i] += alpha + delta - (alpha * (alpha - gamma)) / si;
      h[i] += (alpha / si) * rr[i];
      this.tauT[i] = tr.tau;
    }
    const dv = solveSPD(A, h) || new Array(n).fill(0);
    for (let i = 0; i < n; i++) {
      const du = (rr[i] + coup[i] * dv[i]) / s[i];
      this.v[i] += dv[i];
      this.u[i] += du;
      this.acc[i] = dv[i] / dt;
      this.q[i] += dt * this.v[i];
      this.th[i] += dt * this.u[i];
    }
    this.t = t + dt;
    this.updateGripper(dt);
    // 4) Enregistrement
    if (this.t >= this.nextRec) {
      this.nextRec = this.t + 1 / (sp.recordHz || 200);
      this.record(fk);
    }
  }

  /** Couples articulaires équivalents à la force de poussée (Jᵀ·F au point d’application). */
  pushTorques(fk) {
    const { link, local, force } = this.push;
    const P = mat4TransformPoint(fk.frames[link], local);
    const tau = new Array(this.n).fill(0);
    for (let i = 0; i < Math.min(link, this.n); i++) {
      const { o, z } = fk.axes[i];
      const dir = this.kin.joints[i].direction < 0 ? -1 : 1;
      const jv = cross(z, sub(P, o));
      tau[i] = dir * (jv[0] * force[0] + jv[1] * force[1] + jv[2] * force[2]);
    }
    return tau;
  }

  checkSafety(des) {
    const p = this.params;
    if (!this.enabled) return;
    // Collision géométrique sur la consigne
    if (p.safety.selfCollision || p.safety.floorCollision) {
      const c = this.collision.check(des.q);
      this.collisionState = c;
      if (!c.ok && !this.fault && (this.executor.busy || this.jog)) {
        const what = c.self.length ? `auto-collision segments ${c.self[0].a}/${c.self[0].b}` : `collision avec la table (segment ${c.floor[0].link})`;
        this.estop(`Arrêt de protection : ${what}`);
        return;
      }
    }
    // Détection de choc par résidu de couple : r = τtransmis − τ̂f − (M̂·q̈ + b̂) ≈ τext
    this.residual = this.residual || new Array(this.n).fill(0);
    if (p.safety.collisionDetection && this.push == null && !this.fault && this.t > 0.2) {
      const thr = p.safety.collisionThreshold;
      const tauModel = this.controller.model.rnea(this.q, this.v, this.acc, true);
      for (let i = 0; i < this.n; i++) {
        const fr = p.simulation.enableFriction ? frictionTorque(this.v[i], p.dynamics.friction[i]).tau : 0;
        const r = this.tauT[i] - fr - tauModel[i];
        // Filtrage (≈ 20 ms) pour ignorer les transitoires numériques
        this.residual[i] += 0.05 * (r - this.residual[i]);
        if (Math.abs(this.residual[i]) > thr) {
          this.estop(`Choc détecté sur l’axe ${i + 1} (résidu ${this.residual[i].toFixed(2)} N·m)`);
          this.residual.fill(0);
          break;
        }
      }
    }
  }

  // ------------------------------------------------------------------ pince et objets
  updateGripper(dt) {
    const g = this.params.gripper, gs = this.gripper;
    const step = g.speed * dt;
    const closing = gs.target < gs.pos;
    let next = gs.pos + Math.max(-step, Math.min(step, gs.target - gs.pos));
    gs.blocked = false;
    if (closing && !this.heldObject) {
      const obj = this.objectInGripper();
      if (obj && next <= obj.size) {
        next = obj.size;
        gs.blocked = true;
        this.attachObject(obj);
      }
    } else if (closing && this.heldObject) {
      next = Math.max(next, this.heldObject.size);
      gs.blocked = next <= this.heldObject.size + 1e-6;
    }
    if (!closing && this.heldObject && next > this.heldObject.size + 0.002) this.releaseObject();
    gs.pos = next;
    if (this.heldObject) {
      const T = mat4Mul(this.kin.tcp(this.q), this.heldObject.grasp);
      this.heldObject.pose = T;
    }
  }

  objectInGripper() {
    const T = this.kin.tcp(this.q);
    const P = mat4Pos(T);
    let best = null, bestD = Infinity;
    for (const o of this.objects) {
      const c = mat4Pos(o.pose);
      const d = Math.hypot(c[0] - P[0], c[1] - P[1], c[2] - P[2]);
      if (d < o.size * 0.8 && d < bestD && o.size <= this.params.gripper.strokeMax) { best = o; bestD = d; }
    }
    return best;
  }

  attachObject(o) {
    this.heldObject = o;
    o.grasp = mat4Mul(mat4InvRigid(this.kin.tcp(this.q)), o.pose);
    this.dyn.payloadBody.mass = o.mass;
    this.controller.model.payloadBody.mass = o.mass;
    this.emit('grasp', { object: o });
  }

  releaseObject() {
    const o = this.heldObject;
    this.heldObject = null;
    this.dyn.payloadBody.mass = 0;
    this.controller.model.payloadBody.mass = 0;
    // L’objet tombe sur la table (ou sur un autre objet) en gardant son lacet
    const P = mat4Pos(o.pose);
    const R = mat4Rot(o.pose);
    const yaw = Math.atan2(R[3], R[0]);
    let z = (this.params.limits.floorZ || 0) + o.size / 2;
    for (const other of this.objects) {
      if (other === o) continue;
      const c = mat4Pos(other.pose);
      if (Math.abs(c[0] - P[0]) < (o.size + other.size) / 2 && Math.abs(c[1] - P[1]) < (o.size + other.size) / 2 && c[2] < P[2]) {
        z = Math.max(z, c[2] + other.size / 2 + o.size / 2);
      }
    }
    const c = Math.cos(yaw), sn = Math.sin(yaw);
    o.pose = mat4FromRotPos([c, -sn, 0, sn, c, 0, 0, 0, 1], [P[0], P[1], z]);
    this.emit('release', { object: o });
  }

  // ------------------------------------------------------------------ télémétrie
  record(fk) {
    const n = this.n;
    const rec = { t: this.t };
    const des = this.des;
    for (let i = 0; i < n; i++) {
      rec[`q${i}`] = this.q[i];
      rec[`qd${i}`] = des.q[i];
      rec[`v${i}`] = this.v[i];
      rec[`vd${i}`] = des.qd[i];
      rec[`tau${i}`] = this.tauT[i];
      rec[`cap${i}`] = this.actuators[i].capacity(this.v[i]);
      rec[`err${i}`] = des.q[i] - this.q[i];
      rec[`use${i}`] = this.utilization[i];
      rec[`motor${i}`] = this.tauMotor[i];
    }
    const P = mat4Pos(fk.tcp);
    rec.tcpx = P[0]; rec.tcpy = P[1]; rec.tcpz = P[2];
    const Pd = mat4Pos(this.kin.tcp(des.q));
    rec.tcpxd = Pd[0]; rec.tcpyd = Pd[1]; rec.tcpzd = Pd[2];
    const J = this.kin.jacobian(this.q, fk);
    let vx = 0, vy = 0, vz = 0;
    for (let i = 0; i < n; i++) { vx += J[0][i] * this.v[i]; vy += J[1][i] * this.v[i]; vz += J[2][i] * this.v[i]; }
    rec.tcpv = Math.hypot(vx, vy, vz);
    this.telemetry.push(rec);
  }
}

function gaussian() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
