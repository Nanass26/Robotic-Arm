// ORION Studio — panneau Analyse : dimensionnement, réponse indicielle, auto-réglage,
// espace de travail, efforts dans les roulements, singularités.

import { h, btn, toast, cssVar, attachHelp } from './dom.js';
import { Kinematics, rng } from '../core/kinematics.js';
import { Dynamics } from '../core/dynamics.js';
import { ActuatorModel } from '../core/actuators.js';
import { Simulator } from '../core/simulator.js';
import { autoTune } from '../core/control.js';
import { dot, sub, scale, norm, mat4Pos } from '../core/math3d.js';

const R2D = 180 / Math.PI, D2R = Math.PI / 180;
const sevCls = (m) => (m >= 1.5 ? 'good' : m >= 1.1 ? 'warn' : 'crit');
const sevTxt = { good: 'OK', warn: 'Juste', crit: 'Insuffisant' };

/** Couples requis au pire cas sur un échantillon de l’espace articulaire. */
export function sizingAnalysis(params, samples = 2500, payload = null) {
  const p = structuredClone(params);
  p.dynamics.payload.mass = payload ?? p.dynamics.payloadRated;
  const kin = new Kinematics(p);
  const dyn = new Dynamics(kin, p);
  const acts = p.actuators.joints.map((a) => new ActuatorModel(a, p.actuators.supplyVoltage));
  const n = kin.n;
  const rand = rng(11);
  const lim = p.limits.joints;
  const gMax = new Array(n).fill(0), dMax = new Array(n).fill(0), qAt = new Array(n).fill(null);
  for (let s = 0; s < samples; s++) {
    const q = lim.map((l) => l.min + rand() * (l.max - l.min));
    const f = kin.fk(q);
    const g = dyn.gravityTorque(q, f);
    const M = dyn.massMatrix(q, f, true);
    for (let i = 0; i < n; i++) {
      let inert = 0;
      for (let j = 0; j < n; j++) inert += Math.abs(M[i][j]) * lim[j].amax * p.trajectory.accelScale;
      const tau = Math.abs(g[i]) + inert + p.dynamics.friction[i].coulomb;
      if (Math.abs(g[i]) > gMax[i]) { gMax[i] = Math.abs(g[i]); }
      if (tau > dMax[i]) { dMax[i] = tau; qAt[i] = q; }
    }
  }
  return acts.map((a, i) => {
    const cap0 = a.capacity(0);
    const capV = a.capacity(lim[i].vmax);
    const margin = cap0 / Math.max(dMax[i], 1e-6);
    const marginG = cap0 / Math.max(gMax[i], 1e-6);
    const motorRpm = (lim[i].vmax * a.G * 60) / (2 * Math.PI);
    const cornerRpm = a.type === 'stepper' ? (a.omegaCorner * 60) / (2 * Math.PI) : null;
    return { joint: i, gMax: gMax[i], dMax: dMax[i], cap0, capV, margin, marginG, motorRpm, cornerRpm, G: a.G, qAt: qAt[i] };
  });
}

/** Charge utile maximale pour laquelle toutes les marges statiques restent ≥ 1,2. */
export function maxPayload(params) {
  let lo = 0, hi = 10;
  const ok = (m) => sizingAnalysis(params, 600, m).every((r) => r.marginG >= 1.2 && r.margin >= 1.0);
  if (!ok(0)) return 0;
  for (let it = 0; it < 12; it++) {
    const mid = (lo + hi) / 2;
    if (ok(mid)) lo = mid; else hi = mid;
  }
  return lo;
}

/** Essai indiciel hors ligne sur l’axe j (simulateur dédié). */
export function stepTest(params, q0, joint, ampRad, duration = 1.2) {
  const p = structuredClone(params);
  p.simulation.recordHz = 1000;
  p.simulation.plotWindow = duration + 0.5;
  p.safety.collisionDetection = false;
  p.safety.selfCollision = false;
  p.safety.floorCollision = false;
  p.poses.home = q0.slice();
  const sim = new Simulator(p);
  const dt = p.simulation.dt;
  const pre = 0.2;
  while (sim.t < pre) sim.step(dt);
  const target = q0.slice();
  target[joint] += ampRad;
  sim.hold = { q: target, qd: new Array(sim.n).fill(0), qdd: new Array(sim.n).fill(0) };
  sim.des = sim.hold;
  const t0 = sim.t;
  const T = [], Y = [], U = [];
  const y0 = sim.q[joint];
  while (sim.t < t0 + duration) {
    sim.step(dt);
    T.push(sim.t - t0);
    Y.push(sim.q[joint]);
    U.push(sim.tauT[joint]);
  }
  const yEnd = Y[Y.length - 1];
  const span = target[joint] - y0;
  const rel = (y) => (y - y0) / span;
  let t10 = null, t90 = null, peak = -Infinity;
  for (let k = 0; k < Y.length; k++) {
    const r = rel(Y[k]);
    if (t10 === null && r >= 0.1) t10 = T[k];
    if (t90 === null && r >= 0.9) t90 = T[k];
    peak = Math.max(peak, r);
  }
  let ts = 0;
  for (let k = Y.length - 1; k >= 0; k--) { if (Math.abs(rel(Y[k]) - 1) > 0.02) { ts = T[k]; break; } }
  return {
    T, Y: Y.map((y) => (y - y0) * R2D), ref: ampRad * R2D, U,
    rise: t10 !== null && t90 !== null ? t90 - t10 : null,
    overshoot: Math.max(0, (peak - 1) * 100),
    settle: ts,
    sse: (target[joint] - yEnd) * R2D,
    tauMax: Math.max(...U.map(Math.abs)),
    lost: sim.lostSteps[joint],
  };
}

export class AnalysisPanel {
  constructor(container, app) {
    this.app = app;
    this.c = container;
    this.build();
  }

  build() {
    const app = this.app;
    this.c.replaceChildren();
    // --- Dimensionnement
    this.sizeBox = h('div.dtable-wrap');
    this.payloadNote = h('p.sect__note', '');
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Dimensionnement des moteurs'), btn('Analyser', () => this.runSizing(), { cls: 'btn--sm btn--primary', icon: 'gauge' })),
      h('p.sect__note', 'Pire cas sur 2 500 configurations tirées dans les butées : gravité avec la charge nominale + inertie à accélération max + frottement sec, comparés au couple disponible en sortie de réducteur.'),
      this.sizeBox, this.payloadNote));
    // --- Réponse indicielle
    const jSel = h('select.field#step-joint', { 'aria-label': 'Axe à tester' }, ...Array.from({ length: app.sim.n }, (_, i) => h('option', { value: i }, `J${i + 1}`)));
    const amp = h('input.field.field--num#step-amp', { type: 'text', value: '2', inputmode: 'decimal', style: { width: '64px' }, 'aria-label': 'Amplitude en degrés' });
    this.stepCanvas = h('canvas', { style: { width: '100%', height: '150px', display: 'block' }, role: 'img', 'aria-label': 'Réponse indicielle' });
    this.stepMetrics = h('dl.metric');
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Réponse indicielle & auto-réglage')),
      h('p.sect__note', 'Applique un échelon de consigne sur un axe (autour de la position actuelle) avec le mode de commande courant, dans un simulateur séparé.'),
      h('div.row', jSel, amp, h('span.muted', '°'),
        btn('Tester', () => this.runStep(+jSel.value, parseFloat(amp.value) * D2R), { cls: 'btn--sm btn--primary', icon: 'play' }),
        btn('Auto-régler les gains', () => this.autoTune(+jSel.value, parseFloat(amp.value) * D2R), { cls: 'btn--sm', title: 'Placement de pôles : Kp = J·ω², Kd = 2ζ·J·ω, Ki = Kp·ω/10 (paramètres « Auto-réglage » de la section Commande)' })),
      this.stepCanvas, this.stepMetrics));
    // --- Espace de travail
    this.wsNote = h('p.sect__note', '');
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Espace de travail'),
        h('div.row', btn('Échantillonner', () => this.runWorkspace(), { cls: 'btn--sm', icon: 'cloud' }), btn('Masquer', () => { app.view.showWorkspace(null); this.wsNote.textContent = ''; }, { cls: 'btn--sm btn--ghost' }))),
      h('p.sect__note', 'Nuage de 20 000 positions du TCP ; couleur = manipulabilité (clair : proche d’une singularité, foncé : bonne dextérité).'),
      this.wsNote));
    // --- Efforts et singularités (temps réel)
    this.loadBox = h('div.dtable-wrap');
    this.singBox = h('dl.metric');
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Efforts dans les articulations')),
      h('p.sect__note', 'Efforts transmis par chaque articulation (Newton-Euler), utiles pour choisir les roulements : effort axial, radial, moment de basculement et couple moteur.'),
      this.loadBox));
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Conditionnement & singularités')),
      this.singBox));
  }

  runSizing() {
    const app = this.app;
    const res = sizingAnalysis(app.params);
    const t = h('table.dtable', h('thead', h('tr',
      h('th', 'Axe'), h('th', 'Gravité'), h('th', 'Pire cas'), h('th', 'Disponible'), h('th', 'Marge'), h('th', 'État'))));
    const tb = h('tbody');
    res.forEach((r) => {
      const cls = sevCls(r.margin);
      const rpm = `Moteur à vitesse max : ${r.motorRpm.toFixed(0)} tr/min${r.cornerRpm ? ` (vitesse de coin ${r.cornerRpm.toFixed(0)} tr/min)` : ''}. Couple disponible à vmax : ${r.capV.toFixed(1)} N·m.`;
      const row = h('tr', { tabIndex: 0 },
        h('td', `J${r.joint + 1}`), h('td', `${r.gMax.toFixed(2)} N·m`), h('td', `${r.dMax.toFixed(2)} N·m`),
        h('td', `${r.cap0.toFixed(1)} N·m`), h('td', `×${r.margin.toFixed(2)}`),
        h('td', h(`span.sev.sev--${cls}`, sevTxt[cls])));
      attachHelp(row, cls !== 'good' ? `${rpm} Réduction conseillée : ${Math.ceil((r.G * 1.5) / r.margin)}:1 (actuelle ${r.G}:1), ou moteur plus coupleux, ou accélérations réduites.` : rpm);
      tb.append(row);
    });
    t.append(tb);
    this.sizeBox.replaceChildren(t);
    this.payloadNote.textContent = 'Calcul de la charge utile maximale…';
    setTimeout(() => {
      const m = maxPayload(app.params);
      this.payloadNote.textContent = `Charge utile maximale estimée (marge statique ≥ 1,2) : ${m.toFixed(2)} kg — charge nominale déclarée : ${app.params.dynamics.payloadRated.toFixed(2)} kg.`;
    }, 30);
  }

  runStep(joint, amp) {
    if (!Number.isFinite(amp) || amp === 0) { toast('Amplitude invalide'); return; }
    const app = this.app;
    const r = stepTest(app.params, app.sim.q.slice(), joint, amp);
    this.lastStep = { joint, amp };
    this.drawStep(r);
    const f = (v, d = 1, u = '') => (v === null || !Number.isFinite(v) ? '—' : `${v.toFixed(d)}${u}`);
    this.stepMetrics.replaceChildren(
      h('div', h('dt', 'Montée 10–90 %'), h('dd', f(r.rise ? r.rise * 1000 : null, 0, ' ms'))),
      h('div', h('dt', 'Dépassement'), h('dd', f(r.overshoot, 1, ' %'))),
      h('div', h('dt', 'Stabilisation 2 %'), h('dd', f(r.settle * 1000, 0, ' ms'))),
      h('div', h('dt', 'Erreur finale'), h('dd', f(r.sse, 3, '°'))),
      h('div', h('dt', 'Couple max'), h('dd', f(r.tauMax, 2, ' N·m'))),
      h('div', h('dt', 'Pas perdus'), h('dd', String(r.lost))));
  }

  drawStep(r) {
    const cv = this.stepCanvas;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const W = cv.clientWidth || 300, H = cv.clientHeight || 150;
    cv.width = W * dpr; cv.height = H * dpr;
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const L = 44, Rm = 8, T = 8, B = 18;
    const tMax = r.T[r.T.length - 1];
    const ys = [...r.Y, r.ref, 0];
    let lo = Math.min(...ys), hi = Math.max(...ys);
    const pad = (hi - lo) * 0.1 || 0.1; lo -= pad; hi += pad;
    const X = (t) => L + (t / tMax) * (W - L - Rm);
    const Y = (v) => T + (1 - (v - lo) / (hi - lo)) * (H - T - B);
    ctx.strokeStyle = cssVar('--line'); ctx.lineWidth = 1;
    ctx.font = '11px ' + cssVar('--font-mono'); ctx.fillStyle = cssVar('--muted');
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    for (const v of [0, r.ref]) { const y = Math.round(Y(v)) + 0.5; ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(W - Rm, y); ctx.stroke(); ctx.fillText(`${v.toFixed(1)}°`, L - 4, y); }
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (let t = 0; t <= tMax + 1e-9; t += 0.2) { const x = X(t); ctx.fillText(`${t.toFixed(1)} s`, x, H - B + 3); }
    // Bande ±2 %
    ctx.fillStyle = cssVar('--accent-soft');
    ctx.fillRect(L, Y(r.ref * 1.02), W - L - Rm, Math.abs(Y(r.ref * 0.98) - Y(r.ref * 1.02)));
    ctx.strokeStyle = cssVar('--series-1'); ctx.lineWidth = 2; ctx.lineJoin = 'round';
    ctx.beginPath();
    r.T.forEach((t, k) => { const x = X(t), y = Y(r.Y[k]); if (k) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke();
  }

  autoTune(joint, amp) {
    const app = this.app;
    const gains = autoTune(app.sim.kin, app.params, app.sim.q, app.sim.actuators);
    gains.forEach((g, i) => {
      const c = app.params.control.joints[i];
      c.kp = +g.kp.toFixed(3); c.kd = +g.kd.toFixed(4); c.ki = +g.ki.toFixed(3); c.iLimit = +g.iLimit.toFixed(3);
      c.mitKp = +g.mitKp.toFixed(3); c.mitKd = +g.mitKd.toFixed(4);
    });
    app.onParamChange('control.joints');
    app.paramsPanel?.refresh();
    toast(`Gains recalculés (bande passante ${app.params.control.autotune.bandwidthHz} Hz, ζ = ${app.params.control.autotune.zeta})`);
    if (Number.isFinite(amp) && amp !== 0) this.runStep(joint, amp);
  }

  runWorkspace() {
    const app = this.app;
    const s = app.sim.kin.sampleWorkspace(20000, 5);
    app.view.showWorkspace(s);
    this.wsNote.textContent = `Allonge horizontale max ${(s.stats.reachMax * 1000).toFixed(0)} mm · hauteur TCP de ${(s.stats.zMin * 1000).toFixed(0)} à ${(s.stats.zMax * 1000).toFixed(0)} mm (repère base).`;
  }

  /** Mise à jour temps réel (≈ 4 Hz). */
  refresh() {
    const app = this.app, sim = app.sim;
    if (!this.c.offsetParent) return;
    const res = sim.dyn.rnea(sim.q, sim.v, sim.acc, true, null, null, true);
    const t = h('table.dtable', h('thead', h('tr', h('th', 'Axe'), h('th', 'Axial'), h('th', 'Radial'), h('th', 'Basculement'), h('th', 'Couple'))));
    const tb = h('tbody');
    res.wrenches.forEach((w, i) => {
      const fa = dot(w.f, w.z), fr = norm(sub(w.f, scale(w.z, fa)));
      const tq = dot(w.n, w.z), mt = norm(sub(w.n, scale(w.z, tq)));
      tb.append(h('tr', h('td', `J${i + 1}`), h('td', `${Math.abs(fa).toFixed(1)} N`), h('td', `${fr.toFixed(1)} N`), h('td', `${mt.toFixed(2)} N·m`), h('td', `${tq.toFixed(2)} N·m`)));
    });
    t.append(tb);
    this.loadBox.replaceChildren(t);
    const m = sim.kin.manipulability(sim.q);
    const items = [
      ['Manipulabilité', m.w.toExponential(2)],
      ['σ min', m.sigmaMin.toFixed(4)],
      ['Conditionnement', Number.isFinite(m.cond) ? m.cond.toFixed(1) : '∞'],
    ];
    if (sim.kin.analyticInfo.ok) {
      const t5 = sim.kin.dhVars(4, sim.q[4]).theta;
      items.push(['Poignet |sin θ5|', Math.abs(Math.sin(t5)).toFixed(3)]);
      const W = mat4Pos(sim.kin.fk(sim.q).frames[4]);
      const B = mat4Pos(sim.kin.base);
      items.push(['Centre poignet ↔ axe J1', `${(Math.hypot(W[0] - B[0], W[1] - B[1]) * 1000).toFixed(0)} mm`]);
    }
    this.singBox.replaceChildren(...items.map(([k, v]) => h('div', h('dt', k), h('dd', v))));
  }
}
