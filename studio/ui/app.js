// ORION Studio — application principale : disposition, boucle temps réel, commandes.

import { Simulator } from '../core/simulator.js';
import { defaultParams, SCHEMA_VERSION } from '../core/defaults.js';
import { ProgramRunner } from '../core/program.js';
import { tcpVector } from '../core/kinematics.js';
import { mat4FromRotPos, mat4Pos } from '../core/math3d.js';
import { CONTROL_MODES } from '../core/schema.js';
import { View3D } from './view3d.js';
import { JogPanel, ProgramPanel, AssistantPanel } from './panels.js';
import { ParamsPanel } from './params-panel.js';
import { AnalysisPanel } from './analysis.js';
import { Plot, PLOT_MODES } from './plots.js';
import { RobotLink, RobotPanel } from './serial.js';
import { buildHelp } from './help.js';
import { h, btn, icon, toast, store, debounce, saveFile } from './dom.js';

const R2D = 180 / Math.PI;
const I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];

/** Fusion profonde : complète `stored` avec les clés manquantes de `defaults`. */
function mergeDefaults(defaults, stored) {
  if (Array.isArray(defaults)) {
    if (!Array.isArray(stored)) return defaults;
    return defaults.length && typeof defaults[0] === 'object' && !Array.isArray(defaults[0])
      ? stored.map((s, i) => mergeDefaults(defaults[i] ?? defaults[defaults.length - 1], s))
      : stored;
  }
  if (defaults && typeof defaults === 'object') {
    const out = { ...defaults };
    for (const k of Object.keys(stored || {})) out[k] = k in defaults ? mergeDefaults(defaults[k], stored[k]) : stored[k];
    return out;
  }
  return stored === undefined ? defaults : stored;
}

export class App {
  constructor(root) {
    this.root = root;
    const stored = store.get('params');
    this.params = stored && stored.schemaVersion === SCHEMA_VERSION ? mergeDefaults(defaultParams(), stored) : defaultParams();
    this.objects = this.defaultObjects();
    this.sim = new Simulator(this.params, { objects: this.objects });
    this.link = new RobotLink(this);
    this.paused = false;
    this.runner = null;
    this.logLines = [];
    this.saveParams = debounce(() => store.set('params', this.params), 600);
    this.buildLayout();
    this.view = new View3D(this.viewport, this);
    this.view.buildRobot(this.sim.kin, this.params);
    this.buildPanels();
    this.bindKeys();
    this.observeTheme();
    this.sim.on('fault', (e) => { this.log(e.reason, 'error'); this.runner?.stop(); this.updateBanner(); });
    this.sim.on('stepLoss', (e) => this.logThrottled(`step-${e.joint}`, `Perte de pas sur J${e.joint + 1} (${e.total} pas au total) : couple insuffisant`, 'warn'));
    this.sim.on('grasp', (e) => this.log(`Objet saisi (${(e.object.mass * 1000).toFixed(0)} g)`, 'ok'));
    this.sim.on('release', () => this.log('Objet relâché'));
    this.log('ORION Studio prêt — simulation temps réel démarrée.', 'ok');
    this.last = performance.now();
    this.acc = { hud: 0, jog: 0, plot: 0, ana: 0, robot: 0, stream: 0 };
    requestAnimationFrame((t) => this.frame(t));
  }

  defaultObjects() {
    const cube = (id, x, y, size, mass, color) => ({ id, size, mass, color, pose: mat4FromRotPos(I3, [x, y, size / 2]), home: [x, y] });
    return [cube('cube-bleu', 0.26, -0.12, 0.03, 0.05, 0x2a78d6), cube('cube-orange', 0.2, -0.2, 0.03, 0.05, 0xeb6834), cube('cube-vert', 0.32, -0.21, 0.025, 0.035, 0x1baf7a)];
  }

  // ------------------------------------------------------------------ disposition
  buildLayout() {
    const r = this.root;
    r.classList.add('app');
    const prefs = store.get('layout', { left: true, right: true, bottom: true });
    this.prefs = prefs;
    r.classList.toggle('no-left', !prefs.left);
    r.classList.toggle('no-right', !prefs.right);
    r.classList.toggle('no-bottom', !prefs.bottom);
    // Barre de commande
    const override = h('input#override', { type: 'range', min: '5', max: '100', step: '5', value: String(Math.round(this.params.trajectory.speedOverride * 100)), 'aria-label': 'Vitesse globale' });
    const overrideOut = h('output', { for: 'override' }, `${override.value} %`);
    override.addEventListener('input', () => {
      this.params.trajectory.speedOverride = override.value / 100;
      overrideOut.textContent = `${override.value} %`;
      this.onParamChange('trajectory.speedOverride', true);
    });
    this.overrideInput = override; this.overrideOut = overrideOut;
    const modeSel = h('select.field#control-mode', { 'aria-label': 'Mode de commande', title: 'Mode de commande' },
      ...CONTROL_MODES.map(([v, l]) => h('option', { value: v }, l.split(' (')[0].split(' :')[0])));
    modeSel.value = this.params.control.mode;
    modeSel.addEventListener('change', () => { this.params.control.mode = modeSel.value; this.onParamChange('control.mode'); this.paramsPanel?.refresh(); this.log(`Mode de commande : ${modeSel.selectedOptions[0].textContent}`); });
    this.modeSel = modeSel;
    this.powerChip = h('span.chip', 'Puissance');
    this.stateChip = h('span.chip.chip--plain.hide-narrow', '');
    this.pauseBtn = btn('', () => { this.paused = !this.paused; this.pauseBtn.setAttribute('aria-pressed', String(this.paused)); this.pauseBtn.replaceChildren(icon(this.paused ? 'play' : 'pause')); }, { cls: 'btn--icon', icon: 'pause', title: 'Geler / reprendre la simulation', pressed: false });
    const themeBtn = btn('', () => this.toggleTheme(), { cls: 'btn--icon', icon: 'moon', title: 'Thème clair / sombre' });
    const layoutBtns = ['left', 'bottom', 'right'].map((side) => {
      const b = btn('', () => {
        prefs[side] = !prefs[side];
        r.classList.toggle(`no-${side}`, !prefs[side]);
        b.setAttribute('aria-pressed', String(prefs[side]));
        store.set('layout', prefs);
        setTimeout(() => this.view.resize(), 0);
      }, { cls: 'btn--icon btn--ghost', icon: side, title: `Afficher / masquer le panneau ${side === 'left' ? 'gauche' : side === 'right' ? 'droit' : 'du bas'}`, pressed: prefs[side] });
      return b;
    });
    this.estopBtn = h('button.estop', { type: 'button', title: 'Arrêt d’urgence (Échap)', 'aria-label': 'Arrêt d’urgence', onclick: () => this.emergencyStop() });
    this.nameEl = h('span.brand__sub', this.params.meta.name);
    const top = h('header.topbar',
      h('div.brand', h('span.brand__mark', 'ORION', h('b', '·'), 'STUDIO'), this.nameEl),
      h('div.topbar__group', modeSel),
      h('div.topbar__group.override', h('span.topbar__label', 'Vitesse'), override, overrideOut),
      h('div.topbar__group', this.powerChip, this.stateChip),
      h('div.topbar__spacer'),
      h('div.topbar__group.hide-narrow', ...layoutBtns),
      h('div.topbar__group', this.pauseBtn, themeBtn),
      this.estopBtn);
    // Vue 3D + HUD
    this.viewport = h('main.viewport', { 'aria-label': 'Simulation 3D' });
    this.hudPose = h('div.pose-grid');
    this.hudConfig = h('div.sect__note', '');
    this.manipFill = h('div.manip__fill');
    this.hudTime = h('span.chip.chip--plain', '');
    const hudTL = h('div.hud.hud--tl', h('div.hud-card', this.hudPose, this.hudConfig, h('div.manip', h('span.sect__note', 'Dextérité'), h('div.manip__bar', this.manipFill))));
    const tb = (ic, title, fn, pressed) => btn('', fn, { cls: 'btn--sm btn--icon', icon: ic, title, pressed });
    this.viewBtns = {};
    const toolbar = h('div.toolbar',
      tb('eye', 'Vue isométrique (1)', () => this.view.setView('iso')),
      h('button.btn.btn--sm', { type: 'button', onclick: () => this.view.setView('front'), title: 'Vue de face (2)' }, 'F'),
      h('button.btn.btn--sm', { type: 'button', onclick: () => this.view.setView('side'), title: 'Vue de côté (3)' }, 'C'),
      h('button.btn.btn--sm', { type: 'button', onclick: () => this.view.setView('top'), title: 'Vue de dessus (4)' }, 'D'),
      h('span.sep'),
      (this.viewBtns.move = tb('move', 'Gizmo : translation (T)', () => this.setGizmoMode('translate'), true)),
      (this.viewBtns.rot = tb('rotate', 'Gizmo : rotation (R)', () => this.setGizmoMode('rotate'), false)),
      (this.viewBtns.target = tb('target', 'Afficher le gizmo du TCP (G)', () => this.toggleView('gizmo'), true)),
      h('span.sep'),
      (this.viewBtns.trail = tb('path', 'Trace du TCP', () => this.toggleView('trail'), true)),
      (this.viewBtns.frames = tb('axes', 'Repères DH', () => this.toggleView('frames'), false)),
      (this.viewBtns.capsules = tb('capsule', 'Modèle de collision', () => this.toggleView('capsules'), false)),
      (this.viewBtns.shadows = tb('shadow', 'Ombres', () => this.toggleView('shadows'), true)));
    const hudTR = h('div.hud.hud--tr', toolbar, this.hudTime);
    const hudBL = h('div.hud.hud--bl.hide-narrow', h('span.sect__note', 'Maj + glisser sur le bras : pousser le robot'));
    this.banner = h('div.banner', { hidden: true, role: 'alert' });
    this.viewport.append(hudTL, hudTR, hudBL, this.banner);
    // Docks
    this.docks = {
      left: this.makeDock('left'),
      right: this.makeDock('right'),
      bottom: this.makeDock('bottom'),
      mobile: this.makeDock('mobile'),
    };
    r.append(top, this.docks.left.el, this.viewport, this.docks.right.el, this.docks.bottom.el, this.docks.mobile.el);
  }

  makeDock(side) {
    const tabs = h('div.tabs', { role: 'tablist' });
    const panels = h('div.tabpanels');
    const el = side === 'mobile' ? h('section.mobile-dock', tabs, panels) : h(`${side === 'bottom' ? 'section' : 'aside'}.dock.dock--${side}`, tabs, panels);
    return { el, tabs, panels, items: [] };
  }

  addPanel(dockName, id, title) {
    const dock = this.docks[dockName];
    const panel = h('div.tabpanel', { id: `panel-${id}`, role: 'tabpanel', 'aria-label': title });
    const tab = h('button.tab', { type: 'button', role: 'tab', id: `tab-${id}`, 'aria-controls': `panel-${id}` }, title);
    const item = { id, title, panel, tab, home: dockName };
    tab.addEventListener('click', () => this.selectTab(item));
    dock.items.push(item);
    dock.tabs.append(tab);
    dock.panels.append(panel);
    this.panelItems = this.panelItems || [];
    this.panelItems.push(item);
    return panel;
  }

  selectTab(item) {
    const dock = Object.values(this.docks).find((d) => d.items.includes(item));
    for (const it of dock.items) {
      const on = it === item;
      it.tab.setAttribute('aria-selected', String(on));
      it.panel.hidden = !on;
    }
    store.set(`tab.${dock === this.docks.mobile ? 'mobile' : item.home}`, item.id);
    if (item.id === 'plots') setTimeout(() => this.plot.draw(true), 0);
  }

  /** Écrans étroits : tous les panneaux passent dans un seul dock à onglets sous la vue 3D. */
  applyResponsive() {
    const narrow = window.matchMedia('(max-width: 1100px)').matches;
    if (narrow === this.narrow) return;
    this.narrow = narrow;
    const mob = this.docks.mobile;
    if (narrow) {
      for (const it of this.panelItems) {
        const d = this.docks[it.home];
        d.items = d.items.filter((x) => x !== it);
        mob.items.push(it);
        mob.tabs.append(it.tab);
        mob.panels.append(it.panel);
      }
      this.selectTab(mob.items.find((i) => i.id === store.get('tab.mobile', 'jog')) || mob.items[0]);
    } else {
      mob.items = [];
      for (const it of this.panelItems) {
        const d = this.docks[it.home];
        d.items.push(it);
        d.tabs.append(it.tab);
        d.panels.append(it.panel);
      }
      for (const side of ['left', 'right', 'bottom']) {
        const d = this.docks[side];
        const want = store.get(`tab.${side}`);
        this.selectTab(d.items.find((i) => i.id === want) || d.items[0]);
      }
    }
    setTimeout(() => this.view.resize(), 0);
  }

  buildPanels() {
    this.jogPanel = new JogPanel(this.addPanel('left', 'jog', 'Pilotage'), this);
    this.programPanel = new ProgramPanel(this.addPanel('left', 'program', 'Programme'), this);
    this.assistant = new AssistantPanel(this.addPanel('left', 'assistant', 'Assistant IA'), this);
    this.paramsPanel = new ParamsPanel(this.addPanel('right', 'params', 'Paramètres'), this);
    this.analysis = new AnalysisPanel(this.addPanel('right', 'analysis', 'Analyse'), this);
    this.partsHost = this.addPanel('right', 'parts', 'Pièces 3D');
    this.partsHost.append(h('p.sect__note', 'Génération des pièces imprimables en cours…'));
    buildHelp(this.addPanel('right', 'help', 'Aide'));
    // Courbes
    const plotHost = this.addPanel('bottom', 'plots', 'Courbes');
    const modeSeg = h('div.seg');
    const modeBtns = Object.entries(PLOT_MODES).map(([k, m]) => {
      const b = h('button', { type: 'button', 'aria-pressed': 'false', onclick: () => { this.plot.setMode(k); modeBtns.forEach((x) => x.setAttribute('aria-pressed', String(x === b))); store.set('plotMode', k); } }, m.label);
      modeSeg.append(b);
      return b;
    });
    const bar = h('div.plotbar', h('div', { style: { overflowX: 'auto', maxWidth: '100%' } }, modeSeg), h('span.grow'),
      btn('', () => { this.plot.paused = !this.plot.paused; }, { cls: 'btn--sm btn--icon', icon: 'pause', title: 'Figer les courbes' }),
      btn('CSV', () => saveFile('telemetrie.csv', this.sim.telemetry.toCSV(), 'text/csv'), { cls: 'btn--sm', icon: 'download', title: 'Exporter la télémétrie (CSV)' }),
      btn('', () => { this.sim.telemetry.clear(); this.view.clearTrail(); }, { cls: 'btn--sm btn--icon', icon: 'reset', title: 'Effacer' }));
    plotHost.append(bar);
    this.plot = new Plot(plotHost, this);
    plotHost.append(this.plot.legend);
    const pm = store.get('plotMode', 'pos');
    this.plot.setMode(PLOT_MODES[pm] ? pm : 'pos');
    modeBtns[Object.keys(PLOT_MODES).indexOf(this.plot.mode)].setAttribute('aria-pressed', 'true');
    // Robot réel & journal
    this.robotPanel = new RobotPanel(this.addPanel('bottom', 'robot', 'Robot réel'), this);
    const logHost = this.addPanel('bottom', 'log', 'Journal');
    this.logEl = h('div.log', { role: 'log' });
    logHost.append(this.logEl);
    this.logLines.forEach((l) => this.logEl.append(l));
    this.applyResponsive();
    window.matchMedia('(max-width: 1100px)').addEventListener('change', () => this.applyResponsive());
    if (!this.narrow) {
      for (const side of ['left', 'right', 'bottom']) {
        const d = this.docks[side];
        this.selectTab(d.items.find((i) => i.id === store.get(`tab.${side}`)) || d.items[0]);
      }
    }
  }

  // ------------------------------------------------------------------ journal
  log(msg, level = 'info') {
    const t = this.sim ? this.sim.t : 0;
    const line = h('div.log__line', { dataset: { level } }, h('time', `${t.toFixed(2)} s`), h('span', msg));
    this.logLines.push(line);
    if (this.logLines.length > 400) this.logLines.shift().remove();
    if (this.logEl) { this.logEl.append(line); this.logEl.parentElement.scrollTop = this.logEl.parentElement.scrollHeight; }
    if (level === 'error') toast(msg, 4000);
  }

  logThrottled(key, msg, level) {
    this.throttle = this.throttle || {};
    const now = performance.now();
    if (this.throttle[key] && now - this.throttle[key] < 1500) return;
    this.throttle[key] = now;
    this.log(msg, level);
  }

  monitor(line, dir) { this.log(`${dir === 'tx' ? '→' : '←'} ${line}`, dir === 'tx' ? 'tx' : dir === 'error' ? 'error' : 'info'); }

  // ------------------------------------------------------------------ paramètres
  onParamChange(path, quiet = false) {
    const structural = /^(kinematics|gripper|safety\.linkRadii|limits\.joints|actuators|poses)/.test(path);
    try {
      this.sim.setParams(this.params);
    } catch (e) {
      this.log(`Paramètre refusé (${path}) : ${e.message}`, 'error');
      return;
    }
    if (/^(kinematics|gripper|safety\.linkRadii)/.test(path)) {
      this.view.buildRobot(this.sim.kin, this.params, this.cadForView());
      this.onGeometryChanged?.();
    }
    if (structural) this.jogPanel.build();
    if (path === 'meta.name') this.nameEl.textContent = this.params.meta.name;
    if (path === 'control.mode') this.modeSel.value = this.params.control.mode;
    if (path === 'trajectory.speedOverride') {
      this.overrideInput.value = String(Math.round(this.params.trajectory.speedOverride * 100));
      this.overrideOut.textContent = `${this.overrideInput.value} %`;
    }
    this.saveParams();
    if (!quiet) this.stateDirty = true;
  }

  loadParams(p, msg) {
    this.params = mergeDefaults(defaultParams(), p);
    this.sim.setParams(this.params, { reset: true });
    this.view.buildRobot(this.sim.kin, this.params, this.cadForView());
    this.onGeometryChanged?.();
    this.paramsPanel.build();
    this.jogPanel.build();
    this.modeSel.value = this.params.control.mode;
    this.nameEl.textContent = this.params.meta.name;
    this.overrideInput.value = String(Math.round(this.params.trajectory.speedOverride * 100));
    this.overrideOut.textContent = `${this.overrideInput.value} %`;
    this.saveParams();
    if (msg) { this.log(msg, 'ok'); toast(msg); }
  }

  resetParams() { this.loadParams(defaultParams(), 'Paramètres réinitialisés (ORION-6 MAKER)'); }

  cadForView() { return this.cad && this.cad.compatible(this.params) ? this.cad.viewData : null; }

  // ------------------------------------------------------------------ commandes robot
  guard(fn) {
    try { return fn(); } catch (e) { this.log(e.message, 'error'); return null; }
  }

  setPower(on) {
    this.sim.setEnabled(on);
    if (this.link.connected) this.link.cmd('EN', [on ? 1 : 0]);
    this.log(on ? 'Moteurs sous tension' : 'Moteurs hors tension (le bras peut retomber sous son poids)', on ? 'ok' : 'warn');
  }

  emergencyStop() {
    this.sim.estop('Arrêt d’urgence déclenché par l’opérateur');
    this.runner?.stop();
    if (this.link.connected) this.link.cmd('ESTOP');
    this.estopBtn.classList.add('estop--latched');
    this.updateBanner();
  }

  clearFault() {
    this.sim.clearFault();
    if (this.link.connected) this.link.cmd('CLR');
    this.estopBtn.classList.remove('estop--latched');
    this.updateBanner();
    this.log('Défaut acquitté', 'ok');
  }

  updateBanner() {
    const f = this.sim.fault;
    this.banner.hidden = !f;
    if (f) this.banner.replaceChildren(h('span', f), btn('Acquitter', () => this.clearFault(), { cls: 'btn--sm' }));
  }

  moveJoints(q, opts = {}) { return this.guard(() => this.sim.moveJ(q, opts)); }

  moveNamed(name) {
    const q = this.params.poses[name];
    if (!q) return;
    this.moveJoints(q);
  }

  setHomeHere() {
    this.params.poses.home = this.sim.q.map((v) => +v.toFixed(5));
    this.onParamChange('poses.home');
    this.paramsPanel.refresh();
    toast('Pose Home mémorisée');
  }

  servoTo(q) {
    if (!this.sim.enabled) { toast('Activez les moteurs pour piloter le robot'); return; }
    if (this.sim.fault) { toast('Acquittez le défaut avant de reprendre'); return; }
    if (this.runner && this.runner.state === 'running') this.stopProgram();
    this.sim.servoTo(q);
  }

  moveToPose(T, kind) {
    return this.guard(() => {
      if (kind === 'l') return this.sim.moveL(T);
      const r = this.sim.kin.solveIK(T, { seed: this.sim.plannedEnd() });
      if (!r.ok) throw new Error(`Pose inatteignable : ${r.message || 'hors espace de travail'}`);
      return this.sim.moveJ(r.q);
    });
  }

  setGripper(v) {
    this.sim.setGripper(v);
    if (this.link.connected) this.link.cmd('GRIP', [Math.round(v * 100)]);
  }

  onIO(k, v) { if (this.link.connected) this.link.cmd('DO', [k, v ? 1 : 0]); }

  addCube() {
    const k = this.objects.length;
    const size = 0.025 + (k % 3) * 0.005;
    const x = 0.18 + (k % 4) * 0.05, y = 0.08 + Math.floor(k / 4) * 0.05;
    this.objects.push({ id: `cube-${Date.now()}`, size, mass: 0.04, pose: mat4FromRotPos(I3, [x, y, size / 2]), home: [x, y] });
    this.log(`Cube ajouté en (${(x * 1000).toFixed(0)}, ${(y * 1000).toFixed(0)}) mm`);
  }

  resetScene() {
    if (this.sim.heldObject) this.sim.releaseObject();
    const def = this.defaultObjects();
    this.objects.splice(0, this.objects.length, ...def);
    this.sim.objects = this.objects;
    this.view.objectsLayer.clear();
  }

  previewGhost(q) {
    this.ghostQ = q;
  }

  showMeasured(q) { this.measuredQ = q; }

  knownPoints() {
    const pts = [...this.programPanel.ta.value.matchAll(/POINT\s+([A-Za-z_]\w*)/gi)].map((m) => m[1].toUpperCase());
    return pts;
  }

  // ------------------------------------------------------------------ programmes
  runProgram(text, { fromEditor = true } = {}) {
    if (this.runner && ['running', 'paused', 'userpause'].includes(this.runner.state)) this.runner.stop();
    if (!this.sim.enabled) { toast('Activez les moteurs avant d’exécuter un programme'); return; }
    if (this.sim.fault) { toast('Acquittez le défaut avant d’exécuter un programme'); return; }
    const t0 = this.sim.t;
    const panel = this.programPanel;
    const runner = new ProgramRunner(this.sim, text, {}, {
      log: (m) => this.log(`Programme : ${m}`, 'ok'),
      onLine: (l) => { if (fromEditor) panel.setCurrentLine(l); },
      onDone: () => { panel.setStatus(`Terminé en ${(this.sim.t - t0).toFixed(1)} s.`); if (fromEditor) panel.setCurrentLine(0); this.log('Programme terminé', 'ok'); },
      onError: (e) => { panel.setStatus(`Erreur — ${e.message}`); this.log(e.message, 'error'); },
      onPause: () => panel.setStatus('En pause (instruction PAUSE) — cliquez sur Pause pour reprendre.'),
      onIO: (pin, v) => this.onIO(pin, v),
    });
    if (runner.errors.length) {
      panel.setStatus(`${runner.errors.length} erreur(s) de syntaxe — corrigez avant d’exécuter.`);
      this.log(runner.errors[0].message, 'error');
      return;
    }
    this.runner = runner;
    runner.start();
    panel.setStatus(fromEditor ? 'Exécution en cours…' : 'Exécution d’une commande de l’assistant…');
  }

  pauseProgram() {
    const r = this.runner;
    if (!r) return;
    if (r.state === 'running') { r.pause(); this.sim.stopMotion(); this.programPanel.setStatus('En pause (le mouvement en cours est freiné).'); }
    else if (r.state === 'paused' || r.state === 'userpause') { r.resume(); this.programPanel.setStatus('Reprise…'); }
  }

  stopProgram() {
    if (this.runner) { this.runner.stop(); this.programPanel.setStatus('Arrêté.'); this.programPanel.setCurrentLine(0); }
  }

  executeLines(lines) {
    // Les lignes de l’assistant peuvent référencer les points définis dans l’éditeur
    const defs = this.programPanel.ta.value.split('\n').filter((l) => /^\s*POINT\s/i.test(l));
    this.runProgram([...defs, ...lines].join('\n'), { fromEditor: false });
  }

  appendToProgram(lines) {
    const t = this.programPanel.ta.value.replace(/\s*$/, '');
    this.programPanel.setText(`${t}\n${lines.join('\n')}\n`);
    this.selectTab(this.panelItems.find((i) => i.id === 'program'));
    toast('Lignes ajoutées au programme');
  }

  doAction(a) {
    switch (a.type) {
      case 'stop': this.runner?.stop(); this.sim.stopMotion(); break;
      case 'estop': this.emergencyStop(); break;
      case 'enable': this.setPower(a.value); break;
      case 'mode':
        this.params.control.mode = a.value;
        this.onParamChange('control.mode');
        this.paramsPanel.refresh();
        break;
      case 'homing':
        if (this.link.connected) this.link.cmd('HOME');
        else { this.log('Prise d’origine simulée : retour en Home'); this.moveNamed('home'); }
        break;
      default: break;
    }
  }

  // ------------------------------------------------------------------ gizmo 3D
  onTargetDragStart() { this.dragActive = true; if (this.runner?.state === 'running') this.stopProgram(); }

  onTargetDrag(T) {
    const kin = this.sim.kin;
    const r = kin.solveIK(T, { seed: this.sim.des.q, restarts: 0 });
    this.dragIK = r;
    if (r.ok) {
      this.servoTo(r.q);
      this.ghostQ = null;
    } else {
      this.ghostQ = r.q;
      this.logThrottled('ik', `Cible inatteignable (${r.message || 'hors espace de travail'})`, 'warn');
    }
  }

  onTargetDragEnd() { this.dragActive = false; this.ghostQ = null; }

  onPush(push) { this.sim.setPush(push); }

  setGizmoMode(m) {
    this.view.setGizmoMode(m);
    this.viewBtns.move.setAttribute('aria-pressed', String(m === 'translate'));
    this.viewBtns.rot.setAttribute('aria-pressed', String(m === 'rotate'));
  }

  toggleView(opt) {
    const v = this.view;
    const cur = { gizmo: v.options.gizmo, trail: v.options.trail, frames: v.frameHelpers.visible, capsules: v.capsuleLayer.visible, shadows: v.options.shadows }[opt];
    const on = !cur;
    if (opt === 'gizmo') { v.setGizmoVisible(on); this.viewBtns.target.setAttribute('aria-pressed', String(on)); }
    if (opt === 'trail') { v.setTrailVisible(on); this.viewBtns.trail.setAttribute('aria-pressed', String(on)); }
    if (opt === 'frames') { v.frameHelpers.visible = on; this.viewBtns.frames.setAttribute('aria-pressed', String(on)); }
    if (opt === 'capsules') { v.capsuleLayer.visible = on; this.viewBtns.capsules.setAttribute('aria-pressed', String(on)); }
    if (opt === 'shadows') { v.setShadows(on); this.viewBtns.shadows.setAttribute('aria-pressed', String(on)); }
  }

  // ------------------------------------------------------------------ thème
  toggleTheme() {
    const rootEl = document.documentElement;
    const dark = getComputedStyle(rootEl).colorScheme.includes('dark');
    rootEl.dataset.theme = dark ? 'light' : 'dark';
    store.set('theme', rootEl.dataset.theme);
  }

  observeTheme() {
    const t = store.get('theme');
    if (t && !window.ORION_EMBEDDED) document.documentElement.dataset.theme = t;
    const apply = () => setTimeout(() => { this.view.applyTheme(); this.plot?.draw(true); }, 0);
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', apply);
    new MutationObserver(apply).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
    apply();
  }

  // ------------------------------------------------------------------ clavier
  bindKeys() {
    window.addEventListener('keydown', (e) => {
      const tag = (e.target.tagName || '').toLowerCase();
      if (['input', 'textarea', 'select'].includes(tag) || e.target.isContentEditable) {
        if (e.key === 'Escape') this.emergencyStop();
        return;
      }
      if (e.key === 'Escape') { this.emergencyStop(); return; }
      if (e.key === ' ') { e.preventDefault(); this.runner?.stop(); this.sim.stopMotion(); return; }
      if (e.key === 't' || e.key === 'T') this.setGizmoMode('translate');
      if (e.key === 'r' || e.key === 'R') this.setGizmoMode('rotate');
      if (e.key === 'g' || e.key === 'G') this.toggleView('gizmo');
      const views = { 1: 'iso', 2: 'front', 3: 'side', 4: 'top' };
      if (views[e.key]) this.view.setView(views[e.key]);
    });
  }

  // ------------------------------------------------------------------ boucle
  frame(now) {
    const dt = Math.min(0.1, (now - this.last) / 1000);
    this.last = now;
    const sim = this.sim;
    if (!this.paused) {
      try { sim.advance(dt); } catch (e) { this.log(`Simulation : ${e.message}`, 'error'); this.paused = true; }
    }
    if (this.runner) {
      const st = this.runner.tick();
      if (st === 'running' && this.runner.currentLine) this.programPanel.setCurrentLine(this.runner.currentLine);
    }
    const fk = sim.kin.fk(sim.q);
    this.view.updateRobot(fk, sim.gripper.pos);
    this.view.syncObjects(this.objects);
    this.view.pushTrail(mat4Pos(fk.tcp));
    if (!this.dragActive) this.view.setTargetFromRowMajor(sim.kin.tcp(sim.des.q));
    // Fantôme : solution IK hors d’atteinte, aperçu, ou position mesurée du robot réel
    const gq = this.ghostQ || this.measuredQ;
    if (gq) { this.view.updateGhost(sim.kin.fk(gq).frames, true); this.view.setGhostColor(!this.ghostQ || !!this.dragIK?.ok); } else this.view.updateGhost(null, false);
    const a = this.acc;
    a.hud += dt; a.jog += dt; a.plot += dt; a.ana += dt; a.robot += dt; a.stream += dt;
    if (a.hud > 0.1) { a.hud = 0; this.updateHud(fk); }
    if (a.jog > 0.066) { a.jog = 0; this.jogPanel.refresh(); }
    if (a.plot > 0.033) { a.plot = 0; if (this.plot.canvas.offsetParent) this.plot.draw(); }
    if (a.ana > 0.25) {
      a.ana = 0;
      this.analysis.refresh();
      if (this.view.capsuleLayer.visible) {
        const c = sim.collision.check(sim.q, fk);
        this.view.showCapsules(c.caps, new Set([...c.self.flatMap((x) => [x.a, x.b]), ...c.floor.map((x) => x.link)]));
      }
    }
    if (a.robot > 0.2) { a.robot = 0; this.robotPanel.refresh(); }
    const sHz = this.params.hardware.streamHz || 100;
    if (a.stream > 1 / sHz) { a.stream = 0; this.link.tick(); }
    this.view.render();
    requestAnimationFrame((t) => this.frame(t));
  }

  updateHud(fk) {
    const sim = this.sim;
    const v = tcpVector(fk.tcp);
    const cells = [['X', v[0] * 1000, 'mm'], ['Rx', v[3] * R2D, '°'], ['Y', v[1] * 1000, 'mm'], ['Ry', v[4] * R2D, '°'], ['Z', v[2] * 1000, 'mm'], ['Rz', v[5] * R2D, '°']];
    this.hudPose.replaceChildren(...cells.flatMap(([k, val, u]) => [h('span', k), h('span', `${val.toFixed(u === 'mm' ? 1 : 1)} ${u}`)]));
    if (sim.kin.analyticInfo.ok) this.hudConfig.textContent = sim.kin.configOf(sim.q).label;
    const m = sim.kin.manipulability(sim.q, fk);
    const ref = this.manipRef || (this.manipRef = Math.max(1e-9, sim.kin.manipulability(this.params.poses.home).w * 2.2));
    this.manipFill.style.width = `${Math.min(100, (m.w / ref) * 100).toFixed(0)}%`;
    this.manipFill.style.background = m.sigmaMin < 0.01 ? 'var(--critical)' : m.sigmaMin < 0.03 ? 'var(--warning-fill)' : 'var(--accent)';
    this.powerChip.textContent = sim.enabled ? 'Moteurs ON' : 'Moteurs OFF';
    this.powerChip.className = `chip ${sim.enabled ? 'chip--good' : 'chip--warn'}`;
    const running = this.runner && this.runner.state === 'running';
    this.stateChip.textContent = sim.fault ? 'Défaut' : running ? 'Programme' : sim.executor.busy ? 'Mouvement' : sim.jog ? 'Jog' : 'Arrêt';
    this.hudTime.textContent = `t = ${sim.t.toFixed(1)} s${this.paused ? ' · figé' : ''}${this.link.connected ? ' · robot connecté' : ' · simulation'}`;
    if (!this.banner.hidden !== !!sim.fault) this.updateBanner();
  }
}
