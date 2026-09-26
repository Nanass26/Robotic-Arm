// ORION Studio — panneaux Pilotage, Programme et Assistant.

import { h, btn, icon, toast, attachHelp, saveFile, pickFile, store } from './dom.js';
import { DEMO_PROGRAMS, parseProgram } from '../core/program.js';
import { interpret, NLP_EXAMPLES } from '../core/nlp.js';
import { tcpVector } from '../core/kinematics.js';
import { rpyToMat3, mat4FromRotPos, mat4Pos } from '../core/math3d.js';

const R2D = 180 / Math.PI, D2R = Math.PI / 180;

/** Bouton « maintenir pour jogger » (souris et tactile). */
function holdButton(label, onStart, onStop, title) {
  const b = h('button.btn', { type: 'button', title: title || '', 'aria-label': title || label }, label);
  let active = false;
  b.addEventListener('pointerdown', (e) => { e.preventDefault(); b.setPointerCapture(e.pointerId); active = true; onStart(); });
  const stop = () => { if (active) { active = false; onStop(); } };
  b.addEventListener('pointerup', stop);
  b.addEventListener('pointercancel', stop);
  b.addEventListener('lostpointercapture', stop);
  b.addEventListener('keydown', (e) => { if ((e.key === ' ' || e.key === 'Enter') && !active) { e.preventDefault(); active = true; onStart(); } });
  b.addEventListener('keyup', (e) => { if (e.key === ' ' || e.key === 'Enter') stop(); });
  return b;
}

// ====================================================================== Pilotage
export class JogPanel {
  constructor(container, app) {
    this.app = app;
    this.c = container;
    this.frame = 'base';
    this.build();
  }

  build() {
    const app = this.app, sim = app.sim;
    const n = sim.n;
    this.c.replaceChildren();
    // --- Puissance & poses
    this.powerBtn = btn('Moteurs', () => app.setPower(!sim.enabled), { cls: 'btn--sm', icon: 'power', pressed: sim.enabled, title: 'Mettre les moteurs sous/hors tension' });
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Poses'), this.powerBtn),
      h('div.row',
        btn('Home', () => app.moveNamed('home'), { cls: 'btn--sm', icon: 'home' }),
        btn('Repos', () => app.moveNamed('rest'), { cls: 'btn--sm' }),
        btn('Zéro', () => app.moveNamed('zero'), { cls: 'btn--sm' }),
        btn('Définir Home ici', () => app.setHomeHere(), { cls: 'btn--sm btn--ghost', title: 'Mémorise la position actuelle comme pose Home' }))));

    // --- Articulations
    const jointsBox = h('div.stack');
    this.joints = [];
    for (let i = 0; i < n; i++) {
      const lim = app.params.limits.joints[i];
      const name = app.params.kinematics.joints[i].name || `J${i + 1}`;
      const val = h('div.joint__val', '0.00°');
      const slider = h('input', { type: 'range', min: (lim.min * R2D).toFixed(1), max: (lim.max * R2D).toFixed(1), step: '0.1', 'aria-label': `Consigne ${name}`, id: `jog-slider-${i}` });
      slider.addEventListener('pointerdown', () => { this.dragging = i; });
      slider.addEventListener('pointerup', () => { this.dragging = null; });
      slider.addEventListener('input', () => {
        const q = sim.des.q.slice();
        q[i] = parseFloat(slider.value) * D2R;
        app.servoTo(q);
      });
      const load = h('div.load__fill');
      const tgt = h('span.num', '');
      const lost = h('span.num', '');
      const row = h('div.joint',
        h('div.joint__name', `J${i + 1}`, h('small', name.replace(/^J\d\s*/, ''))),
        h('div.joint__ctrl',
          holdButton('−', () => sim.startJog({ type: 'joint', joint: i, dir: -1 }), () => sim.stopJog(), `${name} −`),
          slider,
          holdButton('+', () => sim.startJog({ type: 'joint', joint: i, dir: 1 }), () => sim.stopJog(), `${name} +`)),
        val,
        h('div.joint__meta', h('span', 'charge'), h('div.load', load), tgt, lost));
      row.querySelectorAll('.btn').forEach((b) => { b.classList.add('btn--sm', 'btn--icon'); });
      attachHelp(load.parentElement, 'Charge moteur : part du couple disponible utilisée (pas-à-pas : |sin| de l’angle de charge ; > 80 % = risque de perte de pas).');
      jointsBox.append(row);
      this.joints.push({ val, slider, load, tgt, lost });
    }
    this.c.append(h('div.sect', h('div.sect__head', h('h3.sect__title', 'Articulations'), h('span.sect__note', 'maintenir − / + pour jogger')), jointsBox));

    // --- Cartésien
    const segBase = h('button', { type: 'button', 'aria-pressed': 'true' }, 'Base');
    const segTool = h('button', { type: 'button', 'aria-pressed': 'false' }, 'Outil');
    const setFrame = (f) => { this.frame = f; segBase.setAttribute('aria-pressed', String(f === 'base')); segTool.setAttribute('aria-pressed', String(f === 'tool')); };
    segBase.onclick = () => setFrame('base');
    segTool.onclick = () => setFrame('tool');
    const grid = h('div.jog-grid');
    const axes = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz'];
    for (const dir of [-1, 1]) {
      axes.forEach((a, k) => {
        grid.append(holdButton(`${a}${dir > 0 ? '+' : '−'}`,
          () => sim.startJog({ type: 'cart', axis: k, dir, frame: this.frame }),
          () => sim.stopJog(), `Jog ${a} ${dir > 0 ? 'positif' : 'négatif'} (repère ${this.frame === 'base' ? 'base' : 'outil'})`));
      });
    }
    // Aller à une pose
    this.poseIn = {};
    const pin = h('div.pose-inputs');
    for (const [k, lab] of [['x', 'X mm'], ['y', 'Y mm'], ['z', 'Z mm'], ['rx', 'Rx °'], ['ry', 'Ry °'], ['rz', 'Rz °']]) {
      const f = h('input.field.field--num', { type: 'text', inputmode: 'decimal', id: `goto-${k}` });
      this.poseIn[k] = f;
      pin.append(h('label', { for: `goto-${k}` }, lab, f));
    }
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Cartésien (TCP)'), h('div.seg', segBase, segTool)),
      grid,
      pin,
      h('div.row',
        btn('Position actuelle', () => this.fillPose(), { cls: 'btn--sm btn--ghost' }),
        h('span.grow'),
        btn('MoveJ', () => this.gotoPose('j'), { cls: 'btn--sm' }),
        btn('MoveL', () => this.gotoPose('l'), { cls: 'btn--sm btn--primary' }))));
    this.fillPose();

    // --- Solutions IK
    this.solBox = h('div.solutions');
    this.solNote = h('span.sect__note', '');
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Solutions du modèle inverse'), btn('Calculer', () => this.computeSolutions(), { cls: 'btn--sm btn--ghost' })),
      this.solNote, this.solBox));

    // --- Pince
    this.gripSlider = h('input.grow', { type: 'range', min: '0', max: '100', step: '1', value: '100', id: 'grip-slider', 'aria-label': 'Ouverture de la pince' });
    this.gripSlider.addEventListener('input', () => app.setGripper(this.gripSlider.value / 100));
    this.gripState = h('span.sect__note', '');
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Pince'), this.gripState),
      h('div.row.row--nowrap', btn('Fermer', () => app.setGripper(0), { cls: 'btn--sm' }), this.gripSlider, btn('Ouvrir', () => app.setGripper(1), { cls: 'btn--sm' }))));

    // --- E/S et scène
    const dos = h('div.io-grid'), dis = h('div.io-grid');
    this.ioBtns = { do: [], di: [] };
    for (let k = 0; k < 8; k++) {
      const bo = h('button', { type: 'button', 'aria-pressed': 'false', title: `Sortie ${k}` }, `${k}`);
      bo.onclick = () => { sim.io.do[k] = !sim.io.do[k]; app.onIO?.(k, sim.io.do[k]); };
      const bi = h('button', { type: 'button', 'aria-pressed': 'false', title: `Entrée simulée ${k}` }, `${k}`);
      bi.onclick = () => { sim.io.di[k] = !sim.io.di[k]; };
      dos.append(bo); dis.append(bi);
      this.ioBtns.do.push(bo); this.ioBtns.di.push(bi);
    }
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Entrées / sorties')),
      h('div.row.row--nowrap', h('span.sect__note', { style: { width: '5.5em' } }, 'Sorties'), h('div.grow', dos)),
      h('div.row.row--nowrap', h('span.sect__note', { style: { width: '5.5em' } }, 'Entrées'), h('div.grow', dis))));
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Scène')),
      h('div.row',
        btn('Ajouter un cube', () => app.addCube(), { cls: 'btn--sm', icon: 'cube' }),
        btn('Replacer les cubes', () => app.resetScene(), { cls: 'btn--sm btn--ghost' }))));
  }

  fillPose() {
    const v = tcpVector(this.app.sim.tcp);
    const fmt = (x) => String(Number(x.toFixed(2)));
    this.poseIn.x.value = fmt(v[0] * 1000); this.poseIn.y.value = fmt(v[1] * 1000); this.poseIn.z.value = fmt(v[2] * 1000);
    this.poseIn.rx.value = fmt(v[3] * R2D); this.poseIn.ry.value = fmt(v[4] * R2D); this.poseIn.rz.value = fmt(v[5] * R2D);
  }

  gotoPose(kind) {
    const g = (k) => parseFloat(String(this.poseIn[k].value).replace(',', '.'));
    const vals = ['x', 'y', 'z', 'rx', 'ry', 'rz'].map(g);
    if (vals.some((v) => Number.isNaN(v))) { toast('Pose incomplète'); return; }
    const T = mat4FromRotPos(rpyToMat3(vals[3] * D2R, vals[4] * D2R, vals[5] * D2R), [vals[0] / 1000, vals[1] / 1000, vals[2] / 1000]);
    this.app.moveToPose(T, kind);
  }

  computeSolutions() {
    const app = this.app, kin = app.sim.kin;
    const T = app.sim.tcp;
    this.solBox.replaceChildren();
    if (!kin.analyticInfo.ok) {
      this.solNote.textContent = `Solution analytique indisponible (${kin.analyticInfo.reasons.join(', ')}) — le solveur numérique est utilisé.`;
      return;
    }
    const sols = kin.ikAnalyticAll(T, app.sim.q);
    this.solNote.textContent = `${sols.length} configurations pour la pose actuelle du TCP (${sols.filter((s) => s.inLimits).length} dans les butées).`;
    sols.forEach((s, k) => {
      const d = kin.jointDistance(s.q, app.sim.q);
      const cur = d < 1e-3;
      const b = h('button.solution', { type: 'button', 'aria-current': String(cur), dataset: { bad: String(!s.inLimits) } },
        h('b', `${k + 1}`),
        h('span', s.config.label),
        h('span.muted.num', s.inLimits ? `${(d * R2D).toFixed(0)}°` : 'hors butées'));
      b.title = s.q.map((v) => `${(v * R2D).toFixed(1)}°`).join('  ');
      b.onclick = () => { if (!s.inLimits) { toast('Cette configuration sort des butées articulaires'); return; } app.moveJoints(s.q); };
      b.onmouseenter = () => app.previewGhost(s.q);
      b.onmouseleave = () => app.previewGhost(null);
      this.solBox.append(b);
    });
  }

  refresh() {
    const sim = this.app.sim;
    for (let i = 0; i < sim.n; i++) {
      const j = this.joints[i];
      if (!j) continue;
      j.val.textContent = `${(sim.q[i] * R2D).toFixed(2)}°`;
      if (this.dragging !== i) j.slider.value = (sim.des.q[i] * R2D).toFixed(1);
      const u = Math.min(1, sim.utilization[i] || 0);
      j.load.style.width = `${(u * 100).toFixed(0)}%`;
      j.load.dataset.level = u > 0.8 ? 'crit' : u > 0.6 ? 'warn' : 'ok';
      j.tgt.textContent = `consigne ${(sim.des.q[i] * R2D).toFixed(1)}°`;
      j.lost.textContent = sim.lostSteps[i] ? `⚠ ${sim.lostSteps[i]} pas perdus` : '';
    }
    this.powerBtn.setAttribute('aria-pressed', String(sim.enabled));
    const g = sim.gripper, gp = this.app.params.gripper;
    if (document.activeElement !== this.gripSlider) this.gripSlider.value = String(Math.round((g.target / gp.strokeMax) * 100));
    this.gripState.textContent = `${(g.pos * 1000).toFixed(1)} mm${sim.heldObject ? ' · objet saisi' : ''}`;
    for (let k = 0; k < 8; k++) {
      this.ioBtns.do[k].setAttribute('aria-pressed', String(!!sim.io.do[k]));
      this.ioBtns.di[k].setAttribute('aria-pressed', String(!!sim.io.di[k]));
    }
  }
}

// ====================================================================== Programme
const KW_RE = /^(\s*)(POINT|PT|VITESSE|SPEED|OVERRIDE|MOVEJ|DEPLJ|MJ|MOVEL|DEPLL|ML|MOVEC|DEPLC|MC|ATTENDRE_ENTREE|WAITDI|WAIT_DI|ATTENDRE|WAIT|TEMPO|PINCE|GRIP|GRIPPER|SORTIE|SETDO|DO|REPETER|RÉPÉTER|REPEAT|LOOP|FIN|END|MESSAGE|PRINT|MSG|PAUSE|HOME|MAISON|REPOS|REST)\b/i;

function esc(s) { return s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;'); }

function highlightLine(line) {
  const ci = line.search(/[#;]/);
  let code = ci >= 0 && !/"[^"]*[#;]/.test(line) ? line.slice(0, ci) : line;
  const cmt = code.length < line.length ? line.slice(code.length) : '';
  let html = '';
  const m = KW_RE.exec(code);
  if (m) { html += esc(m[1]) + `<span class="tok-kw">${esc(m[2])}</span>`; code = code.slice(m[0].length); }
  html += esc(code)
    .replace(/("[^"]*")/g, '<span class="tok-str">$1</span>')
    .replace(/\b(J|X|REL|OUTIL|TOOL)(?=\()/g, '<span class="tok-pt">$1</span>')
    .replace(/(^|[\s(,=])(-?\d+(?:\.\d+)?)/g, '$1<span class="tok-num">$2</span>');
  if (cmt) html += `<span class="tok-cmt">${esc(cmt)}</span>`;
  return html;
}

export class ProgramPanel {
  constructor(container, app) {
    this.app = app;
    this.c = container;
    this.curLine = 0;
    this.build();
  }

  build() {
    const app = this.app;
    this.c.replaceChildren();
    const ex = h('select.field.grow#prog-examples', { 'aria-label': 'Exemples de programmes' },
      h('option', { value: '' }, 'Exemples…'), ...Object.keys(DEMO_PROGRAMS).map((k) => h('option', { value: k }, k)));
    ex.addEventListener('change', () => { if (ex.value) { this.setText(DEMO_PROGRAMS[ex.value]); ex.value = ''; } });
    this.runBtn = btn('Exécuter', () => app.runProgram(this.ta.value), { cls: 'btn--sm btn--primary', icon: 'play' });
    this.pauseBtn = btn('', () => app.pauseProgram(), { cls: 'btn--sm btn--icon', icon: 'pause', title: 'Pause / reprendre' });
    this.stopBtn = btn('', () => app.stopProgram(), { cls: 'btn--sm btn--icon', icon: 'stop', title: 'Arrêter le programme' });
    this.c.append(h('div.row', ex,
      btn('', async () => { const f = await pickFile('.orl,.txt'); if (f) this.setText(f.text); }, { cls: 'btn--sm btn--icon', icon: 'upload', title: 'Ouvrir un programme (.orl)' }),
      btn('', () => saveFile('programme.orl', this.ta.value), { cls: 'btn--sm btn--icon', icon: 'download', title: 'Enregistrer le programme (.orl)' })));
    this.c.append(h('div.row', this.runBtn, this.pauseBtn, this.stopBtn, h('span.grow'),
      btn('Mémoriser le point', () => this.teach(), { cls: 'btn--sm', icon: 'target', title: 'Insère POINT Pn = J(...) avec la position actuelle, puis MOVEJ Pn' })));
    // Éditeur
    this.gutter = h('div.editor__gutter', { 'aria-hidden': 'true' });
    this.hl = h('pre.editor__hl', { 'aria-hidden': 'true' });
    this.ta = h('textarea.editor__ta#program-editor', { spellcheck: false, wrap: 'off', 'aria-label': 'Programme ORL', autocapitalize: 'off', autocomplete: 'off' });
    const wrap = h('div.editor__wrap', this.hl, this.ta);
    this.editor = h('div.editor', this.gutter, wrap);
    this.ta.addEventListener('input', () => { this.render(); store.set('program', this.ta.value); });
    this.ta.addEventListener('scroll', () => { this.hl.scrollTop = this.ta.scrollTop; this.hl.scrollLeft = this.ta.scrollLeft; this.gutter.scrollTop = this.ta.scrollTop; });
    this.ta.addEventListener('keydown', (e) => {
      if (e.key === 'Tab') { e.preventDefault(); document.execCommand?.('insertText', false, '  '); }
      if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { e.preventDefault(); app.runProgram(this.ta.value); }
    });
    this.status = h('div.sect__note', 'Prêt. Ctrl+Entrée pour exécuter.');
    this.errors = h('div.errors');
    this.c.append(this.editor, this.status, this.errors);
    const ref = h('details.pgroup', h('summary', 'Aide-mémoire du langage ORL'), h('div.pgroup__body', h('pre', { style: { margin: 0, font: '12px/1.5 var(--font-mono)', whiteSpace: 'pre-wrap' } },
      `POINT P1 = J(0, -20, 45, 0, 65, 0)     articulaire (°)
POINT P2 = X(250, 0, 150, 180, 0, 0)   cartésien (mm, °)
VITESSE 50                             vitesse globale %
MOVEJ P1 v=80 a=50 z=10                 v, a en % ; z = lissage mm
MOVEL P2 v=100                          v en mm/s
MOVEL REL(0, 0, -50)   MOVEL OUTIL(0, 0, 30)
MOVEC Pvia P2 v=50                      arc de cercle
ATTENDRE 0.5    PINCE OUVRIR|FERMER|50
SORTIE 1 ON     ATTENDRE_ENTREE 2 ON timeout=5
REPETER 3 … FIN     MESSAGE "texte"    PAUSE
HOME   REPOS`)));
    this.c.append(ref);
    this.setText(store.get('program', DEMO_PROGRAMS['Prise et dépose (cubes)']));
  }

  setText(t) { this.ta.value = t; this.render(); store.set('program', t); }

  render() {
    const lines = this.ta.value.split('\n');
    const parsed = parseProgram(this.ta.value);
    const errLines = new Set(parsed.errors.map((e) => e.line));
    this.hl.innerHTML = lines.map((l, i) => {
      const inner = highlightLine(l) || ' ';
      return i + 1 === this.curLine ? `<span class="line-cur">${inner}</span>` : inner;
    }).join('\n') + '\n';
    this.gutter.innerHTML = lines.map((_, i) => {
      const n = i + 1;
      const cls = errLines.has(n) ? 'err' : n === this.curLine ? 'cur' : '';
      return cls ? `<span class="${cls}">${n}</span>` : String(n);
    }).join('\n');
    this.errors.replaceChildren(...parsed.errors.slice(0, 5).map((e) => h('div', e.message)));
  }

  setCurrentLine(n) {
    if (n === this.curLine) return;
    this.curLine = n;
    this.render();
    // Défilement pour garder la ligne visible
    const lh = parseFloat(getComputedStyle(this.ta).lineHeight) || 20;
    const top = (n - 1) * lh;
    if (top < this.ta.scrollTop || top > this.ta.scrollTop + this.ta.clientHeight - lh * 2) this.ta.scrollTop = Math.max(0, top - lh * 3);
  }

  setStatus(text) { this.status.textContent = text; }

  teach() {
    const sim = this.app.sim;
    const text = this.ta.value;
    let k = 1;
    while (new RegExp(`\\bP${k}\\b`, 'i').test(text)) k++;
    const q = sim.q.map((v) => (v * R2D).toFixed(2)).join(', ');
    const ins = `POINT P${k} = J(${q})\nMOVEJ P${k}\n`;
    const pos = this.ta.selectionStart ?? text.length;
    const before = text.slice(0, pos), after = text.slice(pos);
    const sep = before.length && !before.endsWith('\n') ? '\n' : '';
    this.setText(before + sep + ins + after);
    toast(`Point P${k} mémorisé`);
  }
}

// ====================================================================== Assistant
export class AssistantPanel {
  constructor(container, app) {
    this.app = app;
    this.c = container;
    this.build();
  }

  build() {
    const app = this.app;
    this.c.replaceChildren();
    this.chat = h('div.chat', { role: 'log', 'aria-live': 'polite' });
    this.input = h('input.field.grow#assistant-input', { type: 'text', placeholder: 'Ex. « monte de 5 cm puis ouvre la pince »', autocomplete: 'off', 'aria-label': 'Commande en langage naturel' });
    this.input.addEventListener('keydown', (e) => { if (e.key === 'Enter') this.send(); });
    this.auto = h('input#assistant-auto', { type: 'checkbox', checked: store.get('assistantAuto', true) });
    this.auto.addEventListener('change', () => store.set('assistantAuto', this.auto.checked));
    this.tts = h('input#assistant-tts', { type: 'checkbox', checked: store.get('assistantTts', false) });
    this.tts.addEventListener('change', () => store.set('assistantTts', this.tts.checked));
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
    this.micBtn = btn('', () => this.toggleMic(), { cls: 'btn--icon mic', icon: 'mic', title: SR ? 'Dicter une commande (français)' : 'Dictée vocale indisponible dans ce navigateur', pressed: false });
    if (!SR) this.micBtn.disabled = true;
    this.c.append(
      h('div.sect',
        h('div.sect__head', h('h3.sect__title', 'Assistant de commande')),
        h('p.sect__note', 'Décrivez ce que le robot doit faire, en français ou en anglais. L’assistant traduit la phrase en programme ORL, que vous pouvez relire avant exécution. Tout fonctionne localement, sans connexion.')),
      this.chat,
      h('div.row.row--nowrap', this.input, this.micBtn, btn('', () => this.send(), { cls: 'btn--icon btn--primary', icon: 'send', title: 'Envoyer' })),
      h('div.row',
        h('label.row', { for: 'assistant-auto' }, this.auto, 'Exécuter automatiquement'),
        h('label.row', { for: 'assistant-tts' }, this.tts, 'Réponse vocale')),
      h('div.suggest', ...NLP_EXAMPLES.map((ex) => h('button', { type: 'button', onclick: () => { this.input.value = ex; this.send(); } }, ex))));
    this.say('bot', 'Bonjour ! Je pilote ORION-6. Essayez « prends le cube en 260 -120 », « dessine un cercle de 4 cm » ou « tourne la base de 30° à gauche ».');
    void app;
  }

  say(who, text, lines = null) {
    const m = h(`div.msg.msg--${who}`, text);
    if (lines && lines.length) {
      m.append(h('pre', lines.join('\n')));
      m.append(h('div.row',
        btn('Exécuter', () => this.app.executeLines(lines), { cls: 'btn--sm btn--primary', icon: 'play' }),
        btn('Ajouter au programme', () => this.app.appendToProgram(lines), { cls: 'btn--sm' })));
    }
    this.chat.append(m);
    this.chat.scrollTop = this.chat.scrollHeight;
  }

  send(textArg) {
    const text = (textArg ?? this.input.value).trim();
    if (!text) return;
    this.input.value = '';
    this.say('user', text);
    const app = this.app;
    const v = tcpVector(app.sim.tcp);
    const r = interpret(text, {
      q: app.sim.des.q.map((x) => x * R2D),
      tcp: { x: v[0] * 1000, y: v[1] * 1000, z: v[2] * 1000 },
      points: app.knownPoints(),
    });
    this.say('bot', r.reply || (r.understood ? 'OK.' : 'Je n’ai pas compris.'), r.lines);
    if (this.tts.checked && r.reply && window.speechSynthesis) {
      try { const u = new SpeechSynthesisUtterance(r.reply); u.lang = 'fr-FR'; window.speechSynthesis.speak(u); } catch { /* */ }
    }
    for (const a of r.actions) app.doAction(a);
    if (this.auto.checked && r.lines.length) app.executeLines(r.lines);
  }

  toggleMic() {
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!SR) return;
    if (this.rec) { this.rec.stop(); return; }
    const rec = new SR();
    rec.lang = 'fr-FR';
    rec.interimResults = true;
    rec.continuous = false;
    this.rec = rec;
    this.micBtn.setAttribute('aria-pressed', 'true');
    rec.onresult = (e) => {
      let finalT = '', interim = '';
      for (let i = e.resultIndex; i < e.results.length; i++) {
        if (e.results[i].isFinal) finalT += e.results[i][0].transcript; else interim += e.results[i][0].transcript;
      }
      this.input.value = finalT || interim;
      if (finalT) this.send(finalT);
    };
    rec.onerror = (e) => { toast(`Micro : ${e.error === 'not-allowed' ? 'accès refusé' : e.error}`); };
    rec.onend = () => { this.rec = null; this.micBtn.setAttribute('aria-pressed', 'false'); };
    try { rec.start(); } catch { this.rec = null; this.micBtn.setAttribute('aria-pressed', 'false'); }
  }
}

export { icon, mat4Pos };
