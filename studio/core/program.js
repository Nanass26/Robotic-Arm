// ORION-6 — Langage de programmation du robot (ORL) : analyseur + interpréteur.
//
// Syntaxe (mots-clés français OU anglais, insensibles à la casse) :
//   # commentaire                        ; commentaire en fin de ligne
//   POINT P1 = J(0, -20, 45, 0, 65, 0)   définition articulaire (degrés)
//   POINT P2 = X(250, 0, 150, 180, 0, 0) pose cartésienne (mm, degrés roulis/tangage/lacet)
//   VITESSE 50            | SPEED 50      vitesse globale (%)
//   MOVEJ P1 v=80 a=50 z=10               articulaire : v, a en % ; z = zone de lissage (mm)
//   MOVEL P2 v=100 z=5                    linéaire : v en mm/s
//   MOVEL REL(0, 0, -50)                  déplacement relatif (repère base, mm [, degrés])
//   MOVEL OUTIL(0, 0, 30) | TOOL(...)     déplacement relatif dans le repère outil
//   MOVEC Pvia P2 v=50                    arc de cercle passant par Pvia
//   ATTENDRE 0.5          | WAIT 0.5      temporisation (s)
//   PINCE OUVRIR|FERMER|50 | GRIP OPEN|CLOSE|50
//   SORTIE 1 ON           | SETDO 1 1
//   ATTENDRE_ENTREE 2 ON timeout=5 | WAITDI 2 1 timeout=5
//   REPETER 3 ... FIN     | REPEAT 3 ... END
//   MESSAGE "texte"       | PRINT "texte"
//   PAUSE                 attend l’appui sur « Reprendre »
//   HOME / REPOS          raccourcis pour MOVEJ HOME / MOVEJ REPOS

import { DEG, poseToMat4, mat4Mul, mat4Transl, rpyToMat3, mat4FromRotPos, mat4Rot, mat4Pos } from './math3d.js';

const KW = {
  point: ['POINT', 'PT'],
  speed: ['VITESSE', 'SPEED', 'OVERRIDE'],
  movej: ['MOVEJ', 'DEPLJ', 'MJ'],
  movel: ['MOVEL', 'DEPLL', 'ML'],
  movec: ['MOVEC', 'DEPLC', 'MC'],
  wait: ['ATTENDRE', 'WAIT', 'TEMPO'],
  grip: ['PINCE', 'GRIP', 'GRIPPER'],
  setdo: ['SORTIE', 'SETDO', 'DO'],
  waitdi: ['ATTENDRE_ENTREE', 'WAITDI', 'WAIT_DI'],
  repeat: ['REPETER', 'RÉPÉTER', 'REPEAT', 'LOOP'],
  end: ['FIN', 'END', 'ENDREPEAT'],
  message: ['MESSAGE', 'PRINT', 'MSG'],
  pause: ['PAUSE'],
  home: ['HOME', 'MAISON'],
  rest: ['REPOS', 'REST'],
};
const kwOf = (w) => {
  const u = w.toUpperCase();
  for (const [k, list] of Object.entries(KW)) if (list.includes(u)) return k;
  return null;
};

export class ProgramError extends Error {
  constructor(line, msg) { super(`Ligne ${line} : ${msg}`); this.line = line; }
}

/** Analyse une cible : nom de point, J(...), X(...), REL(...), OUTIL(...). */
function parseTarget(tok, line) {
  const m = /^(J|X|REL|OUTIL|TOOL)\s*\(([^)]*)\)$/i.exec(tok);
  if (m) {
    const vals = m[2].split(',').map((s) => parseFloat(s.trim()));
    if (vals.some((v) => Number.isNaN(v))) throw new ProgramError(line, `valeurs invalides dans ${tok}`);
    const kind = m[1].toUpperCase();
    if (kind === 'J') {
      if (vals.length !== 6) throw new ProgramError(line, 'J(...) attend 6 angles (degrés)');
      return { type: 'joint', q: vals.map((v) => v * DEG) };
    }
    const v6 = [...vals, 0, 0, 0, 0, 0, 0].slice(0, 6);
    const pose = { x: v6[0] / 1000, y: v6[1] / 1000, z: v6[2] / 1000, roll: v6[3] * DEG, pitch: v6[4] * DEG, yaw: v6[5] * DEG };
    if (kind === 'X') {
      if (vals.length < 3) throw new ProgramError(line, 'X(...) attend au moins x, y, z (mm)');
      if (vals.length < 6) pose.keepOrientation = true;
      return { type: 'pose', pose };
    }
    return { type: kind === 'REL' ? 'rel' : 'tool', pose };
  }
  if (/^[A-Za-z_][\w]*$/.test(tok)) return { type: 'name', name: tok.toUpperCase() };
  throw new ProgramError(line, `cible inconnue « ${tok} »`);
}

/** Découpe une ligne en jetons en respectant parenthèses et guillemets. */
function tokenize(s) {
  const out = [];
  let cur = '', depth = 0, quote = false;
  for (const ch of s) {
    if (ch === '"') { quote = !quote; cur += ch; continue; }
    if (!quote) {
      if (ch === '(') depth++;
      if (ch === ')') depth--;
      if ((ch === ' ' || ch === '\t') && depth === 0) { if (cur) out.push(cur); cur = ''; continue; }
    }
    cur += ch;
  }
  if (cur) out.push(cur);
  return out;
}

function parseOptions(tokens) {
  const opts = {};
  for (const t of tokens) {
    const m = /^(v|a|z|timeout|vitesse|acc|zone)=(-?[\d.]+)$/i.exec(t);
    if (m) {
      const k = { vitesse: 'v', acc: 'a', zone: 'z' }[m[1].toLowerCase()] || m[1].toLowerCase();
      opts[k] = parseFloat(m[2]);
    }
  }
  return opts;
}

/** Analyse le texte d’un programme → { instructions (arbre), points, errors }. */
export function parseProgram(text) {
  const lines = text.split(/\r?\n/);
  const root = [];
  const stack = [{ body: root }];
  const points = {};
  const errors = [];
  lines.forEach((raw, idx) => {
    const line = idx + 1;
    const s = raw.replace(/[#;].*$/, (m) => (m.includes('"') ? m : '')).trim();
    if (!s) return;
    try {
      const tokens = tokenize(s);
      const op = kwOf(tokens[0]);
      const body = stack[stack.length - 1].body;
      switch (op) {
        case 'point': {
          const m = /^\S+\s+([A-Za-z_]\w*)\s*=\s*(.+)$/.exec(s);
          if (!m) throw new ProgramError(line, 'syntaxe : POINT NOM = J(...) ou X(...)');
          const tgt = parseTarget(m[2].replace(/\s+/g, ''), line);
          if (tgt.type !== 'joint' && tgt.type !== 'pose') throw new ProgramError(line, 'un point doit être J(...) ou X(...)');
          points[m[1].toUpperCase()] = tgt;
          break;
        }
        case 'speed': {
          const v = parseFloat(tokens[1]);
          if (!(v > 0 && v <= 100)) throw new ProgramError(line, 'vitesse entre 1 et 100 %');
          body.push({ op: 'speed', value: v / 100, line });
          break;
        }
        case 'movej': case 'movel': {
          if (!tokens[1]) throw new ProgramError(line, 'cible manquante');
          body.push({ op, target: parseTarget(tokens[1], line), opts: parseOptions(tokens.slice(2)), line });
          break;
        }
        case 'movec': {
          if (!tokens[2]) throw new ProgramError(line, 'MOVEC attend un point de passage et une cible');
          body.push({ op, via: parseTarget(tokens[1], line), target: parseTarget(tokens[2], line), opts: parseOptions(tokens.slice(3)), line });
          break;
        }
        case 'home': case 'rest':
          body.push({ op: 'movej', target: { type: 'name', name: op === 'home' ? 'HOME' : 'REPOS' }, opts: parseOptions(tokens.slice(1)), line });
          break;
        case 'wait': {
          const t = parseFloat(tokens[1]);
          if (!(t >= 0)) throw new ProgramError(line, 'durée invalide');
          body.push({ op: 'wait', time: t, line });
          break;
        }
        case 'grip': {
          const a = (tokens[1] || '').toUpperCase();
          let value;
          if (['OUVRIR', 'OPEN', 'OUVERTE'].includes(a)) value = 1;
          else if (['FERMER', 'CLOSE', 'FERMEE', 'FERMÉE', 'SERRER'].includes(a)) value = 0;
          else { value = parseFloat(a) / 100; if (Number.isNaN(value)) throw new ProgramError(line, 'PINCE OUVRIR | FERMER | 0..100'); }
          body.push({ op: 'grip', value: Math.max(0, Math.min(1, value)), line });
          break;
        }
        case 'setdo': {
          const pin = parseInt(tokens[1], 10);
          const v = /^(1|ON|VRAI|TRUE|HAUT)$/i.test(tokens[2] || '');
          if (!(pin >= 0 && pin < 8)) throw new ProgramError(line, 'sortie 0..7');
          body.push({ op: 'setdo', pin, value: v, line });
          break;
        }
        case 'waitdi': {
          const pin = parseInt(tokens[1], 10);
          const v = /^(1|ON|VRAI|TRUE|HAUT)$/i.test(tokens[2] || '1');
          body.push({ op: 'waitdi', pin, value: v, timeout: parseOptions(tokens.slice(3)).timeout ?? 0, line });
          break;
        }
        case 'repeat': {
          const n = parseInt(tokens[1], 10);
          if (!(n >= 1)) throw new ProgramError(line, 'REPETER n (n ≥ 1)');
          const blk = { op: 'repeat', count: n, body: [], line };
          body.push(blk);
          stack.push(blk);
          break;
        }
        case 'end':
          if (stack.length === 1) throw new ProgramError(line, 'FIN sans REPETER');
          stack.pop();
          break;
        case 'message': {
          const m = /"([^"]*)"/.exec(s);
          body.push({ op: 'message', text: m ? m[1] : tokens.slice(1).join(' '), line });
          break;
        }
        case 'pause':
          body.push({ op: 'pause', line });
          break;
        default:
          throw new ProgramError(line, `instruction inconnue « ${tokens[0]} »`);
      }
    } catch (e) {
      errors.push(e instanceof ProgramError ? e : new ProgramError(line, e.message));
    }
  });
  if (stack.length > 1) errors.push(new ProgramError(lines.length, 'REPETER sans FIN'));
  return { instructions: root, points, errors };
}

/**
 * Interpréteur pas-à-pas piloté par `tick()` (appelé à chaque image).
 * S’appuie sur le simulateur (ou le robot réel via la même interface).
 */
export class ProgramRunner {
  constructor(sim, text, extraPoints = {}, hooks = {}) {
    this.sim = sim;
    this.hooks = hooks; // { log(msg), onLine(line), onDone(), onError(err) }
    const parsed = parseProgram(text);
    this.errors = parsed.errors;
    this.points = { ...extraPoints, ...parsed.points };
    this.program = parsed.instructions;
    this.stack = [{ list: this.program, index: 0, remaining: 1 }];
    this.state = parsed.errors.length ? 'error' : 'ready';
    this.waitUntil = null;
    this.speedOverride = 1;
    this.currentLine = 0;
  }

  log(m) { this.hooks.log?.(m); }

  /** Transformation cible à partir d’une référence (fin de la file de mouvements). */
  resolvePose(tgt, qRef) {
    const kin = this.sim.kin;
    const Tref = kin.tcp(qRef);
    if (tgt.type === 'joint') return kin.tcp(tgt.q);
    if (tgt.type === 'pose') {
      if (tgt.pose.keepOrientation) return mat4FromRotPos(mat4Rot(Tref), [tgt.pose.x, tgt.pose.y, tgt.pose.z]);
      return poseToMat4(tgt.pose);
    }
    if (tgt.type === 'rel') {
      const d = tgt.pose;
      const R = mat4Rot(Tref), p = mat4Pos(Tref);
      const Rd = rpyToMat3(d.roll, d.pitch, d.yaw);
      // Rotation relative exprimée dans le repère base, autour du TCP
      const Rn = [0, 0, 0, 0, 0, 0, 0, 0, 0];
      for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) Rn[r * 3 + c] = Rd[r * 3] * R[c] + Rd[r * 3 + 1] * R[3 + c] + Rd[r * 3 + 2] * R[6 + c];
      return mat4FromRotPos(Rn, [p[0] + d.x, p[1] + d.y, p[2] + d.z]);
    }
    if (tgt.type === 'tool') return mat4Mul(Tref, mat4Mul(mat4Transl(tgt.pose.x, tgt.pose.y, tgt.pose.z), poseToMat4({ roll: tgt.pose.roll, pitch: tgt.pose.pitch, yaw: tgt.pose.yaw })));
    if (tgt.type === 'name') {
      const p = this.lookup(tgt.name);
      return this.resolvePose(p, qRef);
    }
    throw new Error('cible inconnue');
  }

  lookup(name) {
    const poses = this.sim.params.poses || {};
    if (name === 'HOME') return { type: 'joint', q: poses.home };
    if (name === 'REPOS' || name === 'REST') return { type: 'joint', q: poses.rest };
    if (name === 'ZERO') return { type: 'joint', q: poses.zero || new Array(this.sim.n).fill(0) };
    const p = this.points[name];
    if (!p) throw new Error(`point « ${name} » non défini`);
    return p;
  }

  /** Cible articulaire pour MoveJ. */
  resolveJoint(tgt, qRef) {
    if (tgt.type === 'joint') return tgt.q;
    if (tgt.type === 'name') {
      const p = this.lookup(tgt.name);
      if (p.type === 'joint') return p.q;
      return this.resolveJoint(p, qRef);
    }
    const T = this.resolvePose(tgt, qRef);
    const r = this.sim.kin.solveIK(T, { seed: qRef });
    if (!r.ok) throw new Error(`pose inatteignable (${r.message || 'IK'})`);
    return r.q;
  }

  start() {
    if (this.state === 'error') return false;
    this.state = 'running';
    return true;
  }
  pause() { if (this.state === 'running') this.state = 'paused'; }
  resume() { if (this.state === 'paused' || this.state === 'userpause') this.state = 'running'; }
  stop() { this.state = 'stopped'; this.sim.stopMotion(); }

  nextInstruction() {
    while (this.stack.length) {
      const top = this.stack[this.stack.length - 1];
      if (top.index < top.list.length) return top.list[top.index++];
      if (top.remaining > 1) { top.remaining--; top.index = 0; continue; }
      this.stack.pop();
    }
    return null;
  }

  queueLength() { return this.sim.executor.items.length; }

  /** Avance l’interpréteur ; retourne l’état. */
  tick() {
    if (this.state !== 'running') return this.state;
    const sim = this.sim;
    // Ligne courante = segment actif
    const act = sim.executor.last?.tag;
    if (sim.executor.busy && act && act !== this.currentLine) { this.currentLine = act; this.hooks.onLine?.(act); }
    for (let guard = 0; guard < 50; guard++) {
      if (this.waitUntil && !this.waitUntil()) return this.state;
      this.waitUntil = null;
      const ins = this.nextInstruction();
      if (!ins) {
        this.waitUntil = null;
        if (sim.executor.busy) { this.waitUntil = () => !sim.executor.busy; this.stack.push({ list: [], index: 0, remaining: 1 }); return this.state; }
        this.state = 'done';
        this.hooks.onDone?.();
        return this.state;
      }
      try {
        if (!this.exec(ins)) return this.state;
      } catch (e) {
        this.state = 'error';
        this.hooks.onError?.(new ProgramError(ins.line, e.message));
        sim.stopMotion();
        return this.state;
      }
    }
    return this.state;
  }

  /** Exécute une instruction ; retourne false si l’on doit rendre la main (attente). */
  exec(ins) {
    const sim = this.sim;
    const idle = () => !sim.executor.busy;
    const mark = () => { this.currentLine = ins.line; this.hooks.onLine?.(ins.line); };
    switch (ins.op) {
      case 'speed':
        this.speedOverride = ins.value;
        return true;
      case 'movej': case 'movel': case 'movec': {
        // Pas plus de 2 mouvements d’avance dans la file
        if (this.queueLength() >= 2) {
          this.stackPush(ins);
          this.waitUntil = () => this.queueLength() < 2;
          return false;
        }
        const qRef = sim.plannedEnd();
        const o = ins.opts || {};
        const zone = (o.z ?? 0) / 1000;
        if (ins.op === 'movej') {
          const q = this.resolveJoint(ins.target, qRef);
          sim.moveJ(q, { speed: ((o.v ?? 100) / 100) * this.speedOverride, accel: (o.a ?? 100) / 100, zone, tag: ins.line });
        } else if (ins.op === 'movel') {
          const T = this.resolvePose(ins.target, qRef);
          const opts = { accel: (o.a ?? 100) / 100, zone, tag: ins.line };
          if (o.v) opts.speedAbs = (o.v / 1000) * this.speedOverride * sim.params.trajectory.speedOverride; else opts.speed = this.speedOverride;
          sim.moveL(T, opts);
        } else {
          const Tv = this.resolvePose(ins.via, qRef);
          const T = this.resolvePose(ins.target, qRef);
          const opts = { accel: (o.a ?? 100) / 100, zone, tag: ins.line };
          if (o.v) opts.speedAbs = (o.v / 1000) * this.speedOverride * sim.params.trajectory.speedOverride; else opts.speed = this.speedOverride;
          sim.moveC(Tv, T, opts);
        }
        if (!sim.executor.last) mark();
        return true;
      }
      case 'wait': {
        if (!idle()) { this.stackPush(ins); this.waitUntil = idle; return false; }
        mark();
        const tEnd = sim.t + ins.time;
        this.waitUntil = () => sim.t >= tEnd;
        return false;
      }
      case 'grip': {
        if (!idle()) { this.stackPush(ins); this.waitUntil = idle; return false; }
        mark();
        sim.setGripper(ins.value);
        const t0 = sim.t;
        this.waitUntil = () => !sim.gripperBusy() || sim.t - t0 > 3;
        return false;
      }
      case 'setdo':
        if (!idle()) { this.stackPush(ins); this.waitUntil = idle; return false; }
        mark();
        sim.io.do[ins.pin] = ins.value;
        this.hooks.onIO?.(ins.pin, ins.value);
        return true;
      case 'waitdi': {
        if (!idle()) { this.stackPush(ins); this.waitUntil = idle; return false; }
        mark();
        const t0 = sim.t;
        this.waitUntil = () => {
          if (sim.io.di[ins.pin] === ins.value) return true;
          if (ins.timeout > 0 && sim.t - t0 > ins.timeout) throw new Error(`délai dépassé sur l’entrée ${ins.pin}`);
          return false;
        };
        return false;
      }
      case 'message':
        mark();
        this.log(ins.text);
        return true;
      case 'pause':
        if (!idle()) { this.stackPush(ins); this.waitUntil = idle; return false; }
        mark();
        this.state = 'userpause';
        this.hooks.onPause?.();
        return false;
      case 'repeat':
        this.stack.push({ list: ins.body, index: 0, remaining: ins.count });
        return true;
      default:
        return true;
    }
  }

  /** Remet l’instruction en tête pour la ré-exécuter après l’attente. */
  stackPush(ins) {
    this.stack.push({ list: [ins], index: 0, remaining: 1 });
  }
}

/** Programme de démonstration (prise-dépose + dessin). */
export const DEMO_PROGRAMS = {
  'Prise et dépose (cubes)': `# Démo : prise d'un cube et dépose sur la zone B
VITESSE 60
HOME
PINCE OUVRIR
POINT A_HAUT = X(260, -120, 110, 180, 0, 0)
POINT A_BAS  = X(260, -120, 22, 180, 0, 0)
POINT B_HAUT = X(260, 120, 110, 180, 0, 0)
POINT B_BAS  = X(260, 120, 22, 180, 0, 0)
MOVEJ A_HAUT z=20
MOVEL A_BAS v=80
PINCE FERMER
MOVEL A_HAUT v=150 z=15
MOVEJ B_HAUT z=20
MOVEL B_BAS v=80
PINCE OUVRIR
MOVEL B_HAUT v=150
HOME
MESSAGE "Cycle terminé"
`,
  'Dessin : carré + cercle': `# Le TCP dessine un carré de 100 mm puis un cercle
VITESSE 70
HOME
POINT C1 = X(230, -50, 60, 180, 0, 0)
POINT C2 = X(330, -50, 60, 180, 0, 0)
POINT C3 = X(330, 50, 60, 180, 0, 0)
POINT C4 = X(230, 50, 60, 180, 0, 0)
MOVEJ C1
MOVEL C2 v=120 z=3
MOVEL C3 v=120 z=3
MOVEL C4 v=120 z=3
MOVEL C1 v=120
POINT R1 = X(330, 0, 60, 180, 0, 0)
POINT R2 = X(230, 0, 60, 180, 0, 0)
MOVEL X(280, -50, 60, 180, 0, 0) v=120
MOVEC R1 X(280, 50, 60, 180, 0, 0) v=100
MOVEC R2 X(280, -50, 60, 180, 0, 0) v=100
HOME
`,
  'Test de répétabilité': `# Aller-retour entre deux points (mesure de dérive / pertes de pas)
VITESSE 100
POINT PA = J(-40, 20, 20, 30, 60, -45)
POINT PB = J(40, -10, 40, -30, 45, 45)
REPETER 5
  MOVEJ PA
  MOVEJ PB
FIN
HOME
`,
  'Palettisation 2×3': `# Pose 6 positions en grille (déplacements relatifs)
VITESSE 80
HOME
POINT ORIG = X(220, -80, 60, 180, 0, 0)
MOVEJ ORIG
REPETER 2
  MOVEL REL(0, 0, -30) v=100
  ATTENDRE 0.2
  MOVEL REL(0, 0, 30) v=150
  MOVEL REL(0, 80, 0) v=200 z=5
  MOVEL REL(0, 0, -30) v=100
  ATTENDRE 0.2
  MOVEL REL(0, 0, 30) v=150
  MOVEL REL(0, 80, 0) v=200 z=5
  MOVEL REL(0, 0, -30) v=100
  ATTENDRE 0.2
  MOVEL REL(0, 0, 30) v=150
  MOVEL REL(60, -160, 0) v=200
FIN
HOME
`,
};
