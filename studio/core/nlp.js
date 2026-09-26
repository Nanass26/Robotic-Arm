// ORION-6 — Assistant de commande en langage naturel (français / anglais), 100 % local.
// Transforme une phrase en lignes de programme ORL (affichées à l’utilisateur avant exécution)
// ou en actions système (arrêt, activation, mode…). Fonctionne aussi avec la dictée vocale.
//
// Exemples : « monte de 5 cm puis ouvre la pince », « tourne la base de 30 degrés à gauche »,
// « mets l'axe 3 à -20° », « va à la position repos », « prends le cube en 260 -120 »,
// « dessine un cercle de 4 cm », « vitesse 30 % », « salue ».

const NUM_WORDS = {
  zero: 0, 'zéro': 0, un: 1, une: 1, one: 1, deux: 2, two: 2, trois: 3, three: 3, quatre: 4, four: 4,
  cinq: 5, five: 5, six: 6, sept: 7, seven: 7, huit: 8, eight: 8, neuf: 9, nine: 9, dix: 10, ten: 10,
  onze: 11, douze: 12, quinze: 15, vingt: 20, twenty: 20, trente: 30, thirty: 30, quarante: 40, forty: 40,
  cinquante: 50, fifty: 50, soixante: 60, cent: 100, hundred: 100, demi: 0.5, half: 0.5,
};

/** Normalisation : minuscules, apostrophes, nombres en lettres → chiffres, virgule décimale. */
export function normalize(text) {
  let s = ` ${text.toLowerCase()} `
    .replace(/[’`]/g, "'")
    .replace(/(\d),(\d)/g, '$1.$2')
    .replace(/(\d)\s*°/g, '$1 deg ')
    .replace(/([a-zé])-/g, '$1 ')
    .replace(/[!?.;:]+(\s|$)/g, ' , ');
  s = s.replace(/\b([a-zéèêàù]+)\b/g, (w) => (w in NUM_WORDS ? String(NUM_WORDS[w]) : w));
  return s.replace(/\s+/g, ' ');
}

const AXIS_WORDS = [
  { re: /\b(base|j1|axe 1|axis 1|taille|waist)\b/, j: 0 },
  { re: /\b(épaule|epaule|shoulder|j2|axe 2|axis 2)\b/, j: 1 },
  { re: /\b(coude|elbow|j3|axe 3|axis 3)\b/, j: 2 },
  { re: /\b(avant bras|forearm|j4|axe 4|axis 4)\b/, j: 3 },
  { re: /\b(poignet|wrist|j5|axe 5|axis 5)\b/, j: 4 },
  { re: /\b(bride|flange|j6|axe 6|axis 6|outil|tool)\b/, j: 5 },
];

const DIRS = [
  { re: /\b(monte|montes|lève|leve|lever|monter|haut|up|raise|lift)\b/, v: [0, 0, 1], fr: 'monte' },
  { re: /\b(descends|descend|descendre|baisse|baisser|bas|down|lower)\b/, v: [0, 0, -1], fr: 'descends' },
  { re: /\b(avance|avancer|devant|forward|ahead)\b/, v: [1, 0, 0], fr: 'avance' },
  { re: /\b(recule|reculer|arrière|arriere|backward|back)\b/, v: [-1, 0, 0], fr: 'recule' },
  { re: /\b(gauche|left)\b/, v: [0, 1, 0], fr: 'vers la gauche' },
  { re: /\b(droite|right)\b/, v: [0, -1, 0], fr: 'vers la droite' },
];

/** Extrait une longueur en mètres (défaut : cm si non précisé et < 50, sinon mm). */
function parseLength(s) {
  const m = /(-?\d+(?:\.\d+)?)\s*(mm|millim[eè]tres?|cm|centim[eè]tres?|m\b|m[eè]tres?|meters?|inch|pouces?)?/.exec(s);
  if (!m) return null;
  const v = parseFloat(m[1]);
  const u = (m[2] || '').trim();
  if (/^mm|millim/.test(u)) return v / 1000;
  if (/^cm|centim/.test(u)) return v / 100;
  if (/^(m|m[eè]tre|meter)/.test(u)) return v;
  if (/inch|pouce/.test(u)) return v * 0.0254;
  return v < 50 ? v / 100 : v / 1000; // sans unité : « 5 » = 5 cm, « 120 » = 120 mm
}

function parseAngle(s) {
  const m = /(-?\d+(?:\.\d+)?)\s*(deg|degr[eé]s?|degrees?|°|rad|radians?)?/.exec(s);
  if (!m) return null;
  const v = parseFloat(m[1]);
  return /rad/.test(m[2] || '') ? (v * 180) / Math.PI : v;
}

const fmt = (x) => (Math.round(x * 10) / 10).toString();

/**
 * Interprète une phrase.
 * ctx : { q (deg), tcp: {x,y,z (mm)}, points: [noms] }
 * Retourne { lines: [ORL], actions: [{type,…}], reply, understood }
 */
export function interpret(text, ctx = {}) {
  const parts = normalize(text).split(/\s*(?:,|\bpuis\b|\bensuite\b|\bet ensuite\b|\bthen\b|\bafter that\b|\bet\b(?=\s+(?:ouvre|ferme|monte|descends|tourne|va|reviens|avance|recule|pose|prends|attends|dessine))|\band\b(?=\s+(?:open|close|go|move|turn|raise|lower)))\s*/).filter((p) => p.trim().length > 1);
  const out = { lines: [], actions: [], replies: [], understood: true };
  const q = ctx.q ? ctx.q.slice() : [0, 0, 0, 0, 90, 0];
  for (const raw of parts) {
    const s = ` ${raw.trim()} `;
    const r = interpretOne(s, { ...ctx, q });
    if (!r) { out.understood = false; out.replies.push(`Je n’ai pas compris « ${raw.trim()} ».`); continue; }
    out.lines.push(...(r.lines || []));
    out.actions.push(...(r.actions || []));
    if (r.reply) out.replies.push(r.reply);
    if (r.q) r.q.forEach((v, i) => { if (v !== null) q[i] = v; });
  }
  out.reply = out.replies.join(' ');
  return out;
}

function interpretOne(s, ctx) {
  // ---- actions système
  if (/\b(stop|arr[eê]te|arr[eê]t|halte|freeze|immobile)\b/.test(s) && !/moteur|motor/.test(s)) {
    return { actions: [{ type: 'stop' }], reply: 'Arrêt du mouvement.' };
  }
  if (/\b(urgence|emergency)\b/.test(s)) return { actions: [{ type: 'estop' }], reply: 'Arrêt d’urgence !' };
  if (/(d[ée]sactive|coupe|[ée]teins|disable|turn off).*(moteur|puissance|motor|power)/.test(s)) return { actions: [{ type: 'enable', value: false }], reply: 'Moteurs désactivés.' };
  if (/(active|allume|enable|turn on|mets sous tension).*(moteur|puissance|motor|power)/.test(s)) return { actions: [{ type: 'enable', value: true }], reply: 'Moteurs activés.' };
  if (/(prise d.origine|homing|r[ée]f[ée]rence les axes|calibr)/.test(s)) return { actions: [{ type: 'homing' }], reply: 'Prise d’origine lancée.' };
  if (/(main libre|compensation de gravit|gravity comp|teach mode|apprentissage)/.test(s)) return { actions: [{ type: 'mode', value: 'gravity_comp' }], reply: 'Mode main libre : le bras compense son poids, déplacez-le à la souris (Maj + glisser).' };
  if (/(mode (mit|pid|pas|stepper|imp[ée]dance|couple))/.test(s)) {
    const m = /mode (mit|pid|pas|stepper|imp[ée]dance|couple)/.exec(s)[1];
    const value = { mit: 'mit', pid: 'pid', pas: 'stepper', stepper: 'stepper', impédance: 'impedance', impedance: 'impedance', couple: 'computed_torque' }[m];
    return { actions: [{ type: 'mode', value }], reply: `Mode de commande : ${value}.` };
  }
  if (/(o[uù] es.tu|position actuelle|where are you|current position|ta position)/.test(s)) {
    const t = ctx.tcp || { x: 0, y: 0, z: 0 };
    return { reply: `Le TCP est en X ${fmt(t.x)} mm, Y ${fmt(t.y)} mm, Z ${fmt(t.z)} mm.` };
  }
  // ---- vitesse
  let m = /(?:vitesse|speed)\s*(?:à|a|de|to|at)?\s*(\d+)/.exec(s);
  if (m) return { lines: [`VITESSE ${Math.min(100, Math.max(1, parseInt(m[1], 10)))}`], reply: `Vitesse réglée à ${m[1]} %.` };
  if (/(plus vite|acc[ée]l[èe]re|faster|speed up)/.test(s)) return { lines: ['VITESSE 100'], reply: 'Je vais plus vite.' };
  if (/(plus lent|doucement|ralentis|slower|slow down|gently)/.test(s)) return { lines: ['VITESSE 20'], reply: 'Je ralentis.' };
  // ---- pince
  if (/(pince|gripper|griffe|main|hand|serre|l[aâ]che|grip)/.test(s) || /\b(ouvre|ferme|open|close)\b/.test(s)) {
    m = /(\d+)\s*(%|pour ?cent|percent)/.exec(s);
    if (m) return { lines: [`PINCE ${m[1]}`], reply: `Pince à ${m[1]} %.` };
    if (/(ouvr|open|l[aâ]che|release|rel[aâ]che)/.test(s)) return { lines: ['PINCE OUVRIR'], reply: 'J’ouvre la pince.' };
    if (/(ferm|close|serre|attrape|grab|squeeze)/.test(s)) return { lines: ['PINCE FERMER'], reply: 'Je ferme la pince.' };
  }
  // ---- attente
  m = /(?:attends|attendre|patiente|wait|pause de)\s*(\d+(?:\.\d+)?)\s*(ms|millisecondes?|s|sec|secondes?|seconds?)?/.exec(s);
  if (m) {
    const v = parseFloat(m[1]) / (/^ms|milli/.test(m[2] || '') ? 1000 : 1);
    return { lines: [`ATTENDRE ${v}`], reply: `J’attends ${v} s.` };
  }
  // ---- poses nommées
  if (/(maison|home|d[ée]part|initial)/.test(s)) return { lines: ['MOVEJ HOME'], reply: 'Retour à la position Home.' };
  if (/(repos|rest|range.toi|rentre|park|transport)/.test(s)) return { lines: ['MOVEJ REPOS'], reply: 'Position repos.' };
  if (/(z[ée]ro|droit comme|candle|tout droit)/.test(s) && !/(axe|axis|j\d)/.test(s)) return { lines: ['MOVEJ ZERO'], reply: 'Toutes les articulations à zéro.' };
  m = /(?:point|position|pt)\s+([a-z]\w*)/.exec(s);
  if (m && /(va|aller|rejoins|go|move)/.test(s) && (ctx.points || []).map((p) => p.toLowerCase()).includes(m[1])) {
    return { lines: [`MOVEJ ${m[1].toUpperCase()}`], reply: `Je vais au point ${m[1].toUpperCase()}.` };
  }
  // ---- gestes
  if (/(salue|bonjour|coucou|hello|wave|hi\b)/.test(s)) {
    const b = ctx.q;
    const J = (d1, d6) => `MOVEJ J(${fmt(d1)}, -10, -40, 0, 50, ${fmt(d6)}) v=90`;
    return { lines: ['VITESSE 80', J(b[0], 0), J(b[0] - 20, -40), J(b[0] + 20, 40), J(b[0] - 20, -40), J(b[0], 0), 'MOVEJ HOME'], reply: 'Bonjour ! 👋' };
  }
  if (/(danse|dance)/.test(s)) {
    return { lines: ['VITESSE 90', 'REPETER 2', 'MOVEJ J(-30, 10, -20, 60, 40, 90) z=20', 'MOVEJ J(30, -10, 10, -60, 70, -90) z=20', 'FIN', 'MOVEJ HOME'], reply: 'Petite danse !' };
  }
  // ---- dessins
  m = /(dessine|trace|fais|draw|make)\s+(?:un|une|a|1)?\s*(cercle|rond|circle|carr[ée]|square|triangle)/.exec(s);
  if (m) {
    const shape = m[2];
    const size = parseLength(s.slice(m.index + m[0].length)) ?? 0.04;
    const t = ctx.tcp || { x: 260, y: 0, z: 80 };
    const x = t.x, y = t.y, z = t.z, mm = size * 1000;
    if (/cercle|rond|circle/.test(shape)) {
      const r = mm;
      return {
        lines: [
          `MOVEL X(${fmt(x + r)}, ${fmt(y)}, ${fmt(z)}) v=100`,
          `MOVEC X(${fmt(x)}, ${fmt(y + r)}, ${fmt(z)}) X(${fmt(x - r)}, ${fmt(y)}, ${fmt(z)}) v=80`,
          `MOVEC X(${fmt(x)}, ${fmt(y - r)}, ${fmt(z)}) X(${fmt(x + r)}, ${fmt(y)}, ${fmt(z)}) v=80`,
          `MOVEL X(${fmt(x)}, ${fmt(y)}, ${fmt(z)}) v=100`,
        ],
        reply: `Je dessine un cercle de rayon ${fmt(r)} mm autour de la position actuelle.`,
      };
    }
    if (/triangle/.test(shape)) {
      const h = mm * Math.sqrt(3) / 2;
      return {
        lines: [
          `MOVEL X(${fmt(x + h * 2 / 3)}, ${fmt(y)}, ${fmt(z)}) v=100`,
          `MOVEL X(${fmt(x - h / 3)}, ${fmt(y + mm / 2)}, ${fmt(z)}) v=100 z=2`,
          `MOVEL X(${fmt(x - h / 3)}, ${fmt(y - mm / 2)}, ${fmt(z)}) v=100 z=2`,
          `MOVEL X(${fmt(x + h * 2 / 3)}, ${fmt(y)}, ${fmt(z)}) v=100`,
        ],
        reply: `Je dessine un triangle de ${fmt(mm)} mm de côté.`,
      };
    }
    const c = mm / 2;
    return {
      lines: [
        `MOVEL X(${fmt(x + c)}, ${fmt(y - c)}, ${fmt(z)}) v=100`,
        `MOVEL X(${fmt(x + c)}, ${fmt(y + c)}, ${fmt(z)}) v=100 z=2`,
        `MOVEL X(${fmt(x - c)}, ${fmt(y + c)}, ${fmt(z)}) v=100 z=2`,
        `MOVEL X(${fmt(x - c)}, ${fmt(y - c)}, ${fmt(z)}) v=100 z=2`,
        `MOVEL X(${fmt(x + c)}, ${fmt(y - c)}, ${fmt(z)}) v=100`,
      ],
      reply: `Je dessine un carré de ${fmt(mm)} mm.`,
    };
  }
  // ---- prise / dépose à des coordonnées
  m = /(prends|prendre|attrape|saisis|ramasse|pick|grab)\b.*?(-?\d+(?:\.\d+)?)[\s,;]+(-?\d+(?:\.\d+)?)(?:[\s,;]+(-?\d+(?:\.\d+)?))?/.exec(s);
  if (m) {
    const x = parseFloat(m[2]), y = parseFloat(m[3]), z = m[4] !== undefined ? parseFloat(m[4]) : 15;
    return {
      lines: ['PINCE OUVRIR', `MOVEJ X(${x}, ${y}, ${z + 90}, 180, 0, 0) z=15`, `MOVEL X(${x}, ${y}, ${z}, 180, 0, 0) v=80`, 'PINCE FERMER', `MOVEL X(${x}, ${y}, ${z + 90}, 180, 0, 0) v=150`],
      reply: `Je prends l’objet en (${x}, ${y}, ${z}) mm.`,
    };
  }
  m = /(pose|poser|d[ée]pose|place|put|drop)\b.*?(-?\d+(?:\.\d+)?)[\s,;]+(-?\d+(?:\.\d+)?)(?:[\s,;]+(-?\d+(?:\.\d+)?))?/.exec(s);
  if (m) {
    const x = parseFloat(m[2]), y = parseFloat(m[3]), z = m[4] !== undefined ? parseFloat(m[4]) : 15;
    return {
      lines: [`MOVEJ X(${x}, ${y}, ${z + 90}, 180, 0, 0) z=15`, `MOVEL X(${x}, ${y}, ${z}, 180, 0, 0) v=80`, 'PINCE OUVRIR', `MOVEL X(${x}, ${y}, ${z + 90}, 180, 0, 0) v=150`],
      reply: `Je dépose l’objet en (${x}, ${y}, ${z}) mm.`,
    };
  }
  // ---- aller à des coordonnées
  m = /(va|aller|rejoins|d[ée]place.toi|go|move)\b.*?(?:x\s*=?\s*)?(-?\d+(?:\.\d+)?)[\s,;]+(?:y\s*=?\s*)?(-?\d+(?:\.\d+)?)[\s,;]+(?:z\s*=?\s*)?(-?\d+(?:\.\d+)?)/.exec(s);
  if (m) {
    const lin = /(ligne droite|lin[ée]aire|straight|linear)/.test(s);
    return { lines: [`${lin ? 'MOVEL' : 'MOVEJ'} X(${m[2]}, ${m[3]}, ${m[4]})`], reply: `Je vais en X ${m[2]}, Y ${m[3]}, Z ${m[4]} mm.` };
  }
  // ---- articulation : absolue (« mets l'axe 3 à -20 ») ou relative (« tourne la base de 30° à gauche »)
  const ax = AXIS_WORDS.find((a) => a.re.test(s));
  if (ax && /(tourne|pivote|rotate|turn|bouge|move|mets|met|place|set|règle|regle|positionne)/.test(s)) {
    const j = ax.j;
    const mm = ax.re.exec(s);
    const after = s.slice(mm.index + mm[0].length);
    const absolute = /\b(à|a|to|sur|at|=)\s*-?\d/.test(after) && !/\bde\s+-?\d/.test(after);
    let ang = parseAngle(after);
    if (ang === null) ang = 10;
    if (/(droite|right|horaire|clockwise|n[ée]gati)/.test(s) && !/anti/.test(s)) ang = -Math.abs(ang);
    if (/(gauche|left|anti.?horaire|counter)/.test(s)) ang = Math.abs(ang);
    if (/(baisse|descend|down|lower)/.test(s) && (j === 1 || j === 2 || j === 4)) ang = Math.abs(ang);
    if (/(l[eè]ve|monte|up|raise)/.test(s) && (j === 1 || j === 2 || j === 4)) ang = -Math.abs(ang);
    const qn = ctx.q.slice();
    qn[j] = absolute ? ang : qn[j] + ang;
    const res = new Array(6).fill(null); res[j] = qn[j];
    return {
      lines: [`MOVEJ J(${qn.map(fmt).join(', ')})`],
      q: res,
      reply: absolute ? `Axe ${j + 1} à ${fmt(qn[j])}°.` : `Axe ${j + 1} : ${ang > 0 ? '+' : ''}${fmt(ang)}°.`,
    };
  }
  // ---- déplacement cartésien relatif
  const dir = DIRS.find((d) => d.re.test(s));
  if (dir) {
    let L = parseLength(s.slice(s.search(dir.re)));
    if (L === null) L = 0.02;
    const d = dir.v.map((v) => v * L * 1000);
    const toolFrame = /(rep[eè]re outil|tool frame|dans l.axe de l.outil)/.test(s);
    return {
      lines: [`MOVEL ${toolFrame ? 'OUTIL' : 'REL'}(${fmt(d[0])}, ${fmt(d[1])}, ${fmt(d[2])}) v=80`],
      reply: `Je ${dir.fr === 'vers la gauche' || dir.fr === 'vers la droite' ? 'me décale ' + dir.fr : dir.fr} de ${fmt(L * 1000)} mm.`,
    };
  }
  return null;
}

export const NLP_EXAMPLES = [
  'monte de 5 cm puis ouvre la pince',
  'tourne la base de 30 degrés à gauche',
  "mets l'axe 3 à -20°",
  'va à la position repos',
  'prends le cube en 260 -120',
  'pose-le en 260 120',
  'dessine un cercle de 4 cm',
  'vitesse 30 %',
  'salue',
  'où es-tu ?',
];
