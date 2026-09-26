// ORION-6 — Protocoles de communication.
//
// 1) Protocole série USB « ORION-ASCII » (Teensy 4.1) — une commande par ligne, fin '\n'.
//    Somme de contrôle optionnelle « *HH » (XOR des caractères avant '*', en hexadécimal).
//    Voir docs/11-firmware-protocole.md pour la liste complète.
//
// 2) Trames CAN « mode MIT » (contrôleur de Ben Katz — MIT Mini Cheetah, repris par
//    Damiao, CubeMars, etc.) : consigne 8 octets p/v/Kp/Kd/τff, retour position/vitesse/couple.

export const PROTOCOL_VERSION = 1;

export function checksum(s) {
  let c = 0;
  for (let i = 0; i < s.length; i++) c ^= s.charCodeAt(i);
  return c.toString(16).toUpperCase().padStart(2, '0');
}

/** Construit une ligne de commande. `withChecksum` ajoute *HH. */
export function encodeCommand(cmd, args = [], withChecksum = false) {
  const body = [cmd, ...args.map((a) => (typeof a === 'number' ? fmtNum(a) : String(a)))].join(' ');
  return (withChecksum ? `${body}*${checksum(body)}` : body) + '\n';
}

function fmtNum(v) {
  if (Number.isInteger(v)) return String(v);
  return (Math.round(v * 10000) / 10000).toString();
}

/**
 * Consigne articulaire en flux continu : SP seq t_ms q1..qn [v1..vn]
 *   seq  : numéro de séquence (16 bits, reboucle)
 *   tMs  : horodatage de l’émetteur en ms (16 bits, reboucle) — le firmware interpole
 *          entre consignes selon ces instants, indépendamment de la gigue USB
 *   qDeg : positions (°), vDeg : vitesses (°/s, optionnelles → interpolation d’Hermite)
 */
export const streamSetpoint = (seq, tMs, qDeg, vDeg = null) =>
  encodeCommand('SP', [seq & 0xffff, Math.round(tMs) & 0xffff, ...qDeg, ...(vDeg ? vDeg : [])], true);

/** Consigne du pont CAN mode MIT : MC seq t_ms q1..qn [v1..vn [τ1..τn]] (°, °/s, N·m). */
export const mitSetpoint = (seq, tMs, qDeg, vDeg = null, tau = null) =>
  encodeCommand('MC', [seq & 0xffff, Math.round(tMs) & 0xffff, ...qDeg, ...(vDeg ? vDeg : []), ...(vDeg && tau ? tau : [])], true);

export const moveJointsCmd = (durationS, qDeg) => encodeCommand('MJ', [durationS, ...qDeg], true);

export const STATES = ['IDLE', 'READY', 'HOMING', 'RUN', 'HOLD', 'DAMP', 'FAULT', 'ESTOP'];
export const FLAG_BITS = {
  enabled: 0x01, homed: 0x02, moving: 0x04, fault: 0x08, estopInput: 0x10, watchdog: 0x20, limit: 0x40, underrun: 0x80, limited: 0x100,
};

/**
 * Analyse une ligne reçue.
 *   OK …            → { type: 'ok', args }
 *   ERR code msg    → { type: 'err', code, message }
 *   EVT nom …       → { type: 'evt', name, args }
 *   ST état t q1..qn flags di → { type: 'st', state, t, q[], flags{}, di }
 */
export function parseLine(line) {
  let s = line.trim();
  if (!s) return null;
  const star = s.lastIndexOf('*');
  if (star > 0 && /^[0-9A-F]{2}$/i.test(s.slice(star + 1))) {
    const body = s.slice(0, star);
    if (checksum(body) !== s.slice(star + 1).toUpperCase()) return { type: 'bad', raw: line };
    s = body;
  }
  const t = s.split(/\s+/);
  switch (t[0]) {
    case 'OK': return { type: 'ok', args: t.slice(1) };
    case 'ERR': return { type: 'err', code: t[1], message: t.slice(2).join(' ') };
    case 'EVT': return { type: 'evt', name: t[1], args: t.slice(2) };
    case 'ST': {
      const n = (t.length - 5);
      const q = t.slice(3, 3 + n).map(Number);
      const flagsN = parseInt(t[3 + n], 16);
      const flags = {};
      for (const [k, b] of Object.entries(FLAG_BITS)) flags[k] = (flagsN & b) !== 0;
      return { type: 'st', state: t[1], t: Number(t[2]), q, flags, di: parseInt(t[4 + n], 16) };
    }
    default: return { type: 'raw', raw: s };
  }
}

// ---------------------------------------------------------------- CAN mode MIT

export function floatToUint(x, xMin, xMax, bits) {
  const span = xMax - xMin;
  const v = Math.max(xMin, Math.min(xMax, x));
  return Math.round(((v - xMin) * ((1 << bits) - 1)) / span);
}

export function uintToFloat(u, xMin, xMax, bits) {
  return (u * (xMax - xMin)) / ((1 << bits) - 1) + xMin;
}

/**
 * Trame de consigne MIT (8 octets) :
 *   [p(16)] [v(12) | kp(12)] [kd(12) | τ(12)]
 * limites : { pMax, vMax, tMax, kpMax, kdMax } (plages symétriques ±, gains ≥ 0)
 */
export function packMitCommand({ p, v, kp, kd, t }, lim) {
  const pi = floatToUint(p, -lim.pMax, lim.pMax, 16);
  const vi = floatToUint(v, -lim.vMax, lim.vMax, 12);
  const kpi = floatToUint(kp, 0, lim.kpMax, 12);
  const kdi = floatToUint(kd, 0, lim.kdMax, 12);
  const ti = floatToUint(t, -lim.tMax, lim.tMax, 12);
  return Uint8Array.from([
    pi >> 8, pi & 0xff,
    vi >> 4, ((vi & 0xf) << 4) | (kpi >> 8),
    kpi & 0xff,
    kdi >> 4, ((kdi & 0xf) << 4) | (ti >> 8),
    ti & 0xff,
  ]);
}

/**
 * Trame de retour (format Damiao) :
 *   D0 = (ERR << 4) | (ID & 0x0F) ; D1-D2 = position (16 b) ; D3-D4h = vitesse (12 b) ;
 *   D4l-D5 = couple (12 b) ; D6 = T° MOSFET ; D7 = T° rotor.
 */
export function unpackMitFeedback(d, lim) {
  const p = (d[1] << 8) | d[2];
  const v = (d[3] << 4) | (d[4] >> 4);
  const t = ((d[4] & 0xf) << 8) | d[5];
  return {
    id: d[0] & 0x0f,
    err: d[0] >> 4,
    p: uintToFloat(p, -lim.pMax, lim.pMax, 16),
    v: uintToFloat(v, -lim.vMax, lim.vMax, 12),
    t: uintToFloat(t, -lim.tMax, lim.tMax, 12),
    tMos: d[6],
    tRotor: d[7],
  };
}

/** Trames spéciales du mode MIT (identiques chez la plupart des fabricants). */
export const MIT_FRAMES = {
  enable: Uint8Array.from([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc]),
  disable: Uint8Array.from([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd]),
  setZero: Uint8Array.from([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe]),
  clearError: Uint8Array.from([0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb]),
};

export const MIT_ERRORS = {
  0x0: 'désactivé', 0x1: 'activé', 0x8: 'surtension', 0x9: 'sous-tension', 0xa: 'surintensité',
  0xb: 'surchauffe MOSFET', 0xc: 'surchauffe bobinage', 0xd: 'perte de communication', 0xe: 'surcharge',
};

export const hex = (bytes) => Array.from(bytes, (b) => b.toString(16).toUpperCase().padStart(2, '0')).join(' ');
