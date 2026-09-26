// ORION-6 — Unités : stockage interne en SI, affichage en unités « atelier ».
// factor : valeur_affichée = valeur_SI × factor

const R2D = 180 / Math.PI;

export const UNITS = {
  none: { si: '', display: '', factor: 1, decimals: 3 },
  count: { si: '', display: '', factor: 1, decimals: 0 },
  ratio: { si: '', display: ':1', factor: 1, decimals: 2 },
  length: { si: 'm', display: 'mm', factor: 1000, decimals: 2 },
  lengthM: { si: 'm', display: 'm', factor: 1, decimals: 3 },
  angle: { si: 'rad', display: '°', factor: R2D, decimals: 2 },
  angvel: { si: 'rad/s', display: '°/s', factor: R2D, decimals: 1 },
  angacc: { si: 'rad/s²', display: '°/s²', factor: R2D, decimals: 0 },
  angjerk: { si: 'rad/s³', display: '°/s³', factor: R2D, decimals: 0 },
  linvel: { si: 'm/s', display: 'mm/s', factor: 1000, decimals: 1 },
  linacc: { si: 'm/s²', display: 'mm/s²', factor: 1000, decimals: 0 },
  linjerk: { si: 'm/s³', display: 'mm/s³', factor: 1000, decimals: 0 },
  accel: { si: 'm/s²', display: 'm/s²', factor: 1, decimals: 3 },
  mass: { si: 'kg', display: 'kg', factor: 1, decimals: 3 },
  inertia: { si: 'kg·m²', display: 'kg·cm²', factor: 1e4, decimals: 3 },
  rotorInertia: { si: 'kg·m²', display: 'g·cm²', factor: 1e7, decimals: 1 },
  torque: { si: 'N·m', display: 'N·m', factor: 1, decimals: 3 },
  force: { si: 'N', display: 'N', factor: 1, decimals: 1 },
  current: { si: 'A', display: 'A', factor: 1, decimals: 2 },
  voltage: { si: 'V', display: 'V', factor: 1, decimals: 1 },
  resistance: { si: 'Ω', display: 'Ω', factor: 1, decimals: 2 },
  inductance: { si: 'H', display: 'mH', factor: 1e3, decimals: 2 },
  frequency: { si: 'Hz', display: 'Hz', factor: 1, decimals: 1 },
  time: { si: 's', display: 's', factor: 1, decimals: 3 },
  timeMs: { si: 's', display: 'ms', factor: 1e3, decimals: 2 },
  timeUs: { si: 's', display: 'µs', factor: 1e6, decimals: 1 },
  percent: { si: '', display: '%', factor: 100, decimals: 1 },
  stiffness: { si: 'N·m/rad', display: 'N·m/rad', factor: 1, decimals: 2 },
  damping: { si: 'N·m·s/rad', display: 'N·m·s/rad', factor: 1, decimals: 3 },
  integralGain: { si: 'N·m/(rad·s)', display: 'N·m/(rad·s)', factor: 1, decimals: 1 },
  linStiffness: { si: 'N/m', display: 'N/mm', factor: 1e-3, decimals: 3 },
  torqueConst: { si: 'N·m/A', display: 'N·m/A', factor: 1, decimals: 3 },
  speedRpm: { si: 'rad/s', display: 'tr/min', factor: 60 / (2 * Math.PI), decimals: 1 },
  bitrate: { si: 'bit/s', display: 'kbit/s', factor: 1e-3, decimals: 0 },
  pin: { si: '', display: '', factor: 1, decimals: 0 },
};

export function toDisplay(value, unit) {
  const u = UNITS[unit] || UNITS.none;
  return value * u.factor;
}

export function fromDisplay(value, unit) {
  const u = UNITS[unit] || UNITS.none;
  return value / u.factor;
}

/** Formatage court d'une valeur SI en unité d'affichage. */
export function fmt(value, unit, decimals) {
  const u = UNITS[unit] || UNITS.none;
  if (value === undefined || value === null || Number.isNaN(value)) return '—';
  const d = decimals ?? u.decimals;
  const v = value * u.factor;
  const s = Math.abs(v) >= 1e6 || (Math.abs(v) > 0 && Math.abs(v) < 10 ** -d) ? v.toExponential(2) : v.toFixed(d);
  return u.display ? `${s} ${u.display}` : s;
}
