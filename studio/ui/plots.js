// ORION Studio — courbes temps réel (canvas 2D).
// Mesure : trait plein 2 px ; consigne : trait fin pointillé de la même teinte.
// Légende = interrupteurs + valeurs courantes (sert de vue tabulaire). Réticule + infobulle au survol.

import { h, cssVar } from './dom.js';

const R2D = 180 / Math.PI;

export const PLOT_MODES = {
  pos: { label: 'Positions', unit: '°', meas: (i) => `q${i}`, des: (i) => `qd${i}`, k: R2D, joints: true },
  vel: { label: 'Vitesses', unit: '°/s', meas: (i) => `v${i}`, des: (i) => `vd${i}`, k: R2D, joints: true },
  err: { label: 'Erreur de suivi', unit: '°', meas: (i) => `err${i}`, k: R2D, joints: true, zero: true },
  tau: { label: 'Couples', unit: 'N·m', meas: (i) => `tau${i}`, des: (i) => `motor${i}`, desLabel: 'moteur', k: 1, joints: true, zero: true },
  use: { label: 'Charge moteur', unit: '%', meas: (i) => `use${i}`, k: 100, joints: true, fixed: [0, 100], band: 80 },
  tcp: { label: 'TCP (x, y, z)', unit: 'mm', series: [['tcpx', 'tcpxd', 'X'], ['tcpy', 'tcpyd', 'Y'], ['tcpz', 'tcpzd', 'Z']], k: 1000 },
  tcpv: { label: 'Vitesse TCP', unit: 'mm/s', series: [['tcpv', null, 'v TCP']], k: 1000 },
};

function niceStep(range, target = 5) {
  const raw = range / target;
  const p = 10 ** Math.floor(Math.log10(raw));
  const f = raw / p;
  return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p;
}

export class Plot {
  constructor(container, app) {
    this.app = app;
    this.mode = 'pos';
    this.visible = new Set([0, 1, 2, 3, 4, 5]);
    this.paused = false;
    this.canvas = h('canvas', { role: 'img', 'aria-label': 'Courbes de télémétrie' });
    this.tip = h('div.plot-tip', { hidden: true });
    this.wrap = h('div.plot', this.canvas, this.tip);
    this.legend = h('div.legend');
    container.append(this.wrap);
    this.hover = null;
    this.canvas.addEventListener('pointermove', (e) => {
      const r = this.canvas.getBoundingClientRect();
      this.hover = { x: e.clientX - r.left, y: e.clientY - r.top };
      this.draw(true);
    });
    this.canvas.addEventListener('pointerleave', () => { this.hover = null; this.tip.hidden = true; this.draw(true); });
  }

  setMode(m) {
    this.mode = m;
    this.buildLegend();
    this.draw(true);
  }

  seriesList() {
    const m = PLOT_MODES[this.mode];
    const n = this.app.sim.n;
    if (m.joints) {
      return Array.from({ length: n }, (_, i) => ({
        id: i, name: `J${i + 1}`, meas: m.meas(i), des: m.des ? m.des(i) : null, color: `--series-${(i % 6) + 1}`,
      }));
    }
    return m.series.map(([meas, des, name], i) => ({ id: i, name, meas, des, color: `--series-${i + 1}` }));
  }

  buildLegend() {
    this.legend.replaceChildren();
    this.legendVals = [];
    const m = PLOT_MODES[this.mode];
    for (const s of this.seriesList()) {
      const val = h('span.val', '—');
      const b = h('button', { type: 'button', 'aria-pressed': String(this.visible.has(s.id)), onclick: () => {
        if (this.visible.has(s.id)) this.visible.delete(s.id); else this.visible.add(s.id);
        b.setAttribute('aria-pressed', String(this.visible.has(s.id)));
        this.draw(true);
      } }, h('span.key', { style: { background: `var(${s.color})` } }), s.name, val);
      this.legend.append(b);
      this.legendVals.push(val);
    }
    if (m.des) this.legend.append(h('span.muted', `pointillés : ${m.desLabel || 'consigne'}`));
  }

  draw(force = false) {
    if (this.paused && !force) return;
    const tel = this.app.sim.telemetry;
    const cv = this.canvas;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const W = cv.clientWidth, H = cv.clientHeight;
    if (W < 20 || H < 20) return;
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const m = PLOT_MODES[this.mode];
    const t = tel.series('t');
    if (t.length < 2) return;
    const win = this.app.params.simulation.plotWindow || 10;
    const tEnd = t[t.length - 1], tStart = tEnd - win;
    let k0 = 0;
    while (k0 < t.length && t[k0] < tStart) k0++;
    const series = this.seriesList().filter((s) => this.visible.has(s.id));
    const data = series.map((s) => ({
      ...s,
      y: tel.series(s.meas).subarray(k0).map((v) => v * m.k),
      yd: s.des ? tel.series(s.des).subarray(k0).map((v) => v * m.k) : null,
    }));
    const tt = t.subarray(k0);
    // Échelle Y
    let lo = Infinity, hi = -Infinity;
    for (const d of data) {
      for (const v of d.y) { if (v < lo) lo = v; if (v > hi) hi = v; }
      if (d.yd) for (const v of d.yd) { if (Number.isFinite(v)) { if (v < lo) lo = v; if (v > hi) hi = v; } }
    }
    if (m.fixed) { lo = m.fixed[0]; hi = Math.max(m.fixed[1], hi); }
    if (!Number.isFinite(lo)) { lo = -1; hi = 1; }
    if (m.zero) { lo = Math.min(lo, 0); hi = Math.max(hi, 0); }
    if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
    const pad = (hi - lo) * 0.08;
    lo -= pad; hi += pad;
    const step = niceStep(hi - lo, Math.max(2, Math.round(H / 38)));
    lo = Math.floor(lo / step) * step; hi = Math.ceil(hi / step) * step;
    const L = 56, R = 10, T = 8, B = 20;
    const X = (tv) => L + ((tv - tStart) / win) * (W - L - R);
    const Y = (v) => T + (1 - (v - lo) / (hi - lo)) * (H - T - B);
    // Grille et axes (traits fins, pleins, discrets)
    const lineC = cssVar('--line'), muted = cssVar('--muted'), ink = cssVar('--ink-2'), surface = cssVar('--panel');
    ctx.font = '11px ' + cssVar('--font-mono');
    ctx.lineWidth = 1;
    ctx.strokeStyle = lineC;
    ctx.fillStyle = muted;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    const dec = step >= 1 ? 0 : Math.min(3, Math.ceil(-Math.log10(step)));
    for (let v = lo; v <= hi + step * 0.5; v += step) {
      const y = Math.round(Y(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(W - R, y); ctx.stroke();
      ctx.fillText(v.toFixed(dec), L - 6, y);
    }
    if (m.band !== undefined) {
      ctx.fillStyle = cssVar('--critical-soft');
      ctx.fillRect(L, T, W - L - R, Y(m.band) - T);
      ctx.fillStyle = muted;
    }
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const tStep = niceStep(win, Math.max(2, Math.round(W / 90)));
    for (let tv = Math.ceil(tStart / tStep) * tStep; tv <= tEnd; tv += tStep) {
      const x = Math.round(X(tv)) + 0.5;
      ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, H - B); ctx.stroke();
      ctx.fillText(`${tv.toFixed(tStep < 1 ? 1 : 0)} s`, x, H - B + 4);
    }
    ctx.save();
    ctx.translate(12, T + (H - T - B) / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillStyle = muted;
    ctx.textBaseline = 'middle';
    ctx.fillText(m.unit, 0, 0);
    ctx.restore();
    // Tracés
    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, W - L - R, H - T - B); ctx.clip();
    for (const d of data) {
      const col = cssVar(d.color);
      if (d.yd) {
        ctx.strokeStyle = col; ctx.globalAlpha = 0.7; ctx.lineWidth = 1.25; ctx.setLineDash([5, 4]);
        ctx.beginPath();
        for (let k = 0; k < tt.length; k++) { const x = X(tt[k]), y = Y(d.yd[k]); if (k) ctx.lineTo(x, y); else ctx.moveTo(x, y); }
        ctx.stroke();
        ctx.setLineDash([]); ctx.globalAlpha = 1;
      }
      ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.beginPath();
      for (let k = 0; k < tt.length; k++) { const x = X(tt[k]), y = Y(d.y[k]); if (k) ctx.lineTo(x, y); else ctx.moveTo(x, y); }
      ctx.stroke();
      // Point final (anneau couleur fond)
      const xe = X(tt[tt.length - 1]), ye = Y(d.y[d.y.length - 1]);
      ctx.beginPath(); ctx.arc(xe, ye, 4, 0, Math.PI * 2);
      ctx.fillStyle = col; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = surface; ctx.stroke();
    }
    ctx.restore();
    // Valeurs courantes dans la légende
    const all = this.seriesList();
    all.forEach((s, i) => {
      const arr = tel.series(s.meas);
      const v = arr[arr.length - 1] * m.k;
      if (this.legendVals[i]) this.legendVals[i].textContent = `${Number.isFinite(v) ? v.toFixed(m.unit === '%' ? 0 : 2) : '—'} ${m.unit}`;
    });
    // Réticule + infobulle
    if (this.hover && this.hover.x > L && this.hover.x < W - R) {
      const th = tStart + ((this.hover.x - L) / (W - L - R)) * win;
      let k = 0;
      while (k < tt.length - 1 && tt[k] < th) k++;
      const x = Math.round(X(tt[k])) + 0.5;
      ctx.strokeStyle = ink; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, H - B); ctx.stroke();
      this.tip.replaceChildren(h('span.k', 't'), h('span.v', `${tt[k].toFixed(3)} s`));
      for (const d of data) {
        const col = cssVar(d.color);
        ctx.beginPath(); ctx.arc(X(tt[k]), Y(d.y[k]), 4, 0, Math.PI * 2); ctx.fillStyle = col; ctx.fill();
        ctx.lineWidth = 2; ctx.strokeStyle = surface; ctx.stroke();
        this.tip.append(h('span.k', h('i', { style: { background: col } }), d.name), h('span.v', `${d.y[k].toFixed(2)}${d.yd ? ` / ${d.yd[k].toFixed(2)}` : ''} ${m.unit}`));
      }
      this.tip.hidden = false;
      const tw = this.tip.offsetWidth;
      this.tip.style.left = `${Math.min(W - tw - 4, this.hover.x + 12)}px`;
      this.tip.style.top = `${Math.max(4, Math.min(H - this.tip.offsetHeight - 4, this.hover.y - 10))}px`;
    }
  }
}
