// ORION Studio — éditeur de paramètres généré depuis le schéma (core/schema.js).

import { SCHEMA, getPath, setPath, validateParams } from '../core/schema.js';
import { UNITS, toDisplay, fromDisplay } from '../core/units.js';
import { PRESETS } from '../core/defaults.js';
import { h, btn, helpDot, toast, pickFile, saveFile, attachHelp } from './dom.js';
import { toURDF, toMJCF, toFirmwareConfig, toFirmwareSetCommands, toJSON } from '../core/export.js';

const GROUP_FOR_TYPE = { stepper: ['stepper'], bldc_mit: ['servo', 'mit'], servo: ['servo'] };

export class ParamsPanel {
  constructor(container, app) {
    this.app = app;
    this.inputs = []; // { el, read(), path, desc }
    this.container = container;
    this.build();
  }

  build() {
    const app = this.app;
    const c = this.container;
    c.replaceChildren();
    // Préréglages et fichiers
    const presetSel = h('select.field.grow#preset-select', { 'aria-label': 'Préréglage' },
      h('option', { value: '' }, 'Charger un préréglage…'),
      ...PRESETS.map((p) => h('option', { value: p.id }, p.label)));
    presetSel.addEventListener('change', () => {
      const p = PRESETS.find((x) => x.id === presetSel.value);
      if (p) { app.loadParams(p.make(), `Préréglage chargé : ${p.label}`); }
      presetSel.value = '';
    });
    this.search = h('input.field.grow#param-search', { type: 'search', placeholder: 'Rechercher un paramètre (ex. Kp, jeu, courant)…', 'aria-label': 'Rechercher un paramètre' });
    this.search.addEventListener('input', () => this.filter(this.search.value));
    const exp = (name, fn, mime) => () => saveFile(name, fn(app.params), mime);
    const slugName = () => (app.params.meta.name || 'robot').toLowerCase().replace(/[^a-z0-9]+/g, '_');
    c.append(
      h('div.sect',
        h('div.row', presetSel),
        h('div.row',
          btn('Importer', () => this.importJSON(), { cls: 'btn--sm', icon: 'upload', title: 'Charger une configuration .json' }),
          btn('JSON', exp(`${'config'}.json`, toJSON, 'application/json'), { cls: 'btn--sm', icon: 'download', title: 'Exporter la configuration complète' }),
          btn('URDF', () => saveFile(`${slugName()}.urdf`, toURDF(app.params), 'application/xml'), { cls: 'btn--sm', icon: 'download', title: 'Description ROS / MoveIt' }),
          btn('MJCF', () => saveFile(`${slugName()}.xml`, toMJCF(app.params), 'application/xml'), { cls: 'btn--sm', icon: 'download', title: 'Modèle MuJoCo' }),
          btn('Firmware .h', () => saveFile('orion_config.h', toFirmwareConfig(app.params), 'text/x-c'), { cls: 'btn--sm', icon: 'download', title: 'En-tête de configuration du firmware Teensy' }),
          btn('SET', () => saveFile('orion_set_commands.txt', toFirmwareSetCommands(app.params).join('\n') + '\n'), { cls: 'btn--sm', icon: 'download', title: 'Commandes série pour régler le firmware sans recompiler' }),
          btn('', () => { if (this.confirmReset) { app.resetParams(); this.confirmReset = false; resetBtn.title = 'Réinitialiser'; } else { this.confirmReset = true; toast('Cliquez encore pour confirmer la réinitialisation'); setTimeout(() => { this.confirmReset = false; }, 3000); } }, { cls: 'btn--sm btn--icon', icon: 'reset', title: 'Réinitialiser tous les paramètres' })),
        h('div.row', this.search)),
    );
    const resetBtn = c.querySelector('.btn--icon');
    this.warnBox = h('div.warnlist');
    c.append(this.warnBox);
    this.groups = [];
    for (const section of SCHEMA) {
      const body = h('div.pgroup__body', h('p.pgroup__help', section.help || ''));
      let count = 0;
      for (const p of section.params) {
        body.append(this.renderParam(p, section));
        count += p.kind === 'table' ? p.columns.length : 1;
      }
      const det = h('details.pgroup', { dataset: { section: section.id } },
        h('summary', section.title, h('span.count', `${count}`)), body);
      if (['kinematics', 'control'].includes(section.id)) det.open = true;
      c.append(det);
      this.groups.push(det);
    }
    this.refresh();
  }

  renderParam(p) {
    const n = this.app.params.kinematics.joints.length;
    if (p.kind === 'table') return this.renderTable(p, n);
    if (p.kind === 'jointvec') {
      const row = h('tr', h('td', h('div.prow__label', h('span', p.label), helpDot(p.help))));
      for (let j = 0; j < n; j++) row.append(h('td', this.makeInput({ ...p, path: `${p.path}.${j}` })));
      row.append(h('td.unitcol', UNITS[p.unit]?.display || ''));
      const t = h('div.ptable-wrap', h('table.ptable', h('thead', h('tr', h('th', ''), ...Array.from({ length: n }, (_, j) => h('th', `J${j + 1}`)), h('th', ''))), h('tbody', row)));
      return h('div.stack', { dataset: { search: `${p.label} ${p.help || ''} ${p.path}`.toLowerCase() } }, t);
    }
    if (p.kind === 'vec3') {
      const ins = [0, 1, 2].map((k) => this.makeInput({ ...p, kind: 'number', path: `${p.path}.${k}` }));
      return h('div.prow.prow--vec', { dataset: { search: `${p.label} ${p.help || ''} ${p.path}`.toLowerCase() } },
        h('div.prow__label', h('span', p.label), helpDot(p.help)),
        h('div.prow__input', ...ins, h('span.unit', UNITS[p.unit]?.display || '')));
    }
    const input = this.makeInput(p);
    const lab = h('div.prow__label', h('span', { title: p.path }, p.label));
    if (p.help) lab.append(helpDot(p.help));
    return h('div.prow', { dataset: { search: `${p.label} ${p.help || ''} ${p.path}`.toLowerCase() } },
      lab,
      h('div.prow__input', input, p.kind === 'number' || p.kind === 'int' ? h('span.unit', UNITS[p.unit]?.display || '') : null));
  }

  renderTable(p, n) {
    const body = h('tbody');
    const head = h('tr', h('th', p.label), ...Array.from({ length: n }, (_, j) => h('th', `J${j + 1}`)), h('th', ''));
    for (const col of p.columns) {
      const lab = h('div.prow__label', h('span', col.label));
      if (col.help) lab.append(helpDot(col.help));
      const tr = h('tr', { dataset: { search: `${col.label} ${col.help || ''} ${p.label} ${p.path}.${col.key}`.toLowerCase(), group: col.group || '' } }, h('td', lab));
      for (let j = 0; j < n; j++) {
        const td = h('td', this.makeInput({ ...col, path: `${p.path}.${j}.${col.key}` }, { compact: true }));
        td.dataset.joint = j;
        tr.append(td);
      }
      tr.append(h('td.unitcol', col.kind === 'number' ? (UNITS[col.unit]?.display || '') : ''));
      body.append(tr);
    }
    const table = h('table.ptable', h('thead', head), body);
    return h('div.ptable-wrap', { dataset: { table: p.path } }, table);
  }

  makeInput(d) {
    const app = this.app;
    const id = `p-${d.path.replace(/\./g, '-')}`;
    let el;
    if (d.kind === 'enum') {
      el = h('select.field', { id }, ...d.options.map(([v, l]) => h('option', { value: v }, l)));
      el.addEventListener('change', () => this.commit(d, el.value));
    } else if (d.kind === 'bool') {
      el = h('input', { type: 'checkbox', id });
      el.addEventListener('change', () => this.commit(d, el.checked));
    } else if (d.kind === 'text') {
      el = h('input.field', { type: 'text', id });
      el.addEventListener('change', () => this.commit(d, el.value));
    } else {
      el = h('input.field.field--num', { type: 'text', inputmode: 'decimal', id, spellcheck: false, autocomplete: 'off' });
      const u = UNITS[d.unit] || UNITS.none;
      el.addEventListener('change', () => {
        const raw = parseFloat(String(el.value).replace(',', '.'));
        if (Number.isNaN(raw)) { this.refreshInput(this.inputs.find((x) => x.el === el)); return; }
        let v = d.kind === 'int' ? Math.round(raw) : fromDisplay(raw, d.unit);
        if (d.min !== undefined && v < d.min) { v = d.min; toast(`${d.label} : minimum ${fmtV(d.min, d.unit)}`); }
        if (d.max !== undefined && v > d.max) { v = d.max; toast(`${d.label} : maximum ${fmtV(d.max, d.unit)}`); }
        this.commit(d, v);
      });
      el.addEventListener('keydown', (e) => {
        if (e.key !== 'ArrowUp' && e.key !== 'ArrowDown') return;
        e.preventDefault();
        const cur = getPath(app.params, d.path);
        let stepSI = d.step ?? (d.kind === 'int' ? 1 : Math.abs(cur) * 0.05 || 0.01);
        if (e.shiftKey) stepSI *= 10;
        let v = cur + (e.key === 'ArrowUp' ? stepSI : -stepSI);
        if (d.min !== undefined) v = Math.max(d.min, v);
        if (d.max !== undefined) v = Math.min(d.max, v);
        this.commit(d, d.kind === 'int' ? Math.round(v) : v);
      });
      void u;
    }
    this.inputs.push({ el, d });
    return el;
  }

  commit(d, value) {
    setPath(this.app.params, d.path, value);
    this.app.onParamChange(d.path);
    const rec = this.inputs.find((x) => x.d.path === d.path);
    if (rec) this.refreshInput(rec);
    this.updateDims();
  }

  refreshInput(rec) {
    const { el, d } = rec;
    const v = getPath(this.app.params, d.path);
    if (document.activeElement === el && d.kind !== 'bool' && d.kind !== 'enum') return;
    if (d.kind === 'enum') el.value = v;
    else if (d.kind === 'bool') el.checked = !!v;
    else if (d.kind === 'text') el.value = v ?? '';
    else if (v === undefined || v === null) el.value = '';
    else if (d.kind === 'int') el.value = String(v);
    else {
      const u = UNITS[d.unit] || UNITS.none;
      const dv = toDisplay(v, d.unit);
      const dec = Math.abs(dv) > 0 && Math.abs(dv) < 0.01 ? 6 : u.decimals + (Math.abs(dv) < 1 && u.decimals < 3 ? 2 : 0);
      el.value = String(Number(dv.toFixed(Math.min(dec, 8))));
    }
  }

  /** Relit toutes les valeurs (après préréglage, import, auto-réglage…). */
  refresh() {
    for (const rec of this.inputs) this.refreshInput(rec);
    this.updateDims();
    const warns = validateParams(structuredClone(this.app.params));
    this.warnBox.replaceChildren(...warns.slice(0, 6).map((w) => h('div', `⚠ ${w}`)));
  }

  /** Grise les cellules non pertinentes (ex. couple de maintien pour un moteur brushless). */
  updateDims() {
    const acts = this.app.params.actuators.joints;
    this.container.querySelectorAll('[data-table="actuators.joints"] tr[data-group]').forEach((tr) => {
      const g = tr.dataset.group;
      tr.querySelectorAll('td[data-joint]').forEach((td) => {
        const t = acts[+td.dataset.joint]?.type;
        const rel = !g || g === 'sensor' || (GROUP_FOR_TYPE[t] || []).includes(g);
        td.style.opacity = rel ? '' : '0.4';
      });
    });
  }

  filter(q) {
    q = q.trim().toLowerCase();
    for (const det of this.groups) {
      let any = false;
      det.querySelectorAll('[data-search]').forEach((el) => {
        const show = !q || el.dataset.search.includes(q);
        el.hidden = !show;
        if (show) any = true;
      });
      det.hidden = !!q && !any;
      if (q && any) det.open = true;
    }
  }

  async importJSON() {
    const f = await pickFile('.json,application/json');
    if (!f) return;
    try {
      const p = JSON.parse(f.text);
      if (!p.kinematics || !p.dynamics) throw new Error('fichier non reconnu');
      this.app.loadParams(p, `Configuration importée : ${f.name}`);
    } catch (e) {
      toast(`Import impossible : ${e.message}`);
    }
  }
}

function fmtV(v, unit) {
  const u = UNITS[unit] || UNITS.none;
  return `${Number(toDisplay(v, unit).toFixed(u.decimals))} ${u.display}`;
}

export { attachHelp };
