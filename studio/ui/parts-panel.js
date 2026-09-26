// ORION Studio — panneau « Pièces 3D » : pièces imprimables, nomenclature, téléchargements.

import { h, btn, toast, saveFile } from './dom.js';
import { makeZip } from './zip.js';
import { buildBOM, bomToCSV } from '../cad/bom.js';

export class PartsPanel {
  constructor(container, app) {
    this.app = app;
    this.c = container;
    this.selected = null;
  }

  /** Reconstruit la liste après génération de la CAO. */
  build() {
    const app = this.app, cad = app.cad;
    this.c.replaceChildren();
    if (!cad?.asm) {
      this.c.append(h('p.sect__note', app.cad?.ready === false ? 'Chargement du moteur de CAO…' : 'La géométrie actuelle ne correspond pas à l’architecture ORION-6 : pièces non générées. Rechargez le préréglage ORION-6 MAKER pour les obtenir.'));
      return;
    }
    const list = cad.printedParts();
    const bom = buildBOM(cad.asm);
    this.bom = bom;
    const totalMass = list.reduce((s, x) => s + x.mass * (x.part.qty || 1), 0);
    const hours = (totalMass * 1000) / 14;
    const lay = cad.asm.layout;
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Pièces imprimables'), h('span.sect__note', `${list.length} fichiers`)),
      h('dl.metric',
        h('div', h('dt', 'Filament'), h('dd', `${(totalMass * 1000).toFixed(0)} g`)),
        h('div', h('dt', 'Impression'), h('dd', `≈ ${hours.toFixed(0)} h`)),
        h('div', h('dt', 'Budget'), h('dd', `≈ ${Math.round(bom.totalEuro / 10) * 10} €`))),
      h('p.sect__note', `Géométrie générée depuis la table DH : d1 = ${lay.d1.toFixed(0)} mm, a2 = ${lay.a2.toFixed(0)} mm, d4 = ${lay.d4.toFixed(0)} mm → bride à ${lay.d6.toFixed(1)} mm du poignet, TCP à ${lay.tcp.toFixed(1)} mm de la bride. Toutes les pièces tiennent sur un plateau 220 × 220 mm.`),
      h('div.row',
        btn('Tout (.zip)', () => this.downloadAll(), { cls: 'btn--sm btn--primary', icon: 'download' }),
        btn('Nomenclature (CSV)', () => saveFile('orion6_nomenclature.csv', bomToCSV(bom), 'text/csv'), { cls: 'btn--sm', icon: 'download' }),
        btn('Isoler', () => this.highlight(null), { cls: 'btn--sm btn--ghost', title: 'Réafficher toutes les pièces' }))));
    if (cad.asm.warnings.length) this.c.append(h('div.warnlist', ...cad.asm.warnings.map((w) => h('div', `⚠ ${w}`))));
    const groups = new Map();
    for (const x of list) {
      const g = x.part.group || 'Divers';
      if (!groups.has(g)) groups.set(g, []);
      groups.get(g).push(x);
    }
    for (const [g, items] of groups) {
      const box = h('div.parts');
      for (const x of items) {
        const p = x.part;
        const row = h('button.part', { type: 'button', 'aria-current': 'false' },
          h('b', p.name),
          h('span.qty', `×${p.qty || 1}`),
          h('small', `${(x.mass * 1000).toFixed(0)} g · ${x.size.map((v) => v.toFixed(0)).join(' × ')} mm · ${p.material || 'PETG/PLA+'} ${Math.round((p.fill ?? 0.5) * 100)} %${p.note ? ` — ${p.note}` : ''}`));
        row.addEventListener('click', () => { this.highlight(p.id); this.c.querySelectorAll('.part').forEach((r) => r.setAttribute('aria-current', String(r === row))); });
        const dl = btn('STL', () => saveFile(`${p.id}.stl`, new Blob([cad.stlOf(p)], { type: 'model/stl' })), { cls: 'btn--sm', icon: 'download', title: `Télécharger ${p.id}.stl (orienté pour l’impression)` });
        box.append(h('div.row.row--nowrap', h('div.grow', row), dl));
      }
      this.c.append(h('details.pgroup', { open: g.startsWith('Module J2') || g === 'Socle' }, h('summary', g, h('span.count', String(items.length))), h('div.pgroup__body', box)));
    }
    // Nomenclature (composants du commerce)
    const tb = h('tbody');
    for (const i of bom.items.filter((it) => it.cat !== 'Pièces imprimées')) {
      tb.append(h('tr', h('td', i.desc), h('td', `${Math.round(i.qty * 100) / 100} ${i.unit}`), h('td', i.price != null ? `${(i.price * i.qty).toFixed(0)} €` : '')));
    }
    this.c.append(h('details.pgroup', h('summary', 'Composants du commerce', h('span.count', String(bom.items.length))),
      h('div.pgroup__body', h('div.dtable-wrap', h('table.dtable', h('thead', h('tr', h('th', 'Désignation'), h('th', 'Qté'), h('th', 'Prix'))), tb)),
        h('p.sect__note', 'Prix indicatifs pour une commande unitaire en ligne ; ils varient fortement selon les fournisseurs.'))));
  }

  highlight(id) {
    const view = this.app.view;
    view.robot?.root.traverse((o) => {
      if (!o.isMesh) return;
      const on = !id || o.userData.part === id;
      o.material.transparent = !on;
      o.material.opacity = on ? 1 : 0.12;
      o.material.depthWrite = on;
    });
    if (id) toast(`Pièce ${id} mise en évidence (cliquez « Isoler » pour tout réafficher)`);
  }

  downloadAll() {
    const cad = this.app.cad;
    const t0 = performance.now();
    const files = cad.printedParts().map((x) => ({ name: `stl/${x.part.id}.stl`, data: cad.stlOf(x.part) }));
    files.push({ name: 'nomenclature.csv', data: bomToCSV(this.bom) });
    const zip = makeZip(files);
    saveFile('orion6_pieces.zip', new Blob([zip], { type: 'application/zip' }));
    toast(`${files.length} fichiers générés en ${((performance.now() - t0) / 1000).toFixed(1)} s`);
  }
}
