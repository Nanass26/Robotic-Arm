// Génère les parties de la documentation qui proviennent directement des données du projet :
//   docs/03-parametres.md     ← schéma de TOUS les paramètres (studio/core/schema.js) + préréglages
//   docs/07-impression-3d.md  ← liste des pièces imprimées (hardware/parts.json)
//   docs/08-nomenclature.md   ← nomenclature chiffrée (hardware/bom.csv)
// Usage : node tools/gen-docs.mjs   (après npm run build:cad si la CAO a changé)
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { SCHEMA, getPath } from '../studio/core/schema.js';
import { UNITS, toDisplay } from '../studio/core/units.js';
import { makeOrion6Maker, makeOrion6Pro } from '../studio/core/defaults.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const out = (rel, text) => {
  mkdirSync(dirname(join(root, rel)), { recursive: true });
  writeFileSync(join(root, rel), text);
  console.log(`✓ ${rel}`);
};
const GEN = (src) => `<!-- FICHIER GÉNÉRÉ par tools/gen-docs.mjs depuis ${src} — ne pas modifier à la main. -->\n\n`;
const esc = (s) => String(s ?? '').replace(/\|/g, '\\|').replace(/\n/g, ' ');
const nf = (v, d) => {
  if (!Number.isFinite(v)) return String(v);
  const r = Number(v.toFixed(d));
  return r.toLocaleString('fr-FR', { maximumFractionDigits: d, useGrouping: Math.abs(r) >= 10000 });
};

// ------------------------------------------------------------------ 03 — paramètres
function unitLabel(u) {
  const U = UNITS[u];
  if (!U || !U.display || U.display === ':1') return u === 'ratio' ? ':1' : '';
  return U.display;
}
function fmtVal(v, d) {
  if (v === undefined || v === null) return '—';
  if (d.kind === 'bool') return v ? 'oui' : 'non';
  if (d.kind === 'enum') {
    const o = (d.options || []).find((x) => x[0] === v);
    return o ? `${o[1]}` : String(v);
  }
  if (d.kind === 'text') return v === '' ? '—' : `« ${v} »`;
  if (Array.isArray(v)) return v.map((x) => fmtVal(x, { ...d, kind: 'number' })).join(' / ');
  if (typeof v !== 'number') return String(v);
  const U = UNITS[d.unit] || UNITS.none;
  return nf(toDisplay(v, d.unit), U.decimals ?? 3);
}
function fmtRange(d) {
  if (d.kind === 'enum') return (d.options || []).map((o) => `\`${o[0]}\``).join(', ');
  if (d.kind === 'bool') return 'oui / non';
  if (d.min === undefined && d.max === undefined) return '';
  const U = UNITS[d.unit] || UNITS.none;
  const f = (x) => (x === undefined ? '…' : nf(toDisplay(x, d.unit), Math.max(U.decimals ?? 3, 0)));
  return `${f(d.min)} … ${f(d.max)}`;
}
function tableValues(params, tablePath, key, d) {
  const rows = getPath(params, tablePath);
  if (!Array.isArray(rows)) return '—';
  const vals = rows.map((r) => getPath(r, key));
  if (vals.every((v) => v === undefined)) return '—';
  const s = vals.map((v) => fmtVal(v, d));
  return s.every((x) => x === s[0]) ? `${s[0]} (tous)` : s.join(' · ');
}

function genParams() {
  const maker = makeOrion6Maker(), pro = makeOrion6Pro();
  let count = 0, countAxis = 0;
  const L = [];
  for (const sec of SCHEMA) {
    L.push(`## ${sec.title}`, '');
    if (sec.help) L.push(sec.help, '');
    const rows = [];
    for (const p of sec.params) {
      if (p.kind === 'table') {
        rows.push(`| **${esc(p.label)}** — tableau par articulation (J1 · J2 · … · J${getPath(maker, p.path)?.length ?? 6}) | \`${p.path}[i]\` | | | | ${esc(p.help)} |`);
        for (const c of p.columns) {
          count++;
          countAxis += getPath(maker, p.path)?.length ?? 1;
          rows.push(`| ↳ ${esc(c.label)} | \`.${c.key}\` | ${unitLabel(c.unit)} | ${esc(fmtRange(c))} | ${esc(tableValues(maker, p.path, c.key, c))} | ${esc(c.help)}${tableValues(pro, p.path, c.key, c) !== tableValues(maker, p.path, c.key, c) ? ` *PRO : ${esc(tableValues(pro, p.path, c.key, c))}*` : ''} |`);
        }
      } else {
        count++;
        countAxis += p.kind === 'jointvec' ? 6 : p.kind === 'vec3' ? 3 : 1;
        const vm = fmtVal(getPath(maker, p.path), p), vp = fmtVal(getPath(pro, p.path), p);
        rows.push(`| ${esc(p.label)} | \`${p.path}\` | ${unitLabel(p.unit)} | ${esc(fmtRange(p))} | ${esc(vm)} | ${esc(p.help)}${vp !== vm ? ` *PRO : ${esc(vp)}*` : ''} |`);
      }
    }
    L.push('| Paramètre | Clé | Unité | Plage | Valeur ORION-6 MAKER | Rôle |', '|---|---|---|---|---|---|', ...rows, '');
  }
  const head = [
    GEN('studio/core/schema.js et studio/core/defaults.js'),
    '# 3. Référence de tous les paramètres',
    '',
    `ORION Studio expose **${count} paramètres** (soit **${countAxis} valeurs** en comptant chaque articulation). `
      + 'Ils sont tous modifiables en direct dans l’onglet **Paramètres** (champ de recherche, aide au survol de `?`), '
      + 'enregistrés dans le fichier de configuration `JSON` et, pour ceux qui concernent le matériel, exportés vers le firmware '
      + '(`orion_config.h` ou commandes `SET`).',
    '',
    '- Les valeurs sont stockées en unités SI (m, rad, kg, s) et **affichées** dans les unités du tableau (mm, °, g·cm²…).',
    '- La colonne *Valeur* donne le réglage du préréglage **ORION-6 MAKER** (pas-à-pas + réducteurs cycloïdaux imprimés) ; '
      + 'lorsque le préréglage **ORION-6 PRO** (moteurs QDD en mode MIT) diffère, sa valeur est indiquée en italique.',
    '- Pour les tableaux par articulation, les valeurs sont données dans l’ordre J1 · J2 · J3 · J4 · J5 · J6.',
    '- La validation borne automatiquement toute valeur hors plage (import JSON compris) et signale la correction dans le journal.',
    '',
    '## Sommaire',
    '',
    // ancres au format GitHub : minuscules, lettres accentuées conservées, ponctuation retirée
    ...SCHEMA.map((s) => `- [${s.title}](#${s.title.toLowerCase().replace(/[^\p{L}\p{N} -]/gu, '').trim().replace(/ /g, '-')})`),
    '',
  ];
  out('docs/03-parametres.md', [...head, ...L].join('\n'));
}

// ------------------------------------------------------------------ 07 — impression 3D
function genPrint() {
  const info = JSON.parse(readFileSync(join(root, 'hardware/parts.json'), 'utf8'));
  const parts = info.parts;
  const totalG = parts.reduce((s, p) => s + p.massG * p.qty, 0);
  const pieces = parts.reduce((s, p) => s + p.qty, 0);
  const groups = [...new Set(parts.map((p) => p.group))];
  const rows = [];
  for (const g of groups) {
    rows.push(`| **${esc(g)}** | | | | | | |`);
    for (const p of parts.filter((x) => x.group === g)) {
      rows.push(`| \`${p.file}.stl\` | ${esc(p.name)} | ${p.qty} | ${p.material} · ${Math.round(p.fill * 100)} % | ${p.size.map((v) => nf(v, 0)).join(' × ')} | ${nf(p.massG, 0)} g | ${esc(p.note)} |`);
    }
  }
  const text = `${GEN('hardware/parts.json')}# 7. Impression 3D des pièces

Toutes les pièces sont générées par la CAO paramétrique (\`npm run build:cad\`) et se trouvent dans
[\`hardware/stl/\`](../hardware/stl/). Elles tiennent sur un plateau **220 × 220 × 250 mm** (vérifié automatiquement)
et sont déjà **orientées pour l’impression** (face d’appui en Z = 0, sans supports sauf mention).

**Total : ${parts.length} fichiers STL, ${pieces} pièces, ≈ ${nf(totalG / 1000, 2)} kg de matière** (+ 25 % de marge pour les purges et
supports éventuels).

> ⚠️ **Imprimez d’abord \`tolerance-test.stl\`.** Il contient les alésages de goupilles Ø3/Ø4/Ø5, un logement de
> roulement 688, un logement d’insert M3 et une fente de méplat. Si les goupilles n’entrent pas à la main (ajustement
> glissant) ou flottent, corrigez le paramètre *Jeu d’impression* (CAO) ou la compensation XY de votre trancheur **avant**
> d’imprimer les réducteurs. Ce projet n’a pas encore été validé sur un exemplaire physique.

## Réglages conseillés

| Réglage | Pièces de structure | Réducteurs (disques, cames, flasques) | Patins de pince |
|---|---|---|---|
| Matériau | PETG (ou PLA+, ASA près des moteurs chauds) | PETG ou PA-CF si disponible | TPU 95A |
| Buse / couche | 0,4 mm / 0,2 mm | 0,4 mm / **0,15 mm** | 0,4 mm / 0,2 mm |
| Parois | 4 périmètres | **5 périmètres** | 3 |
| Remplissage | voir tableau (gyroïde) | voir tableau (100 % pour cames et disques) | 100 % |
| Couches pleines dessus/dessous | 5 / 5 | 6 / 6 | 4 / 4 |
| Vitesse | 60–100 mm/s | **≤ 50 mm/s** (précision des profils) | 25 mm/s |
| Compensation XY | selon le test de tolérances | idem (profils critiques) | — |
| Supports | non (sauf note) | non | non |

- **Retrait / dilatation** : laissez refroidir les pièces sur le plateau (PETG) pour limiter le gauchissement des carters.
- **Inserts filetés** : fer à 230–245 °C (PETG), poussez l’insert bien perpendiculairement ; laissez refroidir avant de
  visser. Les logements font Ø4,0 mm (M3) et Ø5,6 mm (M4), profondeur 6 mm.
- **Goupilles** : les alésages des couronnes sont prévus pour des goupilles acier trempé ISO 8734 enfoncées à la presse
  (étau). Une goutte de colle frein/époxy est inutile si l’ajustement est correct.
- **Disques cycloïdaux** : ébavurez l’arête inférieure (« pied d’éléphant ») au cutter ; c’est la cause n°1 de points
  durs. Les deux disques d’un même module (A et B) sont décalés d’un demi-lobe : ils ne sont **pas** interchangeables.
- **Cames** : 100 % de remplissage, méplat orienté vers le haut ; vérifiez l’excentricité au comparateur si possible.

## Liste des pièces

| Fichier | Pièce | Qté | Matière · remplissage | Encombrement (mm) | Masse (unité) | Remarques |
|---|---|---|---|---|---|---|
${rows.join('\n')}

## Ordre d’impression conseillé

1. \`tolerance-test\` → ajuster le trancheur.
2. Un module complet **CY-S** (le plus petit, J4–J6) → assembler et tester à la main (rotation fluide, pas de jeu
   radial perceptible, couple d’entraînement faible). Corriger si besoin avant de lancer les autres modules.
3. Modules **CY-M** (J1, J3) et **CY-L** (J2).
4. Structure : socle, plaque de base, tourelle, raccords de bras, coude, poignet, chape.
5. Pince (corps, pignon, doigts, patins TPU).

Les masses ci-dessus proviennent de la CAO (volume × densité × taux de remplissage estimé) ; elles alimentent
directement le modèle dynamique du simulateur (\`studio/core/generated/massprops.js\`).
`;
  out('docs/07-impression-3d.md', text);
}

// ------------------------------------------------------------------ 08 — nomenclature
function genBom() {
  const lines = readFileSync(join(root, 'hardware/bom.csv'), 'utf8').trim().split(/\r?\n/);
  const rows = lines.slice(1).map((l) => {
    const [cat, ref, des, qty, unit, price, note] = l.split(';');
    return { cat, ref, des, qty: +qty, unit, price: price === '' ? null : +price, note };
  });
  const cats = [...new Set(rows.map((r) => r.cat))].filter((c) => c !== 'Pièces imprimées');
  let total = 0;
  const L = [];
  const catTotals = [];
  for (const c of cats) {
    const items = rows.filter((r) => r.cat === c);
    // regroupe les références identiques (issues de plusieurs modules)
    const merged = new Map();
    for (const it of items) {
      const k = `${it.ref}|${it.des}|${it.price}`;
      if (merged.has(k)) merged.get(k).qty += it.qty;
      else merged.set(k, { ...it });
    }
    let sub = 0;
    L.push(`### ${c}`, '', '| Référence | Désignation | Quantité | Prix unitaire indicatif | Sous-total |', '|---|---|---|---|---|');
    for (const it of merged.values()) {
      const st = it.price != null ? it.qty * it.price : 0;
      sub += st;
      L.push(`| ${esc(it.ref)} | ${esc(it.des)}${it.note ? ` — ${esc(it.note)}` : ''} | ${nf(it.qty, 2)} ${it.unit} | ${it.price != null ? `${nf(it.price, 2)} €` : '—'} | ${it.price != null ? `${nf(st, 2)} €` : '—'} |`);
    }
    L.push(`| | **Sous-total ${esc(c.toLowerCase())}** | | | **${nf(sub, 0)} €** |`, '');
    total += sub;
    catTotals.push([c, sub]);
  }
  const printed = rows.filter((r) => r.cat === 'Pièces imprimées');
  const text = `${GEN('hardware/bom.csv')}# 8. Nomenclature (ORION-6 MAKER)

Nomenclature complète générée à partir de la CAO (\`npm run build:cad\` → [\`hardware/bom.csv\`](../hardware/bom.csv),
format tableur, séparateur « ; »). Les prix sont **indicatifs** (achats à l’unité chez des revendeurs grand public,
2026) : ils varient fortement selon les fournisseurs et les quantités.

**Coût matériel estimé : ≈ ${nf(total, 0)} €** (hors imprimante, outillage et frais de port).

| Poste | Montant |
|---|---|
${catTotals.map(([c, v]) => `| ${esc(c)} | ${nf(v, 0)} € |`).join('\n')}
| **Total** | **${nf(total, 0)} €** |

${L.join('\n')}
### Pièces imprimées

${printed.length} pièces (détail, orientation et réglages dans [7. Impression 3D](07-impression-3d.md)).

### Variante ORION-6 PRO (moteurs QDD en mode MIT)

La variante PRO remplace les six ensembles « moteur pas-à-pas + réducteur cycloïdal + driver » par des actionneurs
brushless à réducteur planétaire intégré pilotés en **mode MIT** sur bus CAN :

| Articulation | Actionneur | Couple crête / nominal | Remarque |
|---|---|---|---|
| J1, J2, J3 | Damiao **DM-J4340** (40:1) | 27 / 9 N·m | 24 V, codeur, driver intégré |
| J4, J5, J6 | Damiao **DM-J4310** (10:1) | 7 / 3 N·m | 24 V |
| Liaison | Teensy 4.1 + transceiver CAN 3,3 V (SN65HVD230 ou TJA1051T/3) | — | firmware \`firmware/mit_bridge\` |
| Puissance | Alimentation 24 V ≥ 15 A + arrêt d’urgence coupant la puissance | — | |

> Les brides d’adaptation imprimées pour ces moteurs ne sont **pas encore modélisées** : la CAO fournie est celle de la
> variante MAKER. La variante PRO est complète côté simulation, réglage (mode MIT) et firmware.
`;
  out('docs/08-nomenclature.md', text);
}

genParams();
genPrint();
genBom();
