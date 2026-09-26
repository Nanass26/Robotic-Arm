// ORION-6 — Génération de la CAO : STL imprimables, propriétés de masse, capsules de collision,
// nomenclature, contrôles d’interférence et balayage des articulations.
//
//   node tools/build-cad.mjs            (tout)
//   node tools/build-cad.mjs --fast     (sans balayage des articulations)

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import Module from 'manifold-3d';
import { setupKit, toSTL, massProps } from '../studio/cad/kit.js';
import { buildOrion6, printMesh, partMass } from '../studio/cad/orion6.js';
import { buildBOM, bomToCSV } from '../studio/cad/bom.js';
import { staticInterferences, bboxOverlap } from '../studio/cad/checks.js';
import { makeOrion6Maker, MOTOR_LIBRARY, QDD_LIBRARY } from '../studio/core/defaults.js';
import { Kinematics } from '../studio/core/kinematics.js';
import { mat4InvRigid, mat4Mul, mat4TransformPoint, mat4Rot, mat3T, mat3Mul, inertiaFromVector, inertiaToVector, parallelAxis } from '../studio/core/math3d.js';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const FAST = process.argv.includes('--fast');
const BED = [220, 220, 250];

const wasm = await Module({ wasmBinary: fs.readFileSync(path.join(root, 'node_modules/manifold-3d/manifold.wasm')) });
wasm.setup();
setupKit(wasm);

const params = makeOrion6Maker();
const t0 = Date.now();
const asm = buildOrion6(params);
console.log(`Assemblage : ${asm.parts.length} pièces en ${Date.now() - t0} ms`);
for (const w of asm.warnings) console.log(`  ⚠ ${w}`);

// ------------------------------------------------------------------ STL (pièces imprimées)
const stlDir = path.join(root, 'hardware', 'stl');
fs.rmSync(stlDir, { recursive: true, force: true });
fs.mkdirSync(stlDir, { recursive: true });
const printed = new Map(); // fichier → { part, qty }
for (const p of asm.parts) {
  if (!p.printed) continue;
  // Les pièces de module identiques (même taille) sont regroupées
  const file = p.moduleKey ? `${p.moduleSize}_${p.moduleKey}` : p.id;
  if (printed.has(file)) printed.get(file).qty += p.qty || 1;
  else printed.set(file, { part: p, qty: p.qty || 1, file });
}
const report = [];
const bedIssues = [];
for (const [file, rec] of printed) {
  const m = printMesh(rec.part);
  const bb = m.boundingBox();
  const size = [0, 1, 2].map((k) => bb.max[k] - bb.min[k]);
  const fits = size[2] <= BED[2] && ((size[0] <= BED[0] && size[1] <= BED[1]) || (size[0] <= BED[1] && size[1] <= BED[0]));
  if (!fits) bedIssues.push(`${file} : ${size.map((v) => v.toFixed(0)).join(' × ')} mm`);
  fs.writeFileSync(path.join(stlDir, `${file}.stl`), toSTL(m, file));
  const mp = partMass(rec.part, MOTOR_LIBRARY);
  report.push({ file, name: rec.part.name.replace(/ (CY-[SML])$/, ''), group: rec.part.moduleKey ? `Module ${rec.part.moduleSize}` : rec.part.group, qty: rec.qty, size, massG: mp.mass * 1000, fill: rec.part.fill, material: rec.part.material || 'PETG', note: rec.part.note || '' });
}
console.log(`STL : ${printed.size} fichiers dans hardware/stl/`);
if (bedIssues.length) console.log('  ⚠ Hors plateau 220×220×250 :', bedIssues);

// ------------------------------------------------------------------ propriétés de masse par segment
function linkMassProps(parts, kinParams) {
  const kin = new Kinematics(kinParams);
  const f0 = kin.fk(new Array(6).fill(0));
  const links = [];
  for (let i = 1; i <= 6; i++) {
    const ps = parts.filter((p) => p.link === i && !/^gripper-/.test(p.id));
    links.push(combine(ps, f0.frames[i]));
  }
  const tool = combine(parts.filter((p) => /^gripper-/.test(p.id)), f0.frames[6]);
  return { links, tool };
}

function combine(ps, frame) {
  let m = 0, c = [0, 0, 0];
  const items = ps.map((p) => ({ p, mp: p.mp || partMass(p, MOTOR_LIBRARY) }));
  for (const { mp } of items) { m += mp.mass; c = c.map((v, k) => v + mp.com[k] * mp.mass); }
  if (m <= 0) return { mass: 0, com: [0, 0, 0], inertia: [0, 0, 0, 0, 0, 0] };
  c = c.map((v) => v / m);
  // Inertie au CdG global (axes monde)
  let I = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (const { mp } of items) {
    const Ii = inertiaFromVector(mp.inertia);
    const r = mp.com.map((v, k) => v - c[k]);
    I = I.map((v, k) => v + parallelAxis(Ii, mp.mass, r)[k]);
  }
  // Passage dans le repère DH du segment (pose zéro)
  const inv = mat4InvRigid(frame);
  const comL = mat4TransformPoint(inv, c);
  const R = mat4Rot(frame);
  const IL = mat3Mul(mat3Mul(mat3T(R), I), R);
  return { mass: m, com: comL, inertia: inertiaToVector(IL) };
}

for (const p of asm.parts) p.mp = partMass(p, MOTOR_LIBRARY);
const makerMP = linkMassProps(asm.parts, params);

// Variante PRO : même structure, modules cycloïdaux remplacés par des QDD (cylindre équivalent)
const proParts = asm.parts.filter((p) => !/^J\d-/.test(p.id)).map((p) => ({ ...p }));
const qddModels = ['DM4340', 'DM4340', 'DM4340', 'DM4310', 'DM4310', 'DM4310'];
for (let k = 1; k <= 6; k++) {
  const hs = asm.parts.find((p) => p.id === `J${k}-housing`);
  const mot = asm.parts.find((p) => p.id === `J${k}-motor`);
  const mh = hs.mp, mm = mot.mp;
  const com = mh.com.map((v, i) => (v * mh.mass + mm.com[i] * mm.mass) / (mh.mass + mm.mass));
  const q = QDD_LIBRARY[qddModels[k - 1]];
  // Stator (≈ 70 %) sur le segment amont, rotor (≈ 30 %) sur le segment aval
  const I0 = q.mass * 0.03 * 0.03;
  proParts.push({ id: `qdd${k}-stator`, link: k - 1, mp: { mass: q.mass * 0.7 + 0.04, com, inertia: [I0, I0, I0, 0, 0, 0] } });
  proParts.push({ id: `qdd${k}-rotor`, link: k, mp: { mass: q.mass * 0.3 + 0.03, com, inertia: [I0 / 2, I0 / 2, I0 / 2, 0, 0, 0] } });
}
const proMP = linkMassProps(proParts, params);

// ------------------------------------------------------------------ capsules de collision
function capsulesFor(parts, kinParams) {
  const kin = new Kinematics(kinParams);
  const f0 = kin.fk(new Array(6).fill(0));
  const caps = [];
  const skip = /(cam|disc|pins|outPins|camBearings|supBearing|mainBearing|flange|pad|pinion)$/;
  for (const p of parts) {
    if (p.link < 0 || skip.test(p.id)) continue;
    const bb = p.mesh.boundingBox();
    const size = [0, 1, 2].map((k) => (bb.max[k] - bb.min[k]) / 1000);
    const ctr = [0, 1, 2].map((k) => (bb.max[k] + bb.min[k]) / 2000);
    const ax = size.indexOf(Math.max(...size));
    const others = [0, 1, 2].filter((k) => k !== ax);
    const r = Math.max(size[others[0]], size[others[1]]) / 2 * 0.92;
    const half = Math.max(0, size[ax] / 2 - r);
    const a = ctr.slice(), b = ctr.slice();
    a[ax] -= half; b[ax] += half;
    const link = /^gripper-/.test(p.id) ? 6 : p.link;
    const inv = mat4InvRigid(f0.frames[link]);
    caps.push({ link, part: p.id, p0: mat4TransformPoint(inv, a).map((v) => +v.toFixed(5)), p1: mat4TransformPoint(inv, b).map((v) => +v.toFixed(5)), r: +r.toFixed(4) });
  }
  return caps;
}
const capsules = capsulesFor(asm.parts, params);

// ------------------------------------------------------------------ interférences statiques
const tI = Date.now();
const interferences = staticInterferences(asm);
console.log(`Interférences statiques : ${interferences.length} (contrôle en ${Date.now() - tI} ms)`);
for (const it of interferences.slice(0, 40)) console.log(`  ⚠ ${it.a} ∩ ${it.b} : ${it.volume.toFixed(1)} mm³`);

// ------------------------------------------------------------------ balayage des articulations
let sweep = null;
let sweepMs = 0;
if (!FAST) {
  const tS = Date.now();
  const kin = new Kinematics(params);
  const f0 = kin.fk(new Array(6).fill(0));
  const linkSolids = [];
  for (let i = 0; i <= 6; i++) {
    const ps = asm.parts.filter((p) => (p.link === i) && !/(cam|disc|camBearings|pins|outPins|supBearing)$/.test(p.id));
    linkSolids.push(ps.length ? wasm.Manifold.union(ps.map((p) => p.mesh)) : null);
  }
  // Mat4 manifold : 16 valeurs en colonnes ; translation convertie en mm
  const toRowMajorMat = (T) => [T[0], T[4], T[8], 0, T[1], T[5], T[9], 0, T[2], T[6], T[10], 0, T[3] * 1000, T[7] * 1000, T[11] * 1000, 1];
  const placed = (i, q) => {
    const f = kin.fk(q);
    const M = mat4Mul(f.frames[i], mat4InvRigid(f0.frames[i]));
    return linkSolids[i].transform(toRowMajorMat(M));
  };
  const collides = (q, j, withTable = false) => {
    // Segments fixes (< j) contre segments mobiles (≥ j) ; hors couple de la liaison j elle-même.
    // Les objets WebAssembly temporaires sont libérés explicitement (pas de ramasse-miettes).
    const fixed = [], moving = [];
    for (let i = 0; i <= 6; i++) { if (!linkSolids[i]) continue; (i < j ? fixed : moving).push([i, placed(i, q)]); }
    let hit = null;
    for (const [a, A] of fixed) {
      for (const [b, B] of moving) {
        if (hit) break;
        if (a === j - 1 && b === j) continue; // liaison du joint balayé : conçue pour tourner
        if (!bboxOverlap(A, B, 0.5)) continue;
        const X = A.intersect(B);
        const v = X.volume();
        X.delete();
        if (v > 5) hit = `${a}/${b}`;
      }
    }
    // Contact avec la table
    if (!hit && withTable) for (const [b, B] of moving) if (b >= 2 && B.boundingBox().min[2] < 0) { hit = `table/${b}`; break; }
    for (const [, m] of [...fixed, ...moving]) m.delete();
    return hit;
  };
  // Butées mécaniques : auto-collisions seules, autres axes à zéro (J5 à 0 puis ±90° pour J1–J4)
  const limits = [];
  const sweepDir = (j, dir, bases, withTable) => {
    for (let a = 5; a <= 180; a += 5) {
      for (const base of bases) {
        const q = base.slice();
        q[j - 1] = (dir * a * Math.PI) / 180;
        const hit = collides(q, j, withTable);
        if (hit) return { a, hit };
      }
    }
    return null;
  };
  for (let j = 1; j <= 6; j++) {
    const res = { joint: j, pos: 180, neg: -180, posBy: '', negBy: '', tablePos: null, tableNeg: null };
    const bases = [[0, 0, 0, 0, 0, 0]];
    for (const dir of [1, -1]) {
      const f = sweepDir(j, dir, bases, false);
      if (f) { if (dir > 0) { res.pos = f.a - 5; res.posBy = f.hit; } else { res.neg = -(f.a - 5); res.negBy = f.hit; } }
      if (j === 2 || j === 3) {
        const t = sweepDir(j, dir, [[0, 0, 0, 0, Math.PI / 2, 0]], true);
        if (t) { if (dir > 0) res.tablePos = t.a - 5; else res.tableNeg = -(t.a - 5); }
      }
    }
    limits.push(res);
    const tbl = res.tablePos !== null || res.tableNeg !== null ? `  [pince vers le bas : table à ${res.tableNeg ?? '—'}° / +${res.tablePos ?? '—'}°]` : '';
    console.log(`  J${j} : ${res.neg}° … +${res.pos}°  ${res.negBy || res.posBy ? `(contacts : ${res.negBy || '-'} / ${res.posBy || '-'})` : ''}${tbl}`);
  }
  sweep = { limits };
  sweepMs = Date.now() - tS;
  console.log(`Balayage des articulations en ${(sweepMs / 1000).toFixed(1)} s`);
}

// ------------------------------------------------------------------ fichiers générés
const r6 = (v) => +v.toFixed(6);
const fmtLink = (l) => ({ mass: +l.mass.toFixed(4), com: l.com.map(r6), inertia: l.inertia.map((v) => +v.toExponential(4)) });
const fmtTool = (l) => ({ mass: +l.mass.toFixed(4), com: l.com.map(r6), inertiaDiag: l.inertia.slice(0, 3).map((v) => +v.toExponential(4)) });
const gen = {
  maker: { source: 'CAO (tools/build-cad.mjs)', flangeD6: r6(asm.layout.d6 / 1000), tcpZ: r6(asm.layout.tcp / 1000), links: makerMP.links.map(fmtLink), tool: fmtTool(makerMP.tool) },
  pro: { source: 'CAO structure + actionneurs QDD estimés', flangeD6: r6(asm.layout.d6 / 1000), tcpZ: r6(asm.layout.tcp / 1000), links: proMP.links.map(fmtLink), tool: fmtTool(proMP.tool) },
};
const capsFile = {
  capsules: capsules.map(({ link, p0, p1, r }) => ({ link, p0, p1, r })),
  ignorePairs: [],
};
fs.writeFileSync(path.join(root, 'studio/core/generated/massprops.js'),
  `// FICHIER GÉNÉRÉ par tools/build-cad.mjs — ne pas modifier à la main.\n` +
  `// Propriétés de masse calculées depuis la CAO (PLA/PETG, remplissage par pièce) + composants.\n` +
  `// Repères : DH standard de chaque segment (pose zéro). Unités SI.\n\n` +
  `export const MASS_PROPERTIES = ${JSON.stringify(gen, null, 2)};\n\n` +
  `// Capsules de collision (repère DH de chaque segment) déduites des pièces.\n` +
  `export const COLLISION_CAPSULES = ${JSON.stringify(capsFile)};\n\n` +
  `export const JOINT_SWEEP = ${JSON.stringify(sweep ? sweep.limits : null)};\n`);
const bom = buildBOM(asm);
fs.mkdirSync(path.join(root, 'hardware'), { recursive: true });
fs.writeFileSync(path.join(root, 'hardware/bom.csv'), bomToCSV(bom));
fs.writeFileSync(path.join(root, 'hardware/parts.json'), JSON.stringify({ layout: asm.layout, parts: report, interferences, sweep, bedIssues, massProperties: gen }, null, 2));
console.log(`Masse totale estimée du bras (segments 1–6 + outil) : ${(makerMP.links.reduce((s, l) => s + l.mass, 0) + makerMP.tool.mass).toFixed(2)} kg`);
console.log(`Filament : ${(bom.filamentKg * 1000).toFixed(0)} g ; budget indicatif ≈ ${bom.totalEuro.toFixed(0)} €`);
console.log('Fichiers : hardware/stl/*.stl, hardware/bom.csv, hardware/parts.json, studio/core/generated/massprops.js');
