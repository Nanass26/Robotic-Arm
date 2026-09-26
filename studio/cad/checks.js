// ORION-6 — Contrôles automatiques de la CAO (utilisés par tools/build-cad.mjs et les tests).

import { printMesh } from './orion6.js';

export const BED = [220, 220, 250];

export function bboxOverlap(a, b, margin = 0) {
  const A = a.boundingBox(), B = b.boundingBox();
  for (let k = 0; k < 3; k++) if (A.max[k] + margin < B.min[k] || B.max[k] + margin < A.min[k]) return false;
  return true;
}

// Ajustements serrés voulus : goupilles emmanchées dans le carter / le flasque
const expectedPress = (a, b) => {
  const ids = [a.id, b.id].sort().join('|');
  return /J\d-housing\|J\d-pins|J\d-flange\|J\d-outPins/.test(ids);
};

/** Interférences entre pièces de l’assemblage en pose zéro (volume > 1 mm³, hors appuis plans). */
export function staticInterferences(asm) {
  const out = [];
  const solid = asm.parts.filter((p) => p.link >= 0);
  for (let i = 0; i < solid.length; i++) {
    for (let j = i + 1; j < solid.length; j++) {
      const A = solid[i], B = solid[j];
      if (!bboxOverlap(A.mesh, B.mesh)) continue;
      const X = A.mesh.intersect(B.mesh);
      const v = X.volume();
      if (v <= 1.0 || (expectedPress(A, B) && v < 60)) { X.delete(); continue; }
      const pcs = X.decompose();
      const thick = pcs.some((pc) => { const bb = pc.boundingBox(); return Math.min(bb.max[0] - bb.min[0], bb.max[1] - bb.min[1], bb.max[2] - bb.min[2]) > 0.05; });
      pcs.forEach((pc) => pc.delete());
      X.delete();
      if (thick) out.push({ a: A.id, b: B.id, volume: v });
    }
  }
  return out;
}

/** Pièces qui ne tiennent pas sur le plateau (orientation d’impression, rotation de 90° autorisée). */
export function bedIssues(asm, bed = BED) {
  const out = [];
  for (const p of asm.parts) {
    if (!p.printed) continue;
    const bb = printMesh(p).boundingBox();
    const s = [0, 1, 2].map((k) => bb.max[k] - bb.min[k]);
    const fits = s[2] <= bed[2] && ((s[0] <= bed[0] && s[1] <= bed[1]) || (s[0] <= bed[1] && s[1] <= bed[0]));
    if (!fits) out.push({ id: p.id, size: s });
  }
  return out;
}
