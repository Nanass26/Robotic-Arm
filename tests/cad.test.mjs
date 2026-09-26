import { test } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import Module from 'manifold-3d';
import { setupKit, massProps } from '../studio/cad/kit.js';
import { MODULE_SPECS, moduleDims, checkModule, meshCheck } from '../studio/cad/cycloid.js';
import { buildOrion6 } from '../studio/cad/orion6.js';
import { staticInterferences, bedIssues } from '../studio/cad/checks.js';
import { makeOrion6Maker } from '../studio/core/defaults.js';

const wasm = await Module({ wasmBinary: fs.readFileSync(new URL('../node_modules/manifold-3d/manifold.wasm', import.meta.url)) });
wasm.setup();
setupKit(wasm);

test('Réducteurs cycloïdaux : cotes valides et engrènement sans interférence', () => {
  for (const [k, spec] of Object.entries(MODULE_SPECS)) {
    assert.deepEqual(checkModule(moduleDims(spec)), [], `module ${k}`);
    const m = meshCheck(spec, 18);
    assert.ok(m.minClearance > -1e-6, `${k} : interférence galet/disque ${m.minClearance}`);
    assert.ok(m.maxGap < 1e-3, `${k} : galet sans contact ${m.maxGap}`);
  }
});

test('Propriétés de masse : cube et cylindre', () => {
  const c = massProps(wasm.Manifold.cube([10, 20, 30], true));
  assert.ok(Math.abs(c.volume - 6000) < 1e-6);
  const Ixx = (6000 * (20 * 20 + 30 * 30)) / 12;
  assert.ok(Math.abs(c.inertia[0] - Ixx) / Ixx < 1e-9);
});

test('Assemblage ORION-6 : sans interférence, tout tient sur un plateau 220×220', () => {
  const asm = buildOrion6(makeOrion6Maker());
  assert.deepEqual(asm.warnings, []);
  assert.ok(asm.parts.filter((p) => p.printed).length >= 30);
  for (const p of asm.parts) assert.equal(p.mesh.status(), 'NoError', p.id);
  assert.deepEqual(bedIssues(asm), []);
  assert.deepEqual(staticInterferences(asm), []);
});

test('Géométrie paramétrique : allonger le bras (a2 = 300 mm) reste valide', () => {
  const p = makeOrion6Maker();
  p.kinematics.joints[1].a = 0.3;
  p.kinematics.joints[3].d = 0.26;
  const asm = buildOrion6(p);
  assert.deepEqual(asm.warnings, []);
  assert.deepEqual(staticInterferences(asm), []);
  const tube = asm.parts.find((q) => q.id === 'tube-upper');
  assert.ok(tube.cutLength > 150);
});
