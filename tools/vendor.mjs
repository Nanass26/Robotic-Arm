// Copie les bibliothèques tierces nécessaires au logiciel dans studio/vendor (fonctionnement hors ligne).
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { transformSync } from 'esbuild';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const nm = path.join(root, 'node_modules');
const out = path.join(root, 'studio', 'vendor');
const files = [
  ['three/build/three.module.js', 'three/three.module.js'],
  ['three/build/three.core.js', 'three/three.core.js'],
  ['three/examples/jsm/controls/OrbitControls.js', 'three/addons/controls/OrbitControls.js'],
  ['three/examples/jsm/controls/TransformControls.js', 'three/addons/controls/TransformControls.js'],
  ['three/examples/jsm/environments/RoomEnvironment.js', 'three/addons/environments/RoomEnvironment.js'],
  ['three/LICENSE', 'three/LICENSE'],
  ['manifold-3d/manifold.js', 'manifold/manifold.js'],
  ['manifold-3d/manifold.wasm', 'manifold/manifold.wasm'],
  ['manifold-3d/LICENSE', 'manifold/LICENSE'],
];
for (const [src, dst] of files) {
  const s = path.join(nm, src), d = path.join(out, dst);
  if (!fs.existsSync(s)) { console.error(`Introuvable : ${src} (lancez « npm install »)`); process.exit(1); }
  fs.mkdirSync(path.dirname(d), { recursive: true });
  if (d.endsWith('.js') && src.startsWith('three/')) {
    // Minification (les modules ES et leurs imports relatifs sont conservés)
    const code = fs.readFileSync(s, 'utf8');
    const res = transformSync(code, { minify: true, format: 'esm', target: 'es2020', legalComments: 'inline' });
    fs.writeFileSync(d, res.code);
  } else {
    fs.copyFileSync(s, d);
  }
  console.log(`vendor/${dst}  ${(fs.statSync(d).size / 1024).toFixed(0)} Ko`);
}
