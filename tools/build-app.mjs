// Construit ORION Studio en UN SEUL fichier HTML autonome : dist/ORION-Studio.html
// (JavaScript regroupé et minifié, feuille de style et moteur de CAO WebAssembly intégrés).
// Il s’ouvre par double-clic, sans serveur ni connexion (les polices Google sont facultatives :
// le logiciel retombe sur les polices système hors ligne).
//
// Usage : node tools/build-app.mjs   (ou npm run build:app)
import { build } from 'esbuild';
import { readFileSync, writeFileSync, mkdirSync, statSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const studio = join(root, 'studio');
const vendor = join(studio, 'vendor');

// Résolution des imports « three » et « three/addons/… » (carte d’import de index.html)
const threePlugin = {
  name: 'three-vendor',
  setup(b) {
    b.onResolve({ filter: /^three$/ }, () => ({ path: join(vendor, 'three', 'three.module.js') }));
    b.onResolve({ filter: /^three\/addons\// }, (args) => ({ path: join(vendor, 'three', 'addons', args.path.slice('three/addons/'.length)) }));
  },
};

const t0 = Date.now();
const res = await build({
  entryPoints: [join(studio, 'main.js')],
  bundle: true,
  format: 'esm',
  platform: 'browser',
  target: 'es2020',
  minify: true,
  write: false,
  legalComments: 'eof',
  external: ['node:*'], // branches Node.js du chargeur Emscripten (jamais exécutées dans le navigateur)
  plugins: [threePlugin],
  logLevel: 'warning',
});
// « </script » ne doit pas apparaître dans un script en ligne
const js = res.outputFiles[0].text.replace(/<\/script/gi, '<\\/script');
const css = readFileSync(join(studio, 'css', 'studio.css'), 'utf8');
const wasm = readFileSync(join(vendor, 'manifold', 'manifold.wasm')).toString('base64');
const html = readFileSync(join(studio, 'index.html'), 'utf8');

const head = html
  .replace(/<link rel="stylesheet" href="css\/studio\.css">/, () => `<style>\n${css}\n</style>`)
  .replace(/<script type="importmap">[\s\S]*?<\/script>/, '')
  .replace(/<script type="module" src="main\.js"><\/script>/, () => [
    '<script>',
    '// Moteur de CAO (manifold-3d, WebAssembly) intégré en base64',
    `window.ORION_MANIFOLD_WASM = Uint8Array.from(atob("${wasm}"), (c) => c.charCodeAt(0));`,
    '</script>',
    `<script type="module">\n${js}\n</script>`,
  ].join('\n'));

if (head.includes('src="main.js"') || !head.includes('ORION_MANIFOLD_WASM')) {
  console.error('Échec : index.html n’a pas la structure attendue.');
  process.exit(1);
}
mkdirSync(join(root, 'dist'), { recursive: true });
const out = join(root, 'dist', 'ORION-Studio.html');
writeFileSync(out, head);
console.log(`✓ dist/ORION-Studio.html — ${(statSync(out).size / 1024 / 1024).toFixed(2)} Mo en ${Date.now() - t0} ms`);
