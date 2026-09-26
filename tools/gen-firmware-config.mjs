// Génère les en-têtes de configuration des deux firmwares à partir des préréglages ORION Studio :
//   firmware/orion_fw/src/orion_config.h    ← ORION-6 MAKER (pas-à-pas)
//   firmware/mit_bridge/src/orion_config.h  ← ORION-6 PRO (pont CAN mode MIT)
// Usage : node tools/gen-firmware-config.mjs
import { writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { makeOrion6Maker, makeOrion6Pro } from '../studio/core/defaults.js';
import { toFirmwareConfig } from '../studio/core/export.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const targets = [
  ['firmware/orion_fw/src/orion_config.h', makeOrion6Maker()],
  ['firmware/mit_bridge/src/orion_config.h', makeOrion6Pro()],
];
for (const [path, params] of targets) {
  writeFileSync(join(root, path), toFirmwareConfig(params, { stamp: false }));
  console.log(`✓ ${path} (${params.meta.name})`);
}
