// ORION Studio — pont CAO : génère l’assemblage imprimable (manifold-3d, WebAssembly) dans le
// navigateur, l’affiche à la place du modèle simplifié et exporte les STL prêts à imprimer.

import * as THREE from 'three';
import { setupKit, toSTL } from '../cad/kit.js';
import { buildOrion6, printMesh, partMass } from '../cad/orion6.js';
import { MOTOR_LIBRARY } from '../core/defaults.js';
import { Kinematics } from '../core/kinematics.js';
import { mat4InvRigid } from '../core/math3d.js';

const COLORS = {
  printedLight: 0xeef1f3,
  housing: 0x2f3740,
  accent: 0x0b8c87,
  internal: 0xe86a33,
  motor: 0x1d2227,
  bearing: 0xb8c0c8,
  tube: 0xc9d0d6,
  servo: 0x1c2a4a,
  pad: 0xe86a33,
};

function materialFor(part) {
  let color = COLORS.printedLight, metal = 0.05, rough = 0.5;
  if (part.vendor) {
    if (/Moteur/.test(part.name)) { color = COLORS.motor; metal = 0.5; rough = 0.45; }
    else if (/Roulement|Galets|Doigts/.test(part.name)) { color = COLORS.bearing; metal = 0.9; rough = 0.3; }
    else if (/Tube/.test(part.name)) { color = COLORS.tube; metal = 0.8; rough = 0.35; }
    else if (/Servo/.test(part.name)) { color = COLORS.servo; metal = 0.1; rough = 0.6; }
  } else if (/^J\d-(housing|retainer)/.test(part.id)) color = COLORS.housing;
  else if (/^J\d-(clamp|flange)/.test(part.id)) color = COLORS.accent;
  else if (/^J\d-(disc|cam)/.test(part.id)) color = COLORS.internal;
  else if (/pad/.test(part.id)) color = COLORS.pad;
  else if (/pinion/.test(part.id)) color = COLORS.housing;
  return new THREE.MeshStandardMaterial({ color, metalness: metal, roughness: rough, flatShading: true });
}

/** Compatibilité topologique : la CAO ORION-6 ne suit que les longueurs d1, a2, d4. */
function topologySignature(p) {
  const J = p.kinematics.joints;
  if (p.kinematics.convention !== 'DH' || J.length !== 6) return null;
  const r = (v) => Math.round(v * 1e4);
  return J.map((j, i) => [j.type, r(j.alpha), r(j.thetaOffset), i === 0 || i === 1 || i === 3 || i === 5 ? 'L' : r(j.a) + ':' + r(j.d)].join('/')).join('|');
}

export class CadBridge {
  constructor(app) {
    this.app = app;
    this.ready = false;
    this.building = false;
    this.asm = null;
    this.viewData = null;
    this.refSignature = null;
  }

  async init() {
    if (window.ORION_MANIFOLD_FACTORY) this.Module = window.ORION_MANIFOLD_FACTORY;
    else this.Module = (await import('../vendor/manifold/manifold.js')).default;
    if (!window.ORION_MANIFOLD_WASM && !this.wasmBytes) {
      // Charge le binaire une seule fois ; chaque régénération crée une instance neuve
      // (la mémoire WebAssembly de la précédente est libérée avec elle).
      try {
        const url = new URL('../vendor/manifold/manifold.wasm', import.meta.url);
        this.wasmBytes = new Uint8Array(await (await fetch(url)).arrayBuffer());
      } catch { this.wasmBytes = null; }
    }
    await this.freshInstance();
    this.ready = true;
  }

  async freshInstance() {
    const opts = {};
    const bin = window.ORION_MANIFOLD_WASM || this.wasmBytes;
    if (bin) opts.wasmBinary = bin;
    const wasm = await this.Module(opts);
    wasm.setup();
    setupKit(wasm);
  }

  /** La géométrie courante peut-elle être construite avec les pièces ORION-6 ? */
  compatible(params) {
    const sig = topologySignature(params);
    if (!this.refSignature) this.refSignature = topologySignature(this.app.defaultParamsForCad());
    return !!sig && sig === this.refSignature && params.meta.variant !== 'custom';
  }

  async buildFresh(params) {
    if (this.builtOnce) await this.freshInstance();
    this.builtOnce = true;
    return this.build(params);
  }

  build(params) {
    if (!this.ready) return null;
    const t0 = performance.now();
    const asm = buildOrion6(params);
    this.asm = asm;
    // Repères DH en pose zéro (m) → passage monde → repère de segment
    const kin = new Kinematics(params);
    const f0 = kin.fk(new Array(6).fill(0));
    const inv = f0.frames.map((F) => mat4InvRigid(F));
    const links = Array.from({ length: 7 }, () => []);
    for (const part of asm.parts) {
      if (part.link < 0) continue;
      const mesh = part.mesh.getMesh();
      const np = mesh.numProp, vp = mesh.vertProperties;
      const nv = vp.length / np;
      const T = inv[part.link];
      const pos = new Float32Array(nv * 3);
      for (let i = 0; i < nv; i++) {
        const x = vp[i * np] / 1000, y = vp[i * np + 1] / 1000, z = vp[i * np + 2] / 1000;
        pos[i * 3] = T[0] * x + T[1] * y + T[2] * z + T[3];
        pos[i * 3 + 1] = T[4] * x + T[5] * y + T[6] * z + T[7];
        pos[i * 3 + 2] = T[8] * x + T[9] * y + T[10] * z + T[11];
      }
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      g.setIndex(new THREE.BufferAttribute(new Uint32Array(mesh.triVerts), 1));
      g.computeBoundingSphere();
      links[part.link].push({ id: part.id, geometry: g, material: materialFor(part), finger: part.finger || 0 });
    }
    this.viewData = { links, fingerRef: params.gripper.strokeMax };
    this.buildMs = performance.now() - t0;
    return asm;
  }

  /** Liste des pièces imprimables regroupées, avec masses estimées. */
  printedParts() {
    if (!this.asm) return [];
    return this.asm.parts.filter((p) => p.printed).map((p) => {
      const m = partMass(p, MOTOR_LIBRARY);
      const bb = printMesh(p).boundingBox();
      return { part: p, mass: m.mass, volume: m.volumeCm3, size: [bb.max[0] - bb.min[0], bb.max[1] - bb.min[1], bb.max[2] - bb.min[2]] };
    });
  }

  stlOf(part) { return toSTL(printMesh(part), part.id); }
}
