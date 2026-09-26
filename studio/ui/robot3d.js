// ORION Studio — géométrie 3D du robot.
//   • buildProceduralRobot : modèle générique construit depuis la table DH (tout robot).
//   • Les maillages CAO (pièces imprimables) peuvent remplacer ce modèle (voir attachCadMeshes).
// Chaque segment i est un groupe dont la matrice = repère DH i (monde) ; la géométrie est
// exprimée dans ce repère.

import * as THREE from 'three';
import { capsulesFromDH } from '../core/safety.js';

const up = new THREE.Vector3(0, 1, 0);

/** Cylindre (axe local Y de Three) orienté selon `dir`, centré en `center`. */
function orientedCylinder(radius, length, center, dir, material, radial = 40) {
  const g = new THREE.CylinderGeometry(radius, radius, length, radial, 1);
  const m = new THREE.Mesh(g, material);
  m.position.set(center[0], center[1], center[2]);
  m.quaternion.setFromUnitVectors(up, new THREE.Vector3(dir[0], dir[1], dir[2]).normalize());
  return m;
}

function capsuleBetween(p0, p1, radius, material) {
  const a = new THREE.Vector3(...p0), b = new THREE.Vector3(...p1);
  const len = a.distanceTo(b);
  const g = new THREE.CapsuleGeometry(radius, Math.max(len, 1e-4), 8, 28);
  const m = new THREE.Mesh(g, material);
  m.position.copy(a).add(b).multiplyScalar(0.5);
  if (len > 1e-6) m.quaternion.setFromUnitVectors(up, b.clone().sub(a).normalize());
  return m;
}

/** Axe de l’articulation i (0-indexé) exprimé dans le repère du segment parent et du segment enfant. */
export function jointAxisLocal(kin, i) {
  const j = kin.joints[i];
  const sa = Math.sin(j.alpha), ca = Math.cos(j.alpha);
  if (kin.convention === 'MDH') {
    return {
      parent: { o: [j.a, -sa * j.d, ca * j.d], z: [0, -sa, ca] },
      child: { o: [0, 0, 0], z: [0, 0, 1] },
    };
  }
  return {
    parent: { o: [0, 0, 0], z: [0, 0, 1] },
    child: { o: [-j.a, -sa * j.d, -ca * j.d], z: [0, sa, ca] },
  };
}

export function makeMaterials(dark = false) {
  return {
    shell: new THREE.MeshStandardMaterial({ color: dark ? 0xdfe5ea : 0xf1f4f6, metalness: 0.05, roughness: 0.45 }),
    housing: new THREE.MeshStandardMaterial({ color: 0x2b333b, metalness: 0.35, roughness: 0.42 }),
    accent: new THREE.MeshStandardMaterial({ color: dark ? 0x2bc0b8 : 0x0b8c87, metalness: 0.2, roughness: 0.35 }),
    metal: new THREE.MeshStandardMaterial({ color: 0x9aa4ad, metalness: 0.85, roughness: 0.3 }),
    finger: new THREE.MeshStandardMaterial({ color: 0x3a434c, metalness: 0.2, roughness: 0.55 }),
    pad: new THREE.MeshStandardMaterial({ color: 0xe86a33, metalness: 0.0, roughness: 0.8 }),
  };
}

export function ghostMaterial(color = 0x0b8c87) {
  return new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.18, depthWrite: false });
}

/**
 * Construit le robot procédural.
 * Retourne { root, links[], gripper, meshes[] } — links[0] = base, links[i] = segment i.
 */
export function buildProceduralRobot(kin, params, mats, opts = {}) {
  const n = kin.n;
  const root = new THREE.Group();
  root.name = 'robot';
  const links = [];
  for (let i = 0; i <= n; i++) {
    const g = new THREE.Group();
    g.matrixAutoUpdate = false;
    g.name = `link${i}`;
    root.add(g);
    links.push(g);
  }
  // Échelle : dimension caractéristique du robot
  const reach = kin.joints.reduce((s, j) => s + Math.abs(j.a) + Math.abs(j.d), 0) || 0.5;
  const s = Math.max(0.4, Math.min(3, reach / 0.7));
  const radii = params.safety.linkRadii || [0.052, 0.048, 0.04, 0.034, 0.031, 0.028].map((r) => r * s);
  const meshes = [];
  const tag = (m, link) => { m.userData.link = link; m.castShadow = true; m.receiveShadow = true; meshes.push(m); return m; };

  // Socle
  const d1 = Math.abs(kin.joints[0].d);
  const baseH = Math.max(0.03, Math.min(d1 * 0.55, 0.12 * s));
  const base = new THREE.Mesh(new THREE.CylinderGeometry(radii[0] * 1.35, radii[0] * 1.5, baseH, 48), mats.housing);
  base.rotation.x = Math.PI / 2;
  base.position.z = baseH / 2;
  links[0].add(tag(base, 0));
  const plate = new THREE.Mesh(new THREE.CylinderGeometry(radii[0] * 1.7, radii[0] * 1.7, 0.008, 48), mats.metal);
  plate.rotation.x = Math.PI / 2;
  plate.position.z = 0.004;
  links[0].add(tag(plate, 0));

  // Carters d’articulation (dans le segment parent) + bague de sortie (segment enfant)
  for (let i = 0; i < n; i++) {
    const r = radii[i];
    const ax = jointAxisLocal(kin, i);
    const L = r * 1.45;
    if (kin.joints[i].type === 'prismatic') {
      links[i].add(tag(orientedCylinder(r * 0.6, Math.abs(kin.joints[i].d) + L, ax.parent.o, ax.parent.z, mats.metal), i));
      continue;
    }
    links[i].add(tag(orientedCylinder(r, L, ax.parent.o, ax.parent.z, mats.housing), i));
    const zc = ax.child.z, oc = ax.child.o;
    const ringC = [oc[0] + zc[0] * (L / 2 + 0.003), oc[1] + zc[1] * (L / 2 + 0.003), oc[2] + zc[2] * (L / 2 + 0.003)];
    links[i + 1].add(tag(orientedCylinder(r * 0.92, 0.008, ringC, zc, mats.accent), i + 1));
  }
  // Segments (capsules déduites de la table DH)
  const caps = capsulesFromDH(kin, radii).capsules;
  for (const c of caps) {
    const r = (radii[c.link - 1] ?? radii[radii.length - 1]) * 0.72;
    links[c.link].add(tag(capsuleBetween(c.p0, c.p1, r, mats.shell), c.link));
  }
  // Bride
  const rf = radii[n - 1] * 0.8;
  const fl = orientedCylinder(rf, 0.01, [0, 0, 0.005], [0, 0, 1], mats.metal);
  links[n].add(tag(fl, n));

  const gripper = buildGripper(params, mats, n, tag);
  links[n].add(gripper.group);
  return { root, links, gripper, meshes };
}

/** Pince parallèle simplifiée, cohérente avec le TCP (bouts des doigts centrés sur le TCP). */
export function buildGripper(params, mats, n, tag = (m) => m) {
  const group = new THREE.Group();
  const tool = params.kinematics.tool;
  const g = params.gripper;
  // La pince est construite le long de l’axe z local, orienté de la bride vers le TCP.
  const dir = new THREE.Vector3(tool.x, tool.y, tool.z);
  const tz = dir.length();
  if (tz > 1e-6) group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir.clone().normalize());
  const fingers = { left: null, right: null };
  if (g.type === 'none' || tz < 0.01) {
    return { group, update() {}, tcpZ: tz };
  }
  if (g.type === 'vacuum') {
    group.add(tag(orientedCylinder(0.012, tz - 0.01, [0, 0, (tz - 0.01) / 2], [0, 0, 1], mats.metal), n));
    group.add(tag(orientedCylinder(0.018, 0.01, [0, 0, tz - 0.005], [0, 0, 1], mats.pad), n));
    return { group, update() {}, tcpZ: tz };
  }
  const fl = Math.min(g.fingerLength, tz * 0.8);
  const palmL = Math.max(0.015, tz - fl * 0.8);
  const palmW = Math.max(g.strokeMax + 0.03, 0.05);
  const palm = new THREE.Mesh(new THREE.BoxGeometry(palmW, 0.036, palmL), mats.housing);
  palm.position.z = palmL / 2;
  group.add(tag(palm, n));
  const rail = new THREE.Mesh(new THREE.BoxGeometry(palmW * 0.92, 0.012, 0.008), mats.metal);
  rail.position.z = palmL + 0.004;
  group.add(tag(rail, n));
  for (const side of [-1, 1]) {
    const f = new THREE.Group();
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.01, 0.022, fl), mats.finger);
    body.position.z = fl / 2;
    const pad = new THREE.Mesh(new THREE.BoxGeometry(0.003, 0.02, fl * 0.55), mats.pad);
    pad.position.set(-side * 0.0065, 0, fl * 0.7);
    f.add(tag(body, n), tag(pad, n));
    f.position.z = palmL;
    group.add(f);
    fingers[side < 0 ? 'left' : 'right'] = f;
  }
  return {
    group,
    tcpZ: tz,
    update(opening) {
      const half = Math.max(0, opening) / 2 + 0.0065;
      fingers.left.position.x = -half;
      fingers.right.position.x = half;
    },
  };
}

/** Clone « fantôme » (transparent) d’un robot construit. */
export function makeGhost(robot, material) {
  const root = new THREE.Group();
  const links = robot.links.map((g) => {
    const c = new THREE.Group();
    c.matrixAutoUpdate = false;
    g.traverse((o) => {
      if (o.isMesh && o.parent === g) {
        const m = new THREE.Mesh(o.geometry, material);
        m.position.copy(o.position); m.quaternion.copy(o.quaternion); m.scale.copy(o.scale);
        c.add(m);
      }
    });
    root.add(c);
    return c;
  });
  return { root, links };
}

/** Applique les repères DH (tableaux row-major 4×4) aux groupes de segments. */
export function applyFrames(links, frames) {
  for (let i = 0; i < links.length && i < frames.length; i++) {
    const F = frames[i];
    links[i].matrix.set(F[0], F[1], F[2], F[3], F[4], F[5], F[6], F[7], F[8], F[9], F[10], F[11], 0, 0, 0, 1);
    links[i].matrixWorldNeedsUpdate = true;
  }
}
