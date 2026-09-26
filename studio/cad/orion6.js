// ORION-6 — Assemblage paramétrique complet (pièces imprimables + composants du commerce).
// Toutes les pièces sont construites en mm dans le repère monde, robot en pose zéro :
//   J1 vertical ; J2/J3 horizontaux (axe y) ; bras supérieur vertical ; avant-bras selon +x ;
//   poignet sphérique au point W = (d4, 0, d1 + a2) ; outil selon +x.
// Les longueurs d1, a2, d4 viennent des paramètres DH ; d6 et la longueur d’outil sont calculées.

import { kit, massProps } from './kit.js';
import { MODULE_SPECS, buildModuleParts, moduleDims, checkModule } from './cycloid.js';

const TUBE = { od: 40, id: 36, density: 2.7 }; // tube aluminium Ø40×2 (6060)
const SOCKET = 36; // profondeur d’emboîtement des tubes
const PLA = 1.24; // g/cm³

// Servo de pince MG996R (cotes nominales, à vérifier sur votre modèle)
const SERVO = { l: 40.7, w: 19.7, h: 36.5, tabL: 54.5, tabT: 2.5, tabZ: 27, holeSpan: 48.7, holeSpanW: 10, hole: 2.2, shaftOffset: 10.2, shaftTop: 42.9, mass: 0.055 };

function pcd(n, r, make, phase = 0) { return kit.polar(n, (a) => kit.onCircle(make(), r, a), phase); }

/** Dent de crémaillère / pignon (développante approximée) — module m, z dents. */
function spurGear(m, z, width, bore) {
  const rp = (m * z) / 2, ra = rp + m, rf = rp - 1.25 * m;
  const rb = rp * Math.cos((20 * Math.PI) / 180);
  const pts = [];
  const inv = (a) => Math.tan(a) - a;
  const toothAngle = Math.PI / z; // demi-pas angulaire
  const phiP = Math.acos(rb / rp);
  const halfThick = toothAngle / 2 + inv(phiP);
  for (let i = 0; i < z; i++) {
    const c = (2 * Math.PI * i) / z;
    const flank = [];
    for (let k = 0; k <= 6; k++) {
      const r = Math.max(rb, rf) + ((ra - Math.max(rb, rf)) * k) / 6;
      const phi = Math.acos(Math.min(1, rb / r));
      flank.push([r, halfThick - inv(phi)]);
    }
    pts.push([rf * Math.cos(c - toothAngle), rf * Math.sin(c - toothAngle)]);
    for (const [r, a] of flank) pts.push([r * Math.cos(c - a), r * Math.sin(c - a)]);
    for (const [r, a] of flank.slice().reverse()) pts.push([r * Math.cos(c + a), r * Math.sin(c + a)]);
  }
  let g = kit.poly(pts, width);
  if (bore) g = g.subtract(kit.cyl(width + 0.2, bore / 2).translate([0, 0, -0.1]));
  return { mesh: g, rp, ra };
}

function rack(m, length, height, width) {
  // Crémaillère : ligne primitive en y = 0, dents vers +y, n pas entiers
  const p = Math.PI * m;
  const n = Math.max(2, Math.floor(length / p));
  const total = n * p, x0 = -total / 2;
  const ha = m, hf = 1.25 * m, t = Math.tan((20 * Math.PI) / 180), hw = p / 4;
  const pts = [[x0, -height], [x0 + total, -height], [x0 + total, -hf]];
  for (let i = n - 1; i >= 0; i--) {
    const c = x0 + (i + 0.5) * p;
    pts.push([c + hw + hf * t, -hf], [c + hw - ha * t, ha], [c - hw + ha * t, ha], [c - hw - hf * t, -hf]);
  }
  pts.push([x0, -hf]);
  return kit.poly(pts, width);
}

/**
 * Construit l’assemblage.
 * @param p paramètres (SI) — utilise kinematics.joints[0].d, [1].a, [3].d
 * @returns { parts[], layout, warnings[], capsules[] }
 */
export function buildOrion6(p, opts = {}) {
  const warnings = [];
  const J = p.kinematics.joints;
  const D1 = J[0].d * 1000, A2 = J[1].a * 1000, D4 = J[3].d * 1000;
  const specs = {
    1: { ...MODULE_SPECS.M, motorLen: 48 },
    2: { ...MODULE_SPECS.L, motorLen: 76 },
    3: { ...MODULE_SPECS.M, motorLen: 48 },
    4: { ...MODULE_SPECS.S, motorLen: 40 },
    5: { ...MODULE_SPECS.S, motorLen: 40 },
    6: { ...MODULE_SPECS.S, motorLen: 34 },
  };
  const dims = {};
  for (const k of Object.keys(specs)) {
    dims[k] = moduleDims(specs[k]);
    for (const w of checkModule(dims[k])) warnings.push(`Module J${k} : ${w}`);
  }
  const zW = D1 + A2; // hauteur du poignet
  const parts = [];
  const add = (part) => { parts.push(part); return part; };
  const moduleParts = {};

  // ------------------------------------------------------------ placement des modules
  // place(k, axis, origin) : repère module (z = axe, z=0 face moteur) → monde
  const placement = {};
  const place = (k, axis, origin, ringPhase = 0) => { placement[k] = { axis, origin, ringPhase }; };
  const world = (k, m) => kit.axisTo(m, placement[k].axis).translate(placement[k].origin);

  // Socle et J1
  const d1m = dims[1];
  const basePlateT = 6;
  const zBT = Math.max(basePlateT + 3 + d1m.ringBack + d1m.motorLen, 80); // face supérieure du socle
  place(1, '+z', [0, 0, zBT - d1m.ringBack], 0);
  const zI1 = zBT - d1m.ringBack + d1m.zI; // interface de J1 (dessous de la tourelle)

  // J3 → J2 (déports latéraux choisis pour que l’avant-bras soit dans le plan y = 0)
  const d2m = dims[2], d3m = dims[3], d4m = dims[4], d5m = dims[5], d6m = dims[6];
  const elbowHalf = d4m.housingR + 6.5; // demi-largeur du bloc coude
  const y3I = elbowHalf; // face +y du coude = interface J3
  const yU = y3I + (d3m.zI - d3m.ringBack); // face intérieure du bras supérieur
  const yI2 = yU; // interface J2
  place(3, '-y', [0, y3I + d3m.zI, zW]);
  place(2, '+y', [0, yI2 - d2m.zI, D1]);
  const yT = yI2 - (d2m.zI - d2m.ringBack); // face +y de la tourelle
  if (D1 - d2m.housingR < zI1 + 8.5) warnings.push(`d1 trop faible : le carter de J2 touche la tourelle (d1 ≥ ${(zI1 + 8.5 + d2m.housingR).toFixed(0)} mm)`);

  // J4 dans le coude, J5 et J6 au poignet
  const xe4 = 60; // face +x du coude (appui de la bague de J4)
  place(4, '+x', [xe4 - d4m.ringBack, 0, zW]);
  const xI4 = xe4 - d4m.ringBack + d4m.zI;
  const xW = D4;
  const a6 = 5; // face moteur de J6 en retrait du centre poignet
  place(6, '+x', [xW - a6, 0, zW]);
  const xFlange = xW - a6 + d6m.zI; // bride outil
  const y5I = d6m.ringR + 1.5 + 8; // interface J5 (face de la chape)
  place(5, '-y', [xW, y5I + d5m.zI, zW]);
  const y5R = y5I + (d5m.zI - d5m.ringBack); // appui bague J5 (plaque du poignet)
  const sweepR = Math.max(Math.hypot(a6 + d6m.motorLen, (d6m.mot.size / 2) * Math.SQRT2), Math.hypot(d6m.zFront + 3 - a6, d6m.ringR)) + 3;
  const xWristSock = xW - sweepR - 2; // fin de la douille poignet (côté +x)
  if (xWristSock - SOCKET < xI4 + 8 + SOCKET * 0.4) warnings.push(`d4 trop faible : l’avant-bras est trop court (d4 ≥ ${(xI4 + 8 + SOCKET * 1.4 + sweepR + 2).toFixed(0)} mm)`);

  // ------------------------------------------------------------ modules (pièces)
  const linkOfRole = (k, role) => (role === 'rotor' ? Number(k) : Number(k) - 1);
  for (const k of Object.keys(specs)) {
    const { parts: mp } = buildModuleParts(specs[k]);
    moduleParts[k] = mp;
    for (const part of mp) {
      add({
        id: `J${k}-${part.key}`,
        group: `Module J${k} (${specs[k].label}, ${dims[k].ratio}:1)`,
        name: part.name,
        link: linkOfRole(k, part.role),
        local: part.mesh,
        mesh: world(k, part.mesh),
        printed: part.printed,
        fill: part.fill,
        qty: part.qty,
        note: part.note,
        vendor: part.vendor,
        massKg: part.key === 'motor' ? null : part.massKg,
        density: part.density,
        moduleKey: part.key,
        moduleSize: specs[k].label,
        motor: part.key === 'motor' ? `${specs[k].motor}-${specs[k].motorLen}` : undefined,
        print: part.print,
      });
    }
  }

  // Motifs d’hôte : alésage du carter + inserts de fixation de la bague
  const hostCut = (k, depth = 9) => {
    const d = dims[k];
    const s = specs[k];
    const bore = kit.cyl(d.ringBack + 1, d.housingR + 0.3).translate([0, 0, -1]);
    // Dégagement du moteur (passe par l’alésage lors du montage)
    const motor = kit.cyl(d.motorLen + 2.5, (d.mot.size / 2) * Math.SQRT2 + 1.5).translate([0, 0, -d.motorLen - 1.5]);
    const inserts = kit.polar(s.ring.n, (a) => kit.onCircle(kit.insert(s.ring.size, depth), d.ringBoltR, a, d.ringBack - depth + 0.01), placement[k].ringPhase);
    return world(k, kit.union([bore, motor, inserts]));
  };
  // Motif d’interface de sortie : trous de passage + centrage
  const ifaceCut = (k, depth) => {
    const d = dims[k], s = specs[k];
    const off = 180 / s.iface.n;
    const holes = kit.polar(s.iface.n, (a) => kit.onCircle(kit.counterbore(s.iface.size, depth, Math.min(4, depth - 3)), d.ifaceR, a, d.zI - 0.01), off);
    return world(k, holes);
  };

  // ------------------------------------------------------------ socle (lien 0)
  {
    const R0 = d1m.ringR + 8;
    let plate = kit.cyl(basePlateT, 100);
    const slots = kit.polar(4, (a) => kit.hull([kit.onCircle(kit.cyl(basePlateT + 1, 3.4), 80, a - 4, -0.5), kit.onCircle(kit.cyl(basePlateT + 1, 3.4), 80, a + 4, -0.5)]), 45);
    const socleHoles = kit.polar(6, (a) => kit.onCircle(kit.counterbore(4, basePlateT + 0.02, 3.5).rotate([180, 0, 0]).translate([0, 0, basePlateT + 0.01]), R0 - 6, a), 0);
    plate = kit.diff(plate, [slots, socleHoles, kit.cyl(basePlateT + 1, 30).translate([-55, 0, -0.5])]);
    add({ id: 'base-plate', group: 'Socle', name: 'Plaque de base', link: 0, mesh: plate, printed: true, fill: 0.5, qty: 1, print: {}, note: 'Fixation à la table par 4 vis M6 (lumières Ø6,8). Ouverture pour le passage des câbles.' });

    const H = zBT - basePlateT;
    let socle = kit.tube(H, R0, R0 - 5).translate([0, 0, basePlateT]);
    const top = kit.cyl(8, R0).translate([0, 0, zBT - 8]);
    const foot = kit.tube(6, R0, R0 - 12).translate([0, 0, basePlateT]);
    socle = kit.union([socle, top, foot]);
    socle = kit.diff(socle, [
      hostCut(1, 8.5),
      kit.polar(6, (a) => kit.onCircle(kit.insert(4, 7), R0 - 6, a, basePlateT - 0.01), 0),
      kit.cyl(20, 14).rotate([0, 90, 0]).translate([-R0 - 5, 0, basePlateT + 22]), // passe-câbles arrière
      kit.cbox(30, 24, 60).translate([-R0 + 2, 0, basePlateT + 10]).intersect(kit.cyl(80, R0 + 1).translate([0, 0, 0])).subtract(kit.cyl(90, R0 - 6).translate([0, 0, 0])),
    ]);
    add({ id: 'socle', group: 'Socle', name: 'Socle (fût)', link: 0, mesh: socle, printed: true, fill: 0.4, qty: 1, print: { rot: [180, 0, 0] }, note: 'Porte le module J1. Inserts M4 sur la face supérieure et le pied.' });
  }

  // ------------------------------------------------------------ tourelle (lien 1)
  // « Tambour » horizontal autour du module J2 (moteur logé dans l’alésage) posé sur un pied.
  {
    const z0 = zI1; // dessous de la tourelle
    const floorT = 8;
    const yMot = yI2 - d2m.zI; // face moteur de J2 (monde)
    const yBack = yMot - d2m.motorLen - 8; // fond du tambour
    const Rd = d2m.housingR + 5.5; // rayon extérieur du tambour
    const Rf = d2m.ringR + 1; // rayon de la collerette avant (inserts)
    const drum = kit.cyl(yT - 10 - yBack, Rd).rotate([-90, 0, 0]).translate([0, yBack, D1]);
    const collar = kit.cyl(10, Rf).rotate([-90, 0, 0]).translate([0, yT - 10, D1]);
    const foot = kit.hull([
      kit.cyl(floorT, Math.min(d1m.clampR + 12, yT - 1)).translate([0, 0, z0]),
      kit.cbox(2 * (Rd - 12), yT - yBack - 14, 1).translate([0, (yT + yBack) / 2 - 3, D1 - Rd + 12]),
    ]);
    let turret = kit.union([drum, collar, foot]);
    turret = kit.diff(turret, [
      hostCut(2, 9),
      // Fond du tambour : passage des câbles du moteur J2
      kit.cyl(20, 12).rotate([-90, 0, 0]).translate([0, yBack - 1, D1 - 10]),
      // Interface J1 : vis montées depuis l’intérieur du tambour (avant le module J2)
      world(1, kit.polar(specs[1].iface.n, (a) => kit.onCircle(kit.union([kit.clearance(specs[1].iface.size, 60), kit.cyl(60, 3.2).translate([0, 0, 8])]), dims[1].ifaceR, a, dims[1].zI - 0.01), 180 / specs[1].iface.n)),
      kit.cyl(floorT + 30, 11).translate([0, 0, z0 - 0.5]), // passage central des câbles vers le socle
    ]);
    turret = turret.intersect(kit.cbox(400, 400, 400).translate([0, 0, z0]));
    add({ id: 'turret', group: 'Tourelle', name: 'Tourelle (tambour d’épaule)', link: 1, mesh: turret, printed: true, fill: 0.45, qty: 1, print: { rot: [-90, 0, 0] }, note: 'Tambour horizontal qui reçoit le module J2 (moteur NEMA23 dans l’alésage). Imprimer collerette sur le plateau (alésage vertical), supports sous le pied.' });
  }

  // ------------------------------------------------------------ bras supérieur (lien 2)
  {
    const yTube = yU + 15;
    const sockOuter = TUBE.od / 2 + 4.5;
    const zSock0 = D1 + d2m.ringR + 5; // bas de la douille inférieure
    const zSock1 = zW - d3m.ringR - 5; // haut de la douille supérieure
    const tubeLen = zSock1 - zSock0 - 4;
    if (tubeLen < 2 * SOCKET + 10) warnings.push(`a2 trop court pour le tube du bras (${tubeLen.toFixed(0)} mm)`);
    const sockLen = Math.min(SOCKET, tubeLen / 2 - 2);
    const clampSlot = (z0, dir) => kit.cbox(1.6, 30, sockLen + 2).translate([0, yTube + TUBE.od / 2, z0 - (dir < 0 ? sockLen + 1 : 1)]);
    const clampBolt = (zc) => kit.union([
      kit.clearance(4, 30).rotate([0, 90, 0]).translate([-15, yTube + TUBE.od / 2 + 3, zc]),
      kit.nutTrap(4, 4).rotate([0, 90, 0]).translate([7, yTube + TUBE.od / 2 + 3, zc]),
    ]);
    // Raccord inférieur (sur J2)
    let low = kit.hull([
      kit.cyl(8, d2m.clampR + 4).rotate([-90, 0, 0]).translate([0, yU, D1]),
      kit.cyl(sockLen, sockOuter).translate([0, yTube, zSock0]),
    ]);
    low = kit.union([low, kit.cbox(12, 12, sockLen).translate([0, yTube + TUBE.od / 2 + 3, zSock0])]);
    low = kit.diff(low, [
      kit.cyl(sockLen + 20, TUBE.od / 2 + 0.2).translate([0, yTube, zSock0 + 2]),
      ifaceCut(2, 8),
      kit.cyl(200, d2m.ringR + 3).rotate([90, 0, 0]).translate([0, yU, D1]), // dégagement de J2 (fixe) côté tourelle
      clampSlot(zSock0 + 2, 1),
      clampBolt(zSock0 + sockLen / 2 + 1),
    ]);
    add({ id: 'upperarm-low', group: 'Bras supérieur', name: 'Raccord bas du bras', link: 2, mesh: low, printed: true, fill: 0.6, qty: 1, print: { rot: [90, 0, 0] }, note: 'Vissé sur la sortie de J2 ; serre le tube Ø40 (vis M4 + écrou).' });
    // Raccord supérieur (porte J3)
    let high = kit.hull([
      kit.cyl(12, d3m.ringR + 5).rotate([-90, 0, 0]).translate([0, yU, zW]),
      kit.cyl(sockLen, sockOuter).translate([0, yTube, zSock1 - sockLen]),
    ]);
    high = kit.union([high, kit.cbox(12, 12, sockLen).translate([0, yTube + TUBE.od / 2 + 3, zSock1 - sockLen])]);
    high = kit.diff(high, [
      kit.cyl(sockLen + 20, TUBE.od / 2 + 0.2).translate([0, yTube, zSock1 - sockLen - 22]),
      hostCut(3, 10),
      kit.cyl(200, d3m.ringR + 3).rotate([90, 0, 0]).translate([0, yU, zW]), // dégagement de la sortie de J3 (coude)
      clampSlot(zSock1 - 2, -1),
      clampBolt(zSock1 - sockLen / 2 - 1),
    ]);
    add({ id: 'upperarm-high', group: 'Bras supérieur', name: 'Raccord haut du bras (porte J3)', link: 2, mesh: high, printed: true, fill: 0.6, qty: 1, print: { rot: [90, 0, 0] }, note: 'Reçoit le module J3 (moteur côté extérieur).' });
    add({ id: 'tube-upper', group: 'Bras supérieur', name: `Tube alu Ø${TUBE.od}×2 — ${Math.round(tubeLen)} mm`, link: 2, mesh: kit.tube(tubeLen, TUBE.od / 2, TUBE.id / 2).translate([0, yTube, zSock0 + 2]), printed: false, vendor: true, density: TUBE.density, qty: 1, cutLength: Math.round(tubeLen) });
  }

  // ------------------------------------------------------------ coude (lien 3)
  {
    const xBack = xe4 - d4m.ringBack - d4m.motorLen - 6;
    const half = elbowHalf;
    let elbow = kit.hull([
      kit.cyl(8, d3m.clampR + 5).rotate([90, 0, 0]).translate([0, y3I, zW]),
      kit.roundRect(2 * half, 2 * half, xe4 - xBack, 10).rotate([0, 90, 0]).translate([xBack, 0, zW]),
    ]);
    elbow = kit.diff(elbow, [
      hostCut(4, 9),
      kit.roundRect(d4m.mot.size + 3, d4m.mot.size + 3, xe4 - xBack - 4, 4).rotate([0, 90, 0]).translate([xBack + 4, 0, zW]),
      ifaceCut(3, 8),
      kit.cyl(20, 9).rotate([0, 90, 0]).translate([xBack - 5, 0, zW]), // passe-câbles
      kit.cbox(40, 2 * half + 2, 30).translate([xBack + 25, 0, zW + half - 8]).subtract(kit.cbox(400, 400, 400).translate([0, 0, zW + half - 4 - 200 - 0.01]).translate([0, 0, 0])),
    ]);
    add({ id: 'elbow', group: 'Coude', name: 'Bloc coude (porte J4)', link: 3, mesh: elbow, printed: true, fill: 0.45, qty: 1, print: { rot: [-90, 0, 0] }, note: 'Vissé sur la sortie de J3 ; loge le moteur de J4.' });
  }

  // ------------------------------------------------------------ avant-bras (lien 4)
  {
    const sockOuter = TUBE.od / 2 + 4.5;
    const xS0 = xI4 + 8; // début douille adaptateur
    let adapter = kit.union([
      kit.cyl(8, d4m.clampR + 3).rotate([0, 90, 0]).translate([xI4, 0, zW]),
      kit.cyl(SOCKET * 0.6, sockOuter).rotate([0, 90, 0]).translate([xS0, 0, zW]),
    ]);
    adapter = kit.diff(adapter, [
      kit.cyl(SOCKET, TUBE.od / 2 + 0.2).rotate([0, 90, 0]).translate([xS0 + 1, 0, zW]),
      ifaceCut(4, 8),
      kit.clearance(3, 60).translate([xS0 + SOCKET * 0.35, 0, zW - 30]),
    ]);
    add({ id: 'forearm-adapter', group: 'Avant-bras', name: 'Adaptateur de tube (sortie J4)', link: 4, mesh: adapter, printed: true, fill: 0.6, qty: 1, print: { rot: [0, -90, 0] }, note: 'Tube collé (époxy) + vis M3 traversante.' });
    const xT0 = xS0 + 1, xT1 = xWristSock - 1;
    const tubeLen = xT1 - xT0;
    add({ id: 'tube-forearm', group: 'Avant-bras', name: `Tube alu Ø${TUBE.od}×2 — ${Math.round(tubeLen)} mm`, link: 4, mesh: kit.tube(tubeLen, TUBE.od / 2, TUBE.id / 2).rotate([0, 90, 0]).translate([xT0, 0, zW]), printed: false, vendor: true, density: TUBE.density, qty: 1, cutLength: Math.round(tubeLen) });
    // Poignet : douille + bras latéral + plaque porte-J5
    const plateT = 10;
    const plateR = d5m.ringR + 5;
    const sockX0 = xWristSock - SOCKET * 0.8;
    let wrist = kit.union([
      kit.cyl(SOCKET * 0.8, sockOuter).rotate([0, 90, 0]).translate([sockX0, 0, zW]),
      kit.hull([
        kit.cbox(SOCKET * 0.8, 2, 2 * sockOuter).translate([sockX0 + SOCKET * 0.4, sockOuter * 0.2, zW - sockOuter]),
        kit.cbox(SOCKET * 0.8, plateT, 2 * sockOuter).translate([sockX0 + SOCKET * 0.4, y5R + plateT / 2, zW - sockOuter]),
      ]),
      kit.hull([
        kit.cbox(SOCKET * 0.8, plateT, 2 * sockOuter).translate([sockX0 + SOCKET * 0.4, y5R + plateT / 2, zW - sockOuter]),
        kit.cyl(plateT, plateR).rotate([-90, 0, 0]).translate([xW, y5R, zW]),
      ]),
    ]);
    wrist = kit.diff(wrist, [
      kit.cyl(SOCKET, TUBE.od / 2 + 0.2).rotate([0, 90, 0]).translate([sockX0 - 1, 0, zW]),
      hostCut(5, 9),
      kit.clearance(3, 60).translate([sockX0 + SOCKET * 0.4, 0, zW - 30]),
    ]);
    add({ id: 'wrist', group: 'Avant-bras', name: 'Poignet (porte J5)', link: 4, mesh: wrist, printed: true, fill: 0.55, qty: 1, print: { rot: [90, 0, 0] }, note: 'Reçoit le module J5 ; le moteur de J5 dépasse côté +y.' });
  }

  // ------------------------------------------------------------ chape (lien 5)
  {
    const d6r = d6m;
    const xPlate1 = xW - a6 + d6r.ringBack; // appui de la bague J6
    const plateT = 8;
    const half = d6r.ringR + 1;
    let yoke = kit.union([
      // plaque J6 (plan y-z)
      kit.hull([
        kit.cyl(plateT, half).rotate([0, 90, 0]).translate([xPlate1 - plateT, 0, zW]),
        kit.cbox(plateT, 2, 2 * d5m.clampR + 6).translate([xPlate1 - plateT / 2, y5I - 1, zW - d5m.clampR - 3]),
      ]),
      // plaque J5 (plan x-z)
      kit.hull([
        kit.cyl(plateT, d5m.clampR + 4).rotate([-90, 0, 0]).translate([xW, y5I - plateT, zW]),
        kit.cbox(plateT, plateT, 2 * d5m.clampR + 6).translate([xPlate1 - plateT / 2, y5I - plateT / 2, zW - d5m.clampR - 3]),
      ]),
    ]);
    yoke = kit.diff(yoke, [
      hostCut(6, 8),
      world(5, kit.polar(specs[5].iface.n, (a) => kit.onCircle(kit.counterbore(specs[5].iface.size, plateT, 4), dims[5].ifaceR, a, dims[5].zI - 0.01), 180 / specs[5].iface.n)),
    ]);
    add({ id: 'yoke', group: 'Poignet', name: 'Chape (J5 → J6)', link: 5, mesh: yoke, printed: true, fill: 0.7, qty: 1, print: { rot: [0, 90, 0] }, note: 'Relie la sortie de J5 au carter de J6. Imprimer à plat, remplissage élevé.' });
  }

  // ------------------------------------------------------------ pince (lien 6) — repère outil puis monde
  const gripper = buildGripper(p, dims[6], specs[6]);
  // Repère outil (z = axe outil) → monde : z_outil = +x monde, x_outil (sens d’ouverture) = +z monde
  const toolToWorld = (m) => m.rotate([0, 0, 0]).rotate([0, 90, 0]).rotate([180, 0, 0]).rotate([0, 0, 0]).translate([xFlange, 0, zW]);
  for (const g of gripper.parts) add({ ...g, link: 6, local: g.mesh, mesh: toolToWorld(g.mesh), toolMesh: g.mesh });

  // ------------------------------------------------------------ pièces de calibrage
  add({ id: 'tolerance-test', group: 'Calibrage', name: 'Test de tolérances', link: -1, mesh: toleranceTest(), printed: true, fill: 1.0, qty: 1, print: {}, note: 'À imprimer en premier : alésages de roulements (6802, 6810, 688), goupilles Ø3/4/5, inserts M3/M4, jeu d’emboîtement du tube.' });

  const layout = {
    d1: D1, a2: A2, d4: D4, d6: xFlange - xW, tcp: gripper.tcp, zBT, zI1, yU, y3I, yT, y5I, xe4, xW, xFlange, sweepR,
  };
  return { parts, layout, warnings, dims, specs, placement };
}

/**
 * Pince parallèle à pignon et crémaillères (servo MG996R), dans le repère outil (= repère DH 6) :
 * z = axe outil, x = sens d’ouverture, y = épaisseur. Les deux crémaillères engrènent de part et
 * d’autre du pignon (y = ±rp) et se déplacent en sens opposés.
 */
function buildGripper(p, d6, s6) {
  const g = p.gripper;
  const parts = [];
  const mGear = 1, zGear = 20;
  const gear = spurGear(mGear, zGear, 8, 0);
  const rp = gear.rp; // 10 mm
  const stroke = Math.max(10, g.strokeMax * 1000);
  const open = stroke; // modèle dessiné pince ouverte
  const fingerL = Math.max(20, g.fingerLength * 1000);
  const baseT = 6;
  const sx = SERVO.l / 2 - SERVO.shaftOffset; // centre du servo par rapport à l’arbre (selon x)
  const zDeck = baseT + SERVO.h + 1; // dessus du servo
  const deckT = 4;
  const zRack = baseT + SERVO.shaftTop + 2.5; // plan des crémaillères (sur le palonnier)
  const rackH = 8, backT = 6, fingerT = 8, gap = 0.35;
  const bodyW = Math.max(SERVO.tabL + 8, open + 2 * fingerT + 6);
  const bodyD = 2 * (rp + backT + 4.5);
  // Embase (motif d’interface CY-S) + caisson du servo
  let base = kit.union([kit.cyl(baseT, d6.clampR), kit.cbox(bodyW, bodyD, baseT)]);
  base = kit.diff(base, [kit.polar(s6.iface.n, (a) => kit.onCircle(kit.counterbore(s6.iface.size, baseT + 0.02, 3.2).rotate([180, 0, 0]).translate([0, 0, baseT + 0.01]), d6.ifaceR, a), 180 / s6.iface.n)]);
  let box = kit.cbox(bodyW, bodyD, zDeck + deckT - baseT).translate([0, 0, baseT]);
  box = kit.diff(box, [
    kit.cbox(SERVO.l + 0.6, SERVO.w + 0.6, zDeck - baseT + 0.02).translate([-sx, 0, baseT - 0.01]),
    kit.cbox(SERVO.tabL + 1, SERVO.w + 0.6, 3).translate([-sx, 0, baseT + SERVO.tabZ - 0.5]),
    kit.cyl(deckT + 1, 13).translate([0, 0, zDeck - 0.5]), // passage du palonnier
    kit.cbox(14, bodyD + 2, 10).translate([-sx - SERVO.l / 2 - 4, 0, baseT + 4]), // sortie du câble
    ...[-1, 1].flatMap((sg) => [-1, 1].map((sw) => kit.clearance(2, 14).translate([-sx + sg * SERVO.holeSpan / 2, sw * SERVO.holeSpanW / 2, baseT + SERVO.tabZ - 11]))),
  ]);
  // Glissières : joue extérieure + lèvre de retenue au-dessus de chaque crémaillère
  const rails = [];
  for (const sgn of [1, -1]) {
    const yOut = sgn * (rp + backT + 0.2 + gap + 1.5);
    rails.push(kit.cbox(bodyW, 3, rackH + 3.5).translate([0, yOut + sgn * 0.0, zDeck + deckT]));
    rails.push(kit.cbox(bodyW, 4.5, 1.6).translate([0, sgn * (rp + backT + 0.2 + gap - 1.2), zDeck + deckT + rackH + 1.9]));
  }
  let body = kit.union([base, box, ...rails]);
  parts.push({ id: 'gripper-body', group: 'Pince', name: 'Corps de pince', mesh: body, printed: true, fill: 0.5, qty: 1, print: {}, note: 'Vissé sur la bride J6 (6 × M3). Servo MG996R glissé par le dessous, fixé par 4 vis M2.' });
  // Pignon sur le palonnier rond
  const pinion = kit.diff(gear.mesh.translate([0, 0, zRack]), [
    kit.cyl(10, 3.2).translate([0, 0, zRack - 1]),
    kit.polar(4, (a) => kit.onCircle(kit.clearance(2, 10), 7, a, zRack - 1), 45),
  ]);
  parts.push({ id: 'gripper-pinion', group: 'Pince', name: `Pignon m${mGear} z${zGear}`, mesh: pinion, printed: true, fill: 1.0, qty: 1, print: {}, note: 'Vissé sur le palonnier rond du servo (4 vis M2). PETG 100 %.' });
  // Crémaillères + doigts. Côté A (y > 0) : doigt en x < 0 ; côté B (y < 0) : doigt en x > 0.
  const rackLen = open / 2 + fingerT + 2 * rp + 8;
  const zR = zDeck + deckT + 0.3;
  for (const [sgn, side] of [[1, -1], [-1, 1]]) {
    const xFingerIn = side * (open / 2); // face intérieure du doigt
    const xFingerOut = side * (open / 2 + fingerT);
    // crémaillère : ligne primitive tangente au pignon en y = sgn·rp
    let r = rack(mGear, rackLen, backT, rackH);
    if (sgn > 0) r = r.mirror([0, 1, 0]);
    r = r.translate([0, sgn * (rp + gap), zR]);
    // placée pour relier le doigt au pignon, en phase avec la denture (creux centré en x = 0,
    // car une dent du pignon est centrée sur ±y quand z est multiple de 4)
    const rackX0 = side < 0 ? xFingerOut : xFingerOut - rackLen;
    const pitch = Math.PI * mGear, total = Math.max(2, Math.floor(rackLen / pitch)) * pitch;
    const tx = rackX0 + rackLen / 2;
    const phase = ((tx - total / 2) % pitch + pitch) % pitch;
    r = r.translate([tx - phase, 0, 0]);
    const yIn = sgn > 0 ? -8 : -(rp + backT + gap + 0.2);
    const yOut = sgn > 0 ? rp + backT + gap + 0.2 : 8;
    // Partie basse (au niveau de la crémaillère) puis partie haute rétrécie pour passer sous la lèvre
    const yLip = rp + backT + gap - 3.7; // bord intérieur de la lèvre de retenue
    const yIn2 = sgn > 0 ? yIn : -yLip + 0.6, yOut2 = sgn > 0 ? yLip - 0.6 : yOut;
    const finger = kit.union([
      kit.cbox(fingerT, yOut - yIn, rackH).translate([(xFingerIn + xFingerOut) / 2, (yIn + yOut) / 2, zR]),
      kit.cbox(fingerT, yOut2 - yIn2, fingerL + 6).translate([(xFingerIn + xFingerOut) / 2, (yIn2 + yOut2) / 2, zR + rackH]),
    ]);
    const f = kit.union([r, finger]);
    const pad = kit.cbox(2, 16, fingerL * 0.6).translate([xFingerIn - side * 1, 0, zR + rackH + 6 + fingerL * 0.4]);
    const name = side < 0 ? 'gauche' : 'droit';
    parts.push({ id: `gripper-finger-${side < 0 ? 'left' : 'right'}`, group: 'Pince', name: `Doigt ${name} (crémaillère m1)`, mesh: f, printed: true, fill: 0.8, qty: 1, finger: side, print: { rot: [90, 0, 0] }, note: 'Crémaillère intégrée. Imprimer à plat, 100 % sur la denture conseillé.' });
    parts.push({ id: `gripper-pad-${side < 0 ? 'left' : 'right'}`, group: 'Pince', name: `Patin ${name} (TPU)`, mesh: pad, printed: true, fill: 1.0, qty: 1, finger: side, material: 'TPU', print: { rot: [0, 90, 0] }, note: 'TPU 95A, collé sur le doigt (adhérence).' });
  }
  parts.push({ id: 'gripper-servo', group: 'Pince', name: 'Servomoteur MG996R', mesh: kit.cbox(SERVO.l, SERVO.w, SERVO.h).translate([-sx, 0, baseT]), printed: false, vendor: true, massKg: SERVO.mass, qty: 1 });
  const tcp = zR + rackH + 6 + fingerL * 0.7; // centre des patins
  return { parts, tcp };
}

/** Pièce de calibrage des tolérances. */
function toleranceTest() {
  let plate = kit.roundRect(110, 42, 6, 4);
  const holes = [
    kit.cyl(8, 24 / 2 + 0.05).translate([-38, 0, -1]), // 6802
    kit.cyl(8, 16 / 2 + 0.05).translate([-15, 0, -1]), // 688
    kit.cyl(8, 3 / 2 - 0.025).translate([2, 10, -1]), kit.cyl(8, 4 / 2 - 0.025).translate([10, 10, -1]), kit.cyl(8, 5 / 2 - 0.025).translate([19, 10, -1]),
    kit.cyl(8, 3 / 2 + 0.1).translate([2, -10, -1]), kit.cyl(8, 4 / 2 + 0.1).translate([10, -10, -1]), kit.cyl(8, 5 / 2 + 0.1).translate([19, -10, -1]),
    kit.insert(3, 5).translate([30, 10, 1.01]), kit.insert(4, 5).translate([30, -10, 1.01]),
    kit.cyl(8, 20.2).translate([80, 0, -1]).intersect(kit.cbox(30, 60, 20).translate([60, 0, -2])),
  ];
  plate = kit.diff(plate, holes);
  return plate;
}

/** Propriétés de masse d’une pièce (kg, m, kg·m²) dans le repère monde (pose zéro). */
export function partMass(part, motorLib) {
  const mp = massProps(part.mesh);
  let massKg;
  if (part.massKg) massKg = part.massKg;
  else if (part.motor && motorLib) massKg = motorLib[part.motor]?.mass ?? 0.3;
  else if (part.density) massKg = (mp.volume / 1000) * part.density / 1000;
  else massKg = (mp.volume / 1000) * PLA * (part.fill ?? 0.5) / 1000;
  const rho = mp.volume > 0 ? massKg / (mp.volume * 1e-9) : 0; // kg/m³ équivalent
  return {
    mass: massKg,
    com: mp.com.map((v) => v / 1000),
    // inertie : mm⁵ × ρ(kg/m³) × 1e-15 → kg·m²
    inertia: mp.inertia.map((v) => v * rho * 1e-15),
    volumeCm3: mp.volume / 1000,
  };
}

/** Maillage orienté pour l’impression (posé sur z = 0, centré en x/y). */
export function printMesh(part) {
  let m = part.local ?? part.mesh;
  if (part.print?.flip) m = m.rotate([180, 0, 0]);
  if (part.print?.rot) m = m.rotate(part.print.rot);
  const bb = m.boundingBox();
  return m.translate([-(bb.min[0] + bb.max[0]) / 2, -(bb.min[1] + bb.max[1]) / 2, -bb.min[2]]);
}
