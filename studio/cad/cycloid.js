// ORION-6 — Module d’articulation à réducteur cycloïdal imprimé (tailles S, M, L).
//
// Repère du module : axe = +z (sortie vers +z), face d’appui du moteur en z = 0,
// moteur côté z < 0. Empilement axial :
//   fond du carter (galets goupilles emmanchés) → came excentrique + 2 disques (déphasés de 180°)
//   → épaulement du flasque → roulement principal → couvercle de retenue → bague de sortie.
// Les galets (goupilles acier) sont logés à moitié dans des gorges du carter sur toute leur
// longueur ; les doigts de sortie (goupilles) sont emmanchés dans le flasque et traversent
// les disques.

import { kit, cycloidPoints, polygonIsSimple } from './kit.js';

// Roulements : [d, D, B] (mm) et diamètre extérieur approximatif de la bague intérieure
export const BEARINGS = {
  688: { d: 8, D: 16, B: 5, innerOD: 10.5, mass: 0.004 },
  6802: { d: 15, D: 24, B: 5, innerOD: 17.5, mass: 0.008 },
  6803: { d: 17, D: 26, B: 5, innerOD: 19.5, mass: 0.009 },
  6804: { d: 20, D: 32, B: 7, innerOD: 23.5, mass: 0.019 },
  6810: { d: 50, D: 65, B: 7, innerOD: 54.5, mass: 0.052 },
  6811: { d: 55, D: 72, B: 9, innerOD: 60.5, mass: 0.083 },
  6814: { d: 70, D: 90, B: 10, innerOD: 76, mass: 0.125 },
};

export const MOTORS = {
  NEMA17: { size: 42.3, bolt: 31, boltSize: 3, pilotD: 22, pilotH: 2, shaft: 5, shaftLen: 24, chamfer: 4, threaded: true },
  NEMA23: { size: 57.2, bolt: 47.14, boltSize: 5, pilotD: 38.1, pilotH: 1.6, shaft: 8, shaftLen: 21, chamfer: 5, threaded: false },
};

/** Spécifications des trois tailles de module. */
export const MODULE_SPECS = {
  S: { label: 'CY-S', motor: 'NEMA17', R: 28, N: 21, dp: 4, pinLen: 20, E: 0.8, t: 6, camBrg: 6802, nOut: 6, dOut: 3, outPinLen: 20, Ro: 18.4, main: 6810, tf: 8, ring: { n: 6, size: 3 }, iface: { n: 6, size: 3 } },
  M: { label: 'CY-M', motor: 'NEMA17', R: 32, N: 26, dp: 4, pinLen: 20, E: 0.9, t: 7, camBrg: 6803, nOut: 6, dOut: 4, outPinLen: 24, Ro: 20.8, main: 6811, tf: 7, ring: { n: 6, size: 4 }, iface: { n: 6, size: 3 } },
  L: { label: 'CY-L', motor: 'NEMA23', R: 40, N: 31, dp: 5, pinLen: 24, E: 1.0, t: 8, camBrg: 6804, nOut: 8, dOut: 5, outPinLen: 24, Ro: 26, main: 6814, tf: 9, ring: { n: 6, size: 4 }, iface: { n: 6, size: 4 } },
};

/** Tolérances d’impression (à calibrer avec la pièce « test de tolérances »). */
export const DEFAULT_TOL = {
  bearingSeat: 0.10, // ajout au diamètre des logements de roulement (serrage léger après retrait du plastique)
  pinPress: -0.05, // goupille emmanchée (trou plus petit)
  pinSlide: 0.20, // jeu diamétral goupille libre / gorge
  shaftPress: 0.0, // alésage de came sur l’arbre moteur (méplat)
  gap: 0.5, // jeu axial entre pièces tournantes
};

/** Cotes dérivées du module (toutes en mm, repère du module). */
export function moduleDims(spec, tol = DEFAULT_TOL) {
  const mb = BEARINGS[spec.main], cb = BEARINGS[spec.camBrg], mot = MOTORS[spec.motor];
  const d = { spec, mb, cb, mot, tol };
  d.ratio = spec.N - 1;
  d.tf = spec.tf;
  d.zA = spec.tf + 0.6; // début disque A
  d.zB = d.zA + spec.t + tol.gap; // début disque B
  d.zD = d.zB + spec.t; // fin des disques
  d.zGroove = d.zD + 0.3; // fin des gorges de galets
  d.zF0 = d.zD + 0.6; // face arrière du flasque de sortie
  d.zB0 = d.zD + 2.6; // début du roulement principal
  d.zFront = d.zB0 + mb.B; // face avant du carter
  d.zOut = d.zFront + 3 + 1.5; // face avant du flasque (dépasse du couvercle)
  d.zI = d.zOut + 3; // face d’interface (bague de sortie)
  d.housingR = mb.D / 2 + 4;
  d.ringT = 5;
  d.ringW = spec.ring.size >= 4 ? 10 : 9;
  d.ringR = d.housingR + d.ringW;
  d.ringBoltR = d.housingR + d.ringW / 2;
  d.ringBack = d.zFront - d.ringT;
  d.ledgeR = mb.D / 2 - 1.5;
  d.retInnerR = mb.D / 2 - 1.5;
  d.flangeR = mb.d / 2; // portée du roulement sur le flasque
  d.shoulderR = mb.d / 2 + 1.5;
  d.clampR = Math.min(mb.innerOD / 2 + 1, d.retInnerR - 0.8);
  d.ifaceR = spec.iface.size >= 4 ? mb.d / 2 - 7 : mb.d / 2 - 6;
  d.hubR = cb.d / 2 - spec.E - 0.2; // moyeu de came (passe dans le roulement de came)
  d.rr = spec.dp / 2; // rayon des galets
  d.outHole = spec.dOut + 2 * spec.E + 0.4; // trous de sortie des disques
  d.pinHole = spec.pinLen - (d.zGroove - spec.tf); // profondeur des trous de galets dans le fond
  d.outEmbed = spec.outPinLen - (d.zF0 - spec.tf - 0.3); // enfoncement des doigts dans le flasque
  d.motorLen = spec.motorLen || (spec.motor === 'NEMA23' ? 76 : 40);
  return d;
}

/** Contrôles géométriques : épaisseurs minimales, cohérence des cotes. */
export function checkModule(d) {
  const s = d.spec, issues = [];
  const need = (cond, msg) => { if (!cond) issues.push(msg); };
  const root = s.R - s.E - d.rr; // rayon de fond des lobes
  need(s.E * s.N / s.R < 0.85, `coefficient de raccourcissement ${(s.E * s.N / s.R).toFixed(2)} ≥ 0,85 (risque de point de rebroussement)`);
  need((2 * Math.PI * s.R) / s.N - s.dp > 2.5, 'galets trop serrés');
  need(s.Ro - d.outHole / 2 - d.cb.D / 2 >= 2.4, `paroi disque côté roulement de came : ${(s.Ro - d.outHole / 2 - d.cb.D / 2).toFixed(2)} mm`);
  need(root - (s.Ro + d.outHole / 2) >= 2.4, `paroi disque côté lobes : ${(root - (s.Ro + d.outHole / 2)).toFixed(2)} mm`);
  need(d.hubR - d.mot.shaft / 2 >= 2.4, `paroi du moyeu de came : ${(d.hubR - d.mot.shaft / 2).toFixed(2)} mm`);
  need(d.ledgeR >= s.R + d.rr + 0.2, 'le gradin du roulement bloque l’insertion des galets');
  need(d.shoulderR < s.R - d.rr - 0.4 || d.zF0 > d.zGroove, 'épaulement du flasque en conflit avec les galets');
  need(s.Ro + s.dOut / 2 + 2 <= d.flangeR, 'doigts de sortie hors du flasque');
  need(d.pinHole >= 3 && d.pinHole <= s.tf - 1, `profondeur des trous de galets ${d.pinHole.toFixed(1)} mm incompatible avec le fond (${s.tf} mm)`);
  need(d.outEmbed >= 4, `doigts de sortie trop peu enfoncés (${d.outEmbed.toFixed(1)} mm)`);
  need(d.ifaceR + 2.8 + 1.2 <= d.flangeR, 'inserts d’interface trop près du bord du flasque');
  need(d.clampR > d.flangeR + 1.5, 'bague de sortie trop étroite');
  const pts = cycloidPoints(s.R, d.rr, s.E, s.N, 900);
  need(polygonIsSimple(pts, 3), 'profil cycloïdal auto-sécant');
  return issues;
}

/**
 * Vérification cinématique 2D de l’engrènement : pour plusieurs angles de came, distance minimale
 * entre chaque galet et le profil des deux disques (doit être ≈ 0 au contact, jamais négative).
 * Retourne { phaseB (rad), minClearance (mm), maxGap (mm) }.
 */
export function meshCheck(spec, steps = 36) {
  const rr = spec.dp / 2, N = spec.N, E = spec.E, R = spec.R;
  const prof = cycloidPoints(R, rr, E, N, 2400);
  const dist = (px, py, cx, cy, th) => {
    // distance signée approximative du point (galet) au contour du disque (centre c, rotation th)
    const c = Math.cos(-th), s = Math.sin(-th);
    const lx = (px - cx) * c - (py - cy) * s, ly = (px - cx) * s + (py - cy) * c;
    let best = Infinity;
    for (const [x, y] of prof) best = Math.min(best, Math.hypot(x - lx, y - ly));
    return best;
  };
  let minC = Infinity, maxGapA = 0;
  // Orientation du disque B : décalage d’un demi-lobe
  const phaseB = Math.PI / (N - 1);
  for (let k = 0; k < steps; k++) {
    const phi = (2 * Math.PI * k) / steps;
    const th = -phi / (N - 1);
    for (const [cx, cy, t] of [[E * Math.cos(phi), E * Math.sin(phi), th], [-E * Math.cos(phi), -E * Math.sin(phi), th + phaseB]]) {
      let minPin = Infinity;
      for (let i = 0; i < N; i++) {
        const a = (2 * Math.PI * i) / N;
        const dd = dist(R * Math.cos(a), R * Math.sin(a), cx, cy, t) - rr;
        minPin = Math.min(minPin, dd);
      }
      minC = Math.min(minC, minPin);
      maxGapA = Math.max(maxGapA, minPin);
    }
  }
  return { phaseB, minClearance: minC, maxGap: maxGapA };
}

// ------------------------------------------------------------------ pièces

/** Méplat de l’arbre moteur : plan à (r − 0,5) mm de l’axe, côté +x. */
export const shaftFlat = (mot, len) => kit.cbox(mot.shaft, mot.shaft, len).translate([mot.shaft / 2 - 0.5 + mot.shaft / 2, 0, 0]);

const motorDummy = (d) => {
  const m = d.mot;
  const body = kit.roundRect(m.size, m.size, d.motorLen, m.chamfer).translate([0, 0, -d.motorLen]);
  const pilot = kit.cyl(m.pilotH, m.pilotD / 2);
  const shaft = kit.diff(kit.cyl(m.shaftLen, m.shaft / 2), shaftFlat(m, m.shaftLen + 2).translate([0, 0, -1]));
  return kit.union([body, pilot, shaft]);
};

export function buildModuleParts(spec, tol = DEFAULT_TOL) {
  const d = moduleDims(spec, tol);
  const s = spec, mb = d.mb, cb = d.cb, mot = d.mot;
  const parts = [];
  const add = (p) => { parts.push(p); return p; };

  // ---------------- carter (fixe)
  {
    const floor = kit.cyl(d.tf, d.housingR);
    const wallDisc = kit.tube(d.zGroove - d.tf, d.housingR, s.R).translate([0, 0, d.tf]);
    const ledge = kit.tube(d.zB0 - d.zGroove, d.housingR, d.ledgeR).translate([0, 0, d.zGroove]);
    const seat = kit.tube(mb.B, d.housingR, mb.D / 2 + tol.bearingSeat / 2).translate([0, 0, d.zB0]);
    const ring = kit.tube(d.ringT, d.ringR, mb.D / 2 + tol.bearingSeat / 2).translate([0, 0, d.ringBack]);
    let body = kit.union([floor, wallDisc, ledge, seat, ring]);
    const cuts = [];
    // Gorges des galets (sur toute la hauteur du disque) + trous emmanchés dans le fond
    cuts.push(kit.polar(s.N, (a) => kit.onCircle(kit.cyl(d.zGroove - d.tf + 0.02, d.rr + tol.pinSlide / 2), s.R, a, d.tf - 0.01)));
    cuts.push(kit.polar(s.N, (a) => kit.onCircle(kit.cyl(d.pinHole + 0.01, d.rr + tol.pinPress / 2), s.R, a, d.tf - d.pinHole)));
    // Centrage moteur + passage du moyeu de came
    cuts.push(kit.cyl(mot.pilotH + 0.4, mot.pilotD / 2 + 0.25).translate([0, 0, -0.01]));
    cuts.push(kit.cyl(d.tf + 0.02, d.hubR + 1.0).translate([0, 0, -0.01]));
    // Vis moteur (tête côté intérieur, lamée sous la face du fond)
    const hb = mot.bolt / 2;
    for (const [x, y] of [[hb, hb], [-hb, hb], [hb, -hb], [-hb, -hb]]) {
      const headH = mot.boltSize >= 5 ? 5.4 : 3.4;
      cuts.push(kit.counterbore(mot.boltSize, d.tf + 0.02, headH).translate([x, y, -0.01]));
    }
    // Fixation au support (boulons traversant bague + couvercle, inserts dans la pièce hôte)
    cuts.push(kit.polar(s.ring.n, (a) => kit.onCircle(kit.clearance(s.ring.size, d.ringT + 0.02), d.ringBoltR, a, d.ringBack - 0.01), 0));
    body = kit.diff(body, cuts);
    add({ key: 'housing', name: `Carter ${s.label}`, role: 'stator', printed: true, fill: 0.6, qty: 1, mesh: body, print: { flip: false }, note: 'Imprimer fond sur le plateau, sans supports (les gorges sont verticales).' });
  }

  // ---------------- couvercle de retenue (fixe)
  {
    let ret = kit.tube(3, d.ringR, d.retInnerR).translate([0, 0, d.zFront]);
    ret = kit.diff(ret, kit.polar(s.ring.n, (a) => kit.onCircle(kit.clearance(s.ring.size, 3.2), d.ringBoltR, a, d.zFront - 0.1), 0));
    add({ key: 'retainer', name: `Couvercle ${s.label}`, role: 'stator', printed: true, fill: 0.8, qty: 1, mesh: ret, print: { flip: false }, note: 'Maintient la bague extérieure du roulement principal ; serré par les boulons de fixation du module.' });
  }

  // ---------------- came excentrique (tourne avec le moteur)
  {
    const lobeL = s.t + tol.gap / 2;
    const hub = kit.cyl(d.zA - 2.5, d.hubR).translate([0, 0, 2.5]);
    const lobeA = kit.cyl(lobeL, cb.d / 2).translate([s.E, 0, d.zA]);
    const lobeB = kit.cyl(lobeL, cb.d / 2).translate([-s.E, 0, d.zA + lobeL]);
    const jStart = d.zA + 2 * lobeL;
    const journal = kit.cyl(Math.max(0.1, d.zF0 - jStart) + 5.2, 4).translate([0, 0, jStart]);
    let cam = kit.union([hub, lobeA, lobeB, journal]);
    // Alésage en D (méplat) pour l’arbre moteur
    const bore = kit.diff(kit.cyl(mot.shaftLen + 1, mot.shaft / 2 + tol.shaftPress / 2), shaftFlat(mot, mot.shaftLen + 3).translate([0, 0, -1]));
    cam = kit.diff(cam, bore.translate([0, 0, 2.49]));
    add({ key: 'cam', name: `Came excentrique ${s.label}`, role: 'internal', printed: true, fill: 1.0, qty: 1, mesh: cam, print: { flip: false }, note: `Remplissage 100 %. Excentricité ${s.E} mm. Emmanchée sur l’arbre moteur (méplat), goutte de colle frein.` });
  }

  // ---------------- disques cycloïdaux A et B
  const prof = cycloidPoints(s.R, d.rr, s.E, s.N);
  const discAt = (rotDeg) => {
    let disc = new kit.M.CrossSection([prof]).rotate(rotDeg).extrude(s.t);
    const cuts = [kit.cyl(s.t + 0.02, cb.D / 2 + tol.bearingSeat / 2).translate([0, 0, -0.01])];
    cuts.push(kit.polar(s.nOut, (a) => kit.onCircle(kit.cyl(s.t + 0.02, d.outHole / 2), s.Ro, a, -0.01), 0));
    return kit.diff(disc, cuts);
  };
  const halfLobe = 180 / (s.N - 1);
  add({ key: 'discA', name: `Disque cycloïdal A ${s.label}`, role: 'internal', printed: true, fill: 1.0, qty: 1, mesh: discAt(0).translate([s.E, 0, d.zA]), print: { flip: false, recenter: true }, note: `${s.N - 1} lobes. Remplissage 100 %, 4 périmètres, couche 0,12–0,16 mm pour le profil.` });
  add({ key: 'discB', name: `Disque cycloïdal B ${s.label}`, role: 'internal', printed: true, fill: 1.0, qty: 1, mesh: discAt(halfLobe).translate([-s.E, 0, d.zB]), print: { flip: false, recenter: true }, note: `Identique au disque A mais décalé d’un demi-lobe (${halfLobe.toFixed(2)}°) : repérer « B ».` });

  // ---------------- flasque de sortie (tourne en sortie)
  {
    const shoulder = kit.cyl(d.zB0 - d.zF0, d.shoulderR).translate([0, 0, d.zF0]);
    const seat = kit.cyl(mb.B, d.flangeR - 0.02).translate([0, 0, d.zB0]);
    const front = kit.cyl(d.zOut - d.zFront, d.flangeR - 0.1).translate([0, 0, d.zFront]);
    let fl = kit.union([shoulder, seat, front]);
    const cuts = [];
    cuts.push(kit.polar(s.nOut, (a) => kit.onCircle(kit.cyl(d.outEmbed + 0.3, s.dOut / 2 + tol.pinPress / 2), s.Ro, a, d.zF0 - 0.01), 0));
    cuts.push(kit.cyl(5.3, BEARINGS[688].D / 2 + tol.bearingSeat / 2).translate([0, 0, d.zF0 - 0.01])); // roulement de support de came
    cuts.push(kit.cyl(1.2, 3.2).translate([0, 0, d.zF0 + 5.2])); // dégagement bague intérieure
    const off = 180 / s.iface.n;
    cuts.push(kit.polar(s.iface.n, (a) => kit.onCircle(kit.insert(s.iface.size, 6.5), d.ifaceR, a, d.zOut - 6.5), off));
    cuts.push(kit.cyl(6, 4.5).translate([0, 0, d.zOut - 3])); // évidement central (allègement)
    fl = kit.diff(fl, cuts);
    add({ key: 'flange', name: `Flasque de sortie ${s.label}`, role: 'rotor', printed: true, fill: 0.9, qty: 1, mesh: fl, print: { flip: true }, note: `Imprimer face avant sur le plateau. ${s.iface.n} inserts M${s.iface.size} ; ${s.nOut} doigts Ø${s.dOut} emmanchés.` });
  }

  // ---------------- bague de sortie (serre la bague intérieure du roulement)
  {
    const plate = kit.cyl(3, d.clampR).translate([0, 0, d.zOut]);
    const skirt = kit.tube(d.zOut - d.zFront, d.clampR, d.flangeR + 0.25).translate([0, 0, d.zFront]);
    let cl = kit.union([plate, skirt]);
    const off = 180 / s.iface.n;
    cl = kit.diff(cl, [
      kit.polar(s.iface.n, (a) => kit.onCircle(kit.clearance(s.iface.size, 3.2), d.ifaceR, a, d.zOut - 0.1), off),
      kit.cyl(3.2, 5).translate([0, 0, d.zOut - 0.1]),
    ]);
    add({ key: 'clamp', name: `Bague de sortie ${s.label}`, role: 'rotor', printed: true, fill: 0.9, qty: 1, mesh: cl, print: { flip: true }, note: 'Face d’interface du segment suivant. Serre la bague intérieure du roulement.' });
  }

  // ---------------- composants du commerce (visualisation + masses)
  const steel = (h, ro, ri) => kit.tube(h, ro, ri);
  add({ key: 'motor', name: `Moteur ${s.motor}`, role: 'stator', printed: false, qty: 1, mesh: motorDummy(d), massKg: null, vendor: true });
  add({ key: 'mainBearing', name: `Roulement ${s.main}`, role: 'stator', printed: false, qty: 1, mesh: steel(mb.B, mb.D / 2, mb.d / 2).translate([0, 0, d.zB0]), massKg: mb.mass, vendor: true });
  add({ key: 'camBearings', name: `Roulements ${s.camBrg} (×2)`, role: 'internal', printed: false, qty: 1, mesh: kit.union([
    steel(cb.B, cb.D / 2, cb.d / 2).translate([s.E, 0, d.zA + (s.t - cb.B) / 2]),
    steel(cb.B, cb.D / 2, cb.d / 2).translate([-s.E, 0, d.zB + (s.t - cb.B) / 2]),
  ]), massKg: cb.mass * 2, vendor: true });
  add({ key: 'supBearing', name: 'Roulement 688', role: 'rotor', printed: false, qty: 1, mesh: steel(5, 8, 4).translate([0, 0, d.zF0]), massKg: BEARINGS[688].mass, vendor: true });
  add({ key: 'pins', name: `Galets Ø${s.dp}×${s.pinLen} (×${s.N})`, role: 'stator', printed: false, qty: 1, mesh: kit.polar(s.N, (a) => kit.onCircle(kit.cyl(s.pinLen, d.rr, d.rr, false, 16), s.R, a, d.tf - d.pinHole)), density: 7.85, vendor: true });
  add({ key: 'outPins', name: `Doigts Ø${s.dOut}×${s.outPinLen} (×${s.nOut})`, role: 'rotor', printed: false, qty: 1, mesh: kit.polar(s.nOut, (a) => kit.onCircle(kit.cyl(s.outPinLen, s.dOut / 2, s.dOut / 2, false, 16), s.Ro, a, d.zF0 + d.outEmbed - s.outPinLen)), density: 7.85, vendor: true });
  return { dims: d, parts };
}
