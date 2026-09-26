// ORION-6 — Nomenclature (BOM) calculée depuis l’assemblage.
// Prix indicatifs (€, 2026, achat à l’unité en ligne) — à titre d’ordre de grandeur.

import { partMass } from './orion6.js';
import { MOTOR_LIBRARY } from '../core/defaults.js';

const PRICE = {
  'NEMA17-34': 11, 'NEMA17-40': 12, 'NEMA17-48': 14, 'NEMA17-60': 17, 'NEMA23-56': 24, 'NEMA23-76': 32,
  6802: 1.5, 6803: 1.6, 6804: 2.2, 6810: 6, 6811: 8, 6814: 12, 688: 0.8,
};

export function buildBOM(asm) {
  const items = [];
  const add = (cat, ref, desc, qty, unit = 'pce', price = null, note = '') => {
    const ex = items.find((i) => i.cat === cat && i.ref === ref);
    if (ex) { ex.qty += qty; return; }
    items.push({ cat, ref, desc, qty, unit, price, note });
  };
  // Composants issus de l’assemblage
  for (const p of asm.parts) {
    if (!p.vendor) continue;
    if (p.motor) add('Moteurs', p.motor, `Moteur pas-à-pas ${p.motor.replace('-', ' ')} mm, 1,8°/pas (ex. ${MOTOR_LIBRARY[p.motor]?.holdingTorque} N·m)`, 1, 'pce', PRICE[p.motor]);
    else if (/^J\d-mainBearing/.test(p.id)) { const r = p.name.split(' ').pop(); add('Roulements', r, `Roulement à billes à section mince ${r} 2RS`, 1, 'pce', PRICE[r]); }
    else if (/^J\d-camBearings/.test(p.id)) { const r = p.name.match(/\d{4}/)[0]; add('Roulements', r, `Roulement à billes ${r} 2RS (came)`, 2, 'pce', PRICE[r]); }
    else if (/^J\d-supBearing/.test(p.id)) add('Roulements', '688', 'Roulement 688 2RS (8×16×5, support de came)', 1, 'pce', PRICE[688]);
    else if (/^J\d-pins/.test(p.id) || /^J\d-outPins/.test(p.id)) {
      const m = p.name.match(/Ø(\d+)×(\d+) \(×(\d+)\)/);
      if (m) add('Goupilles', `Ø${m[1]}×${m[2]}`, `Goupille cylindrique acier trempé ISO 8734 Ø${m[1]}×${m[2]} mm`, +m[3], 'pce', m[1] === '5' ? 0.12 : 0.08);
    } else if (/^tube-/.test(p.id)) add('Structure', `Tube Ø40×2 L=${p.cutLength}`, `Tube rond aluminium 6060 Ø40×2 mm, à couper à ${p.cutLength} mm (ébavurer)`, 1, 'pce', 6);
    else if (p.id === 'gripper-servo') add('Pince', 'MG996R', 'Servomoteur MG996R (couple 9–11 kg·cm, pignons métal) + palonnier rond', 1, 'pce', 7);
  }
  // Visserie (estimations par liaison)
  const specs = asm.specs;
  for (const k of Object.keys(specs)) {
    const s = specs[k];
    const ring = s.ring.size, iface = s.iface.size;
    add('Visserie', `M${ring}×16 CHC`, `Vis CHC ISO 4762 M${ring}×16 (fixation des modules)`, s.ring.n, 'pce', 0.08);
    add('Inserts', `M${ring} insert`, `Insert fileté laiton à chaud M${ring} (L ≈ 6 mm)`, s.ring.n, 'pce', 0.1);
    add('Visserie', `M${iface}×16 CHC`, `Vis CHC ISO 4762 M${iface}×16 (interfaces de sortie)`, s.iface.n, 'pce', 0.08);
    add('Inserts', `M${iface} insert`, `Insert fileté laiton à chaud M${iface} (L ≈ 6 mm)`, s.iface.n, 'pce', 0.1);
    if (s.motor === 'NEMA23') { add('Visserie', 'M5×16 CHC', 'Vis CHC M5×16 + écrou M5 (moteur NEMA23)', 4, 'pce', 0.12); add('Visserie', 'Écrou M5', 'Écrou hexagonal M5', 4, 'pce', 0.03); }
    else add('Visserie', 'M3×8 CHC', 'Vis CHC M3×8 (moteur NEMA17)', 4, 'pce', 0.05);
  }
  add('Visserie', 'M4×30 CHC', 'Vis CHC M4×30 + écrou (serrage des tubes)', 2, 'pce', 0.1);
  add('Visserie', 'Écrou M4', 'Écrou hexagonal M4', 2, 'pce', 0.03);
  add('Visserie', 'M3×50 CHC', 'Vis M3×50 + écrou nylstop (goupillage du tube d’avant-bras)', 2, 'pce', 0.15);
  add('Visserie', 'M4×12 CHC', 'Vis CHC M4×12 (plaque de base → socle)', 6, 'pce', 0.06);
  add('Inserts', 'M4 insert', 'Insert fileté laiton à chaud M4 (L ≈ 6 mm)', 6, 'pce', 0.1);
  add('Visserie', 'M3×12 CHC', 'Vis CHC M3×12 (pince → bride J6)', 6, 'pce', 0.05);
  add('Visserie', 'M2×10', 'Vis M2×10 + écrou (servo de pince, pignon)', 8, 'pce', 0.05);
  add('Visserie', 'M6 fixation', 'Vis M6 + rondelle (fixation à la table)', 4, 'pce', 0.2);
  add('Consommables', 'Colle frein', 'Frein filet faible (bleu) + graisse PTFE pour réducteurs', 1, 'lot', 8);
  add('Consommables', 'Époxy', 'Colle époxy bi-composant (tubes, cames)', 1, 'lot', 6);
  // Électronique
  add('Électronique', 'Teensy 4.1', 'Microcontrôleur Teensy 4.1 (600 MHz, USB)', 1, 'pce', 35);
  add('Électronique', 'DM542T', 'Driver pas-à-pas numérique DM542T (20–50 V, 1–4,2 A) — ou TMC5160 / DM320T pour les petits axes', 6, 'pce', 22);
  add('Électronique', 'Alim 36 V', 'Alimentation 36 V 10 A (ex. Mean Well LRS-350-36)', 1, 'pce', 40);
  add('Électronique', 'Buck 6 V', 'Convertisseur abaisseur 36→6 V 5 A (servo de pince)', 1, 'pce', 6);
  add('Électronique', 'Buck 5 V', 'Convertisseur 36→5 V 3 A (Teensy, capteurs)', 1, 'pce', 4);
  add('Électronique', 'Capteur Hall', 'Capteur à effet Hall A3144 (ou NJK-5002C) + aimant néodyme 6×3 mm', 6, 'pce', 1.5);
  add('Électronique', 'Arrêt d’urgence', 'Bouton champignon 22 mm, 1 NF + 1 NO, à accrochage', 1, 'pce', 8);
  add('Électronique', 'Relais', 'Contacteur/relais 12–36 V 20 A (coupure puissance moteurs par l’AU)', 1, 'pce', 10);
  add('Électronique', 'Câble 4G', 'Câble blindé 4×0,5 mm² (moteurs) — env. 8 m', 8, 'm', 1.2);
  add('Électronique', 'Câble 3G', 'Câble 3×0,25 mm² (capteurs, servo) — env. 8 m', 8, 'm', 0.6);
  add('Électronique', 'Connecteurs', 'Connecteurs GX16-4 (moteurs) + XT60 (puissance) + borniers', 1, 'lot', 18);
  add('Électronique', 'Fusible', 'Porte-fusible + fusible 10 A', 1, 'pce', 3);
  // Pièces imprimées et filament
  let filament = 0;
  for (const p of asm.parts) {
    if (!p.printed) continue;
    const m = partMass(p, MOTOR_LIBRARY).mass;
    filament += m * (p.qty || 1);
    add('Pièces imprimées', p.id, p.name, p.qty || 1, 'pce', null, `${p.material || 'PETG/PLA+'} · remplissage ${Math.round((p.fill ?? 0.5) * 100)} %`);
  }
  add('Consommables', 'Filament', `Filament PETG ou PLA+ (≈ ${(filament * 1000).toFixed(0)} g de pièces + 25 % de marge/supports)`, Math.ceil(filament * 1.25 * 10) / 10, 'kg', 22);
  const total = items.reduce((s, i) => s + (i.price ? i.price * i.qty : 0), 0);
  return { items, filamentKg: filament, totalEuro: total };
}

export function bomToCSV(bom) {
  const rows = [['Catégorie', 'Référence', 'Désignation', 'Quantité', 'Unité', 'Prix unitaire indicatif (€)', 'Note']];
  for (const i of bom.items) rows.push([i.cat, i.ref, i.desc, String(Math.round(i.qty * 100) / 100), i.unit, i.price != null ? i.price.toFixed(2) : '', i.note || '']);
  return rows.map((r) => r.map((c) => (/[;"\n]/.test(c) ? `"${c.replace(/"/g, '""')}"` : c)).join(';')).join('\n') + '\n';
}
