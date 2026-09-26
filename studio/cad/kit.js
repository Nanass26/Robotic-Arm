// ORION-6 — Boîte à outils CAO (au-dessus de manifold-3d). Unités : millimètres, angles en degrés.
// Fonctionne dans Node (génération des STL) et dans le navigateur (régénération paramétrique).

let M = null; // { Manifold, CrossSection }

export function setupKit(wasm) {
  M = wasm;
  wasm.setMinCircularAngle?.(4);
  wasm.setMinCircularEdgeLength?.(0.6);
  return kit;
}

export const isReady = () => !!M;

const segs = (r, fine = 1) => Math.max(24, Math.min(160, Math.ceil((2 * Math.PI * r) / (0.9 / fine)) & ~3));

export const kit = {
  get M() { return M; },

  box(size, center = false) { return M.Manifold.cube(size, center); },
  /** Boîte centrée en x, y et posée sur z = 0 (ou centrée en z). */
  cbox(sx, sy, sz, centerZ = false) { return M.Manifold.cube([sx, sy, sz], true).translate([0, 0, centerZ ? 0 : sz / 2]); },
  cyl(h, r, r2 = r, center = false, n = 0) { return M.Manifold.cylinder(h, r, r2, n || segs(Math.max(r, r2)), center); },
  sphere(r, n = 0) { return M.Manifold.sphere(r, n || segs(r)); },
  tube(h, ro, ri, center = false) { return kit.cyl(h, ro, ro, center).subtract(kit.cyl(h + 0.02, ri, ri, center).translate([0, 0, center ? 0 : -0.01])); },

  /** Rectangle à coins arrondis extrudé (centré en x, y). */
  roundRect(sx, sy, h, r) {
    r = Math.min(r, sx / 2 - 0.01, sy / 2 - 0.01);
    const cs = M.CrossSection.square([sx - 2 * r, sy - 2 * r], true).offset(r, 'Round', 2, segs(r));
    return cs.extrude(h);
  },
  /** Polygone extrudé (orientation corrigée automatiquement). */
  poly(points, h) {
    let area = 0;
    for (let i = 0; i < points.length; i++) { const a = points[i], b = points[(i + 1) % points.length]; area += a[0] * b[1] - b[0] * a[1]; }
    const pts = area < 0 ? points.slice().reverse() : points;
    return new M.CrossSection([pts]).extrude(h);
  },
  circle2(r) { return M.CrossSection.circle(r, segs(r)); },

  union(list) { const l = list.filter(Boolean); return l.length === 1 ? l[0] : M.Manifold.union(l); },
  diff(a, list) { const l = (Array.isArray(list) ? list : [list]).filter(Boolean); return l.length ? a.subtract(M.Manifold.union(l)) : a; },
  hull(list) { return M.Manifold.hull(list); },

  /** Répète `make(angleDeg, i)` n fois autour de z. */
  polar(n, make, phase = 0) {
    const out = [];
    for (let i = 0; i < n; i++) out.push(make(phase + (360 * i) / n, i));
    return kit.union(out);
  },
  at(m, x, y, z = 0) { return m.translate([x, y, z]); },
  onCircle(m, r, angleDeg, z = 0) { const a = (angleDeg * Math.PI) / 180; return m.translate([r * Math.cos(a), r * Math.sin(a), z]); },

  // ------------------------------------------------ perçages standard (jeu d’impression inclus)
  /** Trou de passage pour vis métrique (M3 → 3,4 mm). */
  clearance(size, h, extra = 0.4) { return kit.cyl(h, size / 2 + extra / 2, size / 2 + extra / 2); },
  /** Logement d’insert fileté à chaud (laiton), profondeur d. */
  insert(size, depth) {
    const d = { 2: 3.2, 2.5: 3.6, 3: 4.0, 4: 5.6, 5: 6.4, 6: 8.0 }[size] ?? size * 1.35;
    return kit.cyl(depth, d / 2, d / 2);
  },
  /** Lamage de tête de vis CHC (ISO 4762). */
  counterbore(size, depth, headDepth) {
    const head = { 2: 3.8, 2.5: 4.5, 3: 5.5, 4: 7.0, 5: 8.5, 6: 10 }[size] ?? size * 1.7;
    return kit.union([kit.clearance(size, depth), kit.cyl(headDepth, head / 2 + 0.3, head / 2 + 0.3).translate([0, 0, depth - headDepth])]);
  },
  /** Logement d’écrou hexagonal (surplat af). */
  nutTrap(size, depth) {
    const af = { 2: 4, 2.5: 5, 3: 5.5, 4: 7, 5: 8, 6: 10 }[size] ?? size * 1.8;
    const r = (af + 0.35) / Math.sqrt(3);
    return kit.cyl(depth, r, r, false, 6);
  },

  // ------------------------------------------------ transformations nommées
  rx: (m, a) => m.rotate([a, 0, 0]),
  ry: (m, a) => m.rotate([0, a, 0]),
  rz: (m, a) => m.rotate([0, 0, a]),
  /** Oriente l’axe z local vers +x, +y, −y… (pour placer un module dans l’assemblage). */
  axisTo(m, axis) {
    switch (axis) {
      case '+z': return m;
      case '-z': return m.rotate([180, 0, 0]);
      case '+x': return m.rotate([0, 90, 0]);
      case '-x': return m.rotate([0, -90, 0]);
      case '+y': return m.rotate([-90, 0, 0]);
      case '-y': return m.rotate([90, 0, 0]);
      default: throw new Error(`axe inconnu ${axis}`);
    }
  },
};

// ------------------------------------------------------------------ profil cycloïdal
/**
 * Profil d’un disque cycloïdal (épitrochoïde décalée), N galets de rayon rr sur un cercle R,
 * excentricité e → N−1 lobes. Formules classiques (Hydrox / Younis) :
 *   ψ = atan2(sin((1−N)t), R/(eN) − cos((1−N)t))
 *   x = R cos t − rr cos(t + ψ) − e cos(N t)
 *   y = −R sin t + rr sin(t + ψ) + e sin(N t)
 */
export function cycloidPoints(R, rr, e, N, samples = 0) {
  const n = samples || Math.max(720, (N - 1) * 48);
  const pts = [];
  for (let k = 0; k < n; k++) {
    const t = (2 * Math.PI * k) / n;
    const psi = Math.atan2(Math.sin((1 - N) * t), R / (e * N) - Math.cos((1 - N) * t));
    const x = R * Math.cos(t) - rr * Math.cos(t + psi) - e * Math.cos(N * t);
    const y = -R * Math.sin(t) + rr * Math.sin(t + psi) + e * Math.sin(N * t);
    pts.push([x, y]);
  }
  // Orientation trigonométrique (sens direct) exigée par CrossSection
  let area = 0;
  for (let i = 0; i < pts.length; i++) { const a = pts[i], b = pts[(i + 1) % pts.length]; area += a[0] * b[1] - b[0] * a[1]; }
  if (area < 0) pts.reverse();
  return pts;
}

/** Vérifie qu’un polygone ne s’auto-intersecte pas (test O(n²) sur segments non voisins, échantillonné). */
export function polygonIsSimple(pts, stride = 1) {
  const n = pts.length;
  const seg = (i) => [pts[i], pts[(i + 1) % n]];
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const inter = ([p1, p2], [p3, p4]) => {
    const d1 = cross(p3, p4, p1), d2 = cross(p3, p4, p2), d3 = cross(p1, p2, p3), d4 = cross(p1, p2, p4);
    return ((d1 > 0) !== (d2 > 0)) && ((d3 > 0) !== (d4 > 0));
  };
  for (let i = 0; i < n; i += stride) {
    for (let j = i + 2; j < n; j += stride) {
      if (i === 0 && j === n - 1) continue;
      if (inter(seg(i), seg(j))) return false;
    }
  }
  return true;
}

// ------------------------------------------------------------------ maillage, STL, propriétés
export function meshOf(m) {
  const mesh = m.getMesh();
  const np = mesh.numProp;
  const vp = mesh.vertProperties;
  const nv = vp.length / np;
  const positions = new Float32Array(nv * 3);
  for (let i = 0; i < nv; i++) { positions[i * 3] = vp[i * np]; positions[i * 3 + 1] = vp[i * np + 1]; positions[i * 3 + 2] = vp[i * np + 2]; }
  return { positions, indices: new Uint32Array(mesh.triVerts) };
}

/** STL binaire (Uint8Array) — unités mm. */
export function toSTL(m, name = 'orion6') {
  const { positions: P, indices: I } = meshOf(m);
  const nt = I.length / 3;
  const buf = new ArrayBuffer(84 + nt * 50);
  const dv = new DataView(buf);
  const header = `ORION-6 ${name}`.slice(0, 79);
  for (let i = 0; i < header.length; i++) dv.setUint8(i, header.charCodeAt(i) & 0x7f);
  dv.setUint32(80, nt, true);
  let o = 84;
  for (let t = 0; t < nt; t++) {
    const a = I[t * 3] * 3, b = I[t * 3 + 1] * 3, c = I[t * 3 + 2] * 3;
    const ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2];
    const vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2];
    let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1;
    nx /= l; ny /= l; nz /= l;
    dv.setFloat32(o, nx, true); dv.setFloat32(o + 4, ny, true); dv.setFloat32(o + 8, nz, true);
    dv.setFloat32(o + 12, P[a], true); dv.setFloat32(o + 16, P[a + 1], true); dv.setFloat32(o + 20, P[a + 2], true);
    dv.setFloat32(o + 24, P[b], true); dv.setFloat32(o + 28, P[b + 1], true); dv.setFloat32(o + 32, P[b + 2], true);
    dv.setFloat32(o + 36, P[c], true); dv.setFloat32(o + 40, P[c + 1], true); dv.setFloat32(o + 44, P[c + 2], true);
    dv.setUint16(o + 48, 0, true);
    o += 50;
  }
  return new Uint8Array(buf);
}

/**
 * Propriétés de masse d’un solide homogène (décomposition en tétraèdres, Tonon 2004).
 * Retourne { volume (mm³), com [mm], inertia [Ixx,Iyy,Izz,Ixy,Ixz,Iyz] (mm⁵, à multiplier par ρ) au CdG }.
 */
export function massProps(m) {
  const { positions: P, indices: I } = meshOf(m);
  let V = 0, cx = 0, cy = 0, cz = 0;
  let xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0;
  for (let t = 0; t < I.length; t += 3) {
    const a = I[t] * 3, b = I[t + 1] * 3, c = I[t + 2] * 3;
    const x1 = P[a], y1 = P[a + 1], z1 = P[a + 2];
    const x2 = P[b], y2 = P[b + 1], z2 = P[b + 2];
    const x3 = P[c], y3 = P[c + 1], z3 = P[c + 2];
    const v = (x1 * (y2 * z3 - y3 * z2) - y1 * (x2 * z3 - x3 * z2) + z1 * (x2 * y3 - x3 * y2)) / 6;
    V += v;
    cx += v * (x1 + x2 + x3) / 4; cy += v * (y1 + y2 + y3) / 4; cz += v * (z1 + z2 + z3) / 4;
    const f = (p, q, r) => p * p + q * q + r * r + p * q + p * r + q * r;
    const g = (p1, p2, p3, q1, q2, q3) => 2 * p1 * q1 + 2 * p2 * q2 + 2 * p3 * q3 + p1 * q2 + p2 * q1 + p1 * q3 + p3 * q1 + p2 * q3 + p3 * q2;
    xx += v * f(x1, x2, x3) / 10; yy += v * f(y1, y2, y3) / 10; zz += v * f(z1, z2, z3) / 10;
    xy += v * g(x1, x2, x3, y1, y2, y3) / 20; xz += v * g(x1, x2, x3, z1, z2, z3) / 20; yz += v * g(y1, y2, y3, z1, z2, z3) / 20;
  }
  if (Math.abs(V) < 1e-12) return { volume: 0, com: [0, 0, 0], inertia: [0, 0, 0, 0, 0, 0] };
  cx /= V; cy /= V; cz /= V;
  // Moments d’ordre 2 au CdG (théorème de Huygens)
  const Sxx = xx - V * cx * cx, Syy = yy - V * cy * cy, Szz = zz - V * cz * cz;
  const Sxy = xy - V * cx * cy, Sxz = xz - V * cx * cz, Syz = yz - V * cy * cz;
  return {
    volume: V,
    com: [cx, cy, cz],
    inertia: [Syy + Szz, Sxx + Szz, Sxx + Syy, -Sxy, -Sxz, -Syz],
  };
}
