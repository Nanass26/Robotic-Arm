// ORION-6 — Outils mathématiques 3D (vecteurs, rotations, transformations homogènes).
// Conventions :
//   - Vec3 : tableau [x, y, z]
//   - Mat3 : tableau plat de 9 éléments, ligne par ligne (row-major)
//   - Mat4 : tableau plat de 16 éléments, ligne par ligne, transformation homogène
//   - Quaternion : [w, x, y, z] (unitaire)
//   - Angles en radians, longueurs en mètres (SI).

export const DEG = Math.PI / 180;
export const RAD = 180 / Math.PI;
export const TAU = Math.PI * 2;

export const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
export const lerp = (a, b, t) => a + (b - a) * t;

/** Ramène un angle dans ]-π, π]. */
export function wrapAngle(a) {
  a = (a + Math.PI) % TAU;
  if (a < 0) a += TAU;
  return a - Math.PI;
}

// ---------------------------------------------------------------- Vec3
export const v3 = (x = 0, y = 0, z = 0) => [x, y, z];
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const norm = (a) => Math.hypot(a[0], a[1], a[2]);
export function normalize(a) {
  const n = norm(a);
  return n > 1e-15 ? [a[0] / n, a[1] / n, a[2] / n] : [0, 0, 0];
}
export const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
export const addScaled = (a, b, s) => [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s];

// ---------------------------------------------------------------- Mat3
export const mat3Identity = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];

export function mat3Mul(A, B) {
  const C = new Array(9);
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      C[r * 3 + c] = A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
    }
  }
  return C;
}

export const mat3T = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];

export const mat3MulVec = (A, v) => [
  A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
  A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
  A[6] * v[0] + A[7] * v[1] + A[8] * v[2],
];

export function mat3Rx(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [1, 0, 0, 0, c, -s, 0, s, c];
}
export function mat3Ry(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, 0, s, 0, 1, 0, -s, 0, c];
}
export function mat3Rz(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, -s, 0, s, c, 0, 0, 0, 1];
}

/** Matrice de rotation depuis roulis/tangage/lacet (convention ZYX : R = Rz(yaw)·Ry(pitch)·Rx(roll)). */
export function rpyToMat3(roll, pitch, yaw) {
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  ];
}

/** Inverse de rpyToMat3 → [roll, pitch, yaw]. Gère le blocage de cardan (pitch = ±90°). */
export function mat3ToRpy(R) {
  const sp = -R[6];
  if (Math.abs(sp) > 1 - 1e-10) {
    // Blocage de cardan : on fixe roll = 0.
    const pitch = Math.sign(sp) * Math.PI / 2;
    const yaw = Math.atan2(-R[1], R[4]);
    return [0, pitch, yaw];
  }
  return [Math.atan2(R[7], R[8]), Math.asin(clamp(sp, -1, 1)), Math.atan2(R[3], R[0])];
}

/** Rotation d'angle `angle` autour de l'axe unitaire `axis` (formule de Rodrigues). */
export function axisAngleToMat3(axis, angle) {
  const [x, y, z] = normalize(axis);
  const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c,
  ];
}

/** Vecteur rotation (axe × angle) de R (logarithme de SO(3)), robuste près de 0 et π. */
export function mat3ToRotVec(R) {
  const tr = R[0] + R[4] + R[8];
  const cosA = clamp((tr - 1) / 2, -1, 1);
  const angle = Math.acos(cosA);
  if (angle < 1e-9) {
    // Approximation au premier ordre.
    return [(R[7] - R[5]) / 2, (R[2] - R[6]) / 2, (R[3] - R[1]) / 2];
  }
  if (Math.PI - angle < 1e-6) {
    // Proche de π : l'axe se lit sur la diagonale.
    const xx = (R[0] + 1) / 2, yy = (R[4] + 1) / 2, zz = (R[8] + 1) / 2;
    let axis;
    if (xx >= yy && xx >= zz) {
      const x = Math.sqrt(Math.max(xx, 0));
      axis = [x, R[1] / (2 * x), R[2] / (2 * x)];
    } else if (yy >= zz) {
      const y = Math.sqrt(Math.max(yy, 0));
      axis = [R[1] / (2 * y), y, R[5] / (2 * y)];
    } else {
      const z = Math.sqrt(Math.max(zz, 0));
      axis = [R[2] / (2 * z), R[5] / (2 * z), z];
    }
    return scale(normalize(axis), angle);
  }
  const k = angle / (2 * Math.sin(angle));
  return [(R[7] - R[5]) * k, (R[2] - R[6]) * k, (R[3] - R[1]) * k];
}

/** Erreur d'orientation (vecteur rotation) qui amène Rcur vers Rdes, exprimée dans le repère de base. */
export function rotError(Rdes, Rcur) {
  return mat3ToRotVec(mat3Mul(Rdes, mat3T(Rcur)));
}

/** Angle (rad) entre deux orientations. */
export function rotAngleBetween(Ra, Rb) {
  const R = mat3Mul(mat3T(Ra), Rb);
  return Math.acos(clamp((R[0] + R[4] + R[8] - 1) / 2, -1, 1));
}

// ---------------------------------------------------------------- Quaternions [w, x, y, z]
export function mat3ToQuat(R) {
  const tr = R[0] + R[4] + R[8];
  let w, x, y, z;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    w = 0.25 * s; x = (R[7] - R[5]) / s; y = (R[2] - R[6]) / s; z = (R[3] - R[1]) / s;
  } else if (R[0] > R[4] && R[0] > R[8]) {
    const s = Math.sqrt(1 + R[0] - R[4] - R[8]) * 2;
    w = (R[7] - R[5]) / s; x = 0.25 * s; y = (R[1] + R[3]) / s; z = (R[2] + R[6]) / s;
  } else if (R[4] > R[8]) {
    const s = Math.sqrt(1 + R[4] - R[0] - R[8]) * 2;
    w = (R[2] - R[6]) / s; x = (R[1] + R[3]) / s; y = 0.25 * s; z = (R[5] + R[7]) / s;
  } else {
    const s = Math.sqrt(1 + R[8] - R[0] - R[4]) * 2;
    w = (R[3] - R[1]) / s; x = (R[2] + R[6]) / s; y = (R[5] + R[7]) / s; z = 0.25 * s;
  }
  return quatNormalize([w, x, y, z]);
}

export function quatToMat3(q) {
  const [w, x, y, z] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}

export function quatNormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

export function quatMul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

/** Interpolation sphérique (SLERP) entre deux quaternions, t ∈ [0, 1]. */
export function quatSlerp(a, b, t) {
  let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  let bb = b;
  if (d < 0) { d = -d; bb = [-b[0], -b[1], -b[2], -b[3]]; }
  if (d > 0.9995) {
    return quatNormalize([
      a[0] + (bb[0] - a[0]) * t, a[1] + (bb[1] - a[1]) * t,
      a[2] + (bb[2] - a[2]) * t, a[3] + (bb[3] - a[3]) * t,
    ]);
  }
  const th = Math.acos(clamp(d, -1, 1));
  const s = Math.sin(th);
  const wa = Math.sin((1 - t) * th) / s, wb = Math.sin(t * th) / s;
  return [
    a[0] * wa + bb[0] * wb, a[1] * wa + bb[1] * wb,
    a[2] * wa + bb[2] * wb, a[3] * wa + bb[3] * wb,
  ];
}

// ---------------------------------------------------------------- Mat4 (homogène)
export const mat4Identity = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

export function mat4Mul(A, B) {
  const C = new Array(16);
  for (let r = 0; r < 4; r++) {
    const a0 = A[r * 4], a1 = A[r * 4 + 1], a2 = A[r * 4 + 2], a3 = A[r * 4 + 3];
    C[r * 4] = a0 * B[0] + a1 * B[4] + a2 * B[8] + a3 * B[12];
    C[r * 4 + 1] = a0 * B[1] + a1 * B[5] + a2 * B[9] + a3 * B[13];
    C[r * 4 + 2] = a0 * B[2] + a1 * B[6] + a2 * B[10] + a3 * B[14];
    C[r * 4 + 3] = a0 * B[3] + a1 * B[7] + a2 * B[11] + a3 * B[15];
  }
  return C;
}

/** Produit d'une liste de transformations. */
export const mat4Chain = (...Ts) => Ts.reduce((acc, T) => mat4Mul(acc, T), mat4Identity());

/** Inverse d'une transformation rigide (rotation + translation). */
export function mat4InvRigid(T) {
  const R = mat4Rot(T), p = mat4Pos(T);
  const Rt = mat3T(R);
  const pi = mat3MulVec(Rt, p);
  return mat4FromRotPos(Rt, [-pi[0], -pi[1], -pi[2]]);
}

export const mat4Rot = (T) => [T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]];
export const mat4Pos = (T) => [T[3], T[7], T[11]];

export function mat4FromRotPos(R, p) {
  return [R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1];
}

export const mat4Transl = (x, y, z) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
export const mat4Rx = (a) => mat4FromRotPos(mat3Rx(a), [0, 0, 0]);
export const mat4Ry = (a) => mat4FromRotPos(mat3Ry(a), [0, 0, 0]);
export const mat4Rz = (a) => mat4FromRotPos(mat3Rz(a), [0, 0, 0]);

/** Transformation depuis une pose {x, y, z, roll, pitch, yaw} (m, rad). */
export function poseToMat4(p) {
  return mat4FromRotPos(rpyToMat3(p.roll || 0, p.pitch || 0, p.yaw || 0), [p.x || 0, p.y || 0, p.z || 0]);
}

/** Pose {x, y, z, roll, pitch, yaw} depuis une transformation. */
export function mat4ToPose(T) {
  const [roll, pitch, yaw] = mat3ToRpy(mat4Rot(T));
  return { x: T[3], y: T[7], z: T[11], roll, pitch, yaw };
}

export function mat4TransformPoint(T, p) {
  return [
    T[0] * p[0] + T[1] * p[1] + T[2] * p[2] + T[3],
    T[4] * p[0] + T[5] * p[1] + T[6] * p[2] + T[7],
    T[8] * p[0] + T[9] * p[1] + T[10] * p[2] + T[11],
  ];
}

export function mat4TransformDir(T, d) {
  return [
    T[0] * d[0] + T[1] * d[1] + T[2] * d[2],
    T[4] * d[0] + T[5] * d[1] + T[6] * d[2],
    T[8] * d[0] + T[9] * d[1] + T[10] * d[2],
  ];
}

/** Axe (colonne) k ∈ {0: x, 1: y, 2: z} d'une transformation. */
export const mat4Axis = (T, k) => [T[k], T[4 + k], T[8 + k]];

/**
 * Transformation élémentaire Denavit-Hartenberg standard (classique) :
 *   A = Rz(θ) · Tz(d) · Tx(a) · Rx(α)
 */
export function dhStandard(a, alpha, d, theta) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    ct, -st * ca, st * sa, a * ct,
    st, ct * ca, -ct * sa, a * st,
    0, sa, ca, d,
    0, 0, 0, 1,
  ];
}

/**
 * Transformation élémentaire Denavit-Hartenberg modifiée (Craig) :
 *   A = Rx(α_{i-1}) · Tx(a_{i-1}) · Rz(θ_i) · Tz(d_i)
 */
export function dhModified(a, alpha, d, theta) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    ct, -st, 0, a,
    st * ca, ct * ca, -sa, -sa * d,
    st * sa, ct * sa, ca, ca * d,
    0, 0, 0, 1,
  ];
}

/** Convertit une Mat4 row-major en tableau column-major (format Three.js Matrix4.elements). */
export function mat4ToColumnMajor(T) {
  return [
    T[0], T[4], T[8], T[12],
    T[1], T[5], T[9], T[13],
    T[2], T[6], T[10], T[14],
    T[3], T[7], T[11], T[15],
  ];
}

/** Interpolation de pose : position linéaire + orientation SLERP. */
export function mat4Interp(Ta, Tb, t) {
  const pa = mat4Pos(Ta), pb = mat4Pos(Tb);
  const q = quatSlerp(mat3ToQuat(mat4Rot(Ta)), mat3ToQuat(mat4Rot(Tb)), t);
  return mat4FromRotPos(quatToMat3(q), [lerp(pa[0], pb[0], t), lerp(pa[1], pb[1], t), lerp(pa[2], pb[2], t)]);
}

/** Matrice antisymétrique [v]× telle que [v]× · w = v × w. */
export const skew = (v) => [0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0];

/** Tenseur d'inertie 3×3 (row-major) depuis [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]. */
export function inertiaFromVector(I) {
  const [xx, yy, zz, xy, xz, yz] = I;
  return [xx, xy, xz, xy, yy, yz, xz, yz, zz];
}

export const inertiaToVector = (I) => [I[0], I[4], I[8], I[1], I[2], I[5]];

/** Rotation d'un tenseur d'inertie : R · I · Rᵀ. */
export const rotateInertia = (R, I) => mat3Mul(mat3Mul(R, I), mat3T(R));

/** Théorème de Huygens (axes parallèles) : inertie au point décalé de r par rapport au CdG. */
export function parallelAxis(I, m, r) {
  const [x, y, z] = r;
  const rr = x * x + y * y + z * z;
  return [
    I[0] + m * (rr - x * x), I[1] - m * x * y, I[2] - m * x * z,
    I[3] - m * y * x, I[4] + m * (rr - y * y), I[5] - m * y * z,
    I[6] - m * z * x, I[7] - m * z * y, I[8] + m * (rr - z * z),
  ];
}

export const mat3Add = (A, B) => A.map((v, i) => v + B[i]);
export const mat3Scale = (A, s) => A.map((v) => v * s);
