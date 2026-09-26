// ORION-6 — Algèbre linéaire dense pour petites matrices (6×6, 6×N).
// Les matrices sont des tableaux de lignes : A[i][j].

export function zeros(r, c) {
  const A = new Array(r);
  for (let i = 0; i < r; i++) A[i] = new Array(c).fill(0);
  return A;
}

export function identity(n) {
  const A = zeros(n, n);
  for (let i = 0; i < n; i++) A[i][i] = 1;
  return A;
}

export const clone = (A) => A.map((row) => row.slice());

export function transpose(A) {
  const r = A.length, c = A[0].length;
  const T = zeros(c, r);
  for (let i = 0; i < r; i++) for (let j = 0; j < c; j++) T[j][i] = A[i][j];
  return T;
}

export function matMul(A, B) {
  const r = A.length, n = B.length, c = B[0].length;
  const C = zeros(r, c);
  for (let i = 0; i < r; i++) {
    const Ai = A[i], Ci = C[i];
    for (let k = 0; k < n; k++) {
      const a = Ai[k];
      if (a === 0) continue;
      const Bk = B[k];
      for (let j = 0; j < c; j++) Ci[j] += a * Bk[j];
    }
  }
  return C;
}

export function matVec(A, x) {
  const r = A.length, y = new Array(r);
  for (let i = 0; i < r; i++) {
    let s = 0;
    const Ai = A[i];
    for (let j = 0; j < x.length; j++) s += Ai[j] * x[j];
    y[i] = s;
  }
  return y;
}

export const vecNorm = (x) => Math.sqrt(x.reduce((s, v) => s + v * v, 0));
export const vecAdd = (a, b) => a.map((v, i) => v + b[i]);
export const vecSub = (a, b) => a.map((v, i) => v - b[i]);
export const vecScale = (a, s) => a.map((v) => v * s);
export const vecDot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);

/** Résout A·x = b par élimination de Gauss avec pivot partiel. Retourne null si singulière. */
export function solve(A, b) {
  const n = A.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    let piv = col, best = Math.abs(M[col][col]);
    for (let r = col + 1; r < n; r++) {
      const v = Math.abs(M[r][col]);
      if (v > best) { best = v; piv = r; }
    }
    if (best < 1e-14) return null;
    if (piv !== col) { const t = M[piv]; M[piv] = M[col]; M[col] = t; }
    const Mc = M[col], d = Mc[col];
    for (let r = col + 1; r < n; r++) {
      const Mr = M[r], f = Mr[col] / d;
      if (f === 0) continue;
      for (let c = col; c <= n; c++) Mr[c] -= f * Mc[c];
    }
  }
  const x = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let s = M[i][n];
    for (let j = i + 1; j < n; j++) s -= M[i][j] * x[j];
    x[i] = s / M[i][i];
  }
  return x;
}

/** Décomposition de Cholesky (A symétrique définie positive) → L triangulaire inférieure, ou null. */
export function cholesky(A) {
  const n = A.length;
  const L = zeros(n, n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let s = A[i][j];
      for (let k = 0; k < j; k++) s -= L[i][k] * L[j][k];
      if (i === j) {
        if (s <= 0) return null;
        L[i][i] = Math.sqrt(s);
      } else {
        L[i][j] = s / L[j][j];
      }
    }
  }
  return L;
}

export function choleskySolveL(L, b) {
  const n = L.length, y = new Array(n), x = new Array(n);
  for (let i = 0; i < n; i++) {
    let s = b[i];
    for (let k = 0; k < i; k++) s -= L[i][k] * y[k];
    y[i] = s / L[i][i];
  }
  for (let i = n - 1; i >= 0; i--) {
    let s = y[i];
    for (let k = i + 1; k < n; k++) s -= L[k][i] * x[k];
    x[i] = s / L[i][i];
  }
  return x;
}

/** Résout un système symétrique défini positif (Cholesky, repli sur Gauss). */
export function solveSPD(A, b) {
  const L = cholesky(A);
  return L ? choleskySolveL(L, b) : solve(A, b);
}

export function inverse(A) {
  const n = A.length;
  const inv = zeros(n, n);
  for (let j = 0; j < n; j++) {
    const e = new Array(n).fill(0);
    e[j] = 1;
    const col = solve(A, e);
    if (!col) return null;
    for (let i = 0; i < n; i++) inv[i][j] = col[i];
  }
  return inv;
}

export function det(A) {
  const n = A.length;
  const M = clone(A);
  let d = 1;
  for (let col = 0; col < n; col++) {
    let piv = col, best = Math.abs(M[col][col]);
    for (let r = col + 1; r < n; r++) {
      if (Math.abs(M[r][col]) > best) { best = Math.abs(M[r][col]); piv = r; }
    }
    if (best === 0) return 0;
    if (piv !== col) { const t = M[piv]; M[piv] = M[col]; M[col] = t; d = -d; }
    d *= M[col][col];
    for (let r = col + 1; r < n; r++) {
      const f = M[r][col] / M[col][col];
      for (let c = col; c < n; c++) M[r][c] -= f * M[col][c];
    }
  }
  return d;
}

/**
 * Valeurs/vecteurs propres d'une matrice symétrique (méthode de Jacobi cyclique).
 * Retourne { values: [...] (triées décroissantes), vectors: colonnes associées (V[i][k]) }.
 */
export function symEig(A, maxSweeps = 60) {
  const n = A.length;
  const a = clone(A);
  const V = identity(n);
  for (let sweep = 0; sweep < maxSweeps; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += a[p][q] * a[p][q];
    if (off < 1e-22) break;
    for (let p = 0; p < n - 1; p++) {
      for (let q = p + 1; q < n; q++) {
        const apq = a[p][q];
        if (Math.abs(apq) < 1e-300) continue;
        const theta = (a[q][q] - a[p][p]) / (2 * apq);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1), s = t * c;
        for (let k = 0; k < n; k++) {
          const akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < n; k++) {
          const vkp = V[k][p], vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  const idx = [...Array(n).keys()].sort((i, j) => a[j][j] - a[i][i]);
  return {
    values: idx.map((i) => a[i][i]),
    vectors: V.map((row) => idx.map((i) => row[i])),
  };
}

/** Valeurs singulières de J (m×n), triées décroissantes (via valeurs propres de J·Jᵀ). */
export function singularValues(J) {
  const JJt = matMul(J, transpose(J));
  return symEig(JJt).values.map((v) => Math.sqrt(Math.max(v, 0)));
}

/**
 * Moindres carrés amortis (Levenberg-Marquardt / Nakamura) :
 *   dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
 * `w` : poids optionnels par ligne de la tâche (m valeurs).
 */
export function dampedLeastSquares(J, e, lambda, w = null) {
  const m = J.length;
  const Jw = w ? J.map((row, i) => row.map((v) => v * w[i])) : J;
  const ew = w ? e.map((v, i) => v * w[i]) : e;
  const A = matMul(Jw, transpose(Jw));
  for (let i = 0; i < m; i++) A[i][i] += lambda * lambda;
  const y = solveSPD(A, ew);
  if (!y) return null;
  return matVec(transpose(Jw), y);
}
