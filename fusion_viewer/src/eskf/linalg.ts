/** Small dense row-major matrix helpers for the error-state covariance. */

export function zeros(rows: number, cols: number = rows): Float64Array {
  return new Float64Array(rows * cols);
}

export function identity(n: number): Float64Array {
  const m = new Float64Array(n * n);
  for (let i = 0; i < n; i++) m[i * n + i] = 1;
  return m;
}

/** C = A (r×k) · B (k×c). */
export function matMul(a: Float64Array, b: Float64Array, r: number, k: number, c: number): Float64Array {
  const out = new Float64Array(r * c);
  for (let i = 0; i < r; i++) {
    for (let p = 0; p < k; p++) {
      const aip = a[i * k + p];
      if (aip === 0) continue;
      for (let j = 0; j < c; j++) out[i * c + j] += aip * b[p * c + j];
    }
  }
  return out;
}

export function transpose(a: Float64Array, rows: number, cols: number): Float64Array {
  const out = new Float64Array(rows * cols);
  for (let i = 0; i < rows; i++) {
    for (let j = 0; j < cols; j++) out[j * rows + i] = a[i * cols + j];
  }
  return out;
}

export function addScaled(a: Float64Array, b: Float64Array, s: number = 1): Float64Array {
  const out = new Float64Array(a.length);
  for (let i = 0; i < a.length; i++) out[i] = a[i] + s * b[i];
  return out;
}

/** Writes a 3×3 row-major block, multiplied by scale, into an n-column matrix at (row, col). */
export function setBlock3(m: Float64Array, cols: number, row: number, col: number, block: ArrayLike<number>, scale: number = 1): void {
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) m[(row + r) * cols + col + c] = block[r * 3 + c] * scale;
  }
}

export function getBlock3(m: Float64Array, cols: number, row: number, col: number): number[] {
  const out: number[] = [];
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) out.push(m[(row + r) * cols + col + c]);
  }
  return out;
}

/** Returns (A + Aᵀ)/2 for a square matrix. */
export function symmetrize(a: Float64Array, n: number): Float64Array {
  const out = new Float64Array(n * n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) out[i * n + j] = 0.5 * (a[i * n + j] + a[j * n + i]);
  }
  return out;
}

/** Lower-triangular Cholesky factor, or null if the matrix is not positive definite. */
export function cholesky(a: Float64Array, n: number): Float64Array | null {
  const l = new Float64Array(n * n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let sum = a[i * n + j];
      for (let k = 0; k < j; k++) sum -= l[i * n + k] * l[j * n + k];
      if (i === j) {
        if (!(sum > 0)) return null;
        l[i * n + i] = Math.sqrt(sum);
      } else {
        l[i * n + j] = sum / l[j * n + j];
      }
    }
  }
  return l;
}

export interface CovarianceHealth {
  finite: boolean;
  maxAsymmetry: number;
  positiveDefinite: boolean;
}

export function covarianceHealth(p: Float64Array, n: number): CovarianceHealth {
  let finite = true;
  let maxAsymmetry = 0;
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      const v = p[i * n + j];
      if (!Number.isFinite(v)) finite = false;
      maxAsymmetry = Math.max(maxAsymmetry, Math.abs(v - p[j * n + i]));
    }
  }
  return { finite, maxAsymmetry, positiveDefinite: finite && cholesky(p, n) !== null };
}
