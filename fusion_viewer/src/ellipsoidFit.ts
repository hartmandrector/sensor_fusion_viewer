/**
 * Ellipsoid Fitting for Magnetometer Calibration
 * 
 * Implements full 9-parameter ellipsoid fitting to extract:
 * - Hard iron offset (3 parameters): center of ellipsoid
 * - Soft iron correction (6 parameters): symmetric matrix transforming ellipsoid to sphere
 * - Plus implicit rotation if sensor axes are misaligned
 * 
 * The magnetic field distortion model:
 *   m_measured = A * m_true + b
 * 
 * Where A is soft iron (3x3) and b is hard iron offset (3x1).
 * 
 * Calibration transforms:
 *   m_corrected = W * (m_measured - V)
 * 
 * Where V is hard iron offset and W is soft iron correction matrix.
 * 
 * @license MIT
 */

// ============================================================================
// Types
// ============================================================================

export interface EllipsoidFitResult {
  /** Hard iron offset (center of ellipsoid) */
  hardIronOffset: { x: number; y: number; z: number };
  
  /** Soft iron correction matrix (transforms ellipsoid to sphere) */
  softIronMatrix: number[][];  // 3x3 matrix
  
  /** Inverse of soft iron matrix (for applying correction) */
  softIronInverse: number[][];  // 3x3 matrix
  
  /** Combined calibration: corrected = softIronInverse * (raw - hardIronOffset) */
  
  /** Eigenvalues of the ellipsoid shape matrix (axis lengths) */
  eigenvalues: { a: number; b: number; c: number };
  
  /** Eigenvectors of the ellipsoid shape matrix (axis directions) */
  eigenvectors: number[][];  // 3x3, each column is an eigenvector
  
  /** True sphericity: ratio of min to max eigenvalue (1.0 = perfect sphere) */
  sphericity: number;
  
  /** RMS residual error after fitting */
  residualRms: number;
  
  /** Number of samples used */
  sampleCount: number;
  
  /** Fit quality indicator (0-100%) */
  quality: number;
}

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

// ============================================================================
// Matrix Math Helpers
// ============================================================================

/**
 * Create a zero matrix
 */
function zeros(rows: number, cols: number): number[][] {
  return Array(rows).fill(null).map(() => Array(cols).fill(0));
}

/**
 * Create identity matrix
 */
function eye(n: number): number[][] {
  const m = zeros(n, n);
  for (let i = 0; i < n; i++) m[i][i] = 1;
  return m;
}

/**
 * Matrix multiplication: A * B
 */
function matmul(A: number[][], B: number[][]): number[][] {
  const m = A.length;
  const n = B[0].length;
  const p = B.length;
  const C = zeros(m, n);
  
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < n; j++) {
      let sum = 0;
      for (let k = 0; k < p; k++) {
        sum += A[i][k] * B[k][j];
      }
      C[i][j] = sum;
    }
  }
  return C;
}

/**
 * Matrix transpose
 */
function transpose(A: number[][]): number[][] {
  const m = A.length;
  const n = A[0].length;
  const T = zeros(n, m);
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < n; j++) {
      T[j][i] = A[i][j];
    }
  }
  return T;
}

/**
 * Matrix-vector multiplication: A * v
 */
function matvec(A: number[][], v: number[]): number[] {
  const m = A.length;
  const result = Array(m).fill(0);
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < A[i].length; j++) {
      result[i] += A[i][j] * v[j];
    }
  }
  return result;
}

/**
 * 3x3 matrix determinant
 */
function det3x3(M: number[][]): number {
  return (
    M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
    M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
    M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])
  );
}

/**
 * 3x3 matrix inverse
 */
function inverse3x3(M: number[][]): number[][] {
  const det = det3x3(M);
  if (Math.abs(det) < 1e-10) {
    throw new Error('Matrix is singular, cannot invert');
  }
  
  const inv = zeros(3, 3);
  inv[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / det;
  inv[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / det;
  inv[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / det;
  inv[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) / det;
  inv[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / det;
  inv[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) / det;
  inv[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / det;
  inv[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) / det;
  inv[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / det;
  
  return inv;
}

/**
 * Solve linear system Ax = b using Gaussian elimination with partial pivoting
 */
function solve(A: number[][], b: number[]): number[] {
  const n = A.length;
  
  // Augmented matrix
  const aug = A.map((row, i) => [...row, b[i]]);
  
  // Forward elimination with partial pivoting
  for (let col = 0; col < n; col++) {
    // Find pivot
    let maxRow = col;
    let maxVal = Math.abs(aug[col][col]);
    for (let row = col + 1; row < n; row++) {
      if (Math.abs(aug[row][col]) > maxVal) {
        maxVal = Math.abs(aug[row][col]);
        maxRow = row;
      }
    }
    
    // Swap rows
    [aug[col], aug[maxRow]] = [aug[maxRow], aug[col]];
    
    // Check for singular matrix
    if (Math.abs(aug[col][col]) < 1e-10) {
      throw new Error('Matrix is singular');
    }
    
    // Eliminate column
    for (let row = col + 1; row < n; row++) {
      const factor = aug[row][col] / aug[col][col];
      for (let j = col; j <= n; j++) {
        aug[row][j] -= factor * aug[col][j];
      }
    }
  }
  
  // Back substitution
  const x = Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let sum = aug[i][n];
    for (let j = i + 1; j < n; j++) {
      sum -= aug[i][j] * x[j];
    }
    x[i] = sum / aug[i][i];
  }
  
  return x;
}

/**
 * Compute eigenvalues and eigenvectors of a 3x3 symmetric matrix
 * Uses Jacobi iteration method
 */
function symmetricEigen3x3(A: number[][]): { values: number[]; vectors: number[][] } {
  const n = 3;
  const maxIter = 100;
  const tol = 1e-10;
  
  // Copy matrix
  const D = A.map(row => [...row]);
  const V = eye(n);
  
  for (let iter = 0; iter < maxIter; iter++) {
    // Find largest off-diagonal element
    let maxVal = 0;
    let p = 0, q = 1;
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        if (Math.abs(D[i][j]) > maxVal) {
          maxVal = Math.abs(D[i][j]);
          p = i;
          q = j;
        }
      }
    }
    
    if (maxVal < tol) break;
    
    // Compute rotation angle
    const theta = (D[q][q] - D[p][p]) / (2 * D[p][q]);
    const t = Math.sign(theta) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
    const c = 1 / Math.sqrt(t * t + 1);
    const s = t * c;
    
    // Apply rotation to D
    const Dpp = D[p][p];
    const Dqq = D[q][q];
    const Dpq = D[p][q];
    
    D[p][p] = c * c * Dpp - 2 * s * c * Dpq + s * s * Dqq;
    D[q][q] = s * s * Dpp + 2 * s * c * Dpq + c * c * Dqq;
    D[p][q] = 0;
    D[q][p] = 0;
    
    for (let i = 0; i < n; i++) {
      if (i !== p && i !== q) {
        const Dip = D[i][p];
        const Diq = D[i][q];
        D[i][p] = c * Dip - s * Diq;
        D[p][i] = D[i][p];
        D[i][q] = s * Dip + c * Diq;
        D[q][i] = D[i][q];
      }
    }
    
    // Apply rotation to V
    for (let i = 0; i < n; i++) {
      const Vip = V[i][p];
      const Viq = V[i][q];
      V[i][p] = c * Vip - s * Viq;
      V[i][q] = s * Vip + c * Viq;
    }
  }
  
  // Extract eigenvalues and sort
  const values = [D[0][0], D[1][1], D[2][2]];
  const vectors = transpose(V);  // Each row is an eigenvector
  
  // Sort by eigenvalue (descending)
  const indices = [0, 1, 2].sort((a, b) => values[b] - values[a]);
  
  return {
    values: indices.map(i => values[i]),
    vectors: indices.map(i => vectors[i])
  };
}

// ============================================================================
// Ellipsoid Fitting Algorithm
// ============================================================================

/**
 * Fit an ellipsoid to 3D points using least squares
 * 
 * General ellipsoid equation:
 *   ax² + by² + cz² + 2fyz + 2gxz + 2hxy + 2px + 2qy + 2rz + d = 0
 * 
 * We solve for [a, b, c, f, g, h, p, q, r, d] with constraint a + b + c = 3
 * (ensures ellipsoid, not hyperboloid)
 * 
 * Matrix form: x^T * M * x + 2 * n^T * x + d = 0
 * Where M = [[a, h, g], [h, b, f], [g, f, c]] and n = [p, q, r]
 */
export function fitEllipsoid(points: Vector3[]): EllipsoidFitResult {
  const n = points.length;
  
  if (n < 10) {
    throw new Error('Need at least 10 points for ellipsoid fitting');
  }
  
  // Build design matrix D where each row is [x², y², z², 2yz, 2xz, 2xy, 2x, 2y, 2z, 1]
  // We'll use the algebraic method with constraint trace(M) = 1
  
  const D: number[][] = [];
  for (const p of points) {
    D.push([
      p.x * p.x,
      p.y * p.y,
      p.z * p.z,
      2 * p.y * p.z,
      2 * p.x * p.z,
      2 * p.x * p.y,
      2 * p.x,
      2 * p.y,
      2 * p.z,
      1
    ]);
  }
  
  // Solve using constrained least squares
  // We want to minimize ||D * v||² subject to v^T * C * v = 1
  // where C enforces the ellipsoid constraint
  
  // Compute D^T * D
  const DtD = zeros(10, 10);
  for (let i = 0; i < 10; i++) {
    for (let j = 0; j < 10; j++) {
      let sum = 0;
      for (let k = 0; k < n; k++) {
        sum += D[k][i] * D[k][j];
      }
      DtD[i][j] = sum;
    }
  }
  
  // Use a simpler approach: assume d = -1 (ellipsoid = 1), solve linear system
  // Rearrange: ax² + by² + cz² + 2fyz + 2gxz + 2hxy + 2px + 2qy + 2rz = 1
  
  const D2: number[][] = [];
  const b: number[] = [];
  for (const p of points) {
    D2.push([
      p.x * p.x,
      p.y * p.y,
      p.z * p.z,
      2 * p.y * p.z,
      2 * p.x * p.z,
      2 * p.x * p.y,
      2 * p.x,
      2 * p.y,
      2 * p.z
    ]);
    b.push(1);
  }
  
  // Solve least squares: (D2^T * D2) * v = D2^T * b
  const D2t = transpose(D2);
  const D2tD2 = matmul(D2t, D2);
  const D2tb = matvec(D2t, b);
  
  // Add regularization to prevent singular matrix
  for (let i = 0; i < 9; i++) {
    D2tD2[i][i] += 1e-6;
  }
  
  const v = solve(D2tD2, D2tb);
  
  // Extract parameters
  // v = [a, b, c, f, g, h, p, q, r]
  const [a, bCoef, c, f, g, h, px, qy, rz] = v;
  
  // Build matrices
  // M = [[a, h, g], [h, b, f], [g, f, c]]
  const M: number[][] = [
    [a, h, g],
    [h, bCoef, f],
    [g, f, c]
  ];
  
  // n = [p, q, r]
  const nVec = [px, qy, rz];
  
  // Compute center: center = -M^(-1) * n
  const Minv = inverse3x3(M);
  const center = matvec(Minv, nVec.map(x => -x));
  
  // Compute eigenvalues of M (shape of ellipsoid)
  const eigen = symmetricEigen3x3(M);
  
  // Debug: log intermediate values
  console.log('Ellipsoid fit debug:');
  console.log('  M matrix:', M);
  console.log('  Eigenvalues of M:', eigen.values);
  console.log('  Center:', center);
  
  // The eigenvalues determine the ellipsoid shape
  // If all eigenvalues have the same sign, we have an ellipsoid
  // Semi-axes are proportional to 1/sqrt(|eigenvalue|)
  
  // For the standard form, we need: (x-c)^T * M * (x-c) = k
  // Where k = n^T * M^(-1) * n - d  (with d = -1 from our equation = 1)
  // Simplified: k = -center^T * n + 1 = center^T * M * center + 1
  const Mc = matvec(M, center);
  const k = center[0] * Mc[0] + center[1] * Mc[1] + center[2] * Mc[2] + 1;
  
  console.log('  k value:', k);
  
  // For a proper ellipsoid centered at 'center', the eigenvalues of M 
  // should all have the same sign as k for the semi-axes to be real.
  // Semi-axis for eigenvalue λ is sqrt(|k/λ|)
  
  // Check if all eigenvalues have the same sign
  const allNegative = eigen.values.every(e => e < 0);
  const allPositive = eigen.values.every(e => e > 0);
  
  let semiAxes: number[];
  if (allNegative && k < 0) {
    // Valid ellipsoid with negative eigenvalues and negative k
    semiAxes = eigen.values.map(ev => Math.sqrt(Math.abs(k / ev)));
  } else if (allPositive && k > 0) {
    // Valid ellipsoid with positive eigenvalues and positive k
    semiAxes = eigen.values.map(ev => Math.sqrt(k / ev));
  } else {
    // Mixed signs or sign mismatch - still compute but flag issue
    console.warn('  Warning: eigenvalue/k sign mismatch, computing approximate axes');
    semiAxes = eigen.values.map(ev => {
      const absEv = Math.abs(ev);
      const absK = Math.abs(k);
      return absEv > 0 ? Math.sqrt(absK / absEv) : 0;
    });
  }
  
  console.log('  Semi-axes (unsorted):', semiAxes);
  
  // For sphericity calculation, find min and max
  const validAxes = semiAxes.filter(a => a > 0 && isFinite(a));
  const minAxis = validAxes.length > 0 ? Math.min(...validAxes) : 0;
  const maxAxis = validAxes.length > 0 ? Math.max(...validAxes) : 0;
  const sphericity = maxAxis > 0 && minAxis > 0 ? minAxis / maxAxis : 0;
  
  console.log('  Sphericity:', sphericity);
  
  // Soft iron correction matrix
  // We want to transform the ellipsoid to a unit sphere
  // The correction is: W^(-1) * (x - center) where W transforms sphere to ellipsoid
  // W = V * diag(semiAxes) * V^T (eigenvectors and scaled eigenvalues)
  
  // Build the transformation matrix using the eigenvectors (keeping association)
  const V = transpose(eigen.vectors);  // Columns are eigenvectors
  
  // Scale matrix (diagonal with semi-axes - keep original ordering to match eigenvectors)
  const S = zeros(3, 3);
  S[0][0] = semiAxes[0] > 0 && isFinite(semiAxes[0]) ? semiAxes[0] : 1;
  S[1][1] = semiAxes[1] > 0 && isFinite(semiAxes[1]) ? semiAxes[1] : 1;
  S[2][2] = semiAxes[2] > 0 && isFinite(semiAxes[2]) ? semiAxes[2] : 1;
  
  // Soft iron matrix: W = V * S * V^T
  const softIronMatrix = matmul(matmul(V, S), transpose(V));
  
  // Inverse for correction: W^(-1) = V * S^(-1) * V^T
  const Sinv = zeros(3, 3);
  Sinv[0][0] = semiAxes[0] > 0 && isFinite(semiAxes[0]) ? 1 / semiAxes[0] : 1;
  Sinv[1][1] = semiAxes[1] > 0 && isFinite(semiAxes[1]) ? 1 / semiAxes[1] : 1;
  Sinv[2][2] = semiAxes[2] > 0 && isFinite(semiAxes[2]) ? 1 / semiAxes[2] : 1;
  
  const softIronInverse = matmul(matmul(V, Sinv), transpose(V));
  
  // Compute residual RMS
  let residualSum = 0;
  for (const p of points) {
    // Apply correction
    const shifted = [p.x - center[0], p.y - center[1], p.z - center[2]];
    const corrected = matvec(softIronInverse, shifted);
    
    // Distance from unit sphere
    const radius = Math.sqrt(corrected[0] ** 2 + corrected[1] ** 2 + corrected[2] ** 2);
    residualSum += (radius - 1) ** 2;
  }
  const residualRms = Math.sqrt(residualSum / n);
  
  // Quality metric (based on sphericity and residual)
  const quality = Math.max(0, Math.min(100, 
    sphericity * 100 * Math.exp(-residualRms * 10)
  ));
  
  return {
    hardIronOffset: { x: center[0], y: center[1], z: center[2] },
    softIronMatrix,
    softIronInverse,
    eigenvalues: { a: semiAxes[0], b: semiAxes[1], c: semiAxes[2] },
    eigenvectors: eigen.vectors,
    sphericity,
    residualRms,
    sampleCount: n,
    quality
  };
}

/**
 * Apply ellipsoid calibration to a magnetometer reading
 */
export function applyEllipsoidCalibration(
  raw: Vector3,
  calibration: EllipsoidFitResult
): Vector3 {
  const shifted = [
    raw.x - calibration.hardIronOffset.x,
    raw.y - calibration.hardIronOffset.y,
    raw.z - calibration.hardIronOffset.z
  ];
  
  const corrected = matvec(calibration.softIronInverse, shifted);
  
  return { x: corrected[0], y: corrected[1], z: corrected[2] };
}

/**
 * Format soft iron matrix for display
 */
export function formatSoftIronMatrix(matrix: number[][]): string {
  return matrix.map(row => 
    row.map(v => v.toFixed(4)).join('  ')
  ).join('\n');
}
