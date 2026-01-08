/**
 * 6-Position Accelerometer Calibration
 * 
 * Implements full 3x3 accelerometer calibration using 6 static positions.
 * 
 * The calibration model:
 *   a_measured = S * a_true + b
 * 
 * Where:
 *   S = 3x3 matrix (scale factors + cross-axis coupling)
 *   b = 3x1 bias vector
 * 
 * For each position, we know the true gravity direction and solve for S and b.
 * 
 * The 6 positions are:
 *   +X up: gravity = [-1, 0, 0]
 *   -X up: gravity = [+1, 0, 0]
 *   +Y up: gravity = [0, -1, 0]
 *   -Y up: gravity = [0, +1, 0]
 *   +Z up: gravity = [0, 0, -1]
 *   -Z up: gravity = [0, 0, +1]
 * 
 * @license MIT
 */

// ============================================================================
// Types
// ============================================================================

export type AxisOrientation = '+X' | '-X' | '+Y' | '-Y' | '+Z' | '-Z';

export interface StationarySegment {
  /** Start index in data array */
  startIndex: number;
  
  /** End index in data array */
  endIndex: number;
  
  /** Duration in seconds */
  duration: number;
  
  /** Average acceleration during segment */
  avgAccel: { x: number; y: number; z: number };
  
  /** Standard deviation of acceleration during segment */
  stdDev: { x: number; y: number; z: number };
  
  /** Average gyroscope reading during segment (deg/s) - this IS the gyro bias */
  avgGyro: { x: number; y: number; z: number };
  
  /** Standard deviation of gyroscope during segment */
  gyroStdDev: { x: number; y: number; z: number };
  
  /** Detected dominant axis orientation */
  orientation: AxisOrientation;
  
  /** Angle from ideal axis (degrees) - quality indicator */
  angleFromAxis: number;
  
  /** Number of samples */
  sampleCount: number;
  
  /** Whether this segment meets quality criteria */
  isValid: boolean;
}

export interface AccelCalibration6PosResult {
  /** Scale and cross-axis matrix (3x3) */
  scaleMatrix: number[][];
  
  /** Inverse of scale matrix (for applying correction) */
  scaleMatrixInverse: number[][];
  
  /** Accel bias vector */
  bias: { x: number; y: number; z: number };
  
  /** Gyro bias vector (deg/s) - calculated from stationary segments */
  gyroBias: { x: number; y: number; z: number };
  
  /** Gyro bias standard deviation (deg/s) - quality indicator */
  gyroBiasStdDev: { x: number; y: number; z: number };
  
  /** Scale factors extracted from diagonal */
  scaleFactor: { x: number; y: number; z: number };
  
  /** Cross-axis coupling terms */
  crossAxis: {
    xy: number; xz: number;
    yx: number; yz: number;
    zx: number; zy: number;
  };
  
  /** Detected segments for each orientation */
  segments: Map<AxisOrientation, StationarySegment>;
  
  /** RMS residual error */
  residualRms: number;
  
  /** Overall quality (0-100%) */
  quality: number;
  
  /** Warnings/issues detected */
  warnings: string[];
}

export interface AccelSample {
  timestamp: number;
  x: number;
  y: number;
  z: number;
}

export interface CalibrationConfig {
  /** Minimum duration for a valid stationary segment (seconds) */
  minSegmentDuration: number;
  
  /** Maximum angle from ideal axis for valid orientation (degrees) */
  maxAngleFromAxis: number;
  
  /** Gyroscope threshold for stationary detection (deg/s) */
  gyroThreshold: number;
  
  /** Minimum samples per segment */
  minSamplesPerSegment: number;
}

export const DEFAULT_CALIBRATION_CONFIG: CalibrationConfig = {
  minSegmentDuration: 0.5,      // 0.5 seconds minimum
  maxAngleFromAxis: 10,         // 10 degrees tolerance
  gyroThreshold: 3.0,           // 3 deg/s for stationary
  minSamplesPerSegment: 100     // ~0.25s at 416Hz
};

// ============================================================================
// Matrix Math Helpers
// ============================================================================

function zeros(rows: number, cols: number): number[][] {
  return Array(rows).fill(null).map(() => Array(cols).fill(0));
}

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

function det3x3(M: number[][]): number {
  return (
    M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
    M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
    M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])
  );
}

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
 * Solve linear system Ax = b using least squares (pseudoinverse)
 * For overdetermined systems
 */
function solveLeastSquares(A: number[][], b: number[]): number[] {
  // x = (A^T * A)^(-1) * A^T * b
  const At = transpose(A);
  const AtA = matmul(At, A);
  const Atb = matvec(At, b);
  
  // Add regularization
  for (let i = 0; i < AtA.length; i++) {
    AtA[i][i] += 1e-8;
  }
  
  // Solve AtA * x = Atb using Gaussian elimination
  return solve(AtA, Atb);
}

function solve(A: number[][], b: number[]): number[] {
  const n = A.length;
  const aug = A.map((row, i) => [...row, b[i]]);
  
  // Forward elimination with partial pivoting
  for (let col = 0; col < n; col++) {
    let maxRow = col;
    let maxVal = Math.abs(aug[col][col]);
    for (let row = col + 1; row < n; row++) {
      if (Math.abs(aug[row][col]) > maxVal) {
        maxVal = Math.abs(aug[row][col]);
        maxRow = row;
      }
    }
    [aug[col], aug[maxRow]] = [aug[maxRow], aug[col]];
    
    if (Math.abs(aug[col][col]) < 1e-10) {
      throw new Error('Matrix is singular');
    }
    
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

// ============================================================================
// Segment Detection
// ============================================================================

/**
 * True gravity vectors for each orientation (in sensor frame when that axis points up)
 * When +X is up, gravity points in -X direction, etc.
 */
const GRAVITY_VECTORS: Record<AxisOrientation, number[]> = {
  '+X': [-1, 0, 0],
  '-X': [+1, 0, 0],
  '+Y': [0, -1, 0],
  '-Y': [0, +1, 0],
  '+Z': [0, 0, -1],
  '-Z': [0, 0, +1]
};

/**
 * Detect which axis orientation a stationary segment represents
 */
function detectOrientation(avgAccel: { x: number; y: number; z: number }): {
  orientation: AxisOrientation;
  angleFromAxis: number;
} {
  const norm = Math.sqrt(avgAccel.x ** 2 + avgAccel.y ** 2 + avgAccel.z ** 2);
  if (norm < 0.1) {
    return { orientation: '+Z', angleFromAxis: 90 };  // Invalid, no gravity
  }
  
  // Normalize
  const nx = avgAccel.x / norm;
  const ny = avgAccel.y / norm;
  const nz = avgAccel.z / norm;
  
  // Find closest axis
  let bestOrientation: AxisOrientation = '+Z';
  let bestAngle = 180;
  
  for (const [orient, gravity] of Object.entries(GRAVITY_VECTORS) as [AxisOrientation, number[]][]) {
    // Dot product gives cos(angle)
    const dot = nx * gravity[0] + ny * gravity[1] + nz * gravity[2];
    const angle = Math.acos(Math.max(-1, Math.min(1, dot))) * 180 / Math.PI;
    
    if (angle < bestAngle) {
      bestAngle = angle;
      bestOrientation = orient;
    }
  }
  
  return { orientation: bestOrientation, angleFromAxis: bestAngle };
}

/**
 * Detect stationary segments in accelerometer data
 * Uses gyroscope data to detect when device is still
 * Also calculates gyro bias from stationary segments
 */
export function detectStationarySegments(
  accelData: AccelSample[],
  gyroData: { timestamp: number; x: number; y: number; z: number }[],
  config: CalibrationConfig = DEFAULT_CALIBRATION_CONFIG
): StationarySegment[] {
  const segments: StationarySegment[] = [];
  
  if (accelData.length === 0 || gyroData.length === 0) {
    return segments;
  }
  
  // Interpolate gyro data to match accel timestamps
  // Store both magnitude and actual values for gyro bias calculation
  let gyroIndex = 0;
  const interpolatedGyro: { x: number; y: number; z: number; magnitude: number }[] = [];
  
  for (const accel of accelData) {
    // Find closest gyro sample
    while (gyroIndex < gyroData.length - 1 && 
           gyroData[gyroIndex + 1].timestamp <= accel.timestamp) {
      gyroIndex++;
    }
    
    const gyro = gyroData[Math.min(gyroIndex, gyroData.length - 1)];
    const magnitude = Math.sqrt(gyro.x ** 2 + gyro.y ** 2 + gyro.z ** 2);
    interpolatedGyro.push({ x: gyro.x, y: gyro.y, z: gyro.z, magnitude });
  }
  
  // Find stationary segments (gyro below threshold)
  let segmentStart = -1;
  
  for (let i = 0; i < accelData.length; i++) {
    const isStationary = interpolatedGyro[i].magnitude < config.gyroThreshold;
    
    if (isStationary && segmentStart < 0) {
      // Start new segment
      segmentStart = i;
    } else if (!isStationary && segmentStart >= 0) {
      // End segment
      processSegment(segmentStart, i - 1);
      segmentStart = -1;
    }
  }
  
  // Handle segment at end of data
  if (segmentStart >= 0) {
    processSegment(segmentStart, accelData.length - 1);
  }
  
  function processSegment(start: number, end: number): void {
    const duration = accelData[end].timestamp - accelData[start].timestamp;
    const sampleCount = end - start + 1;
    
    if (duration < config.minSegmentDuration || sampleCount < config.minSamplesPerSegment) {
      return;  // Too short
    }
    
    // Calculate accel average and std dev
    let sumAx = 0, sumAy = 0, sumAz = 0;
    for (let i = start; i <= end; i++) {
      sumAx += accelData[i].x;
      sumAy += accelData[i].y;
      sumAz += accelData[i].z;
    }
    const avgAx = sumAx / sampleCount;
    const avgAy = sumAy / sampleCount;
    const avgAz = sumAz / sampleCount;
    
    let varAx = 0, varAy = 0, varAz = 0;
    for (let i = start; i <= end; i++) {
      varAx += (accelData[i].x - avgAx) ** 2;
      varAy += (accelData[i].y - avgAy) ** 2;
      varAz += (accelData[i].z - avgAz) ** 2;
    }
    const stdAx = Math.sqrt(varAx / sampleCount);
    const stdAy = Math.sqrt(varAy / sampleCount);
    const stdAz = Math.sqrt(varAz / sampleCount);
    
    // Calculate gyro average and std dev (this is the gyro bias for this segment)
    let sumGx = 0, sumGy = 0, sumGz = 0;
    for (let i = start; i <= end; i++) {
      sumGx += interpolatedGyro[i].x;
      sumGy += interpolatedGyro[i].y;
      sumGz += interpolatedGyro[i].z;
    }
    const avgGx = sumGx / sampleCount;
    const avgGy = sumGy / sampleCount;
    const avgGz = sumGz / sampleCount;
    
    let varGx = 0, varGy = 0, varGz = 0;
    for (let i = start; i <= end; i++) {
      varGx += (interpolatedGyro[i].x - avgGx) ** 2;
      varGy += (interpolatedGyro[i].y - avgGy) ** 2;
      varGz += (interpolatedGyro[i].z - avgGz) ** 2;
    }
    const stdGx = Math.sqrt(varGx / sampleCount);
    const stdGy = Math.sqrt(varGy / sampleCount);
    const stdGz = Math.sqrt(varGz / sampleCount);
    
    // Detect orientation
    const { orientation, angleFromAxis } = detectOrientation({ x: avgAx, y: avgAy, z: avgAz });
    
    // Check validity
    const isValid = angleFromAxis <= config.maxAngleFromAxis;
    
    segments.push({
      startIndex: start,
      endIndex: end,
      duration,
      avgAccel: { x: avgAx, y: avgAy, z: avgAz },
      stdDev: { x: stdAx, y: stdAy, z: stdAz },
      avgGyro: { x: avgGx, y: avgGy, z: avgGz },
      gyroStdDev: { x: stdGx, y: stdGy, z: stdGz },
      orientation,
      angleFromAxis,
      sampleCount,
      isValid
    });
  }
  
  return segments;
}

/**
 * Select the best segment for each required orientation
 */
export function selectBestSegments(
  segments: StationarySegment[]
): Map<AxisOrientation, StationarySegment> {
  const bestSegments = new Map<AxisOrientation, StationarySegment>();
  
  for (const segment of segments) {
    if (!segment.isValid) continue;
    
    const existing = bestSegments.get(segment.orientation);
    
    // Prefer segment with smaller angle from axis (better alignment)
    // If similar angle, prefer longer segment
    if (!existing || 
        segment.angleFromAxis < existing.angleFromAxis - 1 ||
        (Math.abs(segment.angleFromAxis - existing.angleFromAxis) < 1 && 
         segment.sampleCount > existing.sampleCount)) {
      bestSegments.set(segment.orientation, segment);
    }
  }
  
  return bestSegments;
}

// ============================================================================
// 6-Position Calibration Algorithm
// ============================================================================

/**
 * Perform 6-position accelerometer calibration
 * 
 * The model: a_measured = S * a_true + b
 * 
 * We have 6 positions, each giving us 3 equations:
 *   [ax]   [s11 s12 s13]   [gx]   [bx]
 *   [ay] = [s21 s22 s23] * [gy] + [by]
 *   [az]   [s31 s32 s33]   [gz]   [bz]
 * 
 * That's 18 equations for 12 unknowns (9 scale + 3 bias).
 * We solve using least squares.
 */
export function calibrate6Position(
  segments: Map<AxisOrientation, StationarySegment>
): AccelCalibration6PosResult {
  const warnings: string[] = [];
  
  // Check we have all 6 orientations
  const requiredOrientations: AxisOrientation[] = ['+X', '-X', '+Y', '-Y', '+Z', '-Z'];
  const missingOrientations: AxisOrientation[] = [];
  
  for (const orient of requiredOrientations) {
    if (!segments.has(orient)) {
      missingOrientations.push(orient);
    }
  }
  
  if (missingOrientations.length > 0) {
    warnings.push(`Missing orientations: ${missingOrientations.join(', ')}`);
    
    if (missingOrientations.length > 3) {
      throw new Error(`Need at least 3 orientations for calibration, missing: ${missingOrientations.join(', ')}`);
    }
  }
  
  // Build system of equations
  // For each position: a_measured = S * g_true + b
  // Rearrange: [g_true | I] * [S^T; b^T]^T = a_measured
  // 
  // Actually, easier to solve each axis separately:
  // ax = s11*gx + s12*gy + s13*gz + bx
  // ay = s21*gx + s22*gy + s23*gz + by
  // az = s31*gx + s32*gy + s33*gz + bz
  //
  // For all positions combined:
  // [gx1 gy1 gz1 1 0 0 0 0 0 0 0 0]   [s11]   [ax1]
  // [0 0 0 0 gx1 gy1 gz1 1 0 0 0 0]   [s12]   [ay1]
  // [0 0 0 0 0 0 0 0 gx1 gy1 gz1 1] * [s13] = [az1]
  // ...                                [bx]    ...
  //                                    [s21]
  //                                    [s22]
  //                                    [s23]
  //                                    [by]
  //                                    [s31]
  //                                    [s32]
  //                                    [s33]
  //                                    [bz]
  //
  // Let's use a simpler approach: solve for each row of S and b separately
  
  const positions = Array.from(segments.entries());
  const n = positions.length;
  
  // Build matrices for least squares
  // For row i of S: [gx gy gz 1] * [si1 si2 si3 bi]^T = ai_measured
  
  const G: number[][] = [];  // Design matrix
  const ax: number[] = [];   // X measurements
  const ay: number[] = [];   // Y measurements
  const az: number[] = [];   // Z measurements
  
  for (const [orient, segment] of positions) {
    const g = GRAVITY_VECTORS[orient];
    G.push([g[0], g[1], g[2], 1]);
    ax.push(segment.avgAccel.x);
    ay.push(segment.avgAccel.y);
    az.push(segment.avgAccel.z);
  }
  
  // Solve for each row
  // Row 1: [s11 s12 s13 bx]
  // Row 2: [s21 s22 s23 by]
  // Row 3: [s31 s32 s33 bz]
  
  const row1 = solveLeastSquares(G, ax);
  const row2 = solveLeastSquares(G, ay);
  const row3 = solveLeastSquares(G, az);
  
  // Extract scale matrix and bias
  const scaleMatrix: number[][] = [
    [row1[0], row1[1], row1[2]],
    [row2[0], row2[1], row2[2]],
    [row3[0], row3[1], row3[2]]
  ];
  
  const bias = { x: row1[3], y: row2[3], z: row3[3] };
  
  // Compute inverse for correction
  const scaleMatrixInverse = inverse3x3(scaleMatrix);
  
  // Extract scale factors (diagonal) and cross-axis terms (off-diagonal)
  const scaleFactor = {
    x: scaleMatrix[0][0],
    y: scaleMatrix[1][1],
    z: scaleMatrix[2][2]
  };
  
  const crossAxis = {
    xy: scaleMatrix[0][1],
    xz: scaleMatrix[0][2],
    yx: scaleMatrix[1][0],
    yz: scaleMatrix[1][2],
    zx: scaleMatrix[2][0],
    zy: scaleMatrix[2][1]
  };
  
  // Check for significant cross-axis coupling
  const maxCrossAxis = Math.max(
    Math.abs(crossAxis.xy), Math.abs(crossAxis.xz),
    Math.abs(crossAxis.yx), Math.abs(crossAxis.yz),
    Math.abs(crossAxis.zx), Math.abs(crossAxis.zy)
  );
  
  if (maxCrossAxis > 0.05) {
    warnings.push(`Significant cross-axis coupling detected (max: ${(maxCrossAxis * 100).toFixed(1)}%)`);
  }
  
  // Check scale factors are reasonable (0.9 to 1.1 typically)
  for (const [axis, scale] of Object.entries(scaleFactor)) {
    if (scale < 0.9 || scale > 1.1) {
      warnings.push(`Unusual scale factor for ${axis}: ${scale.toFixed(4)}`);
    }
  }
  
  // Compute residual error
  let residualSum = 0;
  for (const [orient, segment] of positions) {
    const g = GRAVITY_VECTORS[orient];
    const predicted = [
      scaleMatrix[0][0] * g[0] + scaleMatrix[0][1] * g[1] + scaleMatrix[0][2] * g[2] + bias.x,
      scaleMatrix[1][0] * g[0] + scaleMatrix[1][1] * g[1] + scaleMatrix[1][2] * g[2] + bias.y,
      scaleMatrix[2][0] * g[0] + scaleMatrix[2][1] * g[1] + scaleMatrix[2][2] * g[2] + bias.z
    ];
    
    residualSum += (segment.avgAccel.x - predicted[0]) ** 2;
    residualSum += (segment.avgAccel.y - predicted[1]) ** 2;
    residualSum += (segment.avgAccel.z - predicted[2]) ** 2;
  }
  const residualRms = Math.sqrt(residualSum / (n * 3));
  
  // Quality metric
  const orientationQuality = (6 - missingOrientations.length) / 6;
  const scaleQuality = 1 - Math.max(0, maxCrossAxis - 0.01) * 10;
  const residualQuality = Math.exp(-residualRms * 100);
  
  const quality = Math.max(0, Math.min(100,
    orientationQuality * scaleQuality * residualQuality * 100
  ));
  
  // Calculate gyro bias from all stationary segments
  // Weight by number of samples (variance-weighted average)
  let totalSamples = 0;
  let sumGx = 0, sumGy = 0, sumGz = 0;
  let sumVarGx = 0, sumVarGy = 0, sumVarGz = 0;
  
  for (const [, segment] of positions) {
    const weight = segment.sampleCount;
    totalSamples += weight;
    sumGx += segment.avgGyro.x * weight;
    sumGy += segment.avgGyro.y * weight;
    sumGz += segment.avgGyro.z * weight;
    // Accumulate variance (assuming independent segments)
    sumVarGx += (segment.gyroStdDev.x ** 2) * weight;
    sumVarGy += (segment.gyroStdDev.y ** 2) * weight;
    sumVarGz += (segment.gyroStdDev.z ** 2) * weight;
  }
  
  const gyroBias = {
    x: totalSamples > 0 ? sumGx / totalSamples : 0,
    y: totalSamples > 0 ? sumGy / totalSamples : 0,
    z: totalSamples > 0 ? sumGz / totalSamples : 0
  };
  
  // Standard deviation of the combined estimate
  const gyroBiasStdDev = {
    x: totalSamples > 0 ? Math.sqrt(sumVarGx / totalSamples) : 0,
    y: totalSamples > 0 ? Math.sqrt(sumVarGy / totalSamples) : 0,
    z: totalSamples > 0 ? Math.sqrt(sumVarGz / totalSamples) : 0
  };
  
  // Check gyro bias quality
  const avgGyroBiasMag = Math.sqrt(gyroBias.x ** 2 + gyroBias.y ** 2 + gyroBias.z ** 2);
  if (avgGyroBiasMag > 5) {
    warnings.push(`Large gyro bias detected (${avgGyroBiasMag.toFixed(2)} deg/s) - sensor may be faulty`);
  }
  
  return {
    scaleMatrix,
    scaleMatrixInverse,
    bias,
    gyroBias,
    gyroBiasStdDev,
    scaleFactor,
    crossAxis,
    segments,
    residualRms,
    quality,
    warnings
  };
}

/**
 * Apply 6-position calibration to an accelerometer reading
 */
export function applyAccelCalibration(
  raw: { x: number; y: number; z: number },
  calibration: AccelCalibration6PosResult
): { x: number; y: number; z: number } {
  // corrected = S^(-1) * (raw - bias)
  const shifted = [
    raw.x - calibration.bias.x,
    raw.y - calibration.bias.y,
    raw.z - calibration.bias.z
  ];
  
  const corrected = matvec(calibration.scaleMatrixInverse, shifted);
  
  return { x: corrected[0], y: corrected[1], z: corrected[2] };
}

/**
 * Format the scale matrix for display
 */
export function formatScaleMatrix(matrix: number[][]): string {
  return matrix.map(row => 
    row.map(v => v.toFixed(4)).join('  ')
  ).join('\n');
}

/**
 * Get summary of detected segments for display
 */
export function getSegmentSummary(segments: StationarySegment[]): string {
  const lines: string[] = [];
  
  for (const seg of segments) {
    const status = seg.isValid ? '✓' : '✗';
    lines.push(
      `${status} ${seg.orientation}: ${seg.angleFromAxis.toFixed(1)}° from axis, ` +
      `${seg.sampleCount} samples, ${seg.duration.toFixed(2)}s`
    );
  }
  
  return lines.join('\n');
}

/**
 * Format orientation status for UI display (HTML table)
 */
export function formatOrientationStatus(
  bestSegments: Map<AxisOrientation, StationarySegment>
): string {
  const orientations: AxisOrientation[] = ['+X', '-X', '+Y', '-Y', '+Z', '-Z'];
  
  let html = '<table class="orientation-table">';
  html += '<tr><th>Position</th><th>Status</th><th>Angle</th><th>Samples</th></tr>';
  
  for (const orient of orientations) {
    const seg = bestSegments.get(orient);
    if (seg && seg.isValid) {
      html += `<tr class="valid">`;
      html += `<td>${orient} up</td>`;
      html += `<td>✓ Found</td>`;
      html += `<td>${seg.angleFromAxis.toFixed(1)}°</td>`;
      html += `<td>${seg.sampleCount}</td>`;
      html += `</tr>`;
    } else if (seg) {
      html += `<tr class="invalid">`;
      html += `<td>${orient} up</td>`;
      html += `<td>⚠ Poor</td>`;
      html += `<td>${seg.angleFromAxis.toFixed(1)}°</td>`;
      html += `<td>${seg.sampleCount}</td>`;
      html += `</tr>`;
    } else {
      html += `<tr class="missing">`;
      html += `<td>${orient} up</td>`;
      html += `<td>✗ Missing</td>`;
      html += `<td>-</td>`;
      html += `<td>-</td>`;
      html += `</tr>`;
    }
  }
  
  html += '</table>';
  return html;
}
