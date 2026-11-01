import { isFinite3, mul33, skew } from './math3.ts';
import { cholesky, covarianceHealth, identity, matMul, setBlock3, symmetrize, transpose } from './linalg.ts';
import { injectError } from './errorState.ts';
import { ERROR_STATE_SIZE, IDX_THETA, IDX_V } from './types.ts';
import type { EskfState, Mat3, Vec3 } from './types';

/** χ² 3-DOF thresholds for NIS gating. */
export const CHI2_3DOF = { p95: 7.815, p99: 11.345, p999: 16.266 } as const;

/** A GPS velocity measurement in NWU at its effective epoch. */
export interface VelocityMeasurement {
  /** Effective measurement epoch, sensor seconds. */
  time: number;
  /** Velocity in NWU, m/s. */
  velocity: Vec3;
  /** Measurement covariance, (m/s)², row-major 3×3. */
  covariance: Mat3;
}

/** Raw GNSS fields needed for a velocity measurement (subset of GNSSData). */
export interface GnssVelocityFix {
  velN: number;
  velE: number;
  velD: number;
  /** Receiver speed accuracy estimate, m/s. */
  sAcc: number;
  numSV: number;
}

/**
 * Declared approximation for R_v: u-blox sAcc is a scalar speed accuracy, not a
 * full covariance. Each axis variance is (sAccScale·sAcc)², floored, with an
 * optional extra factor on the vertical axis.
 */
export interface GnssVelocityNoiseOptions {
  sAccScale: number;
  /** Per-axis one-sigma floor, m/s. */
  sigmaFloor: number;
  /** Multiplier on the vertical one-sigma. */
  verticalScale: number;
  minNumSV: number;
  /** Fixes with larger sAcc are rejected, m/s. */
  maxSAcc: number;
}

export const DEFAULT_GNSS_VELOCITY_NOISE: GnssVelocityNoiseOptions = {
  sAccScale: 1,
  sigmaFloor: 0.05,
  verticalScale: 1,
  minNumSV: 6,
  maxSAcc: 2,
};

export type MeasurementBuildResult =
  | { ok: true; measurement: VelocityMeasurement }
  | { ok: false; reason: string };

/** Converts a raw NED GNSS fix to an NWU velocity measurement, or explains the rejection. */
export function velocityMeasurementFromGnss(
  fix: GnssVelocityFix,
  time: number,
  options: GnssVelocityNoiseOptions = DEFAULT_GNSS_VELOCITY_NOISE,
): MeasurementBuildResult {
  if (!Number.isFinite(time)) return { ok: false, reason: 'non-finite epoch' };
  if (![fix.velN, fix.velE, fix.velD].every(Number.isFinite)) return { ok: false, reason: 'non-finite velocity' };
  if (!Number.isFinite(fix.sAcc) || fix.sAcc <= 0) return { ok: false, reason: 'missing or invalid sAcc' };
  if (!Number.isFinite(fix.numSV) || fix.numSV < options.minNumSV) {
    return { ok: false, reason: `numSV ${fix.numSV} below ${options.minNumSV}` };
  }
  if (fix.sAcc > options.maxSAcc) return { ok: false, reason: `sAcc ${fix.sAcc} above ${options.maxSAcc}` };

  const sigmaH = Math.max(options.sAccScale * fix.sAcc, options.sigmaFloor);
  const sigmaV = Math.max(options.sAccScale * fix.sAcc * options.verticalScale, options.sigmaFloor);
  return {
    ok: true,
    measurement: {
      time,
      velocity: [fix.velN, -fix.velE, -fix.velD],
      covariance: [sigmaH ** 2, 0, 0, 0, sigmaH ** 2, 0, 0, 0, sigmaV ** 2],
    },
  };
}

/** Solves S·X = B for X, given the Cholesky factor L of S (n×n) and B (n×m). */
function choleskySolve(L: Float64Array, B: Float64Array, n: number, m: number): Float64Array {
  const X = Float64Array.from(B);
  for (let col = 0; col < m; col++) {
    for (let i = 0; i < n; i++) {
      let s = X[i * m + col];
      for (let k = 0; k < i; k++) s -= L[i * n + k] * X[k * m + col];
      X[i * m + col] = s / L[i * n + i];
    }
    for (let i = n - 1; i >= 0; i--) {
      let s = X[i * m + col];
      for (let k = i + 1; k < n; k++) s -= L[k * n + i] * X[k * m + col];
      X[i * m + col] = s / L[i * n + i];
    }
  }
  return X;
}

/**
 * Covariance reset after injecting dθ̂ with right-multiplicative error:
 * δθ⁺ = Log(Exp(−dθ̂)·Exp(dθ̂ + ε)) = J_r(dθ̂) ε + O(ε²), so P ← G P Gᵀ with the
 * exact SO(3) right Jacobian (first order: I − ½[dθ̂]×).
 */
export function resetJacobian(dTheta: Vec3): Float64Array {
  const n = ERROR_STATE_SIZE;
  const G = identity(n);
  const theta = Math.hypot(dTheta[0], dTheta[1], dTheta[2]);
  // Series coefficients for small angles avoid cancellation.
  const a = theta < 1e-4 ? 0.5 - theta * theta / 24 : (1 - Math.cos(theta)) / (theta * theta);
  const b = theta < 1e-4 ? 1 / 6 - theta * theta / 120 : (theta - Math.sin(theta)) / (theta ** 3);
  const K = skew(dTheta);
  const K2 = mul33(K, K);
  const block = [0, 1, 2, 3, 4, 5, 6, 7, 8].map(i => (i % 4 === 0 ? 1 : 0) - a * K[i] + b * K2[i]);
  setBlock3(G, n, IDX_THETA, IDX_THETA, block);
  return G;
}

export interface VelocityUpdateOptions {
  /** NIS rejection threshold; Infinity disables gating. */
  nisGate: number;
}

export const DEFAULT_VELOCITY_UPDATE: VelocityUpdateOptions = { nisGate: CHI2_3DOF.p999 };

export interface VelocityUpdateResult {
  state: EskfState;
  accepted: boolean;
  reason?: string;
  /** z − h(x), NWU m/s. */
  innovation: Vec3;
  /** H P Hᵀ + R, row-major 3×3. */
  innovationCovariance: Mat3;
  nis: number;
  /** Injected error-state correction, or null when rejected. */
  correction: Float64Array | null;
}

/**
 * Colocated GPS velocity update: H = [0 I 0 0 0]. Uses a Cholesky solve,
 * Joseph-form covariance, and the attitude reset Jacobian. Rejected updates
 * return the input state unchanged.
 */
export function updateVelocity(
  state: EskfState,
  meas: VelocityMeasurement,
  options: VelocityUpdateOptions = DEFAULT_VELOCITY_UPDATE,
): VelocityUpdateResult {
  const n = ERROR_STATE_SIZE;
  const { P, nominal } = state;
  const innovation: Vec3 = [
    meas.velocity[0] - nominal.v[0],
    meas.velocity[1] - nominal.v[1],
    meas.velocity[2] - nominal.v[2],
  ];
  const S = new Float64Array(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) S[i * 3 + j] = P[(IDX_V + i) * n + IDX_V + j] + meas.covariance[i * 3 + j];
  }
  const Smat = Array.from(S) as Mat3;
  const reject = (reason: string, nis = NaN): VelocityUpdateResult => (
    { state, accepted: false, reason, innovation, innovationCovariance: Smat, nis, correction: null }
  );

  if (!isFinite3(meas.velocity) || !meas.covariance.every(Number.isFinite)) return reject('non-finite measurement');
  const L = cholesky(symmetrize(S, 3), 3);
  if (!L) return reject('innovation covariance not positive definite');

  const Sinv_r = choleskySolve(L, Float64Array.from(innovation), 3, 1);
  const nis = innovation[0] * Sinv_r[0] + innovation[1] * Sinv_r[1] + innovation[2] * Sinv_r[2];
  if (!Number.isFinite(nis)) return reject('non-finite NIS');
  if (nis > options.nisGate) return reject(`NIS ${nis.toFixed(2)} above gate ${options.nisGate}`, nis);

  // H P (3×15) is the velocity rows of P; Kᵀ = S⁻¹ H P.
  const HP = P.slice(IDX_V * n, (IDX_V + 3) * n);
  const K = transpose(choleskySolve(L, HP, 3, n), 3, n);
  const dx = matMul(K, Float64Array.from(innovation), n, 3, 1);

  const IKH = identity(n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < 3; j++) IKH[i * n + IDX_V + j] -= K[i * 3 + j];
  }
  const R = Float64Array.from(meas.covariance);
  const joseph = matMul(matMul(IKH, P, n, n, n), transpose(IKH, n, n), n, n, n);
  const KRKt = matMul(matMul(K, R, n, 3, 3), transpose(K, n, 3), n, 3, n);
  for (let i = 0; i < joseph.length; i++) joseph[i] += KRKt[i];

  const G = resetJacobian([dx[IDX_THETA], dx[IDX_THETA + 1], dx[IDX_THETA + 2]]);
  const Pnew = symmetrize(matMul(matMul(G, joseph, n, n, n), transpose(G, n, n), n, n, n), n);
  const nominalNew = injectError(nominal, dx);

  if (![...nominalNew.p, ...nominalNew.v, ...nominalNew.q].every(Number.isFinite) || !covarianceHealth(Pnew, n).finite) {
    return reject('update produced a non-finite state or covariance', nis);
  }
  return {
    state: { nominal: nominalNew, P: Pnew },
    accepted: true,
    innovation,
    innovationCovariance: Smat,
    nis,
    correction: dx,
  };
}
