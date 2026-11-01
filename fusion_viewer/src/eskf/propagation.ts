import { add3, isFinite3, mul33, mulMat3Vec3, scale3, skew, sub3 } from './math3.ts';
import { addScaled, covarianceHealth, identity, matMul, setBlock3, symmetrize, transpose } from './linalg.ts';
import { quatExp, quatMultiply, quatNormalize, quatToMatrix } from './quaternion.ts';
import {
  ERROR_STATE_SIZE, IDX_BA, IDX_BG, IDX_P, IDX_THETA, IDX_V, NOISE_SIZE,
} from './types.ts';
import type { EskfState, ImuMeasurement, Mat3, NoiseDensities, NominalState, Vec3 } from './types';

export const STANDARD_GRAVITY = 9.80665;
/** Gravity in NWU, m/s². */
export const GRAVITY_NWU: Vec3 = [0, 0, -STANDARD_GRAVITY];

export interface PropagationOptions {
  /** Largest dt accepted for a single step; larger gaps must be handled by the caller. */
  maxStepSeconds: number;
}

export const DEFAULT_PROPAGATION_OPTIONS: PropagationOptions = { maxStepSeconds: 0.05 };

const I3: Mat3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];

interface CorrectedInputs {
  gyroAvg: Vec3;
  forceStart: Vec3;
  forceEnd: Vec3;
}

function correctInputs(x: NominalState, start: ImuMeasurement, end: ImuMeasurement): CorrectedInputs {
  const g0 = sub3(start.gyro, x.gyroBias);
  const g1 = sub3(end.gyro, x.gyroBias);
  return {
    gyroAvg: scale3(add3(g0, g1), 0.5),
    forceStart: sub3(start.specificForce, x.accelBias),
    forceEnd: sub3(end.specificForce, x.accelBias),
  };
}

/**
 * Propagates the nominal state over [t, t+dt] from IMU samples at both ends.
 * Attitude uses the mean bias-corrected rate; navigation acceleration
 * R(q)f + g is integrated with the trapezoidal rule (exact for constant
 * navigation-frame acceleration). Biases are held constant.
 */
export function propagateNominal(x: NominalState, start: ImuMeasurement, end: ImuMeasurement, dt: number): NominalState {
  const { gyroAvg, forceStart, forceEnd } = correctInputs(x, start, end);
  const q1 = quatNormalize(quatMultiply(x.q, quatExp(scale3(gyroAvg, dt))));
  const a0 = add3(mulMat3Vec3(quatToMatrix(x.q), forceStart), GRAVITY_NWU);
  const a1 = add3(mulMat3Vec3(quatToMatrix(q1), forceEnd), GRAVITY_NWU);
  const a = scale3(add3(a0, a1), 0.5);
  return {
    p: add3(add3(x.p, scale3(x.v, dt)), scale3(a, 0.5 * dt * dt)),
    v: add3(x.v, scale3(a, dt)),
    q: q1,
    gyroBias: [...x.gyroBias],
    accelBias: [...x.accelBias],
  };
}

/**
 * Continuous error-state Jacobian F (15×15) for right-multiplicative attitude error:
 *   dp' = dv
 *   dv' = -R[f]x dθ - R db_a
 *   dθ' = -[ω]x dθ - db_g
 * where ω and f are bias-corrected body measurements.
 */
export function errorStateJacobian(R: Mat3, omega: Vec3, force: Vec3): Float64Array {
  const n = ERROR_STATE_SIZE;
  const F = new Float64Array(n * n);
  setBlock3(F, n, IDX_P, IDX_V, I3);
  setBlock3(F, n, IDX_V, IDX_THETA, mul33(R, skew(force)), -1);
  setBlock3(F, n, IDX_V, IDX_BA, R, -1);
  setBlock3(F, n, IDX_THETA, IDX_THETA, skew(omega), -1);
  setBlock3(F, n, IDX_THETA, IDX_BG, I3, -1);
  return F;
}

/**
 * Noise input matrix G (15×12) for noise vector [n_a, n_g, n_bg, n_ba]:
 * accel white noise enters dv via -R, gyro white noise enters dθ via -I,
 * and bias random walks drive the bias errors directly.
 */
export function noiseInputMatrix(R: Mat3): Float64Array {
  const G = new Float64Array(ERROR_STATE_SIZE * NOISE_SIZE);
  setBlock3(G, NOISE_SIZE, IDX_V, 0, R, -1);
  setBlock3(G, NOISE_SIZE, IDX_THETA, 3, I3, -1);
  setBlock3(G, NOISE_SIZE, IDX_BG, 6, I3);
  setBlock3(G, NOISE_SIZE, IDX_BA, 9, I3);
  return G;
}

/** Continuous noise spectral density matrix Qc (12×12) in the order used by noiseInputMatrix. */
export function continuousNoise(noise: NoiseDensities): Float64Array {
  const densities = [noise.accelNoiseDensity, noise.gyroNoiseDensity, noise.gyroBiasRandomWalk, noise.accelBiasRandomWalk];
  const Qc = new Float64Array(NOISE_SIZE * NOISE_SIZE);
  densities.forEach((d, block) => {
    for (let i = 0; i < 3; i++) Qc[(block * 3 + i) * NOISE_SIZE + block * 3 + i] = d * d;
  });
  return Qc;
}

/**
 * Discretizes over dt: Φ = I + FΔt + ½(FΔt)², and
 * Qd ≈ ½(Φ GQcGᵀ + GQcGᵀ Φᵀ)Δt (trapezoidal approximation of the Van Loan integral).
 */
export function discretize(F: Float64Array, G: Float64Array, Qc: Float64Array, dt: number): { Phi: Float64Array; Qd: Float64Array } {
  const n = ERROR_STATE_SIZE;
  const Fdt = F.map(v => v * dt);
  const Phi = addScaled(addScaled(identity(n), Fdt), matMul(Fdt, Fdt, n, n, n), 0.5);
  const GQGt = matMul(matMul(G, Qc, n, NOISE_SIZE, NOISE_SIZE), transpose(G, n, NOISE_SIZE), n, NOISE_SIZE, n);
  const PhiQ = matMul(Phi, GQGt, n, n, n);
  const Qd = symmetrize(PhiQ, n).map(v => v * dt);
  return { Phi, Qd };
}

export function propagateCovariance(P: Float64Array, Phi: Float64Array, Qd: Float64Array): Float64Array {
  const n = ERROR_STATE_SIZE;
  const PhiP = matMul(Phi, P, n, n, n);
  return symmetrize(addScaled(matMul(PhiP, transpose(Phi, n, n), n, n, n), Qd), n);
}

function validateNoise(noise: NoiseDensities): void {
  for (const [key, value] of Object.entries(noise)) {
    if (!Number.isFinite(value) || value < 0) {
      throw new Error(`Noise density ${key} must be finite and non-negative, got ${value}`);
    }
  }
}

/**
 * One ESKF prediction step from IMU samples at t and t+dt. Throws on invalid
 * dt, non-finite inputs, or a non-finite result instead of returning a plausible state.
 */
export function propagate(
  state: EskfState,
  start: ImuMeasurement,
  end: ImuMeasurement,
  dt: number,
  noise: NoiseDensities,
  options: PropagationOptions = DEFAULT_PROPAGATION_OPTIONS,
): EskfState {
  if (!Number.isFinite(dt) || dt <= 0) throw new Error(`Propagation dt must be positive and finite, got ${dt}`);
  if (dt > options.maxStepSeconds) {
    throw new Error(`Propagation dt ${dt} s exceeds maxStepSeconds ${options.maxStepSeconds} s`);
  }
  for (const m of [start, end]) {
    if (!isFinite3(m.gyro) || !isFinite3(m.specificForce)) throw new Error('IMU measurement contains non-finite values');
  }
  validateNoise(noise);

  const x = state.nominal;
  const { gyroAvg, forceStart, forceEnd } = correctInputs(x, start, end);
  // Linearize at the interval midpoint.
  const qMid = quatNormalize(quatMultiply(x.q, quatExp(scale3(gyroAvg, 0.5 * dt))));
  const RMid = quatToMatrix(qMid);
  const F = errorStateJacobian(RMid, gyroAvg, scale3(add3(forceStart, forceEnd), 0.5));
  const G = noiseInputMatrix(RMid);
  const { Phi, Qd } = discretize(F, G, continuousNoise(noise), dt);

  const nominal = propagateNominal(x, start, end, dt);
  const P = propagateCovariance(state.P, Phi, Qd);

  if (![...nominal.p, ...nominal.v, ...nominal.q].every(Number.isFinite) || !covarianceHealth(P, ERROR_STATE_SIZE).finite) {
    throw new Error('Propagation produced a non-finite state or covariance');
  }
  return { nominal, P };
}

