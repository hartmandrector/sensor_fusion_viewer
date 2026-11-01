import { add3, sub3 } from './math3.ts';
import { quatConjugate, quatExp, quatLog, quatMultiply, quatNormalize } from './quaternion.ts';
import { ERROR_STATE_SIZE, IDX_BA, IDX_BG, IDX_P, IDX_THETA, IDX_V } from './types.ts';
import type { NominalState, Vec3 } from './types';

function slice3(dx: ArrayLike<number>, i: number): Vec3 {
  return [dx[i], dx[i + 1], dx[i + 2]];
}

/** x ⊕ dx: additive for p, v, biases; q ⊗ Exp(dθ) for attitude. */
export function injectError(x: NominalState, dx: ArrayLike<number>): NominalState {
  if (dx.length !== ERROR_STATE_SIZE) throw new Error(`Error vector must have ${ERROR_STATE_SIZE} elements`);
  return {
    p: add3(x.p, slice3(dx, IDX_P)),
    v: add3(x.v, slice3(dx, IDX_V)),
    q: quatNormalize(quatMultiply(x.q, quatExp(slice3(dx, IDX_THETA)))),
    gyroBias: add3(x.gyroBias, slice3(dx, IDX_BG)),
    accelBias: add3(x.accelBias, slice3(dx, IDX_BA)),
  };
}

/** truth ⊖ nominal, the inverse of injectError. */
export function errorBetween(truth: NominalState, nominal: NominalState): Float64Array {
  const dx = new Float64Array(ERROR_STATE_SIZE);
  dx.set(sub3(truth.p, nominal.p), IDX_P);
  dx.set(sub3(truth.v, nominal.v), IDX_V);
  dx.set(quatLog(quatMultiply(quatConjugate(nominal.q), truth.q)), IDX_THETA);
  dx.set(sub3(truth.gyroBias, nominal.gyroBias), IDX_BG);
  dx.set(sub3(truth.accelBias, nominal.accelBias), IDX_BA);
  return dx;
}

/** One-sigma initial uncertainties, per axis. */
export interface InitialUncertainty {
  position: number; // m
  velocity: number; // m/s
  attitude: Vec3; // rad, body-axis error angles
  gyroBias: number; // rad/s
  accelBias: number; // m/s²
}

export function initialCovariance(u: InitialUncertainty): Float64Array {
  const sigmas = [
    u.position, u.position, u.position,
    u.velocity, u.velocity, u.velocity,
    ...u.attitude,
    u.gyroBias, u.gyroBias, u.gyroBias,
    u.accelBias, u.accelBias, u.accelBias,
  ];
  const n = ERROR_STATE_SIZE;
  const P = new Float64Array(n * n);
  sigmas.forEach((s, i) => {
    if (!Number.isFinite(s) || s < 0) throw new Error(`Initial sigma at index ${i} must be finite and non-negative`);
    P[i * n + i] = s * s;
  });
  return P;
}
