/**
 * Shared ESKF types. Conventions (see GPS-VELOCITY-AIDED-ESKF-PLAN.md §4):
 * - Navigation frame N: local North-West-Up (NWU).
 * - Body frame B: firmware IMU axes.
 * - Quaternions: Hamilton, scalar first [w, x, y, z], active body-to-navigation (q_NB).
 * - Attitude error: right-multiplicative, q_true = q_nominal ⊗ Exp(dθ), dθ in body axes.
 * - All quantities SI: m, m/s, m/s², rad, rad/s, s.
 */

export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];
/** Row-major 3×3 matrix. */
export type Mat3 = [number, number, number, number, number, number, number, number, number];

export interface NominalState {
  /** Relative position in N, m. */
  p: Vec3;
  /** Velocity in N, m/s. */
  v: Vec3;
  /** Body-to-navigation attitude q_NB. */
  q: Quat;
  /** Gyro bias in B, rad/s. */
  gyroBias: Vec3;
  /** Accelerometer bias in B, m/s². */
  accelBias: Vec3;
}

export interface EskfState {
  nominal: NominalState;
  /** Error-state covariance, 15×15 row-major, order [dp, dv, dθ, db_g, db_a]. */
  P: Float64Array;
}

/** Measured IMU values for one sample, already converted to SI in body axes. */
export interface ImuMeasurement {
  /** Angular rate, rad/s. */
  gyro: Vec3;
  /** Specific force, m/s². */
  specificForce: Vec3;
}

/**
 * Continuous-time noise densities. Discrete covariance is derived from these and dt.
 */
export interface NoiseDensities {
  /** Accelerometer white noise, m/s²/√Hz. */
  accelNoiseDensity: number;
  /** Gyro white noise, rad/s/√Hz. */
  gyroNoiseDensity: number;
  /** Gyro bias random walk, rad/s²/√Hz. */
  gyroBiasRandomWalk: number;
  /** Accelerometer bias random walk, m/s³/√Hz. */
  accelBiasRandomWalk: number;
}

export const ERROR_STATE_SIZE = 15;
export const NOISE_SIZE = 12;
export const IDX_P = 0;
export const IDX_V = 3;
export const IDX_THETA = 6;
export const IDX_BG = 9;
export const IDX_BA = 12;
