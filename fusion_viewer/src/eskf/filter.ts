import { isFinite3, norm3, scale3, add3, sub3 } from './math3.ts';
import { covarianceHealth } from './linalg.ts';
import { initialCovariance } from './errorState.ts';
import { DEFAULT_PROPAGATION_OPTIONS, propagate } from './propagation.ts';
import { quatMultiply, quatNormalize } from './quaternion.ts';
import { DEFAULT_VELOCITY_UPDATE, updateVelocity } from './update.ts';
import { ERROR_STATE_SIZE, IDX_THETA } from './types.ts';
import type { InitialUncertainty } from './errorState';
import type { PropagationOptions } from './propagation';
import type { VelocityMeasurement, VelocityUpdateOptions } from './update';
import type { EskfState, ImuMeasurement, Mat3, NoiseDensities, Quat, Vec3 } from './types';

export type FilterStatus = 'not-initialized' | 'aided' | 'coasting' | 'unhealthy';

export interface FilterConfig {
  noise: NoiseDensities;
  propagation: PropagationOptions;
  update: VelocityUpdateOptions;
  /** IMU intervals up to this are substepped; longer gaps make the filter unhealthy. */
  maxImuGapSeconds: number;
  /** Without an accepted update for this long, status becomes 'coasting'. */
  coastingAfterSeconds: number;
  /** This many consecutive rejections make the filter unhealthy. */
  maxConsecutiveRejections: number;
  /** A fix whose epoch is this far behind the state time is rejected as stale. */
  staleToleranceSeconds: number;
}

export const DEFAULT_FILTER_CONFIG: FilterConfig = {
  noise: {
    accelNoiseDensity: 2e-3,
    gyroNoiseDensity: 2e-4,
    gyroBiasRandomWalk: 2e-5,
    accelBiasRandomWalk: 2e-4,
  },
  propagation: DEFAULT_PROPAGATION_OPTIONS,
  update: DEFAULT_VELOCITY_UPDATE,
  maxImuGapSeconds: 0.25,
  coastingAfterSeconds: 1,
  maxConsecutiveRejections: 20,
  staleToleranceSeconds: 0.02,
};

export interface FilterInitialization {
  time: number;
  imu: ImuMeasurement;
  attitude: Quat;
  /** Velocity from an accepted GPS fix at the initialization epoch, NWU m/s. */
  velocity: Vec3;
  gyroBias?: Vec3;
  accelBias?: Vec3;
  uncertainty: InitialUncertainty;
  /** Full body-axis attitude covariance (rad²); overrides uncertainty.attitude when given. */
  attitudeCovariance?: Mat3;
}

export type FilterEvent =
  | { kind: 'initialized'; time: number }
  | { kind: 'update'; time: number; accepted: boolean; reason?: string; nis: number; innovation: Vec3 }
  | { kind: 'status'; time: number; from: FilterStatus; to: FilterStatus; reason: string };

/**
 * Roll/pitch-only attitude q_NB from a mean static specific force in body axes,
 * followed by a rotation of `yaw` radians about navigation Up.
 */
export function levelAttitude(meanSpecificForce: Vec3, yaw: number): Quat {
  const f = meanSpecificForce;
  const n = norm3(f);
  if (!(n > 0) || !isFinite3(f)) throw new Error('Leveling requires a finite, non-zero specific force');
  // Shortest rotation taking body f̂ to navigation Up [0,0,1].
  const u: Vec3 = scale3(f, 1 / n);
  const w = 1 + u[2];
  let qTilt: Quat;
  if (w < 1e-9) qTilt = [0, 1, 0, 0];
  else qTilt = quatNormalize([w, u[1], -u[0], 0]);
  const qYaw: Quat = [Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)];
  return quatNormalize(quatMultiply(qYaw, qTilt));
}

/**
 * Event-driven GPS-velocity-aided ESKF. Feed IMU samples and velocity
 * measurements in non-decreasing effective time. GPS epochs between IMU samples
 * are reached with a zero-order hold of the latest IMU sample (causal).
 */
export class GpsVelocityEskf {
  readonly config: FilterConfig;
  readonly events: FilterEvent[] = [];
  private state: EskfState | null = null;
  private time = NaN;
  private lastImu: ImuMeasurement | null = null;
  private lastImuTime = NaN;
  private lastAcceptedTime = NaN;
  private lastMeasurementTime = -Infinity;
  private consecutiveRejections = 0;
  private statusValue: FilterStatus = 'not-initialized';

  constructor(config: Partial<FilterConfig> = {}) {
    this.config = { ...DEFAULT_FILTER_CONFIG, ...config };
  }

  get status(): FilterStatus { return this.statusValue; }
  get currentTime(): number { return this.time; }
  get currentState(): EskfState | null { return this.state; }

  initialize(init: FilterInitialization): void {
    if (!isFinite3(init.velocity) || !init.attitude.every(Number.isFinite) || !Number.isFinite(init.time)) {
      throw new Error('Initialization contains non-finite values');
    }
    const P = initialCovariance(init.uncertainty);
    if (init.attitudeCovariance) {
      const C = init.attitudeCovariance;
      if (!C.every(Number.isFinite)) throw new Error('Attitude covariance must be finite');
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) P[(IDX_THETA + r) * ERROR_STATE_SIZE + IDX_THETA + c] = 0.5 * (C[r * 3 + c] + C[c * 3 + r]);
      }
    }
    this.state = {
      nominal: {
        p: [0, 0, 0],
        v: [...init.velocity],
        q: quatNormalize(init.attitude),
        gyroBias: init.gyroBias ? [...init.gyroBias] : [0, 0, 0],
        accelBias: init.accelBias ? [...init.accelBias] : [0, 0, 0],
      },
      P,
    };
    this.time = init.time;
    this.lastImu = init.imu;
    this.lastImuTime = init.time;
    this.lastAcceptedTime = init.time;
    this.lastMeasurementTime = -Infinity;
    this.consecutiveRejections = 0;
    this.events.push({ kind: 'initialized', time: init.time });
    this.setStatus('aided', 'initialized', init.time);
  }

  /** Propagates to an IMU sample. Returns false when the filter is not running. */
  processImu(time: number, imu: ImuMeasurement): boolean {
    if (!this.isRunning() || !this.lastImu) return false;
    if (!Number.isFinite(time) || time < this.lastImuTime) {
      this.setStatus('unhealthy', `non-monotonic IMU time ${time} after ${this.lastImuTime}`, time);
      return false;
    }
    if (time - this.lastImuTime > this.config.maxImuGapSeconds) {
      this.setStatus('unhealthy', `IMU gap ${(time - this.lastImuTime).toFixed(3)} s`, time);
      return false;
    }
    if (!isFinite3(imu.gyro) || !isFinite3(imu.specificForce)) {
      this.setStatus('unhealthy', 'non-finite IMU sample', time);
      return false;
    }
    // A held GPS epoch may have advanced the state past the previous sample.
    const start = this.lastImu;
    const t0 = this.time;
    const span = time - this.lastImuTime;
    const interp = (t: number): ImuMeasurement => {
      if (span <= 0) return imu;
      const a = (t - this.lastImuTime) / span;
      return {
        gyro: add3(start.gyro, scale3(sub3(imu.gyro, start.gyro), a)),
        specificForce: add3(start.specificForce, scale3(sub3(imu.specificForce, start.specificForce), a)),
      };
    };
    if (!this.advance(t0, time, interp)) return false;
    this.lastImu = imu;
    this.lastImuTime = time;
    this.refreshCoasting(time);
    return true;
  }

  /** Applies a velocity measurement at its effective epoch. Returns true if accepted. */
  processVelocity(meas: VelocityMeasurement): boolean {
    if (!this.isRunning() || !this.lastImu) return false;
    const record = (accepted: boolean, reason: string | undefined, nis: number, innovation: Vec3) => {
      this.events.push({ kind: 'update', time: meas.time, accepted, reason, nis, innovation });
    };
    if (meas.time <= this.lastMeasurementTime) {
      record(false, 'duplicate or out-of-order epoch', NaN, [NaN, NaN, NaN]);
      return false;
    }
    if (meas.time < this.time - this.config.staleToleranceSeconds) {
      record(false, `stale by ${(this.time - meas.time).toFixed(3)} s`, NaN, [NaN, NaN, NaN]);
      return false;
    }
    if (meas.time > this.time) {
      const held = this.lastImu;
      if (meas.time - this.lastImuTime > this.config.maxImuGapSeconds) {
        this.setStatus('unhealthy', 'GPS epoch beyond IMU data', meas.time);
        return false;
      }
      if (!this.advance(this.time, meas.time, () => held)) return false;
    }
    this.lastMeasurementTime = meas.time;

    const result = updateVelocity(this.state!, meas, this.config.update);
    record(result.accepted, result.reason, result.nis, result.innovation);
    if (result.accepted) {
      if (!covarianceHealth(result.state.P, ERROR_STATE_SIZE).positiveDefinite) {
        this.setStatus('unhealthy', 'covariance lost positive definiteness after update', meas.time);
        return false;
      }
      this.state = result.state;
      this.lastAcceptedTime = meas.time;
      this.consecutiveRejections = 0;
      if (this.statusValue === 'coasting') this.setStatus('aided', 'update accepted', meas.time);
    } else {
      this.consecutiveRejections++;
      if (this.consecutiveRejections >= this.config.maxConsecutiveRejections) {
        this.setStatus('unhealthy', `${this.consecutiveRejections} consecutive rejected updates`, meas.time);
      }
    }
    this.refreshCoasting(meas.time);
    return result.accepted;
  }

  private isRunning(): boolean {
    return this.state !== null && (this.statusValue === 'aided' || this.statusValue === 'coasting');
  }

  /** Propagates from t0 to t1 in substeps no longer than maxStepSeconds. */
  private advance(t0: number, t1: number, sample: (t: number) => ImuMeasurement): boolean {
    const total = t1 - t0;
    if (total <= 0) return true;
    const steps = Math.ceil(total / this.config.propagation.maxStepSeconds - 1e-9);
    const dt = total / steps;
    try {
      let s = this.state!;
      for (let k = 0; k < steps; k++) {
        const ta = t0 + k * dt;
        s = propagate(s, sample(ta), sample(ta + dt), dt, this.config.noise, this.config.propagation);
      }
      this.state = s;
      this.time = t1;
      return true;
    } catch (err) {
      this.setStatus('unhealthy', `propagation failed: ${(err as Error).message}`, t1);
      return false;
    }
  }

  private refreshCoasting(time: number): void {
    if (this.statusValue === 'aided' && time - this.lastAcceptedTime > this.config.coastingAfterSeconds) {
      this.setStatus('coasting', `no accepted GPS update for ${(time - this.lastAcceptedTime).toFixed(2)} s`, time);
    }
  }

  private setStatus(to: FilterStatus, reason: string, time: number): void {
    if (to === this.statusValue) return;
    this.events.push({ kind: 'status', time, from: this.statusValue, to, reason });
    this.statusValue = to;
  }
}
