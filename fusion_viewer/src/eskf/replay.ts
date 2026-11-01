import { getBlock3 } from './linalg.ts';
import { mul33, mulMat3Vec3, norm3, transpose33 } from './math3.ts';
import { quatMultiply, quatNormalize, quatToMatrix } from './quaternion.ts';
import { STANDARD_GRAVITY } from './propagation.ts';
import { calibrateImu, remapMatrix, validateCalibration } from './calibration.ts';
import { GpsVelocityEskf, levelAttitude } from './filter.ts';
import { DEFAULT_GNSS_VELOCITY_NOISE, velocityMeasurementFromGnss } from './update.ts';
import { ERROR_STATE_SIZE, IDX_THETA } from './types.ts';
import type { ImuCalibration } from './calibration';
import type { FilterConfig, FilterEvent, FilterStatus } from './filter';
import type { GnssVelocityFix, GnssVelocityNoiseOptions } from './update';
import type { EskfState, ImuMeasurement, Mat3, Quat, Vec3 } from './types';

const DEG = Math.PI / 180;

export interface ReplayImuSample {
  /** Sensor time, s. */
  time: number;
  gyroDps: Vec3;
  accelG: Vec3;
}

export interface ReplayGnssFix extends GnssVelocityFix {
  /** Synchronized sensor time before any estimator shift, s. */
  time: number;
}

export type AttitudeSeed =
  /** Full attitude (ESKF convention) from another source, e.g. device or viewer quaternion. */
  | { kind: 'attitude'; label: string; attitudeAt: (time: number) => Quat | null }
  /** Tilt from mean specific force before the init epoch; heading of the body +Y axis, ° clockwise from North. */
  | { kind: 'level'; headingDeg: number };

export interface ReplayOptions {
  /** Added to GPS epochs: positive moves GPS later; delayed GPS needs a negative shift. */
  gpsTimeShiftSeconds: number;
  /** Earliest initialization epoch, sensor s. */
  startTime: number;
  seed: AttitudeSeed;
  levelWindowSeconds: number;
  tiltSigmaDeg: number;
  headingSigmaDeg: number;
  gyroBiasSigmaDps: number;
  accelBiasSigmaMg: number;
  gnssNoise: GnssVelocityNoiseOptions;
  filter: Partial<FilterConfig>;
  /** Re-initialize at the next usable fix this long after the filter became unhealthy; null disables. */
  reinitializeAfterSeconds: number | null;
}

export const DEFAULT_REPLAY_OPTIONS: Omit<ReplayOptions, 'seed'> = {
  gpsTimeShiftSeconds: 0,
  startTime: -Infinity,
  levelWindowSeconds: 1,
  tiltSigmaDeg: 5,
  headingSigmaDeg: 30,
  gyroBiasSigmaDps: 0.5,
  accelBiasSigmaMg: 30,
  gnssNoise: DEFAULT_GNSS_VELOCITY_NOISE,
  filter: {},
  reinitializeAfterSeconds: 1,
};

export interface ReplayFrame {
  time: number;
  status: FilterStatus;
  /** Body-to-NWU attitude (ESKF convention). */
  q: Quat;
  /** NWU velocity, m/s (NaN before initialization). */
  v: Vec3;
  p: Vec3;
  gyroBias: Vec3;
  accelBias: Vec3;
  /** Calibrated specific force minus estimated accel bias, body axes, m/s². */
  force: Vec3;
  /** One-sigma attitude uncertainty about navigation Up, rad. */
  headingSigma: number;
  /** RMS one-sigma attitude uncertainty about North and West, rad. */
  tiltSigma: number;
}

export type ReplayEvent =
  | FilterEvent
  | { kind: 'gnss-rejected'; time: number; reason: string }
  | { kind: 'reinitialized'; time: number; reason: string };

export interface ReplaySummary {
  initTime: number | null;
  seedLabel: string;
  acceptedUpdates: number;
  rejectedUpdates: number;
  gnssRejected: number;
  reinitializations: number;
  /** RMS of accepted innovations, NWU m/s. */
  innovationRms: Vec3;
  meanNis: number;
  finalStatus: FilterStatus;
  finalGyroBiasDps: Vec3 | null;
  finalAccelBiasMg: Vec3 | null;
  secondsByStatus: Record<FilterStatus, number>;
}

export interface ReplayResult {
  frames: ReplayFrame[];
  events: ReplayEvent[];
  summary: ReplaySummary;
  warnings: string[];
}

/** Rotates q about navigation Up so the horizontal projection of body +Y points at headingDeg (clockwise from North). */
export function setBodyYHeading(q: Quat, headingDeg: number): Quat {
  const y = mulMat3Vec3(quatToMatrix(q), [0, 1, 0]);
  const horizontal = Math.hypot(y[0], y[1]);
  if (horizontal < 0.2) throw new Error('Body +Y is near vertical at initialization; heading of +Y is undefined. Use a quaternion seed.');
  const current = Math.atan2(-y[1], y[0]);
  const delta = current - headingDeg * DEG;
  return quatNormalize(quatMultiply([Math.cos(delta / 2), 0, 0, Math.sin(delta / 2)], q));
}

/** Body-axis attitude covariance from navigation-frame tilt and heading sigmas: Rᵀ diag(σt², σt², σh²) R. */
function attitudeCovarianceBody(q: Quat, tilt: number, heading: number): Mat3 {
  const R = quatToMatrix(q);
  const D: Mat3 = [tilt * tilt, 0, 0, 0, tilt * tilt, 0, 0, 0, heading * heading];
  return mul33(mul33(transpose33(R), D), R);
}

function attitudeSigmas(state: EskfState): { heading: number; tilt: number } {
  const Rt = transpose33(quatToMatrix(state.nominal.q));
  const Ptt = getBlock3(state.P, ERROR_STATE_SIZE, IDX_THETA, IDX_THETA);
  const along = (axis: Vec3) => {
    const u = mulMat3Vec3(Rt, axis);
    let s = 0;
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) s += u[i] * Ptt[i * 3 + j] * u[j];
    return Math.max(s, 0);
  };
  return {
    heading: Math.sqrt(along([0, 0, 1])),
    tilt: Math.sqrt(0.5 * (along([1, 0, 0]) + along([0, 1, 0]))),
  };
}

const NAN3: Vec3 = [NaN, NaN, NaN];

/**
 * Offline, causal-in-measurement-time replay of the GPS-velocity-aided ESKF.
 * Produces one frame per IMU sample. Throws if the inputs cannot initialize the filter.
 */
export function runEskfReplay(
  imu: ReplayImuSample[],
  gnss: ReplayGnssFix[],
  calibration: ImuCalibration,
  options: ReplayOptions,
): ReplayResult {
  validateCalibration(calibration);
  if (imu.length < 2) throw new Error('ESKF replay needs at least two IMU samples');
  if (!Number.isFinite(options.gpsTimeShiftSeconds) || Math.abs(options.gpsTimeShiftSeconds) > 5) {
    throw new Error('GPS time shift must be finite and within ±5 s');
  }
  const remap = remapMatrix(calibration.axisRemap);
  const meas: ImuMeasurement[] = imu.map(s => calibrateImu(s.gyroDps, s.accelG, calibration, remap));
  const fixes = gnss
    .map(f => ({ ...f, time: f.time + options.gpsTimeShiftSeconds }))
    .filter(f => Number.isFinite(f.time))
    .sort((a, b) => a.time - b.time);

  const filter = new GpsVelocityEskf(options.filter);
  const warnings: string[] = [];
  const extraEvents: ReplayEvent[] = [];
  const frames: ReplayFrame[] = new Array(imu.length);
  let initTime: number | null = null;
  const seedLabel = options.seed.kind === 'level' ? `level + body Y heading ${options.seed.headingDeg}°` : options.seed.label;
  let unhealthySince: number | null = null;
  let reinitializations = 0;
  let gnssRejected = 0;

  const uncertainty = (velocitySigma: number) => ({
    position: 1,
    velocity: velocitySigma,
    attitude: [1, 1, 1] as Vec3,
    gyroBias: options.gyroBiasSigmaDps * DEG,
    accelBias: options.accelBiasSigmaMg * 1e-3 * STANDARD_GRAVITY,
  });

  const initializeAt = (fixTime: number, velocity: Vec3, velocitySigma: number, i: number, reason: string | null) => {
    let q: Quat;
    let gyroBias: Vec3 = [0, 0, 0];
    let accelBias: Vec3 = [0, 0, 0];
    const previous = filter.currentState;
    const previousUsable = previous !== null
      && [...previous.nominal.q, ...previous.nominal.gyroBias, ...previous.nominal.accelBias].every(Number.isFinite);
    if (reason !== null && previousUsable) {
      q = quatNormalize(previous.nominal.q);
      gyroBias = previous.nominal.gyroBias;
      accelBias = previous.nominal.accelBias;
    } else if (options.seed.kind === 'attitude') {
      const seeded = options.seed.attitudeAt(fixTime);
      if (!seeded) throw new Error(`${options.seed.label} has no attitude at the initialization epoch ${fixTime.toFixed(3)} s`);
      q = quatNormalize(seeded);
    } else {
      const window = options.levelWindowSeconds;
      let sum: Vec3 = [0, 0, 0];
      let gyroSum = 0;
      let count = 0;
      for (let k = i; k >= 0 && imu[k].time >= fixTime - window; k--) {
        const f = meas[k].specificForce;
        sum = [sum[0] + f[0], sum[1] + f[1], sum[2] + f[2]];
        gyroSum += norm3(meas[k].gyro);
        count++;
      }
      if (count < 5) throw new Error('Not enough IMU samples before the initialization epoch to level');
      const mean: Vec3 = [sum[0] / count, sum[1] / count, sum[2] / count];
      const gRatio = norm3(mean) / STANDARD_GRAVITY;
      const meanRateDps = gyroSum / count / DEG;
      if (Math.abs(gRatio - 1) > 0.05 || meanRateDps > 3) {
        warnings.push(`Leveling window is not static (|f| = ${gRatio.toFixed(3)} g, mean rate ${meanRateDps.toFixed(1)}°/s); tilt seed is unreliable.`);
      }
      q = setBodyYHeading(levelAttitude(mean, 0), options.seed.headingDeg);
    }
    const unc = uncertainty(velocitySigma);
    const attitudeCovariance = attitudeCovarianceBody(q, options.tiltSigmaDeg * DEG, options.headingSigmaDeg * DEG);
    filter.initialize({ time: fixTime, imu: meas[i], attitude: q, velocity, gyroBias, accelBias, uncertainty: unc, attitudeCovariance });
    if (reason !== null) {
      reinitializations++;
      extraEvents.push({ kind: 'reinitialized', time: fixTime, reason });
    } else {
      initTime = fixTime;
    }
  };

  let fixIdx = 0;
  let lastFrame: ReplayFrame | null = null;
  for (let i = 0; i < imu.length; i++) {
    const t = imu[i].time;
    while (fixIdx < fixes.length && fixes[fixIdx].time <= t) {
      const fix = fixes[fixIdx++];
      if (fix.time < options.startTime || i === 0) continue;
      const built = velocityMeasurementFromGnss(fix, fix.time, options.gnssNoise);
      if (!built.ok) {
        gnssRejected++;
        extraEvents.push({ kind: 'gnss-rejected', time: fix.time, reason: built.reason });
        continue;
      }
      const m = built.measurement;
      if (filter.status === 'not-initialized') {
        initializeAt(fix.time, m.velocity, Math.sqrt(m.covariance[0]), i - 1, null);
        continue;
      }
      if (filter.status === 'unhealthy') {
        if (options.reinitializeAfterSeconds !== null && unhealthySince !== null
          && fix.time - unhealthySince >= options.reinitializeAfterSeconds) {
          initializeAt(fix.time, m.velocity, Math.sqrt(m.covariance[0]), i - 1, 'automatic re-initialization after unhealthy state');
          unhealthySince = null;
        }
        continue;
      }
      filter.processVelocity(m);
    }

    if (filter.status !== 'not-initialized' && filter.status !== 'unhealthy') filter.processImu(t, meas[i]);
    if (filter.status === 'unhealthy' && unhealthySince === null) unhealthySince = t;

    const state = filter.currentState;
    let frame: ReplayFrame;
    if (!state) {
      frame = {
        time: t, status: 'not-initialized', q: [1, 0, 0, 0], v: NAN3, p: NAN3,
        gyroBias: NAN3, accelBias: NAN3, force: meas[i].specificForce, headingSigma: NaN, tiltSigma: NaN,
      };
    } else {
      const s = attitudeSigmas(state);
      const ba = state.nominal.accelBias;
      const f = meas[i].specificForce;
      frame = {
        time: t,
        status: filter.status,
        q: state.nominal.q,
        v: state.nominal.v,
        p: state.nominal.p,
        gyroBias: state.nominal.gyroBias,
        accelBias: ba,
        force: [f[0] - ba[0], f[1] - ba[1], f[2] - ba[2]],
        headingSigma: s.heading,
        tiltSigma: s.tilt,
      };
    }
    frames[i] = frame;
    lastFrame = frame;
  }

  if (initTime === null) {
    throw new Error('ESKF never initialized: no usable GPS velocity fix after the start time (check TRACK.CSV sync, numSV/sAcc gates, and start time).');
  }
  // Frames before initialization carry the initial attitude so the display is not a fabricated identity pose.
  const firstInit = frames.find(f => f.status !== 'not-initialized');
  if (firstInit) {
    for (const f of frames) {
      if (f.status !== 'not-initialized') break;
      f.q = firstInit.q;
    }
  }

  const events: ReplayEvent[] = [...filter.events, ...extraEvents].sort((a, b) => a.time - b.time);
  const updates = filter.events.filter((e): e is Extract<FilterEvent, { kind: 'update' }> => e.kind === 'update');
  const accepted = updates.filter(e => e.accepted);
  const rms = (k: number) => accepted.length
    ? Math.sqrt(accepted.reduce((s, e) => s + e.innovation[k] ** 2, 0) / accepted.length) : NaN;
  const secondsByStatus: Record<FilterStatus, number> = { 'not-initialized': 0, aided: 0, coasting: 0, unhealthy: 0 };
  for (let i = 1; i < frames.length; i++) secondsByStatus[frames[i].status] += frames[i].time - frames[i - 1].time;
  const finalState = filter.currentState;

  return {
    frames,
    events,
    warnings,
    summary: {
      initTime,
      seedLabel,
      acceptedUpdates: accepted.length,
      rejectedUpdates: updates.length - accepted.length,
      gnssRejected,
      reinitializations,
      innovationRms: [rms(0), rms(1), rms(2)],
      meanNis: accepted.length ? accepted.reduce((s, e) => s + e.nis, 0) / accepted.length : NaN,
      finalStatus: lastFrame?.status ?? 'not-initialized',
      finalGyroBiasDps: finalState ? finalState.nominal.gyroBias.map(b => b / DEG) as Vec3 : null,
      finalAccelBiasMg: finalState ? finalState.nominal.accelBias.map(b => b / STANDARD_GRAVITY * 1e3) as Vec3 : null,
      secondsByStatus,
    },
  };
}
