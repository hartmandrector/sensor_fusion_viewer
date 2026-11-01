/**
 * Withheld-GPS ("dropout") test. Starting from the filter state at many epochs,
 * propagate on the IMU alone for D seconds — exactly what the ESKF mean does
 * while GPS is missing — and compare the predicted velocity with the GPS fix it
 * did not receive. A hold-last-velocity predictor is the baseline the IMU must beat.
 */

import { calibrateImu, remapMatrix, validateCalibration } from './calibration.ts';
import { propagateNominal } from './propagation.ts';
import { velocityMeasurementFromGnss, DEFAULT_GNSS_VELOCITY_NOISE } from './update.ts';
import type { ImuCalibration } from './calibration';
import type { ReplayFrame, ReplayGnssFix, ReplayImuSample } from './replay';
import type { ImuMeasurement, NominalState, Vec3 } from './types';

export interface DropoutOptions {
  /** Dropout lengths, s (ascending). */
  durations: number[];
  /** Same shift used by the replay: positive moves GPS later. */
  gpsTimeShiftSeconds: number;
  /** Window spacing while GPS speed ≥ movingSpeed, s. */
  movingSpacingSeconds: number;
  /** Window spacing while slower, s. */
  staticSpacingSeconds: number;
  /** Windows starting at a GPS speed at or above this are "moving", m/s. */
  movingSpeed: number;
  /** Windows containing a larger IMU gap are skipped, s. */
  maxImuGapSeconds: number;
  /** GPS reference is interpolated only across fix gaps up to this, s. */
  maxGpsGapSeconds: number;
}

export const DEFAULT_DROPOUT_OPTIONS: DropoutOptions = {
  durations: [1, 2, 5, 10],
  gpsTimeShiftSeconds: 0,
  movingSpacingSeconds: 0.5,
  staticSpacingSeconds: 5,
  movingSpeed: 5,
  maxImuGapSeconds: 0.05,
  maxGpsGapSeconds: 0.5,
};

export interface DropoutWindow {
  /** Window start (last GPS-aided epoch), sensor s. */
  startTime: number;
  /** GPS speed at the start, m/s. */
  speed: number;
  moving: boolean;
  /** Per duration: IMU prediction minus GPS, NWU m/s (null when no reference). */
  imuError: (Vec3 | null)[];
  /** Per duration: held start velocity minus GPS, NWU m/s. */
  holdError: (Vec3 | null)[];
}

export interface DropoutStats {
  duration: number;
  count: number;
  imuRms: number;
  imuMedian: number;
  imuP90: number;
  holdRms: number;
  holdMedian: number;
}

export interface DropoutReport {
  durations: number[];
  windows: DropoutWindow[];
  all: DropoutStats[];
  moving: DropoutStats[];
}

interface GpsReference {
  t: Float64Array;
  v: Vec3[];
}

function buildReference(gnss: ReplayGnssFix[], shift: number): GpsReference {
  const ok: { t: number; v: Vec3 }[] = [];
  for (const fix of gnss) {
    const t = fix.time + shift;
    const built = velocityMeasurementFromGnss(fix, t, DEFAULT_GNSS_VELOCITY_NOISE);
    if (built.ok) ok.push({ t, v: built.measurement.velocity });
  }
  ok.sort((a, b) => a.t - b.t);
  return { t: Float64Array.from(ok, o => o.t), v: ok.map(o => o.v) };
}

function referenceAt(ref: GpsReference, time: number, maxGap: number): Vec3 | null {
  const { t, v } = ref;
  if (t.length === 0 || time < t[0] || time > t[t.length - 1]) return null;
  let lo = 0;
  let hi = t.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (t[mid] <= time) lo = mid; else hi = mid;
  }
  const span = t[hi] - t[lo];
  if (span > maxGap) return null;
  const a = span > 0 ? (time - t[lo]) / span : 0;
  return [0, 1, 2].map(k => v[lo][k] + a * (v[hi][k] - v[lo][k])) as Vec3;
}

function percentile(sorted: number[], p: number): number {
  if (!sorted.length) return NaN;
  return sorted[Math.min(sorted.length - 1, Math.floor(p * sorted.length))];
}

function stats(windows: DropoutWindow[], durations: number[]): DropoutStats[] {
  return durations.map((duration, k) => {
    const imu: number[] = [];
    const hold: number[] = [];
    for (const w of windows) {
      const e = w.imuError[k];
      const h = w.holdError[k];
      if (e && h) {
        imu.push(Math.hypot(...e));
        hold.push(Math.hypot(...h));
      }
    }
    imu.sort((a, b) => a - b);
    hold.sort((a, b) => a - b);
    const rms = (xs: number[]) => (xs.length ? Math.sqrt(xs.reduce((s, x) => s + x * x, 0) / xs.length) : NaN);
    return {
      duration, count: imu.length,
      imuRms: rms(imu), imuMedian: percentile(imu, 0.5), imuP90: percentile(imu, 0.9),
      holdRms: rms(hold), holdMedian: percentile(hold, 0.5),
    };
  });
}

/**
 * Runs the dropout test over a completed replay. `frames` must be 1:1 with `imu`.
 * Windows start only at GPS-aided epochs; the replay itself is not modified.
 */
export function runDropoutTest(
  frames: ReplayFrame[],
  imu: ReplayImuSample[],
  gnss: ReplayGnssFix[],
  calibration: ImuCalibration,
  options: Partial<DropoutOptions> = {},
): DropoutReport {
  const opts = { ...DEFAULT_DROPOUT_OPTIONS, ...options };
  validateCalibration(calibration);
  if (frames.length !== imu.length) throw new Error('Dropout test needs one replay frame per IMU sample');
  const durations = [...opts.durations].sort((a, b) => a - b);
  if (!durations.length || durations.some(d => !(d > 0))) throw new Error('Dropout durations must be positive');
  const longest = durations[durations.length - 1];
  const remap = remapMatrix(calibration.axisRemap);
  const ref = buildReference(gnss, opts.gpsTimeShiftSeconds);
  const meas: (ImuMeasurement | undefined)[] = new Array(imu.length);
  const measAt = (i: number) => (meas[i] ??= calibrateImu(imu[i].gyroDps, imu[i].accelG, calibration, remap));

  const windows: DropoutWindow[] = [];
  let nextMoving = -Infinity;
  let nextStatic = -Infinity;
  for (let i0 = 0; i0 < frames.length; i0++) {
    const f0 = frames[i0];
    const t0 = f0.time;
    if (f0.status !== 'aided' || (t0 < nextMoving && t0 < nextStatic)) continue;
    const gps0 = referenceAt(ref, t0, opts.maxGpsGapSeconds);
    if (!gps0) continue;
    const speed = Math.hypot(...gps0);
    const moving = speed >= opts.movingSpeed;
    if (moving ? t0 < nextMoving : t0 < nextStatic) continue;
    if (moving) nextMoving = t0 + opts.movingSpacingSeconds;
    else nextStatic = t0 + opts.staticSpacingSeconds;

    let x: NominalState = { p: [0, 0, 0], v: f0.v, q: f0.q, gyroBias: f0.gyroBias, accelBias: f0.accelBias };
    const imuError: (Vec3 | null)[] = durations.map(() => null);
    const holdError: (Vec3 | null)[] = durations.map(() => null);
    let k = 0;
    let broken = false;
    for (let i = i0 + 1; i < frames.length && k < durations.length; i++) {
      const dt = imu[i].time - imu[i - 1].time;
      if (!(dt > 0) || dt > opts.maxImuGapSeconds) { broken = true; break; }
      x = propagateNominal(x, measAt(i - 1), measAt(i), dt);
      while (k < durations.length && imu[i].time - t0 >= durations[k]) {
        const gps = referenceAt(ref, imu[i].time, opts.maxGpsGapSeconds);
        if (gps) {
          imuError[k] = [x.v[0] - gps[0], x.v[1] - gps[1], x.v[2] - gps[2]];
          holdError[k] = [f0.v[0] - gps[0], f0.v[1] - gps[1], f0.v[2] - gps[2]];
        }
        k++;
      }
      if (imu[i].time - t0 > longest + 1) break;
    }
    if (broken && imuError.every(e => e === null)) continue;
    windows.push({ startTime: t0, speed, moving, imuError, holdError });
  }
  return {
    durations,
    windows,
    all: stats(windows, durations),
    moving: stats(windows.filter(w => w.moving), durations),
  };
}
