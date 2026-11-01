/**
 * Viewer glue for the GPS-velocity-aided ESKF: gathers inputs from the
 * dataset, TRACK.CSV, calibration and the settings panel, runs the replay
 * when its inputs change, and overlays the result onto the fusion frames.
 */

import { state, type EskfFrameInfo, type FusionFrame } from './appState';
import { getElements } from './uiElements';
import { gpsState } from './gpsIntegration';
import { eulerAndHeading } from './orientationSource';
import { debug } from './constants';
import { deviceToEskfQuaternion, eskfToViewerQuaternion, viewerToEskfQuaternion } from './eskf/calibration';
import { DEFAULT_REPLAY_OPTIONS, runEskfReplay } from './eskf/replay';
import { runDropoutTest, type DropoutReport } from './eskf/dropout';
import { DEFAULT_GNSS_VELOCITY_NOISE } from './eskf/update';
import { DEFAULT_FILTER_CONFIG } from './eskf/filter';
import { quatToMatrix } from './eskf/quaternion';
import { STANDARD_GRAVITY } from './eskf/propagation';
import type { ImuCalibration } from './eskf/calibration';
import type { AttitudeSeed, ReplayGnssFix, ReplayImuSample, ReplayOptions, ReplayResult } from './eskf/replay';
import type { Mat3, Quat, Vec3 } from './eskf/types';

/** Gyro scale measured on new-firmware recordings (Stage A spin test). */
export const NEW_FIRMWARE_GYRO_SCALE = 1.14;
const DEG = Math.PI / 180;

let forceNextRun = false;
const objectIds = new WeakMap<object, number>();
let nextObjectId = 1;
function objectId(obj: object | null | undefined): number {
  if (!obj) return 0;
  let id = objectIds.get(obj);
  if (id === undefined) objectIds.set(obj, id = nextObjectId++);
  return id;
}

type SeedKind = 'device' | 'viewer' | 'level';

interface PanelSettings {
  seed: SeedKind;
  seedHeadingDeg: number;
  headingSigmaDeg: number;
  gyroScale: number;
  gpsShift: number;
  startRelative: number;
  nisGate: number;
  sAccScale: number;
  reinitialize: boolean;
}

function readNumber(input: HTMLInputElement | HTMLSelectElement, name: string, min: number, max: number): number {
  const value = Number(input.value.trim());
  if (!input.value.trim() || !Number.isFinite(value) || value < min || value > max) {
    throw new Error(`${name} must be a number between ${min} and ${max}`);
  }
  return value;
}

function readSettings(): PanelSettings {
  const e = getElements();
  return {
    seed: e.eskfSeed.value as SeedKind,
    seedHeadingDeg: readNumber(e.eskfSeedHeading, '+Y heading', -360, 360),
    headingSigmaDeg: readNumber(e.eskfHeadingSigma, 'Heading σ₀', 0.5, 180),
    gyroScale: readNumber(e.eskfGyroScale, 'Gyro scale', 0.5, 2),
    gpsShift: readNumber(e.eskfGpsShift, 'GPS time shift', -5, 5),
    startRelative: readNumber(e.eskfStartTime, 'Start time', 0, Number.MAX_SAFE_INTEGER),
    nisGate: readNumber(e.eskfNisGate, 'Innovation gate', 1, 1e9),
    sAccScale: readNumber(e.eskfSAccScale, 'sAcc scale', 0.1, 10),
    reinitialize: e.eskfReinit.checked,
  };
}

function currentCalibration(gyroScale: number): ImuCalibration {
  const ahrs = state.fusionAhrs;
  if (!ahrs) throw new Error('Fusion AHRS is not initialized');
  const cal = ahrs.getIMUCalibration();
  const S = ahrs.getAccelScaleMatrix();
  return {
    gyroBiasDps: [cal.gyroBiasX, cal.gyroBiasY, cal.gyroBiasZ],
    gyroScale,
    accelBiasG: [cal.accelOffsetX, cal.accelOffsetY, cal.accelOffsetZ],
    accelScaleMatrix: S ? (S.flat() as Mat3) : null,
    axisRemap: ahrs.getIMUAxisRemap(),
  };
}

function gnssFixes(): ReplayGnssFix[] {
  const data = gpsState.trackDataset?.gnssData ?? [];
  return data
    .filter(g => g.sensorTimestamp !== null)
    .map(g => ({ time: g.sensorTimestamp!, velN: g.velN, velE: g.velE, velD: g.velD, sAcc: g.sAcc, numSV: g.numSV }));
}

/** Index of the frame whose timestamp is nearest t (frames are time-ordered). */
function nearestFrame(frames: FusionFrame[], t: number): number {
  let lo = 0;
  let hi = frames.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (frames[mid].timestamp < t) lo = mid + 1;
    else hi = mid;
  }
  if (lo > 0 && Math.abs(frames[lo - 1].timestamp - t) < Math.abs(frames[lo].timestamp - t)) return lo - 1;
  return lo;
}

function buildSeed(settings: PanelSettings, frames: FusionFrame[], calibration: ImuCalibration): AttitudeSeed {
  if (settings.seed === 'level') return { kind: 'level', headingDeg: settings.seedHeadingDeg };
  if (settings.seed === 'viewer') {
    // Fusion frames are captured now, before the ESKF overlay replaces them.
    const quats = frames.map(f => [f.quaternion.w, f.quaternion.x, f.quaternion.y, f.quaternion.z] as Quat);
    const times = frames.map(f => f.timestamp);
    return {
      kind: 'attitude',
      label: 'Fusion (Ch. 7) attitude',
      attitudeAt: t => {
        const i = nearestFrame(frames, t);
        return Math.abs(times[i] - t) <= 0.1 ? viewerToEskfQuaternion(quats[i]) : null;
      },
    };
  }
  return {
    kind: 'attitude',
    label: 'device quaternion',
    attitudeAt: t => {
      const i = nearestFrame(frames, t);
      for (let k = i; k < frames.length && frames[k].timestamp - t <= 0.1; k++) {
        const q = frames[k].imu?.deviceQuaternion;
        if (q) return deviceToEskfQuaternion([q.w, q.x, q.y, q.z], calibration.axisRemap);
      }
      return null;
    },
  };
}

function seedKey(settings: PanelSettings, frames: FusionFrame[]): string {
  if (settings.seed !== 'viewer') return '';
  let sum = 0;
  for (let i = 0; i < frames.length; i += 16) sum += frames[i].quaternion.w * (1 + (i % 7));
  return sum.toPrecision(12);
}

function frameInfo(f: ReplayResult['frames'][number]): EskfFrameInfo {
  return {
    status: f.status,
    headingSigmaDeg: f.headingSigma / DEG,
    tiltSigmaDeg: f.tiltSigma / DEG,
    gyroBiasDps: f.gyroBias.map(b => b / DEG) as Vec3,
    accelBiasMg: f.accelBias.map(b => (b / STANDARD_GRAVITY) * 1e3) as Vec3,
    velocity: f.v,
  };
}

/** Replaces the attitude-derived fields of each frame with the ESKF solution. */
function overlay(frames: FusionFrame[], result: ReplayResult): void {
  for (let i = 0; i < frames.length; i++) {
    const r = result.frames[i];
    const qv = eskfToViewerQuaternion(r.q);
    const quaternion = { w: qv[0], x: qv[1], y: qv[2], z: qv[3] };
    const { euler, heading } = eulerAndHeading(quaternion);
    const R = quatToMatrix(r.q);
    const f = r.force;
    const up: Vec3 = [R[6], R[7], R[8]]; // Rᵀ·[0,0,1]: navigation Up in body axes
    const g0 = STANDARD_GRAVITY;
    const frame = frames[i];
    frame.quaternion = quaternion;
    frame.euler = euler;
    frame.heading = heading;
    frame.gravity = { x: up[0], y: up[1], z: up[2] };
    frame.expectedGravity = frame.gravity;
    frame.expectedMag = undefined;
    frame.linearAccel = { x: f[0] / g0 - up[0], y: f[1] / g0 - up[1], z: f[2] / g0 - up[2] };
    frame.earthAccel = {
      x: (R[0] * f[0] + R[1] * f[1] + R[2] * f[2]) / g0,
      y: (R[3] * f[0] + R[4] * f[1] + R[5] * f[2]) / g0,
      z: (R[6] * f[0] + R[7] * f[1] + R[8] * f[2] - g0) / g0,
    };
    frame.internalStates = undefined;
    frame.biasState = undefined;
    frame.eskf = frameInfo(r);
  }
}

function formatSummary(run: NonNullable<typeof state.eskfRun>, stale: boolean): string {
  if (run.error) return `ESKF failed: ${run.error}\nShowing Fusion (Ch. 7) output instead.`;
  const result = run.result!;
  const s = result.summary;
  const start = state.dataset?.startTime ?? 0;
  const fmt = (v: Vec3 | null, d: number) => (v ? v.map(x => x.toFixed(d)).join(', ') : '--');
  const lines = [
    `Initialized at ${(s.initTime! - start).toFixed(2)} s from ${s.seedLabel}. Run took ${(run.elapsedMs / 1000).toFixed(1)} s.`,
    `GPS updates: ${s.acceptedUpdates} accepted, ${s.rejectedUpdates} gated, ${s.gnssRejected} pre-rejected (numSV/sAcc).`,
    `Innovation RMS N/W/U: ${fmt(s.innovationRms, 2)} m/s; mean NIS ${s.meanNis.toFixed(2)} (≈3 if consistent).`,
    `Time aided ${s.secondsByStatus.aided.toFixed(0)} s, coasting ${s.secondsByStatus.coasting.toFixed(0)} s, unhealthy ${s.secondsByStatus.unhealthy.toFixed(0)} s; re-inits ${s.reinitializations}.`,
    `Final gyro bias ${fmt(s.finalGyroBiasDps, 3)} °/s; accel bias ${fmt(s.finalAccelBiasMg, 1)} mg.`,
  ];
  if (result.warnings.length) lines.push(`⚠ ${result.warnings.join(' ')}`);
  if (stale) lines.push('⚠ Fusion output changed since this run (seed source). Press Run ESKF to update.');
  return lines.join('\n');
}

/** Whether the ESKF can run with the currently loaded data. */
export function eskfInputsAvailable(): boolean {
  return !!state.dataset && gpsState.isLoaded && !!gpsState.trackDataset;
}

/** Forces the next frame computation to re-run the ESKF even if inputs are unchanged. */
export function requestEskfRun(): void {
  forceNextRun = true;
}

/**
 * Called from computeFusionFrames after the Fusion pass. Re-runs the replay only
 * when its inputs changed (or a run was requested), then overlays the result.
 */
export function applyEskfToFrames(frames: FusionFrame[]): void {
  const e = getElements();
  const force = forceNextRun;
  forceNextRun = false;
  e.eskfRunBtn.disabled = !eskfInputsAvailable();
  if (!eskfInputsAvailable()) {
    e.eskfRunStatus.textContent = 'ESKF needs SENSOR.CSV and a synchronized TRACK.CSV (GPS panel below). Showing Fusion (Ch. 7) output.';
    e.eskfRunStatus.style.color = '#ffcc66';
    return;
  }

  let settings: PanelSettings;
  let calibration: ImuCalibration;
  try {
    settings = readSettings();
    calibration = currentCalibration(settings.gyroScale);
  } catch (error) {
    e.eskfRunStatus.textContent = `ESKF settings invalid: ${error instanceof Error ? error.message : String(error)}. Showing Fusion (Ch. 7) output.`;
    e.eskfRunStatus.style.color = '#ff6666';
    return;
  }

  const dataset = state.dataset!;
  const inputKey = JSON.stringify({
    dataset: objectId(dataset), track: objectId(gpsState.trackDataset), frames: frames.length,
    calibration, settings,
  });
  const currentSeedKey = seedKey(settings, frames);
  let run = state.eskfRun;

  if (force || !run || run.inputKey !== inputKey) {
    const imu: ReplayImuSample[] = frames.map(f => ({
      time: f.timestamp,
      gyroDps: [f.imu!.wx, f.imu!.wy, f.imu!.wz],
      accelG: [f.imu!.ax, f.imu!.ay, f.imu!.az],
    }));
    const options: ReplayOptions = {
      ...DEFAULT_REPLAY_OPTIONS,
      gpsTimeShiftSeconds: settings.gpsShift,
      startTime: dataset.startTime + settings.startRelative,
      seed: buildSeed(settings, frames, calibration),
      headingSigmaDeg: settings.headingSigmaDeg,
      gnssNoise: { ...DEFAULT_GNSS_VELOCITY_NOISE, sAccScale: settings.sAccScale },
      filter: { update: { ...DEFAULT_FILTER_CONFIG.update, nisGate: settings.nisGate } },
      reinitializeAfterSeconds: settings.reinitialize ? DEFAULT_REPLAY_OPTIONS.reinitializeAfterSeconds : null,
    };
    const t0 = performance.now();
    const gnss = gnssFixes();
    let result: ReplayResult | null = null;
    let error: string | null = null;
    try {
      result = runEskfReplay(imu, gnss, calibration, options);
    } catch (err) {
      error = err instanceof Error ? err.message : String(err);
      debug.error('ESKF replay failed:', err);
    }
    run = {
      inputKey, seedKey: currentSeedKey, result, error, elapsedMs: performance.now() - t0,
      inputs: { imu, gnss, calibration, gpsShift: settings.gpsShift },
    };
    state.eskfRun = run;
  }

  const usable = run.result && run.result.frames.length === frames.length;
  if (usable) overlay(frames, run.result!);
  e.eskfRunStatus.textContent = formatSummary(run, run.seedKey !== currentSeedKey);
  e.eskfRunStatus.style.color = run.error ? '#ff6666' : '';
  e.eskfChartsBtn.disabled = !usable;
}

/** True when the current frames carry the ESKF overlay (not a Fusion fallback). */
export function framesUseEskf(): boolean {
  const frames = state.fusionFrames;
  return state.algorithm === 'eskf' && frames.length > 0 && !!frames[frames.length - 1].eskf;
}

const dropoutCache = new WeakMap<ReplayResult, DropoutReport>();

/** GPS-dropout test on the latest ESKF run (cached per result); null when there is no usable run. */
export function getEskfDropoutReport(): DropoutReport | null {
  const run = state.eskfRun;
  if (!run?.result) return null;
  let report = dropoutCache.get(run.result);
  if (!report) {
    const t0 = performance.now();
    report = runDropoutTest(run.result.frames, run.inputs.imu, run.inputs.gnss, run.inputs.calibration, {
      gpsTimeShiftSeconds: run.inputs.gpsShift,
    });
    debug.log(`ESKF dropout test: ${report.windows.length} windows in ${(performance.now() - t0).toFixed(0)} ms`);
    dropoutCache.set(run.result, report);
  }
  return report;
}

/** Live per-frame status in the settings panel. */
export function updateEskfLiveDisplay(info: EskfFrameInfo | undefined): void {
  const e = getElements();
  if (!info) {
    e.eskfLiveStatus.textContent = 'not running (Fusion shown)';
    e.eskfLiveStatus.style.color = '#888';
    e.eskfLiveHeadingSigma.textContent = '--';
    e.eskfLiveTiltSigma.textContent = '--';
    e.eskfLiveGyroBias.textContent = '--';
    e.eskfLiveAccelBias.textContent = '--';
    return;
  }
  const colors: Record<EskfFrameInfo['status'], string> = {
    'not-initialized': '#888', aided: '#6bff6b', coasting: '#ffff6b', unhealthy: '#ff6b6b',
  };
  e.eskfLiveStatus.textContent = info.status === 'not-initialized' ? 'not initialized (seed attitude held)' : info.status;
  e.eskfLiveStatus.style.color = colors[info.status];
  const num = (v: number, d: number) => (Number.isFinite(v) ? v.toFixed(d) : '--');
  e.eskfLiveHeadingSigma.textContent = num(info.headingSigmaDeg, 1);
  e.eskfLiveTiltSigma.textContent = num(info.tiltSigmaDeg, 2);
  e.eskfLiveGyroBias.textContent = info.gyroBiasDps.map(v => num(v, 3)).join(', ');
  e.eskfLiveAccelBias.textContent = info.accelBiasMg.map(v => num(v, 1)).join(', ');
}

/** Resets panel defaults for a newly loaded recording. */
export function onEskfDatasetLoaded(): void {
  const e = getElements();
  state.eskfRun = null;
  const hasDevice = state.dataset?.readings.some(r => r.type === 'IMU' && r.deviceQuaternion) ?? false;
  e.eskfGyroScale.value = hasDevice ? String(NEW_FIRMWARE_GYRO_SCALE) : '1';
  e.eskfGyroScale.title = hasDevice
    ? `Auto-set to ${NEW_FIRMWARE_GYRO_SCALE} because this recording has device quaternions (new firmware).`
    : 'Multiplier on bias-corrected gyro rates.';
  const deviceOption = e.eskfSeed.querySelector<HTMLOptionElement>('option[value="device"]');
  if (deviceOption) deviceOption.disabled = !hasDevice;
  if (!hasDevice && e.eskfSeed.value === 'device') e.eskfSeed.value = 'viewer';
  e.eskfStartTime.max = state.dataset ? state.dataset.duration.toFixed(1) : '';
  e.eskfStartTime.value = '0';
  e.eskfGpsShift.value = '0';
  e.eskfChartsBtn.disabled = true;
  updateEskfSeedVisibility();
}

export function updateEskfSeedVisibility(): void {
  const e = getElements();
  e.eskfSeedHeadingRow.style.display = e.eskfSeed.value === 'level' ? '' : 'none';
}
