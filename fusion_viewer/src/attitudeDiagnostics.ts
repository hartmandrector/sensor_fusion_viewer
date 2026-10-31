import type { IntegrationResult } from './accelerationIntegration';
import type { GPSIntegrationResult } from './gpsTypes';
import type { Vector3 } from './types';

export const DIAGNOSTIC_GRAVITY = 9.80665;
export const MIN_DIAGNOSTIC_FORCE = 0.1 * DIAGNOSTIC_GRAVITY;

export interface AttitudeDiagnosticPoint {
  sensorTime: number;
  imuForce: Vector3 | null;
  gpsForce: Vector3 | null;
  angleDegrees: number | null;
  magnitudeDifference: number | null;
  residualMagnitude: number | null;
  status: 'valid' | 'weak-force' | 'gps-edge-or-gap' | 'imu-gap-or-invalid';
}

export interface AttitudeDiagnostics {
  points: AttitudeDiagnosticPoint[];
  validCount: number;
  weakCount: number;
  excludedCount: number;
  timeOrigin: number;
}

function medianInterval(times: number[]): number {
  const intervals = times.slice(1).map((time, i) => time - times[i]);
  if (intervals.some(dt => !Number.isFinite(dt) || dt <= 0)) {
    throw new Error('Diagnostics require finite, strictly increasing timestamps.');
  }
  intervals.sort((a, b) => a - b);
  return intervals[Math.floor(intervals.length / 2)];
}

function finiteVector(v: Vector3): boolean {
  return [v.x, v.y, v.z].every(Number.isFinite);
}

/**
 * Compare at GPS epochs, interpolating IMU Earth acceleration only inside overlap.
 * The force vectors are a - g; in NWU this adds +9.80665 to linear acceleration Up.
 * No attitude correction is inferred from this residual.
 */
export function computeAttitudeDiagnostics(
  integration: IntegrationResult,
  gps: GPSIntegrationResult,
  gpsHalfWindowSamples: number,
  gpsTimeShiftSeconds = 0
): AttitudeDiagnostics {
  if (!Number.isFinite(gpsTimeShiftSeconds)) throw new Error('GPS time shift must be finite.');
  const { time } = integration;
  const gpsPoints = gps.points;
  if (time.length < 2 || gpsPoints.length < 2) {
    throw new Error('Attitude diagnostics need at least two IMU and GPS samples.');
  }
  if (![integration.accelNorth, integration.accelWest, integration.accelUp]
    .every(values => values.length === time.length)
    || !Number.isInteger(integration.startIndex)
    || integration.startIndex < 0 || integration.startIndex >= time.length) {
    throw new Error('Invalid integration arrays or start index for attitude diagnostics.');
  }
  if (!Number.isInteger(gpsHalfWindowSamples) || gpsHalfWindowSamples < 0) {
    throw new Error('Invalid GPS processing window for attitude diagnostics.');
  }
  const imuInterval = medianInterval(time);
  const gpsTimes = gpsPoints.map(point => point.sensorTime);
  const gpsInterval = medianInterval(gpsTimes);
  if (![time[0], gpsTimes[0]].every(Number.isFinite)) {
    throw new Error('Diagnostics require finite timestamps.');
  }
  // A break anywhere inside the smoothing/derivative support invalidates that epoch.
  const gpsBreaks = [0];
  for (let i = 1; i < gpsTimes.length; i++) {
    gpsBreaks[i] = gpsBreaks[i - 1] + (gpsTimes[i] - gpsTimes[i - 1] > 3 * gpsInterval ? 1 : 0);
  }
  const result: AttitudeDiagnostics = {
    points: [], validCount: 0, weakCount: 0, excludedCount: 0,
    timeOrigin: time[integration.startIndex]
  };
  let imuIndex = integration.startIndex;
  for (let i = 0; i < gpsPoints.length; i++) {
    const sample = gpsPoints[i];
    const t = sample.sensorTime + gpsTimeShiftSeconds;
    if (t < result.timeOrigin || t > time[time.length - 1]) continue;
    const point: AttitudeDiagnosticPoint = {
      sensorTime: t, imuForce: null, gpsForce: null, angleDegrees: null,
      magnitudeDifference: null, residualMagnitude: null, status: 'gps-edge-or-gap'
    };
    result.points.push(point);
    const left = i - gpsHalfWindowSamples, right = i + gpsHalfWindowSamples;
    if (left < 0 || right >= gpsPoints.length || gpsBreaks[right] !== gpsBreaks[left]) {
      result.excludedCount++;
      continue;
    }
    const gpsForce = {
      x: sample.accelNorth, y: sample.accelWest, z: sample.accelUp + DIAGNOSTIC_GRAVITY
    };
    if (!finiteVector(gpsForce)) {
      result.excludedCount++;
      continue;
    }
    while (imuIndex < time.length - 2 && time[imuIndex + 1] < t) imuIndex++;
    const next = imuIndex + 1;
    const dt = time[next] - time[imuIndex];
    if (dt > 3 * imuInterval) {
      point.status = 'imu-gap-or-invalid';
      result.excludedCount++;
      continue;
    }
    const fraction = (t - time[imuIndex]) / dt;
    const interpolate = (values: number[]) =>
      values[imuIndex] + fraction * (values[next] - values[imuIndex]);
    const imuForce = {
      x: interpolate(integration.accelNorth),
      y: interpolate(integration.accelWest),
      z: interpolate(integration.accelUp) + DIAGNOSTIC_GRAVITY
    };
    if (!finiteVector(imuForce)) {
      point.status = 'imu-gap-or-invalid';
      result.excludedCount++;
      continue;
    }
    const imuMagnitude = Math.hypot(imuForce.x, imuForce.y, imuForce.z);
    const gpsMagnitude = Math.hypot(gpsForce.x, gpsForce.y, gpsForce.z);
    point.imuForce = imuForce;
    point.gpsForce = gpsForce;
    point.magnitudeDifference = imuMagnitude - gpsMagnitude;
    point.residualMagnitude = Math.hypot(
      gpsForce.x - imuForce.x, gpsForce.y - imuForce.y, gpsForce.z - imuForce.z
    );
    if (imuMagnitude < MIN_DIAGNOSTIC_FORCE || gpsMagnitude < MIN_DIAGNOSTIC_FORCE) {
      point.status = 'weak-force';
      result.weakCount++;
    } else {
      const cosine = (imuForce.x * gpsForce.x + imuForce.y * gpsForce.y + imuForce.z * gpsForce.z)
        / (imuMagnitude * gpsMagnitude);
      point.angleDegrees = Math.acos(Math.max(-1, Math.min(1, cosine))) * 180 / Math.PI;
      point.status = 'valid';
      result.validCount++;
    }
  }
  if (result.points.length === 0) {
    throw new Error('No GPS/IMU overlap after the integration start time.');
  }
  if (result.validCount + result.weakCount === 0) {
    throw new Error('No usable force comparisons: check GPS processing edges, data gaps and overlap.');
  }
  return result;
}
