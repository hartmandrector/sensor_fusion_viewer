import assert from 'node:assert/strict';
import { test } from 'node:test';
import {
  computeAttitudeDiagnostics, DIAGNOSTIC_GRAVITY as G, MIN_DIAGNOSTIC_FORCE
} from '../src/attitudeDiagnostics.ts';

function fixture(n = 9) {
  const time = Array.from({ length: n }, (_, i) => i);
  return {
    integration: {
      time, accelNorth: time.map(() => 0), accelWest: time.map(() => 0),
      accelUp: time.map(() => 0), startIndex: 0, orientationSource: 'device'
    },
    gps: {
      points: time.map(sensorTime => ({ sensorTime, accelNorth: 0, accelWest: 0, accelUp: 0 }))
    }
  };
}

function near(actual, expected) {
  assert.ok(Math.abs(actual - expected) < 1e-10, `${actual} != ${expected}`);
}

test('stationary force includes positive Earth Up gravity, not linear acceleration', () => {
  const { integration, gps } = fixture();
  const saved = structuredClone({ integration, gps });
  const result = computeAttitudeDiagnostics(integration, gps, 1);
  assert.equal(result.validCount, 7);
  assert.equal(result.excludedCount, 2);
  assert.equal(result.points.length, 9);
  for (const point of result.points.slice(1, -1)) {
    assert.deepEqual(point.imuForce, { x: 0, y: 0, z: G });
    assert.deepEqual(point.gpsForce, point.imuForce);
    near(point.angleDegrees, 0);
    near(point.residualMagnitude, 0);
    near(point.magnitudeDifference, 0);
  }
  assert.equal(result.points[0].angleDegrees, null);
  assert.equal(result.points[0].gpsForce, null);
  assert.deepEqual({ integration, gps }, saved);
});

test('known orthogonal forces yield 90-degree error and signed magnitude difference', () => {
  const { integration, gps } = fixture();
  integration.accelNorth.fill(2);
  integration.accelUp.fill(-G);
  for (const p of gps.points) { p.accelWest = 3; p.accelUp = -G; }
  const point = computeAttitudeDiagnostics(integration, gps, 0).points[4];
  assert.deepEqual(point.imuForce, { x: 2, y: 0, z: 0 });
  assert.deepEqual(point.gpsForce, { x: 0, y: 3, z: 0 });
  near(point.angleDegrees, 90);
  near(point.magnitudeDifference, -1);
  near(point.residualMagnitude, Math.sqrt(13));
});

test('opposing vectors yield 180 degrees without reversing the residual convention', () => {
  const { integration, gps } = fixture();
  integration.accelNorth.fill(2);
  integration.accelUp.fill(-G);
  for (const p of gps.points) { p.accelNorth = -2; p.accelUp = -G; }
  const point = computeAttitudeDiagnostics(integration, gps, 0).points[4];
  near(point.angleDegrees, 180);
  near(point.residualMagnitude, 4);
});

test('ballistic free fall and either weak force suppress angle but preserve magnitudes', () => {
  const { integration, gps } = fixture();
  integration.accelUp.fill(-G);
  for (const p of gps.points) p.accelUp = -G;
  const freeFall = computeAttitudeDiagnostics(integration, gps, 0);
  assert.equal(freeFall.weakCount, 9);
  assert.equal(freeFall.validCount, 0);
  assert.equal(freeFall.points[4].angleDegrees, null);
  near(freeFall.points[4].residualMagnitude, 0);
  gps.points[4].accelUp = 0;
  const mixed = computeAttitudeDiagnostics(integration, gps, 0).points[4];
  assert.equal(mixed.status, 'weak-force');
  near(mixed.magnitudeDifference, -G);
});

test('0.1 g cutoff is measured against both force magnitudes', () => {
  const { integration, gps } = fixture();
  integration.accelUp.fill(-G);
  integration.accelNorth.fill(MIN_DIAGNOSTIC_FORCE);
  for (const p of gps.points) {
    p.accelUp = -G;
    p.accelNorth = MIN_DIAGNOSTIC_FORCE;
  }
  assert.equal(computeAttitudeDiagnostics(integration, gps, 0).points[4].status, 'valid');
  gps.points[4].accelNorth *= 0.99;
  assert.equal(computeAttitudeDiagnostics(integration, gps, 0).points[4].status, 'weak-force');
});

test('IMU interpolates at GPS epochs, clips to start/overlap, never extrapolates', () => {
  const { integration, gps } = fixture();
  integration.startIndex = 2;
  integration.accelNorth = integration.time.map(t => 2 * t);
  gps.points = [-1, 0, 2, 2.5, 3.5, 7.5, 8, 9].map(sensorTime => ({
    sensorTime, accelNorth: 2 * sensorTime, accelWest: 0, accelUp: 0
  }));
  const result = computeAttitudeDiagnostics(integration, gps, 0);
  assert.equal(result.timeOrigin, 2);
  assert.deepEqual(result.points.map(p => p.sensorTime), [2, 2.5, 3.5, 7.5, 8]);
  near(result.points[1].imuForce.x, 5);
  near(result.points[1].residualMagnitude, 0);
});

test('GPS gaps invalidate the full centered processing support, with null plot values', () => {
  const { integration, gps } = fixture(20);
  gps.points = [0, 1, 2, 3, 10, 11, 12, 13, 14].map(sensorTime => ({
    sensorTime, accelNorth: 0, accelWest: 0, accelUp: 0
  }));
  const result = computeAttitudeDiagnostics(integration, gps, 1);
  for (const t of [3, 10]) {
    const point = result.points.find(p => p.sensorTime === t);
    assert.equal(point.status, 'gps-edge-or-gap');
    assert.equal(point.imuForce, null);
    assert.equal(point.angleDegrees, null);
  }
  assert.equal(result.points.find(p => p.sensorTime === 11).status, 'valid');
});

test('IMU gaps are not bridged even when GPS is continuous', () => {
  const { integration, gps } = fixture(20);
  integration.time = [0, 1, 2, 10, 11, 12, 13, 14];
  integration.accelNorth = integration.time.map(() => 0);
  integration.accelWest = integration.time.map(() => 0);
  integration.accelUp = integration.time.map(() => 0);
  const result = computeAttitudeDiagnostics(integration, gps, 0);
  const gap = result.points.find(p => p.sensorTime === 5);
  assert.equal(gap.status, 'imu-gap-or-invalid');
  assert.equal(gap.residualMagnitude, null);
});

test('28-sample GPS half-window excludes exact production edges and full gap support', () => {
  const { integration } = fixture(400);
  const { gps } = fixture(200);
  for (let i = 100; i < gps.points.length; i++) gps.points[i].sensorTime += 100;
  const result = computeAttitudeDiagnostics(integration, gps, 28);
  assert.equal(result.validCount, 88);
  assert.equal(result.excludedCount, 112);
  assert.equal(result.points[71].status, 'valid');
  assert.equal(result.points[72].status, 'gps-edge-or-gap');
  assert.equal(result.points[127].status, 'gps-edge-or-gap');
  assert.equal(result.points[128].status, 'valid');
});

test('nonfinite sensor samples are explicitly excluded, not replaced with zeros', () => {
  const { integration, gps } = fixture();
  integration.accelNorth[4] = NaN;
  gps.points[6].accelUp = Infinity;
  const result = computeAttitudeDiagnostics(integration, gps, 0);
  assert.equal(result.points[4].status, 'imu-gap-or-invalid');
  assert.equal(result.points[6].status, 'gps-edge-or-gap');
  assert.equal(result.points[4].angleDegrees, null);
  assert.ok(result.excludedCount >= 2);
});

test('errors explicitly report absent overlap, short data, bad timestamps, and processing edges', () => {
  const { integration, gps } = fixture();
  assert.throws(() => computeAttitudeDiagnostics(integration, { points: [] }, 0), /at least two/);
  assert.throws(() => computeAttitudeDiagnostics(integration, gps, 10), /No usable/);
  assert.throws(() => computeAttitudeDiagnostics(integration, gps, -1), /processing window/);
  assert.throws(() => computeAttitudeDiagnostics({ ...integration, accelNorth: [] }, gps, 0), /arrays/);
  const duplicated = { points: gps.points.map(p => ({ ...p, sensorTime: 0 })) };
  assert.throws(() => computeAttitudeDiagnostics(integration, duplicated, 0), /strictly increasing/);
  const moved = { points: gps.points.map(p => ({ ...p, sensorTime: p.sensorTime + 100 })) };
  assert.throws(() => computeAttitudeDiagnostics(integration, moved, 0), /No GPS\/IMU overlap/);
  integration.time[4] = integration.time[3];
  assert.throws(() => computeAttitudeDiagnostics(integration, gps, 0), /strictly increasing/);
});

test('selected-source provenance is preserved and does not change the comparison math', () => {
  const { integration, gps } = fixture();
  const device = computeAttitudeDiagnostics(integration, gps, 0);
  const viewer = computeAttitudeDiagnostics({ ...integration, orientationSource: 'viewer' }, gps, 0);
  assert.deepEqual(viewer, device);
});
