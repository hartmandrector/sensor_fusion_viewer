import assert from 'node:assert/strict';
import { test } from 'node:test';
import { estimateGPSLag } from '../src/gpsLag.ts';
import { computeAttitudeDiagnostics } from '../src/attitudeDiagnostics.ts';

function fixture(delay, constant = false) {
  const signal = t => constant ? 10 : 10 + Math.sin(t * 1.7) + 0.6 * Math.sin(t * 3.13) + 0.3 * Math.cos(t * 0.31);
  return {
    timeOrigin: 100,
    points: Array.from({ length: 801 }, (_, i) => {
      const t = i * 0.05;
      return {
        sensorTime: 100 + t, imuForce: { x: 0, y: 0, z: signal(t) },
        gpsForce: { x: 0, y: 0, z: signal(t - delay) }, status: 'valid'
      };
    })
  };
}

for (const delay of [1, -0.6, 0]) {
  for (const signal of ['up', 'magnitude']) {
    test(`${signal}: delayed GPS ${delay}s needs shift ${-delay}s`, () => {
      const result = estimateGPSLag(fixture(delay), 5, 35, signal);
      assert.ok(Math.abs(result.shiftSeconds + delay) < 0.021);
      assert.ok(result.correlation > 0.99);
      assert.ok(result.sampleCount >= 30);
    });
  }
}

test('flat signals and invalid intervals are explicitly rejected', () => {
  assert.throws(() => estimateGPSLag(fixture(0, true), 5, 35, 'up'), /varies by less/);
  assert.throws(() => estimateGPSLag(fixture(0), 35, 5, 'up'), /start < end/);
  assert.throws(() => estimateGPSLag(fixture(0), 90, 100, 'up'), /30 common/);
});

test('gap samples are not filled and all candidates use common epochs', () => {
  const data = fixture(1);
  for (let i = 250; i < 400; i++) data.points[i].gpsForce = null;
  const result = estimateGPSLag(data, 5, 35, 'up');
  assert.ok(Math.abs(result.shiftSeconds + 1) < 0.021);
  assert.ok(result.sampleCount < 600);
});

test('constant GPS is rejected and boundary peaks are flagged', () => {
  const data = fixture(0);
  for (const point of data.points) point.gpsForce.z = 10;
  assert.throws(() => estimateGPSLag(data, 5, 35, 'up'), /insufficient variation/);
  const result = estimateGPSLag(fixture(2), 5, 35, 'up');
  assert.equal(result.shiftSeconds, -2);
  assert.ok(result.warnings.includes('Peak at search boundary'));
});

test('diagnostic shift moves GPS epochs only, preserving original input and clipping overlap', () => {
  const time = [0, 1, 2, 3, 4];
  const integration = { time, startIndex: 0, accelNorth: time, accelWest: time.map(() => 0), accelUp: time.map(() => 0) };
  const gps = { points: time.map(t => ({ sensorTime: t, accelNorth: t, accelWest: 0, accelUp: 0 })) };
  const saved = structuredClone(gps);
  const shifted = computeAttitudeDiagnostics(integration, gps, 0, -1);
  assert.deepEqual(shifted.points.map(p => p.sensorTime), [0, 1, 2, 3]);
  assert.equal(shifted.points[1].gpsForce.x, 2);
  assert.equal(shifted.points[1].imuForce.x, 1);
  assert.deepEqual(gps, saved);
});
