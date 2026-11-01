import assert from 'node:assert/strict';
import { test } from 'node:test';
import { IDENTITY_REMAP } from '../src/eskf/calibration.ts';
import { runDropoutTest } from '../src/eskf/dropout.ts';
import { quatExp, quatMultiply, quatToMatrix } from '../src/eskf/quaternion.ts';
import { mulMat3Vec3, transpose33 } from '../src/eskf/math3.ts';

const G0 = 9.80665;
const DEG = Math.PI / 180;
const CAL = { gyroBiasDps: [0, 0, 0], gyroScale: 1, accelBiasG: [0, 0, 0], accelScaleMatrix: null, axisRemap: IDENTITY_REMAP };
const Q_TRUE = [Math.SQRT1_2, 0, 0, -Math.SQRT1_2];
const A = 3;
const W = 2 * Math.PI / 8;
const V0 = [20, 0, -10];

/** Horizontal acceleration A turning at rate W on top of a constant velocity V0 (NWU); level, constant attitude. */
function simulate(seconds = 30, imuRate = 100, gpsRate = 5) {
  const Rt = transpose33(quatToMatrix(Q_TRUE));
  const vN = t => [V0[0] + A / W * Math.sin(W * t), V0[1] + A / W * (1 - Math.cos(W * t)), V0[2]];
  const imu = [];
  const truth = [];
  for (let k = 0; k <= seconds * imuRate; k++) {
    const t = k / imuRate;
    const f = mulMat3Vec3(Rt, [A * Math.cos(W * t), A * Math.sin(W * t), G0]);
    imu.push({ time: t, gyroDps: [0, 0, 0], accelG: f.map(x => x / G0) });
    truth.push(vN(t));
  }
  const gnss = [];
  for (let k = 0; k <= seconds * gpsRate; k++) {
    const t = k / gpsRate;
    const v = vN(t);
    gnss.push({ time: t, velN: v[0], velE: -v[1], velD: -v[2], sAcc: 0.1, numSV: 12 });
  }
  return { imu, gnss, truth };
}

function framesFrom(imu, truth, q) {
  return imu.map((s, i) => ({
    time: s.time, status: 'aided', q, v: truth[i], p: [0, 0, 0],
    gyroBias: [0, 0, 0], accelBias: [0, 0, 0], force: [0, 0, 0], headingSigma: 0, tiltSigma: 0,
  }));
}

test('dropout: perfect attitude coasts on the IMU far better than holding velocity', () => {
  const { imu, gnss, truth } = simulate();
  const report = runDropoutTest(framesFrom(imu, truth, Q_TRUE), imu, gnss, CAL, { durations: [2, 5] });
  assert.deepEqual(report.durations, [2, 5]);
  assert.ok(report.windows.length > 40, `${report.windows.length} windows`);
  assert.ok(report.windows.every(w => w.moving), 'speed ≈ 22 m/s is moving');
  for (const s of report.all) {
    assert.ok(s.count > 30);
    assert.ok(s.imuRms < 0.02, `${s.duration}s IMU rms ${s.imuRms}`);
    assert.ok(s.holdRms > 2, `${s.duration}s hold rms ${s.holdRms}`);
  }
});

test('dropout: a heading error δ rotates the velocity change: |err| = 2·sin(δ/2)·|Δv|', () => {
  const { imu, gnss, truth } = simulate();
  const q = quatMultiply(quatExp([0, 0, 2 * DEG]), Q_TRUE);
  const report = runDropoutTest(framesFrom(imu, truth, q), imu, gnss, CAL, { durations: [5] });
  // |Δv| over 5 s of a turning acceleration A at rate W: 2A·|sin(W·D/2)|/W.
  const expected = 2 * Math.sin(1 * DEG) * (2 * A * Math.abs(Math.sin(W * 5 / 2)) / W);
  const rms = report.all[0].imuRms;
  assert.ok(Math.abs(rms - expected) < 0.1 * expected, `rms ${rms} vs ${expected}`);
});

test('dropout: GPS shift aligns delayed fixes; static windows are sparse; unaided starts are skipped', () => {
  const { imu, gnss, truth } = simulate();
  const delayed = gnss.map(f => ({ ...f, time: f.time + 1 }));
  const frames = framesFrom(imu, truth, Q_TRUE);
  for (let i = 0; i < 500; i++) frames[i].status = 'coasting';
  const unshifted = runDropoutTest(frames, imu, delayed, CAL, { durations: [2] });
  const shifted = runDropoutTest(frames, imu, delayed, CAL, { durations: [2], gpsTimeShiftSeconds: -1 });
  assert.ok(shifted.all[0].imuRms < 0.02, `shifted rms ${shifted.all[0].imuRms}`);
  assert.ok(unshifted.all[0].imuRms > 1, `unshifted rms ${unshifted.all[0].imuRms}`);
  assert.ok(shifted.windows.every(w => w.startTime >= 5), 'no window starts while coasting');

  const sparse = runDropoutTest(frames, imu, gnss, CAL, { durations: [2], movingSpeed: 100, staticSpacingSeconds: 5 });
  assert.ok(sparse.windows.every(w => !w.moving));
  assert.ok(sparse.windows.length <= 6, `${sparse.windows.length} static windows`);
  assert.equal(sparse.moving[0].count, 0);
});

test('dropout: rejects mismatched inputs and bad durations', () => {
  const { imu, gnss, truth } = simulate(5);
  const frames = framesFrom(imu, truth, Q_TRUE);
  assert.throws(() => runDropoutTest(frames.slice(1), imu, gnss, CAL), /one replay frame per IMU sample/);
  assert.throws(() => runDropoutTest(frames, imu, gnss, CAL, { durations: [0] }), /positive/);
});
