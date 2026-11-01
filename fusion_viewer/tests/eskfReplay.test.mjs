import assert from 'node:assert/strict';
import { test } from 'node:test';
import {
  calibrateImu, deviceToEskfQuaternion, eskfToViewerQuaternion, IDENTITY_REMAP, P_ALGO_FROM_BODY,
  remapMatrix, viewerToEskfQuaternion,
} from '../src/eskf/calibration.ts';
import {
  firmwareEnuToNwu, quatAngleBetween, quatExp, quatFromMatrix, quatMultiply, quatNormalize, quatToMatrix,
} from '../src/eskf/quaternion.ts';
import { mul33, mulMat3Vec3, transpose33 } from '../src/eskf/math3.ts';
import { DEFAULT_REPLAY_OPTIONS, runEskfReplay, setBodyYHeading } from '../src/eskf/replay.ts';
import { getDisplayOrientation } from '../src/orientationSource.ts';

const G0 = 9.80665;
const DEG = Math.PI / 180;
const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);
const closeArr = (a, b, tol, msg) => a.forEach((x, i) => close(x, b[i], tol, `${msg ?? ''}[${i}]`));
const IDENTITY_CAL = { gyroBiasDps: [0, 0, 0], gyroScale: 1, accelBiasG: [0, 0, 0], accelScaleMatrix: null, axisRemap: IDENTITY_REMAP };
/** Body +Y North, +X East, +Z Up. */
const Q_Y_NORTH = [Math.SQRT1_2, 0, 0, -Math.SQRT1_2];

test('quatFromMatrix inverts quatToMatrix (up to sign) across all branches', () => {
  const cases = [[0.1, 0.2, 0.3], [3.0, 0, 0], [0, 3.1, 0.1], [0.05, 0, 3.1], [2, -2, 1]];
  for (const phi of cases) {
    const q = quatExp(phi);
    const back = quatFromMatrix(quatToMatrix(q));
    close(quatAngleBetween(q, back), 0, 1e-9, `phi ${phi}`);
  }
});

test('remapMatrix accepts proper rotations and rejects reflections', () => {
  assert.deepEqual(remapMatrix(IDENTITY_REMAP), [1, 0, 0, 0, 1, 0, 0, 0, 1]);
  assert.deepEqual(remapMatrix({ bodyX: '-Y', bodyY: '+X', bodyZ: '+Z' }), [0, -1, 0, 1, 0, 0, 0, 0, 1]);
  assert.throws(() => remapMatrix({ bodyX: '-X', bodyY: '+Y', bodyZ: '+Z' }), /proper rotation/);
  assert.throws(() => remapMatrix({ bodyX: '+X', bodyY: '+X', bodyZ: '+Z' }), /proper rotation/);
});

test('calibrateImu applies bias, gyro scale, accel matrix, remap, and SI units in viewer order', () => {
  const cal = {
    gyroBiasDps: [1, 2, 3], gyroScale: 1.14, accelBiasG: [0.01, -0.02, 0.03],
    accelScaleMatrix: [2, 0, 0, 0, 1, 0, 0, 0, 1], axisRemap: { bodyX: '-Y', bodyY: '+X', bodyZ: '+Z' },
  };
  const m = calibrateImu([11, 2, 3], [0.51, 0.98, 0.03], cal);
  // gyro: (10,0,0)·1.14 → remap (−y, x, z) = (0, 11.4, 0) °/s
  closeArr(m.gyro, [0, 11.4 * DEG, 0], 1e-12);
  // accel: (0.5, 1, 0) → S → (1, 1, 0) → remap (−1, 1, 0) g
  closeArr(m.specificForce, [-G0, G0, 0], 1e-9);
});

test('viewer quaternion conversion matches the adapter body→algorithm permutation', () => {
  const qEskf = quatExp([0.3, -0.7, 1.2]);
  const qViewer = eskfToViewerQuaternion(qEskf);
  // Adapter: v_NWU = R(q_viewer)·(P·v_B); ESKF: v_NWU = R(q_eskf)·v_B.
  closeArr(mul33(quatToMatrix(qViewer), P_ALGO_FROM_BODY), quatToMatrix(qEskf), 1e-12);
  close(quatAngleBetween(viewerToEskfQuaternion(qViewer), qEskf), 0, 1e-12);
});

test('device ENU quaternion maps to the remapped ESKF body frame', () => {
  const qDev = quatExp([0.2, 0.4, -0.9]);
  close(quatAngleBetween(deviceToEskfQuaternion(qDev, IDENTITY_REMAP), firmwareEnuToNwu(qDev)), 0, 1e-12);
  const remap = { bodyX: '+Z', bodyY: '+X', bodyZ: '+Y' };
  const qB = deviceToEskfQuaternion(qDev, remap);
  const s = [0.3, -0.5, 0.8];
  const viaSensor = mulMat3Vec3(quatToMatrix(firmwareEnuToNwu(qDev)), s);
  const viaBody = mulMat3Vec3(quatToMatrix(qB), mulMat3Vec3(remapMatrix(remap), s));
  closeArr(viaBody, viaSensor, 1e-12);
});

test('device→ESKF→viewer chain equals the verified device display conversion', () => {
  for (const phi of [[0, 0, 0], [0.3, 0.1, -0.2], [1, 2, -0.5], [0, 0, 1.5]]) {
    const q = quatExp(phi);
    const shown = getDisplayOrientation({ imu: { deviceQuaternion: { w: q[0], x: q[1], y: q[2], z: q[3] } } }, 'device').quaternion;
    const viaEskf = eskfToViewerQuaternion(deviceToEskfQuaternion(q, IDENTITY_REMAP));
    close(quatAngleBetween([shown.w, shown.x, shown.y, shown.z], viaEskf), 0, 1e-9, `phi ${phi}`);
  }
});

test('setBodyYHeading points body +Y at the requested heading, clockwise from North', () => {
  const level = [1, 0, 0, 0];
  close(quatAngleBetween(setBodyYHeading(level, 0), Q_Y_NORTH), 0, 1e-12);
  const east = mulMat3Vec3(quatToMatrix(setBodyYHeading(level, 90)), [0, 1, 0]);
  closeArr(east, [0, -1, 0], 1e-12, 'heading 90 → East = −West');
  const tilted = quatMultiply(quatExp([0, 0, 0.4]), quatExp([0.3, 0, 0]));
  const y = mulMat3Vec3(quatToMatrix(setBodyYHeading(tilted, 45)), [0, 1, 0]);
  close(Math.atan2(-y[1], y[0]) / DEG, 45, 1e-9);
});

/** Static for `staticSeconds`, then a horizontal acceleration of magnitude A turning at rate w. */
function simulate({ qTrue, seconds = 20, staticSeconds = 2, A = 3, w = 2 * Math.PI / 8, imuRate = 100, gpsRate = 5, gyroBiasDps = [0, 0, 0] }) {
  const Rt = transpose33(quatToMatrix(qTrue));
  const aN = t => (t < staticSeconds ? [0, 0, 0] : [A * Math.cos(w * (t - staticSeconds)), A * Math.sin(w * (t - staticSeconds)), 0]);
  const vN = t => (t < staticSeconds ? [0, 0, 0]
    : [A / w * Math.sin(w * (t - staticSeconds)), A / w * (1 - Math.cos(w * (t - staticSeconds))), 0]);
  const imu = [];
  for (let k = 0; k <= seconds * imuRate; k++) {
    const t = k / imuRate;
    const a = aN(t);
    const f = mulMat3Vec3(Rt, [a[0], a[1], a[2] + G0]);
    imu.push({ time: t, gyroDps: [...gyroBiasDps], accelG: f.map(x => x / G0) });
  }
  const gnss = [];
  for (let k = 0; k <= seconds * gpsRate; k++) {
    const t = k / gpsRate;
    const v = vN(t);
    gnss.push({ time: t, velN: v[0], velE: -v[1], velD: -v[2], sAcc: 0.05, numSV: 12 });
  }
  return { imu, gnss };
}

test('replay: level seed with a wrong heading converges once horizontal force turns', () => {
  const { imu, gnss } = simulate({ qTrue: Q_Y_NORTH });
  const result = runEskfReplay(imu, gnss, IDENTITY_CAL, {
    ...DEFAULT_REPLAY_OPTIONS, startTime: 1.5, seed: { kind: 'level', headingDeg: 20 },
  });
  assert.equal(result.frames.length, imu.length);
  close(result.summary.initTime, 1.6, 1e-9);
  assert.equal(result.frames[0].status, 'not-initialized');
  assert.equal(result.frames.at(-1).status, 'aided');
  assert.equal(result.summary.reinitializations, 0);
  // Pre-init frames hold the initial attitude, not identity.
  close(quatAngleBetween(result.frames[0].q, result.frames[170].q), 0, 0.01);
  const final = result.frames.at(-1);
  const err = quatAngleBetween(final.q, Q_Y_NORTH) / DEG;
  assert.ok(err < 1, `final attitude error ${err}°`);
  assert.ok(final.headingSigma / DEG < 5, `heading σ ${final.headingSigma / DEG}°`);
  assert.ok(result.summary.acceptedUpdates > 80);
  assert.equal(result.warnings.length, 0);
});

test('replay: heading σ stays large while only gravity is sensed', () => {
  const { imu, gnss } = simulate({ qTrue: Q_Y_NORTH, staticSeconds: 30, seconds: 10 });
  const result = runEskfReplay(imu, gnss, IDENTITY_CAL, {
    ...DEFAULT_REPLAY_OPTIONS, startTime: 1.5, seed: { kind: 'level', headingDeg: 20 },
  });
  const final = result.frames.at(-1);
  assert.ok(final.headingSigma / DEG > 25, `heading σ ${final.headingSigma / DEG}° should remain near the 30° prior`);
  assert.ok(final.tiltSigma / DEG < 2, `tilt σ ${final.tiltSigma / DEG}°`);
});

test('replay: attitude seed, GPS shift, and gyro bias estimation', () => {
  const qTrue = quatNormalize(quatMultiply(Q_Y_NORTH, quatExp([0.2, -0.1, 0])));
  const { imu, gnss } = simulate({ qTrue, seconds: 40, gyroBiasDps: [0.3, -0.2, 0.25] });
  const delayed = gnss.map(f => ({ ...f, time: f.time + 0.4 }));
  const result = runEskfReplay(imu, delayed, IDENTITY_CAL, {
    ...DEFAULT_REPLAY_OPTIONS, startTime: 1, gpsTimeShiftSeconds: -0.4,
    seed: { kind: 'attitude', label: 'truth', attitudeAt: () => qTrue },
  });
  assert.equal(result.summary.seedLabel, 'truth');
  assert.equal(result.summary.rejectedUpdates, 0);
  const err = quatAngleBetween(result.frames.at(-1).q, qTrue) / DEG;
  assert.ok(err < 1, `attitude error ${err}°`);
  closeArr(result.summary.finalGyroBiasDps, [0.3, -0.2, 0.25], 0.05, 'gyro bias');
});

test('replay: GNSS pre-rejections are logged and initialization waits for a usable fix', () => {
  const { imu, gnss } = simulate({ qTrue: Q_Y_NORTH, seconds: 6 });
  const degraded = gnss.map(f => (f.time < 3 ? { ...f, numSV: 3 } : f));
  const result = runEskfReplay(imu, degraded, IDENTITY_CAL, {
    ...DEFAULT_REPLAY_OPTIONS, startTime: 1, seed: { kind: 'level', headingDeg: 0 },
  });
  assert.ok(result.summary.gnssRejected >= 10);
  assert.ok(result.summary.initTime >= 3);
  assert.ok(result.events.some(e => e.kind === 'gnss-rejected'));
});

test('replay: throws clearly when no fix is usable or inputs are invalid', () => {
  const { imu, gnss } = simulate({ qTrue: Q_Y_NORTH, seconds: 3 });
  assert.throws(() => runEskfReplay(imu, gnss.map(f => ({ ...f, numSV: 2 })), IDENTITY_CAL, {
    ...DEFAULT_REPLAY_OPTIONS, startTime: 1, seed: { kind: 'level', headingDeg: 0 },
  }), /never initialized/);
  assert.throws(() => runEskfReplay(imu, gnss, { ...IDENTITY_CAL, axisRemap: { bodyX: '-X', bodyY: '+Y', bodyZ: '+Z' } }, {
    ...DEFAULT_REPLAY_OPTIONS, seed: { kind: 'level', headingDeg: 0 },
  }), /proper rotation/);
  assert.throws(() => runEskfReplay(imu, gnss, IDENTITY_CAL, {
    ...DEFAULT_REPLAY_OPTIONS, gpsTimeShiftSeconds: 10, seed: { kind: 'level', headingDeg: 0 },
  }), /±5 s/);
});

test('replay: IMU gap makes the filter unhealthy and it re-initializes from the next fix', () => {
  const { imu, gnss } = simulate({ qTrue: Q_Y_NORTH, seconds: 12 });
  const gapped = imu.filter(s => s.time < 6 || s.time > 6.5);
  const result = runEskfReplay(gapped, gnss, IDENTITY_CAL, {
    ...DEFAULT_REPLAY_OPTIONS, startTime: 1.5, seed: { kind: 'level', headingDeg: 0 },
  });
  assert.equal(result.summary.reinitializations, 1);
  assert.ok(result.summary.secondsByStatus.unhealthy > 0.5);
  assert.equal(result.frames.at(-1).status, 'aided');
  const noReinit = runEskfReplay(gapped, gnss, IDENTITY_CAL, {
    ...DEFAULT_REPLAY_OPTIONS, startTime: 1.5, reinitializeAfterSeconds: null, seed: { kind: 'level', headingDeg: 0 },
  });
  assert.equal(noReinit.frames.at(-1).status, 'unhealthy');
});
