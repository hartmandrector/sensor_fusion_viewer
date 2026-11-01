import assert from 'node:assert/strict';
import { test } from 'node:test';
import {
  CHI2_3DOF, resetJacobian, updateVelocity, velocityMeasurementFromGnss,
} from '../src/eskf/update.ts';
import { GpsVelocityEskf, levelAttitude } from '../src/eskf/filter.ts';
import { initialCovariance } from '../src/eskf/errorState.ts';
import { covarianceHealth, getBlock3, identity, matMul, transpose } from '../src/eskf/linalg.ts';
import {
  quatAngleBetween, quatConjugate, quatExp, quatLog, quatMultiply, quatRotate, quatToMatrix,
} from '../src/eskf/quaternion.ts';
import { mulMat3Vec3, transpose33 } from '../src/eskf/math3.ts';

const N = 15;
const G0 = 9.80665;
const ZERO_NOISE = { accelNoiseDensity: 0, gyroNoiseDensity: 0, gyroBiasRandomWalk: 0, accelBiasRandomWalk: 0 };
const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);
const diagR = s => [s * s, 0, 0, 0, s * s, 0, 0, 0, s * s];
const UNC = { position: 1, velocity: 0.5, attitude: [0.05, 0.05, 0.05], gyroBias: 0.005, accelBias: 0.1 };

function mulberry32(seed) {
  return () => {
    seed |= 0; seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
function gaussian(rand) {
  return Math.sqrt(-2 * Math.log(rand() + 1e-300)) * Math.cos(2 * Math.PI * rand());
}

const baseState = (overrides = {}, unc = UNC) => ({
  nominal: { p: [0, 0, 0], v: [0, 0, 0], q: [1, 0, 0, 0], gyroBias: [0, 0, 0], accelBias: [0, 0, 0], ...overrides },
  P: initialCovariance(unc),
});

/** Constant attitude, constant navigation acceleration aN. */
function constantAccelTruth(q, aN) {
  const fBody = mulMat3Vec3(transpose33(quatToMatrix(q)), [aN[0], aN[1], aN[2] + G0]);
  return {
    imu: { gyro: [0, 0, 0], specificForce: fBody },
    imuAt: () => ({ gyro: [0, 0, 0], specificForce: fBody }),
    velocity: t => aN.map(a => a * t),
  };
}

/** Constant attitude, horizontal acceleration of magnitude A turning at rate w. */
function turningAccelTruth(q, A, w) {
  const Rt = transpose33(quatToMatrix(q));
  return {
    imuAt: t => ({ gyro: [0, 0, 0], specificForce: mulMat3Vec3(Rt, [A * Math.cos(w * t), A * Math.sin(w * t), G0]) }),
    velocity: t => [A / w * Math.sin(w * t), A / w * (1 - Math.cos(w * t)), 0],
  };
}

test('GNSS NED fix becomes NWU with declared sAcc-based covariance', () => {
  const r = velocityMeasurementFromGnss({ velN: 1, velE: 2, velD: 3, sAcc: 0.4, numSV: 12 }, 5);
  assert.ok(r.ok);
  assert.deepEqual(r.measurement.velocity, [1, -2, -3]);
  close(r.measurement.covariance[0], 0.16, 1e-12);
  close(r.measurement.covariance[8], 0.16, 1e-12);
  const floored = velocityMeasurementFromGnss({ velN: 0, velE: 0, velD: 0, sAcc: 0.01, numSV: 12 }, 0);
  close(floored.measurement.covariance[4], 0.05 ** 2, 1e-12);
  assert.equal(velocityMeasurementFromGnss({ velN: 0, velE: 0, velD: 0, sAcc: 0.3, numSV: 4 }, 0).ok, false);
  assert.equal(velocityMeasurementFromGnss({ velN: 0, velE: 0, velD: 0, sAcc: 5, numSV: 12 }, 0).ok, false);
  assert.equal(velocityMeasurementFromGnss({ velN: NaN, velE: 0, velD: 0, sAcc: 0.3, numSV: 12 }, 0).ok, false);
  assert.equal(velocityMeasurementFromGnss({ velN: 0, velE: 0, velD: 0, sAcc: 0, numSV: 12 }, 0).ok, false);
});

test('update at truth leaves the nominal state unchanged and shrinks velocity variance', () => {
  const s = baseState({ v: [3, -1, 2], q: quatExp([0.2, 0.1, -0.4]) });
  const r = updateVelocity(s, { time: 0, velocity: [3, -1, 2], covariance: diagR(0.1) });
  assert.ok(r.accepted);
  assert.deepEqual(r.state.nominal.v, [3, -1, 2]);
  assert.ok(quatAngleBetween(r.state.nominal.q, s.nominal.q) < 1e-15);
  close(r.state.P[3 * N + 3], 0.25 * 0.01 / 0.26, 1e-12, 'scalar posterior variance');
  const h = covarianceHealth(r.state.P, N);
  assert.ok(h.positiveDefinite && h.maxAsymmetry === 0);
});

test('uncorrelated velocity update matches the scalar Kalman gain', () => {
  const s = baseState();
  const r = updateVelocity(s, { time: 0, velocity: [1, -2, 0.5], covariance: diagR(0.5) });
  const k = 0.25 / (0.25 + 0.25);
  [1, -2, 0.5].forEach((z, i) => close(r.state.nominal.v[i], k * z, 1e-12));
  close(r.nis, (1 + 4 + 0.25) / 0.5, 1e-12);
  assert.deepEqual(r.state.nominal.p, [0, 0, 0], 'no cross-covariance, no position change');
});

test('Joseph form equals (I−KH)P for the optimal gain when no attitude reset occurs', () => {
  const rand = mulberry32(7);
  const A = Float64Array.from({ length: N * N }, () => gaussian(rand) * 0.3);
  const P = matMul(A, transpose(A, N, N), N, N, N);
  for (let i = 0; i < N; i++) P[i * N + i] += 0.1;
  const s = { nominal: baseState().nominal, P };
  const Rm = diagR(0.2);
  const r = updateVelocity(s, { time: 0, velocity: [0, 0, 0], covariance: Rm });
  // Standard form: P − P Hᵀ S⁻¹ H P, with 3×3 S inverted directly.
  const S = [...getBlock3(P, N, 3, 3)].map((v, i) => v + Rm[i]);
  const det = S[0] * (S[4] * S[8] - S[5] * S[7]) - S[1] * (S[3] * S[8] - S[5] * S[6]) + S[2] * (S[3] * S[7] - S[4] * S[6]);
  const Si = [
    S[4] * S[8] - S[5] * S[7], S[2] * S[7] - S[1] * S[8], S[1] * S[5] - S[2] * S[4],
    S[5] * S[6] - S[3] * S[8], S[0] * S[8] - S[2] * S[6], S[2] * S[3] - S[0] * S[5],
    S[3] * S[7] - S[4] * S[6], S[1] * S[6] - S[0] * S[7], S[0] * S[4] - S[1] * S[3],
  ].map(v => v / det);
  const PHt = new Float64Array(N * 3);
  for (let i = 0; i < N; i++) for (let j = 0; j < 3; j++) PHt[i * 3 + j] = P[i * N + 3 + j];
  const K = matMul(PHt, Float64Array.from(Si), N, 3, 3);
  const KHP = matMul(K, transpose(PHt, N, 3), N, 3, N);
  for (let i = 0; i < N * N; i++) close(r.state.P[i], P[i] - KHP[i], 1e-10, `P[${i}]`);
});

test('reset Jacobian matches the numerical derivative of the attitude reset', () => {
  const a = [0.05, -0.08, 0.12];
  const G = resetJacobian(a);
  const h = 1e-6;
  const reset = eps => quatLog(quatMultiply(quatConjugate(quatExp(a)), quatExp(a.map((v, i) => v + eps[i]))));
  for (let j = 0; j < 3; j++) {
    const e = [0, 0, 0]; e[j] = h;
    const em = [0, 0, 0]; em[j] = -h;
    const col = reset(e).map((v, i) => (v - reset(em)[i]) / (2 * h));
    for (let i = 0; i < 3; i++) close(G[(6 + i) * N + 6 + j], col[i], 1e-8, `G[${i},${j}]`);
  }
  for (let i = 0; i < N; i++) {
    if (i >= 6 && i < 9) continue;
    close(G[i * N + i], 1, 0);
  }
});

test('NIS gate rejects outliers and leaves the state untouched', () => {
  const s = baseState({}, { ...UNC, velocity: 0.1 });
  const r = updateVelocity(s, { time: 0, velocity: [5, 0, 0], covariance: diagR(0.1) });
  assert.equal(r.accepted, false);
  assert.match(r.reason, /NIS/);
  assert.ok(r.nis > CHI2_3DOF.p999);
  assert.equal(r.state, s);
});

test('levelAttitude maps the measured specific force to navigation Up and applies yaw', () => {
  for (const f of [[0, 0, 9.8], [0.3, -9.7, 1.2], [-9.8, 0, 0], [0, 0, -9.8]]) {
    const q = levelAttitude(f, 0.7);
    const up = quatRotate(q, f).map(v => v / Math.hypot(...f));
    close(up[2], 1, 1e-12, `f=${f}`);
  }
  const q = levelAttitude([0, 0, 9.8], Math.PI / 2);
  const x = quatRotate(q, [1, 0, 0]);
  close(x[1], 1, 1e-12, 'body X yawed +90° points West');
});

function runFilter({ qTrue, qInit, aN, truth: truthIn, seconds, gpsRate = 5, imuRate = 100, unc = UNC, noise = ZERO_NOISE, R = 0.05 }) {
  const truth = truthIn ?? constantAccelTruth(qTrue, aN);
  const f = new GpsVelocityEskf({ noise });
  f.initialize({ time: 0, imu: truth.imuAt(0), attitude: qInit, velocity: truth.velocity(0), uncertainty: unc });
  const imuDt = 1 / imuRate;
  let nextGps = 1 / gpsRate;
  for (let k = 1; k <= seconds * imuRate; k++) {
    const t = k * imuDt;
    while (nextGps <= t + 1e-12) {
      f.processVelocity({ time: nextGps, velocity: truth.velocity(nextGps), covariance: diagR(R) });
      nextGps += 1 / gpsRate;
    }
    f.processImu(t, truth.imuAt(t));
  }
  return f;
}

test('gate: truth-initialized filter with exact measurements stays at truth', () => {
  const qTrue = quatExp([0.3, -0.2, 0.9]);
  const f = runFilter({ qTrue, qInit: qTrue, aN: [1.5, -0.5, 0.2], seconds: 10 });
  const s = f.currentState.nominal;
  assert.equal(f.status, 'aided');
  ['0', '1', '2'].forEach(i => close(s.v[i], [15, -5, 2][i], 1e-8, 'velocity'));
  ['0', '1', '2'].forEach(i => close(s.p[i], [75, -25, 10][i], 1e-6, 'position'));
  assert.ok(quatAngleBetween(s.q, qTrue) < 1e-9);
  assert.ok(f.events.filter(e => e.kind === 'update').every(e => e.accepted));
});

function sigmaAlong(state, axisNav) {
  const u = mulMat3Vec3(transpose33(quatToMatrix(state.nominal.q)), axisNav);
  const Ptt = getBlock3(state.P, N, 6, 6);
  let v = 0;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) v += u[i] * Ptt[i * 3 + j] * u[j];
  return Math.sqrt(v);
}
const headingSigma = state => sigmaAlong(state, [0, 0, 1]);

const YAW_ERR = 0.15;
const Q_TRUE = quatExp([0.1, -0.05, 0.6]);
const Q_YAWED = quatMultiply([Math.cos(YAW_ERR / 2), 0, 0, Math.sin(YAW_ERR / 2)], Q_TRUE);
const UNC_ALIGN = { ...UNC, attitude: [0.2, 0.2, 0.2], accelBias: 0.01 };

test('constant specific force leaves rotation about that force unobservable', () => {
  const aN = [3, 1, 0];
  const f = runFilter({ qTrue: Q_TRUE, qInit: Q_YAWED, aN, seconds: 20, unc: UNC_ALIGN });
  const fN = [3, 1, G0];
  const fHat = fN.map(v => v / Math.hypot(...fN));
  const residual = quatAngleBetween(f.currentState.nominal.q, Q_TRUE);
  close(residual, YAW_ERR * fHat[2], 1e-3, 'only the component perpendicular to f is corrected');
  assert.ok(sigmaAlong(f.currentState, fHat) > 0.9 * 0.2, 'no false confidence about the force axis');
});

test('heading error is corrected when the specific-force direction changes', () => {
  const truth = turningAccelTruth(Q_TRUE, 3, 2 * Math.PI / 10);
  const f = runFilter({ qTrue: Q_TRUE, qInit: Q_YAWED, truth, seconds: 20, unc: UNC_ALIGN });
  const err = quatAngleBetween(f.currentState.nominal.q, Q_TRUE);
  assert.ok(err < 0.01, `attitude error ${err}`);
  assert.ok(headingSigma(f.currentState) < 0.05, `heading sigma ${headingSigma(f.currentState)}`);
  assert.ok(f.events.filter(e => e.kind === 'update').every(e => e.accepted));
});

test('gate: heading is not falsely confident when it is unobservable (no horizontal force)', () => {
  const qTrue = Q_TRUE;
  const yawErr = YAW_ERR;
  const qInit = Q_YAWED;
  const unc = UNC_ALIGN;
  const f = runFilter({ qTrue, qInit, aN: [0, 0, 0], seconds: 20, unc });
  const sigma = headingSigma(f.currentState);
  assert.ok(sigma > 0.9 * 0.2, `heading sigma ${sigma} collapsed without information`);
  assert.ok(sigma >= yawErr, 'true heading error remains inside one sigma');
  // Tilt is observable through gravity and is still corrected.
  const upTrue = quatRotate(quatConjugate(qTrue), [0, 0, 1]);
  const upEst = quatRotate(quatConjugate(f.currentState.nominal.q), [0, 0, 1]);
  const tilt = Math.acos(Math.min(1, upTrue.reduce((s, v, i) => s + v * upEst[i], 0)));
  assert.ok(tilt < 1e-3, `tilt error ${tilt}`);
});

test('Monte Carlo: NIS of a correctly modeled noisy run is consistent with 3 DOF', () => {
  const noise = { accelNoiseDensity: 2e-3, gyroNoiseDensity: 2e-4, gyroBiasRandomWalk: 0, accelBiasRandomWalk: 0 };
  const imuRate = 100;
  const imuDt = 1 / imuRate;
  const R = 0.1;
  const nisValues = [];
  for (let run = 0; run < 8; run++) {
    const rand = mulberry32(100 + run);
    const qTrue = quatExp([0.1 * run, -0.2, 0.3]);
    const truthImu = constantAccelTruth(qTrue, [0, 0, 0]).imu;
    const f = new GpsVelocityEskf({ noise });
    const unc = { position: 0.01, velocity: 0.01, attitude: [1e-4, 1e-4, 1e-4], gyroBias: 1e-6, accelBias: 1e-6 };
    f.initialize({ time: 0, imu: truthImu, attitude: qTrue, velocity: [0, 0, 0], uncertainty: unc });
    // Simulate truth with the same discrete noise model the filter assumes.
    let qT = qTrue;
    let vT = [0, 0, 0];
    const sa = noise.accelNoiseDensity / Math.sqrt(imuDt);
    const sg = noise.gyroNoiseDensity / Math.sqrt(imuDt);
    let nextGps = 0.2;
    for (let k = 1; k <= 30 * imuRate; k++) {
      const t = k * imuDt;
      // The filter sees a constant IMU; truth = measurement − white noise.
      const omegaTrue = [0, 0, 0].map(() => -gaussian(rand) * sg);
      const forceTrue = truthImu.specificForce.map(v => v - gaussian(rand) * sa);
      const aN = quatRotate(qT, forceTrue);
      aN[2] -= G0;
      vT = vT.map((v, i) => v + aN[i] * imuDt);
      qT = quatMultiply(qT, quatExp(omegaTrue.map(w => w * imuDt)));
      if (t + 1e-12 >= nextGps) {
        const z = vT.map(v => v + gaussian(rand) * R);
        f.processVelocity({ time: nextGps, velocity: z, covariance: diagR(R) });
        nextGps += 0.2;
      }
      f.processImu(t, truthImu);
    }
    for (const e of f.events) if (e.kind === 'update' && e.time > 2) nisValues.push(e.nis);
  }
  const mean = nisValues.reduce((s, v) => s + v, 0) / nisValues.length;
  // 95% interval for the mean of n χ²₃ samples ≈ 3 ± 1.96·√(6/n).
  const half = 1.96 * Math.sqrt(6 / nisValues.length);
  assert.ok(Math.abs(mean - 3) < half * 1.5, `mean NIS ${mean.toFixed(3)} over ${nisValues.length} updates`);
});

test('lifecycle: duplicates, stale fixes, coasting, recovery, and gaps', () => {
  const imu = { gyro: [0, 0, 0], specificForce: [0, 0, G0] };
  const f = new GpsVelocityEskf({ coastingAfterSeconds: 0.5 });
  assert.equal(f.processImu(0.01, imu), false, 'not initialized');
  f.initialize({ time: 0, imu, attitude: [1, 0, 0, 0], velocity: [0, 0, 0], uncertainty: UNC });
  const m = t => ({ time: t, velocity: [0, 0, 0], covariance: diagR(0.1) });
  for (let k = 1; k <= 10; k++) f.processImu(k * 0.01, imu);
  assert.equal(f.processVelocity(m(0.105)), true, 'between IMU samples');
  close(f.currentTime, 0.105, 1e-12);
  assert.equal(f.processVelocity(m(0.105)), false, 'duplicate');
  for (let k = 11; k <= 20; k++) f.processImu(k * 0.01, imu);
  assert.equal(f.processVelocity(m(0.15)), false, 'stale');
  for (let k = 21; k <= 80; k++) f.processImu(k * 0.01, imu);
  assert.equal(f.status, 'coasting');
  assert.equal(f.processVelocity(m(0.8)), true);
  assert.equal(f.status, 'aided');
  assert.equal(f.processImu(1.5, imu), false, 'gap');
  assert.equal(f.status, 'unhealthy');
  assert.equal(f.processVelocity(m(1.6)), false, 'unhealthy filter ignores updates');
  const reasons = f.events.filter(e => e.kind === 'status').map(e => e.to);
  assert.deepEqual(reasons, ['aided', 'coasting', 'aided', 'unhealthy']);
});

test('lifecycle: persistent rejections make the filter unhealthy', () => {
  const imu = { gyro: [0, 0, 0], specificForce: [0, 0, G0] };
  const f = new GpsVelocityEskf({ maxConsecutiveRejections: 3 });
  f.initialize({ time: 0, imu, attitude: [1, 0, 0, 0], velocity: [0, 0, 0], uncertainty: UNC });
  for (let k = 1; k <= 3; k++) {
    f.processImu(k * 0.01, imu);
    f.processVelocity({ time: k * 0.01, velocity: [50, 0, 0], covariance: diagR(0.1) });
  }
  assert.equal(f.status, 'unhealthy');
});

test('resetJacobian is identity for zero correction', () => {
  const G = resetJacobian([0, 0, 0]);
  const I = identity(N);
  for (let i = 0; i < N * N; i++) close(G[i], I[i], 0);
});
