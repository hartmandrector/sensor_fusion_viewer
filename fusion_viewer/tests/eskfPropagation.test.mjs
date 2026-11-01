import assert from 'node:assert/strict';
import { test } from 'node:test';
import {
  DEFAULT_PROPAGATION_OPTIONS, GRAVITY_NWU, errorStateJacobian, propagate, propagateNominal,
} from '../src/eskf/propagation.ts';
import { errorBetween, injectError } from '../src/eskf/errorState.ts';
import { covarianceHealth, identity, matMul, transpose } from '../src/eskf/linalg.ts';
import { quatAngleBetween, quatConjugate, quatExp, quatMultiply, quatNorm, quatRotate, quatToMatrix } from '../src/eskf/quaternion.ts';
import { cross3, mul33, mulMat3Vec3, skew, transpose33 } from '../src/eskf/math3.ts';

const N = 15;
const ZERO_NOISE = { accelNoiseDensity: 0, gyroNoiseDensity: 0, gyroBiasRandomWalk: 0, accelBiasRandomWalk: 0 };
const UP_FORCE = [0, 0, 9.80665];
const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);

const state = (overrides = {}) => ({
  nominal: { p: [0, 0, 0], v: [0, 0, 0], q: [1, 0, 0, 0], gyroBias: [0, 0, 0], accelBias: [0, 0, 0], ...overrides },
  P: new Float64Array(N * N),
});

/** Truth trajectory with constant body rate; specific force keeps the body stationary in N. */
function stationaryRotating(q0, omega, bias = { gyro: [0, 0, 0], accel: [0, 0, 0] }) {
  return t => {
    const qt = quatMultiply(q0, quatExp(omega.map(w => w * t)));
    const fBody = quatRotate(quatConjugate(qt), UP_FORCE);
    return {
      gyro: omega.map((w, i) => w + bias.gyro[i]),
      specificForce: fBody.map((f, i) => f + bias.accel[i]),
      q: qt,
    };
  };
}

function run(initial, sample, dt, steps, noise = ZERO_NOISE) {
  let s = initial;
  for (let k = 0; k < steps; k++) s = propagate(s, sample(k * dt), sample((k + 1) * dt), dt, noise);
  return s;
}

const Q0 = quatExp([0.4, -0.7, 1.1]);

test('gate: stationary at arbitrary attitude, 10 s at 100 Hz', () => {
  const s = run(state({ q: Q0 }), stationaryRotating(Q0, [0, 0, 0]), 0.01, 1000);
  assert.ok(Math.hypot(...s.nominal.v) < 1e-8, `velocity ${s.nominal.v}`);
  assert.ok(Math.abs(quatNorm(s.nominal.q) - 1) < 1e-12);
});

test('gate: stationary while rotating, 10 s at 100 Hz', () => {
  const sample = stationaryRotating(Q0, [0.8, -1.5, 2.3]);
  const s = run(state({ q: Q0 }), sample, 0.01, 1000);
  assert.ok(Math.hypot(...s.nominal.v) < 1e-8, `velocity ${s.nominal.v}`);
  assert.ok(Math.hypot(...s.nominal.p) < 1e-7, `position ${s.nominal.p}`);
  assert.ok(Math.abs(quatNorm(s.nominal.q) - 1) < 1e-12);
  assert.ok(quatAngleBetween(s.nominal.q, sample(10).q) < 1e-10);
});

test('known biases are removed from the measurements', () => {
  const bias = { gyro: [0.02, -0.01, 0.03], accel: [0.3, -0.2, 0.1] };
  const sample = stationaryRotating(Q0, [0.5, 0.2, -1], bias);
  const s = run(state({ q: Q0, gyroBias: bias.gyro, accelBias: bias.accel }), sample, 0.01, 1000);
  assert.ok(Math.hypot(...s.nominal.v) < 1e-8, `velocity ${s.nominal.v}`);
  assert.ok(quatAngleBetween(s.nominal.q, stationaryRotating(Q0, [0.5, 0.2, -1])(10).q) < 1e-10);
});

test('constant navigation acceleration integrates exactly', () => {
  const sample = () => ({ gyro: [0, 0, 0], specificForce: [1, -2, 9.80665 + 0.5] });
  const s = run(state({ v: [3, 0, 0] }), sample, 0.01, 1000);
  [13, -20, 5].forEach((v, i) => close(s.nominal.v[i], v, 1e-9, `v${i}`));
  [80, -100, 25].forEach((p, i) => close(s.nominal.p[i], p, 1e-8, `p${i}`));
});

function rightJacobianInverse(phi) {
  const th = Math.hypot(...phi);
  const S = skew(phi);
  const S2 = mul33(S, S);
  const c = th < 1e-6 ? 1 / 12 : 1 / (th * th) - (1 + Math.cos(th)) / (2 * th * Math.sin(th));
  return [1, 0, 0, 0, 1, 0, 0, 0, 1].map((v, i) => v + 0.5 * S[i] + c * S2[i]);
}

/** Exact nonlinear time derivative of the error state for a nominal/true pair. */
function errorDerivative(x, dx, gyroMeas, forceMeas) {
  const t = injectError(x, dx);
  const w = gyroMeas.map((v, i) => v - x.gyroBias[i]);
  const wt = gyroMeas.map((v, i) => v - t.gyroBias[i]);
  const f = forceMeas.map((v, i) => v - x.accelBias[i]);
  const ft = forceMeas.map((v, i) => v - t.accelBias[i]);
  const dtheta = [dx[6], dx[7], dx[8]];
  const Rd = quatToMatrix(quatExp(dtheta));
  const rateErr = wt.map((v, i) => v - mulMat3Vec3(transpose33(Rd), w)[i]);
  const dvDot = mulMat3Vec3(quatToMatrix(t.q), ft).map((v, i) => v - mulMat3Vec3(quatToMatrix(x.q), f)[i]);
  return [
    ...t.v.map((v, i) => v - x.v[i]),
    ...dvDot,
    ...mulMat3Vec3(rightJacobianInverse(dtheta), rateErr),
    0, 0, 0, 0, 0, 0,
  ];
}

test('gate: analytic F matches finite differences of the nonlinear error dynamics', () => {
  const x = { p: [1, 2, 3], v: [30, -5, -40], q: quatExp([0.3, -0.6, 1.2]), gyroBias: [0.01, -0.02, 0.005], accelBias: [0.2, -0.1, 0.05] };
  const gyroMeas = [0.7, -1.2, 0.4];
  const forceMeas = [2.5, -4, 8.2];
  const omega = gyroMeas.map((v, i) => v - x.gyroBias[i]);
  const force = forceMeas.map((v, i) => v - x.accelBias[i]);
  const F = errorStateJacobian(quatToMatrix(x.q), omega, force);
  const eps = 1e-6;
  for (let j = 0; j < N; j++) {
    const plus = new Float64Array(N); plus[j] = eps;
    const minus = new Float64Array(N); minus[j] = -eps;
    const dp = errorDerivative(x, plus, gyroMeas, forceMeas);
    const dm = errorDerivative(x, minus, gyroMeas, forceMeas);
    for (let i = 0; i < N; i++) {
      const numeric = (dp[i] - dm[i]) / (2 * eps);
      const analytic = F[i * N + j];
      const ok = Math.abs(numeric - analytic) <= Math.max(1e-7, 1e-5 * Math.abs(analytic));
      assert.ok(ok, `F[${i},${j}] analytic ${analytic} numeric ${numeric}`);
    }
  }
});

function timeVaryingSample(t) {
  return { gyro: [Math.sin(2 * t), 0.5 * Math.cos(3 * t), 0.3 + 0.2 * t], specificForce: [1 + Math.sin(t), -2, 9 + Math.cos(2 * t)] };
}

function nominalAfter(dt, T) {
  let x = state({ q: Q0, v: [10, 0, -5] }).nominal;
  const steps = Math.round(T / dt);
  for (let k = 0; k < steps; k++) x = propagateNominal(x, timeVaryingSample(k * dt), timeVaryingSample((k + 1) * dt), dt);
  return x;
}

test('gate: nominal propagation converges at second order with step size', () => {
  const ref = nominalAfter(1e-4, 2);
  const errors = [0.02, 0.01, 0.005].map(dt => {
    const x = nominalAfter(dt, 2);
    return { att: quatAngleBetween(x.q, ref.q), vel: Math.hypot(...x.v.map((v, i) => v - ref.v[i])) };
  });
  for (let k = 0; k < 2; k++) {
    assert.ok(errors[k].att / errors[k + 1].att > 3, `attitude ratio ${errors[k].att / errors[k + 1].att}`);
    assert.ok(errors[k].vel / errors[k + 1].vel > 3, `velocity ratio ${errors[k].vel / errors[k + 1].vel}`);
  }
});

/** Discrete transition of one propagateNominal step, by central differences in the error space. */
function numericPhi(x, s0, s1, dt) {
  const base = propagateNominal(x, s0, s1, dt);
  const Phi = new Float64Array(N * N);
  const eps = 1e-6;
  for (let j = 0; j < N; j++) {
    const plus = new Float64Array(N); plus[j] = eps;
    const minus = new Float64Array(N); minus[j] = -eps;
    const ep = errorBetween(propagateNominal(injectError(x, plus), s0, s1, dt), base);
    const em = errorBetween(propagateNominal(injectError(x, minus), s0, s1, dt), base);
    for (let i = 0; i < N; i++) Phi[i * N + j] = (ep[i] - em[i]) / (2 * eps);
  }
  return Phi;
}

test('gate: covariance transition converges to the true discrete Jacobian with step size', () => {
  const x = { p: [0, 0, 0], v: [30, -5, -40], q: Q0, gyroBias: [0.01, -0.02, 0.005], accelBias: [0.2, -0.1, 0.05] };
  const errs = [0.02, 0.01, 0.005].map(dt => {
    const s0 = timeVaryingSample(0);
    const s1 = timeVaryingSample(dt);
    const PhiNum = numericPhi(x, s0, s1, dt);
    const expected = matMul(PhiNum, transpose(PhiNum, N, N), N, N, N);
    const P1 = propagate({ nominal: x, P: identity(N) }, s0, s1, dt, ZERO_NOISE).P;
    return Math.max(...P1.map((v, i) => Math.abs(v - expected[i])));
  });
  assert.ok(errs[2] < 1e-4, `error at 5 ms ${errs[2]}`);
  assert.ok(errs[0] / errs[1] > 3 && errs[1] / errs[2] > 3, `ratios ${errs[0] / errs[1]}, ${errs[1] / errs[2]}`);
});

test('variance growth matches the declared noise densities', () => {
  const noise = { accelNoiseDensity: 0.05, gyroNoiseDensity: 0.002, gyroBiasRandomWalk: 1e-4, accelBiasRandomWalk: 1e-3 };
  const sample = stationaryRotating(Q0, [0, 0, 0]);
  const T = 10;
  // Gyro-only: attitude variance grows as σ_g² T; bias walks grow as σ_bg² T.
  let s = run(state({ q: Q0 }), sample, 0.01, 1000, { ...ZERO_NOISE, gyroNoiseDensity: noise.gyroNoiseDensity, gyroBiasRandomWalk: 0 });
  for (const i of [6, 7, 8]) close(s.P[i * N + i], noise.gyroNoiseDensity ** 2 * T, 1e-12, `theta${i}`);
  // Accel-only: velocity variance grows as σ_a² T, position as σ_a² T³/3.
  s = run(state({ q: Q0 }), sample, 0.01, 1000, { ...ZERO_NOISE, accelNoiseDensity: noise.accelNoiseDensity });
  for (const i of [3, 4, 5]) close(s.P[i * N + i], noise.accelNoiseDensity ** 2 * T, 1e-9, `v${i}`);
  for (const i of [0, 1, 2]) close(s.P[i * N + i], (noise.accelNoiseDensity ** 2 * T ** 3) / 3, 1e-3 * (noise.accelNoiseDensity ** 2 * T ** 3) / 3, `p${i}`);
  s = run(state({ q: Q0 }), sample, 0.01, 1000, { ...ZERO_NOISE, gyroBiasRandomWalk: noise.gyroBiasRandomWalk, accelBiasRandomWalk: noise.accelBiasRandomWalk });
  for (const i of [9, 10, 11]) close(s.P[i * N + i], noise.gyroBiasRandomWalk ** 2 * T, 1e-15, `bg${i}`);
  for (const i of [12, 13, 14]) close(s.P[i * N + i], noise.accelBiasRandomWalk ** 2 * T, 1e-12, `ba${i}`);
});

test('covariance stays symmetric and positive definite during dynamic motion', () => {
  const noise = { accelNoiseDensity: 0.05, gyroNoiseDensity: 0.002, gyroBiasRandomWalk: 1e-4, accelBiasRandomWalk: 1e-3 };
  const init = state({ q: Q0 });
  init.P = identity(N).map(v => v * 1e-4);
  let s = init;
  for (let k = 0; k < 3000; k++) s = propagate(s, timeVaryingSample(k * 0.01), timeVaryingSample((k + 1) * 0.01), 0.01, noise);
  const health = covarianceHealth(s.P, N);
  assert.ok(health.finite && health.positiveDefinite);
  assert.equal(health.maxAsymmetry, 0);
});

test('tilt error couples into velocity through specific force', () => {
  // A small roll error with level 1 g specific force tilts gravity compensation: dv' = -R[f]x dθ.
  const R = quatToMatrix([1, 0, 0, 0]);
  const F = errorStateJacobian(R, [0, 0, 0], UP_FORCE);
  const dvDot = [3, 4, 5].map(i => F[i * N + 6] * 0.01);
  const expected = cross3([0.01, 0, 0], UP_FORCE);
  dvDot.forEach((v, i) => close(v, expected[i], 1e-15));
  close(GRAVITY_NWU[2], -9.80665, 0);
});

test('propagate rejects invalid steps and inputs explicitly', () => {
  const s = state();
  const m = { gyro: [0, 0, 0], specificForce: UP_FORCE };
  for (const dt of [0, -0.01, NaN, Infinity, DEFAULT_PROPAGATION_OPTIONS.maxStepSeconds * 2]) {
    assert.throws(() => propagate(s, m, m, dt, ZERO_NOISE), /dt/);
  }
  assert.throws(() => propagate(s, m, { gyro: [NaN, 0, 0], specificForce: UP_FORCE }, 0.01, ZERO_NOISE), /non-finite/);
  assert.throws(() => propagate(s, m, m, 0.01, { ...ZERO_NOISE, gyroNoiseDensity: -1 }), /gyroNoiseDensity/);
});
