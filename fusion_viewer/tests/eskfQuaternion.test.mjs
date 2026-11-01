import assert from 'node:assert/strict';
import { test } from 'node:test';
import {
  firmwareEnuToNwu, quatAngleBetween, quatExp, quatLog, quatMultiply, quatNorm, quatNormalize, quatRotate, quatToMatrix,
} from '../src/eskf/quaternion.ts';
import { errorBetween, injectError, initialCovariance } from '../src/eskf/errorState.ts';
import { mul33 } from '../src/eskf/math3.ts';

const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b}`);
const closeVec = (a, b, tol, msg) => a.forEach((v, i) => close(v, b[i], tol, `${msg ?? ''}[${i}]`));

const q = (axis, angle) => {
  const n = Math.hypot(...axis);
  return quatExp(axis.map(v => (v / n) * angle));
};

test('Exp/Log round trip across small, moderate and near-π angles', () => {
  for (const phi of [[1e-11, -2e-11, 3e-12], [0.1, -0.2, 0.3], [1, 2, -0.5], [0, 0, 3.1], [2.2, -1.9, 0.4]]) {
    closeVec(quatLog(quatExp(phi)), phi, 1e-12, 'phi');
    close(quatNorm(quatExp(phi)), 1, 1e-15, 'norm');
  }
});

test('Exp rotates vectors actively: +90° about Z maps X to Y', () => {
  closeVec(quatRotate(quatExp([0, 0, Math.PI / 2]), [1, 0, 0]), [0, 1, 0], 1e-15);
});

test('R(q1 ⊗ q2) = R(q1) R(q2)', () => {
  const q1 = q([1, 2, 3], 0.7);
  const q2 = q([-2, 0.5, 1], 1.9);
  closeVec(quatToMatrix(quatMultiply(q1, q2)), mul33(quatToMatrix(q1), quatToMatrix(q2)), 1e-14);
});

test('quatNormalize rejects degenerate quaternions and preserves sign', () => {
  assert.throws(() => quatNormalize([0, 0, 0, 0]));
  assert.throws(() => quatNormalize([NaN, 0, 0, 1]));
  closeVec(quatNormalize([-2, 0, 0, 0]), [-1, 0, 0, 0], 0);
});

test('quatAngleBetween is sign-invariant and accurate for tiny angles', () => {
  const a = q([1, 1, 0], 0.4);
  close(quatAngleBetween(a, a.map(v => -v)), 0, 1e-15);
  close(quatAngleBetween(a, quatMultiply(a, quatExp([1e-9, 0, 0]))), 1e-9, 1e-17);
});

test('firmware ENU identity maps to the documented NWU known pose', () => {
  const qNB = firmwareEnuToNwu([1, 0, 0, 0]);
  closeVec(quatRotate(qNB, [1, 0, 0]), [0, -1, 0], 1e-15, 'body X (East) -> -West');
  closeVec(quatRotate(qNB, [0, 1, 0]), [1, 0, 0], 1e-15, 'body Y (LED edge North) -> North');
  closeVec(quatRotate(qNB, [0, 0, 1]), [0, 0, 1], 1e-15, 'body Z -> Up');
});

test('firmware ENU conversion changes only the world frame', () => {
  const qEnu = q([0.3, -1, 0.8], 2.1);
  const v = [0.4, -1.3, 2.2];
  const enu = quatRotate(qEnu, v);
  closeVec(quatRotate(firmwareEnuToNwu(qEnu), v), [enu[1], -enu[0], enu[2]], 1e-14);
});

test('injectError and errorBetween are inverses', () => {
  const x = { p: [1, 2, 3], v: [4, -5, 6], q: q([1, -1, 2], 0.9), gyroBias: [0.01, 0, -0.02], accelBias: [0.1, 0.2, -0.3] };
  const dx = Float64Array.from([0.1, -0.2, 0.3, 0.5, 0.4, -0.1, 0.02, -0.05, 0.3, 1e-3, -2e-3, 3e-3, 0.01, 0.02, -0.03]);
  closeVec(errorBetween(injectError(x, dx), x), dx, 1e-14);
});

test('initialCovariance builds a diagonal and rejects invalid sigmas', () => {
  const P = initialCovariance({ position: 1, velocity: 0.5, attitude: [0.1, 0.1, 0.5], gyroBias: 0.01, accelBias: 0.2 });
  close(P[3 * 15 + 3], 0.25, 1e-15);
  close(P[8 * 15 + 8], 0.25, 1e-15);
  close(P[14 * 15 + 14], 0.04, 1e-15);
  assert.equal(P[1], 0);
  assert.throws(() => initialCovariance({ position: -1, velocity: 0, attitude: [0, 0, 0], gyroBias: 0, accelBias: 0 }));
});
