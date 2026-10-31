import assert from 'node:assert/strict';
import { test } from 'node:test';
import { getDisplayOrientation } from '../src/orientationSource.ts';

const frame = {
  timestamp: 10,
  quaternion: { w: 1, x: 0, y: 0, z: 0 },
  euler: { roll: 0, pitch: 0, yaw: 0 },
  heading: 0,
  imu: { deviceQuaternion: { w: 0.7071, x: 0, y: 0, z: 0.7071 } }
};

test('viewer mode preserves the computed orientation', () => {
  const output = getDisplayOrientation(frame, 'viewer');
  assert.equal(output.quaternion, frame.quaternion);
  assert.equal(output.euler, frame.euler);
  assert.equal(output.heading, frame.heading);
});

test('device mode normalizes only its display copy', () => {
  const before = structuredClone(frame);
  const output = getDisplayOrientation(frame, 'device');
  assert.ok(Math.abs(Math.hypot(...Object.values(output.quaternion)) - 1) < 1e-10);
  assert.deepEqual(frame, before);
});

test('missing device samples never fall back to computed orientation', () => {
  assert.equal(getDisplayOrientation({ ...frame, imu: undefined }, 'device'), null);
  assert.equal(getDisplayOrientation({ ...frame, imu: {} }, 'device'), null);
});

test('device orientation is independent of recomputed fusion results', () => {
  const changed = { ...frame, quaternion: { w: 0, x: 1, y: 0, z: 0 }, heading: 180 };
  assert.deepEqual(getDisplayOrientation(changed, 'device'), getDisplayOrientation(frame, 'device'));
});

test('heading stays in range and q/-q produce identical rotation', () => {
  const device = { ...frame, imu: { deviceQuaternion: { w: 0.7071, x: 0, y: 0, z: -0.7071 } } };
  const output = getDisplayOrientation(device, 'device');
  assert.ok(output.heading >= 0 && output.heading < 360);
  const negated = { ...frame, imu: { deviceQuaternion: { w: -0.7071, x: 0, y: 0, z: 0.7071 } } };
  const negatedEuler = getDisplayOrientation(negated, 'device').euler;
  for (const axis of ['roll', 'pitch', 'yaw']) {
    assert.ok(Math.abs(negatedEuler[axis] - output.euler[axis]) < 1e-10);
  }
});

test('device Euler angles remain finite at the flat pose singularity', () => {
  const output = getDisplayOrientation({
    ...frame, imu: { deviceQuaternion: { w: 1, x: 0, y: 0, z: 0 } }
  }, 'device');
  assert.ok(Math.abs(Math.abs(output.euler.pitch) - Math.PI / 2) < 1e-7);
  assert.ok(Object.values(output.euler).every(Number.isFinite));
});

function rotate(q, v) {
  const cross = (a, b) => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
  ];
  const u = [q.x, q.y, q.z];
  const uv = cross(u, v);
  const uuv = cross(u, uv);
  return v.map((value, i) => value + 2 * (q.w * uv[i] + uuv[i]));
}

function assertVector(actual, expected) {
  actual.forEach((value, i) => assert.ok(Math.abs(value - expected[i]) < 1e-10,
    `component ${i}: expected ${expected[i]}, got ${value}`));
}

function sceneQuaternion(recorded) {
  const pose = getDisplayOrientation({ ...frame, imu: { deviceQuaternion: recorded } }, 'device');
  const q = pose.quaternion;
  // The renderer maps its NWU input to Three.js East/Up/South.
  return { w: q.w, x: -q.y, y: q.z, z: -q.x };
}

test('firmware identity renders flat, front face Up, LED North, sensor X East', () => {
  const q = sceneQuaternion({ w: 1, x: 0, y: 0, z: 0 });
  assertVector(rotate(q, [0, 0, -1]), [0, 1, 0]); // Front face
  assertVector(rotate(q, [0, 1, 0]), [0, 0, -1]); // LED edge
  assertVector(rotate(q, [-1, 0, 0]), [1, 0, 0]); // Model X label
});

for (const [axis, vector] of [['East', [1, 0, 0]], ['North', [0, 1, 0]], ['Up', [0, 0, 1]]]) {
  test(`firmware +90 degrees about ${axis} rotates all physical axes correctly`, () => {
    const recorded = {
      w: Math.SQRT1_2,
      x: vector[0] * Math.SQRT1_2,
      y: vector[1] * Math.SQRT1_2,
      z: vector[2] * Math.SQRT1_2
    };
    const scene = sceneQuaternion(recorded);
    for (const [local, body] of [
      [[-1, 0, 0], [1, 0, 0]],
      [[0, 1, 0], [0, 1, 0]],
      [[0, 0, -1], [0, 0, 1]]
    ]) {
      const enu = rotate(recorded, body);
      assertVector(rotate(scene, local), [enu[0], enu[2], -enu[1]]);
    }
  });
}

test('arbitrary firmware rotation preserves physical body directions', () => {
  const raw = { w: 0.6396, x: 0.084, y: 0.0224, z: 0.7637 };
  const norm = Math.hypot(...Object.values(raw));
  const unit = Object.fromEntries(Object.entries(raw).map(([key, value]) => [key, value / norm]));
  const enu = rotate(unit, [0.2, 0.3, 0.4]);
  assertVector(rotate(sceneQuaternion(raw), [-0.2, 0.3, -0.4]), [enu[0], enu[2], -enu[1]]);
});
