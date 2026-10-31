import assert from 'node:assert/strict';
import { test } from 'node:test';
import { getDeviceEarthAcceleration } from '../src/deviceAcceleration.ts';

function imu(q, accel = [0, 0, 1]) {
  return {
    type: 'IMU', timestamp: 12, wx: 0, wy: 0, wz: 0,
    ax: accel[0], ay: accel[1], az: accel[2], temperature: 30,
    deviceQuaternion: q
  };
}

function near(actual, expected) {
  for (const axis of ['x', 'y', 'z']) {
    assert.ok(Math.abs(actual[axis] - expected[axis]) < 1e-10,
      `${axis}: ${actual[axis]} != ${expected[axis]}`);
  }
}

test('flat stationary device removes exactly 1 g without model mounting rotation', () => {
  near(getDeviceEarthAcceleration(imu({ w: 1, x: 0, y: 0, z: 0 })), { x: 0, y: 0, z: 0 });
});

test('identity maps raw East/North/Up acceleration into NWU', () => {
  near(getDeviceEarthAcceleration(imu({ w: 1, x: 0, y: 0, z: 0 }, [0.2, 0.3, 1.4])),
    { x: 0.3, y: -0.2, z: 0.4 });
});

test('90-degree yaw rotates body East to world North', () => {
  near(getDeviceEarthAcceleration(imu({ w: 0.7071, x: 0, y: 0, z: 0.7071 }, [1, 0, 1])),
    { x: 1, y: 0, z: 0 });
});

test('stationary tilted poses remove gravity about each horizontal axis', () => {
  near(getDeviceEarthAcceleration(imu({ w: 0.7071, x: 0.7071, y: 0, z: 0 }, [0, 1, 0])),
    { x: 0, y: 0, z: 0 });
  near(getDeviceEarthAcceleration(imu({ w: 0.7071, x: 0, y: 0.7071, z: 0 }, [-1, 0, 0])),
    { x: 0, y: 0, z: 0 });
});

test('normalization and quaternion sign do not affect acceleration or mutate logged samples', () => {
  const reading = imu({ w: 0.7071, x: 0, y: 0, z: 0.7071 }, [0.2, 0.3, 1.1]);
  const saved = structuredClone(reading);
  const negated = imu({ w: -7.071, x: 0, y: 0, z: -7.071 }, [0.2, 0.3, 1.1]);
  near(getDeviceEarthAcceleration(reading), getDeviceEarthAcceleration(negated));
  assert.deepEqual(reading, saved);
});

test('free fall remains -1 g in Up after gravity removal', () => {
  near(getDeviceEarthAcceleration(imu({ w: 1, x: 0, y: 0, z: 0 }, [0, 0, 0])),
    { x: 0, y: 0, z: -1 });
});

test('missing/invalid inputs report the sample timestamp, never a fallback', () => {
  for (const q of [undefined, { w: 0, x: 0, y: 0, z: 0 }, { w: NaN, x: 0, y: 0, z: 0 }]) {
    assert.throws(() => getDeviceEarthAcceleration(imu(q)), /12\.000000s/);
  }
  assert.throws(() => getDeviceEarthAcceleration(imu({ w: 1, x: 0, y: 0, z: 0 }, [NaN, 0, 1])),
    /Invalid device acceleration input/);
});
