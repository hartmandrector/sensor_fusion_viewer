import type { IMUData } from './csvParser';
import type { Vector3 } from './types';

/** Rotate raw firmware-body acceleration into Earth NWU, removing gravity (g). */
export function getDeviceEarthAcceleration(imu: IMUData): Vector3 {
  const q = imu.deviceQuaternion;
  if (!q) {
    throw new Error(`Missing device quaternion at sensor time ${imu.timestamp.toFixed(6)}s`);
  }
  const norm = Math.hypot(q.w, q.x, q.y, q.z);
  if (!Number.isFinite(norm) || norm === 0 || ![imu.ax, imu.ay, imu.az].every(Number.isFinite)) {
    throw new Error(`Invalid device acceleration input at sensor time ${imu.timestamp.toFixed(6)}s`);
  }
  const w = q.w / norm, x = q.x / norm, y = q.y / norm, z = q.z / norm;
  const { ax, ay, az } = imu;
  const tx = 2 * (y * az - z * ay);
  const ty = 2 * (z * ax - x * az);
  const tz = 2 * (x * ay - y * ax);
  const east = ax + w * tx + y * tz - z * ty;
  const north = ay + w * ty + z * tx - x * tz;
  const up = az + w * tz + x * ty - y * tx;
  // This is a sensor-vector transform, not the 3D model-mounting transform.
  return { x: north, y: -east, z: up - 1 };
}
