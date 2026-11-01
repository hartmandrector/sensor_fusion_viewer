import type { Quaternion } from './types';
import type { FusionFrame } from './appState';

export type OrientationSource = 'viewer' | 'device';

export function getDisplayOrientation(frame: FusionFrame, source: OrientationSource) {
  if (source === 'viewer') {
    return { quaternion: frame.quaternion, euler: frame.euler, heading: frame.heading };
  }
  const recorded = frame.imu?.deviceQuaternion;
  if (!recorded) return null;

  // Firmware: ENU, identity flat/front face up/LED North.
  // Model local -> firmware body = 180 degrees about Y.
  // ENU -> Three.js = -90 degrees about X. Compose A * recorded * B,
  // then undo setOrientation's NWU -> Three.js basis mapping.
  // Normalize only this display copy; CSV rounding can change the norm.
  const norm = Math.hypot(recorded.w, recorded.x, recorded.y, recorded.z);
  const scale = Math.SQRT1_2 / norm;
  const quaternion: Quaternion = {
    w: -(recorded.y + recorded.z) * scale,
    x: (recorded.w - recorded.x) * scale,
    y: (recorded.z - recorded.y) * scale,
    z: (recorded.w + recorded.x) * scale
  };
  return { quaternion, ...eulerAndHeading(quaternion) };
}

/** ZYX Euler angles (rad) and heading (deg, 0–360) of a viewer-convention quaternion. */
export function eulerAndHeading(quaternion: Quaternion) {
  const { w, x, y, z } = quaternion;
  const euler = {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    pitch: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
    yaw: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
  };
  const heading = (euler.yaw * 180 / Math.PI + 360) % 360;
  return { euler, heading };
}
