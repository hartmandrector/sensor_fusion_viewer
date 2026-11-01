import type { Mat3, Quat, Vec3 } from './types';

const SMALL_ANGLE = 1e-8;

export function quatMultiply(a: Quat, b: Quat): Quat {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}

export function quatConjugate(q: Quat): Quat {
  return [q[0], -q[1], -q[2], -q[3]];
}

export function quatNorm(q: Quat): number {
  return Math.hypot(q[0], q[1], q[2], q[3]);
}

/** Normalizes q (sign preserved). Throws on degenerate or non-finite input. */
export function quatNormalize(q: Quat): Quat {
  const n = quatNorm(q);
  if (!Number.isFinite(n) || n < 1e-12) {
    throw new Error(`Cannot normalize quaternion [${q.join(', ')}]`);
  }
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

/** Rotation angle (rad, in [0, π]) between two unit quaternions. */
export function quatAngleBetween(a: Quat, b: Quat): number {
  const d = quatMultiply(quatConjugate(a), b);
  return 2 * Math.atan2(Math.hypot(d[1], d[2], d[3]), Math.abs(d[0]));
}

/** Unit quaternion for rotation vector phi (axis × angle, rad). */
export function quatExp(phi: Vec3): Quat {
  const angle = Math.hypot(phi[0], phi[1], phi[2]);
  const half = 0.5 * angle;
  // sin(half)/angle, with Taylor expansion near zero.
  const k = angle < SMALL_ANGLE ? 0.5 - (angle * angle) / 48 : Math.sin(half) / angle;
  return [Math.cos(half), phi[0] * k, phi[1] * k, phi[2] * k];
}

/** Rotation vector of a unit quaternion, angle in [0, π]. */
export function quatLog(q: Quat): Vec3 {
  const u = q[0] < 0 ? ([-q[0], -q[1], -q[2], -q[3]] as Quat) : q;
  const vn = Math.hypot(u[1], u[2], u[3]);
  if (vn < SMALL_ANGLE) {
    const k = 2 / u[0];
    return [u[1] * k, u[2] * k, u[3] * k];
  }
  const k = (2 * Math.atan2(vn, u[0])) / vn;
  return [u[1] * k, u[2] * k, u[3] * k];
}

/** Rotation matrix R(q) such that v_N = R v_B for q = q_NB. */
export function quatToMatrix(q: Quat): Mat3 {
  const [w, x, y, z] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
  ];
}

export function quatRotate(q: Quat, v: Vec3): Vec3 {
  const r = quatToMatrix(q);
  return [
    r[0] * v[0] + r[1] * v[1] + r[2] * v[2],
    r[3] * v[0] + r[4] * v[1] + r[5] * v[2],
    r[6] * v[0] + r[7] * v[1] + r[8] * v[2],
  ];
}

/** Unit quaternion of a proper rotation matrix (Shepperd's method), scalar part ≥ 0. */
export function quatFromMatrix(m: Mat3): Quat {
  const [m00, m01, m02, m10, m11, m12, m20, m21, m22] = m;
  const trace = m00 + m11 + m22;
  let q: Quat;
  if (trace > 0) {
    const s = 2 * Math.sqrt(1 + trace);
    q = [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s];
  } else if (m00 > m11 && m00 > m22) {
    const s = 2 * Math.sqrt(1 + m00 - m11 - m22);
    q = [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s];
  } else if (m11 > m22) {
    const s = 2 * Math.sqrt(1 + m11 - m00 - m22);
    q = [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s];
  } else {
    const s = 2 * Math.sqrt(1 + m22 - m00 - m11);
    q = [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s];
  }
  if (q[0] < 0) q = [-q[0], -q[1], -q[2], -q[3]];
  return quatNormalize(q);
}

/** q_N_ENU: -90° about Up, so that v_NWU = R(q_N_ENU) v_ENU (North = ENU.y, West = -ENU.x). */
const Q_NWU_FROM_ENU: Quat = [Math.SQRT1_2, 0, 0, -Math.SQRT1_2];

/**
 * Converts a firmware body-to-ENU quaternion into a body-to-NWU estimator quaternion.
 * Only the world frame changes; body axes are untouched.
 */
export function firmwareEnuToNwu(qEnuBody: Quat): Quat {
  return quatNormalize(quatMultiply(Q_NWU_FROM_ENU, qEnuBody));
}
