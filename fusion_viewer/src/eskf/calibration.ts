import { mulMat3Vec3 } from './math3.ts';
import { firmwareEnuToNwu, quatConjugate, quatFromMatrix, quatMultiply, quatNormalize } from './quaternion.ts';
import { STANDARD_GRAVITY } from './propagation.ts';
import type { AxisRemap } from '../types';
import type { ImuMeasurement, Mat3, Quat, Vec3 } from './types';

const DEG_TO_RAD = Math.PI / 180;

/**
 * IMU calibration in the viewer's units and order (see FusionAhrsAdapter.updateIMU):
 * gyro ω = gyroScale · (raw − bias) → remap; accel f = S · (raw − bias) → remap.
 */
export interface ImuCalibration {
  gyroBiasDps: Vec3;
  /** Multiplier on bias-corrected gyro rates; 1 means none. */
  gyroScale: number;
  accelBiasG: Vec3;
  /** Inverse scale matrix applied after bias removal; null means identity. */
  accelScaleMatrix: Mat3 | null;
  axisRemap: AxisRemap;
}

export const IDENTITY_REMAP: AxisRemap = { bodyX: '+X', bodyY: '+Y', bodyZ: '+Z' };

/** Matrix M with body = M · sensor for a signed-permutation remap. Throws unless it is a proper rotation. */
export function remapMatrix(remap: AxisRemap): Mat3 {
  const row = (spec: string): [number, number, number] => {
    const sign = spec[0] === '-' ? -1 : 1;
    const axis = 'XYZ'.indexOf(spec[1]);
    if (axis < 0 || (spec[0] !== '+' && spec[0] !== '-')) throw new Error(`Invalid axis spec ${spec}`);
    const r: [number, number, number] = [0, 0, 0];
    r[axis] = sign;
    return r;
  };
  const m = [...row(remap.bodyX), ...row(remap.bodyY), ...row(remap.bodyZ)] as Mat3;
  const det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
  if (Math.abs(det - 1) > 1e-9) {
    throw new Error(`IMU axis remap ${remap.bodyX},${remap.bodyY},${remap.bodyZ} is not a proper rotation (det ${det}); the ESKF needs a right-handed body frame.`);
  }
  return m;
}

export function validateCalibration(cal: ImuCalibration): void {
  const finite = (v: number[]) => v.every(Number.isFinite);
  if (!finite(cal.gyroBiasDps) || !finite(cal.accelBiasG)) throw new Error('Calibration biases must be finite');
  if (!Number.isFinite(cal.gyroScale) || cal.gyroScale <= 0.5 || cal.gyroScale >= 2) {
    throw new Error(`Gyro scale ${cal.gyroScale} is outside the accepted range (0.5, 2)`);
  }
  if (cal.accelScaleMatrix && (cal.accelScaleMatrix.length !== 9 || !finite(cal.accelScaleMatrix))) {
    throw new Error('Accel scale matrix must be a finite 3×3 matrix');
  }
  remapMatrix(cal.axisRemap);
}

/** Converts raw gyro (deg/s) and accel (g) to SI body-frame measurements. */
export function calibrateImu(gyroDps: Vec3, accelG: Vec3, cal: ImuCalibration, remap: Mat3 = remapMatrix(cal.axisRemap)): ImuMeasurement {
  const k = cal.gyroScale * DEG_TO_RAD;
  const gyro: Vec3 = [
    (gyroDps[0] - cal.gyroBiasDps[0]) * k,
    (gyroDps[1] - cal.gyroBiasDps[1]) * k,
    (gyroDps[2] - cal.gyroBiasDps[2]) * k,
  ];
  let accel: Vec3 = [accelG[0] - cal.accelBiasG[0], accelG[1] - cal.accelBiasG[1], accelG[2] - cal.accelBiasG[2]];
  if (cal.accelScaleMatrix) accel = mulMat3Vec3(cal.accelScaleMatrix, accel);
  const f = mulMat3Vec3(remap, accel);
  return {
    gyro: mulMat3Vec3(remap, gyro),
    specificForce: [f[0] * STANDARD_GRAVITY, f[1] * STANDARD_GRAVITY, f[2] * STANDARD_GRAVITY],
  };
}

/**
 * The viewer's AHRS runs in an algorithm frame A = P·B with NWU = [Bz, Bx, By]
 * (FusionAhrsAdapter.bodyToNWU), so q_viewer = q_eskf ⊗ q_P⁻¹ with R(q_P) = P.
 */
export const P_ALGO_FROM_BODY: Mat3 = [0, 0, 1, 1, 0, 0, 0, 1, 0];
export const Q_ALGO_FROM_BODY: Quat = quatFromMatrix(P_ALGO_FROM_BODY);

export function eskfToViewerQuaternion(qEskf: Quat): Quat {
  return quatNormalize(quatMultiply(qEskf, quatConjugate(Q_ALGO_FROM_BODY)));
}

export function viewerToEskfQuaternion(qViewer: Quat): Quat {
  return quatNormalize(quatMultiply(qViewer, Q_ALGO_FROM_BODY));
}

/**
 * Device quaternion (firmware raw sensor axes → ENU) to ESKF (remapped body → NWU).
 * B = M·S, so R_NB = R_NS·Mᵀ.
 */
export function deviceToEskfQuaternion(qDeviceEnu: Quat, remap: AxisRemap): Quat {
  const qNS = firmwareEnuToNwu(qDeviceEnu);
  const qM = quatFromMatrix(remapMatrix(remap));
  return quatNormalize(quatMultiply(qNS, quatConjugate(qM)));
}
