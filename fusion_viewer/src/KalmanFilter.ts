/**
 * Kalman Filter for 3D Attitude Estimation (Quaternion-based)
 * 
 * This is an Extended Kalman Filter (EKF) for estimating 3D orientation
 * from gyroscope, accelerometer, and magnetometer measurements.
 * 
 * Key design principles:
 * - Explicit uncertainty modeling (Q and R matrices)
 * - Compatible with other sensor fusion systems
 * - Modular calibration handling
 * - Quaternion state representation
 * 
 * References:
 * - "Attitude Representation and Kinematic Propagation for Low-Cost UAVs" by Trawny & Kanade
 * - "The Quaternion Extended Kalman Filter" by Zhou
 */

import { debug, MIN_VECTOR_MAGNITUDE } from './constants';
import type {
  Quaternion,
  EulerAngles,
  MagCalibration,
  IMUCalibration
} from './types';

// ============================================================================
// Constants
// ============================================================================

const DEG_TO_RAD = Math.PI / 180.0;
const RAD_TO_DEG = 180.0 / Math.PI;

// ============================================================================
// Configuration Interface
// ============================================================================

/**
 * Kalman Filter Noise Parameters
 */
export interface KalmanFilterConfig {
  gyroNoiseStd: number;              // °/s standard deviation
  gyroBiasNoiseStd: number;          // °/s² (bias random walk)
  accelNoiseStd: number;             // g standard deviation
  magNoiseStd: number;               // Gauss standard deviation
  initialAttitudeUncertainty: number; // Initial P diagonal for attitude (rad²)
  initialBiasUncertainty: number;    // Initial P diagonal for bias (rad/s)²
  estimateGyroBias: boolean;         // Include bias in state vector
}

export const DEFAULT_KALMAN_CONFIG: KalmanFilterConfig = {
  gyroNoiseStd: 0.1,
  gyroBiasNoiseStd: 0.0001,
  accelNoiseStd: 0.05,
  magNoiseStd: 0.1,
  initialAttitudeUncertainty: 0.1,
  initialBiasUncertainty: 0.01,
  estimateGyroBias: true
};

// ============================================================================
// Vector Math Helpers
// ============================================================================

interface Vector3 {
  x: number;
  y: number;
  z: number;
}

function v3_zero(): Vector3 {
  return { x: 0, y: 0, z: 0 };
}

function v3_sub(a: Vector3, b: Vector3): Vector3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

function v3_scale(v: Vector3, s: number): Vector3 {
  return { x: v.x * s, y: v.y * s, z: v.z * s };
}

function v3_dot(a: Vector3, b: Vector3): number {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function v3_magnitude(v: Vector3): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

function v3_normalize(v: Vector3): Vector3 {
  const mag = v3_magnitude(v);
  if (mag < MIN_VECTOR_MAGNITUDE) return v3_zero();
  return v3_scale(v, 1 / mag);
}

// ============================================================================
// Matrix Math Helpers
// ============================================================================

function zeros(rows: number, cols: number): number[][] {
  return Array(rows).fill(null).map(() => Array(cols).fill(0));
}

function eye(n: number): number[][] {
  const m = zeros(n, n);
  for (let i = 0; i < n; i++) m[i][i] = 1;
  return m;
}

function matmul(A: number[][], B: number[][]): number[][] {
  const m = A.length;
  const n = B[0].length;
  const p = B.length;
  const C = zeros(m, n);
  
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < n; j++) {
      let sum = 0;
      for (let k = 0; k < p; k++) {
        sum += A[i][k] * B[k][j];
      }
      C[i][j] = sum;
    }
  }
  return C;
}

function transpose(A: number[][]): number[][] {
  const m = A.length;
  const n = A[0].length;
  const T = zeros(n, m);
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < n; j++) {
      T[j][i] = A[i][j];
    }
  }
  return T;
}

function matvec(A: number[][], v: number[]): number[] {
  const m = A.length;
  const result = Array(m).fill(0);
  for (let i = 0; i < m; i++) {
    for (let j = 0; j < A[i].length; j++) {
      result[i] += A[i][j] * v[j];
    }
  }
  return result;
}

function inverse3x3(M: number[][]): number[][] {
  const [[a,b,c], [d,e,f], [g,h,i]] = M;
  
  const det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
  if (Math.abs(det) < 1e-10) {
    return eye(3);
  }
  
  return [
    [(e*i - f*h)/det, (c*h - b*i)/det, (b*f - c*e)/det],
    [(f*g - d*i)/det, (a*i - c*g)/det, (c*d - a*f)/det],
    [(d*h - e*g)/det, (b*g - a*h)/det, (a*e - b*d)/det]
  ];
}

// ============================================================================
// Quaternion Math
// ============================================================================

function q_multiply(a: Quaternion, b: Quaternion): Quaternion {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
  };
}

function q_normalize(q: Quaternion): Quaternion {
  const mag = Math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (mag < MIN_VECTOR_MAGNITUDE) return { w: 1, x: 0, y: 0, z: 0 };
  return { w: q.w/mag, x: q.x/mag, y: q.y/mag, z: q.z/mag };
}

function q_conjugate(q: Quaternion): Quaternion {
  return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
}

function q_rotate_vector(q: Quaternion, v: Vector3): Vector3 {
  const p = { w: 0, x: v.x, y: v.y, z: v.z };
  const q_conj = q_conjugate(q);
  const temp = q_multiply(q, p);
  const result_q = q_multiply(temp, q_conj);
  
  return { x: result_q.x, y: result_q.y, z: result_q.z };
}

function q_to_euler(q: Quaternion): EulerAngles {
  const { w, x, y, z } = q;
  
  const sinr_cosp = 2 * (w * x + y * z);
  const cosr_cosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);
  
  const sinp = 2 * (w * y - z * x);
  const pitch = Math.asin(Math.max(-1, Math.min(1, sinp)));
  
  const siny_cosp = 2 * (w * z + x * y);
  const cosy_cosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);
  
  return { roll, pitch, yaw };
}

function euler_to_q(roll: number, pitch: number, yaw: number): Quaternion {
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);
  
  return {
    w: cr * cp * cy + sr * sp * sy,
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy
  };
}

function q_get_heading(q: Quaternion): number {
  const euler = q_to_euler(q);
  return euler.yaw * RAD_TO_DEG;
}

// ============================================================================
// Kalman Filter Class
// ============================================================================

export class KalmanFilter {
  private stateSize: number;
  private x: number[];
  private P: number[][];
  
  private config: KalmanFilterConfig;
  private calibration: {
    imu: IMUCalibration;
    mag: MagCalibration;
  };
  
  private compassHeading: number = 0;
  
  constructor(
    config: Partial<KalmanFilterConfig> = {},
    calibration: Partial<{ imu: IMUCalibration; mag: MagCalibration }> = {}
  ) {
    this.config = { ...DEFAULT_KALMAN_CONFIG, ...config };
    
    this.calibration = {
      imu: {
        gyroBiasX: 0, gyroBiasY: 0, gyroBiasZ: 0,
        accelOffsetX: 0, accelOffsetY: 0, accelOffsetZ: 0,
        ...(calibration.imu || {})
      },
      mag: {
        offsetX: 0, offsetY: 0, offsetZ: 0,
        scaleX: 1, scaleY: 1, scaleZ: 1,
        ...(calibration.mag || {})
      }
    };
    
    this.stateSize = this.config.estimateGyroBias ? 7 : 4;
    
    this.x = new Array(this.stateSize);
    this.x[0] = 1;
    for (let i = 1; i < this.stateSize; i++) this.x[i] = 0;
    
    this.P = eye(this.stateSize);
    for (let i = 0; i < 4; i++) {
      this.P[i][i] = this.config.initialAttitudeUncertainty;
    }
    if (this.config.estimateGyroBias) {
      for (let i = 4; i < 7; i++) {
        this.P[i][i] = this.config.initialBiasUncertainty;
      }
    }
    
    debug.log('Kalman Filter initialized');
  }
  
  public initFromAccelMag(
    ax: number, ay: number, az: number,
    mx: number, my: number, mz: number
  ): void {
    // Simple initialization: compute roll and pitch from accel, yaw from mag
    const a = v3_normalize({ x: ax, y: ay, z: az });
    const m = v3_normalize({ x: mx, y: my, z: mz });
    
    if (v3_magnitude(a) < MIN_VECTOR_MAGNITUDE) {
      // No valid accel, just initialize to identity
      this.x[0] = 1;
      for (let i = 1; i < this.stateSize; i++) this.x[i] = 0;
      return;
    }
    
    // Roll and pitch from accelerometer
    const pitch = Math.atan2(a.x, Math.sqrt(a.y*a.y + a.z*a.z));
    const roll = Math.atan2(-a.y, -a.z);
    
    // For yaw, use magnetometer if valid
    let yaw = 0;
    if (v3_magnitude(m) >= MIN_VECTOR_MAGNITUDE) {
      // Project mag onto horizontal plane (perpendicular to gravity/accel)
      const grav_norm = v3_normalize(a);
      const mag_horiz = v3_sub(m, v3_scale(grav_norm, v3_dot(m, grav_norm)));
      const mag_horiz_norm = v3_normalize(mag_horiz);
      
      // Yaw is angle from X axis in horizontal plane
      yaw = Math.atan2(mag_horiz_norm.y, mag_horiz_norm.x);
    }
    
    const q = euler_to_q(roll, pitch, yaw);
    this.x[0] = q.w;
    this.x[1] = q.x;
    this.x[2] = q.y;
    this.x[3] = q.z;
    
    debug.log('Kalman Filter initialized from accel/mag');
  }
  
  public initFromAccelOnly(ax: number, ay: number, az: number): void {
    const a = v3_normalize({ x: ax, y: ay, z: az });
    
    const pitch = Math.atan2(a.x, Math.sqrt(a.y*a.y + a.z*a.z));
    const roll = Math.atan2(-a.y, -a.z);
    
    const q = euler_to_q(roll, pitch, 0);
    this.x[0] = q.w;
    this.x[1] = q.x;
    this.x[2] = q.y;
    this.x[3] = q.z;
    
    debug.log('Kalman Filter initialized from accel');
  }
  
  public setIMUCalibration(cal: IMUCalibration): void {
    this.calibration.imu = { ...cal };
  }
  
  public setMagCalibration(cal: MagCalibration): void {
    this.calibration.mag = { ...cal };
  }
  
  public update(
    dt: number,
    wx: number, wy: number, wz: number,
    ax: number, ay: number, az: number,
    mx: number, my: number, mz: number
  ): void {
    this.predictStep(dt, wx, wy, wz);
    
    if (Math.sqrt(ax*ax + ay*ay + az*az) > MIN_VECTOR_MAGNITUDE) {
      this.updateAccel(ax, ay, az);
    }
    
    if (Math.sqrt(mx*mx + my*my + mz*mz) > MIN_VECTOR_MAGNITUDE) {
      this.updateMag(mx, my, mz);
    }
    
    this.normalizeQuaternion();
  }
  
  private predictStep(dt: number, wx: number, wy: number, wz: number): void {
    let bx = 0, by = 0, bz = 0;
    if (this.config.estimateGyroBias) {
      bx = this.x[4];
      by = this.x[5];
      bz = this.x[6];
    }
    
    // Expect gyro already calibrated (bias subtracted and in rad/s)
    const wc_x = wx - bx;
    const wc_y = wy - by;
    const wc_z = wz - bz;
    
    const qw = this.x[0], qx = this.x[1], qy = this.x[2], qz = this.x[3];
    
    const qDot_w = 0.5 * (-qx * wc_x - qy * wc_y - qz * wc_z);
    const qDot_x = 0.5 * (qw * wc_x + qy * wc_z - qz * wc_y);
    const qDot_y = 0.5 * (qw * wc_y + qz * wc_x - qx * wc_z);
    const qDot_z = 0.5 * (qw * wc_z + qx * wc_y - qy * wc_x);
    
    this.x[0] += qDot_w * dt;
    this.x[1] += qDot_x * dt;
    this.x[2] += qDot_y * dt;
    this.x[3] += qDot_z * dt;
    
    const F = eye(this.stateSize);
    
    const dF = [
      [0, -wc_x, -wc_y, -wc_z],
      [wc_x, 0, wc_z, -wc_y],
      [wc_y, -wc_z, 0, wc_x],
      [wc_z, wc_y, -wc_x, 0]
    ];
    
    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        F[i][j] += 0.5 * dF[i][j] * dt;
      }
    }
    
    const q_gyro = this.config.gyroNoiseStd * DEG_TO_RAD;
    const Q_diag = [
      q_gyro * q_gyro,
      q_gyro * q_gyro,
      q_gyro * q_gyro,
      q_gyro * q_gyro
    ];
    
    if (this.config.estimateGyroBias) {
      const q_bias = this.config.gyroBiasNoiseStd * DEG_TO_RAD;
      Q_diag.push(q_bias * q_bias, q_bias * q_bias, q_bias * q_bias);
    }
    
    const Q = zeros(this.stateSize, this.stateSize);
    for (let i = 0; i < this.stateSize; i++) {
      Q[i][i] = Q_diag[i];
    }
    
    const FP = matmul(F, this.P);
    const FPF_T = matmul(FP, transpose(F));
    
    for (let i = 0; i < this.stateSize; i++) {
      for (let j = 0; j < this.stateSize; j++) {
        this.P[i][j] = FPF_T[i][j] + Q[i][j];
      }
    }
  }
  
  private updateAccel(ax: number, ay: number, az: number): void {
    const a = { x: ax, y: ay, z: az };
    const a_mag = v3_magnitude(a);
    
    if (a_mag < MIN_VECTOR_MAGNITUDE) return;
    
    const a_norm = v3_normalize(a);
    
    const q = { w: this.x[0], x: this.x[1], y: this.x[2], z: this.x[3] };
    const gravity_world = { x: 0, y: 0, z: -1 };
    let gravity_body = q_rotate_vector(q, gravity_world);
    gravity_body = v3_normalize(gravity_body);  // Ensure unit length
    
    const innovation = v3_sub(a_norm, gravity_body);
    
    const H = zeros(3, this.stateSize);
    
    const eps = 1e-6;
    for (let j = 0; j < 4; j++) {
      const q_pert = { w: this.x[0], x: this.x[1], y: this.x[2], z: this.x[3] };
      (q_pert as any)[['w','x','y','z'][j]] += eps;
      const q_pert_norm = q_normalize(q_pert);
      
      const z_pert = q_rotate_vector(q_pert_norm, gravity_world);
      const dz_dq = v3_scale(v3_sub(z_pert, gravity_body), 1 / eps);
      
      H[0][j] = dz_dq.x;
      H[1][j] = dz_dq.y;
      H[2][j] = dz_dq.z;
    }
    
    const r_accel = this.config.accelNoiseStd;
    const R = zeros(3, 3);
    R[0][0] = r_accel * r_accel;
    R[1][1] = r_accel * r_accel;
    R[2][2] = r_accel * r_accel;
    
    const HP = matmul(H, this.P);
    const HPH_T = matmul(HP, transpose(H));
    const S = zeros(3, 3);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        S[i][j] = HPH_T[i][j] + R[i][j];
      }
    }
    
    const S_inv = inverse3x3(S);
    const H_T = transpose(H);
    const PH_T = matmul(this.P, H_T);
    const K = matmul(PH_T, S_inv);
    
    const innovation_vec = [innovation.x, innovation.y, innovation.z];
    const state_delta = matvec(K, innovation_vec);
    
    for (let i = 0; i < this.stateSize; i++) {
      this.x[i] += state_delta[i];
    }
    
    const KH = matmul(K, H);
    const I_KH = eye(this.stateSize);
    for (let i = 0; i < this.stateSize; i++) {
      for (let j = 0; j < this.stateSize; j++) {
        I_KH[i][j] -= KH[i][j];
      }
    }
    
    const P_new = matmul(I_KH, this.P);
    for (let i = 0; i < this.stateSize; i++) {
      for (let j = 0; j < this.stateSize; j++) {
        this.P[i][j] = P_new[i][j];
      }
    }
  }
  
  private updateMag(mx: number, my: number, mz: number): void {
    // Expect magnetometer already calibrated (hard iron, soft iron applied)
    const m = { x: mx, y: my, z: mz };
    const m_norm = v3_normalize(m);
    if (v3_magnitude(m) < MIN_VECTOR_MAGNITUDE) return;
    
    const q = { w: this.x[0], x: this.x[1], y: this.x[2], z: this.x[3] };
    const mag_world = { x: 1, y: 0, z: 0 };
    let mag_body = q_rotate_vector(q, mag_world);
    mag_body = v3_normalize(mag_body);  // Ensure unit length
    
    const innovation = v3_sub(m_norm, mag_body);
    
    const H = zeros(3, this.stateSize);
    
    const eps = 1e-6;
    for (let j = 0; j < 4; j++) {
      const q_pert = { w: this.x[0], x: this.x[1], y: this.x[2], z: this.x[3] };
      (q_pert as any)[['w','x','y','z'][j]] += eps;
      const q_pert_norm = q_normalize(q_pert);
      
      const z_pert = q_rotate_vector(q_pert_norm, mag_world);
      const dz_dq = v3_scale(v3_sub(z_pert, mag_body), 1 / eps);
      
      H[0][j] = dz_dq.x;
      H[1][j] = dz_dq.y;
      H[2][j] = dz_dq.z;
    }
    
    const r_mag = this.config.magNoiseStd;
    const R = zeros(3, 3);
    R[0][0] = r_mag * r_mag;
    R[1][1] = r_mag * r_mag;
    R[2][2] = r_mag * r_mag;
    
    const HP = matmul(H, this.P);
    const HPH_T = matmul(HP, transpose(H));
    const S = zeros(3, 3);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        S[i][j] = HPH_T[i][j] + R[i][j];
      }
    }
    
    const S_inv = inverse3x3(S);
    const H_T = transpose(H);
    const PH_T = matmul(this.P, H_T);
    const K = matmul(PH_T, S_inv);
    
    const innovation_vec = [innovation.x, innovation.y, innovation.z];
    const state_delta = matvec(K, innovation_vec);
    
    for (let i = 0; i < this.stateSize; i++) {
      this.x[i] += state_delta[i];
    }
    
    const KH = matmul(K, H);
    const I_KH = eye(this.stateSize);
    for (let i = 0; i < this.stateSize; i++) {
      for (let j = 0; j < this.stateSize; j++) {
        I_KH[i][j] -= KH[i][j];
      }
    }
    
    const P_new = matmul(I_KH, this.P);
    for (let i = 0; i < this.stateSize; i++) {
      for (let j = 0; j < this.stateSize; j++) {
        this.P[i][j] = P_new[i][j];
      }
    }
  }
  
  private normalizeQuaternion(): void {
    const q = q_normalize({ w: this.x[0], x: this.x[1], y: this.x[2], z: this.x[3] });
    this.x[0] = q.w;
    this.x[1] = q.x;
    this.x[2] = q.y;
    this.x[3] = q.z;
    this.compassHeading = q_get_heading(q);
  }
  
  public getQuaternion(): Quaternion {
    return { w: this.x[0], x: this.x[1], y: this.x[2], z: this.x[3] };
  }
  
  public getEulerAngles(): EulerAngles {
    const q = this.getQuaternion();
    return q_to_euler(q);
  }
  
  public getHeading(): number {
    return this.compassHeading;
  }
  
  public getGyroBias(): { x: number; y: number; z: number } {
    if (this.config.estimateGyroBias) {
      return { x: this.x[4], y: this.x[5], z: this.x[6] };
    }
    return { x: 0, y: 0, z: 0 };
  }
  
  public reset(): void {
    this.x[0] = 1;
    for (let i = 1; i < this.stateSize; i++) this.x[i] = 0;
    this.P = eye(this.stateSize);
    for (let i = 0; i < 4; i++) {
      this.P[i][i] = this.config.initialAttitudeUncertainty;
    }
    if (this.config.estimateGyroBias) {
      for (let i = 4; i < 7; i++) {
        this.P[i][i] = this.config.initialBiasUncertainty;
      }
    }
  }
}
