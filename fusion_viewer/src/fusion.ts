/**
 * FlySight 2 Sensor Fusion - Madgwick AHRS Algorithm
 * 
 * This implementation is designed to be portable to C/C++ for STM32 firmware.
 * Uses single-precision math, no dynamic allocation, pure functions where possible.
 * 
 * Based on: "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
 * by Sebastian Madgwick (2010)
 * 
 * Reference: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */

import { debug, DEFAULT_AXIS_REMAP, MIN_VECTOR_MAGNITUDE } from './constants';
import type {
  Quaternion,
  EulerAngles,
  MagCalibration,
  IMUCalibration,
  AxisRemap,
  FusionConfig
} from './types';

// Re-export types for backward compatibility
export type { Quaternion, EulerAngles, MagCalibration, IMUCalibration, AxisRemap, FusionConfig };

// Constants
const DEG_TO_RAD = Math.PI / 180.0;
const RAD_TO_DEG = 180.0 / Math.PI;

/**
 * Fusion output structure
 */
export interface FusionOutput {
  quaternion: Quaternion;
  euler: EulerAngles;
  heading: number;  // Magnetic heading in degrees (0-360)
}

/**
 * Madgwick AHRS Filter State
 */
export class MadgwickAHRS {
  // Quaternion state [w, x, y, z]
  private q: Quaternion;
  
  // Configuration
  private beta: number;
  private magCal: MagCalibration;
  private imuCal: IMUCalibration;
  private imuAxisRemap: AxisRemap;
  private magAxisRemap: AxisRemap;
  
  // Last magnetometer values (for async update)
  private lastMagX: number = 0;
  private lastMagY: number = 0;
  private lastMagZ: number = 0;
  private magValid: boolean = false;
  
  constructor(config?: Partial<FusionConfig>) {
    // Initialize to identity quaternion (no rotation)
    this.q = { w: 1, x: 0, y: 0, z: 0 };
    
    // Default configuration
    this.beta = config?.beta ?? 0.1;
    this.magCal = config?.magCalibration ?? {
      offsetX: 0, offsetY: 0, offsetZ: 0,
      scaleX: 1, scaleY: 1, scaleZ: 1
    };
    this.imuCal = config?.imuCalibration ?? {
      gyroBiasX: 0, gyroBiasY: 0, gyroBiasZ: 0,
      accelOffsetX: 0, accelOffsetY: 0, accelOffsetZ: 0
    };
    this.imuAxisRemap = config?.imuAxisRemap ?? DEFAULT_AXIS_REMAP;
    this.magAxisRemap = config?.magAxisRemap ?? DEFAULT_AXIS_REMAP;
  }
  
  /**
   * Reset filter to initial state
   */
  reset(): void {
    this.q = { w: 1, x: 0, y: 0, z: 0 };
    this.magValid = false;
  }
  
  /**
   * Initialize orientation from accelerometer and magnetometer
   * This computes an initial quaternion instead of starting at identity
   * 
   * The internal quaternion is stored in NWU frame (X=North, Y=West, Z=Up),
   * so we compute directly in that frame.
   * 
   * @param ax Accel X in g (raw, before remap but after calibration)
   * @param ay Accel Y in g
   * @param az Accel Z in g
   * @param mx Mag X in gauss (after transform and calibration, before remap)
   * @param my Mag Y in gauss
   * @param mz Mag Z in gauss
   */
  initFromAccelMag(ax: number, ay: number, az: number,
                   mx: number, my: number, mz: number): void {
    debug.log(`initFromAccelMag input (raw): accel=[${ax.toFixed(3)}, ${ay.toFixed(3)}, ${az.toFixed(3)}], mag=[${mx.toFixed(3)}, ${my.toFixed(3)}, ${mz.toFixed(3)}]`);
    
    // Apply axis remapping to get body frame values
    const accelBody = this.applyAxisRemap(ax, ay, az, this.imuAxisRemap);
    const magBody = this.applyAxisRemap(mx, my, mz, this.magAxisRemap);
    
    debug.log(`After axis remap (body): accel=[${accelBody.x.toFixed(3)}, ${accelBody.y.toFixed(3)}, ${accelBody.z.toFixed(3)}], mag=[${magBody.x.toFixed(3)}, ${magBody.y.toFixed(3)}, ${magBody.z.toFixed(3)}]`);
    
    // Transform to NWU frame for computation
    // Body: X=West, Y=Up, Z=North -> NWU: X=North, Y=West, Z=Up
    const accelNWU = this.bodyToNWU(accelBody.x, accelBody.y, accelBody.z);
    const magNWU = this.bodyToNWU(magBody.x, magBody.y, magBody.z);
    
    debug.log(`In NWU frame: accel=[${accelNWU.x.toFixed(3)}, ${accelNWU.y.toFixed(3)}, ${accelNWU.z.toFixed(3)}], mag=[${magNWU.x.toFixed(3)}, ${magNWU.y.toFixed(3)}, ${magNWU.z.toFixed(3)}]`);
    
    ax = accelNWU.x; ay = accelNWU.y; az = accelNWU.z;
    mx = magNWU.x; my = magNWU.y; mz = magNWU.z;
    
    // Normalize accelerometer (gravity direction)
    const aNorm = Math.sqrt(ax*ax + ay*ay + az*az);
    if (aNorm < MIN_VECTOR_MAGNITUDE) {
      debug.warn('Accel magnitude too small for initialization');
      return;
    }
    ax /= aNorm;
    ay /= aNorm;
    az /= aNorm;
    
    // Normalize magnetometer
    const mNorm = Math.sqrt(mx*mx + my*my + mz*mz);
    if (mNorm < MIN_VECTOR_MAGNITUDE) {
      debug.warn('Mag magnitude too small for initialization');
      return;
    }
    mx /= mNorm;
    my /= mNorm;
    mz /= mNorm;
    
    // === Build rotation matrix using TRIAD-like method in NWU frame ===
    // 
    // NWU frame convention (Madgwick standard):
    //   X = North, Y = West, Z = Up
    //
    // Reference vectors in WORLD frame (at identity orientation):
    //   Gravity points DOWN = [0, 0, -1] (negative Z)
    //   Magnetic North points roughly NORTH = [1, 0, 0] (positive X, ignoring dip)
    //
    // Measured vectors in SENSOR frame (after NWU transform):
    //   Accelerometer reads UP direction (reaction to gravity)
    //   up_sensor = [ax, ay, az] (normalized)
    //   Magnetometer reads toward magnetic north (with dip)
    
    // Step 1: "Up" vector in sensor frame (from accelerometer)
    const upX = ax;
    const upY = ay;
    const upZ = az;
    
    // Step 2: Project magnetometer to horizontal plane (remove dip)
    // mag_horizontal = mag - (mag · up) * up
    const magDotUp = mx * upX + my * upY + mz * upZ;
    let northX = mx - magDotUp * upX;
    let northY = my - magDotUp * upY;
    let northZ = mz - magDotUp * upZ;
    
    // Normalize horizontal north
    const nNorm = Math.sqrt(northX*northX + northY*northY + northZ*northZ);
    if (nNorm < MIN_VECTOR_MAGNITUDE) {
      debug.warn('Horizontal mag component too small');
      return;
    }
    northX /= nNorm;
    northY /= nNorm;
    northZ /= nNorm;
    
    // Step 3: West = Up × North (right-hand rule)
    const westX = upY * northZ - upZ * northY;
    const westY = upZ * northX - upX * northZ;
    const westZ = upX * northY - upY * northX;
    
    debug.log(`NWU sensor frame axes (at current orientation):`);
    debug.log(`  Up (sensor):    [${upX.toFixed(3)}, ${upY.toFixed(3)}, ${upZ.toFixed(3)}]`);
    debug.log(`  North (sensor): [${northX.toFixed(3)}, ${northY.toFixed(3)}, ${northZ.toFixed(3)}]`);
    debug.log(`  West (sensor):  [${westX.toFixed(3)}, ${westY.toFixed(3)}, ${westZ.toFixed(3)}]`);
    
    // Step 4: Build rotation matrix
    // 
    // We computed north, west, up = where world axes appear in SENSOR coordinates.
    // These form the COLUMNS of R_world_to_sensor:
    //   R_world_to_sensor * [1,0,0]_world = north_sensor
    //   R_world_to_sensor * [0,1,0]_world = west_sensor
    //   R_world_to_sensor * [0,0,1]_world = up_sensor
    //
    // The Madgwick quaternion q represents R_sensor_to_world:
    //   v_world = q ⊗ v_sensor ⊗ q*
    //
    // R_sensor_to_world = R_world_to_sensor^T = transpose
    //
    // So if R_world_to_sensor has [north, west, up] as columns,
    // then R_sensor_to_world has [north, west, up] as ROWS.
    
    // Build R_sensor_to_world with [north; west; up] as rows
    const r00 = northX,  r01 = northY,  r02 = northZ;  // row 0 = north
    const r10 = westX,   r11 = westY,   r12 = westZ;   // row 1 = west
    const r20 = upX,     r21 = upY,     r22 = upZ;     // row 2 = up
    
    debug.log(`Rotation matrix R_sensor_to_world (rows = world axes in sensor):`);
    debug.log(`  [${r00.toFixed(3)}, ${r01.toFixed(3)}, ${r02.toFixed(3)}]`);
    debug.log(`  [${r10.toFixed(3)}, ${r11.toFixed(3)}, ${r12.toFixed(3)}]`);
    debug.log(`  [${r20.toFixed(3)}, ${r21.toFixed(3)}, ${r22.toFixed(3)}]`);
    
    // Step 5: Convert rotation matrix to quaternion (Shepperd's method)
    const trace = r00 + r11 + r22;
    let qw: number, qx: number, qy: number, qz: number;
    
    if (trace > 0) {
      const s = 0.5 / Math.sqrt(trace + 1.0);
      qw = 0.25 / s;
      qx = (r21 - r12) * s;
      qy = (r02 - r20) * s;
      qz = (r10 - r01) * s;
    } else if (r00 > r11 && r00 > r22) {
      const s = 2.0 * Math.sqrt(1.0 + r00 - r11 - r22);
      qw = (r21 - r12) / s;
      qx = 0.25 * s;
      qy = (r01 + r10) / s;
      qz = (r02 + r20) / s;
    } else if (r11 > r22) {
      const s = 2.0 * Math.sqrt(1.0 + r11 - r00 - r22);
      qw = (r02 - r20) / s;
      qx = (r01 + r10) / s;
      qy = 0.25 * s;
      qz = (r12 + r21) / s;
    } else {
      const s = 2.0 * Math.sqrt(1.0 + r22 - r00 - r11);
      qw = (r10 - r01) / s;
      qx = (r02 + r20) / s;
      qy = (r12 + r21) / s;
      qz = 0.25 * s;
    }
    
    // Normalize quaternion - this is directly in NWU frame
    const qNorm = Math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    this.q = {
      w: qw / qNorm,
      x: qx / qNorm,
      y: qy / qNorm,
      z: qz / qNorm
    };
    
    debug.log(`Initialized orientation (NWU frame): q=[${this.q.w.toFixed(3)}, ${this.q.x.toFixed(3)}, ${this.q.y.toFixed(3)}, ${this.q.z.toFixed(3)}]`);
    
    // Debug: compute Euler angles to verify
    const euler = this.getEulerAngles();
    const headingDeg = euler.yaw * 180 / Math.PI;
    const pitchDeg = euler.pitch * 180 / Math.PI;
    const rollDeg = euler.roll * 180 / Math.PI;
    debug.log(`  -> Heading: ${headingDeg.toFixed(1)}°, Pitch: ${pitchDeg.toFixed(1)}°, Roll: ${rollDeg.toFixed(1)}°`);
  }
  
  /**
   * Initialize orientation from accelerometer only (6-DOF)
   * Sets level orientation with heading = 0 (North)
   * 
   * The internal quaternion is stored in NWU frame (X=North, Y=West, Z=Up),
   * so we compute directly in that frame.
   * 
   * @param ax Accel X in g (raw, before remap but after calibration)
   * @param ay Accel Y in g
   * @param az Accel Z in g
   */
  initFromAccelOnly(ax: number, ay: number, az: number): void {
    // Apply axis remapping to get body frame values
    const accelBody = this.applyAxisRemap(ax, ay, az, this.imuAxisRemap);
    
    // Transform to NWU frame
    const accelNWU = this.bodyToNWU(accelBody.x, accelBody.y, accelBody.z);
    ax = accelNWU.x; ay = accelNWU.y; az = accelNWU.z;
    
    // Normalize accelerometer (gravity direction)
    const aNorm = Math.sqrt(ax*ax + ay*ay + az*az);
    if (aNorm < MIN_VECTOR_MAGNITUDE) {
      debug.warn('Accel magnitude too small for initialization');
      return;
    }
    ax /= aNorm;
    ay /= aNorm;
    az /= aNorm;
    
    // In NWU frame: X=North, Y=West, Z=Up
    // Accelerometer reads UP direction (reaction to gravity)
    // When level, accel = [0, 0, 1] (pointing up = +Z)
    
    // Calculate roll and pitch from accelerometer
    // Roll = rotation around X (North) axis
    // Pitch = rotation around Y (West) axis
    const roll = Math.atan2(ay, az);   // atan2(West, Up)
    const pitch = Math.asin(-ax);       // asin(-North)
    
    // Convert to quaternion with yaw = 0 (heading = North)
    const cy = 1.0; // cos(yaw/2) = cos(0) = 1
    const sy = 0.0; // sin(yaw/2) = sin(0) = 0
    const cp = Math.cos(pitch / 2);
    const sp = Math.sin(pitch / 2);
    const cr = Math.cos(roll / 2);
    const sr = Math.sin(roll / 2);
    
    // Quaternion from Euler angles (ZYX order: yaw, pitch, roll)
    this.q = {
      w: cr * cp * cy + sr * sp * sy,
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy
    };
    
    debug.log(`Initialized orientation from accel only (6-DOF, NWU): q=[${this.q.w.toFixed(3)}, ${this.q.x.toFixed(3)}, ${this.q.y.toFixed(3)}, ${this.q.z.toFixed(3)}]`);
  }
  
  /**
   * Set filter gain (beta)
   */
  setBeta(beta: number): void {
    this.beta = Math.max(0.001, Math.min(1.0, beta));
  }
  
  /**
   * Get current beta value
   */
  getBeta(): number {
    return this.beta;
  }
  
  /**
   * Set magnetometer calibration
   */
  setMagCalibration(cal: MagCalibration): void {
    this.magCal = cal;
  }
  
  /**
   * Set IMU calibration (gyro bias + accel offset)
   */
  setIMUCalibration(cal: IMUCalibration): void {
    this.imuCal = cal;
  }
  
  /**
   * Get current IMU calibration
   */
  getIMUCalibration(): IMUCalibration {
    return { ...this.imuCal };
  }
  
  /**
   * Set gyroscope bias
   */
  setGyroBias(x: number, y: number, z: number): void {
    this.imuCal.gyroBiasX = x;
    this.imuCal.gyroBiasY = y;
    this.imuCal.gyroBiasZ = z;
  }
  
  /**
   * Set accelerometer offset
   */
  setAccelOffset(x: number, y: number, z: number): void {
    this.imuCal.accelOffsetX = x;
    this.imuCal.accelOffsetY = y;
    this.imuCal.accelOffsetZ = z;
  }
  
  /**
   * Set IMU axis remapping
   */
  setIMUAxisRemap(remap: AxisRemap): void {
    this.imuAxisRemap = remap;
  }
  
  /**
   * Set MAG axis remapping
   */
  setMagAxisRemap(remap: AxisRemap): void {
    this.magAxisRemap = remap;
  }
  
  /**
   * Get current axis remapping
   */
  getAxisRemap(): { imu: AxisRemap; mag: AxisRemap } {
    return { imu: { ...this.imuAxisRemap }, mag: { ...this.magAxisRemap } };
  }
  
  /**
   * Apply IMU axis remapping (public for diagnostics)
   */
  applyIMURemap(x: number, y: number, z: number): { x: number; y: number; z: number } {
    return this.applyAxisRemap(x, y, z, this.imuAxisRemap);
  }
  
  /**
   * Apply MAG axis remapping (public for diagnostics)
   */
  applyMagRemap(x: number, y: number, z: number): { x: number; y: number; z: number } {
    return this.applyAxisRemap(x, y, z, this.magAxisRemap);
  }
  
  /**
   * Apply axis remapping to sensor values
   */
  private applyAxisRemap(x: number, y: number, z: number, remap: AxisRemap): { x: number; y: number; z: number } {
    const getAxis = (mapping: string): number => {
      switch (mapping) {
        case '+X': return x;
        case '-X': return -x;
        case '+Y': return y;
        case '-Y': return -y;
        case '+Z': return z;
        case '-Z': return -z;
        default: return 0;
      }
    };
    
    return {
      x: getAxis(remap.bodyX),
      y: getAxis(remap.bodyY),
      z: getAxis(remap.bodyZ)
    };
  }
  
  /**
   * Update with new magnetometer data
   * Called at ~100 Hz when MAG samples arrive
   * 
   * @param mx Raw magnetometer X (gauss)
   * @param my Raw magnetometer Y (gauss)
   * @param mz Raw magnetometer Z (gauss)
   */
  updateMag(mx: number, my: number, mz: number): void {
    // Apply hard iron calibration (offset removal)
    mx = (mx - this.magCal.offsetX) * this.magCal.scaleX;
    my = (my - this.magCal.offsetY) * this.magCal.scaleY;
    mz = (mz - this.magCal.offsetZ) * this.magCal.scaleZ;
    
    // Apply axis remapping (sensor frame to body frame)
    // Use the MAG axis remap dropdowns to configure any axis inversions/swaps
    const remapped = this.applyAxisRemap(mx, my, mz, this.magAxisRemap);
    
    // Store for use in IMU update
    this.lastMagX = remapped.x;
    this.lastMagY = remapped.y;
    this.lastMagZ = remapped.z;
    this.magValid = true;
  }
  
  /**
   * Transform from FlySight body frame to Madgwick NWU frame
   * 
   * FlySight body: X=West, Y=Up, Z=North
   * Madgwick NWU:  X=North, Y=West, Z=Up
   * 
   * Transform: NWU_X = Body_Z, NWU_Y = Body_X, NWU_Z = Body_Y
   */
  private bodyToNWU(x: number, y: number, z: number): { x: number; y: number; z: number } {
    return { x: z, y: x, z: y };
  }
  
  /**
   * Transform from Madgwick NWU frame to FlySight body frame
  /**
   * Update with IMU data (gyro + accel)
   * Called at ~400 Hz for each IMU sample
   * Uses stored magnetometer data for 9-DOF fusion
   * 
   * @param dt Time delta in seconds
   * @param gx Gyro X in deg/s
   * @param gy Gyro Y in deg/s
   * @param gz Gyro Z in deg/s
   * @param ax Accel X in g
   * @param ay Accel Y in g
   * @param az Accel Z in g
   */
  updateIMU(dt: number, gx: number, gy: number, gz: number,
            ax: number, ay: number, az: number): void {
    // Apply IMU calibration - subtract gyro bias
    gx -= this.imuCal.gyroBiasX;
    gy -= this.imuCal.gyroBiasY;
    gz -= this.imuCal.gyroBiasZ;
    
    // Apply IMU calibration - subtract accel offset
    ax -= this.imuCal.accelOffsetX;
    ay -= this.imuCal.accelOffsetY;
    az -= this.imuCal.accelOffsetZ;
    
    // Apply axis remapping (sensor frame to body frame)
    const gyroRemap = this.applyAxisRemap(gx, gy, gz, this.imuAxisRemap);
    gx = gyroRemap.x;
    gy = gyroRemap.y;
    gz = gyroRemap.z;
    
    const accelRemap = this.applyAxisRemap(ax, ay, az, this.imuAxisRemap);
    ax = accelRemap.x;
    ay = accelRemap.y;
    az = accelRemap.z;
    
    // Convert gyro from deg/s to rad/s
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;
    
    // Transform from FlySight body frame (X=West, Y=Up, Z=North) to 
    // Madgwick NWU frame (X=North, Y=West, Z=Up) for the algorithm
    const gyroNWU = this.bodyToNWU(gx, gy, gz);
    const accelNWU = this.bodyToNWU(ax, ay, az);
    
    if (this.magValid) {
      const magNWU = this.bodyToNWU(this.lastMagX, this.lastMagY, this.lastMagZ);
      this.madgwickAHRSupdate(dt, 
        gyroNWU.x, gyroNWU.y, gyroNWU.z,
        accelNWU.x, accelNWU.y, accelNWU.z,
        magNWU.x, magNWU.y, magNWU.z);
    } else {
      this.madgwickAHRSupdateIMU(dt, 
        gyroNWU.x, gyroNWU.y, gyroNWU.z,
        accelNWU.x, accelNWU.y, accelNWU.z);
    }
  }
  
  /**
   * 9-DOF Madgwick AHRS update (gyro + accel + mag)
   * This is the core algorithm from Madgwick's paper
   */
  private madgwickAHRSupdate(
    dt: number,
    gx: number, gy: number, gz: number,  // rad/s
    ax: number, ay: number, az: number,  // g
    mx: number, my: number, mz: number   // gauss (calibrated)
  ): void {
    let q0 = this.q.w, q1 = this.q.x, q2 = this.q.y, q3 = this.q.z;
    let recipNorm: number;
    let s0: number, s1: number, s2: number, s3: number;
    let qDot1: number, qDot2: number, qDot3: number, qDot4: number;
    let hx: number, hy: number;
    let _2q0mx: number, _2q0my: number, _2q0mz: number, _2q1mx: number;
    let _2bx: number, _2bz: number;
    let _4bx: number, _4bz: number;
    let _2q0: number, _2q1: number, _2q2: number, _2q3: number;
    let _2q0q2: number, _2q2q3: number;
    let q0q0: number, q0q1: number, q0q2: number, q0q3: number;
    let q1q1: number, q1q2: number, q1q3: number;
    let q2q2: number, q2q3: number;
    let q3q3: number;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
    
    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    const accelMag = ax * ax + ay * ay + az * az;
    if (accelMag > 0.01) {
      // Normalise accelerometer measurement
      recipNorm = 1.0 / Math.sqrt(accelMag);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;
      
      // Normalise magnetometer measurement
      const magMag = mx * mx + my * my + mz * mz;
      if (magMag > 0.01) {
        recipNorm = 1.0 / Math.sqrt(magMag);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0 * q0 * mx;
        _2q0my = 2.0 * q0 * my;
        _2q0mz = 2.0 * q0 * mz;
        _2q1mx = 2.0 * q1 * mx;
        _2q0 = 2.0 * q0;
        _2q1 = 2.0 * q1;
        _2q2 = 2.0 * q2;
        _2q3 = 2.0 * q3;
        _2q0q2 = 2.0 * q0 * q2;
        _2q2q3 = 2.0 * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;
        
        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 +
             _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 -
             my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = Math.sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 -
               mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0 * _2bx;
        _4bz = 2.0 * _2bz;
        
        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) +
             _2q1 * (2.0 * q0q1 + _2q2q3 - ay) -
             _2bz * q2 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) +
             _2q0 * (2.0 * q0q1 + _2q2q3 - ay) -
             4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) +
             _2bz * q3 * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) +
             _2q3 * (2.0 * q0q1 + _2q2q3 - ay) -
             4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) +
             (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) +
             _2q2 * (2.0 * q0q1 + _2q2q3 - ay) +
             (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
        
        // Normalise step magnitude
        recipNorm = 1.0 / Math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step
        qDot1 -= this.beta * s0;
        qDot2 -= this.beta * s1;
        qDot3 -= this.beta * s2;
        qDot4 -= this.beta * s3;
      }
    }
    
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    
    // Normalise quaternion
    recipNorm = 1.0 / Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    this.q.w = q0 * recipNorm;
    this.q.x = q1 * recipNorm;
    this.q.y = q2 * recipNorm;
    this.q.z = q3 * recipNorm;
  }
  
  /**
   * 6-DOF IMU-only update (gyro + accel, no magnetometer)
   * Used when no valid magnetometer data is available
   */
  private madgwickAHRSupdateIMU(
    dt: number,
    gx: number, gy: number, gz: number,  // rad/s
    ax: number, ay: number, az: number   // g
  ): void {
    let q0 = this.q.w, q1 = this.q.x, q2 = this.q.y, q3 = this.q.z;
    let recipNorm: number;
    let s0: number, s1: number, s2: number, s3: number;
    let qDot1: number, qDot2: number, qDot3: number, qDot4: number;
    let _2q0: number, _2q1: number, _2q2: number, _2q3: number;
    let _4q0: number, _4q1: number, _4q2: number;
    let _8q1: number, _8q2: number;
    let q0q0: number, q1q1: number, q2q2: number, q3q3: number;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx);
    
    // Compute feedback only if accelerometer measurement valid
    const accelMag = ax * ax + ay * ay + az * az;
    if (accelMag > 0.01) {
      // Normalise accelerometer measurement
      recipNorm = 1.0 / Math.sqrt(accelMag);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;
      
      // Auxiliary variables
      _2q0 = 2.0 * q0;
      _2q1 = 2.0 * q1;
      _2q2 = 2.0 * q2;
      _2q3 = 2.0 * q3;
      _4q0 = 4.0 * q0;
      _4q1 = 4.0 * q1;
      _4q2 = 4.0 * q2;
      _8q1 = 8.0 * q1;
      _8q2 = 8.0 * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3;
      
      // Gradient descent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay;
      
      // Normalise step magnitude
      recipNorm = 1.0 / Math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;
      
      // Apply feedback step
      qDot1 -= this.beta * s0;
      qDot2 -= this.beta * s1;
      qDot3 -= this.beta * s2;
      qDot4 -= this.beta * s3;
    }
    
    // Integrate rate of change of quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    
    // Normalise quaternion
    recipNorm = 1.0 / Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    this.q.w = q0 * recipNorm;
    this.q.x = q1 * recipNorm;
    this.q.y = q2 * recipNorm;
    this.q.z = q3 * recipNorm;
  }
  
  /**
   * Get current quaternion
   * 
   * The quaternion is in NWU frame (X=North, Y=West, Z=Up).
   * This is the convention used by the Madgwick algorithm.
   */
  getQuaternion(): Quaternion {
    return { ...this.q };
  }
  
  /**
   * Convert quaternion to Euler angles
   * 
   * In NWU frame (X=North, Y=West, Z=Up):
   * - Roll: rotation around X (North) axis - banking left/right
   * - Pitch: rotation around Y (West) axis - nose up/down
   * - Yaw: rotation around Z (Up) axis - heading
   * 
   * Returns standard aerospace Euler angles (ZYX rotation order)
   */
  getEulerAngles(): EulerAngles {
    const q0 = this.q.w, q1 = this.q.x, q2 = this.q.y, q3 = this.q.z;
    
    // Standard ZYX Euler angle extraction for NWU frame
    // Roll (rotation around X/North axis)
    const sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
    const cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (rotation around Y/West axis)
    const sinp = 2.0 * (q0 * q2 - q3 * q1);
    let pitch: number;
    if (Math.abs(sinp) >= 1) {
      pitch = Math.sign(sinp) * Math.PI / 2;
    } else {
      pitch = Math.asin(sinp);
    }
    
    // Yaw (rotation around Z/Up axis) - this is the heading
    const siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
    const cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);
    
    return { roll, pitch, yaw };
  }
  
  /**
   * Get magnetic heading in degrees (0-360)
   */
  getHeading(): number {
    const euler = this.getEulerAngles();
    let heading = euler.yaw * RAD_TO_DEG;
    
    // Convert to 0-360 range
    if (heading < 0) {
      heading += 360;
    }
    
    return heading;
  }
  
  /**
   * Get full fusion output
   */
  getOutput(): FusionOutput {
    const quaternion = this.getQuaternion();
    const euler = this.getEulerAngles();
    const heading = this.getHeading();
    
    return { quaternion, euler, heading };
  }
  
  /**
   * Get the last calibrated magnetometer values (in device frame)
   */
  getCalibratedMag(): { x: number; y: number; z: number; valid: boolean } {
    return {
      x: this.lastMagX,
      y: this.lastMagY,
      z: this.lastMagZ,
      valid: this.magValid
    };
  }
  
  /**
   * Get current mag calibration
   */
  getMagCalibration(): MagCalibration {
    return { ...this.magCal };
  }
}

// Export utility functions
export { DEG_TO_RAD, RAD_TO_DEG };
