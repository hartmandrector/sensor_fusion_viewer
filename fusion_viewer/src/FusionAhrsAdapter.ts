/**
 * FusionAhrsAdapter - Adapter to integrate FusionAhrs with existing viewer architecture
 * 
 * Provides the same interface as MadgwickAHRS for drop-in replacement.
 * Handles coordinate transforms (sensor → body → world) and calibration.
 * 
 * @license MIT
 */

import { FusionAhrs, type FusionVector, type FusionAhrsSettings, type FusionQuaternion } from './FusionAhrs';
import { FusionBias, type FusionBiasSettings } from './FusionBias';
import { DEFAULT_AXIS_REMAP, MIN_VECTOR_MAGNITUDE, debug } from './constants';
import type { 
  Quaternion, 
  EulerAngles, 
  MagCalibration, 
  IMUCalibration, 
  AxisRemap, 
  FusionConfig 
} from './types';

// ============================================================================
// Constants
// ============================================================================

const DEG_TO_RAD = Math.PI / 180.0;
const RAD_TO_DEG = 180.0 / Math.PI;

// ============================================================================
// Output Type
// ============================================================================

export interface FusionOutput {
  quaternion: Quaternion;
  euler: EulerAngles;
  heading: number;
}

// ============================================================================
// Extended Settings
// ============================================================================

export interface FusionAdapterSettings {
  /** Core AHRS settings */
  ahrs: Partial<FusionAhrsSettings>;
  
  /** Gyro bias estimation settings (null to disable) */
  bias: Partial<FusionBiasSettings> | null;
  
  /** Enable runtime bias estimation */
  enableBiasEstimation: boolean;
}

export const DEFAULT_ADAPTER_SETTINGS: FusionAdapterSettings = {
  ahrs: {
    gain: 0.5,
    gyroscopeRange: 2000,        // FlySight default ±2000°/s
    accelerationRejection: 10,   // 10° threshold
    magneticRejection: 10,       // 10° threshold
    recoveryTriggerPeriod: 416 * 5  // 5 seconds at 416Hz (~2080 samples)
  },
  bias: {},
  enableBiasEstimation: true      // Ch.7 algorithm estimates bias during initial gain ramp
};

// ============================================================================
// FusionAhrsAdapter Class
// ============================================================================

export class FusionAhrsAdapter {
  // Core components
  private ahrs: FusionAhrs;
  private bias: FusionBias;
  private settings: FusionAdapterSettings;
  
  // Calibration
  private magCal: MagCalibration;
  private imuCal: IMUCalibration;
  private imuAxisRemap: AxisRemap;
  private magAxisRemap: AxisRemap;
  
  // Full calibration matrices (optional, for advanced calibration)
  private accelScaleMatrix: number[][] | null = null;
  private softIronMatrix: number[][] | null = null;
  
  // Last magnetometer (for async update)
  private lastMagNWU: FusionVector;
  private lastMagBody: FusionVector;  // Body frame for display
  private magValid: boolean = false;
  
  constructor(config?: Partial<FusionConfig>, adapterSettings?: Partial<FusionAdapterSettings>) {
    // Merge settings
    this.settings = { 
      ...DEFAULT_ADAPTER_SETTINGS,
      ...adapterSettings,
      ahrs: { ...DEFAULT_ADAPTER_SETTINGS.ahrs, ...adapterSettings?.ahrs },
      bias: adapterSettings?.bias ?? DEFAULT_ADAPTER_SETTINGS.bias
    };
    
    // Map beta to gain if provided
    if (config?.beta !== undefined) {
      this.settings.ahrs.gain = config.beta;
    }
    
    // Initialize AHRS
    this.ahrs = new FusionAhrs(this.settings.ahrs);
    this.bias = new FusionBias(this.settings.bias ?? {});
    
    // Calibration defaults
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
    
    this.lastMagNWU = { x: 0, y: 0, z: 0 };
    this.lastMagBody = { x: 0, y: 0, z: 0 };
    this.magValid = false;
  }
  
  // ===========================================================================
  // Configuration
  // ===========================================================================
  
  updateAhrsSettings(settings: Partial<FusionAhrsSettings>): void {
    this.settings.ahrs = { ...this.settings.ahrs, ...settings };
    this.ahrs.applySettings(settings);
  }
  
  updateBiasSettings(settings: Partial<FusionBiasSettings>): void {
    if (this.settings.bias) {
      this.settings.bias = { ...this.settings.bias, ...settings };
      this.bias.applySettings(settings);
    }
  }
  
  setEnableBiasEstimation(enable: boolean): void {
    this.settings.enableBiasEstimation = enable;
    if (!enable) {
      this.bias.reset();
    }
  }
  
  // ===========================================================================
  // Calibration Access (same interface as MadgwickAHRS)
  // ===========================================================================
  
  getMagCalibration(): MagCalibration {
    return { ...this.magCal };
  }
  
  setMagCalibration(cal: MagCalibration): void {
    this.magCal = { ...cal };
  }
  
  getIMUCalibration(): IMUCalibration {
    return { ...this.imuCal };
  }
  
  setIMUCalibration(cal: IMUCalibration): void {
    this.imuCal = { ...cal };
  }
  
  setGyroBias(x: number, y: number, z: number): void {
    this.imuCal.gyroBiasX = x;
    this.imuCal.gyroBiasY = y;
    this.imuCal.gyroBiasZ = z;
  }
  
  setAccelOffset(x: number, y: number, z: number): void {
    this.imuCal.accelOffsetX = x;
    this.imuCal.accelOffsetY = y;
    this.imuCal.accelOffsetZ = z;
  }
  
  /**
   * Set full 3x3 accelerometer scale matrix (inverse of the scale matrix from calibration)
   * This corrects for scale factors and cross-axis coupling
   */
  setAccelScaleMatrix(matrix: number[][] | null): void {
    if (matrix && matrix.length === 3 && matrix[0].length === 3) {
      this.accelScaleMatrix = matrix.map(row => [...row]);
      debug.log('Accel scale matrix set:', this.accelScaleMatrix);
    } else {
      this.accelScaleMatrix = null;
    }
  }
  
  /**
   * Set soft iron correction matrix for magnetometer
   * This corrects for soft iron distortion (ellipsoid → sphere)
   */
  setSoftIronMatrix(matrix: number[][] | null): void {
    if (matrix && matrix.length === 3 && matrix[0].length === 3) {
      this.softIronMatrix = matrix.map(row => [...row]);
      debug.log('Soft iron matrix set:', this.softIronMatrix);
    } else {
      this.softIronMatrix = null;
    }
  }
  
  /**
   * Get current accel scale matrix
   */
  getAccelScaleMatrix(): number[][] | null {
    return this.accelScaleMatrix ? this.accelScaleMatrix.map(row => [...row]) : null;
  }
  
  /**
   * Get current soft iron matrix
   */
  getSoftIronMatrix(): number[][] | null {
    return this.softIronMatrix ? this.softIronMatrix.map(row => [...row]) : null;
  }
  
  // ===========================================================================
  // Axis Remapping
  // ===========================================================================
  
  setIMUAxisRemap(remap: AxisRemap): void {
    this.imuAxisRemap = { ...remap };
  }
  
  setMagAxisRemap(remap: AxisRemap): void {
    this.magAxisRemap = { ...remap };
  }
  
  // ===========================================================================
  // Reset
  // ===========================================================================
  
  reset(): void {
    this.ahrs.reset();
    this.bias.reset();
    this.magValid = false;
    this.lastMagNWU = { x: 0, y: 0, z: 0 };
    this.lastMagBody = { x: 0, y: 0, z: 0 };
  }
  
  // ===========================================================================
  // Coordinate Transforms
  // ===========================================================================
  
  /**
   * Apply axis remap: sensor frame → body frame
   */
  applyAxisRemap(x: number, y: number, z: number, remap: AxisRemap): FusionVector {
    const getValue = (axis: typeof remap.bodyX): number => {
      switch (axis) {
        case '+X': return x;
        case '-X': return -x;
        case '+Y': return y;
        case '-Y': return -y;
        case '+Z': return z;
        case '-Z': return -z;
      }
    };
    return {
      x: getValue(remap.bodyX),
      y: getValue(remap.bodyY),
      z: getValue(remap.bodyZ)
    };
  }
  
  /**
   * Public wrapper for IMU axis remap
   */
  applyIMURemap(x: number, y: number, z: number): FusionVector {
    return this.applyAxisRemap(x, y, z, this.imuAxisRemap);
  }
  
  /**
   * Body frame → NWU world frame transform
   * FlySight body: X=West, Y=Up, Z=North
   * NWU: X=North, Y=West, Z=Up
   */
  private bodyToNWU(bx: number, by: number, bz: number): FusionVector {
    return { x: bz, y: bx, z: by };  // North=bodyZ, West=bodyX, Up=bodyY
  }
  
  // ===========================================================================
  // Update Methods (same interface as MadgwickAHRS)
  // ===========================================================================
  
  /**
   * Update magnetometer (stored for next IMU update)
   */
  updateMag(mx: number, my: number, mz: number): void {
    // Apply hard iron calibration (subtract offset)
    let calX = mx - this.magCal.offsetX;
    let calY = my - this.magCal.offsetY;
    let calZ = mz - this.magCal.offsetZ;
    
    // Apply soft iron matrix if available (full ellipsoid correction)
    if (this.softIronMatrix) {
      const W = this.softIronMatrix;
      const newX = W[0][0] * calX + W[0][1] * calY + W[0][2] * calZ;
      const newY = W[1][0] * calX + W[1][1] * calY + W[1][2] * calZ;
      const newZ = W[2][0] * calX + W[2][1] * calY + W[2][2] * calZ;
      calX = newX;
      calY = newY;
      calZ = newZ;
    } else {
      // Fall back to simple scale factors
      calX *= this.magCal.scaleX;
      calY *= this.magCal.scaleY;
      calZ *= this.magCal.scaleZ;
    }
    
    // Remap to body frame
    const body = this.applyAxisRemap(calX, calY, calZ, this.magAxisRemap);
    this.lastMagBody = { ...body };  // Store body frame for display
    
    // Transform to NWU for algorithm
    this.lastMagNWU = this.bodyToNWU(body.x, body.y, body.z);
    this.magValid = true;
  }
  
  /**
   * Update with IMU data (uses stored magnetometer)
   * 
   * @param dt Time step in seconds
   * @param wx Gyro X in deg/s (raw sensor)
   * @param wy Gyro Y in deg/s
   * @param wz Gyro Z in deg/s
   * @param ax Accel X in g (raw sensor)
   * @param ay Accel Y in g
   * @param az Accel Z in g
   */
  updateIMU(dt: number, wx: number, wy: number, wz: number, ax: number, ay: number, az: number): void {
    // Apply gyro calibration (bias subtraction)
    const gx = (wx - this.imuCal.gyroBiasX) * DEG_TO_RAD;
    const gy = (wy - this.imuCal.gyroBiasY) * DEG_TO_RAD;
    const gz = (wz - this.imuCal.gyroBiasZ) * DEG_TO_RAD;
    
    // Apply accel calibration
    let calAx: number, calAy: number, calAz: number;
    
    if (this.accelScaleMatrix) {
      // Full calibration: corrected = S^(-1) * (raw - bias)
      // First subtract bias
      const bx = ax - this.imuCal.accelOffsetX;
      const by = ay - this.imuCal.accelOffsetY;
      const bz = az - this.imuCal.accelOffsetZ;
      
      // Then apply inverse scale matrix
      const S = this.accelScaleMatrix;
      calAx = S[0][0] * bx + S[0][1] * by + S[0][2] * bz;
      calAy = S[1][0] * bx + S[1][1] * by + S[1][2] * bz;
      calAz = S[2][0] * bx + S[2][1] * by + S[2][2] * bz;
    } else {
      // Simple calibration: just subtract offset
      calAx = ax - this.imuCal.accelOffsetX;
      calAy = ay - this.imuCal.accelOffsetY;
      calAz = az - this.imuCal.accelOffsetZ;
    }
    
    // Remap to body frame
    const gyroBody = this.applyAxisRemap(gx, gy, gz, this.imuAxisRemap);
    const accelBody = this.applyAxisRemap(calAx, calAy, calAz, this.imuAxisRemap);
    
    // Transform to NWU
    const gyroNWU = this.bodyToNWU(gyroBody.x, gyroBody.y, gyroBody.z);
    const accelNWU = this.bodyToNWU(accelBody.x, accelBody.y, accelBody.z);
    
    // Apply runtime bias estimation if enabled
    let finalGyro = gyroNWU;
    if (this.settings.enableBiasEstimation) {
      finalGyro = this.bias.update(gyroNWU, dt);
    }
    
    // Run AHRS update
    if (this.magValid) {
      this.ahrs.update(finalGyro, accelNWU, this.lastMagNWU, dt);
    } else {
      this.ahrs.updateNoMagnetometer(finalGyro, accelNWU, dt);
    }
  }
  
  // ===========================================================================
  // Initialization
  // ===========================================================================
  
  /**
   * Initialize from accelerometer and magnetometer
   */
  initFromAccelMag(ax: number, ay: number, az: number, mx: number, my: number, mz: number): void {
    debug.log(`[FusionAdapter] initFromAccelMag: accel=[${ax.toFixed(3)}, ${ay.toFixed(3)}, ${az.toFixed(3)}]`);
    
    // Transform to body then NWU
    const accelBody = this.applyAxisRemap(ax, ay, az, this.imuAxisRemap);
    const magBody = this.applyAxisRemap(mx, my, mz, this.magAxisRemap);
    
    const accelNWU = this.bodyToNWU(accelBody.x, accelBody.y, accelBody.z);
    const magNWU = this.bodyToNWU(magBody.x, magBody.y, magBody.z);
    
    // Use TRIAD method to compute initial quaternion
    const q = this.computeInitialQuaternion(accelNWU, magNWU);
    if (q) {
      this.ahrs.setQuaternion(q);
      debug.log(`[FusionAdapter] Initialized quaternion: w=${q.w.toFixed(3)}, x=${q.x.toFixed(3)}, y=${q.y.toFixed(3)}, z=${q.z.toFixed(3)}`);
    }
    
    // Store mag for subsequent updates
    this.lastMagNWU = magNWU;
    this.magValid = true;
  }
  
  /**
   * Initialize from accelerometer only (6-DOF)
   */
  initFromAccelOnly(ax: number, ay: number, az: number): void {
    debug.log(`[FusionAdapter] initFromAccelOnly: accel=[${ax.toFixed(3)}, ${ay.toFixed(3)}, ${az.toFixed(3)}]`);
    
    const accelBody = this.applyAxisRemap(ax, ay, az, this.imuAxisRemap);
    const accelNWU = this.bodyToNWU(accelBody.x, accelBody.y, accelBody.z);
    
    // Compute pitch and roll from gravity
    const norm = Math.sqrt(accelNWU.x ** 2 + accelNWU.y ** 2 + accelNWU.z ** 2);
    if (norm < MIN_VECTOR_MAGNITUDE) return;
    
    const ax_n = accelNWU.x / norm;
    const ay_n = accelNWU.y / norm;
    const az_n = accelNWU.z / norm;
    
    // Roll from Y-Z, Pitch from X-Z
    const roll = Math.atan2(ay_n, az_n);
    const pitch = Math.atan2(-ax_n, Math.sqrt(ay_n ** 2 + az_n ** 2));
    
    // Build quaternion (yaw = 0)
    const cr = Math.cos(roll * 0.5);
    const sr = Math.sin(roll * 0.5);
    const cp = Math.cos(pitch * 0.5);
    const sp = Math.sin(pitch * 0.5);
    
    const q: FusionQuaternion = {
      w: cr * cp,
      x: sr * cp,
      y: cr * sp,
      z: -sr * sp
    };
    
    this.ahrs.setQuaternion(q);
    debug.log(`[FusionAdapter] Initialized (6-DOF): roll=${(roll * RAD_TO_DEG).toFixed(1)}°, pitch=${(pitch * RAD_TO_DEG).toFixed(1)}°`);
  }
  
  /**
   * Compute initial quaternion using TRIAD method
   */
  private computeInitialQuaternion(accelNWU: FusionVector, magNWU: FusionVector): FusionQuaternion | null {
    // Normalize accelerometer
    const aNorm = Math.sqrt(accelNWU.x ** 2 + accelNWU.y ** 2 + accelNWU.z ** 2);
    if (aNorm < MIN_VECTOR_MAGNITUDE) return null;
    
    const ax = accelNWU.x / aNorm;
    const ay = accelNWU.y / aNorm;
    const az = accelNWU.z / aNorm;
    
    // Normalize magnetometer
    const mNorm = Math.sqrt(magNWU.x ** 2 + magNWU.y ** 2 + magNWU.z ** 2);
    if (mNorm < MIN_VECTOR_MAGNITUDE) return null;
    
    const mx = magNWU.x / mNorm;
    const my = magNWU.y / mNorm;
    const mz = magNWU.z / mNorm;
    
    // Up vector from accelerometer
    const upX = ax, upY = ay, upZ = az;
    
    // Project mag to horizontal (remove dip)
    const magDotUp = mx * upX + my * upY + mz * upZ;
    let northX = mx - magDotUp * upX;
    let northY = my - magDotUp * upY;
    let northZ = mz - magDotUp * upZ;
    
    const nNorm = Math.sqrt(northX ** 2 + northY ** 2 + northZ ** 2);
    if (nNorm < MIN_VECTOR_MAGNITUDE) return null;
    
    northX /= nNorm;
    northY /= nNorm;
    northZ /= nNorm;
    
    // West = Up × North
    const westX = upY * northZ - upZ * northY;
    const westY = upZ * northX - upX * northZ;
    const westZ = upX * northY - upY * northX;
    
    // Build rotation matrix R (rows are north, west, up)
    // and convert to quaternion
    const r11 = northX, r12 = northY, r13 = northZ;
    const r21 = westX,  r22 = westY,  r23 = westZ;
    const r31 = upX,    r32 = upY,    r33 = upZ;
    
    // Shepperd's method for robust conversion
    const trace = r11 + r22 + r33;
    let w: number, x: number, y: number, z: number;
    
    if (trace > 0) {
      const s = 0.5 / Math.sqrt(trace + 1);
      w = 0.25 / s;
      x = (r32 - r23) * s;
      y = (r13 - r31) * s;
      z = (r21 - r12) * s;
    } else if (r11 > r22 && r11 > r33) {
      const s = 2 * Math.sqrt(1 + r11 - r22 - r33);
      w = (r32 - r23) / s;
      x = 0.25 * s;
      y = (r12 + r21) / s;
      z = (r13 + r31) / s;
    } else if (r22 > r33) {
      const s = 2 * Math.sqrt(1 + r22 - r11 - r33);
      w = (r13 - r31) / s;
      x = (r12 + r21) / s;
      y = 0.25 * s;
      z = (r23 + r32) / s;
    } else {
      const s = 2 * Math.sqrt(1 + r33 - r11 - r22);
      w = (r21 - r12) / s;
      x = (r13 + r31) / s;
      y = (r23 + r32) / s;
      z = 0.25 * s;
    }
    
    // Normalize
    const qNorm = Math.sqrt(w * w + x * x + y * y + z * z);
    return { w: w / qNorm, x: x / qNorm, y: y / qNorm, z: z / qNorm };
  }
  
  // ===========================================================================
  // Output Methods
  // ===========================================================================
  
  /**
   * Get fusion output (same format as MadgwickAHRS)
   */
  getOutput(): FusionOutput {
    const q = this.ahrs.getQuaternion();
    const euler = this.ahrs.getEulerAngles();
    
    // Heading from yaw (convert to 0-360)
    let heading = euler.yaw * RAD_TO_DEG;
    if (heading < 0) heading += 360;
    
    return {
      quaternion: { w: q.w, x: q.x, y: q.y, z: q.z },
      euler: { roll: euler.roll, pitch: euler.pitch, yaw: euler.yaw },
      heading
    };
  }
  
  /**
   * Get calibrated magnetometer (for display - in body frame like MadgwickAHRS)
   */
  getCalibratedMag(): { x: number; y: number; z: number; valid: boolean } {
    return { ...this.lastMagBody, valid: this.magValid };
  }
  
  /**
   * Get linear acceleration (sensor frame, gravity removed)
   */
  getLinearAcceleration(): FusionVector {
    return this.ahrs.getLinearAcceleration();
  }
  
  /**
   * Get Earth-frame acceleration (world frame, gravity removed)
   * This is what you need for trajectory estimation!
   */
  getEarthAcceleration(): FusionVector {
    return this.ahrs.getEarthAcceleration();
  }
  
  /**
   * Get gravity direction in sensor frame
   */
  getGravity(): FusionVector {
    return this.ahrs.getGravity();
  }
  
  /**
   * Get internal states for diagnostics
   */
  getInternalStates() {
    return this.ahrs.getInternalStates();
  }
  
  /**
   * Get status flags
   */
  getFlags() {
    return this.ahrs.getFlags();
  }
  
  /**
   * Get gyro bias estimation state
   */
  getBiasState() {
    return {
      bias: this.bias.getBiasDegrees(),
      isCalibrating: this.bias.isCurrentlyCalibrating(),
      progress: this.bias.getCalibrationProgress(),
      stationaryTime: this.bias.getStationaryTime(),
      gyroMagnitude: this.bias.getGyroMagnitudeDegrees(),
      stationaryThreshold: this.bias.getStationaryThresholdDegrees()
    };
  }
  
  /**
   * Get current gain (may be ramped during initialization)
   */
  getCurrentSettings(): FusionAdapterSettings {
    return { ...this.settings };
  }
}
