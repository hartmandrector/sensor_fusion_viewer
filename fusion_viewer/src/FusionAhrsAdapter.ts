/**
 * FusionAhrsAdapter - Adapter to integrate FusionAhrs with existing viewer architecture
 * 
 * Provides the same interface as MadgwickAHRS for drop-in replacement.
 * Handles coordinate transforms (sensor → body → NWU) and calibration.
 * 
 * ============================================================================
 * COORDINATE SYSTEM PIPELINE
 * ============================================================================
 * 
 * 1. SENSOR FRAME (raw data from IMU/Mag chips)
 *    - Axes defined by sensor orientation on PCB
 *    - May need axis remapping to align with device body
 * 
 * 2. DEVICE BODY FRAME (FlySight 2 convention)
 *    - X = West (left side when facing front)
 *    - Y = Up (toward LED)
 *    - Z = North (out the front face)
 * 
 * 3. NWU ALGORITHM FRAME (Fusion library convention)
 *    - X = North
 *    - Y = West
 *    - Z = Up
 * 
 * INPUT PATH (sensor data → algorithm):
 *   Sensor → (axis remap) → Body → (bodyToNWU) → NWU → FusionAhrs.update()
 * 
 * OUTPUT PATH (algorithm → display):
 *   FusionAhrs outputs → (nwuToBody for body-frame quantities) → Display
 *   
 *   Note: Earth-frame acceleration stays in NWU (it's a world-frame vector)
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
  
  /** Recovery time in seconds (used to calculate recoveryTriggerPeriod based on IMU rate) */
  recoveryTimeSeconds?: number;
}

export const DEFAULT_ADAPTER_SETTINGS: FusionAdapterSettings = {
  ahrs: {
    gain: 0.5,
    gyroscopeRange: 2000,        // FlySight default ±2000°/s
    accelerationRejection: 10,   // 10° threshold
    magneticRejection: 10,       // 10° threshold
    recoveryTriggerPeriod: 416 * 5  // 5 seconds at 416Hz (~2080 samples) - recalculated based on IMU rate
  },
  bias: {},
  enableBiasEstimation: true,     // Ch.7 algorithm estimates bias during initial gain ramp
  recoveryTimeSeconds: 5          // Recovery time in seconds (used to calculate recoveryTriggerPeriod)
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
  
  // Last accelerometer (for compass heading calculation)
  private lastAccelNWU: FusionVector;
  
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
    this.lastAccelNWU = { x: 0, y: 0, z: 0 };
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
   * Body frame → NWU algorithm frame transform
   * 
   * FlySight Body Frame:    NWU Algorithm Frame:
   *   X = West                 X = North
   *   Y = Up                   Y = West
   *   Z = North                Z = Up
   * 
   * Transform: NWU = [Body_Z, Body_X, Body_Y]
   */
  private bodyToNWU(bx: number, by: number, bz: number): FusionVector {
    return { 
      x: bz,  // NWU_X (North) = Body_Z (North)
      y: bx,  // NWU_Y (West)  = Body_X (West)
      z: by   // NWU_Z (Up)    = Body_Y (Up)
    };
  }
  
  /**
   * NWU algorithm frame → Body frame transform (inverse of bodyToNWU)
   * 
   * NWU Algorithm Frame:     FlySight Body Frame:
   *   X = North                 X = West
   *   Y = West                  Y = Up  
   *   Z = Up                    Z = North
   * 
   * Transform: Body = [NWU_Y, NWU_Z, NWU_X]
   */
  private nwuToBody(nx: number, ny: number, nz: number): FusionVector {
    return {
      x: ny,  // Body_X (West)  = NWU_Y (West)
      y: nz,  // Body_Y (Up)    = NWU_Z (Up)
      z: nx   // Body_Z (North) = NWU_X (North)
    };
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
    
    // Store accelerometer for compass heading calculation
    this.lastAccelNWU = accelNWU;
    
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
   * Get gravity direction in body frame
   * 
   * The AHRS returns gravity in NWU frame, we transform to body frame
   * for consistency with sensor data and viewer expectations.
   * 
   * Note: For visualization in world frame, the viewer can ignore this
   * and simply display [0,0,+1] in NWU world (gravity reaction = up).
   */
  getGravityVector(): { x: number; y: number; z: number } {
    const gNWU = this.ahrs.getGravity();
    return this.nwuToBody(gNWU.x, gNWU.y, gNWU.z);
  }
  
  /**
   * Get linear acceleration in body frame (gravity removed)
   * 
   * This is the acceleration the device is experiencing, expressed
   * in the device's coordinate system. Useful for local motion detection.
   */
  getLinearAcceleration(): { x: number; y: number; z: number } {
    const aNWU = this.ahrs.getLinearAcceleration();
    // Transform NWU → Body frame
    return this.nwuToBody(aNWU.x, aNWU.y, aNWU.z);
  }
  
  /**
   * Get Earth-frame acceleration in NWU convention (gravity removed)
   * 
   * This is the acceleration in the global/world frame:
   *   X = North, Y = West, Z = Up
   * 
   * Useful for trajectory estimation and dead reckoning.
   * Note: Returned in NWU, NOT body frame (it's a world-frame quantity).
   */
  getEarthAcceleration(): { x: number; y: number; z: number } {
    const a = this.ahrs.getEarthAcceleration();
    // Earth acceleration stays in NWU (world frame) - no transform needed
    return { x: a.x, y: a.y, z: a.z };
  }
  
  /**
   * Get gravity direction in body frame (alias for getGravityVector)
   */
  getGravity(): FusionVector {
    return this.getGravityVector();
  }
  /**
   * Get expected magnetic reference direction in body frame.
   * This is where the algorithm expects the magnetic field to point
   * based on the current orientation estimate.
   */
  getMagneticReference(): FusionVector {
    return this.ahrs.getMagneticReference();
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

  // ===========================================================================
  // Compass Heading Calculation (x-io FusionCompass Algorithm)
  // ===========================================================================

  /**
   * Calculate tilt-compensated magnetic heading using FusionCompass algorithm.
   * 
   * This implements the x-io Fusion library's FusionCompass function, which
   * computes heading directly from accelerometer and magnetometer without
   * relying on the quaternion. This is useful as an independent heading
   * measurement to verify AHRS behavior.
   * 
   * Algorithm (NWU convention):
   * 1. west = normalize(cross(accelerometer, magnetometer))
   * 2. north = normalize(cross(west, accelerometer))
   * 3. heading = atan2(west.x, north.x)
   * 
   * @param accelNWU Accelerometer in NWU frame (already calibrated and transformed)
   * @param magNWU Magnetometer in NWU frame (already calibrated and transformed)
   * @returns Heading in degrees (0-360)
   */
  private compassHeading(accelNWU: FusionVector, magNWU: FusionVector): number {
    // Normalize accelerometer
    const accelMag = Math.sqrt(accelNWU.x * accelNWU.x + accelNWU.y * accelNWU.y + accelNWU.z * accelNWU.z);
    if (accelMag < 0.01) return 0; // Safety check for near-zero acceleration
    
    const accelNorm = {
      x: accelNWU.x / accelMag,
      y: accelNWU.y / accelMag,
      z: accelNWU.z / accelMag
    };

    // west = cross(accel, mag)
    const west = {
      x: accelNorm.y * magNWU.z - accelNorm.z * magNWU.y,
      y: accelNorm.z * magNWU.x - accelNorm.x * magNWU.z,
      z: accelNorm.x * magNWU.y - accelNorm.y * magNWU.x
    };

    // Normalize west
    const westMag = Math.sqrt(west.x * west.x + west.y * west.y + west.z * west.z);
    if (westMag < 0.01) return 0; // Safety check
    
    const westNorm = {
      x: west.x / westMag,
      y: west.y / westMag,
      z: west.z / westMag
    };

    // north = cross(west, accel)
    const north = {
      x: westNorm.y * accelNorm.z - westNorm.z * accelNorm.y,
      y: westNorm.z * accelNorm.x - westNorm.x * accelNorm.z,
      z: westNorm.x * accelNorm.y - westNorm.y * accelNorm.x
    };

    // heading = atan2(west.x, north.x)
    let headingRad = Math.atan2(westNorm.x, north.x);
    
    // Convert to degrees and normalize to 0-360
    let headingDeg = headingRad * RAD_TO_DEG;
    if (headingDeg < 0) {
      headingDeg += 360;
    }
    
    // Debug - uncomment to trace algorithm
    console.log('compassHeading algorithm:', {
      accelNorm,
      magNWU,
      westNorm,
      north,
      headingRad,
      headingDeg
    });
    
    return headingDeg;
  }

  /**
   * Get compass heading as an independent check of magnetic heading.
   * 
   * This computes heading directly from the last accelerometer and magnetometer
   * readings without using the quaternion. Useful for diagnostics and comparing
   * against the AHRS-derived yaw angle.
   * 
   * @returns Heading in degrees (0-360), or 0 if sensors are invalid
   */
  getCompassHeading(): number {
    if (!this.magValid || Math.abs(this.lastAccelNWU.z + 1) < 0.01) {
      return 0; // No valid data
    }
    return this.compassHeading(this.lastAccelNWU, this.lastMagNWU);
  }

  /**
   * Compute compass heading from arbitrary accel and mag vectors.
   * 
   * This allows computing compass heading from frame data without needing
   * to update the AHRS state. Vectors should be in body frame.
   * 
   * IMPORTANT: This uses the tilt-compensated compass algorithm which only works
   * reliably when the device is roughly upright. For arbitrary orientations,
   * use getCompassHeadingFromMagQuaternion instead with the device quaternion.
   * 
   * @param accelBody Accelerometer in body frame (calibrated)
   * @param magBody Magnetometer in body frame (calibrated)
   * @returns Heading in degrees (0-360), or 0 if inputs are invalid
   */
  getCompassHeadingFromSensors(accelBody: {x: number, y: number, z: number}, magBody: {x: number, y: number, z: number}): number {
    // Transform to NWU for compass calculation
    const accelNWU = this.bodyToNWU(accelBody.x, accelBody.y, accelBody.z);
    const magNWU = this.bodyToNWU(magBody.x, magBody.y, magBody.z);
    
    const heading = this.compassHeading(accelNWU, magNWU);
    
    // Debug logging - uncomment to see transform chain
    console.log('Compass heading (tilt-compensated):', {
      accelBody,
      magBody,
      accelNWU,
      magNWU,
      headingDeg: heading
    });
    
    return heading;
  }

  /**
   * Compute compass heading using magnetometer rotated by device quaternion.
   * 
   * This is the correct way to calculate heading for arbitrary device orientations.
   * It rotates the magnetometer from body frame to world/NWU frame using the
   * device quaternion, then projects to horizontal plane and computes heading.
   * 
   * This matches how the green magnetometer heading vector is calculated.
   * 
   * @param magBody Magnetometer in body frame (calibrated)
   * @param quat Device quaternion (NWU convention)
   * @returns Heading in degrees (0-360), or 0 if inputs are invalid
   */
  getCompassHeadingFromMagQuaternion(magBody: {x: number, y: number, z: number}, quat: FusionQuaternion): number {
    // Transform mag from body to NWU frame
    const magNWU = this.bodyToNWU(magBody.x, magBody.y, magBody.z);
    
    // Rotate magnetometer using quaternion: mag_world = q * mag_nwu * q^(-1)
    // For unit quaternion, q^(-1) = conjugate(q) = (w, -x, -y, -z)
    
    // Quaternion multiplication: q * v is computed as (0, mag_rotated) = q * (0, mag) * q_inv
    // Using the formula: v' = v + 2*q_w*(q_xyz × v) + 2*(q_xyz × (q_xyz × v))
    
    const w = quat.w;
    const x = quat.x;
    const y = quat.y;
    const z = quat.z;
    
    const mx = magNWU.x;
    const my = magNWU.y;
    const mz = magNWU.z;
    
    // Cross product q_xyz × v
    const c1x = y * mz - z * my;
    const c1y = z * mx - x * mz;
    const c1z = x * my - y * mx;
    
    // Cross product q_xyz × (q_xyz × v)
    const c2x = y * c1z - z * c1y;
    const c2y = z * c1x - x * c1z;
    const c2z = x * c1y - y * c1x;
    
    // Full rotation: v' = v + 2*w*(q_xyz × v) + 2*(q_xyz × (q_xyz × v))
    const magWorldX = mx + 2 * w * c1x + 2 * c2x;
    const magWorldY = my + 2 * w * c1y + 2 * c2y;
    const magWorldZ = mz + 2 * w * c1z + 2 * c2z;
    
    // Project to horizontal plane (remove Z/Up component)
    const magHorizontalX = magWorldX;
    const magHorizontalY = magWorldY;
    
    const magHLen = Math.sqrt(magHorizontalX * magHorizontalX + magHorizontalY * magHorizontalY);
    if (magHLen < 0.01) return 0; // Too small to compute heading
    
    // Normalize
    const magHNormX = magHorizontalX / magHLen;
    const magHNormY = magHorizontalY / magHLen;
    
    // Compute heading from projected direction
    // In NWU: heading = atan2(east, north) = atan2(y, x)
    let headingRad = Math.atan2(magHNormY, magHNormX);
    let headingDeg = headingRad * RAD_TO_DEG;
    if (headingDeg < 0) {
      headingDeg += 360;
    }
    
    console.log('Compass heading (quaternion-based):', {
      magBody,
      quat,
      magNWU,
      magWorld: { x: magWorldX, y: magWorldY, z: magWorldZ },
      magHorizontal: { x: magHorizontalX, y: magHorizontalY },
      headingDeg
    });
    
    return headingDeg;
  }
}
