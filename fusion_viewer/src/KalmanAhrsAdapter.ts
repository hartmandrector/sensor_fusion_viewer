/**
 * KalmanAhrsAdapter - Wrapper to integrate KalmanFilter with existing viewer
 * 
 * Provides the same AHRSInterface as MadgwickAHRS and FusionAhrsAdapter
 * for drop-in replacement and comparison.
 */

import { KalmanFilter, DEFAULT_KALMAN_CONFIG } from './KalmanFilter';
import { DEFAULT_AXIS_REMAP, debug } from './constants';
import type {
  Quaternion,
  EulerAngles,
  MagCalibration,
  IMUCalibration,
  AxisRemap,
  FusionConfig
} from './types';

/**
 * Kalman AHRS Adapter - implements AHRSInterface
 */
export class KalmanAhrsAdapter {
  private kalman: KalmanFilter;
  private imuAxisRemap: AxisRemap = DEFAULT_AXIS_REMAP;
  private magAxisRemap: AxisRemap = DEFAULT_AXIS_REMAP;
  private magCalibration: MagCalibration = {
    offsetX: 0, offsetY: 0, offsetZ: 0,
    scaleX: 1, scaleY: 1, scaleZ: 1
  };
  private imuCalibration: IMUCalibration = {
    gyroBiasX: 0, gyroBiasY: 0, gyroBiasZ: 0,
    accelOffsetX: 0, accelOffsetY: 0, accelOffsetZ: 0
  };
  private softIronMatrix: number[][] | null = null;
  private accelScaleMatrix: number[][] | null = null;
  private lastCalibratedMag: { x: number; y: number; z: number; valid: boolean } = {
    x: 0, y: 0, z: 0, valid: false
  };
  private lastRemappedAccel: { x: number; y: number; z: number } = {
    x: 0, y: 0, z: 0
  };

  constructor(config: Partial<FusionConfig> = {}) {
    // Extract calibration from config
    const calibration = {
      imu: config.imuCalibration || {
        gyroBiasX: 0, gyroBiasY: 0, gyroBiasZ: 0,
        accelOffsetX: 0, accelOffsetY: 0, accelOffsetZ: 0
      },
      mag: config.magCalibration || {
        offsetX: 0, offsetY: 0, offsetZ: 0,
        scaleX: 1, scaleY: 1, scaleZ: 1
      }
    };
    
    this.kalman = new KalmanFilter(
      DEFAULT_KALMAN_CONFIG,
      calibration
    );
    
    if (config.imuAxisRemap) this.imuAxisRemap = config.imuAxisRemap;
    if (config.magAxisRemap) this.magAxisRemap = config.magAxisRemap;
    if (config.magCalibration) this.magCalibration = config.magCalibration;
    if (config.imuCalibration) this.imuCalibration = config.imuCalibration;
    
    debug.log('Kalman AHRS Adapter initialized');
  }

  /**
   * Apply axis remapping to sensor values
   * @param values Sensor values {x, y, z}
   * @param remap Axis remapping specification
   */
  private applyAxisRemapping(
    values: { x: number; y: number; z: number },
    remap: AxisRemap
  ): { x: number; y: number; z: number } {
    const sensorArray = [values.x, values.y, values.z];
    const bodyArray = [0, 0, 0];
    
    // Map each body axis from sensor axes
    for (let i = 0; i < 3; i++) {
      const spec = i === 0 ? remap.bodyX : i === 1 ? remap.bodyY : remap.bodyZ;
      const negate = spec.startsWith('-');
      const axisChar = spec.charAt(negate ? 1 : 0);
      const sensorAxis = axisChar === 'X' ? 0 : axisChar === 'Y' ? 1 : 2;
      
      bodyArray[i] = negate ? -sensorArray[sensorAxis] : sensorArray[sensorAxis];
    }
    
    return { x: bodyArray[0], y: bodyArray[1], z: bodyArray[2] };
  }

  /**
   * Reset to identity orientation
   */
  public reset(): void {
    this.kalman.reset();
  }

  /**
   * Update with magnetometer data
   */
  public updateMag(mx: number, my: number, mz: number): void {
    // Apply hard iron offset (subtract)
    let m_cal_x = mx - this.magCalibration.offsetX;
    let m_cal_y = my - this.magCalibration.offsetY;
    let m_cal_z = mz - this.magCalibration.offsetZ;
    
    // Apply soft iron matrix if available
    if (this.softIronMatrix) {
      const m_si_x = 
        this.softIronMatrix[0][0] * m_cal_x +
        this.softIronMatrix[0][1] * m_cal_y +
        this.softIronMatrix[0][2] * m_cal_z;
      const m_si_y =
        this.softIronMatrix[1][0] * m_cal_x +
        this.softIronMatrix[1][1] * m_cal_y +
        this.softIronMatrix[1][2] * m_cal_z;
      const m_si_z =
        this.softIronMatrix[2][0] * m_cal_x +
        this.softIronMatrix[2][1] * m_cal_y +
        this.softIronMatrix[2][2] * m_cal_z;
      m_cal_x = m_si_x;
      m_cal_y = m_si_y;
      m_cal_z = m_si_z;
    } else {
      // Fall back to simple scale factors
      m_cal_x *= this.magCalibration.scaleX;
      m_cal_y *= this.magCalibration.scaleY;
      m_cal_z *= this.magCalibration.scaleZ;
    }
    
    // Apply axis remapping
    const magValues = { x: m_cal_x, y: m_cal_y, z: m_cal_z };
    const magRemapped = this.applyAxisRemapping(magValues, this.magAxisRemap);
    
    // Store calibrated, remapped mag
    this.lastCalibratedMag = { x: magRemapped.x, y: magRemapped.y, z: magRemapped.z, valid: true };
  }

  /**
   * Main update: IMU + mag fusion
   */
  public updateIMU(
    dt: number,
    wx: number, wy: number, wz: number,  // rad/s
    ax: number, ay: number, az: number   // g
  ): void {
    // Apply gyro bias calibration
    let wx_cal = (wx - this.imuCalibration.gyroBiasX) * Math.PI / 180.0;
    let wy_cal = (wy - this.imuCalibration.gyroBiasY) * Math.PI / 180.0;
    let wz_cal = (wz - this.imuCalibration.gyroBiasZ) * Math.PI / 180.0;
    
    // Apply axis remapping to gyro
    const gyroValues = { x: wx_cal, y: wy_cal, z: wz_cal };
    const gyroRemapped = this.applyAxisRemapping(gyroValues, this.imuAxisRemap);
    const wx_remap = gyroRemapped.x;
    const wy_remap = gyroRemapped.y;
    const wz_remap = gyroRemapped.z;

    // Calibrate accelerometer (subtract offset)
    let ax_cal = ax - this.imuCalibration.accelOffsetX;
    let ay_cal = ay - this.imuCalibration.accelOffsetY;
    let az_cal = az - this.imuCalibration.accelOffsetZ;
    
    // Apply axis remapping to accel
    const accelValues = { x: ax_cal, y: ay_cal, z: az_cal };
    const accelRemapped = this.applyAxisRemapping(accelValues, this.imuAxisRemap);
    ax_cal = accelRemapped.x;
    ay_cal = accelRemapped.y;
    az_cal = accelRemapped.z;
    
    // Apply accel scale matrix if available
    let ax_scaled = ax_cal, ay_scaled = ay_cal, az_scaled = az_cal;
    if (this.accelScaleMatrix) {
      ax_scaled = 
        this.accelScaleMatrix[0][0] * ax_cal +
        this.accelScaleMatrix[0][1] * ay_cal +
        this.accelScaleMatrix[0][2] * az_cal;
      ay_scaled =
        this.accelScaleMatrix[1][0] * ax_cal +
        this.accelScaleMatrix[1][1] * ay_cal +
        this.accelScaleMatrix[1][2] * az_cal;
      az_scaled =
        this.accelScaleMatrix[2][0] * ax_cal +
        this.accelScaleMatrix[2][1] * ay_cal +
        this.accelScaleMatrix[2][2] * az_cal;
    }
    
    // Use last calibrated mag or zeros
    const mx = this.lastCalibratedMag.valid ? this.lastCalibratedMag.x : 0;
    const my = this.lastCalibratedMag.valid ? this.lastCalibratedMag.y : 0;
    const mz = this.lastCalibratedMag.valid ? this.lastCalibratedMag.z : 0;
    
    // Cache the remapped acceleration for getGravityVector()
    this.lastRemappedAccel = { x: ax_scaled, y: ay_scaled, z: az_scaled };
    
    // Update Kalman filter
    this.kalman.update(dt, wx_remap, wy_remap, wz_remap, ax_scaled, ay_scaled, az_scaled, mx, my, mz);
  }

  /**
   * Initialize from accel and mag
   */
  public initFromAccelMag(
    ax: number, ay: number, az: number,
    mx: number, my: number, mz: number
  ): void {
    // Apply same calibrations as in updateIMU/updateMag
    
    // Calibrate accelerometer
    let ax_cal = ax - this.imuCalibration.accelOffsetX;
    let ay_cal = ay - this.imuCalibration.accelOffsetY;
    let az_cal = az - this.imuCalibration.accelOffsetZ;
    
    // Apply accel scale matrix if available
    if (this.accelScaleMatrix) {
      const x_scaled = 
        this.accelScaleMatrix[0][0] * ax_cal +
        this.accelScaleMatrix[0][1] * ay_cal +
        this.accelScaleMatrix[0][2] * az_cal;
      const y_scaled =
        this.accelScaleMatrix[1][0] * ax_cal +
        this.accelScaleMatrix[1][1] * ay_cal +
        this.accelScaleMatrix[1][2] * az_cal;
      const z_scaled =
        this.accelScaleMatrix[2][0] * ax_cal +
        this.accelScaleMatrix[2][1] * ay_cal +
        this.accelScaleMatrix[2][2] * az_cal;
      ax_cal = x_scaled;
      ay_cal = y_scaled;
      az_cal = z_scaled;
    }
    
    // Apply axis remapping to accel
    const accelValues = { x: ax_cal, y: ay_cal, z: az_cal };
    const accelRemapped = this.applyAxisRemapping(accelValues, this.imuAxisRemap);
    
    // Calibrate magnetometer (hard iron + soft iron)
    let mx_cal = mx - this.magCalibration.offsetX;
    let my_cal = my - this.magCalibration.offsetY;
    let mz_cal = mz - this.magCalibration.offsetZ;
    
    // Apply soft iron matrix if available
    if (this.softIronMatrix) {
      const x_si = 
        this.softIronMatrix[0][0] * mx_cal +
        this.softIronMatrix[0][1] * my_cal +
        this.softIronMatrix[0][2] * mz_cal;
      const y_si =
        this.softIronMatrix[1][0] * mx_cal +
        this.softIronMatrix[1][1] * my_cal +
        this.softIronMatrix[1][2] * mz_cal;
      const z_si =
        this.softIronMatrix[2][0] * mx_cal +
        this.softIronMatrix[2][1] * my_cal +
        this.softIronMatrix[2][2] * mz_cal;
      mx_cal = x_si;
      my_cal = y_si;
      mz_cal = z_si;
    } else {
      // Fall back to simple scale factors
      mx_cal *= this.magCalibration.scaleX;
      my_cal *= this.magCalibration.scaleY;
      mz_cal *= this.magCalibration.scaleZ;
    }
    
    // Apply axis remapping to mag
    const magValues = { x: mx_cal, y: my_cal, z: mz_cal };
    const magRemapped = this.applyAxisRemapping(magValues, this.magAxisRemap);
    
    // Store for use in updateIMU
    this.lastCalibratedMag = { x: magRemapped.x, y: magRemapped.y, z: magRemapped.z, valid: true };
    
    // Initialize filter with calibrated, remapped data
    this.kalman.initFromAccelMag(accelRemapped.x, accelRemapped.y, accelRemapped.z, 
                                  magRemapped.x, magRemapped.y, magRemapped.z);
  }

  /**
   * Initialize from accel only
   */
  public initFromAccelOnly(ax: number, ay: number, az: number): void {
    // Apply same calibrations as in updateIMU
    
    // Calibrate accelerometer
    let ax_cal = ax - this.imuCalibration.accelOffsetX;
    let ay_cal = ay - this.imuCalibration.accelOffsetY;
    let az_cal = az - this.imuCalibration.accelOffsetZ;
    
    // Apply accel scale matrix if available
    if (this.accelScaleMatrix) {
      const x_scaled = 
        this.accelScaleMatrix[0][0] * ax_cal +
        this.accelScaleMatrix[0][1] * ay_cal +
        this.accelScaleMatrix[0][2] * az_cal;
      const y_scaled =
        this.accelScaleMatrix[1][0] * ax_cal +
        this.accelScaleMatrix[1][1] * ay_cal +
        this.accelScaleMatrix[1][2] * az_cal;
      const z_scaled =
        this.accelScaleMatrix[2][0] * ax_cal +
        this.accelScaleMatrix[2][1] * ay_cal +
        this.accelScaleMatrix[2][2] * az_cal;
      ax_cal = x_scaled;
      ay_cal = y_scaled;
      az_cal = z_scaled;
    }
    
    // Apply axis remapping to accel
    const accelValues = { x: ax_cal, y: ay_cal, z: az_cal };
    const accelRemapped = this.applyAxisRemapping(accelValues, this.imuAxisRemap);
    
    this.kalman.initFromAccelOnly(accelRemapped.x, accelRemapped.y, accelRemapped.z);
  }

  /**
   * Set IMU calibration
   */
  public setIMUCalibration(cal: IMUCalibration): void {
    this.imuCalibration = { ...cal };
    this.kalman.setIMUCalibration(cal);
  }

  /**
   * Set magnetometer calibration
   */
  public setMagCalibration(cal: MagCalibration): void {
    this.magCalibration = { ...cal };
    this.kalman.setMagCalibration(cal);
  }

  /**
   * Get IMU calibration
   */
  public getIMUCalibration(): IMUCalibration {
    return { ...this.imuCalibration };
  }

  /**
   * Apply axis remapping to IMU data (for display purposes)
   */
  public applyIMURemap(x: number, y: number, z: number): { x: number; y: number; z: number } {
    const values = { x, y, z };
    return this.applyAxisRemapping(values, this.imuAxisRemap);
  }

  /**
   * Get magnetometer calibration
   */
  public getMagCalibration(): MagCalibration {
    return { ...this.magCalibration };
  }

  /**
   * Set gyro bias
   */
  public setGyroBias(x: number, y: number, z: number): void {
    this.imuCalibration = {
      ...this.imuCalibration,
      gyroBiasX: x,
      gyroBiasY: y,
      gyroBiasZ: z
    };
    this.kalman.setIMUCalibration(this.imuCalibration);
  }

  /**
   * Set accel offset
   */
  public setAccelOffset(x: number, y: number, z: number): void {
    this.imuCalibration = {
      ...this.imuCalibration,
      accelOffsetX: x,
      accelOffsetY: y,
      accelOffsetZ: z
    };
    this.kalman.setIMUCalibration(this.imuCalibration);
  }

  /**
   * Set IMU axis remap
   */
  public setIMUAxisRemap?(remap: AxisRemap): void {
    this.imuAxisRemap = remap;
  }

  /**
   * Set mag axis remap
   */
  public setMagAxisRemap?(remap: AxisRemap): void {
    this.magAxisRemap = remap;
  }

  /**
   * Set soft iron matrix
   */
  public setSoftIronMatrix?(matrix: number[][] | null): void {
    this.softIronMatrix = matrix;
  }

  /**
   * Set accel scale matrix
   */
  public setAccelScaleMatrix?(matrix: number[][]): void {
    this.accelScaleMatrix = matrix;
  }

  /**
   * Get output quaternion
   */
  public getOutput(): { quaternion: Quaternion; euler: EulerAngles; heading: number } {
    const quaternion = this.kalman.getQuaternion();
    const euler = this.kalman.getEulerAngles();
    const heading = this.kalman.getHeading();
    
    return { quaternion, euler, heading };
  }

  /**
   * Get calibrated magnetometer
   */
  public getCalibratedMag(): { x: number; y: number; z: number; valid: boolean } {
    return { ...this.lastCalibratedMag };
  }

  /**
   * Get gravity vector (body frame acceleration rotated to world frame)
   * Returns the raw acceleration vector from IMU, rotated to world frame
   */
  public getGravityVector(): { x: number; y: number; z: number } {
    const q = this.kalman.getQuaternion();
    const accel_body = this.lastRemappedAccel;
    
    // Rotate body-frame acceleration to world frame: v_world = q ⊗ v_body ⊗ q*
    const p = { w: 0, x: accel_body.x, y: accel_body.y, z: accel_body.z };
    const q_conj = { w: q.w, x: -q.x, y: -q.y, z: -q.z };
    
    // First quaternion product: q ⊗ p
    const temp_w = q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z;
    const temp_x = q.w * p.x + q.x * p.w + q.y * p.z - q.z * p.y;
    const temp_y = q.w * p.y - q.x * p.z + q.y * p.w + q.z * p.x;
    const temp_z = q.w * p.z + q.x * p.y - q.y * p.x + q.z * p.w;
    
    // Second quaternion product: (q ⊗ p) ⊗ q*
    const result_x = temp_w * q_conj.x + temp_x * q_conj.w + temp_y * q_conj.z - temp_z * q_conj.y;
    const result_y = temp_w * q_conj.y - temp_x * q_conj.z + temp_y * q_conj.w + temp_z * q_conj.x;
    const result_z = temp_w * q_conj.z + temp_x * q_conj.y - temp_y * q_conj.x + temp_z * q_conj.w;
    
    return { x: result_x, y: result_y, z: result_z };
  }

  /**
   * Get linear acceleration (body frame, gravity removed)
   */
  public getLinearAcceleration(): { x: number; y: number; z: number } {
    // This would require storing the last accel measurement
    // For now, return zero
    return { x: 0, y: 0, z: 0 };
  }

  /**
   * Get earth acceleration (world frame)
   */
  public getEarthAcceleration(): { x: number; y: number; z: number } {
    return { x: 0, y: 0, z: 0 };
  }

  /**
   * Get compass heading
   */
  public getCompassHeading(): number {
    return this.kalman.getHeading();
  }

  /**
   * Get compass heading from sensors (alternative calculation)
   */
  public getCompassHeadingFromSensors?(): number {
    // Use current quaternion output
    return this.kalman.getHeading();
  }

  /**
   * Get compass heading from mag quaternion
   */
  public getCompassHeadingFromMagQuaternion?(): number {
    return this.kalman.getHeading();
  }
}
