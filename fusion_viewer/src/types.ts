/**
 * Centralized type definitions for the Sensor Fusion Viewer
 */

// ============================================================================
// Quaternion and Orientation Types
// ============================================================================

/**
 * Unit quaternion representing orientation
 * Convention: q = w + xi + yj + zk
 */
export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

/**
 * Euler angles in radians
 * Convention: ZYX (yaw-pitch-roll)
 */
export interface EulerAngles {
  roll: number;   // Rotation around X axis (North in NWU)
  pitch: number;  // Rotation around Y axis (West in NWU)
  yaw: number;    // Rotation around Z axis (Up in NWU)
}

// ============================================================================
// Axis Remapping Types
// ============================================================================

/**
 * Axis specifier with optional sign inversion
 */
export type AxisSpec = '+X' | '-X' | '+Y' | '-Y' | '+Z' | '-Z';

/**
 * Axis remapping configuration
 * Maps body frame axes to sensor physical axes
 */
export interface AxisRemap {
  bodyX: AxisSpec;  // Which sensor axis corresponds to body X
  bodyY: AxisSpec;  // Which sensor axis corresponds to body Y
  bodyZ: AxisSpec;  // Which sensor axis corresponds to body Z
}

// ============================================================================
// Calibration Types
// ============================================================================

/**
 * Magnetometer calibration parameters (hard/soft iron correction)
 */
export interface MagCalibration {
  offsetX: number;
  offsetY: number;
  offsetZ: number;
  scaleX: number;
  scaleY: number;
  scaleZ: number;
}

/**
 * IMU calibration parameters (gyro bias and accel offset)
 */
export interface IMUCalibration {
  gyroBiasX: number;
  gyroBiasY: number;
  gyroBiasZ: number;
  accelOffsetX: number;
  accelOffsetY: number;
  accelOffsetZ: number;
}

/**
 * Complete AHRS configuration
 */
export interface FusionConfig {
  beta?: number;
  magCalibration?: MagCalibration;
  imuCalibration?: IMUCalibration;
  imuAxisRemap?: AxisRemap;
  magAxisRemap?: AxisRemap;
}

// ============================================================================
// Sensor Data Types
// ============================================================================

/**
 * Raw CSV row data from FlySight 2
 */
export interface CsvRow {
  time: number;      // Timestamp in seconds
  wx: number;        // Gyro X (rad/s)
  wy: number;        // Gyro Y (rad/s)
  wz: number;        // Gyro Z (rad/s)
  ax: number;        // Accel X (g)
  ay: number;        // Accel Y (g)
  az: number;        // Accel Z (g)
  mx: number;        // Mag X (raw)
  my: number;        // Mag Y (raw)
  mz: number;        // Mag Z (raw)
}

/**
 * Processed sensor data with calibration applied
 */
export interface SensorData {
  time: number;
  gyroX: number;
  gyroY: number;
  gyroZ: number;
  accelX: number;
  accelY: number;
  accelZ: number;
  magX: number;
  magY: number;
  magZ: number;
}

// ============================================================================
// Visualization Types
// ============================================================================

/**
 * 3D vector for visualization
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Sensor vectors for display
 */
export interface SensorVectors {
  accel: Vector3;
  mag: Vector3;
  gyro: Vector3;
}
