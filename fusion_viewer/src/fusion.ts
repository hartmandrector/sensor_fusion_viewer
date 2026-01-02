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

// Constants
const DEG_TO_RAD = Math.PI / 180.0;
const RAD_TO_DEG = 180.0 / Math.PI;

/**
 * Quaternion representation [w, x, y, z]
 */
export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

/**
 * Euler angles in radians
 */
export interface EulerAngles {
  roll: number;   // Rotation around X axis
  pitch: number;  // Rotation around Y axis
  yaw: number;    // Rotation around Z axis (heading)
}

/**
 * Fusion output structure
 */
export interface FusionOutput {
  quaternion: Quaternion;
  euler: EulerAngles;
  heading: number;  // Magnetic heading in degrees (0-360)
}

/**
 * Magnetometer calibration parameters
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
 * IMU (Gyro + Accel) calibration parameters
 */
export interface IMUCalibration {
  // Gyroscope bias (deg/s) - subtracted from raw readings
  gyroBiasX: number;
  gyroBiasY: number;
  gyroBiasZ: number;
  // Accelerometer offset (g) - subtracted from raw readings
  accelOffsetX: number;
  accelOffsetY: number;
  accelOffsetZ: number;
}

/**
 * Fusion configuration
 */
export interface FusionConfig {
  beta: number;              // Madgwick filter gain (0.01 - 0.5)
  magCalibration: MagCalibration;
  imuCalibration?: IMUCalibration;  // Optional IMU calibration
  applyMagTransform: boolean;  // Apply X,Z axis inversion for LIS2MDL on back of PCB
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
  private applyMagTransform: boolean;
  
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
    this.applyMagTransform = config?.applyMagTransform ?? true;
    this.magCal = config?.magCalibration ?? {
      offsetX: 0, offsetY: 0, offsetZ: 0,
      scaleX: 1, scaleY: 1, scaleZ: 1
    };
    this.imuCal = config?.imuCalibration ?? {
      gyroBiasX: 0, gyroBiasY: 0, gyroBiasZ: 0,
      accelOffsetX: 0, accelOffsetY: 0, accelOffsetZ: 0
    };
  }
  
  /**
   * Reset filter to initial state
   */
  reset(): void {
    this.q = { w: 1, x: 0, y: 0, z: 0 };
    this.magValid = false;
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
   * Set whether to apply magnetometer coordinate transform
   */
  setApplyMagTransform(apply: boolean): void {
    this.applyMagTransform = apply;
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
    // CRITICAL: Apply coordinate transform for LIS2MDL on back of PCB
    // The magnetometer axes are mirrored relative to the IMU
    if (this.applyMagTransform) {
      mx = -mx;  // X axis inverted
      // my stays same
      mz = -mz;  // Z axis inverted
    }
    
    // Apply hard iron calibration (offset removal)
    mx = (mx - this.magCal.offsetX) * this.magCal.scaleX;
    my = (my - this.magCal.offsetY) * this.magCal.scaleY;
    mz = (mz - this.magCal.offsetZ) * this.magCal.scaleZ;
    
    // Store for use in IMU update
    this.lastMagX = mx;
    this.lastMagY = my;
    this.lastMagZ = mz;
    this.magValid = true;
  }
  
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
    
    // Convert gyro from deg/s to rad/s
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;
    
    if (this.magValid) {
      this.madgwickAHRSupdate(dt, gx, gy, gz, ax, ay, az,
                             this.lastMagX, this.lastMagY, this.lastMagZ);
    } else {
      this.madgwickAHRSupdateIMU(dt, gx, gy, gz, ax, ay, az);
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
   */
  getQuaternion(): Quaternion {
    return { ...this.q };
  }
  
  /**
   * Convert quaternion to Euler angles (roll, pitch, yaw)
   * Uses aerospace convention (ZYX rotation order)
   */
  getEulerAngles(): EulerAngles {
    const q0 = this.q.w, q1 = this.q.x, q2 = this.q.y, q3 = this.q.z;
    
    // Roll (rotation around X axis)
    const sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
    const cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (rotation around Y axis)
    const sinp = 2.0 * (q0 * q2 - q3 * q1);
    let pitch: number;
    if (Math.abs(sinp) >= 1) {
      pitch = Math.sign(sinp) * Math.PI / 2; // Use 90 degrees if out of range
    } else {
      pitch = Math.asin(sinp);
    }
    
    // Yaw (rotation around Z axis)
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
}

// Export utility functions
export { DEG_TO_RAD, RAD_TO_DEG };
