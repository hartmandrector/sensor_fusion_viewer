/**
 * FusionAhrs - Chapter 7 AHRS Implementation
 * 
 * Port of x-io Fusion library's improved AHRS algorithm with:
 * - Acceleration rejection for high-G environments
 * - Magnetic distortion rejection
 * - Gyroscope bias estimation
 * - Linear and Earth-frame acceleration outputs
 * 
 * Based on: Madgwick, S. O. H. (2020). "An efficient orientation filter for 
 * inertial and inertial/magnetic sensor arrays" - Chapter 7
 * 
 * Coordinate Convention: NWU (North-West-Up)
 * - X: North
 * - Y: West  
 * - Z: Up
 * 
 * @license MIT
 */

// ============================================================================
// Types
// ============================================================================

export interface FusionVector {
  x: number;
  y: number;
  z: number;
}

export interface FusionQuaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

export interface FusionEuler {
  roll: number;   // radians
  pitch: number;  // radians
  yaw: number;    // radians
}

/**
 * Algorithm settings - all thresholds in degrees
 */
export interface FusionAhrsSettings {
  /** Filter gain (0.5 recommended, higher = faster convergence, more noise) */
  gain: number;
  
  /** Gyroscope measurement range in deg/s (0 = unlimited, used for saturation detection) */
  gyroscopeRange: number;
  
  /** Acceleration rejection threshold in degrees (0 = disabled, 10° recommended) */
  accelerationRejection: number;
  
  /** Magnetic rejection threshold in degrees (0 = disabled, 10° recommended) */
  magneticRejection: number;
  
  /** Recovery trigger period in samples (0 = disabled) */
  recoveryTriggerPeriod: number;
}

/**
 * Internal algorithm states for diagnostics
 */
export interface FusionAhrsInternalStates {
  accelerationError: number;      // degrees
  accelerometerIgnored: boolean;
  accelerationRecoveryTrigger: number;  // 0.0 to 1.0
  magneticError: number;          // degrees
  magnetometerIgnored: boolean;
  magneticRecoveryTrigger: number;      // 0.0 to 1.0
}

/**
 * Status flags
 */
export interface FusionAhrsFlags {
  initialising: boolean;
  angularRateRecovery: boolean;
  accelerationRecovery: boolean;
  magneticRecovery: boolean;
}

// ============================================================================
// Constants
// ============================================================================

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

/** Initial gain during startup (high for fast convergence) */
const INITIAL_GAIN = 10.0;

/** Time to ramp from initial gain to target gain */
const INITIALISATION_PERIOD = 3.0;

/** Default settings */
export const FUSION_AHRS_DEFAULT_SETTINGS: FusionAhrsSettings = {
  gain: 0.5,
  gyroscopeRange: 0,              // Disabled
  accelerationRejection: 90,      // Disabled (90° = accept everything)
  magneticRejection: 90,          // Disabled
  recoveryTriggerPeriod: 0        // Disabled
};

// ============================================================================
// Vector Math Helpers
// ============================================================================

function vectorZero(): FusionVector {
  return { x: 0, y: 0, z: 0 };
}

function vectorIsZero(v: FusionVector): boolean {
  return v.x === 0 && v.y === 0 && v.z === 0;
}

function vectorAdd(a: FusionVector, b: FusionVector): FusionVector {
  return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

function vectorScale(v: FusionVector, s: number): FusionVector {
  return { x: v.x * s, y: v.y * s, z: v.z * s };
}

function vectorDot(a: FusionVector, b: FusionVector): number {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function vectorCross(a: FusionVector, b: FusionVector): FusionVector {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x
  };
}

function vectorMagnitude(v: FusionVector): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

function vectorMagnitudeSquared(v: FusionVector): number {
  return v.x * v.x + v.y * v.y + v.z * v.z;
}

function vectorNormalize(v: FusionVector): FusionVector {
  const mag = vectorMagnitude(v);
  if (mag === 0) return vectorZero();
  return vectorScale(v, 1 / mag);
}

// ============================================================================
// Quaternion Math Helpers
// ============================================================================

function quaternionIdentity(): FusionQuaternion {
  return { w: 1, x: 0, y: 0, z: 0 };
}

function quaternionNormalize(q: FusionQuaternion): FusionQuaternion {
  const mag = Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (mag === 0) return quaternionIdentity();
  return { w: q.w / mag, x: q.x / mag, y: q.y / mag, z: q.z / mag };
}

/**
 * Multiply quaternion by vector (returns quaternion)
 * Used for: q' = q + 0.5 * q ⊗ [0, ω] * dt
 */
function quaternionMultiplyVector(q: FusionQuaternion, v: FusionVector): FusionQuaternion {
  return {
    w: -q.x * v.x - q.y * v.y - q.z * v.z,
    x:  q.w * v.x + q.y * v.z - q.z * v.y,
    y:  q.w * v.y + q.z * v.x - q.x * v.z,
    z:  q.w * v.z + q.x * v.y - q.y * v.x
  };
}

/**
 * Add two quaternions
 */
function quaternionAdd(a: FusionQuaternion, b: FusionQuaternion): FusionQuaternion {
  return { w: a.w + b.w, x: a.x + b.x, y: a.y + b.y, z: a.z + b.z };
}

/**
 * Scale quaternion
 */
function quaternionScale(q: FusionQuaternion, s: number): FusionQuaternion {
  return { w: q.w * s, x: q.x * s, y: q.y * s, z: q.z * s };
}

// ============================================================================
// FusionAhrs Class
// ============================================================================

export class FusionAhrs {
  // Settings (with transformed thresholds)
  private settings: FusionAhrsSettings;
  private accelerationRejectionSquared: number;
  private magneticRejectionSquared: number;
  private gyroscopeRangeThreshold: number;
  
  // Core state
  private quaternion: FusionQuaternion;
  private accelerometer: FusionVector;  // Stored for output calculations
  
  // Initialization state
  private initialising: boolean;
  private rampedGain: number;
  private rampedGainStep: number;
  
  // Angular rate recovery
  private angularRateRecovery: boolean;
  
  // Feedback vectors (scaled by 0.5)
  private halfAccelerometerFeedback: FusionVector;
  private halfMagnetometerFeedback: FusionVector;
  
  // Acceleration rejection state
  private accelerometerIgnored: boolean;
  private accelerationRecoveryTrigger: number;
  private accelerationRecoveryTimeout: number;
  
  // Magnetic rejection state
  private magnetometerIgnored: boolean;
  private magneticRecoveryTrigger: number;
  private magneticRecoveryTimeout: number;
  
  constructor(settings?: Partial<FusionAhrsSettings>) {
    this.settings = { ...FUSION_AHRS_DEFAULT_SETTINGS, ...settings };
    this.accelerationRejectionSquared = 0;
    this.magneticRejectionSquared = 0;
    this.gyroscopeRangeThreshold = 0;
    
    this.quaternion = quaternionIdentity();
    this.accelerometer = vectorZero();
    
    this.initialising = false;
    this.rampedGain = 0;
    this.rampedGainStep = 0;
    
    this.angularRateRecovery = false;
    
    this.halfAccelerometerFeedback = vectorZero();
    this.halfMagnetometerFeedback = vectorZero();
    
    this.accelerometerIgnored = false;
    this.accelerationRecoveryTrigger = 0;
    this.accelerationRecoveryTimeout = 0;
    
    this.magnetometerIgnored = false;
    this.magneticRecoveryTrigger = 0;
    this.magneticRecoveryTimeout = 0;
    
    this.applySettings(this.settings);
    this.reset();
  }
  
  /**
   * Apply settings and compute derived thresholds
   */
  applySettings(settings: Partial<FusionAhrsSettings>): void {
    this.settings = { ...this.settings, ...settings };
    
    // Transform gyroscope range to 98% for early saturation detection
    this.gyroscopeRangeThreshold = this.settings.gyroscopeRange === 0 
      ? Infinity 
      : 0.98 * this.settings.gyroscopeRange;
    
    // Transform rejection thresholds from degrees to squared half-sine
    // threshold = (0.5 * sin(angle))²
    this.accelerationRejectionSquared = this.settings.accelerationRejection === 0
      ? Infinity
      : Math.pow(0.5 * Math.sin(this.settings.accelerationRejection * DEG_TO_RAD), 2);
    
    this.magneticRejectionSquared = this.settings.magneticRejection === 0
      ? Infinity
      : Math.pow(0.5 * Math.sin(this.settings.magneticRejection * DEG_TO_RAD), 2);
    
    // Reset recovery timeouts
    this.accelerationRecoveryTimeout = this.settings.recoveryTriggerPeriod;
    this.magneticRecoveryTimeout = this.settings.recoveryTriggerPeriod;
    
    // Disable rejection features if gain is zero or recovery trigger period is zero
    if (this.settings.gain === 0 || this.settings.recoveryTriggerPeriod === 0) {
      this.accelerationRejectionSquared = Infinity;
      this.magneticRejectionSquared = Infinity;
    }
    
    // Update gain step (may be called mid-session)
    if (!this.initialising) {
      this.rampedGain = this.settings.gain;
    }
    this.rampedGainStep = (INITIAL_GAIN - this.settings.gain) / INITIALISATION_PERIOD;
  }
  
  /**
   * Reset algorithm to initial state
   */
  reset(): void {
    this.quaternion = quaternionIdentity();
    this.accelerometer = vectorZero();
    this.initialising = true;
    this.rampedGain = INITIAL_GAIN;
    this.rampedGainStep = (INITIAL_GAIN - this.settings.gain) / INITIALISATION_PERIOD;
    this.angularRateRecovery = false;
    this.halfAccelerometerFeedback = vectorZero();
    this.halfMagnetometerFeedback = vectorZero();
    this.accelerometerIgnored = false;
    this.accelerationRecoveryTrigger = 0;
    this.accelerationRecoveryTimeout = this.settings.recoveryTriggerPeriod;
    this.magnetometerIgnored = false;
    this.magneticRecoveryTrigger = 0;
    this.magneticRecoveryTimeout = this.settings.recoveryTriggerPeriod;
  }
  
  /**
   * Main update with gyroscope, accelerometer, and magnetometer
   * 
   * @param gyroscope Angular rate in rad/s (body frame, after bias correction)
   * @param accelerometer Acceleration in g (body frame)
   * @param magnetometer Magnetic field (body frame, any units - will be normalized)
   * @param deltaTime Time step in seconds
   */
  update(
    gyroscope: FusionVector,
    accelerometer: FusionVector,
    magnetometer: FusionVector,
    deltaTime: number
  ): void {
    // Store accelerometer for output functions
    this.accelerometer = accelerometer;
    
    // Check for angular rate recovery (gyro saturation)
    // Compare each axis individually in deg/s (gyroscope input is rad/s)
    const gyroDegX = Math.abs(gyroscope.x) * RAD_TO_DEG;
    const gyroDegY = Math.abs(gyroscope.y) * RAD_TO_DEG;
    const gyroDegZ = Math.abs(gyroscope.z) * RAD_TO_DEG;
    if (gyroDegX > this.gyroscopeRangeThreshold ||
        gyroDegY > this.gyroscopeRangeThreshold ||
        gyroDegZ > this.gyroscopeRangeThreshold) {
      const savedQuaternion = { ...this.quaternion };
      this.reset();
      this.quaternion = savedQuaternion;
      this.angularRateRecovery = true;
    }
    
    // Ramp gain during initialization
    this.updateGainRamp(deltaTime);
    
    // Calculate half gravity from current orientation (NWU: gravity is [0, 0, -1])
    const halfGravity = this.getHalfGravity();
    
    // === ACCELEROMETER FEEDBACK ===
    let halfAccelerometerFeedback = vectorZero();
    this.accelerometerIgnored = true;  // Default to ignored
    
    if (!vectorIsZero(accelerometer)) {
      // Compute feedback: cross(normalize(accel), halfGravity)
      const accelNorm = vectorNormalize(accelerometer);
      this.halfAccelerometerFeedback = this.computeFeedback(accelNorm, halfGravity);
      
      // Acceleration rejection - sets accelerometerIgnored
      this.processAccelerationRejection();
      
      // Apply feedback if not ignored
      if (!this.accelerometerIgnored) {
        halfAccelerometerFeedback = this.halfAccelerometerFeedback;
      }
    }
    
    // === MAGNETOMETER FEEDBACK ===
    let halfMagnetometerFeedback = vectorZero();
    this.magnetometerIgnored = true;  // Default to ignored
    
    // Calculate half magnetic reference from current orientation
    const halfMagnetic = this.getHalfMagnetic();
    
    if (!vectorIsZero(magnetometer)) {
      // Extract horizontal component of magnetometer using gravity
      // horizontalMag = normalize(cross(halfGravity, magnetometer))
      const magCrossGrav = vectorCross(halfGravity, magnetometer);
      
      if (!vectorIsZero(magCrossGrav)) {
        const horizontalMag = vectorNormalize(magCrossGrav);
        this.halfMagnetometerFeedback = this.computeFeedback(horizontalMag, halfMagnetic);
        
        // Magnetic rejection - sets magnetometerIgnored
        this.processMagneticRejection();
        
        // Apply feedback if not ignored
        if (!this.magnetometerIgnored) {
          halfMagnetometerFeedback = this.halfMagnetometerFeedback;
        }
      }
    }
    
    // === INTEGRATE ===
    // Combine gyroscope with feedback
    const halfGyroscope = vectorScale(gyroscope, 0.5);
    const feedback = vectorAdd(halfAccelerometerFeedback, halfMagnetometerFeedback);
    const adjustedHalfGyro = vectorAdd(halfGyroscope, vectorScale(feedback, this.rampedGain));
    
    // Quaternion integration: q = q + q̇ * dt
    const qDot = quaternionMultiplyVector(this.quaternion, adjustedHalfGyro);
    this.quaternion = quaternionAdd(this.quaternion, quaternionScale(qDot, deltaTime));
    this.quaternion = quaternionNormalize(this.quaternion);
  }
  
  /**
   * Update without magnetometer (6-DOF)
   */
  updateNoMagnetometer(
    gyroscope: FusionVector,
    accelerometer: FusionVector,
    deltaTime: number
  ): void {
    this.update(gyroscope, accelerometer, vectorZero(), deltaTime);
  }
  
  // ===========================================================================
  // Private Methods
  // ===========================================================================
  
  /**
   * Update gain ramp during initialization
   */
  private updateGainRamp(deltaTime: number): void {
    if (this.initialising) {
      this.rampedGain -= this.rampedGainStep * deltaTime;
      if (this.rampedGain < this.settings.gain || this.settings.gain === 0) {
        this.rampedGain = this.settings.gain;
        this.initialising = false;
        this.angularRateRecovery = false;
      }
    }
  }
  
  /**
   * Compute feedback vector using cross product
   * If vectors are > 90° apart, normalize the result
   */
  private computeFeedback(sensor: FusionVector, reference: FusionVector): FusionVector {
    if (vectorDot(sensor, reference) < 0) {
      // Error > 90°, normalize to prevent instability
      return vectorNormalize(vectorCross(sensor, reference));
    }
    return vectorCross(sensor, reference);
  }
  
  /**
   * Get half gravity vector from quaternion (NWU: [0, 0, -0.5])
   */
  private getHalfGravity(): FusionVector {
    const q = this.quaternion;
    return {
      x: q.x * q.z - q.w * q.y,
      y: q.w * q.x + q.y * q.z,
      z: q.w * q.w - 0.5 + q.z * q.z
    };
  }
  
  /**
   * Get half magnetic reference from quaternion (NWU: pointing West at [0, 0.5, 0])
   * This is the second column of the transposed rotation matrix scaled by 0.5
   */
  private getHalfMagnetic(): FusionVector {
    const q = this.quaternion;
    return {
      x: q.x * q.y + q.w * q.z,
      y: q.w * q.w - 0.5 + q.y * q.y,
      z: q.y * q.z - q.w * q.x
    };
  }
  
  /**
   * Process acceleration rejection logic
   */
  private processAccelerationRejection(): void {
    const errorSquared = vectorMagnitudeSquared(this.halfAccelerometerFeedback);
    
    if (this.initialising || errorSquared <= this.accelerationRejectionSquared) {
      this.accelerometerIgnored = false;
      this.accelerationRecoveryTrigger -= 9;  // Decrement by 9 when accepted
    } else {
      this.accelerometerIgnored = true;
      this.accelerationRecoveryTrigger += 1;  // Increment by 1 when rejected
    }
    
    // Recovery trigger check
    if (this.accelerationRecoveryTrigger > this.accelerationRecoveryTimeout) {
      this.accelerationRecoveryTimeout = 0;
      this.accelerometerIgnored = false;  // Force acceptance
    } else {
      this.accelerationRecoveryTimeout = this.settings.recoveryTriggerPeriod;
    }
    
    // Clamp trigger value
    this.accelerationRecoveryTrigger = Math.max(0, 
      Math.min(this.accelerationRecoveryTrigger, this.settings.recoveryTriggerPeriod));
  }
  
  /**
   * Process magnetic rejection logic
   */
  private processMagneticRejection(): void {
    const errorSquared = vectorMagnitudeSquared(this.halfMagnetometerFeedback);
    
    if (this.initialising || errorSquared <= this.magneticRejectionSquared) {
      this.magnetometerIgnored = false;
      this.magneticRecoveryTrigger -= 9;
    } else {
      this.magnetometerIgnored = true;
      this.magneticRecoveryTrigger += 1;
    }
    
    if (this.magneticRecoveryTrigger > this.magneticRecoveryTimeout) {
      this.magneticRecoveryTimeout = 0;
      this.magnetometerIgnored = false;
    } else {
      this.magneticRecoveryTimeout = this.settings.recoveryTriggerPeriod;
    }
    
    this.magneticRecoveryTrigger = Math.max(0,
      Math.min(this.magneticRecoveryTrigger, this.settings.recoveryTriggerPeriod));
  }
  
  // ===========================================================================
  // Output Methods
  // ===========================================================================
  
  /**
   * Get current orientation quaternion
   */
  getQuaternion(): FusionQuaternion {
    return { ...this.quaternion };
  }
  
  /**
   * Set orientation quaternion directly
   */
  setQuaternion(q: FusionQuaternion): void {
    this.quaternion = quaternionNormalize(q);
  }
  
  /**
   * Get Euler angles (roll, pitch, yaw) in radians
   */
  getEulerAngles(): FusionEuler {
    const q = this.quaternion;
    
    // Roll (x-axis rotation)
    const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    const sinp = 2 * (q.w * q.y - q.z * q.x);
    let pitch: number;
    if (Math.abs(sinp) >= 1) {
      pitch = Math.sign(sinp) * Math.PI / 2;  // Gimbal lock
    } else {
      pitch = Math.asin(sinp);
    }
    
    // Yaw (z-axis rotation)
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);
    
    return { roll, pitch, yaw };
  }
  
  /**
   * Get gravity direction in sensor frame (normalized)
   * NWU: When level, gravity points [0, 0, -1]
   */
  getGravity(): FusionVector {
    const q = this.quaternion;
    return {
      x: 2 * (q.x * q.z - q.w * q.y),
      y: 2 * (q.w * q.x + q.y * q.z),
      z: 2 * (q.w * q.w - 0.5 + q.z * q.z)
    };
  }
  
  /**
   * Get linear acceleration in sensor frame (accelerometer minus gravity)
   * This is the "zero-g" acceleration - motion acceleration only
   */
  getLinearAcceleration(): FusionVector {
    // NWU: Subtract gravity (which points down, so add [0,0,1] to remove it)
    const gravity = this.getGravity();
    return {
      x: this.accelerometer.x - gravity.x,
      y: this.accelerometer.y - gravity.y,
      z: this.accelerometer.z - gravity.z
    };
  }
  
  /**
   * Get acceleration in Earth frame (global frame, gravity removed)
   * Useful for: trajectory estimation, activity detection
   */
  getEarthAcceleration(): FusionVector {
    const q = this.quaternion;
    const a = this.accelerometer;
    
    // Rotate accelerometer to Earth frame: R * a
    // Using quaternion rotation: q * [0,a] * q'
    const ax = 2 * ((0.5 - q.y * q.y - q.z * q.z) * a.x + (q.x * q.y - q.w * q.z) * a.y + (q.x * q.z + q.w * q.y) * a.z);
    const ay = 2 * ((q.x * q.y + q.w * q.z) * a.x + (0.5 - q.x * q.x - q.z * q.z) * a.y + (q.y * q.z - q.w * q.x) * a.z);
    const az = 2 * ((q.x * q.z - q.w * q.y) * a.x + (q.y * q.z + q.w * q.x) * a.y + (0.5 - q.x * q.x - q.y * q.y) * a.z);
    
    // Remove gravity (NWU: gravity is [0, 0, -1])
    return {
      x: ax,
      y: ay,
      z: az + 1.0  // Add 1g to remove gravity (which was -1g in Z)
    };
  }
  
  /**
   * Get internal states for diagnostics
   */
  getInternalStates(): FusionAhrsInternalStates {
    // Convert squared error back to degrees for display
    const feedbackMag = Math.sqrt(vectorMagnitudeSquared(this.halfAccelerometerFeedback));
    const accelError = Math.asin(Math.min(1, 2 * feedbackMag)) * RAD_TO_DEG;
    
    const magFeedbackMag = Math.sqrt(vectorMagnitudeSquared(this.halfMagnetometerFeedback));
    const magError = Math.asin(Math.min(1, 2 * magFeedbackMag)) * RAD_TO_DEG;
    
    const recoveryPeriod = this.settings.recoveryTriggerPeriod || 1;
    
    return {
      accelerationError: isNaN(accelError) ? 0 : accelError,
      accelerometerIgnored: this.accelerometerIgnored,
      accelerationRecoveryTrigger: this.accelerationRecoveryTrigger / recoveryPeriod,
      magneticError: isNaN(magError) ? 0 : magError,
      magnetometerIgnored: this.magnetometerIgnored,
      magneticRecoveryTrigger: this.magneticRecoveryTrigger / recoveryPeriod
    };
  }
  
  /**
   * Get status flags
   */
  getFlags(): FusionAhrsFlags {
    return {
      initialising: this.initialising,
      angularRateRecovery: this.angularRateRecovery,
      accelerationRecovery: this.accelerationRecoveryTimeout === 0,
      magneticRecovery: this.magneticRecoveryTimeout === 0
    };
  }
  
  /**
   * Set heading (yaw) to a specific value while preserving roll and pitch
   */
  setHeading(heading: number): void {
    const euler = this.getEulerAngles();
    
    // Reconstruct quaternion with new heading
    const cy = Math.cos(heading * 0.5);
    const sy = Math.sin(heading * 0.5);
    const cp = Math.cos(euler.pitch * 0.5);
    const sp = Math.sin(euler.pitch * 0.5);
    const cr = Math.cos(euler.roll * 0.5);
    const sr = Math.sin(euler.roll * 0.5);
    
    this.quaternion = {
      w: cr * cp * cy + sr * sp * sy,
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy
    };
  }
}
