/**
 * FusionBias - Runtime Gyroscope Bias Estimation
 * 
 * Estimates gyroscope bias during periods when the sensor is stationary.
 * Uses a low-pass filter to track slowly-varying bias while the sensor
 * is still (gyroscope magnitude below threshold for timeout period).
 * 
 * Based on x-io Fusion library bias correction approach.
 * 
 * @license MIT
 */

import type { FusionVector } from './FusionAhrs';

// ============================================================================
// Constants
// ============================================================================

/** Default threshold in rad/s (~3°/s) for detecting stationary state */
const DEFAULT_STATIONARY_THRESHOLD = 3.0 * (Math.PI / 180);

/** Default timeout in seconds before bias update starts */
const DEFAULT_STATIONARY_TIMEOUT = 1.0;  // Reduced from 5s for faster response

/** Default low-pass filter coefficient (~0.02 Hz at 416 Hz sample rate) */
const DEFAULT_LPF_COEFFICIENT = 0.0001;

// ============================================================================
// FusionBias Class
// ============================================================================

export interface FusionBiasSettings {
  /** Gyroscope magnitude threshold for stationary detection (rad/s) */
  stationaryThreshold: number;
  
  /** Time sensor must be stationary before bias update begins (seconds) */
  stationaryTimeout: number;
  
  /** Low-pass filter coefficient (0-1, lower = slower tracking) */
  lpfCoefficient: number;
}

export const FUSION_BIAS_DEFAULT_SETTINGS: FusionBiasSettings = {
  stationaryThreshold: DEFAULT_STATIONARY_THRESHOLD,
  stationaryTimeout: DEFAULT_STATIONARY_TIMEOUT,
  lpfCoefficient: DEFAULT_LPF_COEFFICIENT
};

export class FusionBias {
  private settings: FusionBiasSettings;
  
  /** Accumulated stationary time */
  private stationaryTimer: number;
  
  /** Current bias estimate */
  private bias: FusionVector;
  
  /** Whether currently updating bias */
  private isCalibrating: boolean;
  
  /** Last gyro magnitude for debugging */
  private lastMagnitude: number;
  
  constructor(settings?: Partial<FusionBiasSettings>) {
    this.settings = { ...FUSION_BIAS_DEFAULT_SETTINGS, ...settings };
    this.stationaryTimer = 0;
    this.bias = { x: 0, y: 0, z: 0 };
    this.isCalibrating = false;
    this.lastMagnitude = 0;
  }
  
  /**
   * Apply settings
   */
  applySettings(settings: Partial<FusionBiasSettings>): void {
    this.settings = { ...this.settings, ...settings };
  }
  
  /**
   * Reset bias estimate to zero
   */
  reset(): void {
    this.stationaryTimer = 0;
    this.bias = { x: 0, y: 0, z: 0 };
    this.isCalibrating = false;
    this.lastMagnitude = 0;
  }
  
  /**
   * Update bias estimate and return corrected gyroscope
   * 
   * @param gyroscope Raw gyroscope reading in rad/s
   * @param deltaTime Time step in seconds
   * @returns Bias-corrected gyroscope in rad/s
   */
  update(gyroscope: FusionVector, deltaTime: number): FusionVector {
    // Calculate corrected gyroscope (subtract current bias)
    const corrected: FusionVector = {
      x: gyroscope.x - this.bias.x,
      y: gyroscope.y - this.bias.y,
      z: gyroscope.z - this.bias.z
    };
    
    // Check if stationary (use corrected values)
    const magnitude = Math.sqrt(
      corrected.x * corrected.x + 
      corrected.y * corrected.y + 
      corrected.z * corrected.z
    );
    this.lastMagnitude = magnitude;
    
    if (magnitude < this.settings.stationaryThreshold) {
      // Accumulate stationary time
      this.stationaryTimer += deltaTime;
      
      // Start bias update after timeout
      if (this.stationaryTimer >= this.settings.stationaryTimeout) {
        this.isCalibrating = true;
        
        // Low-pass filter update: bias = bias + k * (gyro - bias)
        // Which simplifies to: bias += k * corrected (since corrected = gyro - bias)
        const k = this.settings.lpfCoefficient;
        this.bias.x += k * corrected.x;
        this.bias.y += k * corrected.y;
        this.bias.z += k * corrected.z;
      }
    } else {
      // Not stationary - reset timer
      this.stationaryTimer = 0;
      this.isCalibrating = false;
    }
    
    return corrected;
  }
  
  /**
   * Get current bias estimate (rad/s)
   */
  getBias(): FusionVector {
    return { ...this.bias };
  }
  
  /**
   * Set bias directly (for loading saved calibration)
   */
  setBias(bias: FusionVector): void {
    this.bias = { ...bias };
  }
  
  /**
   * Get bias in degrees/s for display
   */
  getBiasDegrees(): FusionVector {
    const RAD_TO_DEG = 180 / Math.PI;
    return {
      x: this.bias.x * RAD_TO_DEG,
      y: this.bias.y * RAD_TO_DEG,
      z: this.bias.z * RAD_TO_DEG
    };
  }
  
  /**
   * Check if currently calibrating (updating bias)
   */
  isCurrentlyCalibrating(): boolean {
    return this.isCalibrating;
  }
  
  /**
   * Get time spent stationary (seconds)
   */
  getStationaryTime(): number {
    return this.stationaryTimer;
  }
  
  /**
   * Get progress toward calibration start (0 to 1)
   */
  getCalibrationProgress(): number {
    return Math.min(1, this.stationaryTimer / this.settings.stationaryTimeout);
  }
  
  /**
   * Get last gyro magnitude (rad/s) for debugging
   */
  getGyroMagnitude(): number {
    return this.lastMagnitude;
  }
  
  /**
   * Get last gyro magnitude in deg/s for display
   */
  getGyroMagnitudeDegrees(): number {
    return this.lastMagnitude * (180 / Math.PI);
  }
  
  /**
   * Get stationary threshold in deg/s for display
   */
  getStationaryThresholdDegrees(): number {
    return this.settings.stationaryThreshold * (180 / Math.PI);
  }
}
