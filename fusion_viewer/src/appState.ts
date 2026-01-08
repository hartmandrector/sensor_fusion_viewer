/**
 * Application State Module
 * 
 * Centralized state management for the sensor fusion viewer.
 * All modules can import and modify this shared state.
 */

import type { SensorDataset, IMUData, MAGData } from './csvParser';
import type { MadgwickAHRS } from './fusion';
import type { FusionAhrsAdapter } from './FusionAhrsAdapter';
import type { OrientationViewer } from './viewer';
import type { MagCalibrationResult } from './magCalibration';
import type { IMUCalibrationResult } from './imuCalibration';
import type { EllipsoidFitResult } from './ellipsoidFit';
import type { AccelCalibration6PosResult } from './accelCalibration6Pos';

// ============================================================================
// Algorithm Type
// ============================================================================

export type AlgorithmType = 'madgwick' | 'fusion';

// ============================================================================
// AHRS Interface (common between algorithms)
// ============================================================================

/**
 * Common AHRS interface that both algorithms implement
 */
export interface AHRSInterface {
  reset(): void;
  updateMag(mx: number, my: number, mz: number): void;
  updateIMU(dt: number, wx: number, wy: number, wz: number, ax: number, ay: number, az: number): void;
  initFromAccelMag(ax: number, ay: number, az: number, mx: number, my: number, mz: number): void;
  initFromAccelOnly(ax: number, ay: number, az: number): void;
  getOutput(): { quaternion: { w: number; x: number; y: number; z: number }; euler: { roll: number; pitch: number; yaw: number }; heading: number };
  getCalibratedMag(): { x: number; y: number; z: number; valid: boolean };
  getIMUCalibration(): { gyroBiasX: number; gyroBiasY: number; gyroBiasZ: number; accelOffsetX: number; accelOffsetY: number; accelOffsetZ: number };
  getMagCalibration(): { offsetX: number; offsetY: number; offsetZ: number; scaleX: number; scaleY: number; scaleZ: number };
  setIMUCalibration(cal: { gyroBiasX: number; gyroBiasY: number; gyroBiasZ: number; accelOffsetX: number; accelOffsetY: number; accelOffsetZ: number }): void;
  setMagCalibration(cal: { offsetX: number; offsetY: number; offsetZ: number; scaleX: number; scaleY: number; scaleZ: number }): void;
  setGyroBias(x: number, y: number, z: number): void;
  setAccelOffset(x: number, y: number, z: number): void;
  applyIMURemap(x: number, y: number, z: number): { x: number; y: number; z: number };
  setIMUAxisRemap?(remap: import('./types').AxisRemap): void;
  setMagAxisRemap?(remap: import('./types').AxisRemap): void;
  // Optional advanced calibration methods
  setSoftIronMatrix?(matrix: number[][]): void;
  setAccelScaleMatrix?(matrix: number[][]): void;
  // Acceleration outputs (gravity-compensated)
  getGravityVector(): { x: number; y: number; z: number };
  getLinearAcceleration(): { x: number; y: number; z: number };
  getEarthAcceleration(): { x: number; y: number; z: number };
}

// ============================================================================
// Fusion Frame Type
// ============================================================================

/**
 * Internal AHRS states for Fusion Ch.7
 */
export interface FusionInternalStates {
  accelerationError: number;
  accelerometerIgnored: boolean;
  accelerationRecoveryTrigger: number;
  magneticError: number;
  magnetometerIgnored: boolean;
  magneticRecoveryTrigger: number;
}

/**
 * Runtime bias estimation state
 */
export interface BiasState {
  bias: { x: number; y: number; z: number };  // degrees/s
  isCalibrating: boolean;
  progress: number;  // 0-1
  stationaryTime: number;  // seconds
  gyroMagnitude: number;  // degrees/s - current gyro magnitude
  stationaryThreshold: number;  // degrees/s - threshold for stationary
}

/**
 * Pre-computed fusion result for a single frame
 */
export interface FusionFrame {
  timestamp: number;
  quaternion: { w: number; x: number; y: number; z: number };
  euler: { roll: number; pitch: number; yaw: number };
  heading: number;
  imu?: IMUData;
  mag?: MAGData;
  calibratedMag?: { x: number; y: number; z: number };
  // Derived acceleration vectors (computed at frame time)
  linearAccel?: { x: number; y: number; z: number };
  earthAccel?: { x: number; y: number; z: number };
  gravity?: { x: number; y: number; z: number };
  // Fusion Ch.7 specific
  internalStates?: FusionInternalStates;
  biasState?: BiasState;
}

// ============================================================================
// Application State
// ============================================================================

/**
 * Global application state
 */
export const state = {
  // Core components
  viewer: null as OrientationViewer | null,
  dataset: null as SensorDataset | null,
  ahrs: null as AHRSInterface | null,
  
  // Current file info
  currentFileName: null as string | null,
  
  // Algorithm selection
  algorithm: 'fusion' as AlgorithmType,
  madgwickAhrs: null as MadgwickAHRS | null,
  fusionAhrs: null as FusionAhrsAdapter | null,
  
  // Playback state
  isPlaying: false,
  playbackIndex: 0,
  playbackSpeed: 1.0,
  lastFrameTime: 0,
  currentSimTime: 0,
  
  // Pre-computed fusion results
  fusionFrames: [] as FusionFrame[],
  
  // Calibration results
  lastMagCalibration: null as MagCalibrationResult | null,
  lastIMUCalibration: null as IMUCalibrationResult | null,
  lastEllipsoidCalibration: null as EllipsoidFitResult | null,
  lastAccel6PosCalibration: null as AccelCalibration6PosResult | null,
  
  // Full calibration matrices (for advanced calibration)
  accelScaleMatrix: null as number[][] | null,
  accelBias: null as number[] | null,
  softIronMatrix: null as number[][] | null,
  
  // Integration results
  integrationResult: null as import('./accelerationIntegration').IntegrationResult | null,
};

// ============================================================================
// Default Calibration Values
// ============================================================================

/**
 * Default calibration values (FlySight S.N. 2-00176)
 * Calibrated with correct MAG axis remap (-X, +Y, -Z)
 */
export const DEFAULT_CALIBRATION = {
  mag: {
    offsetX: -0.3465,
    offsetY: -0.0545,
    offsetZ: -0.5380,
    scaleX: 1,
    scaleY: 1,
    scaleZ: 1,
  },
  gyro: {
    biasX: -0.1339,
    biasY: -0.1801,
    biasZ: -0.2390,
  },
  accel: {
    offsetX: -0.0118,
    offsetY: 0.0123,
    offsetZ: 0.0450,
  },
} as const;

// ============================================================================
// State Helpers
// ============================================================================

/**
 * Reset playback state to initial values
 */
export function resetPlaybackState(): void {
  state.isPlaying = false;
  state.playbackIndex = 0;
  state.currentSimTime = state.dataset?.startTime ?? 0;
}

/**
 * Clear all state (for testing or reset)
 */
export function clearState(): void {
  state.viewer = null;
  state.dataset = null;
  state.currentFileName = null;
  state.ahrs = null;
  state.isPlaying = false;
  state.playbackIndex = 0;
  state.playbackSpeed = 1.0;
  state.lastFrameTime = 0;
  state.currentSimTime = 0;
  state.fusionFrames = [];
  state.lastMagCalibration = null;
  state.lastIMUCalibration = null;
}
