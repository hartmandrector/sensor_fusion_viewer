/**
 * Application State Module
 * 
 * Centralized state management for the sensor fusion viewer.
 * All modules can import and modify this shared state.
 */

import type { SensorDataset, IMUData, MAGData } from './csvParser';
import type { MadgwickAHRS } from './fusion';
import type { OrientationViewer } from './viewer';
import type { MagCalibrationResult } from './magCalibration';
import type { IMUCalibrationResult } from './imuCalibration';

// ============================================================================
// Fusion Frame Type
// ============================================================================

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
  ahrs: null as MadgwickAHRS | null,
  
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
