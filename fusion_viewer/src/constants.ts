/**
 * Centralized constants for the Sensor Fusion Viewer
 */

import type { MagCalibration, IMUCalibration, AxisRemap } from './types';

// ============================================================================
// Debug Configuration
// ============================================================================

/**
 * Enable/disable debug logging throughout the application
 * Set to false for production builds
 */
export const DEBUG = false;

/**
 * Conditional debug logger
 */
export const debug = {
  log: (...args: unknown[]) => {
    if (DEBUG) console.log(...args);
  },
  warn: (...args: unknown[]) => {
    if (DEBUG) console.warn(...args);
  },
  error: (...args: unknown[]) => {
    // Always log errors
    console.error(...args);
  }
};

// ============================================================================
// Device Physical Dimensions (in visualization units, scaled from cm)
// ============================================================================

/**
 * FlySight 2 device dimensions
 * Real dimensions: 5cm x 5cm x 1.5cm (width x height x depth)
 * Scaled for visualization (1 unit = 5cm)
 */
export const DEVICE_DIMENSIONS = {
  WIDTH: 1.0,   // X axis: 5cm
  HEIGHT: 1.0,  // Y axis: 5cm
  DEPTH: 0.3,   // Z axis: 1.5cm
} as const;

/**
 * Screen bezel inset from device edges
 */
export const SCREEN_INSET = 0.08;

/**
 * Screen depth offset from front face
 */
export const SCREEN_DEPTH_OFFSET = 0.01;

// ============================================================================
// Visualization Colors
// ============================================================================

export const COLORS = {
  // Device colors
  DEVICE_BODY: 0x2a2a4a,
  DEVICE_SCREEN: 0x1a1a2e,
  
  // Axis colors (RGB = XYZ convention)
  AXIS_X: 0xff4444,  // Red
  AXIS_Y: 0x44ff44,  // Green
  AXIS_Z: 0x4444ff,  // Blue
  
  // Sensor vector colors
  ACCEL_VECTOR: 0xffff00,  // Yellow
  MAG_VECTOR: 0xff00ff,    // Magenta
  GYRO_VECTOR: 0x00ffff,   // Cyan
  
  // World reference colors
  WORLD_NORTH: 0x4444ff,   // Blue (matches Z)
  WORLD_EAST: 0xff4444,    // Red (matches X)
  WORLD_UP: 0x44ff44,      // Green (matches Y)
  GROUND_GRID: 0x444444,
  
  // Background
  SCENE_BACKGROUND: 0x1a1a2e,
} as const;

// ============================================================================
// Algorithm Defaults
// ============================================================================

/**
 * Default Madgwick filter gain (beta)
 * Lower = smoother but slower response
 * Higher = faster response but more noise
 */
export const DEFAULT_BETA = 0.1;

/**
 * Default sample rate in Hz
 */
export const DEFAULT_SAMPLE_RATE = 100;

/**
 * Minimum vector magnitude for valid normalization
 */
export const MIN_VECTOR_MAGNITUDE = 0.01;

// ============================================================================
// Default Calibration Values
// ============================================================================

/**
 * Default magnetometer calibration (from calibration runs)
 */
export const DEFAULT_MAG_CALIBRATION: MagCalibration = {
  offsetX: 0.13,
  offsetY: 0.03,
  offsetZ: -0.04,
  scaleX: 1.02,
  scaleY: 1.06,
  scaleZ: 0.93,
};

/**
 * Default IMU calibration (from calibration runs)
 */
export const DEFAULT_IMU_CALIBRATION: IMUCalibration = {
  gyroBiasX: -0.0033,
  gyroBiasY: 0.0073,
  gyroBiasZ: -0.0049,
  accelOffsetX: 0,
  accelOffsetY: 0,
  accelOffsetZ: 0,
};

/**
 * Default axis remapping (identity - no remap)
 */
export const DEFAULT_AXIS_REMAP: AxisRemap = {
  bodyX: '+X',
  bodyY: '+Y',
  bodyZ: '+Z',
};

// ============================================================================
// Coordinate System Documentation
// ============================================================================

/**
 * FlySight 2 Body Frame:
 *   X = West (left side of device)
 *   Y = Up (top of device)
 *   Z = North (front face / screen)
 * 
 * Madgwick NWU Frame (internal algorithm):
 *   X = North
 *   Y = West
 *   Z = Up
 * 
 * Three.js World Frame:
 *   X = East (right)
 *   Y = Up
 *   Z = South (camera looks at -Z for North)
 * 
 * Transform: Body → NWU
 *   NWU_X = Body_Z (North)
 *   NWU_Y = Body_X (West)
 *   NWU_Z = Body_Y (Up)
 * 
 * Transform: NWU → Three.js
 *   Three_X = -NWU_Y (East = -West)
 *   Three_Y = NWU_Z (Up)
 *   Three_Z = -NWU_X (South = -North)
 */
