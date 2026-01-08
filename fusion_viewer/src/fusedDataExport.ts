/**
 * Fused Data Export Module
 * 
 * Exports fusion results to CSV including:
 * - Timestamp
 * - Rotation matrix (3x3)
 * - Euler angles
 * - Raw accelerometer (body frame)
 * - Earth-frame acceleration (gravity removed)
 * - Gravity vector
 */

import { state, FusionFrame } from './appState';
import { getElements } from './uiElements';
import { debug } from './constants';

// ============================================================================
// Quaternion to Rotation Matrix
// ============================================================================

/**
 * Convert quaternion to 3x3 rotation matrix
 * Returns matrix as row-major array [r00, r01, r02, r10, r11, r12, r20, r21, r22]
 * 
 * This rotation matrix transforms vectors from body frame to earth/world frame
 */
function quaternionToRotationMatrix(q: { w: number; x: number; y: number; z: number }): number[] {
  const { w, x, y, z } = q;
  
  // Normalize quaternion (should already be normalized, but just in case)
  const n = Math.sqrt(w*w + x*x + y*y + z*z);
  const qw = w / n;
  const qx = x / n;
  const qy = y / n;
  const qz = z / n;
  
  // Rotation matrix elements
  const r00 = 1 - 2*(qy*qy + qz*qz);
  const r01 = 2*(qx*qy - qz*qw);
  const r02 = 2*(qx*qz + qy*qw);
  
  const r10 = 2*(qx*qy + qz*qw);
  const r11 = 1 - 2*(qx*qx + qz*qz);
  const r12 = 2*(qy*qz - qx*qw);
  
  const r20 = 2*(qx*qz - qy*qw);
  const r21 = 2*(qy*qz + qx*qw);
  const r22 = 1 - 2*(qx*qx + qy*qy);
  
  return [r00, r01, r02, r10, r11, r12, r20, r21, r22];
}

/**
 * Rotate a vector from body frame to earth frame using rotation matrix
 */
function rotateToEarthFrame(
  v: { x: number; y: number; z: number },
  rotMatrix: number[]
): { x: number; y: number; z: number } {
  const [r00, r01, r02, r10, r11, r12, r20, r21, r22] = rotMatrix;
  
  return {
    x: r00 * v.x + r01 * v.y + r02 * v.z,
    y: r10 * v.x + r11 * v.y + r12 * v.z,
    z: r20 * v.x + r21 * v.y + r22 * v.z
  };
}

// ============================================================================
// CSV Generation
// ============================================================================

/**
 * Generate CSV content from fusion frames
 */
function generateCSV(frames: FusionFrame[]): string {
  const rows: string[] = [];
  
  // Documentation header (comment lines starting with #)
  rows.push('# FlySight Sensor Fusion Export');
  rows.push(`# Generated: ${new Date().toISOString()}`);
  rows.push(`# Algorithm: ${state.algorithm}`);
  rows.push(`# Source file: ${state.currentFileName || 'unknown'}`);
  rows.push(`# Frames: ${frames.length}`);
  rows.push('#');
  rows.push('# COORDINATE SYSTEMS:');
  rows.push('#   Earth Frame (NWU): X=North, Y=West, Z=Up (right-handed)');
  rows.push('#   Body Frame: Sensor-fixed frame, orientation depends on mounting');
  rows.push('#');
  rows.push('# COLUMN DESCRIPTIONS:');
  rows.push('#   timestamp: Time since data start');
  rows.push('#   r00-r22: Rotation matrix (body-to-earth transform, row-major)');
  rows.push('#            To rotate vector v from body to earth: v_earth = R * v_body');
  rows.push('#   roll/pitch/yaw: Euler angles (earth frame)');
  rows.push('#   accel_body_*: Raw accelerometer reading (body frame)');
  rows.push('#   accel_earth_*: Raw accelerometer rotated to earth frame');
  rows.push('#   earth_accel_*: Acceleration in earth frame with gravity removed');
  rows.push('#   gravity_body_*: Estimated gravity vector in body frame');
  rows.push('#   linear_accel_*: Linear acceleration in body frame (gravity removed)');
  rows.push('#   gyro_*: Angular velocity (body frame)');
  rows.push('#   q*: Orientation quaternion (body-to-earth, scalar-first: w,x,y,z)');
  rows.push('#');
  
  // Column names header
  const headers = [
    'timestamp',
    // Rotation matrix (row-major)
    'r00', 'r01', 'r02',
    'r10', 'r11', 'r12',
    'r20', 'r21', 'r22',
    // Euler angles (degrees)
    'roll', 'pitch', 'yaw',
    // Raw accelerometer (body frame, g)
    'accel_body_x', 'accel_body_y', 'accel_body_z',
    // Raw accelerometer rotated to earth frame (g)
    'accel_earth_x', 'accel_earth_y', 'accel_earth_z',
    // Earth acceleration (gravity removed, g)
    'earth_accel_x', 'earth_accel_y', 'earth_accel_z',
    // Gravity vector in body frame (g)
    'gravity_body_x', 'gravity_body_y', 'gravity_body_z',
    // Linear acceleration (body frame, gravity removed, g)
    'linear_accel_x', 'linear_accel_y', 'linear_accel_z',
    // Raw gyroscope (deg/s)
    'gyro_x', 'gyro_y', 'gyro_z',
    // Quaternion (for reference)
    'qw', 'qx', 'qy', 'qz'
  ];
  
  // Units header
  const units = [
    's',           // timestamp
    // Rotation matrix (dimensionless)
    '-', '-', '-',
    '-', '-', '-',
    '-', '-', '-',
    // Euler angles
    'deg', 'deg', 'deg',
    // Accel body
    'g', 'g', 'g',
    // Accel earth
    'g', 'g', 'g',
    // Earth accel
    'g', 'g', 'g',
    // Gravity body
    'g', 'g', 'g',
    // Linear accel
    'g', 'g', 'g',
    // Gyro
    'deg/s', 'deg/s', 'deg/s',
    // Quaternion (dimensionless)
    '-', '-', '-', '-'
  ];
  
  rows.push(headers.join(','));
  rows.push(units.join(','));
  
  for (const frame of frames) {
    // Compute rotation matrix from quaternion
    const rotMatrix = quaternionToRotationMatrix(frame.quaternion);
    
    // Get raw accelerometer data (body frame)
    const accelBody = frame.imu 
      ? { x: frame.imu.ax, y: frame.imu.ay, z: frame.imu.az }
      : { x: 0, y: 0, z: 0 };
    
    // Rotate raw accelerometer to earth frame
    const accelEarth = rotateToEarthFrame(accelBody, rotMatrix);
    
    // Earth acceleration (from AHRS, gravity removed)
    const earthAccel = frame.earthAccel ?? { x: 0, y: 0, z: 0 };
    
    // Gravity vector in body frame
    const gravityBody = frame.gravity ?? { x: 0, y: 0, z: 0 };
    
    // Linear acceleration (body frame, gravity removed)
    const linearAccel = frame.linearAccel ?? { x: 0, y: 0, z: 0 };
    
    // Gyroscope data
    const gyro = frame.imu
      ? { x: frame.imu.wx, y: frame.imu.wy, z: frame.imu.wz }
      : { x: 0, y: 0, z: 0 };
    
    // Convert euler from radians to degrees
    const RAD_TO_DEG = 180 / Math.PI;
    const rollDeg = frame.euler.roll * RAD_TO_DEG;
    const pitchDeg = frame.euler.pitch * RAD_TO_DEG;
    const yawDeg = frame.euler.yaw * RAD_TO_DEG;
    
    const row = [
      frame.timestamp.toFixed(6),
      // Rotation matrix
      rotMatrix[0].toFixed(6), rotMatrix[1].toFixed(6), rotMatrix[2].toFixed(6),
      rotMatrix[3].toFixed(6), rotMatrix[4].toFixed(6), rotMatrix[5].toFixed(6),
      rotMatrix[6].toFixed(6), rotMatrix[7].toFixed(6), rotMatrix[8].toFixed(6),
      // Euler angles (converted to degrees)
      rollDeg.toFixed(4),
      pitchDeg.toFixed(4),
      yawDeg.toFixed(4),
      // Accelerometer body frame
      accelBody.x.toFixed(6), accelBody.y.toFixed(6), accelBody.z.toFixed(6),
      // Accelerometer earth frame
      accelEarth.x.toFixed(6), accelEarth.y.toFixed(6), accelEarth.z.toFixed(6),
      // Earth acceleration
      earthAccel.x.toFixed(6), earthAccel.y.toFixed(6), earthAccel.z.toFixed(6),
      // Gravity body frame
      gravityBody.x.toFixed(6), gravityBody.y.toFixed(6), gravityBody.z.toFixed(6),
      // Linear acceleration
      linearAccel.x.toFixed(6), linearAccel.y.toFixed(6), linearAccel.z.toFixed(6),
      // Gyroscope
      gyro.x.toFixed(4), gyro.y.toFixed(4), gyro.z.toFixed(4),
      // Quaternion
      frame.quaternion.w.toFixed(6),
      frame.quaternion.x.toFixed(6),
      frame.quaternion.y.toFixed(6),
      frame.quaternion.z.toFixed(6)
    ];
    
    rows.push(row.join(','));
  }
  
  return rows.join('\n');
}

/**
 * Download CSV file
 */
function downloadCSV(content: string, filename: string): void {
  const blob = new Blob([content], { type: 'text/csv;charset=utf-8;' });
  const link = document.createElement('a');
  const url = URL.createObjectURL(blob);
  
  link.setAttribute('href', url);
  link.setAttribute('download', filename);
  link.style.visibility = 'hidden';
  
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
  
  URL.revokeObjectURL(url);
}

// ============================================================================
// Export Handler
// ============================================================================

/**
 * Handle export button click
 */
export function handleExportFusedData(): void {
  const elements = getElements();
  
  if (!state.dataset || state.fusionFrames.length === 0) {
    elements.exportStatus.textContent = 'No data to export. Load a CSV file first.';
    elements.exportStatus.style.color = '#ff6666';
    return;
  }
  
  try {
    elements.exportStatus.textContent = 'Generating CSV...';
    elements.exportStatus.style.color = '#888';
    
    // Generate CSV content
    const csvContent = generateCSV(state.fusionFrames);
    
    // Create filename based on source file and algorithm
    const sourceBasename = state.currentFileName 
      ? state.currentFileName.replace(/\.csv$/i, '')
      : 'fusion_data';
    const filename = `${sourceBasename}_fused_${state.algorithm}.csv`;
    
    // Download
    downloadCSV(csvContent, filename);
    
    elements.exportStatus.textContent = `Exported ${state.fusionFrames.length} frames to ${filename}`;
    elements.exportStatus.style.color = '#66ff66';
    
    debug.log(`Exported ${state.fusionFrames.length} fusion frames to ${filename}`);
    
  } catch (error) {
    const message = error instanceof Error ? error.message : 'Unknown error';
    elements.exportStatus.textContent = `Export failed: ${message}`;
    elements.exportStatus.style.color = '#ff6666';
    debug.error('Export failed:', error);
  }
}
