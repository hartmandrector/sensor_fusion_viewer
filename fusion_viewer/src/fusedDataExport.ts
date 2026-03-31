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
 * - GPS data (when loaded): position, velocity, acceleration, lat/lon/alt
 */

import { state, FusionFrame } from './appState';
import { getElements } from './uiElements';
import { debug } from './constants';
import { gpsState } from './gpsIntegration';
import type { GPSIntegrationPoint } from './gpsTypes';

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
// GPS Interpolation
// ============================================================================

/**
 * Linearly interpolate a GPS value at a given sensor timestamp.
 * Returns null if timestamp is outside GPS coverage.
 */
function interpolateGPSAtTime(
  gpsPoints: GPSIntegrationPoint[],
  t: number
): GPSIntegrationPoint | null {
  if (gpsPoints.length === 0) return null;
  if (t < gpsPoints[0].sensorTime || t > gpsPoints[gpsPoints.length - 1].sensorTime) return null;

  // Binary search for bracketing interval
  let lo = 0, hi = gpsPoints.length - 1;
  while (lo < hi - 1) {
    const mid = (lo + hi) >> 1;
    if (gpsPoints[mid].sensorTime <= t) lo = mid;
    else hi = mid;
  }

  const a = gpsPoints[lo];
  const b = gpsPoints[hi];
  const dt = b.sensorTime - a.sensorTime;
  if (dt <= 0) return a;

  const frac = (t - a.sensorTime) / dt;

  // Lerp all numeric fields
  const lerp = (va: number, vb: number) => va + (vb - va) * frac;

  return {
    sensorTime: t,
    velNorth: lerp(a.velNorth, b.velNorth),
    velWest: lerp(a.velWest, b.velWest),
    velUp: lerp(a.velUp, b.velUp),
    smoothVelNorth: lerp(a.smoothVelNorth, b.smoothVelNorth),
    smoothVelWest: lerp(a.smoothVelWest, b.smoothVelWest),
    smoothVelUp: lerp(a.smoothVelUp, b.smoothVelUp),
    posNorth: lerp(a.posNorth, b.posNorth),
    posWest: lerp(a.posWest, b.posWest),
    posUp: lerp(a.posUp, b.posUp),
    horizontalSpeed: lerp(a.horizontalSpeed, b.horizontalSpeed),
    horizontalDistance: lerp(a.horizontalDistance, b.horizontalDistance),
    smoothHorizontalSpeed: lerp(a.smoothHorizontalSpeed, b.smoothHorizontalSpeed),
    accelNorth: lerp(a.accelNorth, b.accelNorth),
    accelWest: lerp(a.accelWest, b.accelWest),
    accelUp: lerp(a.accelUp, b.accelUp),
    lat: lerp(a.lat, b.lat),
    lon: lerp(a.lon, b.lon),
    hMSL: lerp(a.hMSL, b.hMSL),
    hAcc: lerp(a.hAcc, b.hAcc),
    vAcc: lerp(a.vAcc, b.vAcc),
    numSV: Math.round(lerp(a.numSV, b.numSV)),
  };
}

// ============================================================================
// CSV Generation
// ============================================================================

/**
 * Generate CSV content from fusion frames, optionally merged with GPS data
 */
function generateCSV(frames: FusionFrame[]): string {
  const rows: string[] = [];

  // Check if GPS data is available
  const gpsPoints = gpsState.integrationResult?.points ?? [];
  const hasGPS = gpsPoints.length > 0;
  
  // Documentation header (comment lines starting with #)
  rows.push('# FlySight Sensor Fusion Export');
  rows.push(`# Generated: ${new Date().toISOString()}`);
  rows.push(`# Algorithm: ${state.algorithm}`);
  rows.push(`# Source file: ${state.currentFileName || 'unknown'}`);
  rows.push(`# Frames: ${frames.length}`);
  rows.push(`# GPS data: ${hasGPS ? `${gpsPoints.length} points` : 'not loaded'}`);
  rows.push('#');
  rows.push('# COORDINATE SYSTEMS:');
  rows.push('#   Earth Frame (NWU): X=North, Y=West, Z=Up (right-handed)');
  rows.push('#   Body Frame: Sensor-fixed frame, orientation depends on mounting');
  if (hasGPS) {
    rows.push('#   GPS columns use NWU frame, position relative to first GPS fix');
    rows.push('#   GPS values interpolated to sensor timestamps (NaN = outside GPS coverage)');
  }
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
  rows.push('#   mag_*: Calibrated magnetometer reading (body frame, µT)');
  if (hasGPS) {
    rows.push('#   gps_lat/gps_lon/gps_hMSL: GPS position (WGS84)');
    rows.push('#   gps_pos_north/west/up: Local position (meters from origin, NWU)');
    rows.push('#   gps_vel_north/west/up: GPS velocity (m/s, NWU)');
    rows.push('#   gps_svel_north/west/up: Smoothed GPS velocity (m/s, NWU, SG filtered)');
    rows.push('#   gps_accel_north/west/up: GPS-derived acceleration (m/s², NWU)');
    rows.push('#   gps_hspeed: Horizontal speed (m/s)');
    rows.push('#   gps_numSV: Number of satellites');
  }
  rows.push('#');
  
  // Column names header
  const headers = [
    'timestamp',
    'r00', 'r01', 'r02',
    'r10', 'r11', 'r12',
    'r20', 'r21', 'r22',
    'roll', 'pitch', 'yaw',
    'accel_body_x', 'accel_body_y', 'accel_body_z',
    'accel_earth_x', 'accel_earth_y', 'accel_earth_z',
    'earth_accel_x', 'earth_accel_y', 'earth_accel_z',
    'gravity_body_x', 'gravity_body_y', 'gravity_body_z',
    'linear_accel_x', 'linear_accel_y', 'linear_accel_z',
    'gyro_x', 'gyro_y', 'gyro_z',
    'qw', 'qx', 'qy', 'qz',
    'mag_x', 'mag_y', 'mag_z'
  ];
  
  const units = [
    's',
    '-', '-', '-',
    '-', '-', '-',
    '-', '-', '-',
    'deg', 'deg', 'deg',
    'g', 'g', 'g',
    'g', 'g', 'g',
    'g', 'g', 'g',
    'g', 'g', 'g',
    'g', 'g', 'g',
    'deg/s', 'deg/s', 'deg/s',
    '-', '-', '-', '-',
    'uT', 'uT', 'uT'
  ];

  if (hasGPS) {
    headers.push(
      'gps_lat', 'gps_lon', 'gps_hMSL',
      'gps_pos_north', 'gps_pos_west', 'gps_pos_up',
      'gps_vel_north', 'gps_vel_west', 'gps_vel_up',
      'gps_svel_north', 'gps_svel_west', 'gps_svel_up',
      'gps_accel_north', 'gps_accel_west', 'gps_accel_up',
      'gps_hspeed', 'gps_numSV'
    );
    units.push(
      'deg', 'deg', 'm',
      'm', 'm', 'm',
      'm/s', 'm/s', 'm/s',
      'm/s', 'm/s', 'm/s',
      'm/s2', 'm/s2', 'm/s2',
      'm/s', '-'
    );
  }
  
  rows.push(headers.join(','));
  rows.push(units.join(','));
  
  for (const frame of frames) {
    const rotMatrix = quaternionToRotationMatrix(frame.quaternion);
    
    const accelBody = frame.imu 
      ? { x: frame.imu.ax, y: frame.imu.ay, z: frame.imu.az }
      : { x: 0, y: 0, z: 0 };
    
    const accelEarth = rotateToEarthFrame(accelBody, rotMatrix);
    const earthAccel = frame.earthAccel ?? { x: 0, y: 0, z: 0 };
    const gravityBody = frame.gravity ?? { x: 0, y: 0, z: 0 };
    const linearAccel = frame.linearAccel ?? { x: 0, y: 0, z: 0 };
    
    const gyro = frame.imu
      ? { x: frame.imu.wx, y: frame.imu.wy, z: frame.imu.wz }
      : { x: 0, y: 0, z: 0 };
    
    const RAD_TO_DEG = 180 / Math.PI;
    const rollDeg = frame.euler.roll * RAD_TO_DEG;
    const pitchDeg = frame.euler.pitch * RAD_TO_DEG;
    const yawDeg = frame.euler.yaw * RAD_TO_DEG;
    
    const row = [
      frame.timestamp.toFixed(6),
      rotMatrix[0].toFixed(6), rotMatrix[1].toFixed(6), rotMatrix[2].toFixed(6),
      rotMatrix[3].toFixed(6), rotMatrix[4].toFixed(6), rotMatrix[5].toFixed(6),
      rotMatrix[6].toFixed(6), rotMatrix[7].toFixed(6), rotMatrix[8].toFixed(6),
      rollDeg.toFixed(4),
      pitchDeg.toFixed(4),
      yawDeg.toFixed(4),
      accelBody.x.toFixed(6), accelBody.y.toFixed(6), accelBody.z.toFixed(6),
      accelEarth.x.toFixed(6), accelEarth.y.toFixed(6), accelEarth.z.toFixed(6),
      earthAccel.x.toFixed(6), earthAccel.y.toFixed(6), earthAccel.z.toFixed(6),
      gravityBody.x.toFixed(6), gravityBody.y.toFixed(6), gravityBody.z.toFixed(6),
      linearAccel.x.toFixed(6), linearAccel.y.toFixed(6), linearAccel.z.toFixed(6),
      gyro.x.toFixed(4), gyro.y.toFixed(4), gyro.z.toFixed(4),
      frame.quaternion.w.toFixed(6),
      frame.quaternion.x.toFixed(6),
      frame.quaternion.y.toFixed(6),
      frame.quaternion.z.toFixed(6),
      // Calibrated magnetometer (body frame, µT)
      frame.calibratedMag ? frame.calibratedMag.x.toFixed(4) : 'NaN',
      frame.calibratedMag ? frame.calibratedMag.y.toFixed(4) : 'NaN',
      frame.calibratedMag ? frame.calibratedMag.z.toFixed(4) : 'NaN'
    ];

    // Append GPS columns if available
    if (hasGPS) {
      const gps = interpolateGPSAtTime(gpsPoints, frame.timestamp);
      if (gps) {
        row.push(
          gps.lat.toFixed(8), gps.lon.toFixed(8), gps.hMSL.toFixed(2),
          gps.posNorth.toFixed(3), gps.posWest.toFixed(3), gps.posUp.toFixed(3),
          gps.velNorth.toFixed(3), gps.velWest.toFixed(3), gps.velUp.toFixed(3),
          gps.smoothVelNorth.toFixed(3), gps.smoothVelWest.toFixed(3), gps.smoothVelUp.toFixed(3),
          gps.accelNorth.toFixed(4), gps.accelWest.toFixed(4), gps.accelUp.toFixed(4),
          gps.smoothHorizontalSpeed.toFixed(3),
          gps.numSV.toString()
        );
      } else {
        // Outside GPS time coverage — fill with NaN
        row.push(...Array(17).fill('NaN'));
      }
    }
    
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
