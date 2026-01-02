/**
 * Magnetometer Calibration Tool
 * 
 * Collects magnetometer samples and computes hard iron calibration offsets.
 * 
 * Hard iron calibration: Find the center of the magnetometer data ellipsoid.
 * The offsets are simply: offset = (max + min) / 2 for each axis.
 * 
 * For good calibration:
 * 1. Rotate device slowly through ALL orientations
 * 2. Cover at least 60-90 seconds of tumbling
 * 3. Try to get even coverage (not just spinning around one axis)
 */

import type { MAGData } from './csvParser';

export interface MagCalibrationResult {
  offsetX: number;
  offsetY: number;
  offsetZ: number;
  
  // Statistics
  minX: number; maxX: number;
  minY: number; maxY: number;
  minZ: number; maxZ: number;
  
  // Quality metrics
  sampleCount: number;
  rangeX: number;
  rangeY: number;
  rangeZ: number;
  sphericity: number;  // How close to a sphere (1.0 = perfect)
  magnitude: number;   // Average field magnitude after calibration
}

/**
 * Calculate hard iron calibration from magnetometer samples
 */
export function calculateHardIronCalibration(
  samples: MAGData[],
  applyAxisTransform: boolean = true
): MagCalibrationResult {
  if (samples.length === 0) {
    return {
      offsetX: 0, offsetY: 0, offsetZ: 0,
      minX: 0, maxX: 0, minY: 0, maxY: 0, minZ: 0, maxZ: 0,
      sampleCount: 0, rangeX: 0, rangeY: 0, rangeZ: 0,
      sphericity: 0, magnitude: 0
    };
  }
  
  // Transform samples to device frame if needed
  const transformed = samples.map(s => {
    if (applyAxisTransform) {
      return { x: -s.x, y: s.y, z: -s.z };
    }
    return { x: s.x, y: s.y, z: s.z };
  });
  
  // Find min/max for each axis
  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;
  
  for (const s of transformed) {
    minX = Math.min(minX, s.x);
    maxX = Math.max(maxX, s.x);
    minY = Math.min(minY, s.y);
    maxY = Math.max(maxY, s.y);
    minZ = Math.min(minZ, s.z);
    maxZ = Math.max(maxZ, s.z);
  }
  
  // Hard iron offset = center of the ellipsoid
  const offsetX = (maxX + minX) / 2;
  const offsetY = (maxY + minY) / 2;
  const offsetZ = (maxZ + minZ) / 2;
  
  // Compute ranges
  const rangeX = maxX - minX;
  const rangeY = maxY - minY;
  const rangeZ = maxZ - minZ;
  
  // Compute sphericity (how close ranges are to each other)
  const avgRange = (rangeX + rangeY + rangeZ) / 3;
  const sphericity = avgRange > 0 
    ? 1 - (Math.abs(rangeX - avgRange) + Math.abs(rangeY - avgRange) + Math.abs(rangeZ - avgRange)) / (3 * avgRange)
    : 0;
  
  // Compute average magnitude after calibration
  let totalMag = 0;
  for (const s of transformed) {
    const cx = s.x - offsetX;
    const cy = s.y - offsetY;
    const cz = s.z - offsetZ;
    totalMag += Math.sqrt(cx * cx + cy * cy + cz * cz);
  }
  const magnitude = totalMag / transformed.length;
  
  return {
    offsetX, offsetY, offsetZ,
    minX, maxX, minY, maxY, minZ, maxZ,
    sampleCount: samples.length,
    rangeX, rangeY, rangeZ,
    sphericity,
    magnitude
  };
}

/**
 * Generate points for a 3D scatter plot visualization
 */
export function getMagDataPoints(
  samples: MAGData[],
  applyAxisTransform: boolean = true,
  calibration?: { offsetX: number; offsetY: number; offsetZ: number }
): { x: number; y: number; z: number }[] {
  return samples.map(s => {
    let x = applyAxisTransform ? -s.x : s.x;
    let y = s.y;
    let z = applyAxisTransform ? -s.z : s.z;
    
    if (calibration) {
      x -= calibration.offsetX;
      y -= calibration.offsetY;
      z -= calibration.offsetZ;
    }
    
    return { x, y, z };
  });
}

/**
 * Evaluate calibration quality
 */
export function evaluateCalibrationQuality(result: MagCalibrationResult): {
  quality: 'excellent' | 'good' | 'fair' | 'poor';
  issues: string[];
  recommendations: string[];
} {
  const issues: string[] = [];
  const recommendations: string[] = [];
  
  // Check sample count
  if (result.sampleCount < 100) {
    issues.push(`Only ${result.sampleCount} samples - need more data`);
    recommendations.push('Collect at least 60 seconds of data at 100Hz');
  }
  
  // Check sphericity
  if (result.sphericity < 0.7) {
    issues.push(`Low sphericity (${(result.sphericity * 100).toFixed(0)}%) - ranges are uneven`);
    recommendations.push('Rotate through more orientations, especially around undersampled axes');
  }
  
  // Check if ranges are reasonable (Earth's field is ~0.25-0.65 gauss)
  const avgRange = (result.rangeX + result.rangeY + result.rangeZ) / 3;
  
  if (avgRange < 0.3) {
    issues.push('Very small range - device may not have been rotated enough');
    recommendations.push('Tumble the device through ALL orientations (roll, pitch, yaw)');
  }
  
  // Check field magnitude (should be ~0.25-0.65 gauss for Earth's field)
  if (result.magnitude < 0.2 || result.magnitude > 0.8) {
    issues.push(`Unusual field magnitude (${result.magnitude.toFixed(3)} gauss)`);
    recommendations.push('Ensure calibration is done away from magnetic interference');
  }
  
  // Determine overall quality
  let quality: 'excellent' | 'good' | 'fair' | 'poor';
  if (issues.length === 0 && result.sphericity > 0.9 && result.sampleCount >= 500) {
    quality = 'excellent';
  } else if (issues.length <= 1 && result.sphericity > 0.8) {
    quality = 'good';
  } else if (issues.length <= 2 && result.sphericity > 0.6) {
    quality = 'fair';
  } else {
    quality = 'poor';
  }
  
  return { quality, issues, recommendations };
}
