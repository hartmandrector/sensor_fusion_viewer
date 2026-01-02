/**
 * IMU Calibration Tool
 * 
 * Calculates gyroscope bias and accelerometer offsets from stationary data.
 * 
 * Gyro Bias: Average gyro reading when device is stationary (should be 0)
 * Accel Bias: Offset from expected gravity vector when device is level
 */

import type { IMUData } from './csvParser';

export interface IMUCalibrationResult {
  // Gyroscope bias (deg/s) - subtract from raw readings
  gyroBiasX: number;
  gyroBiasY: number;
  gyroBiasZ: number;
  
  // Accelerometer offset (g) - subtract from raw readings
  accelOffsetX: number;
  accelOffsetY: number;
  accelOffsetZ: number;  // Note: This is offset from 1.0, not 0
  
  // Statistics
  sampleCount: number;
  gyroStdDevX: number;
  gyroStdDevY: number;
  gyroStdDevZ: number;
  accelMagnitude: number;  // Should be ~1.0g
  
  // Quality
  isStationary: boolean;  // Was device actually stationary?
}

/**
 * Calculate IMU calibration from stationary data
 * 
 * IMPORTANT: Device must be stationary and preferably level during this data collection
 */
export function calculateIMUCalibration(samples: IMUData[]): IMUCalibrationResult {
  if (samples.length === 0) {
    return {
      gyroBiasX: 0, gyroBiasY: 0, gyroBiasZ: 0,
      accelOffsetX: 0, accelOffsetY: 0, accelOffsetZ: 0,
      sampleCount: 0,
      gyroStdDevX: 0, gyroStdDevY: 0, gyroStdDevZ: 0,
      accelMagnitude: 0,
      isStationary: false
    };
  }
  
  // Calculate means
  let sumGx = 0, sumGy = 0, sumGz = 0;
  let sumAx = 0, sumAy = 0, sumAz = 0;
  
  for (const s of samples) {
    sumGx += s.wx;
    sumGy += s.wy;
    sumGz += s.wz;
    sumAx += s.ax;
    sumAy += s.ay;
    sumAz += s.az;
  }
  
  const n = samples.length;
  const meanGx = sumGx / n;
  const meanGy = sumGy / n;
  const meanGz = sumGz / n;
  const meanAx = sumAx / n;
  const meanAy = sumAy / n;
  const meanAz = sumAz / n;
  
  // Calculate standard deviations for gyro (to check if stationary)
  let sumSqGx = 0, sumSqGy = 0, sumSqGz = 0;
  for (const s of samples) {
    sumSqGx += (s.wx - meanGx) ** 2;
    sumSqGy += (s.wy - meanGy) ** 2;
    sumSqGz += (s.wz - meanGz) ** 2;
  }
  
  const stdGx = Math.sqrt(sumSqGx / n);
  const stdGy = Math.sqrt(sumSqGy / n);
  const stdGz = Math.sqrt(sumSqGz / n);
  
  // Calculate accel magnitude
  const accelMag = Math.sqrt(meanAx * meanAx + meanAy * meanAy + meanAz * meanAz);
  
  // Check if device was stationary (low gyro std dev and accel ~1g)
  const gyroThreshold = 5.0;  // deg/s std dev threshold
  const isStationary = stdGx < gyroThreshold && stdGy < gyroThreshold && stdGz < gyroThreshold &&
                       accelMag > 0.9 && accelMag < 1.1;
  
  // Gyro bias is just the mean (should be subtracted)
  const gyroBiasX = meanGx;
  const gyroBiasY = meanGy;
  const gyroBiasZ = meanGz;
  
  // Accel offset - we need to account for gravity
  // If device is level: expected is [0, 0, 1] (Z up) or [0, 0, -1] (Z down)
  // We compute offset assuming the dominant axis is gravity
  // For now, assume device orientation and just report the mean
  // The offset is what to subtract to get [0, 0, ±1]
  
  // Simple approach: assume gravity is mostly in Z
  // offset = measured - expected
  const accelOffsetX = meanAx;  // Expected 0
  const accelOffsetY = meanAy;  // Expected 0
  // For Z, we expect ~1.0 (if Z is up) or ~-1.0 (if Z is down)
  // Looking at sample data, az is positive ~0.98, so Z points up
  const accelOffsetZ = meanAz - 1.0;  // Offset from expected 1.0g
  
  return {
    gyroBiasX,
    gyroBiasY,
    gyroBiasZ,
    accelOffsetX,
    accelOffsetY,
    accelOffsetZ,
    sampleCount: n,
    gyroStdDevX: stdGx,
    gyroStdDevY: stdGy,
    gyroStdDevZ: stdGz,
    accelMagnitude: accelMag,
    isStationary
  };
}

/**
 * Analyze IMU data quality
 */
export function analyzeIMUData(samples: IMUData[]): {
  gyroStats: { axis: string; mean: number; stdDev: number; min: number; max: number }[];
  accelStats: { axis: string; mean: number; stdDev: number; min: number; max: number }[];
  noiseLevel: 'low' | 'medium' | 'high';
  recommendations: string[];
} {
  if (samples.length === 0) {
    return {
      gyroStats: [],
      accelStats: [],
      noiseLevel: 'high',
      recommendations: ['No data to analyze']
    };
  }
  
  const calcStats = (values: number[]) => {
    const n = values.length;
    const mean = values.reduce((a, b) => a + b, 0) / n;
    const stdDev = Math.sqrt(values.reduce((a, b) => a + (b - mean) ** 2, 0) / n);
    const min = Math.min(...values);
    const max = Math.max(...values);
    return { mean, stdDev, min, max };
  };
  
  const gxStats = calcStats(samples.map(s => s.wx));
  const gyStats = calcStats(samples.map(s => s.wy));
  const gzStats = calcStats(samples.map(s => s.wz));
  
  const axStats = calcStats(samples.map(s => s.ax));
  const ayStats = calcStats(samples.map(s => s.ay));
  const azStats = calcStats(samples.map(s => s.az));
  
  const gyroStats = [
    { axis: 'X', ...gxStats },
    { axis: 'Y', ...gyStats },
    { axis: 'Z', ...gzStats }
  ];
  
  const accelStats = [
    { axis: 'X', ...axStats },
    { axis: 'Y', ...ayStats },
    { axis: 'Z', ...azStats }
  ];
  
  // Determine noise level based on gyro std dev
  const avgGyroStd = (gxStats.stdDev + gyStats.stdDev + gzStats.stdDev) / 3;
  let noiseLevel: 'low' | 'medium' | 'high';
  if (avgGyroStd < 2) {
    noiseLevel = 'low';
  } else if (avgGyroStd < 10) {
    noiseLevel = 'medium';
  } else {
    noiseLevel = 'high';
  }
  
  // Generate recommendations
  const recommendations: string[] = [];
  
  // Check gyro bias
  if (Math.abs(gxStats.mean) > 1 || Math.abs(gyStats.mean) > 1 || Math.abs(gzStats.mean) > 1) {
    recommendations.push(`Gyro bias detected: X=${gxStats.mean.toFixed(2)}, Y=${gyStats.mean.toFixed(2)}, Z=${gzStats.mean.toFixed(2)} deg/s`);
    recommendations.push('Apply gyro bias calibration for better heading stability');
  }
  
  // Check accel magnitude
  const accelMag = Math.sqrt(axStats.mean ** 2 + ayStats.mean ** 2 + azStats.mean ** 2);
  if (Math.abs(accelMag - 1.0) > 0.05) {
    recommendations.push(`Accel magnitude is ${accelMag.toFixed(3)}g (expected ~1.0g)`);
  }
  
  // Check if device was moving
  if (noiseLevel === 'high') {
    recommendations.push('High motion detected - for calibration, keep device stationary');
  }
  
  if (recommendations.length === 0) {
    recommendations.push('IMU data looks good!');
  }
  
  return { gyroStats, accelStats, noiseLevel, recommendations };
}
