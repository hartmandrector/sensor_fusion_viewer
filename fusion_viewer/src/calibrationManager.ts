/**
 * Calibration Manager Module
 * 
 * Handles magnetometer and IMU calibration UI and calculations.
 */

import { state, DEFAULT_CALIBRATION } from './appState';
import { getElements } from './uiElements';
import { computeFusionFrames, updateDisplay } from './playbackController';
import { getMAGReadings, getIMUReadings } from './csvParser';
import { calculateHardIronCalibration, evaluateCalibrationQuality } from './magCalibration';
import { calculateIMUCalibration, analyzeIMUData } from './imuCalibration';
import type { MagCalibration, IMUCalibration, AxisRemap } from './types';
import { debug } from './constants';

// ============================================================================
// Initialization
// ============================================================================

/**
 * Initialize calibration UI with default values
 */
export function initializeCalibrationUI(): void {
  const elements = getElements();
  
  // Magnetometer offsets
  elements.magOffsetX.value = DEFAULT_CALIBRATION.mag.offsetX.toString();
  elements.magOffsetY.value = DEFAULT_CALIBRATION.mag.offsetY.toString();
  elements.magOffsetZ.value = DEFAULT_CALIBRATION.mag.offsetZ.toString();
  
  // Gyro bias
  elements.gyroBiasX.value = DEFAULT_CALIBRATION.gyro.biasX.toString();
  elements.gyroBiasY.value = DEFAULT_CALIBRATION.gyro.biasY.toString();
  elements.gyroBiasZ.value = DEFAULT_CALIBRATION.gyro.biasZ.toString();
  
  // Accel offset
  elements.accelOffsetX.value = DEFAULT_CALIBRATION.accel.offsetX.toString();
  elements.accelOffsetY.value = DEFAULT_CALIBRATION.accel.offsetY.toString();
  elements.accelOffsetZ.value = DEFAULT_CALIBRATION.accel.offsetZ.toString();
}

/**
 * Get initial AHRS calibration config from defaults
 */
export function getInitialCalibrationConfig() {
  return {
    magCalibration: {
      offsetX: DEFAULT_CALIBRATION.mag.offsetX,
      offsetY: DEFAULT_CALIBRATION.mag.offsetY,
      offsetZ: DEFAULT_CALIBRATION.mag.offsetZ,
      scaleX: DEFAULT_CALIBRATION.mag.scaleX,
      scaleY: DEFAULT_CALIBRATION.mag.scaleY,
      scaleZ: DEFAULT_CALIBRATION.mag.scaleZ,
    },
    imuCalibration: {
      gyroBiasX: DEFAULT_CALIBRATION.gyro.biasX,
      gyroBiasY: DEFAULT_CALIBRATION.gyro.biasY,
      gyroBiasZ: DEFAULT_CALIBRATION.gyro.biasZ,
      accelOffsetX: DEFAULT_CALIBRATION.accel.offsetX,
      accelOffsetY: DEFAULT_CALIBRATION.accel.offsetY,
      accelOffsetZ: DEFAULT_CALIBRATION.accel.offsetZ,
    },
  };
}

// ============================================================================
// Magnetometer Calibration
// ============================================================================

/**
 * Handle magnetometer calibration input change
 */
export function handleMagCalChange(): void {
  if (!state.ahrs) return;
  
  const elements = getElements();
  const cal: MagCalibration = {
    offsetX: parseFloat(elements.magOffsetX.value) || 0,
    offsetY: parseFloat(elements.magOffsetY.value) || 0,
    offsetZ: parseFloat(elements.magOffsetZ.value) || 0,
    scaleX: 1,
    scaleY: 1,
    scaleZ: 1
  };
  
  state.ahrs.setMagCalibration(cal);
  computeFusionFrames();
  updateDisplay(state.playbackIndex);
}

/**
 * Calculate magnetometer calibration from loaded data
 */
export function handleCalculateCalibration(): void {
  if (!state.dataset) return;
  
  const elements = getElements();
  const magSamples = getMAGReadings(state.dataset);
  
  // Calculate calibration on raw mag data
  state.lastMagCalibration = calculateHardIronCalibration(magSamples, false);
  const quality = evaluateCalibrationQuality(state.lastMagCalibration);
  
  // Build result HTML
  let html = `
    <div class="cal-label">Calculated Offsets:</div>
    <div>X: <span class="cal-value">${state.lastMagCalibration.offsetX.toFixed(4)}</span></div>
    <div>Y: <span class="cal-value">${state.lastMagCalibration.offsetY.toFixed(4)}</span></div>
    <div>Z: <span class="cal-value">${state.lastMagCalibration.offsetZ.toFixed(4)}</span></div>
    <div style="margin-top: 0.5rem;">
      <span class="cal-label">Quality:</span> 
      <span class="quality-${quality.quality}">${quality.quality.toUpperCase()}</span>
    </div>
    <div><span class="cal-label">Sphericity:</span> <span class="cal-value">${(state.lastMagCalibration.sphericity * 100).toFixed(0)}%</span></div>
    <div><span class="cal-label">Field Magnitude:</span> <span class="cal-value">${state.lastMagCalibration.magnitude.toFixed(3)} gauss</span></div>
    <div><span class="cal-label">Samples:</span> <span class="cal-value">${state.lastMagCalibration.sampleCount}</span></div>
  `;
  
  if (quality.issues.length > 0) {
    html += `<div style="margin-top: 0.5rem; color: #fbbf24; font-size: 0.75rem;">`;
    for (const issue of quality.issues) {
      html += `⚠ ${issue}<br>`;
    }
    html += `</div>`;
  }
  
  html += `<button class="apply-btn" onclick="window.applyCalibration()">Apply Calibration</button>`;
  
  elements.calibrationResult.innerHTML = html;
  elements.calibrationResult.classList.add('visible');
  
  debug.log('Calibration calculated:', state.lastMagCalibration);
  debug.log('Quality:', quality);
}

/**
 * Apply the calculated calibration to the offset fields
 */
export function applyCalibration(): void {
  if (!state.lastMagCalibration) return;
  
  const elements = getElements();
  elements.magOffsetX.value = state.lastMagCalibration.offsetX.toFixed(4);
  elements.magOffsetY.value = state.lastMagCalibration.offsetY.toFixed(4);
  elements.magOffsetZ.value = state.lastMagCalibration.offsetZ.toFixed(4);
  
  handleMagCalChange();
}

/**
 * Show 3D scatter plot of magnetometer data
 */
export function handleShowMagPlot(): void {
  if (!state.dataset || !state.viewer) return;
  
  const elements = getElements();
  const magSamples = getMAGReadings(state.dataset);
  
  const offsetX = parseFloat(elements.magOffsetX.value) || 0;
  const offsetY = parseFloat(elements.magOffsetY.value) || 0;
  const offsetZ = parseFloat(elements.magOffsetZ.value) || 0;
  
  state.viewer.toggleMagPlot(magSamples, { offsetX, offsetY, offsetZ });
}

// ============================================================================
// IMU Calibration
// ============================================================================

/**
 * Handle IMU calibration input change
 */
export function handleIMUCalChange(): void {
  if (!state.ahrs) return;
  
  const elements = getElements();
  const imuCal: IMUCalibration = {
    gyroBiasX: parseFloat(elements.gyroBiasX.value) || 0,
    gyroBiasY: parseFloat(elements.gyroBiasY.value) || 0,
    gyroBiasZ: parseFloat(elements.gyroBiasZ.value) || 0,
    accelOffsetX: parseFloat(elements.accelOffsetX.value) || 0,
    accelOffsetY: parseFloat(elements.accelOffsetY.value) || 0,
    accelOffsetZ: parseFloat(elements.accelOffsetZ.value) || 0
  };
  
  state.ahrs.setIMUCalibration(imuCal);
  
  if (state.dataset) {
    computeFusionFrames();
    updateDisplay(state.playbackIndex);
  }
}

/**
 * Calculate IMU calibration from stationary data
 */
export function handleCalculateIMUCalibration(): void {
  if (!state.dataset) return;
  
  const elements = getElements();
  const imuSamples = getIMUReadings(state.dataset);
  
  if (imuSamples.length === 0) {
    elements.imuCalibrationResult.innerHTML = '<div class="error">No IMU data found</div>';
    elements.imuCalibrationResult.classList.add('visible');
    return;
  }
  
  state.lastIMUCalibration = calculateIMUCalibration(imuSamples);
  
  let html = '<div class="calibration-values">';
  html += `<strong>IMU Calibration Results</strong><br>`;
  html += `<strong>Gyro Bias (deg/s):</strong><br>`;
  html += `X: ${state.lastIMUCalibration.gyroBiasX.toFixed(4)}<br>`;
  html += `Y: ${state.lastIMUCalibration.gyroBiasY.toFixed(4)}<br>`;
  html += `Z: ${state.lastIMUCalibration.gyroBiasZ.toFixed(4)}<br><br>`;
  html += `<strong>Accel Offset (g):</strong><br>`;
  html += `X: ${state.lastIMUCalibration.accelOffsetX.toFixed(4)}<br>`;
  html += `Y: ${state.lastIMUCalibration.accelOffsetY.toFixed(4)}<br>`;
  html += `Z: ${state.lastIMUCalibration.accelOffsetZ.toFixed(4)}<br><br>`;
  html += `<strong>Statistics:</strong><br>`;
  html += `Samples: ${state.lastIMUCalibration.sampleCount}<br>`;
  html += `Accel Magnitude: ${state.lastIMUCalibration.accelMagnitude.toFixed(4)}g<br>`;
  html += `Gyro StdDev: X=${state.lastIMUCalibration.gyroStdDevX.toFixed(2)}, Y=${state.lastIMUCalibration.gyroStdDevY.toFixed(2)}, Z=${state.lastIMUCalibration.gyroStdDevZ.toFixed(2)}<br>`;
  
  if (!state.lastIMUCalibration.isStationary) {
    html += `<div class="warning">⚠ Device may not have been stationary during data collection. High gyro variance detected.</div>`;
  }
  
  html += `</div>`;
  html += `<button class="apply-btn" onclick="window.applyIMUCalibration()">Apply IMU Calibration</button>`;
  
  elements.imuCalibrationResult.innerHTML = html;
  elements.imuCalibrationResult.classList.add('visible');
  
  debug.log('IMU Calibration calculated:', state.lastIMUCalibration);
}

/**
 * Apply the calculated IMU calibration
 */
export function applyIMUCalibration(): void {
  if (!state.lastIMUCalibration) return;
  
  const elements = getElements();
  elements.gyroBiasX.value = state.lastIMUCalibration.gyroBiasX.toFixed(4);
  elements.gyroBiasY.value = state.lastIMUCalibration.gyroBiasY.toFixed(4);
  elements.gyroBiasZ.value = state.lastIMUCalibration.gyroBiasZ.toFixed(4);
  elements.accelOffsetX.value = state.lastIMUCalibration.accelOffsetX.toFixed(4);
  elements.accelOffsetY.value = state.lastIMUCalibration.accelOffsetY.toFixed(4);
  elements.accelOffsetZ.value = state.lastIMUCalibration.accelOffsetZ.toFixed(4);
  
  handleIMUCalChange();
}

/**
 * Analyze IMU data quality
 */
export function handleAnalyzeIMU(): void {
  if (!state.dataset) return;
  
  const elements = getElements();
  const imuSamples = getIMUReadings(state.dataset);
  
  if (imuSamples.length === 0) {
    elements.imuCalibrationResult.innerHTML = '<div class="error">No IMU data found</div>';
    elements.imuCalibrationResult.classList.add('visible');
    return;
  }
  
  const analysis = analyzeIMUData(imuSamples);
  
  let html = '<div class="calibration-values">';
  html += `<strong>IMU Data Analysis</strong><br><br>`;
  html += `<strong>Noise Level:</strong> ${analysis.noiseLevel.toUpperCase()}<br><br>`;
  
  html += `<strong>Gyroscope Statistics (deg/s):</strong><br>`;
  html += `<table class="stats-table">`;
  html += `<tr><th>Axis</th><th>Mean</th><th>StdDev</th><th>Min</th><th>Max</th></tr>`;
  for (const stat of analysis.gyroStats) {
    html += `<tr><td>${stat.axis}</td><td>${stat.mean.toFixed(3)}</td><td>${stat.stdDev.toFixed(3)}</td><td>${stat.min.toFixed(2)}</td><td>${stat.max.toFixed(2)}</td></tr>`;
  }
  html += `</table><br>`;
  
  html += `<strong>Accelerometer Statistics (g):</strong><br>`;
  html += `<table class="stats-table">`;
  html += `<tr><th>Axis</th><th>Mean</th><th>StdDev</th><th>Min</th><th>Max</th></tr>`;
  for (const stat of analysis.accelStats) {
    html += `<tr><td>${stat.axis}</td><td>${stat.mean.toFixed(4)}</td><td>${stat.stdDev.toFixed(4)}</td><td>${stat.min.toFixed(3)}</td><td>${stat.max.toFixed(3)}</td></tr>`;
  }
  html += `</table><br>`;
  
  html += `<strong>Recommendations:</strong><br>`;
  for (const rec of analysis.recommendations) {
    html += `• ${rec}<br>`;
  }
  
  html += `</div>`;
  
  elements.imuCalibrationResult.innerHTML = html;
  elements.imuCalibrationResult.classList.add('visible');
  
  debug.log('IMU Analysis:', analysis);
}

// ============================================================================
// Axis Remapping
// ============================================================================

/**
 * Handle axis remap change
 */
export function handleAxisRemapChange(): void {
  if (!state.ahrs) return;
  
  const elements = getElements();
  
  const imuRemap: AxisRemap = {
    bodyX: elements.imuRemapX.value as AxisRemap['bodyX'],
    bodyY: elements.imuRemapY.value as AxisRemap['bodyY'],
    bodyZ: elements.imuRemapZ.value as AxisRemap['bodyZ']
  };
  
  const magRemap: AxisRemap = {
    bodyX: elements.magRemapX.value as AxisRemap['bodyX'],
    bodyY: elements.magRemapY.value as AxisRemap['bodyY'],
    bodyZ: elements.magRemapZ.value as AxisRemap['bodyZ']
  };
  
  // Optional methods - check if they exist before calling
  if (state.ahrs?.setIMUAxisRemap) {
    state.ahrs.setIMUAxisRemap(imuRemap);
  }
  if (state.ahrs?.setMagAxisRemap) {
    state.ahrs.setMagAxisRemap(magRemap);
  }
  
  if (state.dataset) {
    computeFusionFrames();
    updateDisplay(state.playbackIndex);
  }
  
  debug.log('IMU axis remap:', imuRemap);
  debug.log('MAG axis remap:', magRemap);
}

// ============================================================================
// Window Exports
// ============================================================================

// Expose functions to window for button onclick handlers
(window as unknown as { applyCalibration: () => void }).applyCalibration = applyCalibration;
(window as unknown as { applyIMUCalibration: () => void }).applyIMUCalibration = applyIMUCalibration;
