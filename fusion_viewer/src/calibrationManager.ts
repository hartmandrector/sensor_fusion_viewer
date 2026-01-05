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
import { fitEllipsoid, formatSoftIronMatrix } from './ellipsoidFit';
import { calibrate6Position, detectStationarySegments, selectBestSegments, formatOrientationStatus } from './accelCalibration6Pos';
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
// Ellipsoid Magnetometer Calibration
// ============================================================================

/**
 * Calculate full ellipsoid (9-parameter) magnetometer calibration
 */
export function handleCalculateEllipsoid(): void {
  if (!state.dataset) return;
  
  const elements = getElements();
  const magSamples = getMAGReadings(state.dataset);
  
  if (magSamples.length < 100) {
    elements.calibrationResult.innerHTML = '<div class="error">Need at least 100 mag samples for ellipsoid fit</div>';
    elements.calibrationResult.classList.add('visible');
    return;
  }
  
  // Extract magnetometer xyz
  const points = magSamples.map(s => ({ x: s.x, y: s.y, z: s.z }));
  
  try {
    const result = fitEllipsoid(points);
    
    // Store the ellipsoid result for later use
    state.lastEllipsoidCalibration = result;
    
    // Build result HTML
    let html = `
      <div class="cal-label">Hard Iron Offset:</div>
      <div>X: <span class="cal-value">${result.hardIronOffset.x.toFixed(4)}</span></div>
      <div>Y: <span class="cal-value">${result.hardIronOffset.y.toFixed(4)}</span></div>
      <div>Z: <span class="cal-value">${result.hardIronOffset.z.toFixed(4)}</span></div>
      <div style="margin-top: 0.5rem;">
        <span class="cal-label">Sphericity:</span> 
        <span class="quality-${result.sphericity > 0.9 ? 'good' : result.sphericity > 0.7 ? 'fair' : 'poor'}">${(result.sphericity * 100).toFixed(1)}%</span>
      </div>
      <div><span class="cal-label">Residual RMS:</span> <span class="cal-value">${result.residualRms.toFixed(4)}</span></div>
    `;
    
    // Display soft iron matrix
    elements.softIronMatrixDisplay.innerHTML = formatSoftIronMatrix(result.softIronMatrix);
    
    html += `<button class="apply-btn" onclick="window.applyEllipsoidCalibration()">Apply Calibration</button>`;
    
    elements.calibrationResult.innerHTML = html;
    elements.calibrationResult.classList.add('visible');
    
    debug.log('Ellipsoid Calibration:', result);
    
  } catch (error) {
    elements.calibrationResult.innerHTML = `<div class="error">Ellipsoid fit failed: ${error}</div>`;
    elements.calibrationResult.classList.add('visible');
    debug.error('Ellipsoid fit failed:', error);
  }
}

/**
 * Apply ellipsoid calibration (hard iron + soft iron)
 */
export function applyEllipsoidCalibration(): void {
  if (!state.lastEllipsoidCalibration) return;
  
  const elements = getElements();
  elements.magOffsetX.value = state.lastEllipsoidCalibration.hardIronOffset.x.toFixed(4);
  elements.magOffsetY.value = state.lastEllipsoidCalibration.hardIronOffset.y.toFixed(4);
  elements.magOffsetZ.value = state.lastEllipsoidCalibration.hardIronOffset.z.toFixed(4);
  
  // Store soft iron matrix in state
  state.softIronMatrix = state.lastEllipsoidCalibration.softIronInverse;
  
  // Apply soft iron matrix to BOTH AHRS instances (so it persists when switching algorithms)
  if (state.fusionAhrs?.setSoftIronMatrix) {
    state.fusionAhrs.setSoftIronMatrix(state.lastEllipsoidCalibration.softIronInverse);
  }
  if (state.madgwickAhrs?.setSoftIronMatrix) {
    state.madgwickAhrs.setSoftIronMatrix(state.lastEllipsoidCalibration.softIronInverse);
  }
  debug.log('Applied soft iron matrix to both AHRS:', state.lastEllipsoidCalibration.softIronInverse);
  
  // Also apply hard iron via normal mag cal change
  handleMagCalChange();
  
  debug.log('Applied ellipsoid calibration (hard iron + soft iron)');
}

// ============================================================================
// 6-Position Accelerometer Calibration
// ============================================================================

/**
 * Calculate 6-position accelerometer calibration
 */
export function handleCalculate6PosCalibration(): void {
  if (!state.dataset) return;
  
  const elements = getElements();
  const imuSamples = getIMUReadings(state.dataset);
  
  if (imuSamples.length < 600) {
    elements.sixPosStatus.innerHTML = '<div class="error">Need at least 600 IMU samples (100 per position)</div>';
    elements.sixPosStatus.classList.add('visible');
    return;
  }
  
  // Convert IMU data to separate accel and gyro arrays for segment detection
  const accelData = imuSamples.map(s => ({
    timestamp: s.timestamp,
    x: s.ax,
    y: s.ay,
    z: s.az
  }));
  
  const gyroData = imuSamples.map(s => ({
    timestamp: s.timestamp,
    x: s.wx,
    y: s.wy,
    z: s.wz
  }));
  
  // Detect stationary segments using gyro threshold
  const segments = detectStationarySegments(accelData, gyroData);
  
  debug.log(`Detected ${segments.length} stationary segments`);
  console.log('6-Pos Cal: Detected segments:', segments);
  
  // Select best segment for each orientation
  const bestSegments = selectBestSegments(segments);
  
  console.log('6-Pos Cal: Best segments:', [...bestSegments.entries()]);
  
  // Display orientation status
  elements.orientationStatus.innerHTML = formatOrientationStatus(bestSegments);
  elements.orientationStatus.classList.add('visible');
  
  // Check if we have all 6 positions
  const validOrientations = bestSegments.size;
  
  if (validOrientations < 6) {
    elements.sixPosStatus.innerHTML = `
      <div class="warning">
        Only ${validOrientations}/6 positions detected.<br>
        Place device in each orientation (±10° tolerance) and keep stationary for 0.5s.
      </div>`;
    elements.sixPosStatus.classList.add('visible');
    return;
  }
  
  // All 6 positions found - run calibration
  try {
    const result = calibrate6Position(bestSegments);
    
    // Store result
    state.lastAccel6PosCalibration = result;
    
    // Build result HTML
    let html = '<div class="calibration-values">';
    html += `<strong>6-Position Accel Calibration</strong><br><br>`;
    
    html += `<strong>Bias (g):</strong><br>`;
    html += `X: ${result.bias.x.toFixed(6)}<br>`;
    html += `Y: ${result.bias.y.toFixed(6)}<br>`;
    html += `Z: ${result.bias.z.toFixed(6)}<br><br>`;
    
    html += `<strong>Cross-Axis Coupling:</strong><br>`;
    const crossAxis = result.crossAxis;
    html += `XY: ${(crossAxis.xy * 100).toFixed(3)}%<br>`;
    html += `XZ: ${(crossAxis.xz * 100).toFixed(3)}%<br>`;
    html += `YZ: ${(crossAxis.yz * 100).toFixed(3)}%<br><br>`;
    
    html += `<strong>Residual RMS:</strong> ${result.residualRms.toFixed(6)} g<br>`;
    
    html += `</div>`;
    html += `<button class="apply-btn" onclick="window.apply6PosCalibration()">Apply Bias Only</button>`;
    html += `<button class="apply-btn" onclick="window.apply6PosFullCalibration()" style="margin-left: 0.5rem;">Apply Full Matrix</button>`;
    
    elements.sixPosStatus.innerHTML = html;
    elements.sixPosStatus.classList.add('visible');
    
    // Display scale matrix
    const S = result.scaleMatrix;
    let matrixHtml = '<table class="matrix-table">';
    matrixHtml += `<tr><td>${S[0][0].toFixed(6)}</td><td>${S[0][1].toFixed(6)}</td><td>${S[0][2].toFixed(6)}</td></tr>`;
    matrixHtml += `<tr><td>${S[1][0].toFixed(6)}</td><td>${S[1][1].toFixed(6)}</td><td>${S[1][2].toFixed(6)}</td></tr>`;
    matrixHtml += `<tr><td>${S[2][0].toFixed(6)}</td><td>${S[2][1].toFixed(6)}</td><td>${S[2][2].toFixed(6)}</td></tr>`;
    matrixHtml += '</table>';
    elements.accelScaleMatrixDisplay.innerHTML = matrixHtml;
    
    debug.log('6-Position Calibration:', result);
    
  } catch (error) {
    elements.sixPosStatus.innerHTML = `<div class="error">Calibration failed: ${error}</div>`;
    elements.sixPosStatus.classList.add('visible');
    debug.error('6-Position calibration failed:', error);
  }
}

/**
 * Apply 6-position calibration (bias only for simple UI)
 */
export function apply6PosCalibration(): void {
  if (!state.lastAccel6PosCalibration) return;
  
  const elements = getElements();
  elements.accelOffsetX.value = state.lastAccel6PosCalibration.bias.x.toFixed(6);
  elements.accelOffsetY.value = state.lastAccel6PosCalibration.bias.y.toFixed(6);
  elements.accelOffsetZ.value = state.lastAccel6PosCalibration.bias.z.toFixed(6);
  
  handleIMUCalChange();
  
  debug.log('Applied 6-pos calibration (bias only)');
}

/**
 * Apply 6-position calibration with full scale matrix
 */
export function apply6PosFullCalibration(): void {
  if (!state.lastAccel6PosCalibration) return;
  
  const elements = getElements();
  
  // Apply bias to UI fields
  elements.accelOffsetX.value = state.lastAccel6PosCalibration.bias.x.toFixed(6);
  elements.accelOffsetY.value = state.lastAccel6PosCalibration.bias.y.toFixed(6);
  elements.accelOffsetZ.value = state.lastAccel6PosCalibration.bias.z.toFixed(6);
  
  // Store the full calibration in state for both AHRS implementations
  state.accelScaleMatrix = state.lastAccel6PosCalibration.scaleMatrixInverse;
  state.accelBias = [
    state.lastAccel6PosCalibration.bias.x,
    state.lastAccel6PosCalibration.bias.y,
    state.lastAccel6PosCalibration.bias.z
  ];
  
  // Apply scale matrix to BOTH AHRS instances (so it persists when switching algorithms)
  if (state.fusionAhrs?.setAccelScaleMatrix) {
    state.fusionAhrs.setAccelScaleMatrix(state.lastAccel6PosCalibration.scaleMatrixInverse);
  }
  if (state.madgwickAhrs?.setAccelScaleMatrix) {
    state.madgwickAhrs.setAccelScaleMatrix(state.lastAccel6PosCalibration.scaleMatrixInverse);
  }
  
  handleIMUCalChange();
  
  debug.log('Applied 6-pos full calibration to both AHRS with scale matrix:', state.lastAccel6PosCalibration.scaleMatrixInverse);
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
(window as unknown as { applyEllipsoidCalibration: () => void }).applyEllipsoidCalibration = applyEllipsoidCalibration;
(window as unknown as { apply6PosCalibration: () => void }).apply6PosCalibration = apply6PosCalibration;
(window as unknown as { apply6PosFullCalibration: () => void }).apply6PosFullCalibration = apply6PosFullCalibration;
