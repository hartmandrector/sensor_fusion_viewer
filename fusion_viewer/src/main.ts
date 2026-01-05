/**
 * FlySight 2 Sensor Fusion Viewer - Main Application
 * 
 * This is the main entry point that orchestrates all modules.
 * Core logic is split into:
 *   - appState.ts: Shared application state
 *   - uiElements.ts: DOM element references
 *   - playbackController.ts: Playback and display logic
 *   - calibrationManager.ts: Calibration handling
 */

import { MadgwickAHRS } from './fusion';
import { FusionAhrsAdapter } from './FusionAhrsAdapter';
import { parseCSV } from './csvParser';
import { OrientationViewer } from './viewer';
import { debug } from './constants';

// State and UI modules
import { state, type AlgorithmType } from './appState';
import { getElements, initializeElements } from './uiElements';

// Controller modules
import {
  computeFusionFrames,
  startPlayback,
  pausePlayback,
  resetPlayback,
  handleSliderChange,
  handleSpeedChange,
  updateDisplay,
} from './playbackController';

import {
  initializeCalibrationUI,
  getInitialCalibrationConfig,
  handleMagCalChange,
  handleCalculateCalibration,
  handleShowMagPlot,
  handleIMUCalChange,
  handleCalculateIMUCalibration,
  handleAnalyzeIMU,
  handleAxisRemapChange,
  handleCalculateEllipsoid,
  handleCalculate6PosCalibration,
} from './calibrationManager';

import { initializeCalibrationFileUI } from './calibrationFile';

// ============================================================================
// Initialization
// ============================================================================

/**
 * Initialize the application
 */
function init(): void {
  // Initialize UI elements
  initializeElements();
  
  // Initialize 3D viewer
  state.viewer = new OrientationViewer('threejs-container');
  
  // Set up event listeners
  setupEventListeners();
  
  // Initialize calibration UI with defaults
  initializeCalibrationUI();
  
  // Initialize calibration file save/load
  initializeCalibrationFileUI();
  
  // Initialize both AHRS algorithms
  const calConfig = getInitialCalibrationConfig();
  const elements = getElements();
  
  // Get initial values from UI
  const gain = parseFloat(elements.betaSlider.value);
  const accelReject = parseFloat(elements.accelRejectSlider.value);
  const magReject = parseFloat(elements.magRejectSlider.value);
  
  // Create both algorithms
  state.madgwickAhrs = new MadgwickAHRS({
    beta: gain,
    ...calConfig,
  });
  
  state.fusionAhrs = new FusionAhrsAdapter(
    { beta: gain, ...calConfig },
    { 
      ahrs: { 
        gain: gain,
        accelerationRejection: accelReject,
        magneticRejection: magReject,
        gyroscopeRange: 2000
      } 
    }
  );
  
  // Set active algorithm based on UI
  state.algorithm = elements.algorithmSelect.value as AlgorithmType;
  state.ahrs = state.algorithm === 'fusion' ? state.fusionAhrs : state.madgwickAhrs;
  
  // Show/hide fusion-specific settings
  updateFusionSettingsVisibility();
  
  debug.log('FlySight Fusion Viewer initialized');
  debug.log(`Algorithm: ${state.algorithm}`);
}

// ============================================================================
// Event Listeners
// ============================================================================

/**
 * Set up all event listeners
 */
function setupEventListeners(): void {
  const elements = getElements();
  
  // File input
  elements.csvFile.addEventListener('change', handleFileSelect);
  
  // Playback controls
  elements.playBtn.addEventListener('click', startPlayback);
  elements.pauseBtn.addEventListener('click', pausePlayback);
  elements.resetBtn.addEventListener('click', resetPlayback);
  elements.timeSlider.addEventListener('input', handleSliderChange);
  elements.speedSelect.addEventListener('change', handleSpeedChange);
  
  // Algorithm selection
  elements.algorithmSelect.addEventListener('change', handleAlgorithmChange);
  
  // Filter parameters
  elements.betaSlider.addEventListener('input', handleBetaChange);
  elements.initFromSensors.addEventListener('change', handleInitModeChange);
  elements.useMagnetometer.addEventListener('change', handleUseMagChange);
  elements.showSensorVectors.addEventListener('change', handleShowVectorsChange);
  
  // Fusion Ch.7 specific
  elements.accelRejectSlider.addEventListener('input', handleAccelRejectChange);
  elements.magRejectSlider.addEventListener('input', handleMagRejectChange);
  
  // Axis remapping
  elements.imuRemapX.addEventListener('change', handleAxisRemapChange);
  elements.imuRemapY.addEventListener('change', handleAxisRemapChange);
  elements.imuRemapZ.addEventListener('change', handleAxisRemapChange);
  elements.magRemapX.addEventListener('change', handleAxisRemapChange);
  elements.magRemapY.addEventListener('change', handleAxisRemapChange);
  elements.magRemapZ.addEventListener('change', handleAxisRemapChange);
  
  // Mag calibration
  elements.magOffsetX.addEventListener('change', handleMagCalChange);
  elements.magOffsetY.addEventListener('change', handleMagCalChange);
  elements.magOffsetZ.addEventListener('change', handleMagCalChange);
  elements.calcCalibrationBtn.addEventListener('click', handleCalculateCalibration);
  elements.showMagPlotBtn.addEventListener('click', handleShowMagPlot);
  
  // IMU calibration
  elements.gyroBiasX.addEventListener('change', handleIMUCalChange);
  elements.gyroBiasY.addEventListener('change', handleIMUCalChange);
  elements.gyroBiasZ.addEventListener('change', handleIMUCalChange);
  elements.accelOffsetX.addEventListener('change', handleIMUCalChange);
  elements.accelOffsetY.addEventListener('change', handleIMUCalChange);
  elements.accelOffsetZ.addEventListener('change', handleIMUCalChange);
  elements.calcIMUCalBtn.addEventListener('click', handleCalculateIMUCalibration);
  elements.analyzeIMUBtn.addEventListener('click', handleAnalyzeIMU);
  
  // Advanced calibration
  elements.calcEllipsoidBtn.addEventListener('click', handleCalculateEllipsoid);
  elements.calc6PosCalBtn.addEventListener('click', handleCalculate6PosCalibration);
}

// ============================================================================
// File Handling
// ============================================================================

/**
 * Handle CSV file selection
 */
async function handleFileSelect(event: Event): Promise<void> {
  const input = event.target as HTMLInputElement;
  const file = input.files?.[0];
  
  if (!file) return;
  
  const elements = getElements();
  elements.fileName.textContent = file.name;
  
  try {
    const content = await file.text();
    state.dataset = parseCSV(content);
    
    debug.log(`Loaded ${state.dataset.readings.length} sensor readings`);
    debug.log(`Firmware: ${state.dataset.firmwareVersion}`);
    debug.log(`Duration: ${state.dataset.duration.toFixed(2)}s`);
    debug.log(`IMU: ${state.dataset.imuCount} samples @ ${state.dataset.imuRate} Hz`);
    debug.log(`MAG: ${state.dataset.magCount} samples @ ${state.dataset.magRate} Hz`);
    
    // Update stats display
    elements.imuCount.textContent = state.dataset.imuCount.toString();
    elements.magCount.textContent = state.dataset.magCount.toString();
    elements.duration.textContent = state.dataset.duration.toFixed(2);
    elements.imuRate.textContent = state.dataset.imuRate.toString();
    elements.magRate.textContent = state.dataset.magRate.toString();
    elements.totalTime.textContent = state.dataset.duration.toFixed(3) + 's';
    
    // Pre-compute fusion frames
    computeFusionFrames();
    
    // Enable playback controls
    elements.playBtn.disabled = false;
    elements.pauseBtn.disabled = false;
    elements.resetBtn.disabled = false;
    elements.timeSlider.disabled = false;
    elements.timeSlider.max = '1000';
    
    // Enable calibration buttons
    elements.calcCalibrationBtn.disabled = false;
    elements.showMagPlotBtn.disabled = false;
    elements.calcIMUCalBtn.disabled = false;
    elements.analyzeIMUBtn.disabled = false;
    elements.calcEllipsoidBtn.disabled = false;
    elements.calc6PosCalBtn.disabled = false;
    
    // Reset playback
    resetPlayback();
    
  } catch (error) {
    debug.error('Error loading CSV:', error);
    elements.fileName.textContent = 'Error loading file';
  }
}

// ============================================================================
// Filter Parameter Handlers
// ============================================================================

/**
 * Handle algorithm selection change
 */
function handleAlgorithmChange(): void {
  const elements = getElements();
  state.algorithm = elements.algorithmSelect.value as AlgorithmType;
  
  // Switch active AHRS
  state.ahrs = state.algorithm === 'fusion' ? state.fusionAhrs : state.madgwickAhrs;
  
  // Copy calibration between algorithms
  if (state.fusionAhrs && state.madgwickAhrs) {
    const imuCal = state.algorithm === 'fusion' 
      ? state.madgwickAhrs.getIMUCalibration()
      : state.fusionAhrs.getIMUCalibration();
    const magCal = state.algorithm === 'fusion'
      ? state.madgwickAhrs.getMagCalibration()
      : state.fusionAhrs.getMagCalibration();
    
    state.ahrs?.setIMUCalibration(imuCal);
    state.ahrs?.setMagCalibration(magCal);
    
    // Also copy matrix calibrations (accel scale matrix and soft iron matrix)
    if (state.accelScaleMatrix && state.ahrs?.setAccelScaleMatrix) {
      state.ahrs.setAccelScaleMatrix(state.accelScaleMatrix);
    }
    if (state.softIronMatrix && state.ahrs?.setSoftIronMatrix) {
      state.ahrs.setSoftIronMatrix(state.softIronMatrix);
    }
  }
  
  updateFusionSettingsVisibility();
  
  debug.log(`Switched to algorithm: ${state.algorithm}`);
  
  if (state.dataset) {
    computeFusionFrames();
    updateDisplay(state.playbackIndex);
  }
}

/**
 * Show/hide fusion-specific settings
 */
function updateFusionSettingsVisibility(): void {
  const elements = getElements();
  elements.fusionSettings.style.display = state.algorithm === 'fusion' ? 'block' : 'none';
}

/**
 * Handle acceleration rejection threshold change
 */
function handleAccelRejectChange(): void {
  const elements = getElements();
  const value = parseFloat(elements.accelRejectSlider.value);
  elements.accelRejectValue.textContent = value.toString();
  
  if (state.fusionAhrs) {
    state.fusionAhrs.updateAhrsSettings({ accelerationRejection: value });
    
    if (state.dataset && state.algorithm === 'fusion') {
      computeFusionFrames();
      updateDisplay(state.playbackIndex);
    }
  }
}

/**
 * Handle magnetic rejection threshold change
 */
function handleMagRejectChange(): void {
  const elements = getElements();
  const value = parseFloat(elements.magRejectSlider.value);
  elements.magRejectValue.textContent = value.toString();
  
  if (state.fusionAhrs) {
    state.fusionAhrs.updateAhrsSettings({ magneticRejection: value });
    
    if (state.dataset && state.algorithm === 'fusion') {
      computeFusionFrames();
      updateDisplay(state.playbackIndex);
    }
  }
}

/**
 * Handle beta slider change
 */
function handleBetaChange(): void {
  const elements = getElements();
  const beta = parseFloat(elements.betaSlider.value);
  elements.betaValue.textContent = beta.toFixed(2);
  
  // Update both algorithms
  if (state.madgwickAhrs) {
    state.madgwickAhrs.setBeta(beta);
  }
  if (state.fusionAhrs) {
    state.fusionAhrs.updateAhrsSettings({ gain: beta });
  }
  
  if (state.dataset) {
    computeFusionFrames();
    updateDisplay(state.playbackIndex);
  }
}

/**
 * Handle init mode change
 */
function handleInitModeChange(): void {
  if (state.dataset) {
    computeFusionFrames();
    updateDisplay(state.playbackIndex);
  }
}

/**
 * Handle magnetometer enable/disable
 */
function handleUseMagChange(): void {
  if (state.dataset) {
    computeFusionFrames();
    updateDisplay(state.playbackIndex);
  }
}

/**
 * Handle show sensor vectors toggle
 */
function handleShowVectorsChange(): void {
  const elements = getElements();
  if (state.viewer) {
    state.viewer.toggleSensorVectors(elements.showSensorVectors.checked);
    updateDisplay(state.playbackIndex);
  }
}

// ============================================================================
// Application Entry Point
// ============================================================================

// Initialize on page load
document.addEventListener('DOMContentLoaded', init);
