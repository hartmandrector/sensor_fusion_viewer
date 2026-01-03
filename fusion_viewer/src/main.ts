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
import { parseCSV } from './csvParser';
import { OrientationViewer } from './viewer';
import { debug } from './constants';

// State and UI modules
import { state } from './appState';
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
} from './calibrationManager';

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
  
  // Initialize AHRS with default settings
  const calConfig = getInitialCalibrationConfig();
  state.ahrs = new MadgwickAHRS({
    beta: 0.1,
    ...calConfig,
  });
  
  debug.log('FlySight Fusion Viewer initialized');
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
  
  // Filter parameters
  elements.betaSlider.addEventListener('input', handleBetaChange);
  elements.initFromSensors.addEventListener('change', handleInitModeChange);
  elements.useMagnetometer.addEventListener('change', handleUseMagChange);
  elements.showSensorVectors.addEventListener('change', handleShowVectorsChange);
  
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
 * Handle beta slider change
 */
function handleBetaChange(): void {
  const elements = getElements();
  const beta = parseFloat(elements.betaSlider.value);
  elements.betaValue.textContent = beta.toFixed(2);
  
  if (state.ahrs) {
    state.ahrs.setBeta(beta);
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
