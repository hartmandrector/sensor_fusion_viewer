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
  handleOrientationSourceChange,
  updateOrientationSourceAvailability,
} from './playbackController';

import {
  initializeCalibrationUI,
  getInitialCalibrationConfig,
  handleMagCalChange,
  handleCalculateCalibration,
  handleShowMagPlot,
  handleShowIMUPlot,
  handleIMUCalChange,
  handleCalculateIMUCalibration,
  handleAnalyzeIMU,
  handleAxisRemapChange,
  handleCalculateEllipsoid,
  handleCalculate6PosCalibration,
} from './calibrationManager';

import { initializeCalibrationExecutive } from './calibrationExecutive';

import { handleExportFusedData } from './fusedDataExport';

import { computeIntegration, ZERO_VELOCITY, type Vec3NWU } from './accelerationIntegration';
import { getGpsVelocityNwuAt } from './gpsIntegration';
import {
  initializeCharts, updateComponentChart, destroyCharts,
  showAttitudeDiagnostics, showIntegrationCharts
} from './integrationCharts';
import {
  updateDiagnosticsComponent, resetDiagnosticsZoom,
  applyDiagnosticTimeShift, estimateDiagnosticLag
} from './attitudeDiagnosticsCharts';

// GPS Integration
import { initializeGPSUI, onSensorFileLoaded, onIntegrationStartTimeChange } from './gpsUIHandler';

// GPS-aided ESKF
import { framesUseEskf, onEskfDatasetLoaded, requestEskfRun, updateEskfSeedVisibility } from './eskfController';
import { destroyEskfDiagnostics, resetEskfDiagnosticsZoom, showEskfDiagnostics } from './eskfDiagnosticsCharts';

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
  
  // Initialize calibration executive (handles save/load and status tracking)
  initializeCalibrationExecutive();
  
  // Initialize GPS UI
  initializeGPSUI();
  
  // Initialize both AHRS algorithms
  const calConfig = getInitialCalibrationConfig();
  const elements = getElements();
  
  // Get initial values from UI
  const gain = parseFloat(elements.betaSlider.value);
  const accelReject = parseFloat(elements.accelRejectSlider.value);
  const magReject = parseFloat(elements.magRejectSlider.value);
  const recoveryTime = parseFloat(elements.recoveryTimeSlider.value);
  
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
      },
      recoveryTimeSeconds: recoveryTime
    }
  );
  
  // Set active algorithm based on UI
  state.algorithm = elements.algorithmSelect.value as AlgorithmType;
  state.ahrs = state.algorithm === 'madgwick' ? state.madgwickAhrs : state.fusionAhrs;
  
  // Show/hide fusion-specific settings
  updateFusionSettingsVisibility();
  
  // Apply initial checkbox states to viewer
  state.viewer.toggleSensorVectors(elements.showSensorVectors.checked);
  state.viewer.toggleLinearAccel(elements.showLinearAccel.checked);
  state.viewer.toggleEarthAccel(elements.showEarthAccel.checked);
  state.viewer.toggleGravity(elements.showGravity.checked);
  state.viewer.toggleHeading(elements.showHeading.checked);
  state.viewer.toggleCompassHeading(elements.showCompassHeading.checked);
  
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
  elements.orientationSource.addEventListener('change', handleOrientationSourceChange);
  
  // Filter parameters
  elements.betaSlider.addEventListener('input', handleBetaChange);
  elements.initFromSensors.addEventListener('change', handleInitModeChange);
  elements.useMagnetometer.addEventListener('change', handleUseMagChange);
  elements.showSensorVectors.addEventListener('change', handleShowVectorsChange);
  elements.showLinearAccel.addEventListener('change', handleShowLinearAccelChange);
  elements.showEarthAccel.addEventListener('change', handleShowEarthAccelChange);
  elements.showVelocity.addEventListener('change', () => {
    if (state.viewer) updateDisplay(state.playbackIndex);
  });
  elements.showGravity.addEventListener('change', handleShowGravityChange);
  elements.showHeading.addEventListener('change', handleShowHeadingChange);
  elements.showCompassHeading.addEventListener('change', handleShowCompassHeadingChange);
  
  // Fusion Ch.7 specific
  elements.accelRejectSlider.addEventListener('input', handleAccelRejectChange);
  elements.magRejectSlider.addEventListener('input', handleMagRejectChange);
  elements.recoveryTimeSlider.addEventListener('input', handleRecoveryTimeChange);
  
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
  elements.showIMUPlotBtn.addEventListener('click', handleShowIMUPlot);
  
  // Advanced calibration
  elements.calcEllipsoidBtn.addEventListener('click', handleCalculateEllipsoid);
  elements.calc6PosCalBtn.addEventListener('click', handleCalculate6PosCalibration);
  
  // Export
  elements.exportFusedDataBtn.addEventListener('click', handleExportFusedData);
  
  // Acceleration Integration
  elements.integrationStartSlider.addEventListener('input', handleIntegrationStartChange);
  elements.integrationInitVelocity.addEventListener('change', invalidateIntegration);
  elements.calculateIntegrationBtn.addEventListener('click', handleCalculateIntegration);
  window.addEventListener('integration-inputs-changed', invalidateIntegration);
  elements.showChartsBtn.addEventListener('click', handleShowCharts);
  elements.backToViewerBtn.addEventListener('click', handleBackToViewer);
  elements.showDiagnosticsBtn.addEventListener('click', showAttitudeDiagnostics);
  elements.backToIntegrationBtn.addEventListener('click', showIntegrationCharts);
  elements.diagnosticsComponent.addEventListener('change', updateDiagnosticsComponent);
  elements.resetDiagnosticsZoomBtn.addEventListener('click', resetDiagnosticsZoom);
  elements.applyDiagnosticsShift.addEventListener('click', applyDiagnosticTimeShift);
  elements.estimateLagBtn.addEventListener('click', estimateDiagnosticLag);
  elements.componentSelect.addEventListener('change', handleComponentSelectChange);

  // GPS-aided ESKF
  for (const input of [
    elements.eskfSeed, elements.eskfSeedHeading, elements.eskfHeadingSigma, elements.eskfGyroScale,
    elements.eskfGpsShift, elements.eskfStartTime, elements.eskfNisGate, elements.eskfSAccScale, elements.eskfReinit,
  ]) {
    input.addEventListener('change', () => {
      updateEskfSeedVisibility();
      recomputeEskf();
    });
  }
  elements.eskfRunBtn.addEventListener('click', () => recomputeEskf(true));
  elements.eskfChartsBtn.addEventListener('click', handleShowEskfCharts);
  elements.resetEskfZoomBtn.addEventListener('click', resetEskfDiagnosticsZoom);
  window.addEventListener('gps-data-loaded', () => {
    recomputeEskf();
    if (state.viewer && state.dataset) updateDisplay(state.playbackIndex);
  });
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
    state.currentFileName = file.name;
    elements.diagnosticsGpsShift.value = '0';
    elements.lagStart.value = '0';
    elements.lagEnd.value = Math.min(60, state.dataset.duration).toFixed(1);
    updateOrientationSourceAvailability();
    
    // Store raw CSV for GPS timestamp sync
    onSensorFileLoaded(content);
    onEskfDatasetLoaded();
    
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
    elements.maxGyroMagnitude.textContent = state.dataset.maxGyroMagnitude.toFixed(1);
    elements.maxAccelMagnitude.textContent = state.dataset.maxAccelMagnitude.toFixed(2);
    elements.totalTime.textContent = state.dataset.duration.toFixed(3) + 's';
    
    // Update recovery trigger period based on actual IMU rate
    const recoveryTimeSeconds = parseFloat(elements.recoveryTimeSlider.value);
    const recoveryTriggerPeriod = Math.round(state.dataset.imuRate * recoveryTimeSeconds);
    debug.log(`Recovery trigger: ${recoveryTimeSeconds}s @ ${state.dataset.imuRate}Hz = ${recoveryTriggerPeriod} samples`);
    if (state.fusionAhrs) {
      state.fusionAhrs.updateAhrsSettings({ recoveryTriggerPeriod });
    }
    
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
    elements.showIMUPlotBtn.disabled = false;
    elements.calcEllipsoidBtn.disabled = false;
    elements.calc6PosCalBtn.disabled = false;
    
    // Enable export button
    elements.exportFusedDataBtn.disabled = false;
    
    // Enable integration controls
    // Slider uses relative time (0 to duration), but we store the absolute start time as a data attribute
    elements.integrationStartSlider.disabled = false;
    elements.integrationStartSlider.min = '0';
    elements.integrationStartSlider.max = state.dataset.duration.toFixed(3);
    elements.integrationStartSlider.value = '0';
    elements.integrationStartSlider.dataset.startTime = state.dataset.startTime.toFixed(6);
    elements.integrationStartTime.textContent = '0.000s';
    elements.calculateIntegrationBtn.disabled = false;
    
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
  const usesFusion = state.algorithm !== 'madgwick';
  
  // Switch active AHRS (the ESKF reuses the Fusion AHRS calibration and output)
  state.ahrs = usesFusion ? state.fusionAhrs : state.madgwickAhrs;
  
  // Copy calibration between algorithms
  if (state.fusionAhrs && state.madgwickAhrs) {
    const imuCal = usesFusion 
      ? state.madgwickAhrs.getIMUCalibration()
      : state.fusionAhrs.getIMUCalibration();
    const magCal = usesFusion
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
  elements.eskfSettings.style.display = state.algorithm === 'eskf' ? 'block' : 'none';
}

/**
 * Recompute frames after an ESKF input change (or an explicit run request).
 */
function recomputeEskf(force = false): void {
  if (force) requestEskfRun();
  if (state.dataset && state.algorithm === 'eskf') {
    computeFusionFrames();
    updateDisplay(state.playbackIndex);
  }
}

function handleShowEskfCharts(): void {
  destroyCharts();
  showEskfDiagnostics();
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
 * Handle recovery time change
 * Calculates recoveryTriggerPeriod from IMU rate and recovery time in seconds
 */
function handleRecoveryTimeChange(): void {
  const elements = getElements();
  const recoveryTimeSeconds = parseFloat(elements.recoveryTimeSlider.value);
  elements.recoveryTimeValue.textContent = recoveryTimeSeconds.toFixed(1);
  
  if (state.fusionAhrs && state.dataset) {
    // Calculate recovery trigger period from IMU rate
    const imuRate = state.dataset.imuRate;
    const recoveryTriggerPeriod = Math.round(imuRate * recoveryTimeSeconds);
    
    debug.log(`Recovery time: ${recoveryTimeSeconds}s @ ${imuRate}Hz = ${recoveryTriggerPeriod} samples`);
    
    state.fusionAhrs.updateAhrsSettings({ recoveryTriggerPeriod });
    
    if (state.algorithm === 'fusion') {
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

/**
 * Handle show linear acceleration toggle
 */
function handleShowLinearAccelChange(): void {
  const elements = getElements();
  if (state.viewer) {
    state.viewer.toggleLinearAccel(elements.showLinearAccel.checked);
    updateDisplay(state.playbackIndex);
  }
}

/**
 * Handle show earth acceleration toggle
 */
function handleShowEarthAccelChange(): void {
  const elements = getElements();
  if (state.viewer) {
    state.viewer.toggleEarthAccel(elements.showEarthAccel.checked);
    updateDisplay(state.playbackIndex);
  }
}

/**
 * Handle show gravity toggle
 */
function handleShowGravityChange(): void {
  const elements = getElements();
  if (state.viewer) {
    state.viewer.toggleGravity(elements.showGravity.checked);
    updateDisplay(state.playbackIndex);
  }
}

/**
 * Handle show heading toggle
 */
function handleShowHeadingChange(): void {
  const elements = getElements();
  if (state.viewer) {
    state.viewer.toggleHeading(elements.showHeading.checked);
    updateDisplay(state.playbackIndex);
  }
}

/**
 * Handle show compass heading toggle
 */
function handleShowCompassHeadingChange(): void {
  const elements = getElements();
  if (state.viewer) {
    state.viewer.toggleCompassHeading(elements.showCompassHeading.checked);
    updateDisplay(state.playbackIndex);
  }
}

// ============================================================================
// Acceleration Integration Handlers
// ============================================================================

/**
 * Handle integration start time slider change
 */
function handleIntegrationStartChange(): void {
  invalidateIntegration();
  const elements = getElements();
  const value = parseFloat(elements.integrationStartSlider.value);
  elements.integrationStartTime.textContent = value.toFixed(3) + 's';
  
  // Update app state with absolute time
  const datasetStartTime = parseFloat(elements.integrationStartSlider.dataset.startTime || '0');
  state.integrationStartTime = datasetStartTime + value;
  
  // Update GPS integration if loaded
  onIntegrationStartTimeChange();
}

/**
 * Discard results when their source or inputs change.
 */
function invalidateIntegration(): void {
  const elements = getElements();
  elements.lagStatus.textContent = 'Lag estimate cleared: inputs changed. Positive GPS shift moves GPS later; delayed GPS needs a negative shift. Estimation is diagnostic only and does not auto-apply.';
  state.integrationResult = null;
  elements.showChartsBtn.disabled = true;
  elements.integrationStatus.textContent = `Inputs changed. Calculate integration using ${
    state.orientationSource === 'device' ? 'device quaternions + raw acceleration' : 'viewer-computed acceleration'
  }. TRACK is a comparison, not a velocity correction.`;
  elements.integrationStatus.style.color = '';
  if (elements.chartsPanel.style.display === 'flex') handleBackToViewer();
}

/**
 * Initial velocity for the integration per the panel selection. ESKF uses the
 * filter velocity at the start frame (aided/coasting only); GPS uses the
 * TRACK velocity at startTime − shift (ESKF panel's GPS time shift).
 */
function resolveInitialVelocity(startTime: number): { velocity: Vec3NWU; source: string } {
  const elements = getElements();
  const mode = elements.integrationInitVelocity.value;
  if (mode === 'zero') return { velocity: ZERO_VELOCITY, source: 'zero' };

  if (mode === 'eskf' || mode === 'auto') {
    const frame = framesUseEskf() ? state.fusionFrames.find(f => f.timestamp >= startTime) : undefined;
    const eskf = frame?.eskf;
    if (eskf && (eskf.status === 'aided' || eskf.status === 'coasting')) {
      const [north, west, up] = eskf.velocity;
      return { velocity: { north, west, up }, source: `ESKF (${eskf.status})` };
    }
    if (mode === 'eskf') {
      throw new Error('ESKF initial velocity needs the ESKF algorithm with an aided or coasting filter at the start time.');
    }
  }

  const rawShift = Number(elements.eskfGpsShift.value);
  const shift = Number.isFinite(rawShift) ? rawShift : 0;
  const gps = getGpsVelocityNwuAt(startTime - shift);
  if (gps) {
    return { velocity: { north: gps.x, west: gps.y, up: gps.z }, source: `GPS (shift ${shift.toFixed(2)} s)` };
  }
  if (mode === 'gps') {
    throw new Error('GPS initial velocity needs a synchronized TRACK.CSV with a fix near the start time.');
  }
  return { velocity: ZERO_VELOCITY, source: 'zero (no ESKF/GPS velocity at start)' };
}

function handleCalculateIntegration(): void {
  const elements = getElements();
  
  console.log('Calculate Integration clicked');
  
  if (!state.fusionFrames.length) {
    console.log('No fusion frames available');
    return;
  }
  
  // Slider value is relative (0 to duration), convert to absolute timestamp
  const relativeTime = parseFloat(elements.integrationStartSlider.value);
  const datasetStartTime = parseFloat(elements.integrationStartSlider.dataset.startTime || '0');
  const startTime = datasetStartTime + relativeTime;
  
  console.log(`Relative time: ${relativeTime.toFixed(3)}s, Dataset start: ${datasetStartTime.toFixed(3)}s`);
  
  console.log(`Computing integration from t=${startTime.toFixed(3)}s`);
  
  const chartsPanelVisible = window.getComputedStyle(elements.chartsPanel).display !== 'none';
  invalidateIntegration();
  try {
    const init = resolveInitialVelocity(startTime);
    state.integrationResult = computeIntegration(
      state.fusionFrames, startTime, state.orientationSource, init.velocity, init.source,
    );
  } catch (error) {
    const message = error instanceof Error ? error.message : String(error);
    debug.error('Acceleration integration failed:', error);
    elements.integrationStatus.textContent = `Integration failed: ${message}`;
    elements.integrationStatus.style.color = '#ff6666';
    return;
  }
  const { initialVelocity: v0, initialVelocitySource } = state.integrationResult;
  const initText = `Initial velocity: ${initialVelocitySource} [N ${v0.north.toFixed(1)}, W ${v0.west.toFixed(1)}, U ${v0.up.toFixed(1)}] m/s.`;
  elements.integrationStatus.textContent = (state.orientationSource === 'device'
    ? 'Device integration: recorded ENU quaternion + raw logged acceleration, gravity removed, converted to NWU. No viewer calibration or remapping. '
    : framesUseEskf()
      ? 'Viewer integration: ESKF (GPS-aided) attitude with calibrated acceleration (open loop after the start). '
      : 'Viewer integration: viewer-computed Earth acceleration. ') + initText;
  
  if (state.integrationResult) {
    console.log(`Integration complete: ${state.integrationResult.time.length} points, startIndex=${state.integrationResult.startIndex}`);
    elements.showChartsBtn.disabled = false;
    
    // If charts panel is visible, update the charts immediately
    console.log(`Charts panel visible: ${chartsPanelVisible}`);
    
    if (chartsPanelVisible) {
      console.log('Updating charts...');
      elements.viewerPanel.style.display = 'none';
      elements.chartsPanel.style.display = 'flex';
      elements.showChartsBtn.textContent = 'Update Charts';
      initializeCharts(state.integrationResult);
    }
  }
}

/**
 * Handle show charts button click
 */
function handleShowCharts(): void {
  const elements = getElements();
  
  if (!state.integrationResult) {
    debug.error('No integration results available');
    return;
  }
  
  // Hide 3D viewer, show charts panel
  elements.viewerPanel.style.display = 'none';
  elements.chartsPanel.style.display = 'flex';
  
  // Initialize charts with data
  initializeCharts(state.integrationResult);
  
  // Update button text
  elements.showChartsBtn.textContent = 'Update Charts';
}

/**
 * Handle back to viewer button click
 */
function handleBackToViewer(): void {
  const elements = getElements();
  
  // Hide charts panel, show 3D viewer
  elements.chartsPanel.style.display = 'none';
  elements.viewerPanel.style.display = 'block';
  
  // Destroy charts to free memory
  destroyCharts();
  destroyEskfDiagnostics();
  elements.eskfDiagnosticsView.hidden = true;
  
  // Update button text
  elements.showChartsBtn.textContent = 'Show Integration Charts';
}

/**
 * Handle component select dropdown change
 */
function handleComponentSelectChange(): void {
  const elements = getElements();
  const component = elements.componentSelect.value;
  
  updateComponentChart(component as any);
}

// ============================================================================
// Application Entry Point
// ============================================================================

// Initialize on page load
document.addEventListener('DOMContentLoaded', init);
