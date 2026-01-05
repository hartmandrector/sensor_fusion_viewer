/**
 * Calibration File Module
 * 
 * Handles saving and loading calibration data to/from JSON files.
 * Supports partial calibrations - any subset of calibration data can be saved/loaded.
 */

import { state } from './appState';
import { getElements } from './uiElements';
import { handleIMUCalChange, handleMagCalChange, handleAxisRemapChange } from './calibrationManager';
import { computeFusionFrames, updateDisplay } from './playbackController';
import { formatSoftIronMatrix } from './ellipsoidFit';
import type { CalibrationFile, AxisRemap } from './types';
import { debug } from './constants';

const CALIBRATION_FILE_VERSION = 1;

// ============================================================================
// Save Calibration
// ============================================================================

/**
 * Build calibration file object from current state
 */
export function buildCalibrationFile(): CalibrationFile {
  const elements = getElements();
  
  const cal: CalibrationFile = {
    version: CALIBRATION_FILE_VERSION,
    createdAt: new Date().toISOString(),
  };
  
  // Hard Iron (mag offset)
  const hardIronX = parseFloat(elements.magOffsetX.value);
  const hardIronY = parseFloat(elements.magOffsetY.value);
  const hardIronZ = parseFloat(elements.magOffsetZ.value);
  if (!isNaN(hardIronX) && !isNaN(hardIronY) && !isNaN(hardIronZ)) {
    // Only save if not all zeros (uncalibrated)
    if (hardIronX !== 0 || hardIronY !== 0 || hardIronZ !== 0) {
      cal.hardIron = { x: hardIronX, y: hardIronY, z: hardIronZ };
    }
  }
  
  // Soft Iron Matrix
  if (state.softIronMatrix) {
    cal.softIronMatrix = state.softIronMatrix;
  }
  
  // Gyro Bias
  const gyroBiasX = parseFloat(elements.gyroBiasX.value);
  const gyroBiasY = parseFloat(elements.gyroBiasY.value);
  const gyroBiasZ = parseFloat(elements.gyroBiasZ.value);
  if (!isNaN(gyroBiasX) && !isNaN(gyroBiasY) && !isNaN(gyroBiasZ)) {
    if (gyroBiasX !== 0 || gyroBiasY !== 0 || gyroBiasZ !== 0) {
      cal.gyroBias = { x: gyroBiasX, y: gyroBiasY, z: gyroBiasZ };
    }
  }
  
  // Accel Bias
  const accelBiasX = parseFloat(elements.accelOffsetX.value);
  const accelBiasY = parseFloat(elements.accelOffsetY.value);
  const accelBiasZ = parseFloat(elements.accelOffsetZ.value);
  if (!isNaN(accelBiasX) && !isNaN(accelBiasY) && !isNaN(accelBiasZ)) {
    if (accelBiasX !== 0 || accelBiasY !== 0 || accelBiasZ !== 0) {
      cal.accelBias = { x: accelBiasX, y: accelBiasY, z: accelBiasZ };
    }
  }
  
  // Accel Scale Matrix
  if (state.accelScaleMatrix) {
    cal.accelScaleMatrix = state.accelScaleMatrix;
  }
  
  // IMU Axis Remap
  const imuAxisRemap = getCurrentIMUAxisRemap(elements);
  if (imuAxisRemap) {
    cal.imuAxisRemap = imuAxisRemap;
  }
  
  // Mag Axis Remap
  const magAxisRemap = getCurrentMagAxisRemap(elements);
  if (magAxisRemap) {
    cal.magAxisRemap = magAxisRemap;
  }
  
  return cal;
}

/**
 * Get current IMU axis remap from UI
 */
function getCurrentIMUAxisRemap(elements: ReturnType<typeof getElements>): AxisRemap | null {
  const bodyX = elements.imuRemapX.value as AxisRemap['bodyX'];
  const bodyY = elements.imuRemapY.value as AxisRemap['bodyY'];
  const bodyZ = elements.imuRemapZ.value as AxisRemap['bodyZ'];
  
  // Only include if not default (+X, +Y, +Z)
  if (bodyX !== '+X' || bodyY !== '+Y' || bodyZ !== '+Z') {
    return { bodyX, bodyY, bodyZ };
  }
  return null;
}

/**
 * Get current Mag axis remap from UI
 */
function getCurrentMagAxisRemap(elements: ReturnType<typeof getElements>): AxisRemap | null {
  const bodyX = elements.magRemapX.value as AxisRemap['bodyX'];
  const bodyY = elements.magRemapY.value as AxisRemap['bodyY'];
  const bodyZ = elements.magRemapZ.value as AxisRemap['bodyZ'];
  
  // Only include if not default (+X, +Y, +Z)
  if (bodyX !== '+X' || bodyY !== '+Y' || bodyZ !== '+Z') {
    return { bodyX, bodyY, bodyZ };
  }
  return null;
}

/**
 * Save calibration to JSON file
 */
export function saveCalibrationFile(): void {
  const cal = buildCalibrationFile();
  
  // Count what's included
  const included: string[] = [];
  if (cal.hardIron) included.push('hard iron');
  if (cal.softIronMatrix) included.push('soft iron matrix');
  if (cal.gyroBias) included.push('gyro bias');
  if (cal.accelBias) included.push('accel bias');
  if (cal.accelScaleMatrix) included.push('accel scale matrix');
  if (cal.imuAxisRemap) included.push('IMU axis remap');
  if (cal.magAxisRemap) included.push('mag axis remap');
  
  if (included.length === 0) {
    updateCalibrationFileStatus('No calibration data to save', 'warning');
    return;
  }
  
  // Create JSON blob
  const json = JSON.stringify(cal, null, 2);
  const blob = new Blob([json], { type: 'application/json' });
  
  // Generate filename with timestamp
  const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19);
  const filename = `flysight_calibration_${timestamp}.json`;
  
  // Trigger download
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  a.click();
  URL.revokeObjectURL(url);
  
  updateCalibrationFileStatus(`Saved: ${included.join(', ')}`, 'success');
  debug.log('Saved calibration file:', cal);
}

// ============================================================================
// Load Calibration
// ============================================================================

/**
 * Load calibration from JSON file
 */
export function loadCalibrationFile(file: File): void {
  const reader = new FileReader();
  
  reader.onload = (e) => {
    try {
      const json = e.target?.result as string;
      const cal = JSON.parse(json) as CalibrationFile;
      
      if (!cal.version) {
        throw new Error('Invalid calibration file: missing version');
      }
      
      applyCalibrationFile(cal);
    } catch (error) {
      updateCalibrationFileStatus(`Load failed: ${error}`, 'error');
      debug.log('Failed to load calibration file:', error);
    }
  };
  
  reader.onerror = () => {
    updateCalibrationFileStatus('Failed to read file', 'error');
  };
  
  reader.readAsText(file);
}

/**
 * Apply loaded calibration to UI and state
 */
function applyCalibrationFile(cal: CalibrationFile): void {
  const elements = getElements();
  const applied: string[] = [];
  
  // Hard Iron
  if (cal.hardIron) {
    elements.magOffsetX.value = cal.hardIron.x.toString();
    elements.magOffsetY.value = cal.hardIron.y.toString();
    elements.magOffsetZ.value = cal.hardIron.z.toString();
    applied.push('hard iron');
  }
  
  // Soft Iron Matrix
  if (cal.softIronMatrix) {
    state.softIronMatrix = cal.softIronMatrix;
    
    // Apply to both AHRS instances
    if (state.fusionAhrs?.setSoftIronMatrix) {
      state.fusionAhrs.setSoftIronMatrix(cal.softIronMatrix);
    }
    if (state.madgwickAhrs?.setSoftIronMatrix) {
      state.madgwickAhrs.setSoftIronMatrix(cal.softIronMatrix);
    }
    
    // Update display
    elements.softIronMatrixDisplay.innerHTML = formatSoftIronMatrix(matrixToDisplayFormat(cal.softIronMatrix));
    applied.push('soft iron matrix');
  }
  
  // Gyro Bias
  if (cal.gyroBias) {
    elements.gyroBiasX.value = cal.gyroBias.x.toString();
    elements.gyroBiasY.value = cal.gyroBias.y.toString();
    elements.gyroBiasZ.value = cal.gyroBias.z.toString();
    applied.push('gyro bias');
  }
  
  // Accel Bias
  if (cal.accelBias) {
    elements.accelOffsetX.value = cal.accelBias.x.toString();
    elements.accelOffsetY.value = cal.accelBias.y.toString();
    elements.accelOffsetZ.value = cal.accelBias.z.toString();
    applied.push('accel bias');
  }
  
  // Accel Scale Matrix
  if (cal.accelScaleMatrix) {
    state.accelScaleMatrix = cal.accelScaleMatrix;
    
    // Apply to both AHRS instances
    if (state.fusionAhrs?.setAccelScaleMatrix) {
      state.fusionAhrs.setAccelScaleMatrix(cal.accelScaleMatrix);
    }
    if (state.madgwickAhrs?.setAccelScaleMatrix) {
      state.madgwickAhrs.setAccelScaleMatrix(cal.accelScaleMatrix);
    }
    applied.push('accel scale matrix');
  }
  
  // IMU Axis Remap
  if (cal.imuAxisRemap) {
    elements.imuRemapX.value = cal.imuAxisRemap.bodyX;
    elements.imuRemapY.value = cal.imuAxisRemap.bodyY;
    elements.imuRemapZ.value = cal.imuAxisRemap.bodyZ;
    applied.push('IMU axis remap');
  }
  
  // Mag Axis Remap
  if (cal.magAxisRemap) {
    elements.magRemapX.value = cal.magAxisRemap.bodyX;
    elements.magRemapY.value = cal.magAxisRemap.bodyY;
    elements.magRemapZ.value = cal.magAxisRemap.bodyZ;
    applied.push('mag axis remap');
  }
  
  // Apply changes to AHRS
  handleIMUCalChange();
  handleMagCalChange();
  handleAxisRemapChange();
  
  // Recompute if data loaded
  if (state.dataset) {
    computeFusionFrames();
    updateDisplay(state.playbackIndex);
  }
  
  const info = cal.createdAt ? ` (from ${new Date(cal.createdAt).toLocaleDateString()})` : '';
  updateCalibrationFileStatus(`Loaded: ${applied.join(', ')}${info}`, 'success');
  debug.log('Loaded calibration file:', cal);
}

/**
 * Convert softIronMatrix array to display format expected by formatSoftIronMatrix
 */
function matrixToDisplayFormat(matrix: number[][]): number[][] {
  // formatSoftIronMatrix expects the same format - just pass through
  return matrix;
}

// ============================================================================
// UI Helpers
// ============================================================================

/**
 * Update calibration file status display
 */
function updateCalibrationFileStatus(message: string, type: 'success' | 'warning' | 'error' | 'info' = 'info'): void {
  const statusEl = document.getElementById('calibrationFileStatus');
  if (!statusEl) return;
  
  statusEl.textContent = message;
  statusEl.style.color = {
    success: '#4CAF50',
    warning: '#FF9800',
    error: '#f44336',
    info: '#888'
  }[type];
}

// ============================================================================
// Event Handler Setup
// ============================================================================

/**
 * Initialize calibration file UI handlers
 */
export function initializeCalibrationFileUI(): void {
  const saveBtn = document.getElementById('saveCalibrationBtn');
  const loadBtn = document.getElementById('loadCalibrationBtn');
  const fileInput = document.getElementById('calibrationFileInput') as HTMLInputElement;
  
  if (saveBtn) {
    saveBtn.addEventListener('click', saveCalibrationFile);
  }
  
  if (loadBtn && fileInput) {
    loadBtn.addEventListener('click', () => fileInput.click());
    fileInput.addEventListener('change', (e) => {
      const file = (e.target as HTMLInputElement).files?.[0];
      if (file) {
        loadCalibrationFile(file);
        fileInput.value = ''; // Reset for same file selection
      }
    });
  }
}
