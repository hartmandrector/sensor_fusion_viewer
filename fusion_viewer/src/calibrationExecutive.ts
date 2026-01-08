/**
 * Calibration Executive Module
 * 
 * Central management of calibration state, provenance tracking, and summary display.
 * Tracks where each calibration came from (defaults, loaded from file, calculated),
 * whether it's been saved, and provides a comprehensive status summary.
 */

import { state } from './appState';
import { getElements } from './uiElements';
import { computeFusionFrames, updateDisplay } from './playbackController';
import { handleIMUCalChange, handleMagCalChange, handleAxisRemapChange } from './calibrationManager';
import { formatSoftIronMatrix } from './ellipsoidFit';
import type { CalibrationFile, AxisRemap } from './types';
import { debug } from './constants';

// ============================================================================
// Types
// ============================================================================

/**
 * Source of a calibration value
 */
export type CalibrationSource = 'default' | 'loaded' | 'calculated';

/**
 * Metadata for a single calibration component
 */
export interface CalibrationMeta {
  source: CalibrationSource;
  sourceFile: string | null;      // Filename (JSON for loaded, CSV for calculated)
  timestamp: Date | null;         // When it was loaded/calculated
  isDirty: boolean;               // True if calculated but not yet saved
  isApplied: boolean;             // True if currently applied to AHRS
}

/**
 * Complete calibration state tracking
 */
export interface CalibrationState {
  hardIron: CalibrationMeta;
  softIronMatrix: CalibrationMeta;
  gyroBias: CalibrationMeta;
  accelBias: CalibrationMeta;
  accelScaleMatrix: CalibrationMeta;
  imuAxisRemap: CalibrationMeta;
  magAxisRemap: CalibrationMeta;
}

/**
 * Current calibration values (for display)
 */
export interface CalibrationValues {
  hardIron: { x: number; y: number; z: number } | null;
  softIronMatrix: number[][] | null;
  gyroBias: { x: number; y: number; z: number } | null;
  accelBias: { x: number; y: number; z: number } | null;
  accelScaleMatrix: number[][] | null;
  imuAxisRemap: AxisRemap | null;
  magAxisRemap: AxisRemap | null;
}

// ============================================================================
// State
// ============================================================================

const CALIBRATION_FILE_VERSION = 1;

/**
 * Default metadata for a calibration component
 */
function createDefaultMeta(): CalibrationMeta {
  return {
    source: 'default',
    sourceFile: null,
    timestamp: null,
    isDirty: false,
    isApplied: true
  };
}

/**
 * Calibration state tracking (internal)
 */
const calibrationState: CalibrationState = {
  hardIron: createDefaultMeta(),
  softIronMatrix: createDefaultMeta(),
  gyroBias: createDefaultMeta(),
  accelBias: createDefaultMeta(),
  accelScaleMatrix: createDefaultMeta(),
  imuAxisRemap: createDefaultMeta(),
  magAxisRemap: createDefaultMeta(),
};

/**
 * Last loaded/saved file info
 */
let lastLoadedFile: { name: string; timestamp: Date } | null = null;
let lastSavedFile: { name: string; timestamp: Date } | null = null;

// ============================================================================
// State Management
// ============================================================================

/**
 * Update calibration source metadata
 * @param isApplied - Whether the calibration is immediately applied (true for loaded, false for calculated)
 */
export function updateCalibrationSource(
  component: keyof CalibrationState,
  source: CalibrationSource,
  sourceFile: string | null = null,
  isApplied: boolean = true
): void {
  calibrationState[component] = {
    source,
    sourceFile,
    timestamp: new Date(),
    isDirty: source === 'calculated',  // Calculated values are dirty until saved
    isApplied
  };
  
  updateSummaryDisplay();
  debug.log(`Calibration ${component} updated: source=${source}, file=${sourceFile}, applied=${isApplied}`);
}

/**
 * Mark a calibration component as applied
 */
export function markCalibrationApplied(component: keyof CalibrationState): void {
  calibrationState[component].isApplied = true;
  updateSummaryDisplay();
}

/**
 * Mark all calibrations as saved (no longer dirty)
 */
export function markAllCalibrationsSaved(filename: string): void {
  for (const key of Object.keys(calibrationState) as (keyof CalibrationState)[]) {
    if (calibrationState[key].isDirty) {
      calibrationState[key].isDirty = false;
    }
  }
  lastSavedFile = { name: filename, timestamp: new Date() };
  updateSummaryDisplay();
}

/**
 * Get the current calibration state
 */
export function getCalibrationState(): CalibrationState {
  return { ...calibrationState };
}

/**
 * Check if any calibrations are unsaved
 */
export function hasUnsavedCalibrations(): boolean {
  return Object.values(calibrationState).some(meta => meta.isDirty);
}

/**
 * Get list of unsaved calibration components
 */
export function getUnsavedCalibrations(): string[] {
  const unsaved: string[] = [];
  for (const [key, meta] of Object.entries(calibrationState)) {
    if (meta.isDirty) {
      unsaved.push(formatComponentName(key as keyof CalibrationState));
    }
  }
  return unsaved;
}

// ============================================================================
// Current Values
// ============================================================================

/**
 * Get current calibration values from UI/state
 */
export function getCurrentValues(): CalibrationValues {
  const elements = getElements();
  
  return {
    hardIron: {
      x: parseFloat(elements.magOffsetX.value) || 0,
      y: parseFloat(elements.magOffsetY.value) || 0,
      z: parseFloat(elements.magOffsetZ.value) || 0
    },
    softIronMatrix: state.softIronMatrix,
    gyroBias: {
      x: parseFloat(elements.gyroBiasX.value) || 0,
      y: parseFloat(elements.gyroBiasY.value) || 0,
      z: parseFloat(elements.gyroBiasZ.value) || 0
    },
    accelBias: {
      x: parseFloat(elements.accelOffsetX.value) || 0,
      y: parseFloat(elements.accelOffsetY.value) || 0,
      z: parseFloat(elements.accelOffsetZ.value) || 0
    },
    accelScaleMatrix: state.accelScaleMatrix,
    imuAxisRemap: {
      bodyX: elements.imuRemapX.value as AxisRemap['bodyX'],
      bodyY: elements.imuRemapY.value as AxisRemap['bodyY'],
      bodyZ: elements.imuRemapZ.value as AxisRemap['bodyZ']
    },
    magAxisRemap: {
      bodyX: elements.magRemapX.value as AxisRemap['bodyX'],
      bodyY: elements.magRemapY.value as AxisRemap['bodyY'],
      bodyZ: elements.magRemapZ.value as AxisRemap['bodyZ']
    }
  };
}

// ============================================================================
// Summary Display
// ============================================================================

/**
 * Format a component name for display
 */
function formatComponentName(key: keyof CalibrationState): string {
  const names: Record<keyof CalibrationState, string> = {
    hardIron: 'Hard Iron',
    softIronMatrix: 'Soft Iron Matrix',
    gyroBias: 'Gyro Bias',
    accelBias: 'Accel Bias',
    accelScaleMatrix: 'Accel Scale Matrix',
    imuAxisRemap: 'IMU Axis Remap',
    magAxisRemap: 'Mag Axis Remap'
  };
  return names[key];
}

/**
 * Format source for display
 */
function formatSource(meta: CalibrationMeta): string {
  switch (meta.source) {
    case 'default':
      return '<span class="cal-source-default">Default</span>';
    case 'loaded':
      return `<span class="cal-source-loaded">Loaded</span>`;
    case 'calculated':
      return `<span class="cal-source-calculated">Calculated</span>`;
  }
}

/**
 * Format status (Calculated/Applied/Unsaved)
 */
function formatStatus(meta: CalibrationMeta): string {
  // If calculated but not yet applied, show "Pending"
  if (meta.source === 'calculated' && !meta.isApplied) {
    return '<span class="cal-pending">⏳ Pending</span>';
  }
  // If dirty (calculated and not saved), show "Unsaved"
  if (meta.isDirty) {
    return '<span class="cal-unsaved">⚠ Unsaved</span>';
  }
  // If applied and not default, show "Applied"
  if (meta.isApplied && meta.source !== 'default') {
    return '<span class="cal-applied">✓ Applied</span>';
  }
  return '';
}

/**
 * Check if a value is non-default (non-zero for simple values)
 */
function isNonDefault(component: keyof CalibrationState, values: CalibrationValues): boolean {
  switch (component) {
    case 'hardIron':
      return values.hardIron !== null && 
        (values.hardIron.x !== 0 || values.hardIron.y !== 0 || values.hardIron.z !== 0);
    case 'softIronMatrix':
      return values.softIronMatrix !== null;
    case 'gyroBias':
      return values.gyroBias !== null &&
        (values.gyroBias.x !== 0 || values.gyroBias.y !== 0 || values.gyroBias.z !== 0);
    case 'accelBias':
      return values.accelBias !== null &&
        (values.accelBias.x !== 0 || values.accelBias.y !== 0 || values.accelBias.z !== 0);
    case 'accelScaleMatrix':
      return values.accelScaleMatrix !== null;
    case 'imuAxisRemap':
      return values.imuAxisRemap !== null &&
        (values.imuAxisRemap.bodyX !== '+X' || values.imuAxisRemap.bodyY !== '+Y' || values.imuAxisRemap.bodyZ !== '+Z');
    case 'magAxisRemap':
      return values.magAxisRemap !== null &&
        (values.magAxisRemap.bodyX !== '+X' || values.magAxisRemap.bodyY !== '+Y' || values.magAxisRemap.bodyZ !== '+Z');
    default:
      return false;
  }
}

/**
 * Generate calibration summary HTML
 */
export function generateSummaryHTML(): string {
  const values = getCurrentValues();
  const hasUnsaved = hasUnsavedCalibrations();
  
  let html = '<div class="calibration-summary">';
  
  // Header with save status
  html += '<div class="cal-summary-header">';
  html += '<strong>Calibration Status</strong>';
  if (hasUnsaved) {
    html += ' <span class="cal-unsaved-badge">Unsaved Changes</span>';
  }
  html += '</div>';
  
  // File info
  if (lastLoadedFile) {
    html += `<div class="cal-file-info">Loaded: ${lastLoadedFile.name} (${formatTimeAgo(lastLoadedFile.timestamp)})</div>`;
  }
  if (lastSavedFile) {
    html += `<div class="cal-file-info">Last saved: ${lastSavedFile.name} (${formatTimeAgo(lastSavedFile.timestamp)})</div>`;
  }
  
  html += '<table class="cal-summary-table">';
  html += '<tr><th>Component</th><th>Source</th><th>Status</th></tr>';
  
  // List each component
  const components: (keyof CalibrationState)[] = [
    'hardIron', 'softIronMatrix', 'gyroBias', 'accelBias', 
    'accelScaleMatrix', 'imuAxisRemap', 'magAxisRemap'
  ];
  
  for (const component of components) {
    const meta = calibrationState[component];
    const hasValue = isNonDefault(component, values);
    
    if (hasValue || meta.source !== 'default') {
      html += '<tr>';
      html += `<td>${formatComponentName(component)}</td>`;
      html += `<td>${formatSource(meta)}</td>`;
      html += `<td>${formatStatus(meta)}</td>`;
      html += '</tr>';
    }
  }
  
  html += '</table>';
  
  // Show "all defaults" if nothing is configured
  const activeComponents = components.filter(c => 
    isNonDefault(c, values) || calibrationState[c].source !== 'default'
  );
  if (activeComponents.length === 0) {
    html += '<div class="cal-all-defaults">Using all default values</div>';
  }
  
  html += '</div>';
  
  return html;
}

/**
 * Format time ago string
 */
function formatTimeAgo(date: Date): string {
  const seconds = Math.floor((Date.now() - date.getTime()) / 1000);
  
  if (seconds < 60) return 'just now';
  if (seconds < 3600) return `${Math.floor(seconds / 60)}m ago`;
  if (seconds < 86400) return `${Math.floor(seconds / 3600)}h ago`;
  return date.toLocaleDateString();
}

/**
 * Update the summary display in the UI
 */
export function updateSummaryDisplay(): void {
  const summaryEl = document.getElementById('calibrationSummary');
  if (summaryEl) {
    summaryEl.innerHTML = generateSummaryHTML();
  }
}

// ============================================================================
// JSON File Operations
// ============================================================================

/**
 * Build calibration file object from current state
 */
export function buildCalibrationFile(): CalibrationFile {
  const values = getCurrentValues();
  
  const cal: CalibrationFile = {
    version: CALIBRATION_FILE_VERSION,
    createdAt: new Date().toISOString(),
  };
  
  // Hard Iron
  if (values.hardIron && (values.hardIron.x !== 0 || values.hardIron.y !== 0 || values.hardIron.z !== 0)) {
    cal.hardIron = values.hardIron;
  }
  
  // Soft Iron Matrix
  if (values.softIronMatrix) {
    cal.softIronMatrix = values.softIronMatrix;
  }
  
  // Gyro Bias
  if (values.gyroBias && (values.gyroBias.x !== 0 || values.gyroBias.y !== 0 || values.gyroBias.z !== 0)) {
    cal.gyroBias = values.gyroBias;
  }
  
  // Accel Bias
  if (values.accelBias && (values.accelBias.x !== 0 || values.accelBias.y !== 0 || values.accelBias.z !== 0)) {
    cal.accelBias = values.accelBias;
  }
  
  // Accel Scale Matrix
  if (values.accelScaleMatrix) {
    cal.accelScaleMatrix = values.accelScaleMatrix;
  }
  
  // IMU Axis Remap (only if non-default)
  if (values.imuAxisRemap && 
      (values.imuAxisRemap.bodyX !== '+X' || values.imuAxisRemap.bodyY !== '+Y' || values.imuAxisRemap.bodyZ !== '+Z')) {
    cal.imuAxisRemap = values.imuAxisRemap;
  }
  
  // Mag Axis Remap (only if non-default)
  if (values.magAxisRemap &&
      (values.magAxisRemap.bodyX !== '+X' || values.magAxisRemap.bodyY !== '+Y' || values.magAxisRemap.bodyZ !== '+Z')) {
    cal.magAxisRemap = values.magAxisRemap;
  }
  
  return cal;
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
    updateFileStatus('No calibration data to save', 'warning');
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
  
  // Mark as saved
  markAllCalibrationsSaved(filename);
  
  updateFileStatus(`Saved: ${included.join(', ')}`, 'success');
  debug.log('Saved calibration file:', cal);
}

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
      
      applyCalibrationFile(cal, file.name);
    } catch (error) {
      updateFileStatus(`Load failed: ${error}`, 'error');
      debug.log('Failed to load calibration file:', error);
    }
  };
  
  reader.onerror = () => {
    updateFileStatus('Failed to read file', 'error');
  };
  
  reader.readAsText(file);
}

/**
 * Apply loaded calibration to UI and state
 */
function applyCalibrationFile(cal: CalibrationFile, filename: string): void {
  const elements = getElements();
  const applied: string[] = [];
  
  // Hard Iron
  if (cal.hardIron) {
    elements.magOffsetX.value = cal.hardIron.x.toString();
    elements.magOffsetY.value = cal.hardIron.y.toString();
    elements.magOffsetZ.value = cal.hardIron.z.toString();
    updateCalibrationSource('hardIron', 'loaded', filename);
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
    elements.softIronMatrixDisplay.innerHTML = formatSoftIronMatrix(cal.softIronMatrix);
    updateCalibrationSource('softIronMatrix', 'loaded', filename);
    applied.push('soft iron matrix');
  }
  
  // Gyro Bias
  if (cal.gyroBias) {
    elements.gyroBiasX.value = cal.gyroBias.x.toString();
    elements.gyroBiasY.value = cal.gyroBias.y.toString();
    elements.gyroBiasZ.value = cal.gyroBias.z.toString();
    updateCalibrationSource('gyroBias', 'loaded', filename);
    applied.push('gyro bias');
  }
  
  // Accel Bias
  if (cal.accelBias) {
    elements.accelOffsetX.value = cal.accelBias.x.toString();
    elements.accelOffsetY.value = cal.accelBias.y.toString();
    elements.accelOffsetZ.value = cal.accelBias.z.toString();
    updateCalibrationSource('accelBias', 'loaded', filename);
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
    updateCalibrationSource('accelScaleMatrix', 'loaded', filename);
    applied.push('accel scale matrix');
  }
  
  // IMU Axis Remap
  if (cal.imuAxisRemap) {
    elements.imuRemapX.value = cal.imuAxisRemap.bodyX;
    elements.imuRemapY.value = cal.imuAxisRemap.bodyY;
    elements.imuRemapZ.value = cal.imuAxisRemap.bodyZ;
    updateCalibrationSource('imuAxisRemap', 'loaded', filename);
    applied.push('IMU axis remap');
  }
  
  // Mag Axis Remap
  if (cal.magAxisRemap) {
    elements.magRemapX.value = cal.magAxisRemap.bodyX;
    elements.magRemapY.value = cal.magAxisRemap.bodyY;
    elements.magRemapZ.value = cal.magAxisRemap.bodyZ;
    updateCalibrationSource('magAxisRemap', 'loaded', filename);
    applied.push('mag axis remap');
  }
  
  // Mark all loaded values as not dirty (they came from a file)
  for (const key of Object.keys(calibrationState) as (keyof CalibrationState)[]) {
    if (calibrationState[key].source === 'loaded') {
      calibrationState[key].isDirty = false;
    }
  }
  
  // Update last loaded file
  lastLoadedFile = { name: filename, timestamp: new Date() };
  
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
  updateFileStatus(`Loaded: ${applied.join(', ')}${info}`, 'success');
  updateSummaryDisplay();
  debug.log('Loaded calibration file:', cal);
}

/**
 * Update calibration file status display
 */
function updateFileStatus(message: string, type: 'success' | 'warning' | 'error' | 'info' = 'info'): void {
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
// Integration with Calibration Calculations
// ============================================================================

/**
 * Called when sphere fit calibration is calculated
 */
export function onSphereFitCalculated(sourceFile: string | null): void {
  updateCalibrationSource('hardIron', 'calculated', sourceFile, false);
}

/**
 * Called when sphere fit calibration is applied
 */
export function onSphereFitApplied(): void {
  markCalibrationApplied('hardIron');
}

/**
 * Called when ellipsoid fit calibration is calculated
 */
export function onEllipsoidFitCalculated(sourceFile: string | null): void {
  updateCalibrationSource('hardIron', 'calculated', sourceFile, false);
  updateCalibrationSource('softIronMatrix', 'calculated', sourceFile, false);
}

/**
 * Called when ellipsoid fit calibration is applied
 */
export function onEllipsoidFitApplied(): void {
  markCalibrationApplied('hardIron');
  markCalibrationApplied('softIronMatrix');
}

/**
 * Called when IMU calibration is calculated
 */
export function onIMUCalibrationCalculated(sourceFile: string | null): void {
  updateCalibrationSource('gyroBias', 'calculated', sourceFile, false);
  updateCalibrationSource('accelBias', 'calculated', sourceFile, false);
}

/**
 * Called when IMU calibration is applied
 */
export function onIMUCalibrationApplied(): void {
  markCalibrationApplied('gyroBias');
  markCalibrationApplied('accelBias');
}

/**
 * Called when 6-position accel calibration is calculated
 * Note: 6-pos calibrates accelerometer (bias + scale matrix) AND gyro bias from stationary segments
 */
export function on6PosCalibrationCalculated(sourceFile: string | null): void {
  updateCalibrationSource('accelBias', 'calculated', sourceFile, false);
  updateCalibrationSource('accelScaleMatrix', 'calculated', sourceFile, false);
  updateCalibrationSource('gyroBias', 'calculated', sourceFile, false);
}

/**
 * Called when 6-pos gyro bias only is applied
 */
export function on6PosGyroBiasApplied(): void {
  markCalibrationApplied('gyroBias');
}

/**
 * Called when 6-pos accel bias only is applied
 */
export function on6PosAccelBiasApplied(): void {
  markCalibrationApplied('accelBias');
}

/**
 * Called when 6-pos all bias is applied (accel + gyro)
 */
export function on6PosAllBiasApplied(): void {
  markCalibrationApplied('accelBias');
  markCalibrationApplied('gyroBias');
}

/**
 * Called when 6-pos full matrix calibration is applied
 * Applies accel bias, accel scale matrix, and gyro bias
 */
export function on6PosFullMatrixApplied(): void {
  markCalibrationApplied('accelBias');
  markCalibrationApplied('accelScaleMatrix');
  markCalibrationApplied('gyroBias');
}

/**
 * Called when axis remap is changed
 */
export function onAxisRemapChanged(type: 'imu' | 'mag'): void {
  const component = type === 'imu' ? 'imuAxisRemap' : 'magAxisRemap';
  // If it's not from a loaded file, mark as manual edit
  if (calibrationState[component].source !== 'loaded') {
    calibrationState[component].source = 'calculated';
    calibrationState[component].isDirty = true;
    calibrationState[component].timestamp = new Date();
  }
  updateSummaryDisplay();
}

// ============================================================================
// Initialization
// ============================================================================

/**
 * Initialize calibration executive UI handlers
 */
export function initializeCalibrationExecutive(): void {
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
        fileInput.value = '';  // Reset for same file selection
      }
    });
  }
  
  // Initial summary display
  updateSummaryDisplay();
}

/**
 * Reset all calibration tracking to defaults
 */
export function resetCalibrationState(): void {
  for (const key of Object.keys(calibrationState) as (keyof CalibrationState)[]) {
    calibrationState[key] = createDefaultMeta();
  }
  lastLoadedFile = null;
  lastSavedFile = null;
  updateSummaryDisplay();
}
