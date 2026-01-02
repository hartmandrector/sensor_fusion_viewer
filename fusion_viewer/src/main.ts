/**
 * FlySight 2 Sensor Fusion Viewer - Main Application
 * 
 * Integrates CSV parsing, Madgwick AHRS fusion, and 3D visualization
 * with playback controls for tuning and validation.
 */

import { MadgwickAHRS, RAD_TO_DEG, type MagCalibration, type IMUCalibration, type AxisRemap } from './fusion';
import { parseCSV, getMAGReadings, getIMUReadings, type SensorDataset, type IMUData, type MAGData } from './csvParser';
import { OrientationViewer } from './viewer';
import { calculateHardIronCalibration, evaluateCalibrationQuality, type MagCalibrationResult } from './magCalibration';
import { calculateIMUCalibration, analyzeIMUData, type IMUCalibrationResult } from './imuCalibration';

// Application state
let viewer: OrientationViewer | null = null;
let dataset: SensorDataset | null = null;
let ahrs: MadgwickAHRS | null = null;

// Playback state
let isPlaying = false;
let playbackIndex = 0;
let playbackSpeed = 1.0;
let lastFrameTime = 0;
let currentSimTime = 0;

// Pre-computed fusion results for smooth playback
interface FusionFrame {
  timestamp: number;
  quaternion: { w: number; x: number; y: number; z: number };
  euler: { roll: number; pitch: number; yaw: number };
  heading: number;
  imu?: IMUData;
  mag?: MAGData;
  calibratedMag?: { x: number; y: number; z: number };
}
let fusionFrames: FusionFrame[] = [];

// DOM elements
const elements = {
  csvFile: document.getElementById('csvFile') as HTMLInputElement,
  fileName: document.getElementById('fileName') as HTMLSpanElement,
  playBtn: document.getElementById('playBtn') as HTMLButtonElement,
  pauseBtn: document.getElementById('pauseBtn') as HTMLButtonElement,
  resetBtn: document.getElementById('resetBtn') as HTMLButtonElement,
  timeSlider: document.getElementById('timeSlider') as HTMLInputElement,
  currentTime: document.getElementById('currentTime') as HTMLSpanElement,
  totalTime: document.getElementById('totalTime') as HTMLSpanElement,
  speedSelect: document.getElementById('speedSelect') as HTMLSelectElement,
  betaSlider: document.getElementById('betaSlider') as HTMLInputElement,
  betaValue: document.getElementById('betaValue') as HTMLSpanElement,
  initFromSensors: document.getElementById('initFromSensors') as HTMLInputElement,
  useMagnetometer: document.getElementById('useMagnetometer') as HTMLInputElement,
  showSensorVectors: document.getElementById('showSensorVectors') as HTMLInputElement,
  imuRemapX: document.getElementById('imuRemapX') as HTMLSelectElement,
  imuRemapY: document.getElementById('imuRemapY') as HTMLSelectElement,
  imuRemapZ: document.getElementById('imuRemapZ') as HTMLSelectElement,
  magRemapX: document.getElementById('magRemapX') as HTMLSelectElement,
  magRemapY: document.getElementById('magRemapY') as HTMLSelectElement,
  magRemapZ: document.getElementById('magRemapZ') as HTMLSelectElement,
  rawAccelDisplay: document.getElementById('rawAccelDisplay') as HTMLSpanElement,
  rawMagDisplay: document.getElementById('rawMagDisplay') as HTMLSpanElement,
  magOffsetX: document.getElementById('magOffsetX') as HTMLInputElement,
  magOffsetY: document.getElementById('magOffsetY') as HTMLInputElement,
  magOffsetZ: document.getElementById('magOffsetZ') as HTMLInputElement,
  heading: document.getElementById('heading') as HTMLSpanElement,
  pitch: document.getElementById('pitch') as HTMLSpanElement,
  roll: document.getElementById('roll') as HTMLSpanElement,
  qw: document.getElementById('qw') as HTMLSpanElement,
  qx: document.getElementById('qx') as HTMLSpanElement,
  qy: document.getElementById('qy') as HTMLSpanElement,
  qz: document.getElementById('qz') as HTMLSpanElement,
  gyroX: document.getElementById('gyroX') as HTMLSpanElement,
  gyroY: document.getElementById('gyroY') as HTMLSpanElement,
  gyroZ: document.getElementById('gyroZ') as HTMLSpanElement,
  accelX: document.getElementById('accelX') as HTMLSpanElement,
  accelY: document.getElementById('accelY') as HTMLSpanElement,
  accelZ: document.getElementById('accelZ') as HTMLSpanElement,
  magX: document.getElementById('magX') as HTMLSpanElement,
  magY: document.getElementById('magY') as HTMLSpanElement,
  magZ: document.getElementById('magZ') as HTMLSpanElement,
  imuCount: document.getElementById('imuCount') as HTMLSpanElement,
  magCount: document.getElementById('magCount') as HTMLSpanElement,
  duration: document.getElementById('duration') as HTMLSpanElement,
  imuRate: document.getElementById('imuRate') as HTMLSpanElement,
  calcCalibrationBtn: document.getElementById('calcCalibrationBtn') as HTMLButtonElement,
  showMagPlotBtn: document.getElementById('showMagPlotBtn') as HTMLButtonElement,
  calibrationResult: document.getElementById('calibrationResult') as HTMLDivElement,
  // IMU calibration elements
  gyroBiasX: document.getElementById('gyroBiasX') as HTMLInputElement,
  gyroBiasY: document.getElementById('gyroBiasY') as HTMLInputElement,
  gyroBiasZ: document.getElementById('gyroBiasZ') as HTMLInputElement,
  accelOffsetX: document.getElementById('accelOffsetX') as HTMLInputElement,
  accelOffsetY: document.getElementById('accelOffsetY') as HTMLInputElement,
  accelOffsetZ: document.getElementById('accelOffsetZ') as HTMLInputElement,
  calcIMUCalBtn: document.getElementById('calcIMUCalBtn') as HTMLButtonElement,
  analyzeIMUBtn: document.getElementById('analyzeIMUBtn') as HTMLButtonElement,
  imuCalibrationResult: document.getElementById('imuCalibrationResult') as HTMLDivElement
};

// Store last calibration result
let lastCalibrationResult: MagCalibrationResult | null = null;
let lastIMUCalibrationResult: IMUCalibrationResult | null = null;

// Default calibration values (FlySight S.N. 2-00176)
// Calibrated with correct MAG axis remap (-X, +Y, -Z)
const DEFAULT_MAG_OFFSET = {
  x: -0.3465,
  y: -0.0545,
  z: -0.5380
};

const DEFAULT_GYRO_BIAS = {
  x: -0.1339,
  y: -0.1801,
  z: -0.2390
};

const DEFAULT_ACCEL_OFFSET = {
  x: -0.0118,
  y: 0.0123,
  z: 0.0450
};

/**
 * Initialize the application
 */
function init(): void {
  // Initialize 3D viewer
  viewer = new OrientationViewer('threejs-container');
  
  // Set up event listeners
  setupEventListeners();
  
  // Set default calibration values in UI - Magnetometer
  elements.magOffsetX.value = DEFAULT_MAG_OFFSET.x.toString();
  elements.magOffsetY.value = DEFAULT_MAG_OFFSET.y.toString();
  elements.magOffsetZ.value = DEFAULT_MAG_OFFSET.z.toString();
  
  // Set default calibration values in UI - Gyro bias
  elements.gyroBiasX.value = DEFAULT_GYRO_BIAS.x.toString();
  elements.gyroBiasY.value = DEFAULT_GYRO_BIAS.y.toString();
  elements.gyroBiasZ.value = DEFAULT_GYRO_BIAS.z.toString();
  
  // Set default calibration values in UI - Accel offset
  elements.accelOffsetX.value = DEFAULT_ACCEL_OFFSET.x.toString();
  elements.accelOffsetY.value = DEFAULT_ACCEL_OFFSET.y.toString();
  elements.accelOffsetZ.value = DEFAULT_ACCEL_OFFSET.z.toString();
  
  // Initialize AHRS with default settings including all calibration
  ahrs = new MadgwickAHRS({
    beta: 0.1,
    magCalibration: {
      offsetX: DEFAULT_MAG_OFFSET.x,
      offsetY: DEFAULT_MAG_OFFSET.y,
      offsetZ: DEFAULT_MAG_OFFSET.z,
      scaleX: 1,
      scaleY: 1,
      scaleZ: 1
    },
    imuCalibration: {
      gyroBiasX: DEFAULT_GYRO_BIAS.x,
      gyroBiasY: DEFAULT_GYRO_BIAS.y,
      gyroBiasZ: DEFAULT_GYRO_BIAS.z,
      accelOffsetX: DEFAULT_ACCEL_OFFSET.x,
      accelOffsetY: DEFAULT_ACCEL_OFFSET.y,
      accelOffsetZ: DEFAULT_ACCEL_OFFSET.z
    }
  });
  
  console.log('FlySight Fusion Viewer initialized');
}

/**
 * Set up all event listeners
 */
function setupEventListeners(): void {
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

/**
 * Handle CSV file selection
 */
async function handleFileSelect(event: Event): Promise<void> {
  const input = event.target as HTMLInputElement;
  const file = input.files?.[0];
  
  if (!file) return;
  
  elements.fileName.textContent = file.name;
  
  try {
    const content = await file.text();
    dataset = parseCSV(content);
    
    console.log(`Loaded ${dataset.readings.length} sensor readings`);
    console.log(`Firmware: ${dataset.firmwareVersion}`);
    console.log(`Duration: ${dataset.duration.toFixed(2)}s`);
    console.log(`IMU: ${dataset.imuCount} samples @ ${dataset.imuRate} Hz`);
    console.log(`MAG: ${dataset.magCount} samples @ ${dataset.magRate} Hz`);
    
    // Update stats display
    elements.imuCount.textContent = dataset.imuCount.toString();
    elements.magCount.textContent = dataset.magCount.toString();
    elements.duration.textContent = dataset.duration.toFixed(2);
    elements.imuRate.textContent = dataset.imuRate.toString();
    elements.totalTime.textContent = dataset.duration.toFixed(3) + 's';
    
    // Pre-compute fusion frames
    computeFusionFrames();
    
    // Enable playback controls
    elements.playBtn.disabled = false;
    elements.pauseBtn.disabled = false;
    elements.resetBtn.disabled = false;
    elements.timeSlider.disabled = false;
    elements.timeSlider.max = '1000';  // Use 1000 steps for smooth slider
    
    // Enable calibration buttons
    elements.calcCalibrationBtn.disabled = false;
    elements.showMagPlotBtn.disabled = false;
    elements.calcIMUCalBtn.disabled = false;
    elements.analyzeIMUBtn.disabled = false;
    
    // Reset playback
    resetPlayback();
    
  } catch (error) {
    console.error('Error loading CSV:', error);
    elements.fileName.textContent = 'Error loading file';
  }
}

/**
 * Pre-compute all fusion frames for smooth playback
 */
function computeFusionFrames(): void {
  if (!dataset || !ahrs) return;
  
  fusionFrames = [];
  ahrs.reset();
  
  let lastIMUTimestamp = dataset.startTime;
  let lastIMU: IMUData | undefined;
  let lastMAG: MAGData | undefined;
  
  // If init from sensors is enabled, we need to find initial accel+mag
  const initFromSensors = elements.initFromSensors.checked;
  const useMag = elements.useMagnetometer.checked;
  let initAccel: IMUData | null = null;
  let initMag: MAGData | null = null;
  
  // First pass: find initial readings if needed
  if (initFromSensors) {
    for (const reading of dataset.readings) {
      if (reading.type === 'IMU' && !initAccel) {
        initAccel = reading;
      }
      if (reading.type === 'MAG' && !initMag) {
        initMag = reading;
      }
      if (initAccel && initMag) break;
    }
    
    if (initAccel && (initMag || !useMag)) {
      // Apply calibration to initial readings
      const ax = initAccel.ax - (ahrs.getIMUCalibration().accelOffsetX);
      const ay = initAccel.ay - (ahrs.getIMUCalibration().accelOffsetY);
      const az = initAccel.az - (ahrs.getIMUCalibration().accelOffsetZ);
      
      if (useMag && initMag) {
        // Apply mag calibration (axis remap is handled internally by initFromAccelMag)
        const magCal = ahrs.getMagCalibration();
        const mx = (initMag.x - magCal.offsetX) * magCal.scaleX;
        const my = (initMag.y - magCal.offsetY) * magCal.scaleY;
        const mz = (initMag.z - magCal.offsetZ) * magCal.scaleZ;
        
        ahrs.initFromAccelMag(ax, ay, az, mx, my, mz);
        console.log('Initialized from first accel+mag sample');
      } else {
        // 6-DOF init from accel only - sets level but heading = 0
        ahrs.initFromAccelOnly(ax, ay, az);
        console.log('Initialized from first accel sample (6-DOF)');
      }
    }
  }
  
  let frameCount = 0;
  for (const reading of dataset.readings) {
    if (reading.type === 'MAG') {
      lastMAG = reading;
      if (useMag) {
        ahrs.updateMag(reading.x, reading.y, reading.z);
      }
    } else if (reading.type === 'IMU') {
      let dt = reading.timestamp - lastIMUTimestamp;
      lastIMUTimestamp = reading.timestamp;
      
      // Sanity check: dt should be small (< 0.1s for 10Hz minimum rate)
      if (dt > 0.1 || dt < 0) {
        dt = 0.0025;  // Default to ~400Hz
      }
      
      lastIMU = reading;
      
      ahrs.updateIMU(
        dt > 0 ? dt : 0.0025,  // Default to ~400Hz if dt is 0
        reading.wx, reading.wy, reading.wz,
        reading.ax, reading.ay, reading.az
      );
      
      frameCount++;
      
      // Store frame (also store calibrated mag for visualization)
      const output = ahrs.getOutput();
      const calMag = ahrs.getCalibratedMag();
      
      fusionFrames.push({
        timestamp: reading.timestamp,
        quaternion: output.quaternion,
        euler: output.euler,
        heading: output.heading,
        imu: lastIMU,
        mag: lastMAG,
        calibratedMag: calMag.valid ? { x: calMag.x, y: calMag.y, z: calMag.z } : undefined
      });
    }
  }
  
  console.log(`Computed ${fusionFrames.length} fusion frames`);
}

/**
 * Start playback
 */
function startPlayback(): void {
  if (!dataset || fusionFrames.length === 0) return;
  
  isPlaying = true;
  lastFrameTime = performance.now();
  requestAnimationFrame(playbackLoop);
}

/**
 * Pause playback
 */
function pausePlayback(): void {
  isPlaying = false;
}

/**
 * Reset playback to beginning
 */
function resetPlayback(): void {
  isPlaying = false;
  playbackIndex = 0;
  currentSimTime = dataset?.startTime ?? 0;
  elements.timeSlider.value = '0';
  updateDisplay(0);
}

/**
 * Playback loop
 */
function playbackLoop(timestamp: number): void {
  if (!isPlaying || !dataset || fusionFrames.length === 0) return;
  
  const realDeltaMs = timestamp - lastFrameTime;
  lastFrameTime = timestamp;
  
  // Advance simulation time based on playback speed
  const simDeltaSec = (realDeltaMs / 1000) * playbackSpeed;
  currentSimTime += simDeltaSec;
  
  // Find the frame for current simulation time
  while (playbackIndex < fusionFrames.length - 1 && 
         fusionFrames[playbackIndex + 1].timestamp <= currentSimTime) {
    playbackIndex++;
  }
  
  // Check if we've reached the end
  if (playbackIndex >= fusionFrames.length - 1) {
    isPlaying = false;
    playbackIndex = fusionFrames.length - 1;
  }
  
  // Update display
  updateDisplay(playbackIndex);
  
  // Update slider position
  const progress = (currentSimTime - dataset.startTime) / dataset.duration;
  elements.timeSlider.value = (progress * 1000).toString();
  
  // Continue loop
  if (isPlaying) {
    requestAnimationFrame(playbackLoop);
  }
}

/**
 * Handle time slider change
 */
function handleSliderChange(): void {
  if (!dataset || fusionFrames.length === 0) return;
  
  const progress = parseFloat(elements.timeSlider.value) / 1000;
  currentSimTime = dataset.startTime + progress * dataset.duration;
  
  // Find frame index for this time
  playbackIndex = 0;
  for (let i = 0; i < fusionFrames.length; i++) {
    if (fusionFrames[i].timestamp <= currentSimTime) {
      playbackIndex = i;
    } else {
      break;
    }
  }
  
  updateDisplay(playbackIndex);
}

/**
 * Handle playback speed change
 */
function handleSpeedChange(): void {
  playbackSpeed = parseFloat(elements.speedSelect.value);
}

/**
 * Handle beta slider change
 */
function handleBetaChange(): void {
  const beta = parseFloat(elements.betaSlider.value);
  elements.betaValue.textContent = beta.toFixed(2);
  
  if (ahrs) {
    ahrs.setBeta(beta);
    // Recompute fusion frames with new beta
    computeFusionFrames();
    // Update display at current position
    updateDisplay(playbackIndex);
  }
}

/**
 * Handle magnetometer calibration change
 */
function handleMagCalChange(): void {
  if (!ahrs) return;
  
  const cal: MagCalibration = {
    offsetX: parseFloat(elements.magOffsetX.value) || 0,
    offsetY: parseFloat(elements.magOffsetY.value) || 0,
    offsetZ: parseFloat(elements.magOffsetZ.value) || 0,
    scaleX: 1,
    scaleY: 1,
    scaleZ: 1
  };
  
  ahrs.setMagCalibration(cal);
  computeFusionFrames();
  updateDisplay(playbackIndex);
}

/**
 * Handle init mode change
 */
function handleInitModeChange(): void {
  if (dataset) {
    computeFusionFrames();
    updateDisplay(playbackIndex);
  }
}

/**
 * Handle magnetometer enable/disable
 */
function handleUseMagChange(): void {
  if (dataset) {
    computeFusionFrames();
    updateDisplay(playbackIndex);
  }
}

/**
 * Handle show sensor vectors toggle
 */
function handleShowVectorsChange(): void {
  if (viewer) {
    viewer.toggleSensorVectors(elements.showSensorVectors.checked);
    updateDisplay(playbackIndex);
  }
}

/**
 * Handle axis remap change
 */
function handleAxisRemapChange(): void {
  if (!ahrs) return;
  
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
  
  ahrs.setIMUAxisRemap(imuRemap);
  ahrs.setMagAxisRemap(magRemap);
  
  if (dataset) {
    computeFusionFrames();
    updateDisplay(playbackIndex);
  }
  
  console.log('IMU axis remap:', imuRemap);
  console.log('MAG axis remap:', magRemap);
}

/**
 * Update all displays for a given frame index
 */
function updateDisplay(frameIndex: number): void {
  if (!dataset || fusionFrames.length === 0 || !viewer) return;
  
  const frame = fusionFrames[Math.min(frameIndex, fusionFrames.length - 1)];
  
  // Update 3D viewer
  viewer.setOrientation(frame.quaternion);
  
  // Update sensor vectors if enabled
  if (elements.showSensorVectors.checked && frame.imu && ahrs) {
    // Get calibrated accel in body frame (apply calibration + axis remap)
    const imuCal = ahrs.getIMUCalibration();
    const rawAccel = {
      x: frame.imu.ax - imuCal.accelOffsetX,
      y: frame.imu.ay - imuCal.accelOffsetY,
      z: frame.imu.az - imuCal.accelOffsetZ
    };
    // Apply axis remap to get body frame
    const accel = ahrs.applyIMURemap(rawAccel.x, rawAccel.y, rawAccel.z);
    
    viewer.updateSensorVectors(accel, frame.calibratedMag || null);
  }
  
  // Update orientation display
  elements.heading.textContent = frame.heading.toFixed(1);
  elements.pitch.textContent = (frame.euler.pitch * RAD_TO_DEG).toFixed(1);
  elements.roll.textContent = (frame.euler.roll * RAD_TO_DEG).toFixed(1);
  
  // Update quaternion display
  elements.qw.textContent = frame.quaternion.w.toFixed(3);
  elements.qx.textContent = frame.quaternion.x.toFixed(3);
  elements.qy.textContent = frame.quaternion.y.toFixed(3);
  elements.qz.textContent = frame.quaternion.z.toFixed(3);
  
  // Update current time
  const relativeTime = frame.timestamp - dataset.startTime;
  elements.currentTime.textContent = relativeTime.toFixed(3) + 's';
  
  // Update raw sensor data
  if (frame.imu) {
    elements.gyroX.textContent = frame.imu.wx.toFixed(2);
    elements.gyroY.textContent = frame.imu.wy.toFixed(2);
    elements.gyroZ.textContent = frame.imu.wz.toFixed(2);
    elements.accelX.textContent = frame.imu.ax.toFixed(4);
    elements.accelY.textContent = frame.imu.ay.toFixed(4);
    elements.accelZ.textContent = frame.imu.az.toFixed(4);
    
    // Update diagnostic display with raw and remapped values
    const imuCal = ahrs?.getIMUCalibration();
    const calAccel = {
      x: frame.imu.ax - (imuCal?.accelOffsetX ?? 0),
      y: frame.imu.ay - (imuCal?.accelOffsetY ?? 0),
      z: frame.imu.az - (imuCal?.accelOffsetZ ?? 0)
    };
    const remapped = ahrs?.applyIMURemap(calAccel.x, calAccel.y, calAccel.z);
    if (remapped) {
      elements.rawAccelDisplay.innerHTML = 
        `Raw: [${calAccel.x.toFixed(2)}, ${calAccel.y.toFixed(2)}, ${calAccel.z.toFixed(2)}]<br>` +
        `Body: [${remapped.x.toFixed(2)}, ${remapped.y.toFixed(2)}, ${remapped.z.toFixed(2)}]`;
    }
  }
  
  if (frame.mag) {
    elements.magX.textContent = frame.mag.x.toFixed(3);
    elements.magY.textContent = frame.mag.y.toFixed(3);
    elements.magZ.textContent = frame.mag.z.toFixed(3);
    
    // Update diagnostic raw display (after mag transform but before axis remap)
    if (frame.calibratedMag) {
      elements.rawMagDisplay.textContent = `Mag: X=${frame.calibratedMag.x.toFixed(3)}, Y=${frame.calibratedMag.y.toFixed(3)}, Z=${frame.calibratedMag.z.toFixed(3)}`;
    }
  }
}

/**
 * Calculate magnetometer calibration from loaded data
 */
function handleCalculateCalibration(): void {
  if (!dataset) return;
  
  const magSamples = getMAGReadings(dataset);
  
  // Calculate calibration on raw mag data (no axis transform)
  // The axis remap dropdowns handle coordinate transformation separately
  lastCalibrationResult = calculateHardIronCalibration(magSamples, false);
  const quality = evaluateCalibrationQuality(lastCalibrationResult);
  
  // Build result HTML
  let html = `
    <div class="cal-label">Calculated Offsets:</div>
    <div>X: <span class="cal-value">${lastCalibrationResult.offsetX.toFixed(4)}</span></div>
    <div>Y: <span class="cal-value">${lastCalibrationResult.offsetY.toFixed(4)}</span></div>
    <div>Z: <span class="cal-value">${lastCalibrationResult.offsetZ.toFixed(4)}</span></div>
    <div style="margin-top: 0.5rem;">
      <span class="cal-label">Quality:</span> 
      <span class="quality-${quality.quality}">${quality.quality.toUpperCase()}</span>
    </div>
    <div><span class="cal-label">Sphericity:</span> <span class="cal-value">${(lastCalibrationResult.sphericity * 100).toFixed(0)}%</span></div>
    <div><span class="cal-label">Field Magnitude:</span> <span class="cal-value">${lastCalibrationResult.magnitude.toFixed(3)} gauss</span></div>
    <div><span class="cal-label">Samples:</span> <span class="cal-value">${lastCalibrationResult.sampleCount}</span></div>
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
  
  console.log('Calibration calculated:', lastCalibrationResult);
  console.log('Quality:', quality);
}

/**
 * Apply the calculated calibration to the offset fields
 */
function applyCalibration(): void {
  if (!lastCalibrationResult) return;
  
  elements.magOffsetX.value = lastCalibrationResult.offsetX.toFixed(4);
  elements.magOffsetY.value = lastCalibrationResult.offsetY.toFixed(4);
  elements.magOffsetZ.value = lastCalibrationResult.offsetZ.toFixed(4);
  
  // Trigger recalculation
  handleMagCalChange();
}

// Expose to window for button onclick
(window as unknown as { applyCalibration: () => void }).applyCalibration = applyCalibration;

/**
 * Show 3D scatter plot of magnetometer data
 */
function handleShowMagPlot(): void {
  if (!dataset || !viewer) return;
  
  const magSamples = getMAGReadings(dataset);
  
  // Get current calibration offsets
  const offsetX = parseFloat(elements.magOffsetX.value) || 0;
  const offsetY = parseFloat(elements.magOffsetY.value) || 0;
  const offsetZ = parseFloat(elements.magOffsetZ.value) || 0;
  
  // Toggle visualization in the viewer (show raw data without transform)
  viewer.toggleMagPlot(magSamples, { offsetX, offsetY, offsetZ });
}

/**
 * Handle IMU calibration input changes
 */
function handleIMUCalChange(): void {
  if (!ahrs) return;
  
  const imuCal: IMUCalibration = {
    gyroBiasX: parseFloat(elements.gyroBiasX.value) || 0,
    gyroBiasY: parseFloat(elements.gyroBiasY.value) || 0,
    gyroBiasZ: parseFloat(elements.gyroBiasZ.value) || 0,
    accelOffsetX: parseFloat(elements.accelOffsetX.value) || 0,
    accelOffsetY: parseFloat(elements.accelOffsetY.value) || 0,
    accelOffsetZ: parseFloat(elements.accelOffsetZ.value) || 0
  };
  
  ahrs.setIMUCalibration(imuCal);
  
  // Recompute if we have data
  if (dataset) {
    computeFusionFrames();
    updateDisplay(playbackIndex);
  }
}

/**
 * Calculate IMU calibration from stationary data
 */
function handleCalculateIMUCalibration(): void {
  if (!dataset) return;
  
  const imuSamples = getIMUReadings(dataset);
  
  if (imuSamples.length === 0) {
    elements.imuCalibrationResult.innerHTML = '<div class="error">No IMU data found</div>';
    elements.imuCalibrationResult.classList.add('visible');
    return;
  }
  
  lastIMUCalibrationResult = calculateIMUCalibration(imuSamples);
  
  let html = '<div class="calibration-values">';
  html += `<strong>IMU Calibration Results</strong><br>`;
  html += `<strong>Gyro Bias (deg/s):</strong><br>`;
  html += `X: ${lastIMUCalibrationResult.gyroBiasX.toFixed(4)}<br>`;
  html += `Y: ${lastIMUCalibrationResult.gyroBiasY.toFixed(4)}<br>`;
  html += `Z: ${lastIMUCalibrationResult.gyroBiasZ.toFixed(4)}<br><br>`;
  html += `<strong>Accel Offset (g):</strong><br>`;
  html += `X: ${lastIMUCalibrationResult.accelOffsetX.toFixed(4)}<br>`;
  html += `Y: ${lastIMUCalibrationResult.accelOffsetY.toFixed(4)}<br>`;
  html += `Z: ${lastIMUCalibrationResult.accelOffsetZ.toFixed(4)}<br><br>`;
  html += `<strong>Statistics:</strong><br>`;
  html += `Samples: ${lastIMUCalibrationResult.sampleCount}<br>`;
  html += `Accel Magnitude: ${lastIMUCalibrationResult.accelMagnitude.toFixed(4)}g<br>`;
  html += `Gyro StdDev: X=${lastIMUCalibrationResult.gyroStdDevX.toFixed(2)}, Y=${lastIMUCalibrationResult.gyroStdDevY.toFixed(2)}, Z=${lastIMUCalibrationResult.gyroStdDevZ.toFixed(2)}<br>`;
  
  if (!lastIMUCalibrationResult.isStationary) {
    html += `<div class="warning">⚠ Device may not have been stationary during data collection. High gyro variance detected.</div>`;
  }
  
  html += `</div>`;
  html += `<button class="apply-btn" onclick="window.applyIMUCalibration()">Apply IMU Calibration</button>`;
  
  elements.imuCalibrationResult.innerHTML = html;
  elements.imuCalibrationResult.classList.add('visible');
  
  console.log('IMU Calibration calculated:', lastIMUCalibrationResult);
}

/**
 * Apply the calculated IMU calibration to the input fields
 */
function applyIMUCalibration(): void {
  if (!lastIMUCalibrationResult) return;
  
  elements.gyroBiasX.value = lastIMUCalibrationResult.gyroBiasX.toFixed(4);
  elements.gyroBiasY.value = lastIMUCalibrationResult.gyroBiasY.toFixed(4);
  elements.gyroBiasZ.value = lastIMUCalibrationResult.gyroBiasZ.toFixed(4);
  elements.accelOffsetX.value = lastIMUCalibrationResult.accelOffsetX.toFixed(4);
  elements.accelOffsetY.value = lastIMUCalibrationResult.accelOffsetY.toFixed(4);
  elements.accelOffsetZ.value = lastIMUCalibrationResult.accelOffsetZ.toFixed(4);
  
  // Trigger recalculation
  handleIMUCalChange();
}

// Expose to window for button onclick
(window as unknown as { applyIMUCalibration: () => void }).applyIMUCalibration = applyIMUCalibration;

/**
 * Analyze IMU data quality
 */
function handleAnalyzeIMU(): void {
  if (!dataset) return;
  
  const imuSamples = getIMUReadings(dataset);
  
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
  
  console.log('IMU Analysis:', analysis);
}

// Initialize on page load
document.addEventListener('DOMContentLoaded', init);
