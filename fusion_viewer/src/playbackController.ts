/**
 * Playback Controller Module
 * 
 * Handles playback state, timing, and the main playback loop.
 * Also manages fusion frame computation.
 */

import { state, type FusionFrame } from './appState';
import { getElements } from './uiElements';
import { RAD_TO_DEG } from './fusion';
import { debug } from './constants';

// ============================================================================
// Fusion Frame Computation
// ============================================================================

/**
 * Pre-compute all fusion frames for smooth playback
 */
export function computeFusionFrames(): void {
  const { dataset, ahrs } = state;
  const elements = getElements();
  
  if (!dataset || !ahrs) return;
  
  state.fusionFrames = [];
  ahrs.reset();
  
  let lastIMUTimestamp = dataset.startTime;
  let lastIMU: FusionFrame['imu'];
  let lastMAG: FusionFrame['mag'];
  
  // Check initialization settings
  const initFromSensors = elements.initFromSensors.checked;
  const useMag = elements.useMagnetometer.checked;
  
  // Initialize from first readings if enabled
  if (initFromSensors) {
    initializeFromFirstReadings(dataset, ahrs, useMag);
  }
  
  // Process all readings
  for (const reading of dataset.readings) {
    if (reading.type === 'MAG') {
      lastMAG = reading;
      if (useMag) {
        ahrs.updateMag(reading.x, reading.y, reading.z);
      }
    } else if (reading.type === 'IMU') {
      let dt = reading.timestamp - lastIMUTimestamp;
      lastIMUTimestamp = reading.timestamp;
      
      // Sanity check: dt should be reasonable
      if (dt > 0.1 || dt < 0) {
        dt = 0.0025;  // Default to ~400Hz
      }
      
      lastIMU = reading;
      
      ahrs.updateIMU(
        dt > 0 ? dt : 0.0025,
        reading.wx, reading.wy, reading.wz,
        reading.ax, reading.ay, reading.az
      );
      
      // Store frame
      const output = ahrs.getOutput();
      const calMag = ahrs.getCalibratedMag();
      
      state.fusionFrames.push({
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
  
  debug.log(`Computed ${state.fusionFrames.length} fusion frames`);
}

/**
 * Initialize AHRS from first accel+mag readings
 */
function initializeFromFirstReadings(
  dataset: NonNullable<typeof state.dataset>,
  ahrs: NonNullable<typeof state.ahrs>,
  useMag: boolean
): void {
  let initAccel: FusionFrame['imu'] | null = null;
  let initMag: FusionFrame['mag'] | null = null;
  
  // Find first IMU and MAG readings
  for (const reading of dataset.readings) {
    if (reading.type === 'IMU' && !initAccel) {
      initAccel = reading;
    }
    if (reading.type === 'MAG' && !initMag) {
      initMag = reading;
    }
    if (initAccel && initMag) break;
  }
  
  if (!initAccel) return;
  if (useMag && !initMag) return;
  
  // Apply calibration to initial readings
  const imuCal = ahrs.getIMUCalibration();
  const ax = initAccel.ax - imuCal.accelOffsetX;
  const ay = initAccel.ay - imuCal.accelOffsetY;
  const az = initAccel.az - imuCal.accelOffsetZ;
  
  if (useMag && initMag) {
    const magCal = ahrs.getMagCalibration();
    const mx = (initMag.x - magCal.offsetX) * magCal.scaleX;
    const my = (initMag.y - magCal.offsetY) * magCal.scaleY;
    const mz = (initMag.z - magCal.offsetZ) * magCal.scaleZ;
    
    ahrs.initFromAccelMag(ax, ay, az, mx, my, mz);
    debug.log('Initialized from first accel+mag sample');
  } else {
    ahrs.initFromAccelOnly(ax, ay, az);
    debug.log('Initialized from first accel sample (6-DOF)');
  }
}

// ============================================================================
// Playback Control
// ============================================================================

/**
 * Start playback
 */
export function startPlayback(): void {
  if (!state.dataset || state.fusionFrames.length === 0) return;
  
  state.isPlaying = true;
  state.lastFrameTime = performance.now();
  requestAnimationFrame(playbackLoop);
}

/**
 * Pause playback
 */
export function pausePlayback(): void {
  state.isPlaying = false;
}

/**
 * Reset playback to beginning
 */
export function resetPlayback(): void {
  const elements = getElements();
  
  state.isPlaying = false;
  state.playbackIndex = 0;
  state.currentSimTime = state.dataset?.startTime ?? 0;
  elements.timeSlider.value = '0';
  updateDisplay(0);
}

/**
 * Main playback loop
 */
function playbackLoop(timestamp: number): void {
  if (!state.isPlaying || !state.dataset || state.fusionFrames.length === 0) return;
  
  const elements = getElements();
  const realDeltaMs = timestamp - state.lastFrameTime;
  state.lastFrameTime = timestamp;
  
  // Advance simulation time based on playback speed
  const simDeltaSec = (realDeltaMs / 1000) * state.playbackSpeed;
  state.currentSimTime += simDeltaSec;
  
  // Find the frame for current simulation time
  while (state.playbackIndex < state.fusionFrames.length - 1 && 
         state.fusionFrames[state.playbackIndex + 1].timestamp <= state.currentSimTime) {
    state.playbackIndex++;
  }
  
  // Check if we've reached the end
  if (state.playbackIndex >= state.fusionFrames.length - 1) {
    state.isPlaying = false;
    state.playbackIndex = state.fusionFrames.length - 1;
  }
  
  // Update display
  updateDisplay(state.playbackIndex);
  
  // Update slider position
  const progress = (state.currentSimTime - state.dataset.startTime) / state.dataset.duration;
  elements.timeSlider.value = (progress * 1000).toString();
  
  // Continue loop
  if (state.isPlaying) {
    requestAnimationFrame(playbackLoop);
  }
}

// ============================================================================
// Playback Handlers
// ============================================================================

/**
 * Handle time slider change
 */
export function handleSliderChange(): void {
  if (!state.dataset || state.fusionFrames.length === 0) return;
  
  const elements = getElements();
  const progress = parseFloat(elements.timeSlider.value) / 1000;
  state.currentSimTime = state.dataset.startTime + progress * state.dataset.duration;
  
  // Find frame index for this time
  state.playbackIndex = 0;
  for (let i = 0; i < state.fusionFrames.length; i++) {
    if (state.fusionFrames[i].timestamp <= state.currentSimTime) {
      state.playbackIndex = i;
    } else {
      break;
    }
  }
  
  updateDisplay(state.playbackIndex);
}

/**
 * Handle playback speed change
 */
export function handleSpeedChange(): void {
  const elements = getElements();
  state.playbackSpeed = parseFloat(elements.speedSelect.value);
}

// ============================================================================
// Display Update
// ============================================================================

/**
 * Update all displays for a given frame index
 */
export function updateDisplay(frameIndex: number): void {
  const { dataset, viewer, ahrs, fusionFrames } = state;
  const elements = getElements();
  
  if (!dataset || fusionFrames.length === 0 || !viewer) return;
  
  const frame = fusionFrames[Math.min(frameIndex, fusionFrames.length - 1)];
  
  // Update 3D viewer
  viewer.setOrientation(frame.quaternion);
  
  // Update sensor vectors if enabled
  if (elements.showSensorVectors.checked && frame.imu && ahrs) {
    const imuCal = ahrs.getIMUCalibration();
    
    // Accelerometer (calibrated and remapped)
    const rawAccel = {
      x: frame.imu.ax - imuCal.accelOffsetX,
      y: frame.imu.ay - imuCal.accelOffsetY,
      z: frame.imu.az - imuCal.accelOffsetZ
    };
    const accel = ahrs.applyIMURemap(rawAccel.x, rawAccel.y, rawAccel.z);
    
    // Gyroscope (calibrated and remapped) - values are in deg/s, convert to rad/s
    const DEG_TO_RAD = Math.PI / 180;
    const rawGyro = {
      x: (frame.imu.wx - imuCal.gyroBiasX) * DEG_TO_RAD,
      y: (frame.imu.wy - imuCal.gyroBiasY) * DEG_TO_RAD,
      z: (frame.imu.wz - imuCal.gyroBiasZ) * DEG_TO_RAD
    };
    const gyro = ahrs.applyIMURemap(rawGyro.x, rawGyro.y, rawGyro.z);
    
    viewer.updateSensorVectors(accel, frame.calibratedMag || null, gyro);
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
    updateIMUDisplay(frame.imu, ahrs);
  }
  
  if (frame.mag) {
    updateMagDisplay(frame.mag, frame.calibratedMag);
  }
}

/**
 * Update IMU display values
 */
function updateIMUDisplay(
  imu: NonNullable<FusionFrame['imu']>,
  ahrs: typeof state.ahrs
): void {
  const elements = getElements();
  
  elements.gyroX.textContent = imu.wx.toFixed(2);
  elements.gyroY.textContent = imu.wy.toFixed(2);
  elements.gyroZ.textContent = imu.wz.toFixed(2);
  elements.accelX.textContent = imu.ax.toFixed(4);
  elements.accelY.textContent = imu.ay.toFixed(4);
  elements.accelZ.textContent = imu.az.toFixed(4);
  
  // Update diagnostic display
  if (ahrs) {
    const imuCal = ahrs.getIMUCalibration();
    const calAccel = {
      x: imu.ax - imuCal.accelOffsetX,
      y: imu.ay - imuCal.accelOffsetY,
      z: imu.az - imuCal.accelOffsetZ
    };
    const remapped = ahrs.applyIMURemap(calAccel.x, calAccel.y, calAccel.z);
    elements.rawAccelDisplay.innerHTML = 
      `Raw: [${calAccel.x.toFixed(2)}, ${calAccel.y.toFixed(2)}, ${calAccel.z.toFixed(2)}]<br>` +
      `Body: [${remapped.x.toFixed(2)}, ${remapped.y.toFixed(2)}, ${remapped.z.toFixed(2)}]`;
  }
}

/**
 * Update magnetometer display values
 */
function updateMagDisplay(
  mag: NonNullable<FusionFrame['mag']>,
  calibratedMag: FusionFrame['calibratedMag']
): void {
  const elements = getElements();
  
  elements.magX.textContent = mag.x.toFixed(3);
  elements.magY.textContent = mag.y.toFixed(3);
  elements.magZ.textContent = mag.z.toFixed(3);
  
  if (calibratedMag) {
    elements.rawMagDisplay.textContent = 
      `Mag: X=${calibratedMag.x.toFixed(3)}, Y=${calibratedMag.y.toFixed(3)}, Z=${calibratedMag.z.toFixed(3)}`;
  }
}
