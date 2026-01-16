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
  const { dataset, ahrs, algorithm, fusionAhrs } = state;
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
      
      // Build base frame with acceleration vectors computed NOW (while AHRS state is current)
      const frame: FusionFrame = {
        timestamp: reading.timestamp,
        quaternion: output.quaternion,
        euler: output.euler,
        heading: output.heading,
        imu: lastIMU,
        mag: lastMAG,
        calibratedMag: calMag.valid ? { x: calMag.x, y: calMag.y, z: calMag.z } : undefined,
        // Store acceleration vectors computed from current AHRS state
        linearAccel: ahrs.getLinearAcceleration(),
        earthAccel: ahrs.getEarthAcceleration(),
        gravity: ahrs.getGravityVector()
      };
      
      // Add Fusion-specific states if using Fusion algorithm
      if (algorithm === 'fusion' && fusionAhrs) {
        frame.internalStates = fusionAhrs.getInternalStates();
        frame.biasState = fusionAhrs.getBiasState();
      }
      
      state.fusionFrames.push(frame);
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
  const { dataset, viewer, ahrs, fusionFrames, algorithm } = state;
  const elements = getElements();
  
  if (!dataset || fusionFrames.length === 0 || !viewer) return;
  
  const frame = fusionFrames[Math.min(frameIndex, fusionFrames.length - 1)];
  
  // Update 3D viewer
  viewer.setOrientation(frame.quaternion);
  
  // Update heading vector (compass direction in world frame)
  viewer.updateHeadingVector(frame.heading);
  
  // Update compass heading vector (FusionCompass algorithm)
  // Compute from current frame's IMU data using FusionCompass (Chapter 7)
  if (ahrs && frame.imu) {
    // Get calibration data for this frame
    const magCal = ahrs.getMagCalibration();
    
    // Calibrate accelerometer
   // const accelCal = {
   //   x: frame.imu.ax - imuCal.accelOffsetX,
   //   y: frame.imu.ay - imuCal.accelOffsetY,
   //   z: frame.imu.az - imuCal.accelOffsetZ
   // };
    
    // Only compute compass heading if we have valid mag data
    if (frame.mag) {
      // Calibrate magnetometer
      let magCal_x = frame.mag.x - magCal.offsetX;
      let magCal_y = frame.mag.y - magCal.offsetY;
      let magCal_z = frame.mag.z - magCal.offsetZ;
      
      // Apply scale factors
      magCal_x *= magCal.scaleX;
      magCal_y *= magCal.scaleY;
      magCal_z *= magCal.scaleZ;
      
      // Use quaternion-based compass heading for arbitrary device orientations
      // This properly accounts for device tilt
      const magBody = { x: magCal_x, y: magCal_y, z: magCal_z };
      const compassHeading = ahrs.getCompassHeadingFromMagQuaternion?.(magBody, frame.quaternion) || 0;
      viewer.updateCompassHeading(compassHeading);
    }
  }
  
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
    
    // Update linear acceleration, earth acceleration, and gravity vectors
    // Use pre-computed values from the frame (computed when AHRS state was current)
    viewer.updateAccelerationVectors(
      frame.linearAccel || null,
      frame.earthAccel || null,
      frame.gravity || null
    );
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
  
  // Update Fusion-specific displays
  if (algorithm === 'fusion' && frame.internalStates) {
    updateFusionDisplay(frame.internalStates, frame.biasState);
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
/**
 * Update Fusion Ch.7 specific status displays
 */
function updateFusionDisplay(
  internalStates: NonNullable<FusionFrame['internalStates']>,
  biasState?: FusionFrame['biasState']
): void {
  const elements = getElements();
  
  // Acceleration status
  elements.accelStatus.textContent = internalStates.accelerometerIgnored ? '⚠️ IGN' : '✓ OK';
  elements.accelStatus.style.color = internalStates.accelerometerIgnored ? '#ff6b6b' : '#6bff6b';
  elements.accelError.textContent = internalStates.accelerationError.toFixed(1);
  
  // Magnetic status
  elements.magStatus.textContent = internalStates.magnetometerIgnored ? '⚠️ IGN' : '✓ OK';
  elements.magStatus.style.color = internalStates.magnetometerIgnored ? '#ff6b6b' : '#6bff6b';
  elements.magError.textContent = internalStates.magneticError.toFixed(1);
  
  // Build flags string
  const flags: string[] = [];
  if (internalStates.accelerationRecoveryTrigger > 0) {
    flags.push(`A:${(internalStates.accelerationRecoveryTrigger * 100).toFixed(0)}%`);
  }
  if (internalStates.magneticRecoveryTrigger > 0) {
    flags.push(`M:${(internalStates.magneticRecoveryTrigger * 100).toFixed(0)}%`);
  }
  elements.ahrsFlags.textContent = flags.length > 0 ? flags.join(' ') : 'Ready';
  
  // Runtime bias display
  if (biasState) {
    elements.runtimeBiasX.textContent = biasState.bias.x.toFixed(3);
    elements.runtimeBiasY.textContent = biasState.bias.y.toFixed(3);
    elements.runtimeBiasZ.textContent = biasState.bias.z.toFixed(3);
    
    if (biasState.isCalibrating) {
      elements.biasCalStatus.textContent = '🔄 Updating';
      elements.biasCalStatus.style.color = '#6bff6b';
    } else if (biasState.progress > 0) {
      elements.biasCalStatus.textContent = '⏳ Stationary';
      elements.biasCalStatus.style.color = '#ffff6b';
    } else {
      elements.biasCalStatus.textContent = '⏸️ Moving';
      elements.biasCalStatus.style.color = '#888';
    }
    
    elements.biasProgress.textContent = biasState.progress > 0 
      ? `(${(biasState.progress * 100).toFixed(0)}%)` 
      : '';
    
    // Show gyro magnitude for debugging
    elements.gyroMagnitude.textContent = biasState.gyroMagnitude.toFixed(1);
    elements.stationaryThreshold.textContent = biasState.stationaryThreshold.toFixed(1);
    
    // Color code gyro magnitude based on threshold
    const magColor = biasState.gyroMagnitude < biasState.stationaryThreshold ? '#6bff6b' : '#ff6b6b';
    elements.gyroMagnitude.style.color = magColor;
  }
}