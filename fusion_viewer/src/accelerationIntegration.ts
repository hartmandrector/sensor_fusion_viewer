/**
 * Acceleration Integration Module
 * 
 * Integrates earth-frame acceleration to compute velocity and position.
 * Uses NWU (North-West-Up) coordinate system.
 * 
 * Integration method:
 *   v_new = v_old + a * dt
 *   p_new = p_old + v_new * dt
 * 
 * Starting conditions: v = [0,0,0], p = [0,0,0] at integration start time
 */

import { state, FusionFrame } from './appState';
import { debug } from './constants';

// ============================================================================
// Types
// ============================================================================

export interface IntegrationResult {
  // Time array (seconds from data start)
  time: number[];
  
  // Velocity in m/s (NWU: North, West, Up)
  velNorth: number[];
  velWest: number[];
  velUp: number[];
  
  // Position in meters (NWU: North, West, Up)
  posNorth: number[];
  posWest: number[];
  posUp: number[];
  
  // Acceleration in m/s² (for reference, converted from g)
  accelNorth: number[];
  accelWest: number[];
  accelUp: number[];
  
  // Derived quantities for charts
  horizontalSpeed: number[];      // sqrt(vN² + vW²)
  horizontalDistance: number[];   // sqrt(pN² + pW²)
  
  // Integration start info
  startTime: number;
  startIndex: number;
}

// ============================================================================
// Constants
// ============================================================================

const G_TO_MS2 = 9.80665;  // 1g in m/s²

// ============================================================================
// Integration
// ============================================================================

/**
 * Compute velocity and position by integrating earth-frame acceleration
 * 
 * @param frames - Array of fusion frames with earth acceleration data
 * @param startTime - Time to start integration (frames before this have v=0, p=0)
 * @returns Integration result with time series data for all frames
 */
export function computeIntegration(
  frames: FusionFrame[],
  startTime: number
): IntegrationResult {
  const n = frames.length;
  
  // Initialize arrays
  const result: IntegrationResult = {
    time: new Array(n),
    velNorth: new Array(n),
    velWest: new Array(n),
    velUp: new Array(n),
    posNorth: new Array(n),
    posWest: new Array(n),
    posUp: new Array(n),
    accelNorth: new Array(n),
    accelWest: new Array(n),
    accelUp: new Array(n),
    horizontalSpeed: new Array(n),
    horizontalDistance: new Array(n),
    startTime,
    startIndex: 0
  };
  
  if (n === 0) return result;
  
  // Find start index
  let startIdx = 0;
  for (let i = 0; i < n; i++) {
    if (frames[i].timestamp >= startTime) {
      startIdx = i;
      break;
    }
  }
  result.startIndex = startIdx;
  
  // Initialize state at start
  let vN = 0, vW = 0, vU = 0;
  let pN = 0, pW = 0, pU = 0;
  let prevTime = frames[startIdx]?.timestamp ?? 0;
  
  // Process ALL frames
  for (let i = 0; i < n; i++) {
    const frame = frames[i];
    const t = frame.timestamp;
    
    // Get earth acceleration (defaults to 0 if not available)
    // Earth frame is NWU: X=North, Y=West, Z=Up
    const aN = (frame.earthAccel?.x ?? 0) * G_TO_MS2;
    const aW = (frame.earthAccel?.y ?? 0) * G_TO_MS2;
    const aU = (frame.earthAccel?.z ?? 0) * G_TO_MS2;
    
    result.time[i] = t;
    result.accelNorth[i] = aN;
    result.accelWest[i] = aW;
    result.accelUp[i] = aU;
    
    if (i < startIdx) {
      // Before integration start - everything is zero (like spreadsheet)
      result.velNorth[i] = 0;
      result.velWest[i] = 0;
      result.velUp[i] = 0;
      result.posNorth[i] = 0;
      result.posWest[i] = 0;
      result.posUp[i] = 0;
      result.horizontalSpeed[i] = 0;
      result.horizontalDistance[i] = 0;
    } else if (i === startIdx) {
      // At integration start - initialize to zero
      result.velNorth[i] = 0;
      result.velWest[i] = 0;
      result.velUp[i] = 0;
      result.posNorth[i] = 0;
      result.posWest[i] = 0;
      result.posUp[i] = 0;
      result.horizontalSpeed[i] = 0;
      result.horizontalDistance[i] = 0;
      prevTime = t;
    } else {
      // After integration start - integrate
      const dt = t - prevTime;
      
      // Integrate velocity: v_new = v_old + a * dt
      vN += aN * dt;
      vW += aW * dt;
      vU += aU * dt;
      
      // Integrate position: p_new = p_old + v_new * dt
      pN += vN * dt;
      pW += vW * dt;
      pU += vU * dt;
      
      result.velNorth[i] = vN;
      result.velWest[i] = vW;
      result.velUp[i] = vU;
      result.posNorth[i] = pN;
      result.posWest[i] = pW;
      result.posUp[i] = pU;
      
      // Derived quantities
      result.horizontalSpeed[i] = Math.sqrt(vN * vN + vW * vW);
      result.horizontalDistance[i] = Math.sqrt(pN * pN + pW * pW);
      
      prevTime = t;
    }
  }
  
  debug.log(`Integration computed: ${n} frames, start at t=${startTime.toFixed(3)}s (index ${startIdx})`);
  
  return result;
}

/**
 * Get integration result from state, computing if needed
 */
export function getOrComputeIntegration(startTime: number): IntegrationResult | null {
  if (state.fusionFrames.length === 0) {
    return null;
  }
  
  return computeIntegration(state.fusionFrames, startTime);
}
