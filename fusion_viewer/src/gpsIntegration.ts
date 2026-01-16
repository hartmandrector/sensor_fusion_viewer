/**
 * GPS Integration module
 * Handles converting GPS data to chart-compatible format and managing GPS state
 */

import { GNSSData, GPSIntegrationPoint, GPSIntegrationResult, TrackDataset, TimeSyncResult } from './gpsTypes';
import { parseTrackCSV } from './trackParser';
import { parseTIMEEntries, computeTimeSync, applyTimeSyncToGPS, checkTimeOverlap } from './timestampSync';
import { state } from './appState';
import { smoothGPSVelocities } from './sgFilter';
import { calculateAcceleration } from './mathUtils';

// Earth radius for geodetic calculations (meters)
const EARTH_RADIUS_M = 6371000;

/**
 * GPS Integration State - kept separate from main app state
 */
export interface GPSState {
  trackDataset: TrackDataset | null;
  rawSensorCSV: string | null;          // Raw content for $TIME parsing
  syncResult: TimeSyncResult | null;
  integrationResult: GPSIntegrationResult | null;
  isLoaded: boolean;
  statusMessage: string;
}

export const gpsState: GPSState = {
  trackDataset: null,
  rawSensorCSV: null,
  syncResult: null,
  integrationResult: null,
  isLoaded: false,
  statusMessage: 'No GPS data loaded'
};

/**
 * Store the raw sensor CSV content for later $TIME parsing
 * This should be called when the main sensor file is loaded
 */
export function storeRawSensorCSV(content: string): void {
  gpsState.rawSensorCSV = content;
}

/**
 * Convert latitude/longitude to local North/East position (meters)
 * Uses simple equirectangular projection, accurate for small distances
 */
function latLonToLocalNE(
  lat: number, 
  lon: number, 
  originLat: number, 
  originLon: number
): { north: number; east: number } {
  const originLatRad = originLat * Math.PI / 180;
  
  const dLat = (lat - originLat) * Math.PI / 180;
  const dLon = (lon - originLon) * Math.PI / 180;
  
  const north = dLat * EARTH_RADIUS_M;
  const east = dLon * EARTH_RADIUS_M * Math.cos(originLatRad);
  
  return { north, east };
}

/**
 * Convert GPS data to integration-compatible format
 * Converts from NED (GPS native) to NWU (integration frame)
 * Creates local position coordinates relative to first GPS fix
 */
export function convertGPSToIntegration(
  gnssData: GNSSData[],
  startSensorTime: number
): GPSIntegrationResult | null {
  
  // Filter to only entries with valid sensor timestamps
  const validData = gnssData.filter(g => g.sensorTimestamp !== null);
  
  if (validData.length === 0) {
    return null;
  }
  
  // Sort by sensor timestamp
  validData.sort((a, b) => a.sensorTimestamp! - b.sensorTimestamp!);
  
  // Find the first GPS point at or after start time for origin
  let originIndex = validData.findIndex(g => g.sensorTimestamp! >= startSensorTime);
  if (originIndex < 0) {
    originIndex = 0; // Use first available if none after start time
  }
  
  const origin = validData[originIndex];
  const originLat = origin.lat;
  const originLon = origin.lon;
  const originAlt = origin.hMSL;
  
  const points: GPSIntegrationPoint[] = [];
  
  for (const gnss of validData) {
    const sensorTime = gnss.sensorTimestamp!;
    
    // Convert lat/lon to local NE coordinates
    const { north, east } = latLonToLocalNE(gnss.lat, gnss.lon, originLat, originLon);
    
    // Convert NED velocity to NWU
    // NED: North, East, Down
    // NWU: North, West, Up
    const velNorth = gnss.velN;
    const velWest = -gnss.velE;   // East -> West (negate)
    const velUp = -gnss.velD;     // Down -> Up (negate)
    
    // Convert position to NWU
    const posNorth = north;
    const posWest = -east;        // East -> West (negate)
    const posUp = gnss.hMSL - originAlt;  // Relative altitude
    
    // Derived values
    const horizontalSpeed = Math.sqrt(velNorth * velNorth + velWest * velWest);
    const horizontalDistance = Math.sqrt(posNorth * posNorth + posWest * posWest);
    
    points.push({
      sensorTime,
      velNorth,
      velWest,
      velUp,
      // Placeholders for smoothed velocities - will be filled in below
      smoothVelNorth: 0,
      smoothVelWest: 0,
      smoothVelUp: 0,
      posNorth,
      posWest,
      posUp,
      horizontalSpeed,
      horizontalDistance,
      smoothHorizontalSpeed: 0,
      // Acceleration placeholders - filled after smoothing
      accelNorth: 0,
      accelWest: 0,
      accelUp: 0,
      lat: gnss.lat,
      lon: gnss.lon,
      hMSL: gnss.hMSL,
      hAcc: gnss.hAcc,
      vAcc: gnss.vAcc,
      numSV: gnss.numSV
    });
  }
  
  // Apply Savitzky-Golay smoothing to velocities
  // Uses 3-pass filter with window sizes: 25, 21, 11
  const smoothedVelocities = smoothGPSVelocities(points);
  
  // Update points with smoothed values
  for (let i = 0; i < points.length; i++) {
    const smooth = smoothedVelocities[i];
    points[i].smoothVelNorth = smooth.velNorth;
    points[i].smoothVelWest = smooth.velWest;
    points[i].smoothVelUp = smooth.velUp;
    points[i].smoothHorizontalSpeed = Math.sqrt(
      smooth.velNorth * smooth.velNorth + smooth.velWest * smooth.velWest
    );
  }
  
  // Calculate acceleration from smoothed velocity using linear regression
  // Uses 21-point centered window for slope estimation (dVel/dTime = acceleration)
  // Odd window size ensures symmetric centering around each point
  const ACCEL_WINDOW_SIZE = 21;
  const accelerations = calculateAcceleration(
    points,
    ACCEL_WINDOW_SIZE,
    (p) => p.sensorTime,
    (p) => p.smoothVelNorth,
    (p) => p.smoothVelWest,
    (p) => p.smoothVelUp
  );
  
  // Update points with acceleration values
  for (let i = 0; i < points.length; i++) {
    points[i].accelNorth = accelerations.accelNorth[i];
    points[i].accelWest = accelerations.accelWest[i];
    points[i].accelUp = accelerations.accelUp[i];
  }
  
  return {
    points,
    originLat,
    originLon,
    originAlt,
    startSensorTime: points[0].sensorTime,
    endSensorTime: points[points.length - 1].sensorTime
  };
}

/**
 * Load and process a TRACK.CSV file
 * Returns success status and message
 */
export async function loadTrackCSV(file: File): Promise<{ success: boolean; message: string }> {
  
  // Check if we have sensor data loaded
  if (!state.dataset) {
    return { 
      success: false, 
      message: 'Please load a SENSOR.CSV file first' 
    };
  }
  
  if (!gpsState.rawSensorCSV) {
    return { 
      success: false, 
      message: 'Sensor CSV content not available for timestamp sync' 
    };
  }
  
  try {
    // Read the file
    const content = await file.text();
    
    // Parse TRACK.CSV
    const trackDataset = parseTrackCSV(content);
    
    if (trackDataset.gnssData.length === 0) {
      return { 
        success: false, 
        message: 'No GNSS data found in file. Is this a valid TRACK.CSV?' 
      };
    }
    
    // Parse $TIME entries from sensor CSV
    const timeEntries = parseTIMEEntries(gpsState.rawSensorCSV);
    
    if (timeEntries.length === 0) {
      return { 
        success: false, 
        message: 'No $TIME entries in SENSOR.CSV. Cannot sync timestamps.' 
      };
    }
    
    // Compute time synchronization
    const syncResult = computeTimeSync(timeEntries, trackDataset);
    
    if (!syncResult.success) {
      return { 
        success: false, 
        message: syncResult.errorMessage || 'Time sync failed' 
      };
    }
    
    // Apply sync to GPS data
    applyTimeSyncToGPS(trackDataset, syncResult);
    
    // Check for time overlap with sensor data
    const overlapCheck = checkTimeOverlap(state.dataset, trackDataset, syncResult);
    
    if (!overlapCheck.hasOverlap) {
      return { 
        success: false, 
        message: overlapCheck.message 
      };
    }
    
    // Store in state
    gpsState.trackDataset = trackDataset;
    gpsState.syncResult = syncResult;
    gpsState.isLoaded = true;
    gpsState.statusMessage = overlapCheck.message;
    
    // Compute GPS integration result using current start time
    updateGPSIntegration();
    
    return { 
      success: true, 
      message: overlapCheck.message 
    };
    
  } catch (error) {
    const errorMessage = error instanceof Error ? error.message : 'Unknown error';
    return { 
      success: false, 
      message: `Failed to parse TRACK.CSV: ${errorMessage}` 
    };
  }
}

/**
 * Update GPS integration result based on current integration start time
 * Should be called when start time slider changes
 */
export function updateGPSIntegration(): void {
  if (!gpsState.trackDataset || !gpsState.isLoaded) {
    gpsState.integrationResult = null;
    return;
  }
  
  // Get integration start time from app state
  const startTime = state.integrationStartTime;
  
  // Convert GPS data to integration format
  gpsState.integrationResult = convertGPSToIntegration(
    gpsState.trackDataset.gnssData,
    startTime
  );
  
  if (gpsState.integrationResult) {
    console.log(`GPS integration updated: ${gpsState.integrationResult.points.length} points from ${startTime.toFixed(3)}s`);
  }
}

/**
 * Get GPS data for charting within the current time range
 */
export function getGPSDataForCharts(): GPSIntegrationResult | null {
  return gpsState.integrationResult;
}

/**
 * Clear GPS state
 */
export function clearGPSData(): void {
  gpsState.trackDataset = null;
  gpsState.syncResult = null;
  gpsState.integrationResult = null;
  gpsState.isLoaded = false;
  gpsState.statusMessage = 'No GPS data loaded';
}

/**
 * Check if GPS data is available for charting
 */
export function hasGPSData(): boolean {
  return gpsState.isLoaded && gpsState.integrationResult !== null;
}
