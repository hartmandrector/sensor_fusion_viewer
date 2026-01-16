/**
 * GPS-related type definitions for TRACK.CSV parsing and timestamp synchronization
 */

/**
 * Raw GNSS data from TRACK.CSV
 * Coordinate frame: NED (North-East-Down)
 */
export interface GNSSData {
  type: 'GNSS';
  isoTime: string;        // Original ISO 8601 timestamp from file
  timestamp: number;      // Computed: milliseconds since Unix epoch
  sensorTimestamp: number | null; // Computed: aligned to sensor time (seconds since boot)
  lat: number;            // degrees
  lon: number;            // degrees
  hMSL: number;           // meters (altitude above mean sea level)
  velN: number;           // m/s (velocity North)
  velE: number;           // m/s (velocity East)  
  velD: number;           // m/s (velocity Down)
  hAcc: number;           // m (horizontal accuracy)
  vAcc: number;           // m (vertical accuracy)
  sAcc: number;           // m/s (speed accuracy)
  numSV: number;          // number of satellites
}

/**
 * TIME sync record from SENSOR.CSV
 * Links sensor timestamps to GPS time
 */
export interface TIMEData {
  type: 'TIME';
  timestamp: number;      // Sensor time (seconds since boot)
  tow: number;            // GPS Time of Week (seconds)
  week: number;           // GPS week number
}

/**
 * Parsed TRACK.CSV dataset
 */
export interface TrackDataset {
  gnssData: GNSSData[];
  firmwareVersion: string | null;
  deviceId: string | null;
  sessionId: string | null;
}

/**
 * Time synchronization result
 */
export interface TimeSyncResult {
  success: boolean;
  errorMessage?: string;
  
  // The computed offset: sensorTime + offset = gpsEpochMs
  sensorToGpsOffsetMs: number;
  
  // Overlapping time range in sensor time (seconds)
  overlapStartSensorTime: number;
  overlapEndSensorTime: number;
  
  // Number of GPS samples in overlap
  matchedGpsCount: number;
}

/**
 * GPS data converted to NWU frame with local position (meters from origin)
 * This is what we'll display on charts alongside sensor integration
 */
export interface GPSIntegrationPoint {
  sensorTime: number;     // Aligned sensor timestamp (seconds)
  
  // Raw velocity in NWU frame (m/s) - directly from GPS
  velNorth: number;
  velWest: number;
  velUp: number;
  
  // Smoothed velocity in NWU frame (m/s) - after SG filtering
  smoothVelNorth: number;
  smoothVelWest: number;
  smoothVelUp: number;
  
  // Position in local NWU frame (meters from first GPS fix)
  posNorth: number;
  posWest: number;
  posUp: number;
  
  // Derived from raw velocity
  horizontalSpeed: number;
  horizontalDistance: number;
  
  // Derived from smoothed velocity
  smoothHorizontalSpeed: number;
  
  // Acceleration in NWU frame (m/s²) - derived from smoothed velocity
  accelNorth: number;
  accelWest: number;
  accelUp: number;
  
  // Original GPS data for reference
  lat: number;
  lon: number;
  hMSL: number;
  hAcc: number;
  vAcc: number;
  numSV: number;
}

/**
 * Complete GPS integration result for charting
 */
export interface GPSIntegrationResult {
  points: GPSIntegrationPoint[];
  
  // Origin for local coordinate conversion
  originLat: number;
  originLon: number;
  originAlt: number;
  
  // Time range
  startSensorTime: number;
  endSensorTime: number;
}
