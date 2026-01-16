/**
 * Timestamp synchronization between SENSOR.CSV and TRACK.CSV
 * 
 * The key insight is that SENSOR.CSV contains $TIME entries that provide
 * the link between sensor timestamps (seconds since boot) and GPS time
 * (week number + time of week).
 * 
 * $TIME format: $TIME,sensorTime,tow,week
 * Example: $TIME,152654.461,227191.000,2377
 * 
 * GPS timestamps in TRACK.CSV are ISO 8601 format which can be converted
 * to GPS week/TOW for alignment.
 */

import { TIMEData, GNSSData, TimeSyncResult, TrackDataset } from './gpsTypes';
import { SensorDataset, IMUData } from './csvParser';

// GPS epoch: January 6, 1980 00:00:00 UTC
const GPS_EPOCH_MS = Date.UTC(1980, 0, 6, 0, 0, 0, 0);
const SECONDS_PER_WEEK = 604800;
const MS_PER_SECOND = 1000;

/**
 * Parse $TIME entries from raw CSV content
 * This extracts the timing synchronization data that links sensor time to GPS time
 */
export function parseTIMEEntries(csvContent: string): TIMEData[] {
  const lines = csvContent.split('\n');
  const timeEntries: TIMEData[] = [];
  
  for (const line of lines) {
    const trimmed = line.trim();
    if (trimmed.startsWith('$TIME,')) {
      const parts = trimmed.split(',');
      if (parts.length >= 4) {
        timeEntries.push({
          type: 'TIME',
          timestamp: parseFloat(parts[1]),  // Sensor time
          tow: parseFloat(parts[2]),        // Time of week (seconds)
          week: parseInt(parts[3], 10)      // GPS week number
        });
      }
    }
  }
  
  console.log(`Found ${timeEntries.length} $TIME entries in SENSOR.CSV`);
  return timeEntries;
}

/**
 * Convert GPS week and time-of-week to milliseconds since Unix epoch
 */
export function gpsWeekTowToUnixMs(week: number, tow: number): number {
  const gpsMs = (week * SECONDS_PER_WEEK + tow) * MS_PER_SECOND;
  return GPS_EPOCH_MS + gpsMs;
}

/**
 * Convert milliseconds since Unix epoch to GPS week and time-of-week
 */
export function unixMsToGpsWeekTow(unixMs: number): { week: number; tow: number } {
  const gpsMs = unixMs - GPS_EPOCH_MS;
  const totalSeconds = gpsMs / MS_PER_SECOND;
  const week = Math.floor(totalSeconds / SECONDS_PER_WEEK);
  const tow = totalSeconds - (week * SECONDS_PER_WEEK);
  return { week, tow };
}

/**
 * Compute the time synchronization between sensor data and GPS data
 * 
 * Returns the offset such that: sensorTime + offset = gpsEpochMs
 * This allows us to convert any sensor timestamp to absolute GPS time
 */
export function computeTimeSync(
  timeEntries: TIMEData[],
  trackDataset: TrackDataset
): TimeSyncResult {
  
  if (timeEntries.length === 0) {
    return {
      success: false,
      errorMessage: 'No $TIME entries found in SENSOR.CSV. Cannot synchronize timestamps.',
      sensorToGpsOffsetMs: 0,
      overlapStartSensorTime: 0,
      overlapEndSensorTime: 0,
      matchedGpsCount: 0
    };
  }
  
  if (trackDataset.gnssData.length === 0) {
    return {
      success: false,
      errorMessage: 'No GNSS data found in TRACK.CSV.',
      sensorToGpsOffsetMs: 0,
      overlapStartSensorTime: 0,
      overlapEndSensorTime: 0,
      matchedGpsCount: 0
    };
  }
  
  // Use the first $TIME entry to establish the offset
  // sensorTime (seconds) + offset = gpsEpochMs (milliseconds)
  const firstTime = timeEntries[0];
  const gpsEpochMs = gpsWeekTowToUnixMs(firstTime.week, firstTime.tow);
  const sensorTimeMs = firstTime.timestamp * MS_PER_SECOND;
  const offsetMs = gpsEpochMs - sensorTimeMs;
  
  console.log('Time sync calculation:');
  console.log(`  First $TIME: sensor=${firstTime.timestamp}s, week=${firstTime.week}, tow=${firstTime.tow}s`);
  console.log(`  GPS epoch ms: ${gpsEpochMs} (${new Date(gpsEpochMs).toISOString()})`);
  console.log(`  Offset: ${offsetMs}ms`);
  
  // Get GPS time range
  const gpsTimestamps = trackDataset.gnssData.map(g => g.timestamp);
  const gpsStartMs = Math.min(...gpsTimestamps);
  const gpsEndMs = Math.max(...gpsTimestamps);
  
  console.log(`  GPS range: ${new Date(gpsStartMs).toISOString()} to ${new Date(gpsEndMs).toISOString()}`);
  
  // Convert GPS time range to sensor time
  // sensorTime = (gpsEpochMs - offsetMs) / 1000
  const gpsStartSensorTime = (gpsStartMs - offsetMs) / MS_PER_SECOND;
  const gpsEndSensorTime = (gpsEndMs - offsetMs) / MS_PER_SECOND;
  
  console.log(`  GPS in sensor time: ${gpsStartSensorTime.toFixed(3)}s to ${gpsEndSensorTime.toFixed(3)}s`);
  
  // Check if there's any overlap (we'd need sensor data range too, but for now assume there is)
  if (gpsEndSensorTime < 0) {
    return {
      success: false,
      errorMessage: 'GPS data is from before sensor recording started. No overlap.',
      sensorToGpsOffsetMs: offsetMs,
      overlapStartSensorTime: 0,
      overlapEndSensorTime: 0,
      matchedGpsCount: 0
    };
  }
  
  return {
    success: true,
    sensorToGpsOffsetMs: offsetMs,
    overlapStartSensorTime: gpsStartSensorTime,
    overlapEndSensorTime: gpsEndSensorTime,
    matchedGpsCount: trackDataset.gnssData.length
  };
}

/**
 * Apply time synchronization to GPS data, setting sensorTimestamp on each entry
 */
export function applyTimeSyncToGPS(
  trackDataset: TrackDataset,
  syncResult: TimeSyncResult
): void {
  if (!syncResult.success) {
    return;
  }
  
  for (const gnss of trackDataset.gnssData) {
    // Convert GPS epoch time to sensor time
    // sensorTime = (gpsEpochMs - offsetMs) / 1000
    gnss.sensorTimestamp = (gnss.timestamp - syncResult.sensorToGpsOffsetMs) / MS_PER_SECOND;
  }
  
  console.log('Applied time sync to GPS data');
  if (trackDataset.gnssData.length > 0) {
    const first = trackDataset.gnssData[0];
    const last = trackDataset.gnssData[trackDataset.gnssData.length - 1];
    console.log(`  First GPS: ${first.isoTime} -> sensor time ${first.sensorTimestamp?.toFixed(3)}s`);
    console.log(`  Last GPS: ${last.isoTime} -> sensor time ${last.sensorTimestamp?.toFixed(3)}s`);
  }
}

/**
 * Find GPS entries that fall within a sensor time range
 */
export function findGPSInRange(
  gnssData: GNSSData[],
  startSensorTime: number,
  endSensorTime: number
): GNSSData[] {
  return gnssData.filter(g => 
    g.sensorTimestamp !== null &&
    g.sensorTimestamp >= startSensorTime &&
    g.sensorTimestamp <= endSensorTime
  );
}

/**
 * Check if sensor dataset and track dataset have overlapping time ranges
 */
export function checkTimeOverlap(
  sensorDataset: SensorDataset,
  trackDataset: TrackDataset,
  syncResult: TimeSyncResult
): { hasOverlap: boolean; message: string } {
  
  if (!syncResult.success) {
    return { hasOverlap: false, message: syncResult.errorMessage || 'Sync failed' };
  }
  
  // Get sensor time range from readings (filter to IMU data)
  const imuReadings = sensorDataset.readings.filter(r => r.type === 'IMU') as IMUData[];
  if (imuReadings.length === 0) {
    return { hasOverlap: false, message: 'No IMU data in sensor file' };
  }
  
  const sensorStart = imuReadings[0].timestamp;
  const sensorEnd = imuReadings[imuReadings.length - 1].timestamp;
  
  const gpsStart = syncResult.overlapStartSensorTime;
  const gpsEnd = syncResult.overlapEndSensorTime;
  
  // Check for overlap
  const overlapStart = Math.max(sensorStart, gpsStart);
  const overlapEnd = Math.min(sensorEnd, gpsEnd);
  
  if (overlapStart >= overlapEnd) {
    return {
      hasOverlap: false,
      message: `No time overlap. Sensor: ${sensorStart.toFixed(1)}s-${sensorEnd.toFixed(1)}s, GPS: ${gpsStart.toFixed(1)}s-${gpsEnd.toFixed(1)}s`
    };
  }
  
  const overlapDuration = overlapEnd - overlapStart;
  const gpsInRange = findGPSInRange(trackDataset.gnssData, overlapStart, overlapEnd);
  
  return {
    hasOverlap: true,
    message: `Matched ${gpsInRange.length} GPS samples over ${overlapDuration.toFixed(1)}s overlap`
  };
}
