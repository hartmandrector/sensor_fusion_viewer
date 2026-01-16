/**
 * Parser for FlySight TRACK.CSV files containing GPS data
 */

import { GNSSData, TrackDataset } from './gpsTypes';

/**
 * Parse ISO 8601 timestamp to milliseconds since Unix epoch
 * Format: 2025-07-29T15:06:28.150Z
 */
function parseISOTimestamp(isoString: string): number {
  return new Date(isoString).getTime();
}

/**
 * Parse a single GNSS data line
 * Format: $GNSS,time,lat,lon,hMSL,velN,velE,velD,hAcc,vAcc,sAcc,numSV
 */
function parseGNSSLine(parts: string[]): GNSSData | null {
  if (parts.length < 12) {
    console.warn('GNSS line has insufficient fields:', parts);
    return null;
  }

  const isoTime = parts[1];
  const timestamp = parseISOTimestamp(isoTime);
  
  if (isNaN(timestamp)) {
    console.warn('Invalid timestamp in GNSS line:', isoTime);
    return null;
  }

  return {
    type: 'GNSS',
    isoTime,
    timestamp,
    sensorTimestamp: null, // Will be computed during sync
    lat: parseFloat(parts[2]),
    lon: parseFloat(parts[3]),
    hMSL: parseFloat(parts[4]),
    velN: parseFloat(parts[5]),
    velE: parseFloat(parts[6]),
    velD: parseFloat(parts[7]),
    hAcc: parseFloat(parts[8]),
    vAcc: parseFloat(parts[9]),
    sAcc: parseFloat(parts[10]),
    numSV: parseInt(parts[11], 10)
  };
}

/**
 * Parse a complete TRACK.CSV file
 */
export function parseTrackCSV(content: string): TrackDataset {
  const lines = content.split('\n');
  
  const dataset: TrackDataset = {
    gnssData: [],
    firmwareVersion: null,
    deviceId: null,
    sessionId: null
  };

  let inDataSection = false;

  for (const line of lines) {
    const trimmed = line.trim();
    if (!trimmed) continue;

    // Check for data section start
    if (trimmed === '$DATA') {
      inDataSection = true;
      continue;
    }

    // Parse variable declarations
    if (trimmed.startsWith('$VAR,')) {
      const parts = trimmed.split(',');
      if (parts.length >= 3) {
        const varName = parts[1];
        const varValue = parts[2];
        
        switch (varName) {
          case 'FIRMWARE_VER':
            dataset.firmwareVersion = varValue;
            break;
          case 'DEVICE_ID':
            dataset.deviceId = varValue;
            break;
          case 'SESSION_ID':
            dataset.sessionId = varValue;
            break;
        }
      }
      continue;
    }

    // Skip header lines
    if (trimmed.startsWith('$FLYS') || 
        trimmed.startsWith('$COL') || 
        trimmed.startsWith('$UNIT')) {
      continue;
    }

    // Parse data lines
    if (inDataSection && trimmed.startsWith('$GNSS,')) {
      const parts = trimmed.split(',');
      const gnssData = parseGNSSLine(parts);
      if (gnssData) {
        dataset.gnssData.push(gnssData);
      }
    }
  }

  console.log(`Parsed TRACK.CSV: ${dataset.gnssData.length} GNSS entries`);
  
  return dataset;
}

/**
 * Get the time range of GPS data in milliseconds
 */
export function getGPSTimeRange(dataset: TrackDataset): { startMs: number; endMs: number } | null {
  if (dataset.gnssData.length === 0) {
    return null;
  }
  
  const timestamps = dataset.gnssData.map(g => g.timestamp);
  return {
    startMs: Math.min(...timestamps),
    endMs: Math.max(...timestamps)
  };
}
