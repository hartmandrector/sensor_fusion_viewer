/**
 * FlySight 2 SENSOR.CSV Parser
 * 
 * Parses the FlySight 2 sensor log format with interleaved sensor data.
 * 
 * Format:
 * - Header lines start with $FLYS, $VAR, $COL, $UNIT
 * - $DATA marks the start of data section
 * - Data lines: $IMU, $MAG, $BARO, etc.
 */

export type SensorType = 'IMU' | 'MAG' | 'BARO' | 'HUM' | 'TIME' | 'VBAT' | 'UNKNOWN';

export interface IMUData {
  type: 'IMU';
  timestamp: number;  // seconds since boot
  wx: number;         // Gyro X (deg/s)
  wy: number;         // Gyro Y (deg/s)
  wz: number;         // Gyro Z (deg/s)
  ax: number;         // Accel X (g)
  ay: number;         // Accel Y (g)
  az: number;         // Accel Z (g)
  temperature: number; // °C
}

export interface MAGData {
  type: 'MAG';
  timestamp: number;  // seconds since boot
  x: number;          // Magnetic field X (gauss)
  y: number;          // Magnetic field Y (gauss)
  z: number;          // Magnetic field Z (gauss)
  temperature: number; // °C
}

export interface BAROData {
  type: 'BARO';
  timestamp: number;   // seconds since boot
  pressure: number;    // Pa
  temperature: number; // °C
}

export interface HUMData {
  type: 'HUM';
  timestamp: number;   // seconds since boot
  humidity: number;    // percent
  temperature: number; // °C
}

export type SensorReading = IMUData | MAGData | BAROData | HUMData;

export interface SensorDataset {
  firmwareVersion: string;
  deviceId: string;
  sessionId: string;
  readings: SensorReading[];
  
  // Computed stats
  startTime: number;
  endTime: number;
  duration: number;
  imuCount: number;
  magCount: number;
  baroCount: number;
  imuRate: number;  // Hz
  magRate: number;  // Hz
}

/**
 * Parse a FlySight 2 SENSOR.CSV file
 */
export function parseCSV(content: string): SensorDataset {
  const lines = content.split(/\r?\n/);
  
  const dataset: SensorDataset = {
    firmwareVersion: '',
    deviceId: '',
    sessionId: '',
    readings: [],
    startTime: 0,
    endTime: 0,
    duration: 0,
    imuCount: 0,
    magCount: 0,
    baroCount: 0,
    imuRate: 0,
    magRate: 0
  };
  
  let inDataSection = false;
  
  for (const line of lines) {
    const trimmed = line.trim();
    if (!trimmed) continue;
    
    // Parse header lines
    if (!inDataSection) {
      if (trimmed === '$DATA') {
        inDataSection = true;
        continue;
      }
      
      // Parse $VAR lines
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
      }
      continue;
    }
    
    // Parse data lines
    const reading = parseDataLine(trimmed);
    if (reading) {
      dataset.readings.push(reading);
      
      // Count by type
      switch (reading.type) {
        case 'IMU':
          dataset.imuCount++;
          break;
        case 'MAG':
          dataset.magCount++;
          break;
        case 'BARO':
          dataset.baroCount++;
          break;
      }
    }
  }
  
  // Compute statistics
  if (dataset.readings.length > 0) {
    dataset.startTime = dataset.readings[0].timestamp;
    dataset.endTime = dataset.readings[dataset.readings.length - 1].timestamp;
    dataset.duration = dataset.endTime - dataset.startTime;
    
    // Compute sample rates
    if (dataset.imuCount > 1 && dataset.duration > 0) {
      dataset.imuRate = Math.round(dataset.imuCount / dataset.duration);
    }
    if (dataset.magCount > 1 && dataset.duration > 0) {
      dataset.magRate = Math.round(dataset.magCount / dataset.duration);
    }
  }
  
  return dataset;
}

/**
 * Parse a single data line
 */
function parseDataLine(line: string): SensorReading | null {
  const parts = line.split(',');
  if (parts.length < 2) return null;
  
  const type = parts[0];
  
  switch (type) {
    case '$IMU':
      if (parts.length >= 9) {
        return {
          type: 'IMU',
          timestamp: parseFloat(parts[1]),
          wx: parseFloat(parts[2]),
          wy: parseFloat(parts[3]),
          wz: parseFloat(parts[4]),
          ax: parseFloat(parts[5]),
          ay: parseFloat(parts[6]),
          az: parseFloat(parts[7]),
          temperature: parseFloat(parts[8])
        };
      }
      break;
      
    case '$MAG':
      if (parts.length >= 6) {
        return {
          type: 'MAG',
          timestamp: parseFloat(parts[1]),
          x: parseFloat(parts[2]),
          y: parseFloat(parts[3]),
          z: parseFloat(parts[4]),
          temperature: parseFloat(parts[5])
        };
      }
      break;
      
    case '$BARO':
      if (parts.length >= 4) {
        return {
          type: 'BARO',
          timestamp: parseFloat(parts[1]),
          pressure: parseFloat(parts[2]),
          temperature: parseFloat(parts[3])
        };
      }
      break;
      
    case '$HUM':
      if (parts.length >= 4) {
        return {
          type: 'HUM',
          timestamp: parseFloat(parts[1]),
          humidity: parseFloat(parts[2]),
          temperature: parseFloat(parts[3])
        };
      }
      break;
  }
  
  return null;
}

/**
 * Get all readings up to a given timestamp
 */
export function getReadingsUpTo(dataset: SensorDataset, timestamp: number): SensorReading[] {
  return dataset.readings.filter(r => r.timestamp <= timestamp);
}

/**
 * Find the reading index for a given timestamp
 */
export function findReadingIndex(dataset: SensorDataset, timestamp: number): number {
  // Binary search for efficiency
  let low = 0;
  let high = dataset.readings.length - 1;
  
  while (low < high) {
    const mid = Math.floor((low + high + 1) / 2);
    if (dataset.readings[mid].timestamp <= timestamp) {
      low = mid;
    } else {
      high = mid - 1;
    }
  }
  
  return low;
}

/**
 * Get IMU readings only
 */
export function getIMUReadings(dataset: SensorDataset): IMUData[] {
  return dataset.readings.filter((r): r is IMUData => r.type === 'IMU');
}

/**
 * Get MAG readings only  
 */
export function getMAGReadings(dataset: SensorDataset): MAGData[] {
  return dataset.readings.filter((r): r is MAGData => r.type === 'MAG');
}
