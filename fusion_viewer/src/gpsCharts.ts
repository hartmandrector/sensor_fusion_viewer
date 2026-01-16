/**
 * GPS Charts Integration Module
 * 
 * Extends the integration charts to display GPS data alongside sensor fusion integration.
 * Provides functions to add GPS traces to existing charts.
 * 
 * Chart Colors:
 * - Sensor Fusion Integration: Original colors (blue, green, orange, magenta)
 * - GPS Ground Truth: Cyan/teal (#00ffff)
 * - GPS-Fused Integration: Yellow/gold (#ffcc00) - future
 */

import { Chart, ChartDataset } from 'chart.js';
import { GPSIntegrationResult } from './gpsTypes';
import { gpsState, hasGPSData, getGPSDataForCharts } from './gpsIntegration';

// ============================================================================
// Types
// ============================================================================

type ComponentType = 
  | 'pos-north' | 'pos-west' | 'pos-up'
  | 'vel-north' | 'vel-west' | 'vel-up'
  | 'accel-north' | 'accel-west' | 'accel-up';

// GPS line color (blue for ground truth)
const GPS_COLOR = '#4488ff';
const GPS_COLOR_ALPHA = 'rgba(68, 136, 255, 0.7)';

// ============================================================================
// GPS Data Extraction
// ============================================================================

/**
 * Extract component data from GPS integration result for charting
 */
export function getGPSComponentData(
  gpsResult: GPSIntegrationResult,
  component: ComponentType
): { time: number[]; values: number[]; label: string; unit: string } {
  const points = gpsResult.points;
  const time = points.map(p => p.sensorTime);
  
  let values: number[];
  let label: string;
  let unit: string;
  
  switch (component) {
    case 'pos-north':
      values = points.map(p => p.posNorth);
      label = 'GPS Position North';
      unit = 'm';
      break;
    case 'pos-west':
      values = points.map(p => p.posWest);
      label = 'GPS Position West';
      unit = 'm';
      break;
    case 'pos-up':
      values = points.map(p => p.posUp);
      label = 'GPS Position Up';
      unit = 'm';
      break;
    case 'vel-north':
      values = points.map(p => p.velNorth);
      label = 'GPS Velocity North';
      unit = 'm/s';
      break;
    case 'vel-west':
      values = points.map(p => p.velWest);
      label = 'GPS Velocity West';
      unit = 'm/s';
      break;
    case 'vel-up':
      values = points.map(p => p.velUp);
      label = 'GPS Velocity Up';
      unit = 'm/s';
      break;
    case 'accel-north':
      values = points.map(p => p.accelNorth);
      label = 'GPS Acceleration North';
      unit = 'm/s²';
      break;
    case 'accel-west':
      values = points.map(p => p.accelWest);
      label = 'GPS Acceleration West';
      unit = 'm/s²';
      break;
    case 'accel-up':
      values = points.map(p => p.accelUp);
      label = 'GPS Acceleration Up';
      unit = 'm/s²';
      break;
  }
  
  return { time, values, label, unit };
}

/**
 * Extract smoothed velocity data from GPS integration result
 * Only returns data for velocity components, null otherwise
 */
export function getGPSSmoothVelocityData(
  gpsResult: GPSIntegrationResult,
  component: ComponentType
): { time: number[]; values: number[]; label: string; unit: string } | null {
  const points = gpsResult.points;
  const time = points.map(p => p.sensorTime);
  
  let values: number[];
  let label: string;
  const unit = 'm/s';
  
  switch (component) {
    case 'vel-north':
      values = points.map(p => p.smoothVelNorth);
      label = 'GPS Smooth Vel North';
      break;
    case 'vel-west':
      values = points.map(p => p.smoothVelWest);
      label = 'GPS Smooth Vel West';
      break;
    case 'vel-up':
      values = points.map(p => p.smoothVelUp);
      label = 'GPS Smooth Vel Up';
      break;
    default:
      // Only velocity components have smoothed data
      return null;
  }
  
  return { time, values, label, unit };
}

/**
 * Create GPS dataset for component chart (time series)
 */
export function createGPSComponentDataset(
  gpsResult: GPSIntegrationResult,
  component: ComponentType
): ChartDataset<'line'> | null {
  const { time, values, label } = getGPSComponentData(gpsResult, component);
  
  if (values.length === 0) {
    return null;
  }
  
  // For component chart, we need x,y format for scatter/line
  const data = time.map((t, i) => ({ x: t, y: values[i] }));
  
  return {
    label,
    data: data as any,
    borderColor: GPS_COLOR,
    backgroundColor: GPS_COLOR_ALPHA,
    borderWidth: 1,
    pointRadius: 1,
    fill: false,
    tension: 0,
    order: 0 // Draw on top
  };
}

/**
 * Create GPS dataset for top-down chart (North vs West position)
 */
export function createGPSTopDownDataset(
  gpsResult: GPSIntegrationResult
): ChartDataset<'scatter'> {
  const points = gpsResult.points.map(p => ({
    x: p.posWest,
    y: p.posNorth
  }));
  
  return {
    label: 'GPS Position',
    data: points,
    borderColor: GPS_COLOR,
    backgroundColor: GPS_COLOR_ALPHA,
    pointRadius: 1,
    showLine: true,
    borderWidth: 1,
    order: 0
  };
}

/**
 * Create GPS dataset for profile chart (Horizontal Distance vs Altitude)
 */
export function createGPSProfileDataset(
  gpsResult: GPSIntegrationResult
): ChartDataset<'scatter'> {
  const points = gpsResult.points.map(p => ({
    x: p.horizontalDistance,
    y: p.posUp
  }));
  
  return {
    label: 'GPS Profile',
    data: points,
    borderColor: GPS_COLOR,
    backgroundColor: GPS_COLOR_ALPHA,
    pointRadius: 1,
    showLine: true,
    borderWidth: 1,
    order: 0
  };
}

/**
 * Create GPS dataset for speed chart (Horizontal Speed vs Vertical Speed)
 */
export function createGPSSpeedDataset(
  gpsResult: GPSIntegrationResult
): ChartDataset<'scatter'> {
  const points = gpsResult.points.map(p => ({
    x: p.horizontalSpeed,
    y: p.velUp
  }));
  
  return {
    label: 'GPS Speed',
    data: points,
    borderColor: GPS_COLOR,
    backgroundColor: GPS_COLOR_ALPHA,
    pointRadius: 1,
    showLine: true,
    borderWidth: 1,
    order: 0
  };
}

/**
 * Create GPS smoothed speed dataset for speed chart (Smooth Horizontal Speed vs Smooth Vertical Speed)
 */
export function createGPSSmoothSpeedDataset(
  gpsResult: GPSIntegrationResult
): ChartDataset<'scatter'> {
  const points = gpsResult.points.map(p => ({
    x: p.smoothHorizontalSpeed,
    y: p.smoothVelUp
  }));
  
  return {
    label: 'GPS Smooth Speed',
    data: points,
    borderColor: '#00ff88',
    backgroundColor: 'rgba(0, 255, 136, 0.3)',
    pointRadius: 1,
    showLine: true,
    borderWidth: 1,
    order: 0
  };
}

/**
 * Create GPS start marker for top-down chart
 */
export function createGPSStartMarker(
  gpsResult: GPSIntegrationResult
): ChartDataset<'scatter'> | null {
  if (gpsResult.points.length === 0) return null;
  
  const first = gpsResult.points[0];
  return {
    label: 'GPS Start',
    data: [{ x: first.posWest, y: first.posNorth }],
    borderColor: '#00ffff',
    backgroundColor: '#00ffff',
    pointRadius: 8,
    pointStyle: 'triangle',
    order: -1
  };
}

// ============================================================================
// Chart Update Functions
// ============================================================================

/**
 * Add GPS datasets to an existing chart
 * Returns true if GPS data was added
 */
export function addGPSToChart(
  chart: Chart,
  chartType: 'component' | 'topDown' | 'profile' | 'speed',
  component?: ComponentType
): boolean {
  if (!hasGPSData()) {
    return false;
  }
  
  const gpsResult = getGPSDataForCharts();
  if (!gpsResult) {
    return false;
  }
  
  let gpsDataset: ChartDataset<any> | null = null;
  
  switch (chartType) {
    case 'component':
      if (component) {
        gpsDataset = createGPSComponentDataset(gpsResult, component);
      }
      break;
    case 'topDown':
      gpsDataset = createGPSTopDownDataset(gpsResult);
      // Also add start marker
      const startMarker = createGPSStartMarker(gpsResult);
      if (startMarker) {
        chart.data.datasets.push(startMarker);
      }
      break;
    case 'profile':
      gpsDataset = createGPSProfileDataset(gpsResult);
      break;
    case 'speed':
      gpsDataset = createGPSSpeedDataset(gpsResult);
      break;
  }
  
  if (gpsDataset) {
    chart.data.datasets.push(gpsDataset);
    return true;
  }
  
  return false;
}

/**
 * Remove GPS datasets from a chart
 */
export function removeGPSFromChart(chart: Chart): void {
  chart.data.datasets = chart.data.datasets.filter(ds => {
    const label = ds.label?.toLowerCase() || '';
    return !label.includes('gps');
  });
}

/**
 * Check if GPS data should be included in charts
 */
export function shouldIncludeGPS(): boolean {
  return gpsState.isLoaded && gpsState.integrationResult !== null;
}
