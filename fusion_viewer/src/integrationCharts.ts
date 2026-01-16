/**
 * Integration Charts Module
 * 
 * Provides 4 chart views for visualizing integrated acceleration data:
 * 1. Component Chart - Time vs selected component (dropdown)
 * 2. Top-Down Chart - Position North vs Position West
 * 3. Profile Chart - Horizontal Distance vs Altitude
 * 4. Speed Chart - Horizontal Speed vs Vertical Speed
 * 
 * Uses Chart.js for rendering.
 * 
 * Chart Lines:
 * - Blue: Sensor Fusion Integration
 * - Cyan: GPS Ground Truth (when loaded)
 * - Yellow/Gold: GPS-Fused Integration (future)
 */

import { Chart, ChartConfiguration, ChartDataset, registerables } from 'chart.js';
import zoomPlugin from 'chartjs-plugin-zoom';
import { IntegrationResult } from './accelerationIntegration';
import { debug } from './constants';
import { 
  shouldIncludeGPS, 
  createGPSTopDownDataset, 
  createGPSProfileDataset, 
  createGPSSpeedDataset,
  createGPSSmoothSpeedDataset,
  createGPSStartMarker,
  getGPSComponentData,
  getGPSSmoothVelocityData
} from './gpsCharts';
import { getGPSDataForCharts } from './gpsIntegration';

// Register Chart.js components and plugins
Chart.register(...registerables, zoomPlugin);

// ============================================================================
// Types
// ============================================================================

type ComponentType = 
  | 'pos-north' | 'pos-west' | 'pos-up'
  | 'vel-north' | 'vel-west' | 'vel-up'
  | 'accel-north' | 'accel-west' | 'accel-up';

// ============================================================================
// Chart Instances
// ============================================================================

let componentChart: Chart | null = null;
let topDownChart: Chart | null = null;
let profileChart: Chart | null = null;
let speedChart: Chart | null = null;

// Current integration result
let currentIntegration: IntegrationResult | null = null;

// ============================================================================
// Chart Creation
// ============================================================================

/**
 * Create or update the component time-series chart
 * 
 * Shows data starting from the integration start time.
 * Time axis is relative (seconds from start of data).
 */
function createComponentChart(
  canvas: HTMLCanvasElement,
  data: IntegrationResult,
  component: ComponentType
): Chart {
  // Destroy existing chart
  if (componentChart) {
    componentChart.destroy();
  }
  
  // Get data for selected component
  let values: number[];
  let label: string;
  let unit: string;
  
  switch (component) {
    case 'pos-north':
      values = data.posNorth;
      label = 'Position North';
      unit = 'm';
      break;
    case 'pos-west':
      values = data.posWest;
      label = 'Position West';
      unit = 'm';
      break;
    case 'pos-up':
      values = data.posUp;
      label = 'Position Up';
      unit = 'm';
      break;
    case 'vel-north':
      values = data.velNorth;
      label = 'Velocity North';
      unit = 'm/s';
      break;
    case 'vel-west':
      values = data.velWest;
      label = 'Velocity West';
      unit = 'm/s';
      break;
    case 'vel-up':
      values = data.velUp;
      label = 'Velocity Up';
      unit = 'm/s';
      break;
    case 'accel-north':
      values = data.accelNorth;
      label = 'Acceleration North';
      unit = 'm/s²';
      break;
    case 'accel-west':
      values = data.accelWest;
      label = 'Acceleration West';
      unit = 'm/s²';
      break;
    case 'accel-up':
      values = data.accelUp;
      label = 'Acceleration Up';
      unit = 'm/s²';
      break;
  }
  
  // Filter data to only show from startIndex onwards (integration start time)
  const startIdx = data.startIndex;
  const filteredTime = data.time.slice(startIdx);
  const filteredValues = values.slice(startIdx);
  
  // Get the start time offset for relative time display
  const timeOffset = filteredTime.length > 0 ? filteredTime[0] : 0;
  
  // Create scatter data for integration with RELATIVE time (x,y format)
  const integrationData = filteredTime.map((t, i) => ({ x: t - timeOffset, y: filteredValues[i] }));
  
  // Determine time bounds (relative: 0 to duration)
  const minTime = 0;
  const maxTime = filteredTime.length > 0 ? filteredTime[filteredTime.length - 1] - timeOffset : 1;
  
  const config: ChartConfiguration = {
    type: 'scatter',
    data: {
      datasets: [{
        label: `${label} (${unit})`,
        data: integrationData,
        borderColor: '#ff44ff',
        backgroundColor: 'rgba(255, 68, 255, 0.1)',
        borderWidth: 1,
        pointRadius: 0,
        showLine: true,
        fill: true
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      plugins: {
        title: {
          display: true,
          text: `${label} vs Time`,
          color: '#fff'
        },
        legend: {
          labels: { color: '#ccc' }
        },
        zoom: {
          pan: {
            enabled: true,
            mode: 'x'
          },
          zoom: {
            wheel: {
              enabled: true
            },
            pinch: {
              enabled: true
            },
            drag: {
              enabled: true,
              backgroundColor: 'rgba(68, 136, 255, 0.3)',
              borderColor: '#4488ff',
              borderWidth: 1
            },
            mode: 'x'
          }
        }
      },
      scales: {
        x: {
          type: 'linear',
          min: minTime,
          max: maxTime,
          title: { display: true, text: 'Relative Time (s)', color: '#ccc' },
          ticks: { color: '#888', maxTicksLimit: 10 },
          grid: { color: '#333' }
        },
        y: {
          title: { display: true, text: `${label} (${unit})`, color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        }
      }
    }
  };
  
  // Add GPS data if available (filter to show only from start time onwards)
  if (shouldIncludeGPS()) {
    const gpsResult = getGPSDataForCharts();
    if (gpsResult) {
      const gpsData = getGPSComponentData(gpsResult, component);
      if (gpsData.values.length > 0) {
        // Filter GPS data to only show from integration start time onwards
        // Convert to relative time using same offset
        const gpsFiltered: { x: number; y: number }[] = [];
        for (let i = 0; i < gpsData.time.length; i++) {
          if (gpsData.time[i] >= timeOffset) {
            gpsFiltered.push({ x: gpsData.time[i] - timeOffset, y: gpsData.values[i] });
          }
        }
        
        if (gpsFiltered.length > 0) {
          const gpsDataset: ChartDataset<'scatter'> = {
            label: `GPS ${label}`,
            data: gpsFiltered,
            borderColor: '#4488ff',
            backgroundColor: 'rgba(68, 136, 255, 0.3)',
            borderWidth: 1,
            pointRadius: 1,
            showLine: true,
            fill: false
          };
          config.data.datasets.push(gpsDataset as any);
        }
      }
      
      // Add smoothed GPS velocity if this is a velocity component
      const smoothData = getGPSSmoothVelocityData(gpsResult, component);
      if (smoothData && smoothData.values.length > 0) {
        const smoothFiltered: { x: number; y: number }[] = [];
        for (let i = 0; i < smoothData.time.length; i++) {
          if (smoothData.time[i] >= timeOffset) {
            smoothFiltered.push({ x: smoothData.time[i] - timeOffset, y: smoothData.values[i] });
          }
        }
        
        if (smoothFiltered.length > 0) {
          const smoothDataset: ChartDataset<'scatter'> = {
            label: smoothData.label,
            data: smoothFiltered,
            borderColor: '#00ff88',  // Green for smoothed
            backgroundColor: 'rgba(0, 255, 136, 0.3)',
            borderWidth: 1,
            pointRadius: 1,
            showLine: true,
            fill: false
          };
          config.data.datasets.push(smoothDataset as any);
        }
      }
    }
  }
  
  componentChart = new Chart(canvas, config);
  return componentChart;
}

/**
 * Create or update the top-down position chart (North vs West)
 * 
 * Note: X axis is reversed so that East appears on the right (positive West on left).
 * When GPS data is present, bounds are calculated from GPS only (ground truth).
 */
function createTopDownChart(
  canvas: HTMLCanvasElement,
  data: IntegrationResult
): Chart {
  if (topDownChart) {
    topDownChart.destroy();
  }
  
  // Calculate square size based on container
  const wrapper = canvas.parentElement;
  if (wrapper) {
    const wrapperWidth = wrapper.clientWidth;
    const wrapperHeight = wrapper.clientHeight;
    const squareSize = Math.min(wrapperWidth, wrapperHeight);
    // Set canvas display size (CSS pixels)
    canvas.style.width = `${squareSize}px`;
    canvas.style.height = `${squareSize}px`;
  }
  
  // Create scatter data points
  const points = data.posNorth.map((n, i) => ({
    x: data.posWest[i],
    y: n
  }));
  
  // Check if GPS data is available for bounds calculation
  let useGPSBounds = false;
  let gpsResult: ReturnType<typeof getGPSDataForCharts> = null;
  let boundsMinX = Infinity, boundsMaxX = -Infinity;
  let boundsMinY = Infinity, boundsMaxY = -Infinity;
  
  if (shouldIncludeGPS()) {
    gpsResult = getGPSDataForCharts();
    if (gpsResult && gpsResult.points.length > 0) {
      useGPSBounds = true;
      // Calculate bounds from GPS data only (ground truth)
      for (const p of gpsResult.points) {
        boundsMinX = Math.min(boundsMinX, p.posWest);
        boundsMaxX = Math.max(boundsMaxX, p.posWest);
        boundsMinY = Math.min(boundsMinY, p.posNorth);
        boundsMaxY = Math.max(boundsMaxY, p.posNorth);
      }
    }
  }
  
  // If no GPS, calculate bounds from integration data
  if (!useGPSBounds) {
    for (let i = 0; i < data.posWest.length; i++) {
      boundsMinX = Math.min(boundsMinX, data.posWest[i]);
      boundsMaxX = Math.max(boundsMaxX, data.posWest[i]);
      boundsMinY = Math.min(boundsMinY, data.posNorth[i]);
      boundsMaxY = Math.max(boundsMaxY, data.posNorth[i]);
    }
  }
  
  // Calculate fixed aspect ratio bounds (1:1 scale for X and Y)
  const rangeX = boundsMaxX - boundsMinX;
  const rangeY = boundsMaxY - boundsMinY;
  const maxRange = Math.max(rangeX, rangeY) * 1.1; // Add 10% padding
  const centerX = (boundsMinX + boundsMaxX) / 2;
  const centerY = (boundsMinY + boundsMaxY) / 2;
  
  // Final bounds with equal aspect ratio
  const finalMinX = centerX - maxRange / 2;
  const finalMaxX = centerX + maxRange / 2;
  const finalMinY = centerY - maxRange / 2;
  const finalMaxY = centerY + maxRange / 2;
  
  const config: ChartConfiguration = {
    type: 'scatter',
    data: {
      datasets: [{
        label: 'Position',
        data: points,
        borderColor: '#ff44ff',
        backgroundColor: 'rgba(255, 68, 255, 0.5)',
        pointRadius: 1,
        showLine: true,
        borderWidth: 1
      }, {
        // Start point marker
        label: 'Start',
        data: [{ x: data.posWest[data.startIndex], y: data.posNorth[data.startIndex] }],
        borderColor: '#ffff00',
        backgroundColor: '#ffff00',
        pointRadius: 8,
        pointStyle: 'circle'
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: true,
      aspectRatio: 1, // Force 1:1 aspect ratio for undistorted top-down view
      animation: false,
      plugins: {
        title: {
          display: false // Title moved to external HTML
        },
        legend: {
          display: false // Legend moved to external HTML
        }
      },
      scales: {
        x: {
          type: 'linear',
          min: finalMinX,
          max: finalMaxX,
          reverse: true, // Reverse so East (negative West) is on the right
          title: { display: true, text: 'West (m)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        },
        y: {
          type: 'linear',
          min: finalMinY,
          max: finalMaxY,
          title: { display: true, text: 'North (m)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        }
      }
    }
  };
  
  // Add GPS data if available
  if (gpsResult && gpsResult.points.length > 0) {
    const gpsDataset = createGPSTopDownDataset(gpsResult);
    config.data.datasets.push(gpsDataset as any);
    
    const gpsStart = createGPSStartMarker(gpsResult);
    if (gpsStart) {
      config.data.datasets.push(gpsStart as any);
    }
  }
  
  topDownChart = new Chart(canvas, config);
  
  // Populate custom legend
  updateTopDownLegend(config.data.datasets as any[]);
  
  return topDownChart;
}

/**
 * Update the custom legend for the top-down chart
 */
function updateTopDownLegend(datasets: { label: string; borderColor: string; pointStyle?: string }[]): void {
  const legendContainer = document.getElementById('topDownLegend');
  if (!legendContainer) return;
  
  legendContainer.innerHTML = '';
  
  for (const dataset of datasets) {
    // Skip start point markers in legend (they're shown with their own style)
    if (dataset.label === 'Start' || dataset.label === 'GPS Start') continue;
    
    const item = document.createElement('div');
    item.className = 'top-down-legend-item';
    
    const colorBox = document.createElement('span');
    colorBox.className = 'top-down-legend-color';
    colorBox.style.backgroundColor = dataset.borderColor as string;
    
    const label = document.createElement('span');
    label.className = 'top-down-legend-label';
    label.textContent = dataset.label;
    
    item.appendChild(colorBox);
    item.appendChild(label);
    legendContainer.appendChild(item);
  }
  
  // Add start marker legend items
  const startItem = document.createElement('div');
  startItem.className = 'top-down-legend-item';
  startItem.innerHTML = `<span class="top-down-legend-color" style="background-color: #ffff00; width: 8px; height: 8px; border-radius: 50%;"></span><span class="top-down-legend-label">Start</span>`;
  legendContainer.appendChild(startItem);
}

/**
 * Create or update the profile chart (Horizontal Distance vs Altitude)
 * 
 * When GPS data is present, bounds are calculated from GPS only (ground truth).
 */
function createProfileChart(
  canvas: HTMLCanvasElement,
  data: IntegrationResult
): Chart {
  if (profileChart) {
    profileChart.destroy();
  }
  
  // Create scatter data points
  const points = data.horizontalDistance.map((h, i) => ({
    x: h,
    y: data.posUp[i]
  }));
  
  // Check if GPS data is available for bounds calculation
  let useGPSBounds = false;
  let gpsResult: ReturnType<typeof getGPSDataForCharts> = null;
  let boundsMinX = Infinity, boundsMaxX = -Infinity;
  let boundsMinY = Infinity, boundsMaxY = -Infinity;
  
  if (shouldIncludeGPS()) {
    gpsResult = getGPSDataForCharts();
    if (gpsResult && gpsResult.points.length > 0) {
      useGPSBounds = true;
      // Calculate bounds from GPS data only (ground truth)
      for (const p of gpsResult.points) {
        boundsMinX = Math.min(boundsMinX, p.horizontalDistance);
        boundsMaxX = Math.max(boundsMaxX, p.horizontalDistance);
        boundsMinY = Math.min(boundsMinY, p.posUp);
        boundsMaxY = Math.max(boundsMaxY, p.posUp);
      }
    }
  }
  
  // If no GPS, calculate bounds from integration data
  if (!useGPSBounds) {
    for (let i = 0; i < data.horizontalDistance.length; i++) {
      boundsMinX = Math.min(boundsMinX, data.horizontalDistance[i]);
      boundsMaxX = Math.max(boundsMaxX, data.horizontalDistance[i]);
      boundsMinY = Math.min(boundsMinY, data.posUp[i]);
      boundsMaxY = Math.max(boundsMaxY, data.posUp[i]);
    }
  }
  
  // Add padding to bounds
  const padX = (boundsMaxX - boundsMinX) * 0.05 || 10;
  const padY = (boundsMaxY - boundsMinY) * 0.05 || 10;
  
  const config: ChartConfiguration = {
    type: 'scatter',
    data: {
      datasets: [{
        label: 'Profile',
        data: points,
        borderColor: '#ff44ff',
        backgroundColor: 'rgba(255, 68, 255, 0.5)',
        pointRadius: 1,
        showLine: true,
        borderWidth: 1
      }, {
        // Start point marker
        label: 'Start',
        data: [{ x: data.horizontalDistance[data.startIndex], y: data.posUp[data.startIndex] }],
        borderColor: '#ffff00',
        backgroundColor: '#ffff00',
        pointRadius: 8,
        pointStyle: 'circle'
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      plugins: {
        title: {
          display: true,
          text: 'Profile View',
          color: '#fff'
        },
        legend: {
          labels: { color: '#ccc' }
        }
      },
      scales: {
        x: {
          type: 'linear',
          min: boundsMinX - padX,
          max: boundsMaxX + padX,
          title: { display: true, text: 'Horizontal Distance (m)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        },
        y: {
          type: 'linear',
          min: boundsMinY - padY,
          max: boundsMaxY + padY,
          title: { display: true, text: 'Altitude (m)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        }
      }
    }
  };
  
  // Add GPS data if available
  if (gpsResult && gpsResult.points.length > 0) {
    const gpsDataset = createGPSProfileDataset(gpsResult);
    config.data.datasets.push(gpsDataset as any);
  }
  
  profileChart = new Chart(canvas, config);
  return profileChart;
}

/**
 * Create or update the speed chart (Horizontal Speed vs Vertical Speed)
 * 
 * When GPS data is present, bounds are calculated from GPS only (ground truth).
 */
function createSpeedChart(
  canvas: HTMLCanvasElement,
  data: IntegrationResult
): Chart {
  if (speedChart) {
    speedChart.destroy();
  }
  
  // Create scatter data points
  const points = data.horizontalSpeed.map((h, i) => ({
    x: h,
    y: data.velUp[i]
  }));
  
  // Check if GPS data is available for bounds calculation
  let useGPSBounds = false;
  let gpsResult: ReturnType<typeof getGPSDataForCharts> = null;
  let boundsMinX = Infinity, boundsMaxX = -Infinity;
  let boundsMinY = Infinity, boundsMaxY = -Infinity;
  
  if (shouldIncludeGPS()) {
    gpsResult = getGPSDataForCharts();
    if (gpsResult && gpsResult.points.length > 0) {
      useGPSBounds = true;
      // Calculate bounds from GPS data only (ground truth)
      for (const p of gpsResult.points) {
        boundsMinX = Math.min(boundsMinX, p.horizontalSpeed);
        boundsMaxX = Math.max(boundsMaxX, p.horizontalSpeed);
        boundsMinY = Math.min(boundsMinY, p.velUp);
        boundsMaxY = Math.max(boundsMaxY, p.velUp);
      }
    }
  }
  
  // If no GPS, calculate bounds from integration data
  if (!useGPSBounds) {
    for (let i = 0; i < data.horizontalSpeed.length; i++) {
      boundsMinX = Math.min(boundsMinX, data.horizontalSpeed[i]);
      boundsMaxX = Math.max(boundsMaxX, data.horizontalSpeed[i]);
      boundsMinY = Math.min(boundsMinY, data.velUp[i]);
      boundsMaxY = Math.max(boundsMaxY, data.velUp[i]);
    }
  }
  
  // Add padding to bounds
  const padX = (boundsMaxX - boundsMinX) * 0.05 || 1;
  const padY = (boundsMaxY - boundsMinY) * 0.05 || 1;
  
  const config: ChartConfiguration = {
    type: 'scatter',
    data: {
      datasets: [{
        label: 'Speed',
        data: points,
        borderColor: '#ff44ff',
        backgroundColor: 'rgba(255, 68, 255, 0.5)',
        pointRadius: 1,
        showLine: true,
        borderWidth: 1
      }, {
        // Start point marker
        label: 'Start',
        data: [{ x: data.horizontalSpeed[data.startIndex], y: data.velUp[data.startIndex] }],
        borderColor: '#ffff00',
        backgroundColor: '#ffff00',
        pointRadius: 8,
        pointStyle: 'circle'
      }]
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      plugins: {
        title: {
          display: true,
          text: 'Speed Chart',
          color: '#fff'
        },
        legend: {
          labels: { color: '#ccc' }
        }
      },
      scales: {
        x: {
          type: 'linear',
          min: boundsMinX - padX,
          max: boundsMaxX + padX,
          title: { display: true, text: 'Horizontal Speed (m/s)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        },
        y: {
          type: 'linear',
          min: boundsMinY - padY,
          max: boundsMaxY + padY,
          title: { display: true, text: 'Vertical Speed (m/s)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        }
      }
    }
  };
  
  // Add GPS data if available
  if (gpsResult && gpsResult.points.length > 0) {
    const gpsDataset = createGPSSpeedDataset(gpsResult);
    config.data.datasets.push(gpsDataset as any);
    
    // Add smoothed GPS speed (green)
    const gpsSmoothDataset = createGPSSmoothSpeedDataset(gpsResult);
    config.data.datasets.push(gpsSmoothDataset as any);
  }
  
  speedChart = new Chart(canvas, config);
  return speedChart;
}

// ============================================================================
// Public API
// ============================================================================

/**
 * Initialize all charts with current integration data
 */
export function initializeCharts(data: IntegrationResult): void {
  currentIntegration = data;
  
  // Get canvas elements
  const componentCanvas = document.getElementById('componentChartCanvas') as HTMLCanvasElement;
  const topDownCanvas = document.getElementById('topDownChartCanvas') as HTMLCanvasElement;
  const profileCanvas = document.getElementById('profileChartCanvas') as HTMLCanvasElement;
  const speedCanvas = document.getElementById('speedChartCanvas') as HTMLCanvasElement;
  
  if (!componentCanvas || !topDownCanvas || !profileCanvas || !speedCanvas) {
    debug.error('Chart canvas elements not found');
    return;
  }
  
  // Get current component selection
  const componentSelect = document.getElementById('componentSelect') as HTMLSelectElement;
  const component = (componentSelect?.value || 'vel-up') as ComponentType;
  
  // Create all charts
  createComponentChart(componentCanvas, data, component);
  createTopDownChart(topDownCanvas, data);
  createProfileChart(profileCanvas, data);
  createSpeedChart(speedCanvas, data);
  
  debug.log('Integration charts initialized');
}

/**
 * Update component chart when dropdown selection changes
 */
export function updateComponentChart(component: ComponentType): void {
  if (!currentIntegration) return;
  
  const canvas = document.getElementById('componentChartCanvas') as HTMLCanvasElement;
  if (!canvas) return;
  
  createComponentChart(canvas, currentIntegration, component);
}

/**
 * Destroy all charts
 */
export function destroyCharts(): void {
  if (componentChart) {
    componentChart.destroy();
    componentChart = null;
  }
  if (topDownChart) {
    topDownChart.destroy();
    topDownChart = null;
  }
  if (profileChart) {
    profileChart.destroy();
    profileChart = null;
  }
  if (speedChart) {
    speedChart.destroy();
    speedChart = null;
  }
  currentIntegration = null;
}

/**
 * Check if charts are currently active
 */
export function areChartsActive(): boolean {
  return currentIntegration !== null;
}

/**
 * Refresh all charts with current data (useful when GPS data is loaded)
 */
export function refreshCharts(): void {
  if (currentIntegration) {
    initializeCharts(currentIntegration);
  }
}

// Listen for GPS data loaded event to refresh charts
if (typeof window !== 'undefined') {
  window.addEventListener('gps-data-loaded', () => {
    refreshCharts();
  });
}
