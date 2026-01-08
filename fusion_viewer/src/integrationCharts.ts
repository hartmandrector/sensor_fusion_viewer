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
 */

import { Chart, ChartConfiguration, registerables } from 'chart.js';
import { IntegrationResult } from './accelerationIntegration';
import { debug } from './constants';

// Register Chart.js components
Chart.register(...registerables);

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
  
  const config: ChartConfiguration = {
    type: 'line',
    data: {
      labels: data.time.map(t => t.toFixed(2)),
      datasets: [{
        label: `${label} (${unit})`,
        data: values,
        borderColor: '#4488ff',
        backgroundColor: 'rgba(68, 136, 255, 0.1)',
        borderWidth: 1,
        pointRadius: 0,
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
        }
      },
      scales: {
        x: {
          title: { display: true, text: 'Time (s)', color: '#ccc' },
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
  
  componentChart = new Chart(canvas, config);
  return componentChart;
}

/**
 * Create or update the top-down position chart (North vs West)
 */
function createTopDownChart(
  canvas: HTMLCanvasElement,
  data: IntegrationResult
): Chart {
  if (topDownChart) {
    topDownChart.destroy();
  }
  
  // Create scatter data points
  const points = data.posNorth.map((n, i) => ({
    x: data.posWest[i],
    y: n
  }));
  
  const config: ChartConfiguration = {
    type: 'scatter',
    data: {
      datasets: [{
        label: 'Position',
        data: points,
        borderColor: '#44ff44',
        backgroundColor: 'rgba(68, 255, 68, 0.5)',
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
      maintainAspectRatio: false,
      animation: false,
      plugins: {
        title: {
          display: true,
          text: 'Top-Down View (Position)',
          color: '#fff'
        },
        legend: {
          labels: { color: '#ccc' }
        }
      },
      scales: {
        x: {
          title: { display: true, text: 'West (m)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        },
        y: {
          title: { display: true, text: 'North (m)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        }
      }
    }
  };
  
  topDownChart = new Chart(canvas, config);
  return topDownChart;
}

/**
 * Create or update the profile chart (Horizontal Distance vs Altitude)
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
  
  const config: ChartConfiguration = {
    type: 'scatter',
    data: {
      datasets: [{
        label: 'Profile',
        data: points,
        borderColor: '#ff8844',
        backgroundColor: 'rgba(255, 136, 68, 0.5)',
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
          title: { display: true, text: 'Horizontal Distance (m)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        },
        y: {
          title: { display: true, text: 'Altitude (m)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        }
      }
    }
  };
  
  profileChart = new Chart(canvas, config);
  return profileChart;
}

/**
 * Create or update the speed chart (Horizontal Speed vs Vertical Speed)
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
          title: { display: true, text: 'Horizontal Speed (m/s)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        },
        y: {
          title: { display: true, text: 'Vertical Speed (m/s)', color: '#ccc' },
          ticks: { color: '#888' },
          grid: { color: '#333' }
        }
      }
    }
  };
  
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
