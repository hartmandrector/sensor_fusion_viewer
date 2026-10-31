import { Chart, type ChartDataset } from 'chart.js';
import { computeAttitudeDiagnostics, MIN_DIAGNOSTIC_FORCE } from './attitudeDiagnostics';
import type { AttitudeDiagnosticPoint, AttitudeDiagnostics } from './attitudeDiagnostics';
import type { IntegrationResult } from './accelerationIntegration';
import type { GPSIntegrationResult } from './gpsTypes';
import { GPS_ACCELERATION_WINDOW_SIZE } from './gpsIntegration';
import { DEFAULT_GPS_SMOOTHING_WINDOWS } from './sgFilter';
import { getElements } from './uiElements';
import { estimateGPSLag } from './gpsLag';
import { debug } from './constants';

type PlotPoint = { x: number; y: number | null };
type DiagnosticChart = Chart<'line', PlotPoint[]>;

const charts: DiagnosticChart[] = [];
let diagnostics: AttitudeDiagnostics | null = null;
let currentInputs: { integration: IntegrationResult; gps: GPSIntegrationResult } | null = null;

function halfWindowSamples(): number {
  return DEFAULT_GPS_SMOOTHING_WINDOWS.reduce((sum, size) => sum + (size - 1) / 2, 0)
    + (GPS_ACCELERATION_WINDOW_SIZE - 1) / 2;
}

export function applyDiagnosticTimeShift(): void {
  if (!currentInputs) return;
  const { integration, gps } = currentInputs;
  try {
    const text = getElements().diagnosticsGpsShift.value.trim();
    const shift = text ? Number(text) : NaN;
    if (!Number.isFinite(shift) || Math.abs(shift) > 5) throw new Error('Enter a GPS shift between -5 and +5 seconds.');
    renderAttitudeDiagnostics(integration, gps);
    getElements().lagStatus.textContent = `Applied GPS shift ${shift.toFixed(2)}s to diagnostics only. Positive moves GPS later; delayed GPS needs negative. Original timestamps and integration are unchanged.`;
  } catch (error) {
    debug.error('GPS diagnostic time shift failed:', error);
    getElements().lagStatus.textContent = `Shift failed: ${error instanceof Error ? error.message : String(error)}. Previous plots are unchanged.`;
  }
}

export function estimateDiagnosticLag(): void {
  if (!currentInputs) return;
  const elements = getElements();
  try {
    const base = computeAttitudeDiagnostics(currentInputs.integration, currentInputs.gps, halfWindowSamples());
    const result = estimateGPSLag(
      base, elements.lagStart.value.trim() ? Number(elements.lagStart.value) : NaN,
      elements.lagEnd.value.trim() ? Number(elements.lagEnd.value) : NaN,
      elements.lagSignal.value === 'magnitude' ? 'magnitude' : 'up'
    );
    elements.diagnosticsGpsShift.value = result.shiftSeconds.toFixed(2);
    elements.lagStatus.textContent = `Suggested absolute GPS shift ${result.shiftSeconds.toFixed(2)}s; correlation ${result.correlation.toFixed(3)}, ${result.sampleCount} common samples. ${
      result.warnings.length ? `CAUTION: ${result.warnings.join('; ')}. ` : ''
    }Not applied: click Apply shift to preview. +/-2s search, 0.02s spacing; smoothing, bandwidth and motion can mimic delay.`;
  } catch (error) {
    debug.error('GPS lag estimation failed:', error);
    elements.lagStatus.textContent = `Lag estimate unavailable: ${error instanceof Error ? error.message : String(error)}`;
  }
}

function series(
  label: string,
  color: string,
  value: (point: AttitudeDiagnosticPoint) => number | null
): ChartDataset<'line', PlotPoint[]> {
  if (!diagnostics) throw new Error('Attitude diagnostics have not been computed.');
  const { timeOrigin } = diagnostics;
  return {
    label,
    data: diagnostics.points.map(point => ({
      x: point.sensorTime - timeOrigin, y: value(point)
    })),
    borderColor: color, backgroundColor: color, borderWidth: 1,
    pointRadius: 0, pointHitRadius: 6, spanGaps: false
  };
}

function syncRange(chart: Chart): void {
  for (const other of charts) {
    if (other.id === chart.id || !other.options.scales?.x) continue;
    other.options.scales.x.min = chart.scales.x.min;
    other.options.scales.x.max = chart.scales.x.max;
    other.update('none');
  }
}

function createChart(
  canvas: HTMLCanvasElement,
  title: string,
  unit: string,
  datasets: ChartDataset<'line', PlotPoint[]>[],
  angle = false
): void {
  charts.push(new Chart<'line', PlotPoint[]>(canvas, {
    type: 'line',
    data: { datasets },
    options: {
      responsive: true, maintainAspectRatio: false, animation: false,
      parsing: false, normalized: true,
      interaction: { intersect: false, mode: 'index' },
      plugins: {
        title: { display: true, text: title, color: '#eee' },
        legend: { labels: { color: '#ccc', boxWidth: 16 } },
        tooltip: {
          callbacks: {
            title: items => `${items[0]?.parsed.x?.toFixed(3)}s after integration start`,
            afterBody: items => {
              const status = diagnostics?.points[items[0]?.dataIndex]?.status;
              return status === 'weak-force' ? 'Weak force: direction error is not evaluated.' : '';
            }
          }
        },
        zoom: {
          pan: { enabled: true, mode: 'x', onPanComplete: ({ chart }) => syncRange(chart) },
          zoom: {
            wheel: { enabled: true }, pinch: { enabled: true }, mode: 'x',
            onZoomComplete: ({ chart }) => syncRange(chart)
          }
        }
      },
      scales: {
        x: {
          type: 'linear',
          title: { display: true, text: 'Time after integration start (s)', color: '#ccc' },
          ticks: { color: '#aaa', maxTicksLimit: 8 }, grid: { color: '#333' }
        },
        y: {
          min: angle ? 0 : undefined, max: angle ? 180 : undefined,
          title: { display: true, text: unit, color: '#ccc' },
          ticks: { color: '#aaa' }, grid: { color: '#333' }
        }
      }
    }
  }));
}

function componentDatasets(): ChartDataset<'line', PlotPoint[]>[] {
  const axis = getElements().diagnosticsComponent.value;
  const key = axis === 'north' ? 'x' : axis === 'west' ? 'y' : 'z';
  return [
    series('IMU rotated specific force', '#ff44ff', point => point.imuForce?.[key] ?? null),
    series('GPS a - g (smoothed)', '#4488ff', point => point.gpsForce?.[key] ?? null)
  ];
}

export function updateDiagnosticsComponent(): void {
  if (!charts[0] || !diagnostics) return;
  charts[0].data.datasets = componentDatasets();
  charts[0].update('none');
}

export function resetDiagnosticsZoom(): void {
  for (const chart of charts) chart.resetZoom();
}

export function destroyAttitudeDiagnostics(): void {
  for (const chart of charts) chart.destroy();
  charts.length = 0;
  diagnostics = null;
  currentInputs = null;
}

export function renderAttitudeDiagnostics(integration: IntegrationResult, gps: GPSIntegrationResult): void {
  const elements = getElements();
  const shift = elements.diagnosticsGpsShift.value.trim() ? Number(elements.diagnosticsGpsShift.value) : NaN;
  if (!Number.isFinite(shift) || Math.abs(shift) > 5) throw new Error('GPS diagnostic shift must be between -5 and +5 seconds.');
  // Compute first so a failed manual preview does not erase the existing plots.
  const computed = computeAttitudeDiagnostics(integration, gps, halfWindowSamples(), shift);
  destroyAttitudeDiagnostics();
  currentInputs = { integration, gps };
  diagnostics = computed;
  elements.diagnosticsStatus.textContent = `${
    integration.orientationSource === 'device' ? 'Device quaternion + raw accelerometer' : 'Viewer-computed calibrated acceleration'
  } | GPS shift ${shift.toFixed(2)}s | ${diagnostics.validCount.toLocaleString()} direction comparisons, ${
    diagnostics.weakCount.toLocaleString()} weak-force samples, ${
    diagnostics.excludedCount.toLocaleString()} edge/gap/invalid samples excluded.`;
  elements.diagnosticsStatus.style.color = '';
  createChart(elements.forceComponentCanvas, 'Specific force in Earth NWU', 'm/s²', componentDatasets());
  createChart(elements.forceAngleCanvas, 'Vector direction disagreement (not full attitude error)', 'degrees', [
    series('Angular disagreement', '#ffcc44', point => point.angleDegrees)
  ], true);
  createChart(elements.forceErrorCanvas, 'Magnitude and vector disagreement', 'm/s²', [
    series('|IMU| - |GPS| (signed)', '#ff44ff', point => point.magnitudeDifference),
    series('|GPS - IMU|', '#44ddaa', point => point.residualMagnitude)
  ]);
  createChart(elements.forceMagnitudeCanvas, 'Force strength / weak-direction threshold', 'm/s²', [
    series('|IMU force|', '#ff44ff', point => point.imuForce
      ? Math.hypot(point.imuForce.x, point.imuForce.y, point.imuForce.z) : null),
    series('|GPS force|', '#4488ff', point => point.gpsForce
      ? Math.hypot(point.gpsForce.x, point.gpsForce.y, point.gpsForce.z) : null),
    { ...series('0.1 g heuristic threshold', '#ffcc44', () => MIN_DIAGNOSTIC_FORCE), borderDash: [5, 5] }
  ]);
}
