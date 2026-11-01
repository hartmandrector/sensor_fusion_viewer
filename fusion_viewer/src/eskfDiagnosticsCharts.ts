/**
 * ESKF diagnostics chart page: heading, attitude uncertainty and status,
 * GPS velocity innovations, NIS, and bias estimates.
 */

import { Chart, type ChartDataset } from 'chart.js';
import { state } from './appState';
import { getElements } from './uiElements';
import { getEskfDropoutReport } from './eskfController';
import { deviceToEskfQuaternion } from './eskf/calibration';
import { quatToMatrix } from './eskf/quaternion';
import { STANDARD_GRAVITY } from './eskf/propagation';
import type { ReplayFrame, ReplayResult } from './eskf/replay';
import type { Quat } from './eskf/types';

type PlotPoint = { x: number; y: number | null };
type EskfChart = Chart<'line', PlotPoint[]>;

const charts: EskfChart[] = [];
const MAX_POINTS = 5000;
const DEG = Math.PI / 180;
const STATUS_LEVEL = { 'not-initialized': 0, aided: 1, coasting: 2, unhealthy: 3 } as const;
const STATUS_NAMES = ['not initialized', 'aided', 'coasting', 'unhealthy'];

/** Heading of the body +Y axis projected on the horizontal, degrees clockwise from North. */
function bodyYHeadingDeg(q: Quat): number | null {
  const R = quatToMatrix(q);
  const n = R[1];
  const w = R[4];
  if (Math.hypot(n, w) < 0.2) return null;
  return ((Math.atan2(-w, n) / DEG) + 360) % 360;
}

/** Breaks the line where a wrapped angle jumps across 0/360. */
function unwrapGaps(points: PlotPoint[]): PlotPoint[] {
  const out: PlotPoint[] = [];
  for (let i = 0; i < points.length; i++) {
    const prev = out.length ? out[out.length - 1].y : null;
    const y = points[i].y;
    if (prev !== null && y !== null && Math.abs(y - prev) > 180) out.push({ x: points[i].x, y: null });
    out.push(points[i]);
  }
  return out;
}

function syncRange(chart: Chart): void {
  for (const other of charts) {
    if (other.id === chart.id || !other.options.scales?.x) continue;
    other.options.scales.x.min = chart.scales.x.min;
    other.options.scales.x.max = chart.scales.x.max;
    other.update('none');
  }
}

function line(label: string, color: string, data: PlotPoint[], extra: Partial<ChartDataset<'line', PlotPoint[]>> = {}): ChartDataset<'line', PlotPoint[]> {
  return {
    label, data, borderColor: color, backgroundColor: color, borderWidth: 1,
    pointRadius: 0, pointHitRadius: 6, spanGaps: false, ...extra,
  };
}

function dots(label: string, color: string, data: PlotPoint[], radius = 1.5): ChartDataset<'line', PlotPoint[]> {
  return line(label, color, data, { showLine: false, pointRadius: radius });
}

function createChart(
  canvas: HTMLCanvasElement,
  title: string,
  unit: string,
  datasets: ChartDataset<'line', PlotPoint[]>[],
  options: { logY?: boolean; statusAxis?: boolean } = {},
): void {
  const scales: Record<string, object> = {
    x: {
      type: 'linear',
      title: { display: true, text: 'Time from start of recording (s)', color: '#ccc' },
      ticks: { color: '#aaa', maxTicksLimit: 8 }, grid: { color: '#333' },
    },
    y: {
      type: options.logY ? 'logarithmic' : 'linear',
      title: { display: true, text: unit, color: '#ccc' },
      ticks: { color: '#aaa' }, grid: { color: '#333' },
    },
  };
  if (options.statusAxis) {
    scales.status = {
      position: 'right', min: -0.5, max: 3.5,
      ticks: { color: '#aaa', stepSize: 1, callback: (v: number | string) => STATUS_NAMES[Number(v)] ?? '' },
      grid: { drawOnChartArea: false },
    };
  }
  charts.push(new Chart<'line', PlotPoint[]>(canvas, {
    type: 'line',
    data: { datasets },
    options: {
      responsive: true, maintainAspectRatio: false, animation: false,
      parsing: false, normalized: true,
      interaction: { intersect: false, mode: 'nearest', axis: 'x' },
      plugins: {
        title: { display: true, text: title, color: '#eee' },
        legend: { labels: { color: '#ccc', boxWidth: 16 } },
        tooltip: { callbacks: { title: items => `${items[0]?.parsed.x?.toFixed(3)} s` } },
        zoom: {
          pan: { enabled: true, mode: 'x', onPanComplete: ({ chart }) => syncRange(chart) },
          zoom: {
            wheel: { enabled: true }, pinch: { enabled: true }, mode: 'x',
            onZoomComplete: ({ chart }) => syncRange(chart),
          },
        },
      },
      scales,
    },
  }));
}

function decimate(frames: ReplayFrame[]): number[] {
  const stride = Math.max(1, Math.ceil(frames.length / MAX_POINTS));
  const idx: number[] = [];
  for (let i = 0; i < frames.length; i += stride) {
    idx.push(i);
    // Keep status transitions exact.
    const next = Math.min(i + stride, frames.length) - 1;
    if (frames[next].status !== frames[i].status) {
      for (let k = i + 1; k <= next; k++) if (frames[k].status !== frames[k - 1].status) idx.push(k - 1, k);
    }
  }
  return [...new Set(idx)].sort((a, b) => a - b);
}

function render(result: ReplayResult): void {
  const e = getElements();
  const origin = state.dataset?.startTime ?? 0;
  const frames = result.frames;
  const idx = decimate(frames);
  const initialized = (i: number) => frames[i].status !== 'not-initialized';
  const t = (i: number) => frames[i].time - origin;
  const pts = (fn: (f: ReplayFrame) => number | null): PlotPoint[] =>
    idx.map(i => ({ x: t(i), y: initialized(i) ? fn(frames[i]) : null }));

  const remap = state.fusionAhrs?.getIMUAxisRemap();
  const deviceHeading: PlotPoint[] = remap
    ? idx.map(i => {
      const q = state.fusionFrames[i]?.imu?.deviceQuaternion;
      return { x: t(i), y: q ? bodyYHeadingDeg(deviceToEskfQuaternion([q.w, q.x, q.y, q.z], remap)) : null };
    })
    : [];
  const headingSets = [line('ESKF', '#44ddff', unwrapGaps(pts(f => bodyYHeadingDeg(f.q))))];
  if (deviceHeading.some(p => p.y !== null)) headingSets.push(line('Device quaternion', '#ffaa44', unwrapGaps(deviceHeading)));
  createChart(e.eskfHeadingCanvas, 'Heading of sensor +Y (clockwise from North)', 'deg', headingSets);

  createChart(e.eskfSigmaCanvas, 'Attitude 1σ and filter status', 'deg', [
    line('Heading σ', '#44ddff', pts(f => f.headingSigma / DEG)),
    line('Tilt σ', '#88ff88', pts(f => f.tiltSigma / DEG)),
    line('Status', '#ff6666', idx.map(i => ({ x: t(i), y: STATUS_LEVEL[frames[i].status] })), { yAxisID: 'status', stepped: true }),
  ], { statusAxis: true });

  const updates = result.events.filter((ev): ev is Extract<typeof ev, { kind: 'update' }> => ev.kind === 'update');
  const accepted = updates.filter(u => u.accepted);
  const gated = updates.filter(u => !u.accepted && u.innovation.every(Number.isFinite));
  const innov = (list: typeof updates, k: number) => list.map(u => ({ x: u.time - origin, y: u.innovation[k] }));
  const rejectedNorm = gated.map(u => ({ x: u.time - origin, y: Math.hypot(...u.innovation) }));
  createChart(e.eskfInnovationCanvas, 'GPS velocity innovation (GPS − ESKF prediction)', 'm/s', [
    dots('North', '#ff6666', innov(accepted, 0)),
    dots('West', '#66ff66', innov(accepted, 1)),
    dots('Up', '#6688ff', innov(accepted, 2)),
    dots('Gated (|innovation|)', '#ffffff', rejectedNorm, 3),
  ]);

  const gate = state.eskfRun?.result ? Number(e.eskfNisGate.value) : NaN;
  const nisSets = [
    dots('NIS accepted', '#44ddff', accepted.map(u => ({ x: u.time - origin, y: Math.max(u.nis, 1e-3) }))),
    dots('NIS gated', '#ff6666', gated.filter(u => Number.isFinite(u.nis)).map(u => ({ x: u.time - origin, y: u.nis })), 3),
  ];
  if (Number.isFinite(gate) && gate < 1e6 && frames.length) {
    nisSets.push(line('Gate', '#ffff66', [{ x: t(0), y: gate }, { x: t(frames.length - 1), y: gate }], { borderDash: [6, 4] }));
  }
  nisSets.push(line('Expected mean (3)', '#888888', frames.length ? [{ x: t(0), y: 3 }, { x: t(frames.length - 1), y: 3 }] : [], { borderDash: [2, 4] }));
  createChart(e.eskfNisCanvas, 'Normalized innovation squared (3 DOF)', 'NIS', nisSets, { logY: true });

  createChart(e.eskfGyroBiasCanvas, 'Estimated gyro bias (body axes, after calibration)', 'deg/s', [
    line('X', '#ff6666', pts(f => f.gyroBias[0] / DEG)),
    line('Y', '#66ff66', pts(f => f.gyroBias[1] / DEG)),
    line('Z', '#6688ff', pts(f => f.gyroBias[2] / DEG)),
  ]);
  createChart(e.eskfAccelBiasCanvas, 'Estimated accel bias (body axes, after calibration)', 'mg', [
    line('X', '#ff6666', pts(f => (f.accelBias[0] / STANDARD_GRAVITY) * 1e3)),
    line('Y', '#66ff66', pts(f => (f.accelBias[1] / STANDARD_GRAVITY) * 1e3)),
    line('Z', '#6688ff', pts(f => (f.accelBias[2] / STANDARD_GRAVITY) * 1e3)),
  ]);

  renderDropout(origin);
}

const DROPOUT_COLORS = ['#88ff88', '#44ddff', '#ffaa44', '#ff6666', '#cc88ff'];

function renderDropout(origin: number): void {
  const e = getElements();
  const report = getEskfDropoutReport();
  if (!report) {
    e.eskfDropoutTable.textContent = '';
    return;
  }
  const norm = (v: readonly number[] | null) => (v ? Math.hypot(...v) : null);
  const sets = report.durations.map((d, k) => dots(`IMU ${d} s`, DROPOUT_COLORS[k % DROPOUT_COLORS.length],
    report.windows.map(w => ({ x: w.startTime - origin, y: norm(w.imuError[k]) })).filter(p => p.y !== null)));
  const holdK = report.durations.indexOf(5) >= 0 ? report.durations.indexOf(5) : report.durations.length - 1;
  sets.push(dots(`Hold ${report.durations[holdK]} s`, '#777777',
    report.windows.map(w => ({ x: w.startTime - origin, y: norm(w.holdError[holdK]) })).filter(p => p.y !== null), 1));
  createChart(e.eskfDropoutCanvas, 'Simulated GPS dropout: |velocity error| at end of dropout (by start time)', 'm/s', sets);

  const f = (x: number) => (Number.isFinite(x) ? x.toFixed(2).padStart(7) : '      -');
  const rows = (label: string, list: typeof report.all) => [
    `${label}`,
    '  dropout   n    IMU rms  median    p90 | hold rms  median',
    ...list.map(s => `  ${String(s.duration).padStart(5)} s ${String(s.count).padStart(5)} ${f(s.imuRms)} ${f(s.imuMedian)} ${f(s.imuP90)} | ${f(s.holdRms)} ${f(s.holdMedian)}`),
  ];
  e.eskfDropoutTable.textContent = [
    ...rows('All aided windows (m/s)', report.all),
    ...rows('Moving windows only (GPS speed ≥ 5 m/s)', report.moving),
  ].join('\n');
}

export function destroyEskfDiagnostics(): void {
  for (const chart of charts) chart.destroy();
  charts.length = 0;
}

export function resetEskfDiagnosticsZoom(): void {
  for (const chart of charts) chart.resetZoom();
}

/** Shows the ESKF page in the charts panel (independent of acceleration integration). */
export function showEskfDiagnostics(): void {
  const e = getElements();
  destroyEskfDiagnostics();
  e.viewerPanel.style.display = 'none';
  e.chartsPanel.style.display = 'flex';
  e.integrationChartsView.style.display = 'none';
  e.attitudeDiagnosticsView.hidden = true;
  e.eskfDiagnosticsView.hidden = false;
  e.showDiagnosticsBtn.hidden = true;
  e.backToIntegrationBtn.hidden = true;
  e.integrationChartTitle.textContent = 'ESKF diagnostics - GPS-velocity-aided';
  e.chartsPanel.scrollTop = 0;
  const result = state.eskfRun?.result;
  if (!result) {
    e.eskfDiagnosticsStatus.textContent = 'No ESKF result. Select the ESKF algorithm, load TRACK.CSV and press Run ESKF.';
    return;
  }
  e.eskfDiagnosticsStatus.textContent = e.eskfRunStatus.textContent ?? '';
  render(result);
}
