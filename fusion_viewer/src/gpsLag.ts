import type { AttitudeDiagnosticPoint, AttitudeDiagnostics } from './attitudeDiagnostics';

export type LagSignal = 'up' | 'magnitude';
export const LAG_SEARCH_LIMIT = 2;
export const LAG_SEARCH_STEP = 0.02;

function value(point: AttitudeDiagnosticPoint, stream: 'imuForce' | 'gpsForce', signal: LagSignal) {
  const force = point[stream];
  if (!force) return null;
  return signal === 'up' ? force.z : Math.hypot(force.x, force.y, force.z);
}

/** Positive shift moves GPS later; delayed GPS therefore needs a negative shift. */
export function estimateGPSLag(
  diagnostics: AttitudeDiagnostics,
  startSeconds: number,
  endSeconds: number,
  signal: LagSignal
) {
  if (![startSeconds, endSeconds].every(Number.isFinite) || startSeconds >= endSeconds) {
    throw new Error('Lag interval must have finite start < end times.');
  }
  const points = diagnostics.points;
  if (points.length < 3) throw new Error('Not enough diagnostic samples for lag estimation.');
  const intervals = points.slice(1).map((p, i) => p.sensorTime - points[i].sensorTime).sort((a, b) => a - b);
  const maxGap = 3 * intervals[Math.floor(intervals.length / 2)];
  const gpsAt = (time: number): number | null => {
    let lo = 0, hi = points.length - 1;
    if (time < points[lo].sensorTime || time > points[hi].sensorTime) return null;
    while (hi - lo > 1) {
      const mid = (lo + hi) >>> 1;
      if (points[mid].sensorTime <= time) lo = mid;
      else hi = mid;
    }
    const a = value(points[lo], 'gpsForce', signal), b = value(points[hi], 'gpsForce', signal);
    const dt = points[hi].sensorTime - points[lo].sensorTime;
    if (a === null || b === null || dt > maxGap || dt <= 0) return null;
    return a + (b - a) * (time - points[lo].sensorTime) / dt;
  };
  const steps = Math.round(LAG_SEARCH_LIMIT / LAG_SEARCH_STEP);
  const shifts = Array.from({ length: 2 * steps + 1 }, (_, i) => (i - steps) * LAG_SEARCH_STEP);
  const eligible = points.filter(p => {
    const relative = p.sensorTime - diagnostics.timeOrigin;
    return relative >= startSeconds && relative <= endSeconds && value(p, 'imuForce', signal) !== null;
  });
  const stride = Math.max(1, Math.ceil(eligible.length / 3000));
  // Use exactly the same IMU epochs for every candidate, never fill gaps or extrapolate.
  const anchors = eligible.filter((p, i) => i % stride === 0
    && shifts.every(shift => gpsAt(p.sensorTime - shift) !== null));
  if (anchors.length < 30) {
    throw new Error('Need at least 30 common valid samples across the +/-2s search. Select a longer interval away from gaps/edges.');
  }
  const xs = anchors.map(p => value(p, 'imuForce', signal) ?? NaN);
  const meanX = xs.reduce((sum, x) => sum + x, 0) / xs.length;
  const varianceX = xs.reduce((sum, x) => sum + (x - meanX) ** 2, 0);
  if (Math.sqrt(varianceX / xs.length) < 0.1) {
    throw new Error('IMU signal varies by less than 0.1 m/s² RMS. Select an interval with distinct acceleration changes.');
  }
  const scores = shifts.map(shift => {
    const ys = anchors.map(p => gpsAt(p.sensorTime - shift) ?? NaN);
    const meanY = ys.reduce((sum, y) => sum + y, 0) / ys.length;
    let covariance = 0, varianceY = 0;
    ys.forEach((y, i) => {
      covariance += (xs[i] - meanX) * (y - meanY);
      varianceY += (y - meanY) ** 2;
    });
    return Math.sqrt(varianceY / ys.length) < 0.1 ? null : covariance / Math.sqrt(varianceX * varianceY);
  });
  let bestIndex = -1;
  scores.forEach((score, i) => {
    if (score !== null && (bestIndex < 0 || score > (scores[bestIndex] ?? -Infinity))) bestIndex = i;
  });
  if (bestIndex < 0) throw new Error('GPS signal has insufficient variation for lag estimation.');
  const shift = shifts[bestIndex], correlation = scores[bestIndex] ?? 0;
  const competing = scores.some((score, i) => score !== null
    && Math.abs(shifts[i] - shift) >= 0.3 && score >= correlation - 0.02);
  const warnings: string[] = [];
  if (correlation < 0.6) warnings.push('Weak correlation');
  if (competing) warnings.push('Broad or competing correlation peak');
  if (bestIndex === 0 || bestIndex === shifts.length - 1) warnings.push('Peak at search boundary');
  return { shiftSeconds: shift, correlation, sampleCount: anchors.length, warnings };
}
