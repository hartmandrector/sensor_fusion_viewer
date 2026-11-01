# Single-device GPS-velocity-aided error-state filter

Status: Stages B–C implemented; Stage E viewer exposure and first Stage D
replay results added 2026-10-31 (see section 11).

## 1. Objective and scope

Estimate orientation and motion together from one FlySight's IMU and GPS, so
GPS velocity innovations can constrain attitude drift and inertial biases during
dynamic flight without assuming the accelerometer measures gravity alone.

Start with the helmet unit alone. The belly unit is optional future work, not a
dependency. Preserve recorded firmware orientation and existing viewer-computed
orientation as independent comparison baselines.

First implementation: offline chronological replay in the TypeScript viewer,
using only measurements available up to each processed measurement epoch.
Keep the mathematical core independent of the DOM/Three.js for deterministic
tests and eventual firmware porting. Offline replay is not evidence that the
firmware can run it within its real-time resource budget.

### Non-goals for the first version

- Multi-device fusion, wingsuit aerodynamic assumptions, or CG trajectory recovery.
- Assuming GPS course equals helmet heading.
- A GPS-derived quaternion at each fix, or a heading-only correction.
- Replacing recorded orientation or existing filter results.
- Fusing GPS-derived acceleration as an additional independent measurement.
- Barometer aiding, online magnetometer calibration, online timing estimation,
  position measurement updates, or backward smoothing.
- Safety-critical navigation or a flight-certified solution.

## 2. Evidence and limits

Current viewer observations:

- Up specific force aligns reasonably well after timing adjustment, while
  horizontal components and vector direction can disagree substantially.
- On the examined track, several selected intervals suggest roughly a one-second
  GPS acceleration alignment offset. One 1140-1230 s diagnostic interval suggested
  -1.04 s with correlation 0.790.
- Integrated velocity still drifts, including vertically. Pure Earth-yaw error
  cannot explain vertical drift; tilt, bias, scale, timing, and measurement errors
  must remain candidate causes.
- Viewer magnetometer calibration fitted over the recording has future-data
  information unavailable to a real-time device.

These are diagnostic observations, not attitude ground truth or proof of a
hardware latency. Existing diagnostics compare differently filtered signals.
GPS used to aid the estimator cannot also be treated as independent validation
of its attitude accuracy.

## 3. Relationship to the earlier EKF design

The [existing EKF design](./EKF-SENSOR-FUSION-DESIGN.md) primarily describes an
attitude/bias filter, accelerometer gravity-direction updates, and external
orientation references from an aerodynamic "flying filter."

This plan is a separate, narrower implementation track: a strapdown inertial
navigation error-state filter with **direct GPS velocity updates**. It does not
inherit the earlier nine-state layout or continuous gravity-direction update.
During flight, specific force is a propagation input, not a gravity measurement.
The earlier design remains useful background for optional magnetic aiding and
future multi-device geometry, but its equations must not be mixed into this
model without reconciling conventions and measurement independence.

## 4. Coordinate and unit contract

Use one explicit estimator contract:

| Quantity | Convention |
|---|---|
| Body frame B | New firmware IMU axes; identity pose X=East, Y=North, Z=Up |
| Navigation frame N | Local North-West-Up (NWU), aligned to true/geodetic North |
| Quaternion | Hamilton, scalar first `[w,x,y,z]`; active body-to-navigation rotation |
| Composition | `R(q1 * q2) = R(q1) R(q2)` |
| Gyro | rad/s, converted from logged deg/s exactly once |
| Specific force | m/s², converted from logged g exactly once |
| Gravity | `g_N = [0,0,-9.80665]` m/s² for the initial local model |
| GPS velocity | Logged NED -> NWU: `[velN,-velE,-velD]` m/s |
| Time | Monotonic sensor seconds; synchronization and latency are explicit |
| Biases | Gyro in rad/s, accel in m/s², both in fixed sensor/body axes |

Firmware orientation is body-to-Earth ENU. Convert the **world frame only** for
an estimator seed: `R_NB = R_N_ENU R_ENU_B`, with
`R_N_ENU = [[0,1,0],[-1,0,0],[0,0,1]]`.
Equivalently, left-multiply the firmware quaternion by a -90-degree Z rotation.
An identity firmware quaternion is therefore not an identity NWU estimator
quaternion. Check with physical vector tests.

Do not use the fixed model-mounting transform from
[orientationSource.ts](./fusion_viewer/src/orientationSource.ts) for sensor
propagation. Add an explicit output adapter for the existing viewer model;
physical poses and estimator quaternions must not be confused.

Legacy sensor layouts and the viewer's historical body remapping require a
documented adapter before use. Do not silently assume all recordings share the
new firmware axes. Verify body-to-world direction and handedness against known
poses and measured force, not labels alone.

Magnetic yaw and true/geodetic yaw differ by declination. Any magnetic
initialization/aiding must declare its reference and conversion; never silently
compare magnetic North against GNSS true North.

## 5. State and propagation

Use a conventional 15-dimensional error state and 16-component nominal state:

```text
Nominal: x = [p_N(3), v_N(3), q_NB(4), b_g_B(3), b_a_B(3)]
Error:  dx = [dp_N(3), dv_N(3), dtheta_B(3), db_g_B(3), db_a_B(3)]
Covariance: P, 15 x 15
```

Position is initially relative and is propagated for visualization; velocity
updates do not anchor absolute position. Keep the 15-state layout conventional
rather than inventing a smaller state solely to save a little prototype code.

For measured angular velocity `w_m` and specific force `f_m`:

```text
w = w_m - b_g
f = f_m - b_a
q_next = q * Exp(w * dt)
a_N = R(q) f + g_N
v_next = v + a_N dt
p_next = p + v dt + 0.5 a_N dt^2
```

`Exp(phi)` is the unit quaternion for rotation vector phi, with half-angle
handled internally. Use midpoint orientation/force integration (and document
sample timing) rather than blindly evaluating everything at an end sample.
Test coning/rapid rotation; add higher-order integration only if evidence
requires it.

Biases follow random walks. With right-multiplicative body-frame attitude error
`q_true = q_nominal * Exp(dtheta)`, the first-order continuous dynamics include:

```text
dp_dot     = dv
dv_dot     = -R[f]x dtheta - R db_a - R n_a
dtheta_dot = -[w]x dtheta - db_g - n_g
db_g_dot   = n_bg
db_a_dot   = n_ba
```

Here `[u]x v = u cross v`. Derive `F`, `G`, discrete transition `Phi`, and `Qd`
consistently from this definition. Validate Jacobians by finite differences.
Noise densities and bias random walks need physical units and proper dt scaling;
do not tune arbitrary per-sample covariance constants.

No separate continuous accelerometer "gravity correction" during flight.
Optional stationary alignment/zero-velocity updates require conservative,
explicit stationarity detection and must not mistake free fall for rest.

## 6. GPS velocity measurement update

Baseline assumes the GNSS antenna and IMU are colocated, with the approximation
declared in the run configuration.

```text
z_v = [velN,-velE,-velD]
h(x) = v_N
r_v = z_v - h(x)
H_v = [0, I, 0, 0, 0]
S = H_v P H_v^T + R_v
K = P H_v^T S^-1
dx = K r_v
```

There is no direct attitude term in this colocated measurement Jacobian.
Attitude corrections arise through the propagated velocity/attitude and bias
cross-covariances. This coupling is essential: a stand-alone velocity blend
does not provide the requested attitude correction.

Use an SPD solve (e.g. Cholesky) instead of explicitly inverting S. Apply
corrections to p, v, biases, and `q <- q * Exp(dtheta)`. Normalize q, use the
Joseph covariance update, then apply the error-injection/reset Jacobian to P.
Derive and test that reset for the chosen right-error convention.

Use raw GNSS velocity, **not** SG-smoothed velocity or its derivative. Preserve
`sAcc` from raw [GNSSData](./fusion_viewer/src/gpsTypes.ts) into estimator input.
Check the receiver's sAcc definition before choosing R_v; do not assume a scalar
reported accuracy is a full 3-axis covariance. Start with a declared,
conservative diagonal approximation plus a configurable floor.

Reject nonfinite/invalid fixes and duplicates; compute normalized innovation
squared `NIS = r_v^T S^-1 r_v` and gate using a documented 3-DOF chi-square
threshold. Log accepted/rejected updates and reasons. Persistent rejection
requires an explicit unhealthy/reinitialization state, not silent dead reckoning
labelled as a healthy solution.

Do not also fuse derived GPS acceleration or course as independent information
from the same fixes. Do not feed an AHRS quaternion using the same IMU into each
update as though it were an independent measurement.

## 7. Timing is a prerequisite, not a tuning afterthought

The diagnostic shift must remain separate from estimator timing.

1. Audit raw timestamp meaning: GNSS measurement epoch vs sentence arrival/log
   time, sensor sampling vs publication time, GPS time vs UTC and leap seconds.
2. Audit [timestampSync.ts](./fusion_viewer/src/timestampSync.ts): current alignment
   uses the first `$TIME` entry. Examine offsets across all valid entries for
   drift, outliers, resets, week boundaries, and unit/time-scale mistakes.
3. Distinguish a fixed clock offset, receiver velocity-filter group delay, and
   logging/transport delay. Fix deterministic synchronization faults upstream
   rather than compensating them twice.
4. Validate raw GPS velocity alignment against IMU-predicted delta velocity over
   multiple dynamic windows. Record uncertainty and sensitivity to the seed
   attitude/biases. Do not optimize and evaluate on only one chosen maneuver.
5. Introduce a separate, per-run estimator offset:
   `t_effective = t_synced_GPS + estimatorGpsTimeShiftSeconds`.
   Positive moves GPS later; delayed data requires a negative shift. Default
   remains zero until validated. Never silently copy the diagnostic -1 s.

Offline replay should merge events by effective measurement epoch and propagate
exactly to each GPS update. Split IMU intervals as necessary with a documented
interpolation/hold model; no future IMU samples may be used for a claimed causal
mode. Keep original epochs and the applied offset in output provenance.

Offline reordering can use a measurement delivered later at its earlier effective
epoch, so it is causal in measurement time, not necessarily available at wall-clock
time. A future real-time implementation needs a bounded delayed-state buffer and
replay, or a delayed fusion horizon plus forward output prediction. Simply applying
an old velocity measurement to the latest state is not acceptable.

## 8. Initialization, observability, and lifecycle

### Initialization

- Prefer a verified preflight stationary interval for tilt and gyro bias.
  Stationarity is an input/check, not assumed because speed is low.
- Initialize velocity from an accepted GPS fix at the initialization epoch,
  not zero. Set relative position to zero at that same epoch.
- Yaw seed options: validated magnetometer with declination handling, a
  documented recorded/viewer quaternion seed, or an explicitly uncertain manual
  seed. A single GPS course is not helmet yaw.
- Set realistic uncertainty for seed attitude, velocity, and biases. Whole-file
  calibration or a seed using future samples must mark the run as offline-aided.
- Large yaw uncertainty can exceed the local ESKF's convergence range. Test
  multiple initial errors; report "not aligned" rather than promise recovery
  from arbitrary orientation. Multi-hypothesis/global initialization is later work.
- Reject unsupported frame/calibration/timing configurations explicitly.

### Observability

- Motion with changing specific-force directions can make attitude/bias errors
  observable through GPS velocity updates. Heading is not equally observable in
  every trajectory, and bias/attitude errors can be confounded.
- During near-ballistic free fall, specific force approaches zero, so GPS
  velocity supplies little instantaneous attitude information through this
  coupling. Gyro propagation and uncertainty growth are expected.
- GPS ground-track direction is not head orientation or necessarily body heading.
- Do not claim a heading confidence solely from a small velocity residual.

### Lifecycle

Use explicit states: not initialized, aligning, aided, coasting/GPS outage,
unhealthy, complete. Display the reason for transitions.

Use actual dt, detect nonmonotonic samples and gaps, and never replace a large
gap with a nominal dt. Bounded short intervals may be substepped; long gaps need
a declared stop/reinitialization policy. Check P symmetry, positive definiteness,
finite state, quaternion norm, and solver failures. Preserve run results and
errors; never silently return a plausible identity orientation or zero acceleration.

## 9. Calibration, magnetometer, and sensor geometry

### Calibration ownership

Choose one static sensor calibration in native sensor axes before propagation.
Estimate residual gyro/accel biases after that calibration. Record which
offset/scale terms are static and which are state estimates to prevent
double subtraction or double axis remapping.

For initial experiments support an explicit raw-calibration baseline and a
declared calibrated baseline. Do not inherit this viewer's device-specific
defaults invisibly. Calibration based on future data must be labelled.
Firmware quaternion is a baseline/optional seed, not a replacement propagation
input: use the logged gyros for this estimator.

### Magnetometer

Keep magnetic aiding off in the first velocity-aided prototype to isolate the
GPS/IMU coupling. A magnetic seed is permissible if explicitly labelled.
Later add optional magnetic updates with calibrated frame, field reference,
declination, gating, and disturbance checks. Do not introduce online magnetic
bias states in the first version.

### Antenna-to-IMU lever arm

The relevant same-device offset is **IMU to GNSS antenna**, in meters and body
axes, not sensor-to-CG. If r_B points from IMU to antenna:

```text
v_antenna_N = v_IMU_N + R_NB (w_B cross r_B)
```

The initial colocated approximation must be checked during rapid head motion.
Measure this physical offset and extend the velocity measurement model/Jacobian
if its effect is material compared with GPS noise. Acceleration-point
comparisons would additionally involve angular-acceleration/centripetal terms.

The belly device is reported within a few cm of wingsuit CG (x/c about 0.39);
the helmet sensor is near the back/top of the head (x/c about 0.05, about 4 cm
from the top of the head). These are context, not a complete 3D lever arm.
Do not infer meter offsets without chord length, reference point, signs and
all three components. The head and torso are not one rigid body.

## 10. Repository integration and outputs

Suggested implementation boundaries, subject to existing-pattern review:

1. Pure ESKF math/core: quaternion propagation, covariance, GPS updates, no UI.
2. Replay adapter: parsed IMU/GNSS, calibration, timing, initialization, provenance.
3. Independent per-epoch result arrays and diagnostics.
4. Viewer/source selection, integration/chart adapters, export.

Reuse:

- [csvParser.ts](./fusion_viewer/src/csvParser.ts) for timestamped IMU samples.
- [gpsTypes.ts](./fusion_viewer/src/gpsTypes.ts) and raw track dataset for GNSS
  velocities and accuracy, not the smoothed chart representation.
- Existing calibration definitions after auditing frame and ownership.
- Integration charts and force diagnostics for comparison, with source labels.

Do not overwrite existing `fusionFrames` or mutate recorded quaternions to insert
the new solution. Decide whether to generalize the two-value orientation-source
type or add a solution registry only when wiring the third source; avoid a broad
UI refactor before the core is validated.

Run outputs should include:

- q_NB, velocity, relative position, bias estimates, Earth linear acceleration.
- Attitude/velocity/bias uncertainty; small-angle convention and units explicit.
- GPS innovations, innovation covariance/NIS, gates and rejection reasons.
- Effective and original timestamps, aiding status and data-gap events.
- Calibration, initialization, frame adapter, noise/gate parameters, timing
  offset, algorithm version, file/session identity, and offline/causal provenance.

Invalidate dependent plots on changed inputs/configuration. Exports must
identify source and frame. Keep the current diagnostic time shift independent
and never implicitly alter a completed estimator run.

## 11. Staged implementation and completion gates

### Stage A: data/timing contract

- Confirm sensor axes, quaternion direction, true/magnetic North distinction,
  raw sample units and timestamp semantics with fixtures.
- Audit full `$TIME` sequence and raw GNSS sAcc semantics.
- Specify initialization and static-calibration inputs; document offset policy.
- Select representative stationary, flight, rotation, near-free-fall and dropout
  windows. Separate tuning windows from evaluation windows.

**Gate:** reproducible timing/frame report and known-pose tests. No GPS-derived
acceleration shift is silently promoted to velocity latency.

#### Stage A findings: static table recording (2026-10-31, session `9a53759370c4c01dad7efdfc`)

Setup reported by user: device flat on a table, front face up, LED edge toward
North (X East, Y North expected). Firmware `v2026.05.13.develop-43-g00f1590`.
About 65 s of `SENSOR.CSV`, 21.3 s of 20 Hz `TRACK.CSV` (4–11 satellites,
fix still converging), plus `EVENT.CSV` and `RAW.UBX`.

Timing (resolved):

- `$TIME,t,tow,week`: `t` is the sensor time of a 1 PPS edge. `tow` is a
  **UTC-based** time-of-week; the u-blox `TIM-TP` messages in `RAW.UBX` report
  timeBase = UTC, with tows matching `$TIME` one-for-one. The final `NAV-PVT`
  has GPS iTOW 529143.3 s at UTC 02:58:45.300, so GPS time = UTC + 18 s, and
  the `$TIME` tow (529125 at 64.78 s) is UTC. `TRACK.CSV` timestamps are UTC.
  The current sync in `timestampSync.ts`, which does **not** subtract leap
  seconds, is therefore correct. Subtracting 18 s would push GNSS samples past
  the end of the sensor log. With the current mapping, the last GNSS epoch
  (65.18 s) precedes the last IMU sample (65.22 s) and the shutdown events.
- 18 PPS entries: offset std 0.37 ms, worst deviation 1.34 ms (late edge
  timestamps), clock drift 2.7 ppm. The first entry, used by the viewer, is
  itself a 1 ms outlier. A median/linear fit is preferable but the effect is
  negligible relative to 50 ms GNSS epochs. The 1 s lag seen in flight is **not**
  a clock-sync error.
- IMU: 101.36 Hz (dt 9.82–9.91 ms), monotonic, no gaps. MAG ~103 Hz.
  GNSS epochs every 50 ms, aligned to UTC.
- `RAW.UBX` begins with data from an earlier session (2026-10-22); only its tail
  belongs to this recording. Treat it as non-authoritative without filtering.

Axes, gravity, and noise (stationary 5–60 s):

- Specific force `[0.0058, 0.0086, 0.9925]` g: body +Z is Up and the reported
  sign is specific force (not gravity). Apparent tilt 0.5°/−0.3° (table + bias).
- |f| = 0.99255 g vs. local gravity 0.99921 g (41.23°N, 1275 m): **−6.7 mg**
  (−0.065 m/s²) magnitude error. Uncorrected, this alone integrates to about
  4 m/s of vertical velocity error in 60 s. Initial accel-bias sigma must be
  at least 0.1 m/s².
- Gyro mean `[-0.528, 0.034, 0.002]` deg/s: **X bias about −0.53 deg/s**
  (9.2 mrad/s), slowly varying (−0.514 to −0.534) with temperature. Logged gyro
  is not bias-compensated.
- White-noise densities: gyro 6.2e-5–1.0e-4 rad/s/√Hz (0.004–0.006 deg/s/√Hz);
  accel 1.1e-3–1.3e-3 m/s²/√Hz. Bias random walks not identifiable from 55 s.

GNSS velocity (stationary, after the first 4 s):

- Per-axis velocity std 0.07–0.13 m/s, mean within 0.03 m/s of zero.
  Reported `sAcc` median 0.32–0.47 m/s, i.e. about 3–4× the observed
  per-axis scatter, so `R = sAcc²` per axis is conservative here.
- The first 4 s (4 satellites, sAcc ~2.4 m/s, vD error up to several m/s) must
  be gated. Gate on `numSV` and `sAcc` in addition to NIS.

Heading (**unresolved**):

- Firmware quaternion jumps from identity to about `[0.005, 0.003, 0.004, 0.9999]`
  at 0.85 s and stays there: **body +Y heading about 180° (South)**, versus the
  reported LED-edge-North placement.
- `EVENT.CSV`: `MagCal init q=GOOD hi=[223,172,21]e-3g`; later reports show
  `hi=[0,0,0]` and `in=` values equal to the raw field ×2000.
- Raw magnetometer: |B| 0.543 G, inclination 63.6°, body-Y magnetic heading −64.5°.
  After subtracting the stored hard iron: |B| 0.512 G, H only 0.068 G,
  inclination 82.4°, heading 175.6° (consistent with the firmware quaternion).
  The expected local inclination is about 65°, so the stored hard-iron
  correction appears inconsistent with this environment. However, a single
  indoor pose cannot separate hard iron from local disturbance. The magnetometer
  axis mapping to IMU axes also remains unverified.
- Needed to resolve: an independent true-North reference for the placement
  (not a phone compass indoors), and a magnetometer rotation recording
  outdoors.

Still required for the Stage A gate:

1. Resolve the heading discrepancy above (LED-edge direction vs. body +Y,
   magnetometer axes, stored hard-iron validity).
2. Six-face static poses (each axis up and down) for accel axis/sign/scale.
3. Rotations about each body axis for gyro sign, and agreement with the
   firmware quaternion.
4. A dynamic recording with good GNSS (walk/drive with turns, or a flight) to
   measure GNSS velocity latency with the IMU-predicted delta-velocity method.

User confirmation: the table placement was aligned to a north-facing street, so
LED edge toward true North is trusted. The firmware's ~180° heading is
therefore a firmware magnetometer-calibration/heading error, not a placement error.

#### Stage A findings: hand-held six-pose and flip recording (2026-10-31 03:09)

Poses (quiet but hand-held): +Z 5.5–7.8 s, −Z 11.2–15.8 s, +Y 19.3–23.8 s,
−Y 28.5–32.8 s, +X 35.5–39.5 s, −X 43.2–47.8 s, with fast flips between them.

- **Accelerometer axes/signs confirmed.** Each up-axis reads about +1 g
  (specific force). Per-axis offset/scale from ± poses: X 3.0 mg / 0.987,
  Y 10.1 mg / 0.997, Z 0.7 mg / 0.998. These are hand-held, so they are only
  indicative; the X scale deficit (−1.3%) is worth confirming on a fixture.
- **Firmware quaternion tilt is consistent** with measured gravity in every
  pose (1–4°), confirming body axes and quaternion direction for tilt.
- **Gyro axes/signs confirmed.** Across the 90° flips, integrated gyro predicts
  the next gravity direction to 8–11°; with all signs flipped the error is about 165°.
- **Gyro scale problem (blocking).** Integrated bias-corrected gyro
  under-rotates on every flip. A per-axis scale fit across five flips gives
  `[1.131, 1.132, 1.142]` and reduces the gravity-prediction residuals from
  8–23° to 0.5–3°. Per flip, the best scale is 1.10 for the slowest (160 dps peak)
  and 1.13–1.15 for the fast ones (270–355 dps). There is no saturation (max 352 dps).
  The firmware quaternion's body rate exceeds the logged gyro by 5–9% in fast
  motion. Hypotheses: a sensitivity/full-scale conversion mismatch in the logged
  values, or logged samples not representative of the rate stream
  (decimation/filtering). An uncorrected 13% scale error makes free-inertial
  attitude useless within one maneuver and would also corrupt any gyro-driven
  estimator, so it must be resolved or calibrated before Stage D. Add a gyro
  scale input to static calibration (not an online state initially).
- **Magnetometer mapping unresolved.** The sphere-fit hard iron
  `[0.146, -0.081, -0.147]` G has a 0.022 G residual (hand-held, indoors).
  The magnetometer dip should be constant (~65°) across poses but varies by
  25–56° under identity, the documented legacy remap `(-X,+Y,-Z)`, or any other
  signed permutation. Whether the new firmware logs magnetometer values in IMU
  axes is unknown. The magnetometer is off in the first ESKF, so this is not
  blocking. Use an outdoor rotation recording to resolve it.

Requested next recordings:

1. Gyro scale: on the table, rotate exactly 360° about Z (start and end
   against the same straight edge), once slowly (~30 dps) and once fast
   (~300 dps), with 5 s still before and after each turn. Repeat on edge for X and Y
   if possible. This separates constant scale from rate-dependent loss.
2. Outdoor magnetometer rotation away from metal for axis mapping and hard/soft iron.
3. Dynamic outdoor recording with good GNSS for velocity-latency estimation.

#### Stage A findings: Z spin recording and saved calibrations (2026-10-31 03:19)

- Slow 360° Z turn (peak 105 dps) integrates to 313.1° (scale 1.150); fast
  turn (peak 464 dps) integrates to 318.6° (scale 1.130). The error is a near
  constant gyro scale of ~1.14, not rate-dependent sample loss. It agrees with
  the per-axis flip fit [1.131, 1.132, 1.142]. Open question: did old-firmware
  data share this scale, or did the firmware change the gyro configuration?
- The logged magnetometer is raw, not on-device corrected: the fresh sphere
  centre is [0.146, −0.081, −0.147] G (radius 0.52, 4.2% spread).
- Saved calibrations found: `flysight_calibration_2026-01-05…`, `2026-03-29…`,
  `2026-05-22…` and `2026-05-31…`. They all share gyroBias
  [−0.134, −0.180, −0.239] dps and their hardIron is near [−0.36, −0.13, −0.41] G. On
  new-firmware data, they leave a 21% magnitude spread and per-pose dip
  angles that vary wildly. The May-31 accel bias makes |f| worse (0.955–1.043
  vs raw 0.991–1.010). The measured gyro bias is now ≈ [−0.55, 0.05, 0.1] dps.
- Conclusion: reuse the calibration **format and pipeline** (`CalibrationFile`,
  the same bias → matrix → remap order as `FusionAhrsAdapter`), but generate fresh
  calibration JSONs from new-firmware data using the viewer's existing tools
  with a selected time window. Add an optional, backward-compatible gyro scale
  field.

### Stage B: pure strapdown propagation

- Implement nominal state, quaternion math, midpoint integration, and 15-state
  covariance propagation with physically dimensioned noise.
- Simulate known trajectories and bias errors; verify Jacobians numerically.

**Gate:** noiseless stationary/orientation tests below 1e-8 m/s velocity change
over 10 s at 100 Hz, quaternion norm error below 1e-12 after normalization, and
each nonzero numerical Jacobian block agreeing within 1e-5 relative or 1e-7
absolute tolerance on moderate, nonsingular test cases. Test step-size convergence.

**Status: implemented** in [fusion_viewer/src/eskf/](./fusion_viewer/src/eskf/)
(`types`, `math3`, `quaternion`, `linalg`, `errorState`, `propagation`), with
tests in `fusion_viewer/tests/eskfQuaternion.test.mjs` and
`eskfPropagation.test.mjs`. Attitude uses the mean bias-corrected rate per
interval; navigation acceleration uses the trapezoidal rule; F is linearized at
the interval midpoint; `Φ = I + FΔt + ½(FΔt)²` and `Qd` uses a trapezoidal
Van Loan approximation. F is checked against finite differences of the exact
nonlinear error dynamics, and Φ against the numerical discrete Jacobian of the
nominal step (second-order convergence). Steps above `maxStepSeconds`
(default 0.05 s) are rejected; gap handling belongs to the replay adapter.
Not yet connected to recorded data or the viewer.

### Stage C: GPS velocity update and error reset

- Add velocity initialization, raw GPS updates, covariance solve, gating,
  injection/reset, residual bias estimates, and health transitions.
- Handle effective update times, duplicate fixes, gaps, and dropouts.

**Gate:** same synthetic trajectory initialized at the truth with zero noise
remains at truth within numerical tolerance; analytic constant-force cases
match expected velocity/position. With modeled noise, covariance stays symmetric
and SPD, and Monte Carlo NIS/NEES coverage is assessed against declared bounds.
Unobservable cases must not produce falsely confident heading.

**Status: implemented** in `fusion_viewer/src/eskf/update.ts` and `filter.ts`,
with tests in `fusion_viewer/tests/eskfUpdate.test.mjs`.

- `velocityMeasurementFromGnss`: NED → NWU. R_v is the declared diagonal
  approximation (sAccScale·sAcc)², with a 0.05 m/s floor and an optional
  vertical factor. It rejects numSV < 6, sAcc > 2 m/s, and non-finite or
  missing values.
- `updateVelocity`: Cholesky solve, NIS gate (χ²₃ 99.9% = 16.27 by default),
  Joseph form, and the exact SO(3) right Jacobian J_r(dθ̂) as the reset, so
  P ← G P Gᵀ (first order: I − ½[dθ̂]×). Rejected updates return the input
  state unchanged.
- `GpsVelocityEskf`: event-driven runner.
  - Statuses: `not-initialized`, `aided`, `coasting` (no accepted update for
    1 s), and `unhealthy` (IMU gap > 0.25 s, non-monotonic or non-finite input,
    propagation failure, loss of SPD, or 20 consecutive rejections).
  - IMU intervals are substepped to ≤ `maxStepSeconds` with linear
    interpolation. GPS epochs between IMU samples are reached with a causal
    zero-order hold of the latest IMU sample.
  - It rejects duplicate or out-of-order epochs, and fixes that are stale by
    more than 20 ms.
  - Every update and status transition is logged with its reason.
- `levelAttitude(meanForce, yaw)` gives the roll/pitch seed.
- Tests check:
  - The filter stays at truth.
  - The scalar Kalman gain.
  - Joseph form equals (I−KH)P.
  - The reset Jacobian against a numerical derivative (1e-8).
  - NIS gating and the lifecycle transitions.
  - The 8-run Monte Carlo mean NIS against 3 DOF.
  - Observability: constant specific force corrects only the attitude components
    perpendicular to f, with no false confidence about the f axis. A turning
    horizontal acceleration corrects heading. With gravity only, heading σ stays
    ≥ 90% of its prior.
- A sign mutation in the Joseph term fails 5 tests.
- NEES coverage and IMU-side calibration (gyro scale ≈1.14 on the new
  firmware) are not yet in this stage.

User confirmation (2026-10-31): old-firmware recordings integrated correctly,
so the ~1.14 gyro scale is specific to the new firmware's logged gyro and must
be handled by an explicit calibration input, not assumed globally.

### Stage D: offline replay and comparisons

- Run helmet data with magnetometer updates off.
- Compare timing variants, calibration variants, and initial yaw/bias sensitivity.
- Compare existing viewer, recorded device, and ESKF results independently.

**Gate:** report GPS velocity innovations/RMS, bias evolution, drift during
withheld-fix windows, rejection rates, uncertainty, and performance cost.
No numerical "heading accuracy" target is justified without independent truth.
Do not tune only to minimize residuals on the same GPS updates already fused.

### Stage E: viewer exposure

- Add a clearly named third solution source with proper model-frame adapter.
- Integrate output acceleration using the new state's velocity initialization
  rules; distinguish estimated velocity from the old zero-start open-loop
  acceleration integration.
- Add innovations/bias/uncertainty/status charts and source-labelled exports.
- Preserve existing Device/Viewer behavior and legacy recording support.

**Gate:** source selection, recalculation, reset, track reload, error paths,
charts and exports tested end to end; existing regression suite/build pass.

**Status: replay + viewer exposure implemented (2026-10-31); velocity-initialized
integration and the GPS-dropout test added (2026-11-01); exports not yet done.**

- Exposed as an *algorithm* ("ESKF (GPS-aided)"), not a third orientation
  source, so all existing 3D/vector/integration GUI paths are reused. With the
  ESKF selected, Fusion still runs first (it supplies the optional viewer seed
  and fallback frames), then `applyEskfToFrames` overlays quaternion, gravity,
  linear/earth acceleration and per-frame ESKF info.
- Pure layers: `eskf/calibration.ts` (IMU calibration, axis remap, ESKF↔viewer
  and device→ESKF frame adapters, verified equal to the existing device display
  conversion) and `eskf/replay.ts` (seeding, GPS time shift, pre-rejection,
  automatic re-initialization, per-sample frames and a summary).
- Viewer glue: `eskfController.ts` (settings panel, caching, overlay) and
  `eskfDiagnosticsCharts.ts` (heading vs device, σ + status, innovations,
  NIS, gyro bias, accel bias).
- Tests: `tests/eskfReplay.test.mjs` (13) and `tests/eskfDropout.test.mjs` (4);
  full suite 105/105.
- Acceleration integration has an **Initial velocity** selector: Auto (ESKF
  velocity at the start frame if aided/coasting, else GPS, else zero), ESKF,
  GPS (TRACK velocity at `start − shift`, using the ESKF panel's GPS time
  shift), or Zero. Position still starts at zero; after the start the
  integration is open loop (attitude only from the selected solution).
- GPS-dropout test (`eskf/dropout.ts`, 7th ESKF diagnostics chart + table):
  from aided frames (every 0.5 s moving, 5 s static) the ESKF nominal state is
  propagated on the IMU alone for 1/2/5/10 s and compared with the shifted GPS
  velocity it never received; a hold-last-velocity predictor is the baseline.
  Covariance/bias are not updated during the window, matching the filter mean
  during a real outage.

#### Stage D first results: Molaire head jump (2026-09-19, 1313 s, 101 Hz)

Device seed, gyro scale 1.14, saved viewer calibration, NIS gate 99.9%.
Exit ≈ 1207 s, landing ≈ 1288 s.

| GPS shift | Accepted / gated | Mean NIS | Unhealthy / re-inits | In-flight accepted, NIS median / p90 |
|---|---|---|---|---|
| 0 s | 24485 / 770 | 0.72 | 35 s / 33 (all in flight) | — |
| −1.0 s | 25948 / 20 | 0.59 | 0 s / 0 | 1644 of 1660, 1.8 / 6.2 |
| −1.2 s | 25519 / 281 | 0.70 | 8 s / 8 | 1259 of 1492, 2.7 / 36 |

- The ≈1 s GPS lag found by the lag estimator is decisive: without the shift
  the filter gates the in-flight updates and repeatedly re-initializes.
- Gyro bias settles at about (−0.45, 0.25…0.37, 0.27) °/s after the first
  motion near 195 s and stays steady through the jump.
- Ground mean NIS ≈ 0.6 means `sAcc` overstates the static velocity error
  (sAcc scale < 1 is a candidate to test).
- While static with sensor +Y about 27° from vertical, ESKF heading drifts
  about 0.1 °/s, but heading σ only grows about 0.017 °/s. The vertical-axis
  gyro bias is unobservable there and `gyroBiasRandomWalk = 2e-5` appears about
  3× too small for this gyro. In flight, heading σ drops to about 0.1°.
- Runtime: about 11–17 s for the full 22 min recording (synchronous; the UI
  blocks). Dropout test: about 1.5 s more on first chart open.

GPS-dropout results, same run (shift −1.0 s), |velocity error| in m/s:

| Dropout | Moving n | IMU RMS / median / p90 | Hold RMS / median | All aided: IMU RMS / median |
|---|---|---|---|---|
| 1 s | 158 | 0.47 / 0.34 / 0.73 | 5.1 / 3.8 | 0.33 / 0.18 |
| 2 s | 158 | 0.60 / 0.43 / 0.97 | 9.5 / 7.0 | 0.42 / 0.20 |
| 5 s | 158 | 1.15 / 0.70 / 1.52 | 18.4 / 14.4 | 0.84 / 0.26 |
| 10 s | 158 | 1.68 / 1.35 / 2.27 | 25.7 / 21.6 | 1.34 / 0.45 |

- In flight the IMU coast is 10–15× better than holding velocity and grows
  roughly linearly (≈0.15 m/s per s), i.e. a ≈1° attitude/≈15 mg level error,
  not runaway. Static windows near 15 s, 185–215 s and 1050 s are the largest
  ground errors (unconverged start, gyro-bias settling, walking).
- This is the baseline metric for noise tuning; GPS errors themselves
  (≈0.1–0.3 m/s) set its floor at short durations.

### Later

Optional gated magnetometer aiding, measured antenna lever arm, barometer,
multi-device experiments, fixed-lag/backward smoothing, firmware profiling/port.
Each must demonstrate value before expanding the state/model.

## 12. Required test cases

| Case | Expected behavior |
|---|---|
| Stationary, correct attitude | Gravity cancels; velocity remains constant |
| Constant velocity, tilted physical device | Body force rotates correctly; no fictitious navigation acceleration |
| Known ENU/NWU rotations and q/-q | Same physical trajectory; no renderer transform leaks into propagation |
| Sustained acceleration and coordinated turns | No assumption that body force is gravity |
| Known gyro/accel biases | Recovery only where motion makes them observable |
| Pure yaw error with rich horizontal excitation | Assess convergence and true-North alignment |
| Static/weakly excited motion and free fall | No artificial heading observability or overconfidence |
| Moving initialization | First accepted GPS velocity used, not zero |
| GPS spikes, invalid/duplicate fixes, outages | Explicit gates/health states; reasonable uncertainty growth |
| Known +/- clock offsets and delayed delivery | Correct sign/epoch handling; no double compensation |
| Nonuniform dt and long IMU gap | Reproducible, explicit gap policy |
| GPS antenna lever arm during head rotation | Quantify colocated approximation error |
| Bad calibration/frame or covariance failure | Explicit error, not success-shaped fallback |
| Whole-record calibration vs preflight-only | Future-data advantage labelled, not called real-time performance |

## 13. Open decisions before implementation

- Verified raw GPS velocity epoch/latency and sAcc interpretation.
- Which recordings/configurations share the new firmware body convention.
- Initial calibration/seed choices, stationary interval selection, and whether a
  no-magnetometer initialization can achieve adequate yaw convergence.
- Measured sensor noise/bias random-walk settings and conservative initial P.
- Initial GPS gating probability, gap thresholds, and reinitialization policy.
- Antenna-to-IMU 3D displacement on each physical unit.
- Independent attitude reference or a controlled test for heading validation.

These are explicit unknowns. Do not substitute CG offsets, GPS course, diagnostic
lag, or the existing quaternion for missing measurement information.

## References

- [Solà: Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/abs/1711.02508).
- [PX4 EKF2 navigation-filter documentation](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html):
  practical GPS/IMU aiding, heading observability and delayed fusion.
- [Existing attitude-EKF design](./EKF-SENSOR-FUSION-DESIGN.md).
- [Viewer coordinate-system notes](./COORDINATE_SYSTEMS.md), with historical
  sensor/body assumptions to audit rather than copy unchanged.
- Firmware BLE/IMU documentation and MotionFX configuration in the companion
  firmware repository; verify against the revision that produced each recording.
