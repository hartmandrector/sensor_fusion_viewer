# Single-device GPS-velocity-aided error-state filter

Status: planning draft, 2026-10-27. No estimator is implemented by this document.

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

### Stage B: pure strapdown propagation

- Implement nominal state, quaternion math, midpoint integration, and 15-state
  covariance propagation with physically dimensioned noise.
- Simulate known trajectories and bias errors; verify Jacobians numerically.

**Gate:** noiseless stationary/orientation tests below 1e-8 m/s velocity change
over 10 s at 100 Hz, quaternion norm error below 1e-12 after normalization, and
each nonzero numerical Jacobian block agreeing within 1e-5 relative or 1e-7
absolute tolerance on moderate, nonsingular test cases. Test step-size convergence.

### Stage C: GPS velocity update and error reset

- Add velocity initialization, raw GPS updates, covariance solve, gating,
  injection/reset, residual bias estimates, and health transitions.
- Handle effective update times, duplicate fixes, gaps, and dropouts.

**Gate:** same synthetic trajectory initialized at the truth with zero noise
remains at truth within numerical tolerance; analytic constant-force cases
match expected velocity/position. With modeled noise, covariance stays symmetric
and SPD, and Monte Carlo NIS/NEES coverage is assessed against declared bounds.
Unobservable cases must not produce falsely confident heading.

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
