# Error-State EKF Sensor Fusion — Design Document

> Replacing/augmenting Madgwick AHRS with a proper EKF for FlySight 2 head orientation.
> Living document — updated with code. Last revised: 2026-03-31.

---

## Table of Contents

1. [Motivation — Why Replace Madgwick](#1-motivation--why-replace-madgwick)
2. [Algorithm Selection](#2-algorithm-selection)
3. [Error-State EKF Architecture](#3-error-state-ekf-architecture)
4. [State Vector & Dynamics](#4-state-vector--dynamics)
5. [Measurement Models](#5-measurement-models)
6. [External Reference Integration](#6-external-reference-integration--the-flying-filter)
7. [Adaptive Noise — Continuous Rejection](#7-adaptive-noise--continuous-rejection)
8. [Filter Lifecycle & Timing](#8-filter-lifecycle--timing)
9. [Implementation Plan](#9-implementation-plan)
10. [Calibration Interface](#10-calibration-interface)
11. [Reference Material](#11-reference-material)
12. [Open Questions](#12-open-questions)

---

## 1. Motivation — Why Replace Madgwick

The Madgwick algorithm (revised version, Ch.7 of his PhD thesis) is a complementary filter that integrates gyro and applies a gradient descent correction from accelerometer and magnetometer. It works well for simple orientation tracking in benign conditions. In our application — wingsuit BASE / skydiving with dynamic acceleration, magnetic distortion near aircraft, and the need for GPS-fused heading — it falls short in specific, important ways:

### Problems Observed (2026-03-31 testing session)

1. **Binary rejection is too coarse.** Acceleration and magnetic rejection are threshold-based: trust the sensor fully or ignore it entirely. There's no middle ground. During sustained dynamic flight (2-3g turns, body accelerations), the filter oscillates between trusting and ignoring the accelerometer, causing visible wobble.

2. **Heading is the weakest axis.** The magnetometer provides the only heading reference. When it's rejected (magnetic distortion near aircraft, soft iron from helmet hardware), heading drifts at the gyro bias rate with no recovery mechanism except the magnetometer itself coming back. The recovery dynamics are unpredictable.

3. **No uncertainty awareness.** The algorithm outputs a quaternion with no confidence information. There's no way to know "pitch is good but heading is uncertain." Every axis has the same single gain β.

4. **No external reference injection.** We have high-quality heading/pitch/roll estimates from our GPS-based flying filter (SG pipeline + orientation EKF with full aerodynamic model). There's no principled way to feed that information into the Madgwick algorithm. The only workaround discovered was dynamically adjusting calibration offsets — a hack that works but has no theoretical basis.

5. **No gyro bias estimation.** Madgwick integrates gyro measurements at face value (optionally with a separate bias estimator). Temperature-dependent bias drift accumulates during dissociation periods when the correction term is suppressed.

### What We Need

- **Continuous sensor weighting** instead of binary accept/reject
- **External reference vectors** from the flying filter (heading, pitch, roll) as first-class measurements with tunable confidence
- **Per-axis uncertainty** — know when heading is good vs drifting
- **Gyro bias tracking** — prevent drift during dynamic periods
- **Same output format** — quaternion at sensor rate (400 Hz), exportable as fused CSV

---

## 2. Algorithm Selection

### Candidates Evaluated

| Algorithm | Type | Strengths | Weaknesses | Verdict |
|-----------|------|-----------|------------|---------|
| **Madgwick** (current) | Complementary | Fast, simple, well-tested | Binary rejection, no uncertainty, no bias, no external refs | Replace |
| **Mahony** | Complementary (PI controller) | Faster convergence than Madgwick, integral term tracks bias | Still complementary — same fundamental limitations | Not enough improvement |
| **Direct Quaternion EKF** (4-7 state) | EKF on quaternion directly | Proper covariance, external refs possible | Quaternion normalization issues, linearization around large errors less accurate | Good but not ideal |
| **Error-State Quaternion EKF** | EKF on error perturbation | Best linearization (error always small), clean injection, no normalization issues, standard in industry | More conceptually complex, two-layer architecture | **Selected** |
| **UKF** (Unscented) | Sigma-point filter | No Jacobians needed, better for high nonlinearity | More expensive (~2N+1 sigma points per step), overkill for quaternion kinematics which are mildly nonlinear | Overkill |
| **Madgwick → EKF chain** | Two-stage | Reuses existing Madgwick, EKF refines | Madgwick output IS the nominal state in error-state formulation — just skip the middleman | Absorbed into error-state approach |

### Selection: Error-State Quaternion EKF (Solà formulation)

The error-state approach is the industry standard for INS/AHRS (used in VectorNav, Xsens, PX4 autopilot). It separates the problem into:

1. **Nominal state** — integrated forward using gyro (fast, every sample, no filter overhead)
2. **Error state** — small perturbation tracked by EKF (only updated when measurements arrive)

The key insight: the error is always small (because it's reset after each correction), so linearization is always accurate. This avoids the main failure mode of direct quaternion EKFs where large errors make the Jacobian approximation poor.

---

## 3. Error-State EKF Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    NOMINAL STATE                         │
│   (runs at IMU rate: 400 Hz, no filter overhead)        │
│                                                          │
│   q̂ ← q̂ ⊗ q((ω_gyro - b̂_gyro) · dt)                   │
│                                                          │
│   This is just gyro integration. Fast. Always runs.     │
└────────────────────────┬────────────────────────────────┘
                         │ q̂ (nominal quaternion)
                         │
┌────────────────────────▼────────────────────────────────┐
│                    ERROR-STATE EKF                        │
│   (runs at measurement rate: variable)                   │
│                                                          │
│   State: δx = [δθ(3), δb_gyro(3), δb_mag(3)]           │
│   Covariance: P (9×9)                                    │
│                                                          │
│   PREDICT: propagate error covariance                    │
│     δx̂ = F · δx  (F from gyro dynamics)                 │
│     P̂ = F·P·Fᵀ + Q                                     │
│                                                          │
│   UPDATE (when measurements arrive):                     │
│     Innovation: y = z_meas - h(q̂)                       │
│     Kalman gain: K = P̂·Hᵀ·(H·P̂·Hᵀ + R)⁻¹              │
│     Error correction: δx = K · y                        │
│     Covariance: P = (I - K·H) · P̂                      │
│                                                          │
│   INJECT: apply correction to nominal state              │
│     q ← q̂ ⊗ q(δθ)   (small rotation from error)       │
│     b_gyro ← b̂_gyro + δb_gyro                           │
│     b_mag ← b̂_mag + δb_mag                              │
│     δx ← 0   (reset error state)                        │
│     P ← G·P·Gᵀ  (reset covariance rotation)            │
└─────────────────────────────────────────────────────────┘
```

### Why Two Layers?

The nominal state handles the high-rate gyro integration (400 Hz) with zero overhead — just quaternion multiplication. The EKF only runs when measurements arrive (could be every sample for accel/mag, or every 50-200ms for flying filter references). This separation means:

- IMU processing is always fast regardless of how many measurement sources exist
- Adding a new measurement source (GPS heading, flying filter, barometer) doesn't slow down the core integration
- The error is always near zero → Jacobians are always accurate

---

## 4. State Vector & Dynamics

### Error State (9 elements)

| Index | Symbol | Units | Description |
|-------|--------|-------|-------------|
| 0-2 | δθ | rad | Rotation error (axis-angle, small) |
| 3-5 | δb_gyro | rad/s | Gyroscope bias error |
| 6-8 | δb_mag | µT | Magnetometer bias/distortion error |

### Nominal State (maintained separately, not in EKF)

| Symbol | Type | Description |
|--------|------|-------------|
| q̂ | quaternion | Orientation (integrated from gyro) |
| b̂_gyro | 3-vector | Current gyro bias estimate (rad/s) |
| b̂_mag | 3-vector | Current mag bias estimate (µT) |

### State Transition (Continuous)

The error-state dynamics (Solà §5.3.3):

```
δθ̇ = -[ω×] δθ - δb_gyro + n_θ
δḃ_gyro = n_b_gyro      (random walk)
δḃ_mag = n_b_mag         (random walk)
```

Where `[ω×]` is the skew-symmetric matrix of the gyro measurement (minus bias), and `n` terms are process noise.

### Discrete State Transition Matrix F (9×9)

```
F = | R(ω·dt)ᵀ   -I·dt    0    |
    |    0          I      0    |
    |    0          0      I    |
```

Where `R(ω·dt)` is the rotation matrix corresponding to the gyro-integrated rotation over dt. For small dt (2.5ms at 400 Hz), the first-order approximation `I - [ω×]·dt` is sufficient.

### Process Noise Q (9×9, diagonal)

| States | Symbol | Typical Value | Physical Meaning |
|--------|--------|---------------|------------------|
| δθ | σ²_θ | (0.01 rad/s · √dt)² | Gyro white noise |
| δb_gyro | σ²_bg | (0.001 rad/s² · dt)² | Gyro bias instability (random walk) |
| δb_mag | σ²_bm | (0.1 µT/s · dt)² | Mag environment change rate |

These need tuning from FlySight sensor datasheets:
- **LSM6DSO gyro**: noise density 3.8 mdps/√Hz → σ_θ ≈ 6.6e-5 rad/√s
- **LSM6DSO gyro bias**: in-run stability ~5°/hr → σ_bg ≈ 2.4e-5 rad/s/√s
- **LIS2MDL mag**: noise density 3 mGauss/√Hz → σ for mag measurements

---

## 5. Measurement Models

Each measurement compares a predicted value h(q̂) against the actual sensor reading. The innovation (residual) drives the correction.

### 5.1 Accelerometer (Gravity Direction)

**When to use:** Every IMU sample (400 Hz), with adaptive R based on acceleration magnitude.

**Model:** The accelerometer measures gravity in body frame (plus linear acceleration noise):

```
z_accel = R(q̂)ᵀ · [0, 0, g]ᵀ + noise

h(q̂) = R(q̂)ᵀ · g_ref      (predicted gravity direction in body frame)
```

**Jacobian H_accel** (3×9): derivative of h w.r.t. error state δθ:

```
H_accel = [ [R(q̂)ᵀ · g_ref ×]   0₃ₓ₃   0₃ₓ₃ ]
```

Where `[v×]` is the skew-symmetric matrix of vector v.

**Measurement noise R_accel:**
- Base value from sensor datasheet
- **Scaled up** when `||a_measured|| - g` is large (dynamic acceleration detected)
- This replaces Madgwick's binary acceleration rejection with continuous trust scaling

```
R_accel = R_accel_base · (1 + k_accel · ||a_meas - g_expected||²)
```

### 5.2 Magnetometer (Heading Direction)

**When to use:** Every mag sample (up to 400 Hz on LIS2MDL), with adaptive R based on field magnitude deviation.

**Model:** The magnetometer measures Earth's magnetic field in body frame:

```
z_mag = R(q̂)ᵀ · m_ref + b_mag + noise

h(q̂) = R(q̂)ᵀ · m_ref + b̂_mag
```

Where `m_ref` is the expected magnetic field in NED frame (from WMM/IGRF model or measured during calibration).

**Jacobian H_mag** (3×9):

```
H_mag = [ [R(q̂)ᵀ · m_ref ×]   0₃ₓ₃   I₃ₓ₃ ]
```

The `I₃ₓ₃` in the last block means the EKF can estimate magnetometer bias in real time — this is the continuous version of the hard/soft iron calibration we do offline.

**Measurement noise R_mag:**
- Base value from sensor datasheet  
- **Scaled up** when `||m_measured|| - ||m_expected||` is large (magnetic distortion detected)
- This replaces Madgwick's binary magnetic rejection

```
R_mag = R_mag_base · (1 + k_mag · (||m_meas|| - ||m_ref||)² / ||m_ref||²)
```

### 5.3 Heading-Only Magnetometer Update (Optional)

Sometimes we only want the magnetometer to correct heading, not pitch/roll (since gravity already handles those better). This uses a scalar measurement:

```
z_heading = atan2(m_N_horizontal, m_E_horizontal)
h_heading = predicted heading from q̂ + m_ref
```

This 1×9 measurement model is simpler and avoids the mag corrupting pitch/roll during distortions.

---

## 6. External Reference Integration — The Flying Filter

**This is the key architectural advantage over Madgwick.**

Our polar-visualizer orientation EKF produces heading/pitch/roll from GPS + aerodynamic model at 5-20 Hz. These are high-quality, independent estimates based on completely different sensors (GPS vs IMU). They can be injected as measurements into the sensor fusion EKF.

### 6.1 Euler Angle Reference Measurements

When the flying filter produces an orientation estimate:

```
z_ref = [φ_flying, θ_flying, ψ_flying]     (roll, pitch, heading from GPS/aero model)
h_ref = euler_angles(q̂)                     (predicted Euler angles from nominal quaternion)
```

**Jacobian H_ref** (3×9): derivative of Euler extraction w.r.t. δθ:

```
H_ref = [ J_euler(q̂)   0₃ₓ₃   0₃ₓ₃ ]
```

Where `J_euler` is the 3×3 Jacobian of the Euler angle extraction from the quaternion, evaluated at q̂.

**Measurement noise R_ref** (3×3, diagonal — per-axis confidence):

| Axis | R value | Depends on |
|------|---------|------------|
| Roll (φ) | R_roll | Airspeed — poor at low V, good at high V |
| Pitch (θ) | R_pitch | α confidence — good in steady flight, poor during transition |
| Heading (ψ) | R_heading | GPS quality (HDOP), airspeed, track stability |

```
R_ref = diag(σ²_roll(V), σ²_pitch(V, α_conf), σ²_heading(HDOP, V))
```

### 6.2 Reference Vector Measurement (Alternative)

Instead of Euler angles, inject the flying filter's orientation as a reference direction vector (e.g., body-X should point in the direction of flight):

```
z_flight_dir = R(q̂_flying) · [1, 0, 0]ᵀ      (forward direction from flying filter)
h_flight_dir = R(q̂_sensor) · [1, 0, 0]ᵀ       (forward direction from sensor quaternion)
```

This avoids Euler angle singularities and wrapping issues. The measurement is a 3-vector comparison in the inertial frame.

### 6.3 When to Trust the Flying Filter

The flying filter is only useful when the vehicle is actually flying:

| Flight Mode | Flying Filter Trust | Reason |
|-------------|-------------------|--------|
| GROUND | **None** — don't inject | No airspeed → no aero model → garbage orientation |
| FREEFALL | **Low** | Drag-only, poor heading/roll observability |
| WINGSUIT | **High** | Full aero model, good α/β estimation |
| CANOPY | **High** | Good aero model, different vehicle polar |

R_ref should be set to infinity (= ignore measurement) on the ground and during freefall, and tightened during stable wingsuit/canopy flight.

---

## 7. Adaptive Noise — Continuous Rejection

The biggest improvement over Madgwick: instead of binary accept/reject, the EKF uses **continuous R scaling** to modulate trust in each sensor.

### Acceleration Trust

```
accel_deviation = ||a_measured|| - g       (how far from 1g)
R_accel_adaptive = R_accel_base · (1 + k₁ · accel_deviation²)
```

| Condition | accel_deviation | R_accel effect | Behavior |
|-----------|-----------------|----------------|----------|
| Static / 1g flight | ~0 | R_base (low) | Accel fully trusted for pitch/roll |
| Mild dynamics (1.5g) | ~0.5g | ~2× R_base | Accel somewhat trusted |
| Hard dynamics (3g) | ~2g | ~17× R_base | Accel mostly ignored, gyro dominates |
| Extreme (5g+) | ~4g | ~65× R_base | Accel essentially ignored |

This is a **smooth curve**, not a threshold. The filter naturally transitions between trusting the accelerometer and trusting the gyro.

### Magnetic Trust

```
mag_deviation = (||m_measured|| - ||m_expected||) / ||m_expected||
R_mag_adaptive = R_mag_base · (1 + k₂ · mag_deviation²)
```

Same idea: field strength deviation from expected → continuous distrust. Soft iron distortion from nearby metal gradually reduces mag trust rather than binary-rejecting it.

### Innovation-Based Adaptive R (Advanced)

Monitor the innovation sequence (measurement residual) relative to expected innovation covariance S:

```
normalized_innovation = yᵀ · S⁻¹ · y     (should be ~χ² distributed with m DOF)
```

If consistently large → sensor is degraded → inflate R. If consistently small → sensor is good → shrink R. This is the Normalized Innovation Squared (NIS) test, same approach used in the translational Kalman filter.

---

## 8. Filter Lifecycle & Timing

```mermaid
sequenceDiagram
    participant IMU as IMU (400 Hz)
    participant NOM as Nominal State
    participant EKF as Error-State EKF
    participant FLY as Flying Filter (5-20 Hz)
    participant OUT as Output

    loop Every IMU sample (2.5 ms)
        IMU->>NOM: gyro ω, accel a, mag m
        NOM->>NOM: q̂ ← q̂ ⊗ q((ω - b̂) · dt)
        
        NOM->>EKF: Predict (propagate P)
        
        EKF->>EKF: Accel update (adaptive R)
        EKF->>EKF: Mag update (adaptive R)
        
        EKF->>NOM: Inject δθ → correct q̂
        EKF->>EKF: Reset δx = 0
        
        NOM->>OUT: q (corrected quaternion)
    end

    FLY->>EKF: Reference orientation (when available)
    Note over EKF: Additional measurement update<br/>with flying filter R based on<br/>airspeed, GPS quality, flight mode
```

### Initialization

1. **Static detection**: If gyro magnitude < threshold for N samples → device is stationary
2. **Initial pitch/roll**: From accelerometer (gravity direction) — use TRIAD or simple trig
3. **Initial heading**: From magnetometer (if calibrated) or 0° (if no mag cal)
4. **Initial covariance P₀**: Large for heading (~30°²), moderate for pitch/roll (~5°²), large for biases
5. **Fast convergence period**: First 2-3 seconds, inflate process noise (like Madgwick's startup ramp) to rapidly converge from initial guess

### Steady State

- Nominal integration at 400 Hz (just quaternion kinematics — fast)
- Accel/mag measurement updates at 400 Hz (or downsampled if needed for performance)
- Flying filter updates at 5-20 Hz (when GPS samples arrive)
- Gyro bias converges over 10-30 seconds of flight
- Mag bias tracks slow environmental changes

### Export

Same format as current Madgwick output:
- Quaternion (qw, qx, qy, qz) per sample
- Euler angles (yaw, pitch, roll) per sample
- Calibrated sensor data (accel, gyro, mag in body frame)
- **New**: covariance diagonal (uncertainty per axis)
- **New**: gyro bias estimate
- **New**: which measurements were active/degraded per sample

---

## 9. Implementation Plan

### Phase 1: Core Error-State EKF (TypeScript, in sensor fusion viewer)

- [ ] Quaternion math utilities (multiply, conjugate, to/from axis-angle, to/from rotation matrix)
- [ ] Error-state EKF class: state vector, P, Q, R matrices
- [ ] Nominal state integrator (gyro → quaternion)
- [ ] Prediction step (propagate P using F, Q)
- [ ] Accelerometer measurement update
- [ ] Magnetometer measurement update
- [ ] Injection step (correct nominal state, reset error)
- [ ] Initialization from static accel/mag

### Phase 2: Adaptive Noise & Rejection

- [ ] Adaptive R_accel based on acceleration magnitude
- [ ] Adaptive R_mag based on field magnitude deviation
- [ ] Innovation monitoring (NIS test for each sensor)
- [ ] Configurable gain/rejection parameters (replaces Madgwick's β, accelRejection, magRejection)

### Phase 3: Flying Filter Reference Integration

- [ ] Reference measurement model (Euler angles or direction vector)
- [ ] Adaptive R_ref based on flight mode, airspeed, GPS quality
- [ ] Interface to receive flying filter output (orientation + confidence)
- [ ] Bidirectional: sensor EKF quaternion available to flying filter as high-rate IMU measurement

### Phase 4: Viewer Integration

- [ ] Drop-in replacement for Madgwick output in fusion viewer
- [ ] Side-by-side comparison mode (Madgwick vs EKF)
- [ ] Covariance ellipsoid visualization
- [ ] Bias estimate plots
- [ ] Sensor trust indicators (how much is each sensor contributing)

### Phase 5: Real-Time / Firmware

- [ ] C port for FlySight 2 firmware (if viable)
- [ ] Or: post-processing only, running on exported CSV data
- [ ] Performance profiling at 400 Hz on embedded target

---

## 10. Calibration Interface

The EKF changes how calibration works:

### What Stays the Same
- **Accelerometer 6-position calibration** — still needed for scale/offset/cross-axis
- **Magnetometer hard/soft iron calibration** — still needed for initial offset and ellipsoid correction

### What Changes
- **Mag bias is now estimated online.** The δb_mag state tracks slow changes in magnetic environment. The offline calibration provides the initial estimate; the filter refines it continuously.
- **Dynamic calibration offsets as a tuning knob** — Hartman's discovery that adjusting calibration dynamically can correct heading becomes a formal mechanism: the flying filter heading measurement achieves the same thing through proper Kalman fusion instead of ad-hoc offset manipulation.
- **Rejection settings become R matrix entries** — instead of Madgwick's binary thresholds (accelRejection=10°, magRejection=10°), you set base measurement noise and scaling factors. More parameters, but each one has a clear physical meaning.

### UI Controls (replacing Madgwick settings)

| Old (Madgwick) | New (EKF) | Physical Meaning |
|----------------|-----------|------------------|
| Gain (β) | Q_gyro (process noise) | How much to trust gyro vs other sensors |
| Acceleration rejection | k_accel (R scaling factor) | How aggressively to distrust accelerometer during dynamics |
| Magnetic rejection | k_mag (R scaling factor) | How aggressively to distrust magnetometer during distortion |
| Recovery trigger period | (automatic via covariance) | EKF naturally recovers when sensor quality improves |
| — (not available) | R_ref_heading | How much to trust flying filter heading |
| — (not available) | R_ref_pitch | How much to trust flying filter pitch |

---

## 11. Reference Material

### PDFs in this project directory

| File | Description |
|------|-------------|
| `sola-quaternion-error-state-ekf.pdf` | **Joan Solà — "Quaternion kinematics for the error-state Kalman filter"** (arXiv:1711.02508, 2017). THE reference for this implementation. 80 pages covering quaternion math, Lie group structure, error-state formulation, and complete EKF derivation. Sections 4-5 are the core. |
| `madgwick-phd-thesis.pdf` | **Sebastian Madgwick PhD thesis** (2014). Chapter 3: original algorithm. Chapter 7: revised algorithm (what we currently use via xio Fusion). Understanding the limitations motivates the EKF. |
| `markley-attitude-representations.pdf` | **F. Landis Markley — "Attitude Representations for Kalman Filtering"** (NASA, 2003). Survey of quaternion vs rotation vector representations for attitude EKFs. Discusses multiplicative vs additive quaternion error. |
| `kalman-original-1960.pdf` | **Rudolf Kalman — "A New Approach to Linear Filtering and Prediction Problems"** (1960). The original paper. Historical reference. |
| `lsm6dso.pdf` | LSM6DSO IMU datasheet — gyro/accel noise parameters for Q/R tuning |
| `lis2mdl.pdf` | LIS2MDL magnetometer datasheet — mag noise parameters |
| `an5259-lsm6dso-finite-state-machine-stmicroelectronics.pdf` | LSM6DSO FSM application note |
| `an5272-lsm6dso-machine-learning-core-stmicroelectronics.pdf` | LSM6DSO ML core application note |

### Online References

| Resource | URL | What's useful |
|----------|-----|---------------|
| AHRS Python library | ahrs.readthedocs.io | Working EKF, UKF, Madgwick, Mahony implementations. Good for validation. |
| xio Fusion (C library) | github.com/xioTechnologies/Fusion | Best Madgwick implementation. Our current algorithm basis. Rejection/recovery source code. |
| Kalman & Bayesian Filters in Python | github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python | Tutorial notebooks. Ch.14 on adaptive filtering for dynamic R. |
| VectorNav INS primer | vectornav.com/resources/inertial-navigation-primer | Attitude transformations, kinematics reference. |

### Key Equations to Know (Solà paper references)

| Concept | Solà Section | Equation |
|---------|-------------|----------|
| Quaternion product | §2.3 | q₁ ⊗ q₂ |
| Quaternion → rotation matrix | §2.4 | R{q} |
| Quaternion kinematics (gyro integration) | §4.1 | q̇ = ½ q ⊗ ω |
| Error-state dynamics | §5.3.3 | δθ̇ = -[ω×]δθ - δb + n |
| Error-state F matrix | §5.3.3 | F (full 15-state, we use 9-state subset) |
| Error-state injection | §5.4 | q ← q̂ ⊗ q(δθ), reset |
| Measurement Jacobian (gravity) | §7.1.2 | H for accelerometer observation |

---

## 12. Open Questions

### 12.1 — State Size: 9 vs 12 vs 15

Solà's full formulation is 15-state (δθ, δv, δp, δb_gyro, δb_accel). We don't need velocity or position (flying filter handles that). Options:

- **9-state** (δθ, δb_gyro, δb_mag): Minimum for our needs. Mag bias helps with dynamic calibration.
- **12-state** (add δb_accel): Accelerometer bias estimation. Useful if accel calibration drifts.
- **15-state** (add δv, δp): Full INS. Overkill — flying filter handles translation.

**Recommendation:** Start with 9, add accel bias (12) if needed after testing.

### 12.2 — Magnetometer: Full 3D vs Heading-Only

Full 3D mag measurement (3 equations) constrains all 3 orientation axes. Heading-only (1 scalar) only constrains yaw. Full 3D is more information but couples mag errors into pitch/roll. Heading-only is cleaner but wastes inclination information.

**Recommendation:** Start with heading-only. Add full 3D as option. Compare.

### 12.3 — Update Rate

Should accel/mag updates run at full 400 Hz or be downsampled? Full rate = more information but more computation. 50-100 Hz updates with 400 Hz gyro integration is common in industry.

**Recommendation:** Start at full 400 Hz (post-processing, not real-time constrained). Downsample later for firmware if needed.

### 12.4 — Flying Filter ↔ Sensor EKF Coupling Direction

Currently proposed as one-way: flying filter → sensor EKF (reference measurements). But the sensor EKF produces 400 Hz body rates that could be fed back to the flying filter's orientation EKF as high-rate measurements (replacing the SG-derived pseudo-measurements).

This creates a circular dependency. Options:
- **One-way only** (flying filter → sensor EKF): Simpler, no coupling issues
- **Alternating updates**: Flying filter at GPS rate, sensor EKF at IMU rate, each feeds the other
- **Unified filter**: Single large EKF that combines translational + rotational + sensor states (complex, probably not worth it)

**Recommendation:** Start one-way. Explore bidirectional in Phase 3 if heading quality is still insufficient.

### 12.5 — Madgwick as Fallback

Keep Madgwick as a selectable algorithm in the viewer for comparison and as a fallback when the EKF diverges (e.g., during initialization or sensor failure).

---

## Changelog

| Date | Change |
|------|--------|
| 2026-03-31 | Initial design document |

---

*This document defines the sensor fusion EKF architecture for the FlySight 2 project. It is the single source of truth for the algorithm design. Update it when the design changes.*
