# Sensor Fusion Algorithm Research & Recommendations

## Executive Summary for Flying/Skydiving Applications

**Primary Recommendation: x-io Fusion Library**

For your use case (skydiving, airplane calibration with G-forces and banking), the **x-io Fusion library** is the best choice. It's the evolution of the Madgwick algorithm with critical features specifically designed for high-dynamics scenarios.

---

## The Problem with Basic AHRS in Flight

Standard sensor fusion algorithms (basic Madgwick, basic Mahony) assume:
- Accelerometer measures **only gravity**
- Any acceleration = noise to be filtered

**In flight, this assumption is WRONG:**
- Airplane banking: accelerometer sees centrifugal + gravity
- Freefall: accelerometer sees ~0g
- Pull-out: accelerometer sees >1g
- The accelerometer is NOT measuring gravity direction!

---

## Algorithm Comparison for High-Dynamics

| Algorithm | Accel Rejection | Mag Rejection | Gyro Bias Est | Complexity | Best For |
|-----------|----------------|---------------|---------------|------------|----------|
| **x-io Fusion** | ✅ YES | ✅ YES | ✅ YES | Low | **Flying, high-G** |
| Madgwick (basic) | ❌ No | ❌ No | ❌ No | Low | Static/slow motion |
| Mahony | ❌ No | ❌ No | ✅ YES | Low | Slow motion |
| Extended Kalman (EKF) | ⚠️ Manual | ⚠️ Manual | ✅ YES | High | Custom tuning |
| Unscented Kalman (UKF) | ⚠️ Manual | ⚠️ Manual | ✅ YES | Very High | Research |

---

## Recommended: x-io Fusion Library

### Why Fusion is Perfect for Your Use Case

**Repository**: https://github.com/xioTechnologies/Fusion

Based on chapter 7 of Madgwick's PhD thesis (the *improved* algorithm, not the basic one from chapter 3 that everyone uses).

### Key Features for Flight:

#### 1. Acceleration Rejection
```
Reduces errors from linear and rotational motion accelerations.
- Calculates error between measured inclination and algorithm output
- If error > threshold, IGNORES accelerometer for that update
- Effectively: "If we're under G-forces, trust the gyro more"
```

**Settings:**
- `accelerationRejection`: Threshold in degrees (recommend 10°)
- `recoveryTriggerPeriod`: How long before recovery activates

#### 2. Magnetic Rejection
```
Reduces errors from temporary magnetic distortions.
- Same principle as acceleration rejection
- Compares heading measurement vs algorithm output
- Ignores magnetometer if difference exceeds threshold
```

**Settings:**
- `magneticRejection`: Threshold in degrees (recommend 10°)

#### 3. Angular Rate Recovery
```
Detects when gyroscope is saturated (exceeds measurement range).
- Triggers reinitialization if angular rate > 98% of gyro range
- Prevents filter from getting lost during extreme maneuvers
```

**Settings:**
- `gyroscopeRange`: Set to your sensor's max (2000 dps for LSM6DSO)

#### 4. Automatic Gyroscope Bias Estimation
```
Detects stationary periods and estimates gyro offset.
- Compensates for temperature drift
- Fine-tunes calibration during use
- Requires gyro < ±3 dps while stationary
```

### Fusion Algorithm Settings Structure

```c
typedef struct {
    FusionConvention convention;     // NWU, ENU, or NED
    float gain;                       // 0.5 typical, 0 = gyro only
    float gyroscopeRange;            // deg/s (sensor max)
    float accelerationRejection;     // degrees threshold
    float magneticRejection;         // degrees threshold
    unsigned int recoveryTriggerPeriod; // samples
} FusionAhrsSettings;
```

### Fusion Outputs
- **Quaternion**: Full orientation
- **Gravity vector**: Direction of gravity in sensor frame
- **Linear acceleration**: Accelerometer minus gravity
- **Earth acceleration**: Linear accel in Earth frame

---

## Alternative: Extended Kalman Filter (EKF)

If Fusion doesn't meet your needs, EKF gives maximum flexibility but requires more tuning.

### EKF Advantages:
- Can explicitly model different noise characteristics
- Can incorporate GPS velocity for better heading
- Can handle non-linear dynamics

### EKF Implementation Notes:
From the AHRS library documentation:

```python
# State vector: quaternion [qw, qx, qy, qz]
# Control input: angular velocity from gyroscope
# Measurement: accelerometer + magnetometer

# Noise parameters (tune these!):
sigma_gyro = 0.3   # rad/s - gyroscope noise
sigma_accel = 0.5  # m/s² - accelerometer noise  
sigma_mag = 0.8    # µT - magnetometer noise
```

### EKF for Flight:
To handle flight dynamics, you would need to:
1. Increase `sigma_accel` during high-G (trust accel less)
2. Use GPS velocity to validate/correct heading
3. Detect freefall and switch to gyro-only mode

---

## Mahony Filter (Alternative)

Good middle ground with gyro bias estimation.

### Key Equation:
```
q̇ = ½ q ⊗ (Ω - b̂ + kP·ωmes)
ḃ = -kI·ωmes
```

Where:
- `kP`: Proportional gain (default 1.0)
- `kI`: Integral gain (default 0.3)
- `ωmes`: Error term from accel/mag measurements
- `b̂`: Estimated gyro bias

### Mahony Tip for Flight:
> "When the IMU is subject to high magnitude accelerations (takeoff, landing 
> manoeuvres, etc.) it may be wise to **reduce the relative weighing of the 
> accelerometer data** compared to the magnetometer data."

You could implement dynamic weighting based on:
- Magnitude of acceleration (|a| - 1g)
- GPS-derived acceleration
- Your existing aerodynamic model

---

## Staged/Hybrid Approach

Your mention of "stages" is a valid approach:

### Stage 1: Complementary Filter (Fast)
- Simple weighted blend of gyro integration + accel/mag correction
- Very fast, low latency
- Use for short-term orientation

### Stage 2: Madgwick/Fusion (Medium)
- Gradient descent correction
- More robust to noise
- ~100µs per update

### Stage 3: EKF (Optional, for special cases)
- Full probabilistic estimation
- Can incorporate GPS, airspeed, etc.
- ~1ms per update

---

## Your Aerodynamic Kalman Filter Integration

You mentioned having a Kalman filter for aerodynamic orientation. Potential integration:

### Option A: Use Aero KF for Gravity Direction
```c
// During flight, use your aero KF to estimate "apparent vertical"
// Feed this to AHRS instead of raw accelerometer
float aero_gravity[3] = get_aero_vertical();
Fusion_UpdateAHRS(gyro, aero_gravity, mag);  // Modified!
```

### Option B: Validate Magnetometer Heading
```c
// Use GPS track to validate magnetic heading during flight
float gps_track = get_gps_ground_track();
float mag_heading = get_mag_heading();
float heading_error = gps_track - mag_heading;

if (abs(heading_error) > 30) {
    // Magnetic disturbance detected, trust gyro
    use_mag = false;
}
```

### Option C: Freefall Detection
```c
// During freefall, accelerometer is useless for tilt
float accel_mag = sqrt(ax*ax + ay*ay + az*az);
if (accel_mag < 0.3) {  // Less than 0.3g
    // Freefall! Only trust gyro + magnetometer
    Fusion_UpdateAHRS(gyro, NULL, mag);  // No accel correction
}
```

---

## Magnetometer Calibration in Flight

Your concern about calibrating in the airplane is valid.

### Hard Iron Calibration Requirements:
- Need to rotate through ALL orientations
- 60-90 seconds of tumbling
- Ideally done on ground, away from aircraft

### Soft Iron Calibration:
- More complex, usually done with offline tools
- Can be approximated as diagonal scale factors

### In-Flight Calibration Strategy:
1. Do basic calibration on ground before jump
2. Use the first few seconds of freefall for heading alignment
3. During tumbling in freefall, you naturally cover many orientations
4. Could refine calibration during stable flight portion

### Calibration in Airplane (Bank Angle Issue):
- The airplane's bank won't affect magnetometer calibration
- Magnetic field is fixed relative to Earth
- Your sitting position doesn't matter for mag
- **BUT**: Hard iron from aircraft electronics WILL affect it
- **Solution**: Calibrate on ground, use magnetic rejection during flight

---

## Recommended Implementation Path

### Phase 1: Get Fusion Working (1-2 days)
1. Port x-io Fusion to your test program
2. Test with your sensor data
3. Verify heading accuracy in static tests

### Phase 2: Add Dynamics Handling (2-3 days)
1. Tune acceleration rejection threshold
2. Test with simulated high-G data
3. Add freefall detection

### Phase 3: Integration (1 week)
1. Integrate with your aero Kalman filter
2. Add GPS heading validation
3. Test full VR calibration workflow

### Phase 4: FlySight Firmware (when ready)
1. Port proven algorithm to STM32
2. Integrate with BLE transmission
3. Test on actual hardware

---

## Research Papers & References

### Primary (Read These):
1. **Madgwick PhD Thesis** (Chapter 7 - the improved algorithm)
   - https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf
   - This is what Fusion is based on

2. **Original Madgwick Paper** (2011)
   - "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
   - IEEE International Conference on Rehabilitation Robotics

3. **Mahony et al.** (2008)
   - "Nonlinear Complementary Filters on the Special Orthogonal Group"
   - IEEE Transactions on Automatic Control

### Secondary (If Needed):
4. **Sabatini** (2011)
   - "Quaternion-Based Extended Kalman Filter for Determining Orientation by Inertial and Magnetic Sensing"
   - IEEE Transactions on Biomedical Engineering

5. **Valenti et al.** (2015)
   - "Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs"
   - Sensors Journal

### Aviation-Specific:
6. **Gebre-Egziabher et al.** (2006)
   - "MAV Attitude Determination by Vector Matching"
   - IEEE Aerospace and Electronic Systems

---

## Quick Links

| Resource | URL |
|----------|-----|
| x-io Fusion (C) | https://github.com/xioTechnologies/Fusion |
| AHRS Python Library | https://ahrs.readthedocs.io/en/latest/filters.html |
| Madgwick C Original | https://x-io.co.uk/downloads/madgwick_algorithm_c.zip |
| Madgwick PhD Thesis | https://x-io.co.uk/downloads/madgwick-phd-thesis.pdf |

---

## Summary Recommendation

**For your VR headset calibration during skydiving:**

1. **Use x-io Fusion** - It has acceleration rejection built in
2. **Enable acceleration rejection** at 10° threshold
3. **Enable magnetic rejection** at 10° threshold
4. **Calibrate magnetometer on ground** before each session
5. **Consider GPS heading validation** during calibration routine
6. **Test with real freefall data** to validate behavior

The Fusion library handles 90% of the "flying" problems automatically. You may need to add a freefall detector that switches to pure gyro integration when |accel| < 0.3g.
