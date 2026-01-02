# FlySight 2 Sensor Fusion Testing Program - Project Handoff

## Executive Summary

Build a **standalone C/C++ sensor fusion testing program** that:
1. Parses FlySight 2 SENSOR.CSV log files
2. Implements AHRS (Attitude and Heading Reference System) sensor fusion
3. Outputs orientation quaternions/Euler angles
4. Is designed for **direct integration** into STM32 firmware after validation

The end goal is VR headset heading calibration using FlySight 2's magnetometer as an absolute heading reference during skydiving.

---


---

## CRITICAL UPDATE: Magnetometer Coordinate Transform

**The magnetometer (LIS2MDL) is mounted on the BACK of the PCB, opposite from the IMU.**

This means the magnetometer axes are mirrored relative to the device frame:

`c
// APPLY THIS TRANSFORM BEFORE ANY OTHER MAG PROCESSING
float mx_device = -mag_raw_x;  // X axis inverted
float my_device =  mag_raw_y;  // Y axis same
float mz_device = -mag_raw_z;  // Z axis inverted
`

See `COORDINATE_SYSTEMS.md` for full details with diagrams and verification procedures.

---


## Hardware Platform Reference

### Target MCU (for eventual integration)
- **MCU**: STM32WB5MMGH (Cortex-M4F @ 64MHz + Cortex-M0+ for BLE)
- **FPU**: Single-precision hardware floating point
- **Compiler**: arm-none-eabi-gcc
- **Language**: C (C99), can use C++ if needed but pure C preferred for firmware

### Sensors

#### IMU: LSM6DSO (ST Microelectronics)
- **Interface**: SPI @ 10MHz
- **Accelerometer**:
  - Range: ±2g, ±4g, ±8g, ±16g (configurable via `Accel_FS` config)
  - ODR: 12.5 to 6667 Hz (configurable via `Accel_ODR` config)
  - Default: ±8g, 416 Hz
  - Units in CSV: **g** (1g = 9.80665 m/s²)
  
- **Gyroscope**:
  - Range: ±125, ±250, ±500, ±1000, ±2000 dps (configurable via `Gyro_FS`)
  - ODR: 12.5 to 6667 Hz (configurable via `Gyro_ODR`)
  - Default: ±2000 dps, 416 Hz
  - Units in CSV: **deg/s** (degrees per second)

- **Temperature**: Internal die temperature, °C

#### Magnetometer: LIS2MDL (ST Microelectronics)
- **Interface**: I2C @ 400kHz
- **Range**: ±50 gauss (fixed)
- **ODR**: 10, 20, 50, 100 Hz (configurable via `Mag_ODR`)
- **Default**: 100 Hz
- **Units in CSV**: **gauss**
- **Note**: Much lower sample rate than IMU - fusion algorithm must handle asynchronous updates

#### Barometer: LPS22HH
- **ODR**: 1-200 Hz (configurable via `Baro_ODR`)
- **Units**: Pa (pressure), °C (temperature)
- **Use case**: Altitude reference, not needed for heading fusion

---

## SENSOR.CSV File Format

### Header Structure
```
$FLYS,1
$VAR,FIRMWARE_VER,v2024.12.30
$VAR,DEVICE_ID,0017003b3253501720373657
$VAR,SESSION_ID,4b0d914c2105048d02093406
$COL,BARO,time,pressure,temperature
$UNIT,BARO,s,Pa,deg C
$COL,HUM,time,humidity,temperature
$UNIT,HUM,s,percent,deg C
$COL,MAG,time,x,y,z,temperature
$UNIT,MAG,s,gauss,gauss,gauss,deg C
$COL,IMU,time,wx,wy,wz,ax,ay,az,temperature
$UNIT,IMU,s,deg/s,deg/s,deg/s,g,g,g,deg C
$COL,TIME,time,tow,week
$UNIT,TIME,s,s,
$COL,VBAT,time,voltage
$UNIT,VBAT,s,volt
$DATA
```

### Data Lines
```
$IMU,232.009,10.375,-21.362,-32.348,0.11816,-0.12109,0.80615,27.73
$IMU,232.011,-6.103,-10.742,-8.483,0.14111,-0.14648,0.98046,27.73
$MAG,232.255,-0.351,0.172,-0.871,27.3
$BARO,232.249,86424.51,28.79
```

### Field Definitions

#### $IMU
| Field | Index | Description | Units |
|-------|-------|-------------|-------|
| time | 1 | Timestamp since boot | seconds |
| wx | 2 | Gyro X (roll rate) | deg/s |
| wy | 3 | Gyro Y (pitch rate) | deg/s |
| wz | 4 | Gyro Z (yaw rate) | deg/s |
| ax | 5 | Accel X | g |
| ay | 6 | Accel Y | g |
| az | 7 | Accel Z | g |
| temp | 8 | Die temperature | °C |

#### $MAG
| Field | Index | Description | Units |
|-------|-------|-------------|-------|
| time | 1 | Timestamp since boot | seconds |
| x | 2 | Magnetic field X | gauss |
| y | 3 | Magnetic field Y | gauss |
| z | 4 | Magnetic field Z | gauss |
| temp | 5 | Die temperature | °C |

### Timestamp Characteristics
- **IMU**: ~2.4ms between samples at 416 Hz (actual ~400 Hz observed)
- **MAG**: ~10ms between samples at 100 Hz
- **Timestamps are NOT perfectly uniform** - use actual dt between samples for integration
- Sensors are **asynchronous** - MAG updates arrive interleaved with IMU data

---

## Coordinate System

### FlySight 2 Body Frame (NED convention assumed)
Verify with physical device, but typical convention:
- **X**: Forward (out the "nose" of the device)
- **Y**: Right
- **Z**: Down

When device is flat on table:
- ax ≈ 0, ay ≈ 0, az ≈ +1.0 g (gravity pointing down)
- Magnetometer points toward magnetic north (X positive when device faces north)

### Output Frame
- Standard aerospace NED (North-East-Down) or
- ENU (East-North-Up) for compatibility with Android

---

## Recommended Sensor Fusion Algorithm

### Primary Recommendation: Madgwick Filter
- **Pros**: Computationally efficient, single tuning parameter (beta), well-documented
- **Cons**: Can have issues with magnetic disturbances
- **Reference Implementation**: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/

### Alternative: Mahony Filter
- **Pros**: Two tuning parameters (Kp, Ki) allow integral feedback for gyro bias
- **Cons**: Slightly more complex tuning
- **Good for**: Long-duration flights where gyro bias drift matters

### Key Implementation Notes

```c
// Madgwick filter - key parameters
typedef struct {
    float q0, q1, q2, q3;  // Quaternion (w, x, y, z)
    float beta;             // Filter gain (typically 0.01 - 0.5)
    float sampleFreq;       // Expected sample frequency for normalization
} MadgwickAHRS;

// CRITICAL: Handle asynchronous sensor updates
// - Run gyro integration at IMU rate (~400 Hz)
// - Apply magnetometer correction only when new MAG sample arrives (~100 Hz)
// - Use actual dt from timestamps, not assumed fixed rate
```

### Asynchronous Update Strategy

```c
void fusion_update(float timestamp, float gx, float gy, float gz, 
                   float ax, float ay, float az,
                   float mx, float my, float mz, bool mag_valid) {
    
    float dt = timestamp - last_timestamp;
    last_timestamp = timestamp;
    
    if (mag_valid) {
        // Full 9-DOF update with magnetometer
        MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
    } else {
        // 6-DOF update (gyro + accel only)
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
    }
}
```

---

## Magnetometer Calibration

### Hard Iron Calibration (Offset)
Permanent magnetic fields from device electronics cause constant offset.

```c
typedef struct {
    float offset_x, offset_y, offset_z;  // Hard iron offsets
} MagCalibration;

// Apply calibration
float mx_cal = mx_raw - cal.offset_x;
float my_cal = my_raw - cal.offset_y;
float mz_cal = mz_raw - cal.offset_z;
```

### Soft Iron Calibration (Scale/Rotation)
Non-spherical response due to nearby ferromagnetic materials.

```c
typedef struct {
    float offset[3];      // Hard iron
    float scale[3][3];    // Soft iron (3x3 matrix)
} MagCalibrationFull;

// Apply full calibration
float m_raw[3] = {mx, my, mz};
float m_cal[3];
for (int i = 0; i < 3; i++) {
    m_cal[i] = 0;
    for (int j = 0; j < 3; j++) {
        m_cal[i] += cal.scale[i][j] * (m_raw[j] - cal.offset[j]);
    }
}
```

### Calibration Data Collection Procedure
1. Place FlySight in a location away from magnetic interference
2. Enable logging with high MAG ODR (100 Hz)
3. Slowly rotate device through ALL orientations over 60-90 seconds:
   - Rotate 360° around X axis
   - Rotate 360° around Y axis
   - Rotate 360° around Z axis
   - Tumble randomly to fill in gaps
4. Fit collected points to ellipsoid, compute transformation to sphere
5. Tools: Use MATLAB, Python (scipy), or implement ellipsoid fitting in C

### Expected Magnetometer Values
- Earth's field strength: ~0.25 to 0.65 gauss depending on location
- In sample data: magnitude ≈ sqrt(0.35² + 0.17² + 0.87²) ≈ 0.95 gauss
- This suggests some hard iron offset (ideal would be ~0.5 gauss)

---

## Data Collection Procedures

### Test 1: Static Calibration
**Purpose**: Verify sensor readings, calibrate magnetometer
**Procedure**:
1. Place device on flat, level surface
2. Point device North (use compass reference)
3. Record 30 seconds of static data
4. Rotate to East, South, West - 30 seconds each
5. Record expected: az ≈ 1.0g, gyros ≈ 0, mag should rotate

### Test 2: Magnetometer Calibration Collection
**Purpose**: Collect data for hard/soft iron calibration
**Procedure**:
1. Config: `Mag_ODR: 3` (100 Hz)
2. Start logging
3. Tumble device slowly through all orientations for 90 seconds
4. Stop logging
5. Process data offline to compute calibration parameters

### Test 3: Dynamic Rotation Test
**Purpose**: Validate fusion algorithm heading tracking
**Procedure**:
1. Start with device pointing North on flat surface
2. Slowly rotate 90° clockwise (to East) over 5 seconds
3. Hold 5 seconds
4. Rotate 90° more (to South), hold
5. Continue full 360° rotation
6. Verify fusion output matches expected headings

### Test 4: Simulated Freefall
**Purpose**: Test behavior during high-G, high rotation rates
**Procedure**:
1. Record data during actual skydive OR
2. Simulate by shaking/rotating vigorously
3. Verify fusion doesn't diverge under stress

### Test 5: VR Headset Integration Simulation
**Purpose**: Simulate the actual use case
**Procedure**:
1. Mount FlySight on VR headset (or simulate mounting)
2. Perform head movements: look left, right, up, down
3. Verify heading output is stable and accurate
4. Test calibration handshake: quick alignment routine

---

## Program Architecture

### Recommended File Structure

```
sensor_fusion_test/
├── src/
│   ├── main.c                 # Test harness, CSV parsing, output
│   ├── csv_parser.c           # SENSOR.CSV parsing
│   ├── csv_parser.h
│   ├── fusion.c               # Madgwick/Mahony implementation
│   ├── fusion.h
│   ├── mag_calibration.c      # Magnetometer calibration
│   ├── mag_calibration.h
│   ├── quaternion.c           # Quaternion math utilities
│   ├── quaternion.h
│   └── config.h               # Compile-time configuration
├── data/
│   └── sample_sensor.csv      # Test data files
├── output/
│   └── (generated results)
├── Makefile
└── README.md
```

### Key Design Requirements for Firmware Integration

1. **No dynamic memory allocation** - Use static buffers
2. **No standard library dependencies** except math.h
3. **Single-precision float only** (no double)
4. **Avoid division where possible** - Use reciprocals
5. **No file I/O in fusion code** - CSV parsing separate from fusion
6. **Configurable at compile time** - Use #defines for tuning

### API Design for Firmware Integration

```c
// fusion.h - This exact API should work in firmware

#ifndef FUSION_H
#define FUSION_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float q[4];           // Quaternion [w, x, y, z]
    float euler[3];       // Euler angles [roll, pitch, yaw] in radians
    float heading;        // Magnetic heading in degrees (0-360)
} FusionOutput;

typedef struct {
    float beta;           // Madgwick gain
    float mag_offset[3];  // Hard iron calibration
    float mag_scale[3];   // Soft iron diagonal (simplified)
} FusionConfig;

// Initialize fusion state
void Fusion_Init(const FusionConfig *config);

// Update with IMU data only (called at IMU rate, ~400 Hz)
void Fusion_UpdateIMU(float dt, float gx, float gy, float gz,
                      float ax, float ay, float az);

// Update with magnetometer data (called at MAG rate, ~100 Hz)
void Fusion_UpdateMag(float mx, float my, float mz);

// Get current orientation
void Fusion_GetOutput(FusionOutput *output);

// Reset to initial state
void Fusion_Reset(void);

#endif
```

### CSV Parser (Test Harness Only)

```c
// csv_parser.h - NOT for firmware, test harness only

typedef enum {
    SENSOR_IMU,
    SENSOR_MAG,
    SENSOR_BARO,
    SENSOR_UNKNOWN
} SensorType;

typedef struct {
    SensorType type;
    float timestamp;
    union {
        struct { float wx, wy, wz, ax, ay, az, temp; } imu;
        struct { float x, y, z, temp; } mag;
        struct { float pressure, temp; } baro;
    } data;
} SensorReading;

// Parse one line from CSV, returns false on EOF or error
bool CSV_ParseLine(FILE *f, SensorReading *reading);
```

---

## Output Format

### Console Output (for validation)
```
Time: 232.009s | Heading: 45.2° | Pitch: 2.1° | Roll: -0.3°
Time: 232.011s | Heading: 45.3° | Pitch: 2.2° | Roll: -0.2°
...
```

### CSV Output (for analysis)
```
time,qw,qx,qy,qz,roll,pitch,yaw,heading
232.009,0.9238,0.0,0.0,0.3826,0.1,2.1,45.2,45.2
232.011,0.9237,0.0,0.0,0.3827,0.1,2.2,45.3,45.3
```

### Visualization (optional but recommended)
- Plot heading vs time
- Plot quaternion components vs time
- 3D visualization of orientation (can use Python matplotlib for post-processing)

---

## Tuning Parameters

### Madgwick Filter

| Parameter | Range | Notes |
|-----------|-------|-------|
| beta | 0.01 - 0.5 | Higher = faster convergence, more noise |
| Start | 0.1 | Good starting point |
| Static | 0.033 | Original Madgwick paper value |
| Dynamic | 0.3 - 0.5 | For high-motion scenarios |

### Recommendations
- Start with beta = 0.1
- If heading drifts slowly, increase beta
- If heading is noisy/jittery, decrease beta
- Consider adaptive beta based on motion (low during static, high during motion)

---

## Unit Conversions Reference

```c
// Convert degrees to radians (required for fusion math)
#define DEG_TO_RAD  (M_PI / 180.0f)
#define RAD_TO_DEG  (180.0f / M_PI)

// Gyro data comes in deg/s, convert to rad/s for fusion
float gx_rad = gx_deg * DEG_TO_RAD;

// Accel data is already in g, most algorithms expect this

// Mag data is in gauss, normalize before use
float mag_norm = sqrtf(mx*mx + my*my + mz*mz);
float mx_n = mx / mag_norm;
float my_n = my / mag_norm;
float mz_n = mz / mag_norm;
```

---

## Testing Checklist

- [ ] CSV parser correctly reads all sensor types
- [ ] Timestamps are correctly parsed as floats
- [ ] Fusion initializes to identity quaternion
- [ ] Static test: output stable, heading correct
- [ ] Rotation test: heading tracks rotation correctly
- [ ] Mag calibration improves heading accuracy
- [ ] No numerical instabilities (NaN, inf)
- [ ] Memory usage is bounded (no leaks)
- [ ] Code compiles with arm-none-eabi-gcc
- [ ] No double precision operations
- [ ] No malloc/free calls in fusion code

---

## Sample Data Reference

From actual FlySight 2 log (v2024.12.30):

```csv
$IMU,232.009,10.375,-21.362,-32.348,0.11816,-0.12109,0.80615,27.73
$IMU,232.011,-6.103,-10.742,-8.483,0.14111,-0.14648,0.98046,27.73
$MAG,232.255,-0.351,0.172,-0.871,27.3
$MAG,232.265,-0.358,0.177,-0.876,27.6
```

Observations from sample:
- IMU rate: ~400 Hz (2.4ms between samples)
- MAG rate: ~100 Hz (10ms between samples)
- Device at rest: az ≈ 0.98g (slight tilt)
- Gyros show small motion (device being handled)
- Mag values suggest device pointing roughly south (negative X)

---

## Integration Path Back to FlySight Firmware

1. **Validate fusion.c/h in test program** with real sensor data
2. **Copy fusion.c/h directly** into FlySight/fusion.c, FlySight/fusion.h
3. **Add to build** in STM32CubeIDE project
4. **Call Fusion_Init()** during active mode initialization
5. **Call Fusion_UpdateIMU()** from IMU data ready callback
6. **Call Fusion_UpdateMag()** from MAG data ready callback
7. **Transmit quaternion via BLE** using existing MAG BLE characteristic (repurpose or add new)

### Firmware Integration Points

```c
// In FlySight/active_mode.c or similar

// During init:
FusionConfig config = {
    .beta = 0.1f,
    .mag_offset = {0.0f, 0.0f, 0.0f},  // From calibration
    .mag_scale = {1.0f, 1.0f, 1.0f}
};
Fusion_Init(&config);

// In IMU callback (FS_IMU_DataReady_Callback or similar):
Fusion_UpdateIMU(dt, gx*DEG_TO_RAD, gy*DEG_TO_RAD, gz*DEG_TO_RAD, ax, ay, az);

// In MAG callback:
Fusion_UpdateMag(mx, my, mz);

// For BLE transmission:
FusionOutput output;
Fusion_GetOutput(&output);
// Send output.q[0..3] over BLE
```

---

## Resources

### Madgwick Filter
- Original paper: https://x-io.co.uk/res/doc/madgwick_internal_report.pdf
- Reference implementation: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
- GitHub: https://github.com/xioTechnologies/Fusion

### Magnetometer Calibration
- Ellipsoid fitting: https://www.mathworks.com/help/nav/ug/magnetometer-calibration.html
- Quality tutorial: https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration

### STM32 Reference
- FlySight 2 firmware repo: https://github.com/flysight/flysight-2-firmware
- LSM6DSO datasheet: ST website
- LIS2MDL datasheet: ST website

---

## Known Issues / Gotchas

1. **Timestamp rollover**: Timestamps are floats, may lose precision after long sessions
2. **Magnetic declination**: Magnetometer gives magnetic north, not true north. Add local declination offset for true heading.
3. **Body frame alignment**: Verify sensor axes match expected orientation
4. **Initial convergence**: Filter needs a few seconds of static data to converge to correct orientation
5. **Gimbal lock**: Euler angles have singularities at ±90° pitch - use quaternions internally

---

## Success Criteria

1. **Static heading accuracy**: ±2° when device is stationary
2. **Dynamic heading tracking**: Heading change matches physical rotation within ±5°
3. **Convergence time**: < 3 seconds from startup to stable heading
4. **Computational efficiency**: < 100µs per fusion update on Cortex-M4
5. **Memory footprint**: < 1KB RAM for fusion state

---

*Document created: January 2, 2026*
*For: FlySight 2 VR Heading Calibration Project*
*Target firmware version: v2024.12.30 (confirmed working IMU)*

