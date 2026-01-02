# FlySight 2 Sensor Fusion Viewer

A 3D visualization tool for tuning and validating the FlySight 2 sensor fusion algorithm.

## Features

- **CSV Parsing**: Load FlySight 2 SENSOR.CSV files with interleaved IMU/MAG data
- **Madgwick AHRS**: Real-time sensor fusion with tunable parameters
- **3D Visualization**: Three.js-based orientation display with device model
- **Playback Controls**: Play, pause, seek through sensor data at various speeds
- **Parameter Tuning**: Adjust filter gain (beta) and magnetometer calibration in real-time

## Quick Start

```bash
# Install dependencies
npm install

# Start development server
npm run dev
```

Then open http://localhost:3000 in your browser.

## Usage

1. Click "Load Sensor CSV" and select a FlySight 2 SENSOR.CSV file
2. Use playback controls to step through the data
3. Adjust the **Beta** slider to tune filter responsiveness
4. Toggle **Apply Axis Transform** for magnetometer coordinate correction
5. Set **Hard Iron Offset** values for magnetometer calibration

## Critical Notes

### Magnetometer Coordinate Transform

The LIS2MDL magnetometer is mounted on the **back** of the PCB, opposite from the IMU. This means the magnetometer axes are mirrored:

```c
// Must apply this transform!
float mx_device = -mx_raw;  // X inverted
float my_device =  my_raw;  // Y same
float mz_device = -mz_raw;  // Z inverted
```

**If you disable the axis transform, heading will be completely wrong.**

### Sensor Rates

- IMU: ~400 Hz (accelerometer + gyroscope)
- Magnetometer: ~100 Hz
- Data is interleaved in the CSV file

### Units

- Gyroscope: deg/s (converted to rad/s internally)
- Accelerometer: g
- Magnetometer: gauss

## Files

### Web Application (`src/`)

| File | Description |
|------|-------------|
| `main.ts` | Application entry point, event handling |
| `fusion.ts` | Madgwick AHRS implementation (TypeScript) |
| `csvParser.ts` | FlySight CSV file parser |
| `viewer.ts` | Three.js 3D visualization |
| `styles.css` | UI styling |

### Firmware (`firmware/`)

| File | Description |
|------|-------------|
| `fusion.h` | C header for STM32 integration |
| `fusion.c` | C implementation (portable, no malloc) |

## Algorithm

This project implements the **Madgwick AHRS filter** for 9-DOF sensor fusion:

- Uses gradient descent to minimize error between sensor measurements and estimated orientation
- Single tuning parameter: **beta** (filter gain)
  - Higher beta = faster convergence, more noise
  - Lower beta = slower convergence, smoother output
  - Recommended starting value: **0.1**

### Asynchronous Update Strategy

Since IMU and MAG run at different rates:

1. Store the latest magnetometer reading when it arrives (~100 Hz)
2. Run full 9-DOF fusion on each IMU sample using stored MAG data (~400 Hz)
3. Use actual timestamps for dt calculation, not assumed fixed rate

## Integration into STM32 Firmware

The `firmware/` folder contains C code ready for direct integration:

```c
#include "fusion.h"

// Initialize
FusionConfig config = {
    .beta = 0.1f,
    .apply_mag_transform = true
};
Fusion_Init(&config);

// In IMU callback (~400 Hz):
Fusion_UpdateIMU(dt, gx*DEG_TO_RAD, gy*DEG_TO_RAD, gz*DEG_TO_RAD, ax, ay, az);

// In MAG callback (~100 Hz):
Fusion_UpdateMag(mx, my, mz);  // Transform applied internally

// Get output:
FusionOutput output;
Fusion_GetOutput(&output);
// Use output.heading, output.q[], etc.
```

## Success Criteria

- Static heading accuracy: ±2°
- Dynamic heading tracking: ±5°
- Convergence time: < 3 seconds
- Computational efficiency: < 100µs per update on Cortex-M4

## References

- [Madgwick Filter Paper](https://x-io.co.uk/res/doc/madgwick_internal_report.pdf)
- [x-io Fusion Library](https://github.com/xioTechnologies/Fusion)
- [LSM6DSO Datasheet](https://www.st.com/resource/en/datasheet/lsm6dso.pdf)
- [LIS2MDL Datasheet](https://www.st.com/resource/en/datasheet/lis2mdl.pdf)
