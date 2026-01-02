# FlySight 2 Sensor Fusion Test Program - Complete Handoff Package

## Quick Start for New AI Agent

**Goal**: Build a standalone C sensor fusion test program that:
1. Parses FlySight 2 SENSOR.CSV files
2. Implements Madgwick AHRS filter
3. Handles magnetometer coordinate transform (critical!)
4. Outputs heading/orientation for VR headset calibration
5. Is designed for direct integration into STM32 firmware

---

## Files in This Package

| File | Description |
|------|-------------|
| `README.md` | This file - start here |
| `PROJECT_OVERVIEW.md` | Detailed specs, algorithms, API design |
| `COORDINATE_SYSTEMS.md` | **CRITICAL** - Sensor axis transformations |
| `sample_sensor_data.csv` | Real FlySight 2 sensor log for testing |
| `images/` | Photos of hardware showing sensor locations |

---

## Critical Points (Read First!)

### 1. Magnetometer is on OPPOSITE side of PCB from IMU

The LIS2MDL magnetometer is mounted on the **back** of the board. This means:

```c
// MUST apply this transform to magnetometer data
float mx_device = -mx_raw;  // X inverted
float my_device =  my_raw;  // Y same  
float mz_device = -mz_raw;  // Z inverted
```

**If you skip this, heading will be completely wrong.**

### 2. Sensors Run at Different Rates

- IMU: ~400 Hz (2.4ms between samples)
- Magnetometer: ~100 Hz (10ms between samples)
- Data is interleaved in CSV, use timestamps for dt

### 3. Units in CSV

- Gyro: deg/s → convert to rad/s for fusion
- Accel: g (no conversion needed)
- Mag: gauss (normalize before use)

---

## Recommended Implementation Order

1. **CSV Parser** - Read sample_sensor_data.csv
2. **Quaternion Math** - multiply, normalize, to_euler
3. **Madgwick Filter** - Start with beta=0.1
4. **Coordinate Transform** - Apply mag axis inversion
5. **Mag Calibration** - Hard iron offset removal
6. **Output** - Heading in degrees, Euler angles
7. **Validation** - Compare to known orientations

---

## Expected Output (Static Device)

When device is flat, LED up, pointing North:
- Roll: ~0°
- Pitch: ~0°  
- Heading: ~0° (or 360°)

When rotated 90° clockwise (pointing East):
- Heading: ~90°

---

## Hardware Summary

| Component | Part Number | Interface | Rate |
|-----------|-------------|-----------|------|
| MCU | STM32WB5MMGH | - | 64MHz |
| IMU | LSM6DSO | SPI | 416 Hz |
| Magnetometer | LIS2MDL | I2C | 100 Hz |
| GPS | u-blox NEO-M9N | UART | 5 Hz |
| Barometer | LPS22HH | I2C | configurable |

---

## Integration Target

The fusion code must be:
- Pure C (C99)
- No malloc/free
- Single-precision float only
- <1KB RAM
- <100µs per update on Cortex-M4

Final destination: `FlySight/fusion.c` and `FlySight/fusion.h` in firmware repo.

---

## Test Data Info

`sample_sensor_data.csv` contains:
- ~15 seconds of data
- Device at rest (minor handling)
- IMU working correctly (v2024.12.30 firmware)
- Firmware version in header

---

## Questions?

The PROJECT_OVERVIEW.md has exhaustive detail on:
- CSV format with all fields
- Madgwick algorithm explanation
- Magnetometer calibration procedures
- Data collection procedures
- Complete API design
- Tuning parameters

The COORDINATE_SYSTEMS.md explains:
- Physical sensor locations (with photos)
- Axis transformations needed
- Verification tests

---

## Success Criteria

1. Heading accuracy ±2° when static
2. Heading tracks rotation within ±5°
3. No numerical instability
4. Converges in <3 seconds
5. Code compiles with arm-none-eabi-gcc
