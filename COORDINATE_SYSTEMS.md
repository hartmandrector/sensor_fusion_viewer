# FlySight 2 Coordinate Systems - Critical Information

## Device Orientation Reference

When looking at the **FRONT** of the FlySight 2 (the side with the LED light):

```
        LED (top front)
            ↑
      +Y    |
       ↑    |
       |    |
       |    |
  ←----+---→ +X (right side)
       |
       ↓
      -Y
   USB port / Power button (bottom front)
```

- **+X**: Points to the RIGHT (when looking at front)
- **+Y**: Points UP toward the LED
- **+Z**: Points OUT of the front face (toward viewer)
- **-Z**: Points INTO the device (toward back/battery)

This coordinate system is:
- Printed on the back case label
- Silkscreened on the PCB near the IMU

---

## Sensor Locations on PCB

### TOP side of PCB (front of device, same side as LED):
- **IMU (LSM6DSO)** - Accelerometer + Gyroscope
- **GPS (u-blox NEO-M9N)**
- **Barometer**
- **Humidity sensor**

### BOTTOM side of PCB (back of device, battery side):
- **Magnetometer (LIS2MDL)** - U18
- Decoupling capacitors C36, C37

---

## CRITICAL: Magnetometer Axis Transformation

Since the magnetometer is mounted on the **opposite side** of the PCB from the IMU, the coordinate systems are **mirrored**.

### Physical Reality:
When you flip a PCB over:
- X axis: **REVERSED** (mirrored left-right)
- Y axis: Same direction (up is still up)
- Z axis: **REVERSED** (what was "out" is now "in")

### Axis Mapping (Magnetometer to Device Frame):

```c
// Raw magnetometer readings from LIS2MDL (U18 on back of PCB)
float mag_raw_x, mag_raw_y, mag_raw_z;

// Transform to device coordinate frame (matching IMU)
float mag_device_x = -mag_raw_x;  // X is mirrored
float mag_device_y =  mag_raw_y;  // Y is same
float mag_device_z = -mag_raw_z;  // Z is mirrored
```

### Why This Matters:
If you don't apply this transformation:
- Heading will be wrong by 180° in some orientations
- Roll/pitch coupling with heading will be inverted
- Sensor fusion will produce garbage output

### Verification Test:
1. Point device NORTH (LED facing north)
2. Device should be flat (LED up)
3. Expected magnetometer readings (after transform):
   - mag_x: **positive** (pointing north)
   - mag_y: ~0 (horizontal)
   - mag_z: **negative** in northern hemisphere (field dips down)

---

## LIS2MDL Datasheet Reference

The LIS2MDL has its coordinate system printed on the package. When mounted on the **bottom** of the PCB:

```
    LIS2MDL on BOTTOM of PCB (looking at bottom of board)
    
    Package marking shows X, Y arrows
    But when assembled, viewing from FRONT of device:
    
    The magnetometer's +X points LEFT (device's -X)
    The magnetometer's +Z points INTO device (device's -Z)
```

---

## Complete Sensor Data Processing Pipeline

```c
// 1. Read raw IMU data (already in device frame)
float gx = imu_raw.wx * DEG_TO_RAD;  // deg/s -> rad/s
float gy = imu_raw.wy * DEG_TO_RAD;
float gz = imu_raw.wz * DEG_TO_RAD;
float ax = imu_raw.ax;  // Already in g
float ay = imu_raw.ay;
float az = imu_raw.az;

// 2. Read raw magnetometer data and TRANSFORM to device frame
float mx = -mag_raw.x;  // Mirror X
float my =  mag_raw.y;  // Y unchanged
float mz = -mag_raw.z;  // Mirror Z

// 3. Apply magnetometer calibration (in device frame)
mx = (mx - cal.offset_x) * cal.scale_x;
my = (my - cal.offset_y) * cal.scale_y;
mz = (mz - cal.offset_z) * cal.scale_z;

// 4. Normalize magnetometer
float mag_norm = sqrtf(mx*mx + my*my + mz*mz);
mx /= mag_norm;
my /= mag_norm;
mz /= mag_norm;

// 5. Run fusion algorithm
Fusion_Update(dt, gx, gy, gz, ax, ay, az, mx, my, mz);
```

---

## Verification Procedure

### Test 1: Gravity Vector
1. Place device flat, LED facing up
2. Expected accelerometer: ax≈0, ay≈0, az≈+1.0g
3. This confirms IMU Z points out the front (same as case marking)

### Test 2: Magnetometer Polarity
1. Place device flat, LED facing up
2. Point USB port (bottom) toward magnetic SOUTH
3. This means +Y points NORTH
4. Expected magnetometer (after transform): my should be **positive**

### Test 3: Heading Rotation
1. Start with device pointing North
2. Rotate 90° clockwise (now pointing East)
3. Heading output should change from 0° to 90°
4. If it goes to 270°, the X-axis transform is wrong

---

## Summary Table

| Sensor | Location | X Transform | Y Transform | Z Transform |
|--------|----------|-------------|-------------|-------------|
| IMU (LSM6DSO) | TOP (front) | +1 (none) | +1 (none) | +1 (none) |
| Magnetometer (LIS2MDL) | BOTTOM (back) | **-1 (invert)** | +1 (none) | **-1 (invert)** |

---

## Notes for Implementation

1. **Apply transform BEFORE calibration** - The calibration values are in device frame
2. **Apply transform in firmware** - Don't rely on post-processing
3. **Document clearly** - Future maintainers need to know why axes are inverted
4. **Test empirically** - Verify with compass app or known heading reference

---

## Reference Images

See the `images/` folder for:
- `coord_system_case.jpg` - Case label showing X, Y, Z axes
- `coord_system_pcb.jpg` - PCB silkscreen with axes
- `magnetometer_u18.jpg` - U18 (LIS2MDL) location on back of PCB
- `pcb_top.jpg` - Top of PCB showing GPS, IMU location
- `serial_number.jpg` - S.N.: 2-00176, date 2023/17

These confirm the physical orientation described above.
