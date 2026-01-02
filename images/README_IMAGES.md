# Hardware Photo Descriptions

Copy your photos into this folder with these names:

## Image Files to Add

### 1. `magnetometer_u18_back.jpg`
- Shows: Bottom of PCB with U18 (LIS2MDL magnetometer) and capacitors C36, C37
- Key info: Magnetometer is on BACK of board, opposite from IMU

### 2. `pcb_debug_header.jpg`  
- Shows: SWD debug header with pins labeled: PB10, SWO, SWCLK, SWDIO, NRST, GND
- Also shows: Serial number sticker "00176" and date "2023/17"

### 3. `pcb_top_gps.jpg`
- Shows: Top of PCB with u-blox NEO-M9N-00B-00 GPS module
- Also shows: Coordinate system silkscreen (X, Y, Z arrows)
- Note: IMU is also on this side (top)

### 4. `pcb_top_full.jpg`
- Shows: Full view of top PCB
- Visible: GPS, coordinate arrows, various components
- Note: IMU (LSM6DSO) location visible

### 5. `case_label_coords.jpg`
- Shows: Back of case with serial number S.N.: 2-00176
- Shows: Coordinate system diagram (Y up, X right, Z toward viewer)
- Shows: CE/FCC markings, "FLYSIGHT.CA", module IDs

---

## Coordinate System From Photos

From the case label and PCB silkscreen, the device coordinate system is:

```
Looking at FRONT of device (LED side):

     +Y (up, toward LED)
      ↑
      |
      |
←-----+-----→ +X (right)
      |
      |
      ↓
     -Y (down, toward USB port)

+Z points OUT of screen (toward you)
-Z points INTO device (toward battery/back)
```

---

## Key Observations

1. **Serial Number**: 2-00176 (early production unit, week 17 of 2023)

2. **Magnetometer Location**: U18 on back of PCB
   - This is CRITICAL for coordinate transformation
   - X and Z axes are inverted relative to IMU

3. **GPS Module**: u-blox NEO-M9N-00B-00
   - Date code: 2222 (week 22 of 2022)
   - Batch: 0501 12

4. **Debug Access**: SWD header available for ST-Link programming
   - Would allow flashing custom firmware if bootloader unavailable
