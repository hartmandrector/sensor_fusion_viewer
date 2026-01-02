/**
 * FlySight 2 Sensor Fusion - Madgwick AHRS Algorithm
 * 
 * C implementation for STM32 firmware integration.
 * 
 * Features:
 * - Single-precision float only (for Cortex-M4F FPU)
 * - No dynamic memory allocation
 * - No standard library dependencies (except math.h)
 * - Portable and efficient
 * 
 * Based on: "An efficient orientation filter for inertial and inertial/magnetic sensor arrays"
 * by Sebastian Madgwick (2010)
 * 
 * CRITICAL: The LIS2MDL magnetometer is mounted on the BACK of the PCB.
 * Apply axis transform (invert X and Z) before calling Fusion_UpdateMag().
 * 
 * Usage:
 *   Fusion_Init(&config);
 *   // In IMU callback (~400 Hz):
 *   Fusion_UpdateIMU(dt, gx*DEG_TO_RAD, gy*DEG_TO_RAD, gz*DEG_TO_RAD, ax, ay, az);
 *   // In MAG callback (~100 Hz):
 *   Fusion_UpdateMag(-mx, my, -mz);  // Note: X and Z inverted!
 *   // Get output:
 *   Fusion_GetOutput(&output);
 */

#ifndef FUSION_H
#define FUSION_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Constants */
#define FUSION_DEG_TO_RAD  (0.01745329252f)  /* PI / 180 */
#define FUSION_RAD_TO_DEG  (57.29577951f)    /* 180 / PI */

/**
 * Magnetometer calibration parameters
 */
typedef struct {
    float offset[3];    /* Hard iron offsets [x, y, z] */
    float scale[3];     /* Soft iron scale factors [x, y, z] */
} FusionMagCal;

/**
 * IMU (Gyro + Accel) calibration parameters
 */
typedef struct {
    float gyro_bias[3];     /* Gyro bias [x, y, z] in deg/s */
    float accel_offset[3];  /* Accel offset [x, y, z] in g */
} FusionIMUCal;

/**
 * Fusion configuration
 */
typedef struct {
    float beta;                 /* Madgwick filter gain (0.01 - 0.5) */
    FusionMagCal mag_cal;       /* Magnetometer calibration */
    FusionIMUCal imu_cal;       /* IMU calibration */
    bool apply_mag_transform;   /* Apply X,Z axis inversion for LIS2MDL */
} FusionConfig;

/**
 * Fusion output structure
 */
typedef struct {
    float q[4];         /* Quaternion [w, x, y, z] */
    float euler[3];     /* Euler angles [roll, pitch, yaw] in radians */
    float heading;      /* Magnetic heading in degrees (0-360) */
} FusionOutput;

/**
 * Initialize the fusion filter
 * 
 * @param config Pointer to configuration structure
 */
void Fusion_Init(const FusionConfig* config);

/**
 * Reset filter to identity quaternion
 */
void Fusion_Reset(void);

/**
 * Set filter gain (beta)
 * 
 * @param beta New beta value (clamped to 0.001 - 1.0)
 */
void Fusion_SetBeta(float beta);

/**
 * Set magnetometer calibration
 * 
 * @param cal Pointer to calibration parameters
 */
void Fusion_SetMagCal(const FusionMagCal* cal);

/**
 * Set IMU calibration (gyro bias + accel offset)
 * 
 * @param cal Pointer to calibration parameters
 */
void Fusion_SetIMUCal(const FusionIMUCal* cal);

/**
 * Update with new magnetometer data
 * 
 * Called at MAG sample rate (~100 Hz).
 * IMPORTANT: Apply coordinate transform before calling!
 *   mx_device = -mx_raw;
 *   mz_device = -mz_raw;
 * 
 * @param mx Magnetometer X (gauss, device frame)
 * @param my Magnetometer Y (gauss, device frame)
 * @param mz Magnetometer Z (gauss, device frame)
 */
void Fusion_UpdateMag(float mx, float my, float mz);

/**
 * Update with IMU data (gyroscope + accelerometer)
 * 
 * Called at IMU sample rate (~400 Hz).
 * Uses stored magnetometer data for 9-DOF fusion.
 * 
 * @param dt Time delta in seconds
 * @param gx Gyroscope X (rad/s)
 * @param gy Gyroscope Y (rad/s)
 * @param gz Gyroscope Z (rad/s)
 * @param ax Accelerometer X (g)
 * @param ay Accelerometer Y (g)
 * @param az Accelerometer Z (g)
 */
void Fusion_UpdateIMU(float dt, 
                      float gx, float gy, float gz,
                      float ax, float ay, float az);

/**
 * Get current orientation output
 * 
 * @param output Pointer to output structure to fill
 */
void Fusion_GetOutput(FusionOutput* output);

/**
 * Get current quaternion directly
 * 
 * @param q Array of 4 floats to fill [w, x, y, z]
 */
void Fusion_GetQuaternion(float q[4]);

/**
 * Get magnetic heading in degrees
 * 
 * @return Heading (0-360 degrees)
 */
float Fusion_GetHeading(void);

#ifdef __cplusplus
}
#endif

#endif /* FUSION_H */
