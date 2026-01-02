/**
 * FlySight 2 Sensor Fusion - Madgwick AHRS Algorithm
 * 
 * C implementation for STM32 firmware integration.
 * See fusion.h for documentation.
 */

#include "fusion.h"
#include <math.h>

/* Static state - no dynamic allocation */
static struct {
    /* Quaternion state [w, x, y, z] */
    float q[4];
    
    /* Configuration */
    float beta;
    FusionMagCal mag_cal;
    FusionIMUCal imu_cal;
    bool apply_mag_transform;
    
    /* Last magnetometer values (for async update) */
    float last_mag[3];
    bool mag_valid;
} fusion_state;

/* Fast inverse square root (Quake III algorithm) */
/* Can be replaced with 1.0f/sqrtf() if FPU is fast enough */
static float inv_sqrt(float x)
{
    /* Use standard library for accuracy - FPU handles this well */
    return 1.0f / sqrtf(x);
}

void Fusion_Init(const FusionConfig* config)
{
    /* Initialize quaternion to identity (no rotation) */
    fusion_state.q[0] = 1.0f;  /* w */
    fusion_state.q[1] = 0.0f;  /* x */
    fusion_state.q[2] = 0.0f;  /* y */
    fusion_state.q[3] = 0.0f;  /* z */
    
    /* Store configuration */
    if (config != NULL) {
        fusion_state.beta = config->beta;
        fusion_state.mag_cal = config->mag_cal;
        fusion_state.imu_cal = config->imu_cal;
        fusion_state.apply_mag_transform = config->apply_mag_transform;
    } else {
        /* Defaults - calibrated for FlySight S.N. 2-00176 */
        fusion_state.beta = 0.1f;
        
        /* Magnetometer hard iron offsets */
        fusion_state.mag_cal.offset[0] = 0.3465f;   /* X hard iron offset */
        fusion_state.mag_cal.offset[1] = -0.0545f;  /* Y hard iron offset */
        fusion_state.mag_cal.offset[2] = 0.5380f;   /* Z hard iron offset */
        fusion_state.mag_cal.scale[0] = 1.0f;
        fusion_state.mag_cal.scale[1] = 1.0f;
        fusion_state.mag_cal.scale[2] = 1.0f;
        
        /* IMU calibration - gyro bias (deg/s) */
        fusion_state.imu_cal.gyro_bias[0] = -0.2203f;  /* X gyro bias */
        fusion_state.imu_cal.gyro_bias[1] = -0.1055f;  /* Y gyro bias */
        fusion_state.imu_cal.gyro_bias[2] = -0.2263f;  /* Z gyro bias */
        
        /* IMU calibration - accel offset (g) */
        fusion_state.imu_cal.accel_offset[0] = 0.0175f;  /* X accel offset */
        fusion_state.imu_cal.accel_offset[1] = 0.0266f;  /* Y accel offset */
        fusion_state.imu_cal.accel_offset[2] = 0.0140f;  /* Z accel offset */
        
        fusion_state.apply_mag_transform = true;
    }
    
    /* Invalidate magnetometer data */
    fusion_state.last_mag[0] = 0.0f;
    fusion_state.last_mag[1] = 0.0f;
    fusion_state.last_mag[2] = 0.0f;
    fusion_state.mag_valid = false;
}

void Fusion_Reset(void)
{
    fusion_state.q[0] = 1.0f;
    fusion_state.q[1] = 0.0f;
    fusion_state.q[2] = 0.0f;
    fusion_state.q[3] = 0.0f;
    fusion_state.mag_valid = false;
}

void Fusion_SetBeta(float beta)
{
    if (beta < 0.001f) beta = 0.001f;
    if (beta > 1.0f) beta = 1.0f;
    fusion_state.beta = beta;
}

void Fusion_SetMagCal(const FusionMagCal* cal)
{
    if (cal != NULL) {
        fusion_state.mag_cal = *cal;
    }
}

void Fusion_SetIMUCal(const FusionIMUCal* cal)
{
    if (cal != NULL) {
        fusion_state.imu_cal = *cal;
    }
}

void Fusion_UpdateMag(float mx, float my, float mz)
{
    /* Apply coordinate transform for LIS2MDL on back of PCB */
    if (fusion_state.apply_mag_transform) {
        mx = -mx;  /* X axis inverted */
        /* my stays same */
        mz = -mz;  /* Z axis inverted */
    }
    
    /* Apply hard iron calibration (offset removal) */
    mx = (mx - fusion_state.mag_cal.offset[0]) * fusion_state.mag_cal.scale[0];
    my = (my - fusion_state.mag_cal.offset[1]) * fusion_state.mag_cal.scale[1];
    mz = (mz - fusion_state.mag_cal.offset[2]) * fusion_state.mag_cal.scale[2];
    
    /* Store for use in IMU update */
    fusion_state.last_mag[0] = mx;
    fusion_state.last_mag[1] = my;
    fusion_state.last_mag[2] = mz;
    fusion_state.mag_valid = true;
}

/**
 * 9-DOF Madgwick AHRS update (gyro + accel + mag)
 */
static void madgwick_ahrs_update_9dof(
    float dt,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float mx, float my, float mz)
{
    float q0 = fusion_state.q[0];
    float q1 = fusion_state.q[1];
    float q2 = fusion_state.q[2];
    float q3 = fusion_state.q[3];
    
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx;
    float _2bx, _2bz, _4bx, _4bz;
    float _2q0, _2q1, _2q2, _2q3;
    float _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;
    
    float beta = fusion_state.beta;
    
    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    /* Compute feedback only if accelerometer measurement valid */
    float accelMag = ax * ax + ay * ay + az * az;
    if (accelMag > 0.01f) {
        /* Normalise accelerometer measurement */
        recipNorm = inv_sqrt(accelMag);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        /* Normalise magnetometer measurement */
        float magMag = mx * mx + my * my + mz * mz;
        if (magMag > 0.01f) {
            recipNorm = inv_sqrt(magMag);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;
            
            /* Auxiliary variables */
            _2q0mx = 2.0f * q0 * mx;
            _2q0my = 2.0f * q0 * my;
            _2q0mz = 2.0f * q0 * mz;
            _2q1mx = 2.0f * q1 * mx;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q0q2 = 2.0f * q0 * q2;
            _2q2q3 = 2.0f * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;
            
            /* Reference direction of Earth's magnetic field */
            hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 +
                 _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
            hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 -
                 my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 -
                   mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;
            
            /* Gradient decent algorithm corrective step */
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
                 _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
                 _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                 (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                 _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
                 _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
                 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
                 _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                 (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                 (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
                 _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
                 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) +
                 (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                 (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                 (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
                 _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
                 (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
                 (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
                 _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
            
            /* Normalise step magnitude */
            recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;
            
            /* Apply feedback step */
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }
    }
    
    /* Integrate rate of change of quaternion */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    
    /* Normalise quaternion */
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    fusion_state.q[0] = q0 * recipNorm;
    fusion_state.q[1] = q1 * recipNorm;
    fusion_state.q[2] = q2 * recipNorm;
    fusion_state.q[3] = q3 * recipNorm;
}

/**
 * 6-DOF IMU-only update (gyro + accel, no magnetometer)
 */
static void madgwick_ahrs_update_6dof(
    float dt,
    float gx, float gy, float gz,
    float ax, float ay, float az)
{
    float q0 = fusion_state.q[0];
    float q1 = fusion_state.q[1];
    float q2 = fusion_state.q[2];
    float q3 = fusion_state.q[3];
    
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3;
    float _4q0, _4q1, _4q2;
    float _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;
    
    float beta = fusion_state.beta;
    
    /* Rate of change of quaternion from gyroscope */
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    /* Compute feedback only if accelerometer measurement valid */
    float accelMag = ax * ax + ay * ay + az * az;
    if (accelMag > 0.01f) {
        /* Normalise accelerometer measurement */
        recipNorm = inv_sqrt(accelMag);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        /* Auxiliary variables */
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        
        /* Gradient descent algorithm corrective step */
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        /* Normalise step magnitude */
        recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        /* Apply feedback step */
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    
    /* Integrate rate of change of quaternion */
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    
    /* Normalise quaternion */
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    fusion_state.q[0] = q0 * recipNorm;
    fusion_state.q[1] = q1 * recipNorm;
    fusion_state.q[2] = q2 * recipNorm;
    fusion_state.q[3] = q3 * recipNorm;
}

void Fusion_UpdateIMU(float dt,
                      float gx, float gy, float gz,
                      float ax, float ay, float az)
{
    /* Apply IMU calibration - subtract gyro bias */
    gx -= fusion_state.imu_cal.gyro_bias[0];
    gy -= fusion_state.imu_cal.gyro_bias[1];
    gz -= fusion_state.imu_cal.gyro_bias[2];
    
    /* Apply IMU calibration - subtract accel offset */
    ax -= fusion_state.imu_cal.accel_offset[0];
    ay -= fusion_state.imu_cal.accel_offset[1];
    az -= fusion_state.imu_cal.accel_offset[2];
    
    if (fusion_state.mag_valid) {
        madgwick_ahrs_update_9dof(dt, gx, gy, gz, ax, ay, az,
                                  fusion_state.last_mag[0],
                                  fusion_state.last_mag[1],
                                  fusion_state.last_mag[2]);
    } else {
        madgwick_ahrs_update_6dof(dt, gx, gy, gz, ax, ay, az);
    }
}

void Fusion_GetOutput(FusionOutput* output)
{
    if (output == NULL) return;
    
    /* Copy quaternion */
    output->q[0] = fusion_state.q[0];
    output->q[1] = fusion_state.q[1];
    output->q[2] = fusion_state.q[2];
    output->q[3] = fusion_state.q[3];
    
    /* Compute Euler angles */
    float q0 = fusion_state.q[0];
    float q1 = fusion_state.q[1];
    float q2 = fusion_state.q[2];
    float q3 = fusion_state.q[3];
    
    /* Roll (rotation around X axis) */
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    output->euler[0] = atan2f(sinr_cosp, cosr_cosp);
    
    /* Pitch (rotation around Y axis) */
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        output->euler[1] = copysignf(3.14159265f / 2.0f, sinp);
    } else {
        output->euler[1] = asinf(sinp);
    }
    
    /* Yaw (rotation around Z axis) */
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    output->euler[2] = atan2f(siny_cosp, cosy_cosp);
    
    /* Compute heading (0-360 degrees) */
    float heading = output->euler[2] * FUSION_RAD_TO_DEG;
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    output->heading = heading;
}

void Fusion_GetQuaternion(float q[4])
{
    if (q == NULL) return;
    
    q[0] = fusion_state.q[0];
    q[1] = fusion_state.q[1];
    q[2] = fusion_state.q[2];
    q[3] = fusion_state.q[3];
}

float Fusion_GetHeading(void)
{
    float q0 = fusion_state.q[0];
    float q1 = fusion_state.q[1];
    float q2 = fusion_state.q[2];
    float q3 = fusion_state.q[3];
    
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float yaw = atan2f(siny_cosp, cosy_cosp);
    
    float heading = yaw * FUSION_RAD_TO_DEG;
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    
    return heading;
}
