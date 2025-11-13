#include "filter.h"
#include <math.h>
#include <stdlib.h>

#define DEG2RAD(x) ((x) * (3.14159265358979f / 180.0f))
#define RAD2DEG(x) ((x) * (180.0f / 3.14159265358979f))

#define GYRO_SCALE   ((1.0f / 65.5f) * DEG2RAD(1.0f)) // raw -> rad/s
#define ACCEL_SCALE  (1.0f / 8192.0f)                 // raw -> g

// ====== Internal ======
static inline float inv_sqrt(float x) {
    if (x <= 0) return 0;
    return 1.0f / sqrtf(x);
}

// ====== Inisialisasi ======
void filter_init(filter_t *f, float sample_hz) {
    if (!f) return;
    if (sample_hz <= 0) sample_hz = 500.0f;
    f->samplePeriod = 1.0f / sample_hz;

    f->twoKp = 2.0f;
    f->twoKi = 0.0f;
    f->q0 = 1; f->q1 = f->q2 = f->q3 = 0;
    f->integralFBx = f->integralFBy = f->integralFBz = 0;
    f->euler_deadzone = 0.5f;
    f->last_roll = f->last_pitch = f->last_yaw = 0;
}

// ====== Gain ======
void filter_set_gains(filter_t *f, float Kp, float Ki) {
    if (!f) return;
    f->twoKp = 2.0f * Kp;
    f->twoKi = 2.0f * Ki;
}

// ====== Deadzone ======
void filter_set_deadzone(filter_t *f, float deg) {
    if (!f) return;
    if (deg < 0) deg = 0;
    f->euler_deadzone = deg;
}

// ====== Mahony Update dari RAW ======
void filter_update_raw(filter_t *f,
                       float ax_raw, float ay_raw, float az_raw,
                       float gx_raw, float gy_raw, float gz_raw)
{
    if (!f) return;

    float gx = gx_raw * GYRO_SCALE;
    float gy = gy_raw * GYRO_SCALE;
    float gz = gz_raw * GYRO_SCALE;
    float ax = ax_raw * ACCEL_SCALE;
    float ay = ay_raw * ACCEL_SCALE;
    float az = az_raw * ACCEL_SCALE;

    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm <= 0.0f) return;
    ax /= norm; ay /= norm; az /= norm;

    float vx = 2.0f * (f->q1 * f->q3 - f->q0 * f->q2);
    float vy = 2.0f * (f->q0 * f->q1 + f->q2 * f->q3);
    float vz = f->q0 * f->q0 - f->q1 * f->q1 - f->q2 * f->q2 + f->q3 * f->q3;

    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    if (f->twoKi > 0.0f) {
        f->integralFBx += f->twoKi * ex * f->samplePeriod;
        f->integralFBy += f->twoKi * ey * f->samplePeriod;
        f->integralFBz += f->twoKi * ez * f->samplePeriod;
        gx += f->integralFBx;
        gy += f->integralFBy;
        gz += f->integralFBz;
    } else {
        f->integralFBx = f->integralFBy = f->integralFBz = 0.0f;
    }

    gx += f->twoKp * ex;
    gy += f->twoKp * ey;
    gz += f->twoKp * ez;

    gx *= 0.5f * f->samplePeriod;
    gy *= 0.5f * f->samplePeriod;
    gz *= 0.5f * f->samplePeriod;

    float qa = f->q0, qb = f->q1, qc = f->q2;
    f->q0 += (-qb * gx - qc * gy - f->q3 * gz);
    f->q1 += (qa * gx + qc * gz - f->q3 * gy);
    f->q2 += (qa * gy - qb * gz + f->q3 * gx);
    f->q3 += (qa * gz + qb * gy - qc * gx);

    float invNorm = inv_sqrt(f->q0*f->q0 + f->q1*f->q1 + f->q2*f->q2 + f->q3*f->q3);
    f->q0 *= invNorm; f->q1 *= invNorm; f->q2 *= invNorm; f->q3 *= invNorm;
}

// ====== Ambil Euler dengan Deadzone ======
void filter_get_euler(filter_t *f, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (!f) return;

    float roll = atan2f(2.0f*(f->q0*f->q1 + f->q2*f->q3),
                        1 - 2.0f*(f->q1*f->q1 + f->q2*f->q2));
    float pitch = asinf(2.0f*(f->q0*f->q2 - f->q3*f->q1));
    float yaw = atan2f(2.0f*(f->q0*f->q3 + f->q1*f->q2),
                       1 - 2.0f*(f->q2*f->q2 + f->q3*f->q3));

    roll *= RAD2DEG(1.0f);
    pitch *= RAD2DEG(1.0f);
    yaw *= RAD2DEG(1.0f);

    if (fabsf(roll - f->last_roll) < f->euler_deadzone) roll = f->last_roll;
    else f->last_roll = roll;

    if (fabsf(pitch - f->last_pitch) < f->euler_deadzone) pitch = f->last_pitch;
    else f->last_pitch = pitch;

    if (fabsf(yaw - f->last_yaw) < f->euler_deadzone) yaw = f->last_yaw;
    else f->last_yaw = yaw;

    if (roll_deg)  *roll_deg = roll;
    if (pitch_deg) *pitch_deg = pitch;
    if (yaw_deg)   *yaw_deg = yaw;
}
