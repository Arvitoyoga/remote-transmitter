#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct {
    float twoKp, twoKi;
    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;
    float samplePeriod;

    float euler_deadzone;
    float last_roll, last_pitch, last_yaw;
} filter_t;

void filter_init(filter_t *f, float sample_hz);
void filter_set_gains(filter_t *f, float Kp, float Ki);
void filter_update_raw(filter_t *f,
                       float ax_raw, float ay_raw, float az_raw,
                       float gx_raw, float gy_raw, float gz_raw);
void filter_get_euler(filter_t *f, float *roll_deg, float *pitch_deg, float *yaw_deg);
void filter_set_deadzone(filter_t *f, float deg);

#ifdef __cplusplus
}
#endif

#endif
