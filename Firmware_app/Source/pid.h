#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#include <stdbool.h>

#define DYNAMIC_INT_CALMP   1

typedef struct {
    float kp;
    float ki;
    float kd;
    float alpha;

    float integral;
	float last_error;
    float last_d_error;
} tPID;

void PID_init(tPID *pid, float kp, float ki, float kd, float dt, float alpha);
void PID_setting(tPID *pid, float kp, float ki, float kd, float dt, float alpha);
void PID_reset(tPID *pid);
extern inline float PI_compute_parallel(tPID *pid, float error, float intMin, float intMax, float outMin, float outMax, float feed_forward);
extern inline float PI_compute_serial(tPID *pid, float error, float intMin, float intMax, float outMin, float outMax, float feed_forward);
extern inline float PID_compute_parallel(tPID *pid, float error, float intMin, float intMax, float outMin, float outMax, float feed_forward);

#endif
