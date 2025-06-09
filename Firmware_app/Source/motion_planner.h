#ifndef __MOTION_PLANNER_H__
#define __MOTION_PLANNER_H__

#include <stdint.h>
#include <stdbool.h>

void MP_reset(void);
bool MP_is_run(void);

int  MP_trapezoid_pos_calculate(float p0, float p1, float v0, float v1, float v_max, float a_max, float d_max);
void MP_trapezoid_pos_execute(float *pos_out, float *vel_out, float *acc_out);

void MP_trapezoid_vel_calculate(float v0, float v1, float a_max, float d_max);
void MP_trapezoid_vel_execute(float *vel_out, float *acc_out);

void MP_trapezoid_torque_calculate(float t0, float t1, float slope);
void MP_trapezoid_torque_execute(float *torque_out);

#endif
