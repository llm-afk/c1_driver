#include "motion_planner.h"
#include "util.h"
#include "motor_ctrl.h"

typedef struct {
    float sigma;

    float p0, p1;
    float v0, v1;
    float a_max, d_max;
    float vf;

    float d_pre;
    float Ld_pre;
    float Td_pre;
    float p0_pre;
    float v0_pre;

    float La, Lv, Ld;
    float Ta, Tv, Td;

    bool is_run;
    uint32_t tick;
} tMotionPlanner;

static tMotionPlanner mPlanner;

void MP_reset(void)
{
    mPlanner.is_run = false;
    mPlanner.tick = 0;
}

bool MP_is_run(void)
{
    return mPlanner.is_run;
}

int MP_trapezoid_pos_calculate(float p0, float p1, float v0, float v1, float v_max, float a_max, float d_max)
{
    tMotionPlanner mp[1];

    if(FLOAT_EQU(p0, p1)){
        return -1;
    }

    mp->sigma = __ARM_signbitf(p1-p0) ? -1.0f : +1.0f;
    mp->p0 = mp->sigma * p0;
    mp->p1 = mp->sigma * p1;
    mp->v0 = mp->sigma * v0;
    mp->v1 = mp->sigma * v1;

    if(mp->v1 < 0.0f){
        return -2;
    }

    if(mp->v0 < 0.0f){
        // pre deceleration to 0
        mp->d_pre = +d_max;
        mp->Ld_pre = -POW2(mp->v0) / (2.0f * d_max);
        mp->Td_pre = -mp->v0 / d_max;
        mp->p0_pre = mp->p0;
        mp->v0_pre = mp->v0;
        mp->p0 += mp->Ld_pre;
        mp->v0 = 0.0f;
    }else if(mp->v0 > v_max){
        // pre deceleration to v_max
        mp->d_pre = -d_max;
        mp->Ld_pre = (POW2(mp->v0) - POW2(v_max)) / (2.0f * d_max);
        mp->Td_pre = (mp->v0-v_max) / d_max;
        mp->p0_pre = mp->p0;
        mp->v0_pre = mp->v0;
        mp->p0 += mp->Ld_pre;
        mp->v0 = v_max;
    }else{
        mp->Td_pre = 0.0f;
    }

    float v0_sqr = POW2(mp->v0);
    float v1_sqr = POW2(mp->v1);

    float h = mp->p1 - mp->p0;
    float s = (2.0f*a_max*d_max*h + a_max*v1_sqr + d_max*v0_sqr) / (a_max + d_max);

    if(s < 0.0f){
        return -3;
    }

    float v = sqrtf(s);
    if(v > v_max){
        v = v_max;
    }

    if(v < mp->v0 || v < mp->v1){
        return -4;
    }

    float v_sqr = POW2(v);

    mp->La = (v_sqr - v0_sqr) / (2.0f * a_max);
    mp->Ld = (v_sqr - v1_sqr) / (2.0f * d_max);
    mp->Lv = h - mp->La - mp->Ld;
    mp->Ta = (v - mp->v0) / a_max;
    mp->Td = (v - mp->v1) / d_max;
    mp->Tv = mp->Lv / v;
    mp->vf = v;
    mp->a_max = a_max;
    mp->d_max = d_max;

    mp->tick = 0;
    mp->is_run = true;

    // copy
    mPlanner = mp[0];

    return 0;
}

void MP_trapezoid_pos_execute(float *pos_out, float *vel_out, float *acc_out)
{
    if(!mPlanner.is_run){
        return;
    }

    float time = (mPlanner.tick++) * SERVO_CTRL_PERIOD;

    float pos;
    float vel;
    float acc;

    if(time < mPlanner.Td_pre){
        float t = time;
        pos = mPlanner.p0_pre + mPlanner.v0_pre * t + 0.5f * mPlanner.d_pre * POW2(t);
        vel = mPlanner.v0_pre + mPlanner.d_pre * t;
        acc = mPlanner.d_pre;
    }else if(time < mPlanner.Td_pre + mPlanner.Ta){
        float t = time - mPlanner.Td_pre;
        pos = mPlanner.p0 + mPlanner.v0 * t + 0.5f * mPlanner.a_max * POW2(t);
        vel = mPlanner.v0 + mPlanner.a_max * t;
        acc = mPlanner.a_max;
    }else if(time < mPlanner.Td_pre + mPlanner.Ta + mPlanner.Tv){
        float t = time - mPlanner.Td_pre - mPlanner.Ta;
        pos = mPlanner.p0 + mPlanner.La + mPlanner.vf * t;
        vel = mPlanner.vf;
        acc = 0.0f;
    }else if(time < mPlanner.Td_pre + mPlanner.Ta + mPlanner.Tv + mPlanner.Td){
        float t = time - mPlanner.Td_pre - mPlanner.Ta - mPlanner.Tv;
        pos = mPlanner.p0 + mPlanner.La + mPlanner.Lv + mPlanner.vf * t - 0.5f * mPlanner.d_max * POW2(t);
        vel = mPlanner.vf - mPlanner.d_max * t;
        acc = -mPlanner.d_max;
    }else{
        // Motion complete
        pos = mPlanner.p1;
        vel = mPlanner.v1;
        acc = 0.0f;
        mPlanner.is_run = false;
    }

    *pos_out = mPlanner.sigma * pos;
    *vel_out = mPlanner.sigma * vel;
    *acc_out = mPlanner.sigma * acc;
}

void MP_trapezoid_vel_calculate(float v0, float v1, float a_max, float d_max)
{
    float Td, Ta;
    float d, a;
    float vf;

    if(v1 > v0){
        if(v0 >= 0){
            d = 0;
            Td = 0;
            a = a_max;
            Ta = (v1-v0) / a;
            vf = v0;
        }else{
            if(v1 <= 0){
                d = d_max;
                Td = (v1-v0) / d;
                a = 0;
                Ta = 0;
                vf = v1;
            }else{
                d = d_max;
                Td = (0-v0) / d;
                a = a_max;
                Ta = (v1-0) / a;
                vf = 0;
            }
        }
    }else if(v1 < v0){
        if(v1 >= 0){
            d = -d_max;
            Td = (v1-v0) / d;
            a = 0;
            Ta = 0;
            vf = v1;
        }else{
            if(v0 <= 0){
                d = 0;
                Td = 0;
                a = -a_max;
                Ta = (v1-v0) / a;
                vf = v0;
            }else{
                d = -d_max;
                Td = (0-v0) / d;
                a = -a_max;
                Ta = (v1-0) / a;
                vf = 0;
            }
        }
    }else{
        d = 0;
        Td = 0;
        a = 0;
        Ta = 0;
    }

    mPlanner.tick = 0;
    mPlanner.is_run = true;

    // copy
    mPlanner.d_max = d;
    mPlanner.Td = Td;
    mPlanner.a_max = a;
    mPlanner.Ta = Ta;
    mPlanner.vf = vf;
    mPlanner.v0 = v0;
    mPlanner.v1 = v1;
}

void MP_trapezoid_vel_execute(float *vel_out, float *acc_out)
{
    if(!mPlanner.is_run){
        return;
    }

    float time = (mPlanner.tick++) * SERVO_CTRL_PERIOD;

    float vel;
    float acc;

    if(time < mPlanner.Td){
        float t = time;
        vel = mPlanner.v0 + mPlanner.d_max * t;
        acc = mPlanner.d_max;
    }else if(time < mPlanner.Td + mPlanner.Ta){
        float t = time - mPlanner.Td;
        vel = mPlanner.vf + mPlanner.a_max * t;
        acc = mPlanner.a_max;
    }else{
        // complete
        vel = mPlanner.v1;
        acc = 0.0f;
        mPlanner.is_run = false;
    }

    *vel_out = vel;
    *acc_out = acc;
}

void MP_trapezoid_torque_calculate(float t0, float t1, float slope)
{
    float a;
    float Ta;

    if(t1 > t0){
        a = +slope;
        Ta = (t1-t0)/a;
    }else if(t1 < t0){
        a = -slope;
        Ta = (t1-t0)/a;
    }else{
        a = 0;
        Ta = 0;
    }

    mPlanner.tick = 0;
    mPlanner.is_run = true;

    // copy
    mPlanner.a_max = a;
    mPlanner.Ta = Ta;
    mPlanner.v0 = t0;
    mPlanner.v1 = t1;
}

void MP_trapezoid_torque_execute(float *torque_out)
{
    if(!mPlanner.is_run){
        return;
    }

    float time = (mPlanner.tick++) * SERVO_CTRL_PERIOD;

    float torque;

    if(time < mPlanner.Ta){
        float t = time;
        torque = mPlanner.v0 + mPlanner.a_max * t;
    }else{
        // complete
        torque = mPlanner.v1;
        mPlanner.is_run = false;
    }

    *torque_out = torque;
}
