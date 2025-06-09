#include "pid.h"

void PID_init(tPID *pid, float kp, float ki, float kd, float dt, float alpha)
{
    PID_setting(pid, kp, ki, kd, dt, alpha);
    PID_reset(pid);
}

void PID_setting(tPID *pid, float kp, float ki, float kd, float dt, float alpha)
{
    pid->kp = kp;
    pid->ki = ki * dt;
    pid->kd = kd / dt;
    pid->alpha = alpha;
}

void PID_reset(tPID *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_d_error = 0.0f;
}

inline float PI_compute_parallel(tPID *pid, float error, float intMin, float intMax, float outMin, float outMax, float feed_forward)
{
    float p_term   = pid->kp * error;
    pid->integral += pid->ki * (0.5f * (error + pid->last_error));
    
#if DYNAMIC_INT_CALMP
    if(pid->integral > (intMax - p_term)){
        pid->integral = (intMax - p_term);
    }else if(pid->integral < (intMin - p_term)){
        pid->integral = (intMin - p_term);
    }
#else
    if(pid->integral > intMax){
        pid->integral = intMax;
    }else if(pid->integral < intMin){
        pid->integral = intMin;
    }
#endif

    float result = p_term + pid->integral + feed_forward;

    if(result > outMax){
        result = outMax;
    }else if(result < outMin){
        result = outMin;
    }
    
    pid->last_error = error;
    
    return result;
}

inline float PI_compute_serial(tPID *pid, float error, float intMin, float intMax, float outMin, float outMax, float feed_forward)
{
    float p_term   = pid->kp * error;
    pid->integral += pid->kp * pid->ki * (0.5f * (error + pid->last_error));
    
#if DYNAMIC_INT_CALMP
    if(pid->integral > (intMax - p_term)){
        pid->integral = (intMax - p_term);
    }else if(pid->integral < (intMin - p_term)){
        pid->integral = (intMin - p_term);
    }
#else
    if(pid->integral > intMax){
        pid->integral = intMax;
    }else if(pid->integral < intMin){
        pid->integral = intMin;
    }
#endif

    float result = p_term + pid->integral + feed_forward;

    if(result > outMax){
        result = outMax;
    }else if(result < outMin){
        result = outMin;
    }
    
    pid->last_error = error;
    
    return result;
}

inline float PID_compute_parallel(tPID *pid, float error, float intMin, float intMax, float outMin, float outMax, float feed_forward)
{
    float d_error = error - pid->last_error;
    
    float p_term   = pid->kp * error;
    pid->integral += pid->ki * (0.5f * (error + pid->last_error));
    float d_term   = pid->kd * (pid->alpha * d_error + (1.0f - pid->alpha) * pid->last_d_error);
    
#if DYNAMIC_INT_CALMP
    if(pid->integral > (intMax - p_term)){
        pid->integral = (intMax - p_term);
    }else if(pid->integral < (intMin - p_term)){
        pid->integral = (intMin - p_term);
    }
#else
    if(pid->integral > intMax){
        pid->integral = intMax;
    }else if(pid->integral < intMin){
        pid->integral = intMin;
    }
#endif

    float result = p_term + pid->integral + d_term + feed_forward;

    if(result > outMax){
        result = outMax;
    }else if(result < outMin){
        result = outMin;
    }
    
    pid->last_error = error;
    pid->last_d_error = d_error;
    
    return result;
}
