#ifndef __SOC_H__
#define __SOC_H__

#include "main.h"
#include <math.h>

typedef struct {
    uint32_t id:24;
    uint32_t dlc:8;
    uint8_t  data[64];
} CanFrame;

static const uint8_t dlc_to_len_table[] = {0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};

#define PWM_FREQUENCY          20000
#define CURRENT_MEASURE_HZ     PWM_FREQUENCY
#define CURRENT_MEASURE_PERIOD (float) (1.0f / (float) CURRENT_MEASURE_HZ)
    
#define TIMER0_CLK_MHz         120
#define PWM_PERIOD_CYCLES      (uint16_t)((TIMER0_CLK_MHz * (uint32_t) 1000000u / ((uint32_t) (PWM_FREQUENCY))) & 0xFFFE)
#define PWM_PERIOD_CYCLES_DIV  (uint16_t)(PWM_PERIOD_CYCLES / 2U)

#define SHUNT_RESISTENCE       (0.002f)
#define V_SCALE                ((float) (31.0f * 3.3f / 4095.0f))
#include "od.h"
#define I_SCALE                ((float) ((3.3f / 4095.0f) / SHUNT_RESISTENCE / g_i_scale))
    
#define MAX_MODULATION         0.95f

#define READ_IPHASE_A_ADC()    ((uint16_t) (ADC_IDATA0(ADC0)))
#define READ_IPHASE_B_ADC()    ((uint16_t) (ADC_IDATA1(ADC0)))
#define READ_IPHASE_C_ADC()    ((uint16_t) (ADC_IDATA0(ADC1)))

extern uint16_t adc_buff[3];
extern int16_t  phase_a_adc_offset;
extern int16_t  phase_b_adc_offset;
extern int16_t  phase_c_adc_offset;

static inline uint32_t get_timestamp(void) { return (uint32_t)SysTick->VAL; }
static inline uint32_t get_us_since(uint32_t timestamp) { int32_t diff=timestamp-(int32_t)SysTick->VAL; if(diff<0){diff += 0xFFFFFFUL;} return (diff / 120); }

static inline float SOC_read_vbus(void)
{
    return (float) (adc_buff[0]) * V_SCALE;
}

static inline float calculate_temperature(float R, float R0, float B, float T0)
{
    float T_kelvin = 1.0f / (1.0f / T0 + (1.0f / B) * logf(R / R0));
    float T_celsius = T_kelvin - 273.15f;
    return T_celsius;
}

static inline float SOC_read_drv_temp(void)
{
    float v = 3.3f * adc_buff[2] / 4095.0f;
    float r = v * 10000 / (3.3f - v);
    return calculate_temperature(r, 10000, 3950, 298.15);
}

static inline float SOC_read_motor_temp(void)
{
    float v = 3.3f * adc_buff[1] / 4095.0f;
    float r = v * 10000 / (3.3f - v);
    return calculate_temperature(r, 10000, 3950, 298.15);
}

static inline float SOC_read_iphase_a(void)
{
    return (float) (READ_IPHASE_A_ADC() - phase_a_adc_offset) * I_SCALE;
}

static inline float SOC_read_iphase_b(void)
{
    return (float) (READ_IPHASE_B_ADC() - phase_b_adc_offset) * I_SCALE;
}

static inline float SOC_read_iphase_c(void)
{
    return (float) (READ_IPHASE_C_ADC() - phase_c_adc_offset) * I_SCALE;
}

static inline void set_a_duty(uint32_t duty)
{
    TIMER_CH2CV(TIMER0) = duty;
}
static inline void set_b_duty(uint32_t duty)
{
    TIMER_CH1CV(TIMER0) = duty;
}
static inline void set_c_duty(uint32_t duty)
{
    TIMER_CH0CV(TIMER0) = duty;
}

void SOC_init(void);
void SOC_pwm_enable(void);
void SOC_pwm_disable(void);
int  SOC_calibration_offset(void);

int SOC_can_init(int can_idx, int data_idx);
int SOC_can_is_tx_busy(void);
int SOC_can_transmit(CanFrame *frame);
int SOC_can_transmit_block(CanFrame *frame);


#endif
