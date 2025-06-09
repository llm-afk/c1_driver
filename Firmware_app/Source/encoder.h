#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "motor_ctrl.h"

#define ENCODER_CALIB_CURRENT   2.0f
#define ENCODER_PLL_BANDWIDTH   2000.0f     // rad/s
#define ENCODER_PLL_DT          CURRENT_CTRL_PERIOD

#define ENCODER_BITS            (14)

#define ENCODER_CPR             (1 << ENCODER_BITS)
#define ENCODER_CPR_F           ((float)ENCODER_CPR)
#define ENCODER_CPR_DIV         (ENCODER_CPR >> 1)

#define CALIB_LUT_BITS          (7)
#define ENCODER_OFFSET_LUT_NUM  (1<<CALIB_LUT_BITS)
#define SAMPLES_PER_PPAIR       128U

#define MAX_PP                  30

typedef struct {
    float phase_set;
    int32_t calib_step;
    int32_t loop_count;
    float next_sample_time;
    int32_t sample_count;
    int32_t errors[MAX_PP * SAMPLES_PER_PPAIR];
} tEncoderCalib;

typedef struct {
    int32_t calib_valid;
    int32_t encoder_ex_offset;
    int32_t encoder_reverse;
    int32_t encoder_offset;
    int32_t encoder_offset_lut[ENCODER_OFFSET_LUT_NUM];
    uint32_t crc;
} tEncoderConfig;

typedef struct {
    uint8_t need_init;
    tEncoderCalib Calib;
    tEncoderConfig Config;
    
    int raw;
    int count_in_cpr;
    int count_in_cpr_prev;
    
    int64_t shadow_count;   // count
    
    float vel;              // count/s
    float phase;
    float phase_vel;

    // pll use
    float pll_pos;
    float pll_vel;
    
    float pll_kp;
    float pll_ki;
    float snap_threshold;
} tEncoder;

extern tEncoder Encoder;

void ENCODER_init(void);

void ENCODER_calib_start(void);
void ENCODER_calib_end(void);
void ENCODER_calib_loop(float dt);

int32_t ENCODER_EX_read(void);
int32_t ENCODER_EX_read_rectified(void);
int32_t ENCODER_read(void);
void ENCODER_loop(void);

#endif
