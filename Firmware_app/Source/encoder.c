#include "encoder.h"
#include "com_can.h"
#include "util.h"
#include <string.h>
#include "motor_ctrl.h"
#include "flash_interface.h"


tEncoder Encoder = {
    .need_init = 20,
    .shadow_count = 0,
    .pll_pos = 0,
    .pll_vel = 0,
    .pll_kp = 2.0f * ENCODER_PLL_BANDWIDTH,
    .pll_ki = 0.25f * POW2(2.0f * ENCODER_PLL_BANDWIDTH),
    .snap_threshold = 0.5f * ENCODER_PLL_DT * ( 0.25f * POW2(2.0f * ENCODER_PLL_BANDWIDTH) ),
};

void ENCODER_init(void)
{
    memcpy(&Encoder.Config, (uint8_t*)(FLASH_BASE + ENCODER_CALIB_PAGE*FLASH_PAGE_SIZE), sizeof(tEncoderConfig));
    uint32_t crc = crc32((uint8_t*)&Encoder.Config, sizeof(tEncoderConfig)-4);

    if(crc != Encoder.Config.crc){
        
            /* Both locations invalid — reset to defaults */
            Encoder.Config.calib_valid = false;
            Encoder.Config.encoder_ex_offset = 0;
            Encoder.Config.encoder_offset = 0;
            for(int i=0; i<ENCODER_OFFSET_LUT_NUM; i++){
                Encoder.Config.encoder_offset_lut[i] = 0;
            }
        }
}

static void save_encoder_config(void)
{
    // Erase 2 pages because 1040 bytes exceeds the 1024 byte FLASH_PAGE_SIZE
    FI_flash_erase_page(ENCODER_CALIB_PAGE);
    FI_flash_erase_page(ENCODER_CALIB_PAGE + 1);
    
    // Program
    Encoder.Config.crc = crc32((uint8_t*)&Encoder.Config, sizeof(tEncoderConfig)-4);
    FI_flash_write((uint8_t*)(FLASH_BASE + ENCODER_CALIB_PAGE * FLASH_PAGE_SIZE), (uint8_t*)&Encoder.Config, sizeof(tEncoderConfig));
}

void ENCODER_calib_start(void)
{
    // 每次重新开始校准时，清空上一次可能的磁铁丢失报错
    if(ERROR_IS_SET(ERR_ENC_MISSING)) 
    {
        ERROR_CLR(ERR_ENC_MISSING);
    }

    // Reset calib result
    Encoder.Config.calib_valid = false;
    Encoder.Config.encoder_reverse = 0;
    Encoder.Config.encoder_offset = 0;
    for(int i=0; i<ENCODER_OFFSET_LUT_NUM; i++){
        Encoder.Config.encoder_offset_lut[i] = 0;
    }
    
    Encoder.Calib.calib_step = 0;
}

void ENCODER_calib_end(void)
{
    // 故意留空：如果校准失败报错了，让红灯一直挂着，直到下次重新校准
}

void ENCODER_calib_loop(float dt)
{
    static int count_raw_start;
    static int32_t in_max = 0;
    static int32_t in_min = 16383;
    static int32_t ex_max = 0;
    static int32_t ex_min = 16383;
    
    const float time = 10.24f;
    const float phase_delta = MOTOR_POLE_PAIRS * M_2PI * dt / time;
    const float sample_time_delta = time / (float)(MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR);

    const float t = (float)Encoder.Calib.loop_count * dt;
    const float voltage = g_encoder_calib_current * MOTOR_PHASE_R;
    
    switch(Encoder.Calib.calib_step){
        case 0: // Init
            Encoder.Calib.phase_set = 0;
            Encoder.Calib.loop_count = 0;
            in_max = 0;
            in_min = 16383;
            ex_max = 0;
            ex_min = 16383;
            Encoder.Calib.calib_step ++;
            break;

        case 1: // Lock
            MC_modulate((voltage * t / 2.0f), 0, Encoder.Calib.phase_set);
            if (t >= 2.0f){
                count_raw_start = Encoder.raw;
                Encoder.Calib.calib_step ++;
            }
            break;
            
        case 2: // cw find direction
            Encoder.Calib.phase_set += phase_delta;
            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
            if(Encoder.Calib.phase_set >= 4.0f * M_2PI){
                int diff = Encoder.raw - count_raw_start;
                if (diff > +ENCODER_CPR_DIV) {
                    diff -= ENCODER_CPR;
                } else if (diff < -ENCODER_CPR_DIV) {
                    diff += ENCODER_CPR;
                }

                if (diff < 0) {
                    if (Encoder.Config.encoder_reverse) {
                        Encoder.Config.encoder_reverse = 0;
                    } else {
                        Encoder.Config.encoder_reverse = 1;
                    }
                }
                
                Encoder.Calib.phase_set = 0;
                Encoder.Calib.calib_step ++;
            }
            break;
            
        case 3: // CW dumy
            Encoder.Calib.phase_set += phase_delta;
            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
            if(Encoder.Calib.phase_set >= M_2PI){
                Encoder.Calib.phase_set = 0;
                Encoder.Calib.loop_count = 0;
                Encoder.Calib.sample_count = 0;
                Encoder.Calib.next_sample_time = 0;
                Encoder.Calib.calib_step ++;
            }
            break;

        case 4: // CW loop
            if(Encoder.Calib.sample_count < (MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR)){
                if(t >= Encoder.Calib.next_sample_time){
                    Encoder.Calib.next_sample_time += sample_time_delta;
                    
                    int count_ref = (Encoder.Calib.phase_set * ENCODER_CPR_F) / (M_2PI * (float)MOTOR_POLE_PAIRS);
                    int error = Encoder.raw - count_ref;
                    error += ENCODER_CPR * (error<0);
                    Encoder.Calib.errors[Encoder.Calib.sample_count] = error;
                    
                    Encoder.Calib.sample_count ++;
                }
                
                Encoder.Calib.phase_set += phase_delta;
            }else{
                Encoder.Calib.sample_count --;
                Encoder.Calib.loop_count = 0;
                Encoder.Calib.calib_step ++;
                break;
            }
            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
            break;
            
        case 5: // CW dumy
            if(Encoder.Calib.loop_count > (0.5f/dt)){
                Encoder.Calib.loop_count = 0;
                Encoder.Calib.calib_step ++;
                break;
            }
            Encoder.Calib.phase_set += phase_delta;
            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
            break;
        
        case 6: // CCW dumy
            if(Encoder.Calib.loop_count > (0.5f/dt)){
                Encoder.Calib.loop_count = 0;
                Encoder.Calib.next_sample_time = 0;
                Encoder.Calib.calib_step ++;
                break;
            }
            Encoder.Calib.phase_set -= phase_delta;
            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
            break;
        
        case 7: // CCW loop
            if(Encoder.Calib.sample_count >= 0){
                if(t > Encoder.Calib.next_sample_time){
                    Encoder.Calib.next_sample_time += sample_time_delta;
                    
                    int count_ref = (Encoder.Calib.phase_set * ENCODER_CPR_F) / (M_2PI * (float)MOTOR_POLE_PAIRS);
                    int error = Encoder.raw - count_ref;
                    error += ENCODER_CPR * (error<0);
                    Encoder.Calib.errors[Encoder.Calib.sample_count] = (Encoder.Calib.errors[Encoder.Calib.sample_count] + error) / 2;

                    Encoder.Calib.sample_count --;
                }
                
                Encoder.Calib.phase_set -= phase_delta;
            }else{
                Encoder.Calib.calib_step ++;
                break;
            }
            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
            break;
            
        case 8: // Calculate
            {
                // Missing Magnet Check (threshold set to half a turn = 8192 counts)
                if((in_max - in_min) < 8192 || (ex_max - ex_min) < 8192) 
                {
                    COM_CAN_report_err(ERR_ENC_MISSING);
                    Encoder.Config.calib_valid = false;
                    Encoder.Calib.calib_step = 9; // Abort calibration
                    break;
                }

                // Calculate average offset
                int64_t moving_avg = 0;
                for(int i = 0; i<(MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR); i++){
                    moving_avg += Encoder.Calib.errors[i];
                }
                Encoder.Config.encoder_offset = moving_avg/(MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR);
                
                //DEBUG("ENCODER_OFFSET: %d\n", Encoder.Config.encoder_offset);
                
                // FIR and map measurements to lut
                int window = SAMPLES_PER_PPAIR;
                int lut_offset = Encoder.Calib.errors[0] * ENCODER_OFFSET_LUT_NUM / ENCODER_CPR;
                
                int total_samples = MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR;
                
                for(int i = 0; i < ENCODER_OFFSET_LUT_NUM; i++){
                    // 1. Calculate fractional center index
                    float center_f = (float)i * total_samples / ENCODER_OFFSET_LUT_NUM;
                    int center_i = (int)center_f;
                    float frac = center_f - center_i;
                    
                    int64_t moving_avg_floor = 0;
                    int64_t moving_avg_ceil = 0;
                    
                    // 2. Compute sliding window sum for floor and ceil positions
                    for(int j = (-window)/2; j < (window)/2; j++){
                        int idx1 = center_i + j;
                        int idx2 = center_i + 1 + j;
                        
                        // Robust bounds wrapping for any array size
                        while(idx1 < 0) idx1 += total_samples;
                        while(idx1 >= total_samples) idx1 -= total_samples;
                        
                        while(idx2 < 0) idx2 += total_samples;
                        while(idx2 >= total_samples) idx2 -= total_samples;
                        
                        moving_avg_floor += Encoder.Calib.errors[idx1];
                        moving_avg_ceil  += Encoder.Calib.errors[idx2];
                    }
                    
                    // 3. Linear interpolation
                    float avg_floor_f = (float)moving_avg_floor / window;
                    float avg_ceil_f  = (float)moving_avg_ceil / window;
                    int64_t moving_avg = (int64_t)(avg_floor_f * (1.0f - frac) + avg_ceil_f * frac);
                    
                    // 4. Map to final LUT index with robust wrapping
                    int lut_index = lut_offset + i;
                    while(lut_index >= ENCODER_OFFSET_LUT_NUM) {
                        lut_index -= ENCODER_OFFSET_LUT_NUM;
                    }
                    while(lut_index < 0) {
                        lut_index += ENCODER_OFFSET_LUT_NUM;
                    }
                    
                    Encoder.Config.encoder_offset_lut[lut_index] = moving_avg - Encoder.Config.encoder_offset;
                }

                ERROR_CLR(ERR_ENC_CALIB);
                SW_ERROR_RESET();
                Encoder.Config.calib_valid = true;
                
                save_encoder_config();
                
                MC_set_state(MCS_IDLE);
                Encoder.Calib.calib_step ++;
            }
            break;

        default:
            break;
    }
    
    // Update extents for missing magnet detection
    if(Encoder.raw > in_max) in_max = Encoder.raw;
    if(Encoder.raw < in_min) in_min = Encoder.raw;
    
    int32_t ex_raw = ENCODER_EX_read();
    if(ex_raw > ex_max) ex_max = ex_raw;
    if(ex_raw < ex_min) ex_min = ex_raw;

    Encoder.Calib.loop_count ++;
}

int32_t ENCODER_EX_read(void)
{
    uint16_t data[2];
    uint16_t sample_data;
    
    EX_NCS_RESET();
    spi_i2s_data_transmit(SPI2, 0x8300);
    while(RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE));
    data[0] = spi_i2s_data_receive(SPI2);
    EX_NCS_SET();
    
    EX_NCS_RESET();
    spi_i2s_data_transmit(SPI2, 0x8400);
    while(RESET == spi_i2s_flag_get(SPI2, SPI_FLAG_RBNE));
    data[1] = spi_i2s_data_receive(SPI2);
    EX_NCS_SET();
    
    sample_data = ((data[0] & 0x00FF) << 8) | (data[1] & 0x00FF);
    
    return (sample_data >> 2);
}

int32_t ENCODER_read(void)
{
    uint16_t data[2];
    uint16_t sample_data;
    
    ENC_NCS_RESET();
    spi_i2s_data_transmit(SPI0, 0x8300);
    while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
    data[0] = spi_i2s_data_receive(SPI0);
    ENC_NCS_SET();
    
    ENC_NCS_RESET();
    spi_i2s_data_transmit(SPI0, 0x8400);
    while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
    data[1] = spi_i2s_data_receive(SPI0);
    ENC_NCS_SET();
    
    sample_data = ((data[0] & 0x00FF) << 8) | (data[1] & 0x00FF);
    
    return (sample_data >> 2);
}
float raw_rad_data = 0.0f;
	int init_in;
	uint16_t init_ex;
	uint16_t init_in_offset;
	uint16_t init_ex_offset;
int raw_encoder = 0;
float Real_Velocity;
#define VEL_AVERAGE_FILTER_NUM 32
float vel_vec[VEL_AVERAGE_FILTER_NUM] = {0.0f};
float Velocity_Filtered;


#include <stdint.h>
#include <math.h>

// --- ?????????? ---
#define ENCODER_RESOLUTION 16384 
#define TWO_PI 6.28318530718f 
#define US_TO_S_FACTOR 1000000.0f 

// --- ---- ---
// ????????(????)
#define MAX_PHYSICAL_SPEED_RADS 1000000.0f 

// --- ?????? ---
#define MAF_FILTER_SIZE 32 

// --- ?????? ---
static uint16_t last_position = 0;
static float speed_history[MAF_FILTER_SIZE] = {0.0f};
static uint8_t history_index = 0; 
static uint8_t sample_count = 0; 
static uint8_t is_first_call = 1;


/**
 * @brief ?14???????????????????? (rad/s)
 *
 * @param current_position ??????????? (0 ~ 16383)
 * @param delta_time_us ???????????????? (??,us)
 * @return float ?????? (rad/s),???? [-MAX_PHYSICAL_SPEED_RADS, +MAX_PHYSICAL_SPEED_RADS] ???
 */
float get_angular_velocity_rads_v3(uint16_t current_position, int64_t delta_time_us) {
    
    // 1. ??????
    if (is_first_call) {
        last_position = current_position;
        is_first_call = 0;
        return 0.0f; 
    }

    // 2. ?????????
    if (delta_time_us <= 0) {
        return 0.0f; 
    }
    
    float delta_time_s = (float)delta_time_us / US_TO_S_FACTOR;

    // 3. ??????? (?P),????????
    int32_t delta_position;
    delta_position = (int32_t)current_position - last_position; 
    const int32_t half_resolution = ENCODER_RESOLUTION / 2;
    
    if (delta_position > half_resolution) {
        delta_position -= ENCODER_RESOLUTION;
    } else if (delta_position < -half_resolution) {
        delta_position += ENCODER_RESOLUTION;
    }
    
    
    // 4. ??????? (Raw Angular Velocity) (rad/s)
    float raw_angular_velocity = ((float)delta_position / ENCODER_RESOLUTION) * (TWO_PI / delta_time_s);

    
    // 5. ?? ??:??????/???
    // ??????,???????????????
    float limited_raw_velocity = raw_angular_velocity;
    
    if (limited_raw_velocity > MAX_PHYSICAL_SPEED_RADS) {
        limited_raw_velocity = MAX_PHYSICAL_SPEED_RADS;
    } else if (limited_raw_velocity < -MAX_PHYSICAL_SPEED_RADS) {
        limited_raw_velocity = -MAX_PHYSICAL_SPEED_RADS;
    }
    // ?????????????????????????????

    
    // 6. ???? (Moving Average Filter)
    
    // A. ????????
    speed_history[history_index] = limited_raw_velocity;
    
    // B. ?????????
    history_index = (history_index + 1) % MAF_FILTER_SIZE;
    if (sample_count < MAF_FILTER_SIZE) {
        sample_count++;
    }

    // C. ?????
    float speed_sum = 0.0f;
    for (uint8_t i = 0; i < sample_count; i++) {
        speed_sum += speed_history[i];
    }
    
    float filtered_angular_velocity = speed_sum / sample_count;

    // 7. ???????
    last_position = current_position;
    
    return filtered_angular_velocity;
}
//int16_t multi_test = 0;

// int32_t Get_Multi_Turns(){
//	int16_t Multi_Turns;

//	uint16_t Mech_Angle_Err = IN_ENCODER_OFFSET;
//	uint16_t Mech_Angle_Side_Err = EX_ENCODER_OFFSET;
//	uint16_t encoder_two = ENCODER_EX_read();
//	int encoder_one = Encoder.raw;
//	uint16_t Mech_Angle = encoder_one;
//	static bool init_multi = true;
//	uint16_t  Mech_Differ = -(65535 - (((encoder_one << 2) - (Mech_Angle_Err << 2)) -  ((encoder_two << 2) - (Mech_Angle_Side_Err << 2))));
//	if((uint16_t)((encoder_one << 2) - (Mech_Angle_Err << 2)) >= 55535){
//		Mech_Differ -= 800;
//	}
//	if((uint16_t)((encoder_one << 2) - (Mech_Angle_Err << 2)) <= 10000){
//		Mech_Differ += 800;
//	}					

//	int16_t Multi_Turns_1 = (Mech_Differ <= 32767 ) ? floor(((uint32_t)Mech_Differ * 18) / 65536) : floor(((uint32_t)Mech_Differ * 18) / 65536) - 18;

//	
//	if(1){
//		init_in_offset = IN_ENCODER_OFFSET;
//		init_ex_offset = EX_ENCODER_OFFSET;
//		init_in = Encoder.raw;
//		init_ex = encoder_two;
//		Multi_Turns = (Mech_Differ <= 32767 ) ? floor(((uint32_t)Mech_Differ * 18) / 65536) : floor(((uint32_t)Mech_Differ * 18) / 65536) - 18;
//	}
// 	return multi_test;
// }

//int32_t multi_turn_check_111 = 0;
//bool init_multi = true;


//int32_t Multi_Turns;

void multi_encoder(void)
{
    // 计算上电零点偏移
    static uint16_t init = 0;
    static float pos_fast_offset = 0.0f; // 转子上电瞬间相对零点的偏移(rad)

    if(init == 0)
    {
        init = 1;
        int32_t in_off = IN_ENCODER_OFFSET;
        int32_t ex_off = EX_ENCODER_OFFSET;
        int32_t ex_raw = ENCODER_EX_read();
        
        if (ODObjs.polarity) {
            ex_raw = ENCODER_CPR - 1 - ex_raw;
            ex_off = ENCODER_CPR - 1 - ex_off;
        }
        
        int16_t enc_err_config = (in_off - ex_off) << 2;
        int16_t enc_err_mes = (Encoder.raw - ex_raw) << 2;
        int16_t enc_err = enc_err_mes - enc_err_config;  
        
        pos_fast_offset = enc_err / ((((float)g_multi_sec_gear - (float)g_multi_pri_gear) / (float)g_multi_sec_gear) * 65536.0f) * M_2PI; // 计算出转子上电瞬间相对零点的偏移
    }

    // 计算角度增量
    static int16_t enc_degree_last = 0;
    int16_t enc_degree_delta = 0;
    static int64_t enc_degree_fast_sum = 0;
    static uint16_t flag = 0;
    if(flag == 0)
    {
        enc_degree_last = Encoder.count_in_cpr;
        flag = 1;
    }
    enc_degree_delta = Encoder.count_in_cpr - enc_degree_last;
    if(enc_degree_delta > ENCODER_CPR_DIV) enc_degree_delta -= ENCODER_CPR;
    else if(enc_degree_delta < -ENCODER_CPR_DIV) enc_degree_delta += ENCODER_CPR;
    enc_degree_fast_sum += enc_degree_delta;
    enc_degree_last = Encoder.count_in_cpr;
    
    raw_rad_data = (pos_fast_offset + (enc_degree_fast_sum / (float)ENCODER_CPR * M_2PI)) / g_gear_ratio; 
}


//int encoder_count = 0;



// int32_t in_encoder_turns = 0;
// int32_t ex_encoder_turns = 0;
void ENCODER_loop(void)
{
	// encoder_count ++;
//	if(encoder_count > 10){
//		return;
//	}
//
    Encoder.raw = ENCODER_read();
    if (Encoder.Config.encoder_reverse) {
        Encoder.raw = ENCODER_CPR - 1 - Encoder.raw;
    }
    
    /* Linearization */
    int n     = ENCODER_BITS - CALIB_LUT_BITS;
    int off_1 = Encoder.Config.encoder_offset_lut[(Encoder.raw)>>n];                              // lookup table lower entry
    int off_2 = Encoder.Config.encoder_offset_lut[((Encoder.raw>>n)+1)%ENCODER_OFFSET_LUT_NUM];   // lookup table higher entry
    int off_interp = off_1 + ((off_2 - off_1)*(Encoder.raw - ((Encoder.raw>>n)<<n))>>n);          // Interpolate between lookup table entries
    
    int count = Encoder.raw - off_interp - Encoder.Config.encoder_offset;
    
    /*  Wrap in ENCODER_CPR */
    while (count > ENCODER_CPR)
        count -= ENCODER_CPR;
    while (count < 0)
        count += ENCODER_CPR;
    
    Encoder.count_in_cpr = count;
    
    if(Encoder.need_init){
        Encoder.count_in_cpr_prev = Encoder.count_in_cpr;
        Encoder.pll_pos = Encoder.count_in_cpr;
        Encoder.pll_vel = 0;
        
        Encoder.need_init --;
        
        multi_encoder();
        return;
    }
		multi_encoder();
//ex_encoder_turns = Get_Multi_Turns();
//in_encoder_turns = multi_turn_check_111;
    /* Delta count */
    int delta_count = Encoder.count_in_cpr - Encoder.count_in_cpr_prev;
    Encoder.count_in_cpr_prev = Encoder.count_in_cpr;
    while(delta_count > +ENCODER_CPR_DIV) delta_count -= ENCODER_CPR;
    while(delta_count < -ENCODER_CPR_DIV) delta_count += ENCODER_CPR;

    // Run pll (for now pll is in units of encoder counts)
    // Predict current pos
    Encoder.pll_pos += ENCODER_PLL_DT * Encoder.pll_vel;
    // Discrete phase detector
    float delta_pos = Encoder.count_in_cpr - floorf(Encoder.pll_pos);
    while(delta_pos > +ENCODER_CPR_DIV) delta_pos -= ENCODER_CPR_F;
    while(delta_pos < -ENCODER_CPR_DIV) delta_pos += ENCODER_CPR_F;
    // PLL feedback
    Encoder.pll_pos += ENCODER_PLL_DT * Encoder.pll_kp * delta_pos;
    while(Encoder.pll_pos > ENCODER_CPR) Encoder.pll_pos -= ENCODER_CPR_F;
    while(Encoder.pll_pos < 0          ) Encoder.pll_pos += ENCODER_CPR_F;
    Encoder.pll_vel += ENCODER_PLL_DT * Encoder.pll_ki * delta_pos;
    
    // Align delta-sigma on zero to prevent jitter
    if (ABS(Encoder.pll_vel) < Encoder.snap_threshold) {
        Encoder.pll_vel = 0.0f;
    }
//		Velocity_Filtered = get_angular_velocity_rads_v3(Encoder.raw, 500);
    /* Outputs from Encoder for Controller */
    Encoder.shadow_count += delta_count;
    Encoder.phase = Encoder.pll_pos * M_2PI * MOTOR_POLE_PAIRS / ENCODER_CPR_F;
    Encoder.vel = Encoder.pll_vel;
    Encoder.phase_vel = Encoder.vel * M_2PI * MOTOR_POLE_PAIRS / ENCODER_CPR_F;
//		Velocity_Filtered = Encoder.vel/2608.917197/g_gear_ratio;
	  Velocity_Filtered = get_angular_velocity_rads_v3(Encoder.raw, 50) / g_gear_ratio;


}


void SmoothTransition(
    double t, 
    double T, 
    double q_start, 
    double q_offset, 
    double *q, 
    double *qd
) {
    // 1. ??????????? q_final
    // q_final = q_start + q_offset
    double q_final = q_start + q_offset; 
    
    // 2. ??????????
    if (t >= T) {
        *q = q_final;
        *qd = 0.0;
        return;
    }
    
    // 3. ???????? t
    if (t <= 0.0) {
        *q = q_start;
        *qd = 0.0;
        return;
    }

    // --- ???? ---
    const double pi = M_PI; // M_PI ??? math.h ?
    double delta_q = q_final - q_start; // ??? (rad)
    double ratio = t / T;               // ????? t/T (0?1)
    
    // ????
    *q = q_start + delta_q * 0.5 * (1.0 - cos(pi * ratio));
    
    // ????
    // ??: cos(pi*ratio) ???? -sin(pi*ratio) * (pi/T),
    // ?? 1 - cos(...) ???? +sin(pi*ratio) * (pi/T)
    *qd = delta_q * 0.5 * (pi / T) * sin(pi * ratio);
}


typedef struct {
    double time_current;  // t: ???? (s)
    double time_total;    // T: ????? (s)
    double q_start;       // q0: ????????? (rad)
    double q_offset;      // q1_offset: ????/?? (rad)
} MotionInput;

/**
 * @brief 2. ???????? (TrajectorySetpoint)
 * ???????????
 */
typedef struct {
    double position;    // q: ???????? (rad)
    double velocity;    // qd: ???????? (rad/s)
} TrajectorySetpoint;


/**
 * @brief ????????????????
 * * @param input ????????????
 * @return TrajectorySetpoint ?????????????
 */
TrajectorySetpoint SmoothTransition_Struct(const MotionInput *input) {
    
    TrajectorySetpoint output;
    
    // 1. ??????????? q_final
    // q_final = q_start + q_offset
    double q_final = input->q_start + input->q_offset; 
    
    // 2. ??????????
    if (input->time_current >= input->time_total) {
        output.position = q_final;
        output.velocity = 0.0;
        return output;
    }
    
    // 3. ???????? t (??????)
    if (input->time_current <= 0.0) {
        output.position = input->q_start;
        output.velocity = 0.0;
        return output;
    }

    // --- ???? ---
    double delta_q = q_final - input->q_start; // ??? (rad)
    double ratio = input->time_current / input->time_total; // ????? t/T (0?1)
    
    // ????: q = q_start + delta_q * 0.5 * (1 - cos(pi * ratio))
    output.position = input->q_start + delta_q * 0.5 * (1.0 - cos(M_PI * ratio));
    
    // ????: qd = delta_q * 0.5 * (pi / T) * sin(pi * ratio)
    output.velocity = delta_q * 0.5 * (M_PI / input->time_total) * sin(M_PI * ratio);
    
    return output;
}

// ----------------------------------------------------------------------------
// General Slip Check for Dual Encoder (16-bit Abstracted)
// ----------------------------------------------------------------------------
// uint16_t ENCODER_slip_check(float tolerance_rad)
// {
//     // Skip check if not using dual encoder branches
//     if(g_current_branch == BRANCH_UNKNOWN) {
//         return 0;
//     }

//     // Skip check if encoder is not calibrated (magnets might not be installed or aligned)
//     if(!Encoder.Config.calib_valid) {
//         return 0;
//     }

//     // Abstract 14-bit MT6816 data to 16-bit space, casting floats to uint16_t first
//     uint16_t in_16b = (uint16_t)((Encoder.raw & 0x3FFF) << 2);
//     uint16_t ex_16b = (uint16_t)(((uint16_t)EX_ENCODER_VALUE & 0x3FFF) << 2);
    
//     uint16_t offset_in_16b = (uint16_t)((IN_ENCODER_OFFSET & 0x3FFF) << 2);
//     uint16_t offset_ex_16b = (uint16_t)((EX_ENCODER_OFFSET & 0x3FFF) << 2);

//     // Calculate nominal mismatch from saved EEPROM offsets (perfect rigid relationship at zero)
//     // Formula: (P_s * IN - P_r * EX) mod 65536
//     uint16_t nom_mismatch = (uint16_t)((uint32_t)g_multi_pri_gear * offset_in_16b - 
//                                        (uint32_t)g_multi_sec_gear * offset_ex_16b);

//     // Calculate current mismatch from real-time raw values
//     uint16_t cur_mismatch = (uint16_t)((uint32_t)g_multi_pri_gear * in_16b - 
//                                        (uint32_t)g_multi_sec_gear * ex_16b);

//     // Difference (signed 16-bit automatically handles wraparound)
//     int16_t diff = (int16_t)(cur_mismatch - nom_mismatch);

//     // Calculate threshold based on physical inner shaft radians
//     // diff represents error: 1 count of in_16b gives g_multi_pri_gear counts of diff.
//     // diff_thresh = (TOLERANCE_RAD / 2PI) * 65536 * g_multi_pri_gear
//     int32_t diff_thresh = (int32_t)((tolerance_rad / M_2PI) * 65536.0f * (float)g_multi_pri_gear);

//     // Clamp threshold to prevent Nyquist limit overflow silent failures
//     if (diff_thresh > 28000) {
//         diff_thresh = 28000; 
//     }

//     if(abs((int)diff) > diff_thresh){
//         //return ERR_ENC_SLIP;
//     }

//     return 0;
// }
