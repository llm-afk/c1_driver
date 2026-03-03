#include "encoder.h"
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
    memcpy(&Encoder.Config, (uint8_t*)(ENCODER_CALIB_PAGE*FLASH_PAGE_SIZE), sizeof(tEncoderConfig));
    uint32_t crc = crc32((uint8_t*)&Encoder.Config, sizeof(tEncoderConfig)-4);
    if(crc != Encoder.Config.crc){
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
    // Erase
    FI_flash_erase_page(ENCODER_CALIB_PAGE);
    
    // Program
    Encoder.Config.crc = crc32((uint8_t*)&Encoder.Config, sizeof(tEncoderConfig)-4);
    FI_flash_write((uint8_t*)(FLASH_BASE + ENCODER_CALIB_PAGE * FLASH_PAGE_SIZE), (uint8_t*)&Encoder.Config, sizeof(tEncoderConfig));
}

void ENCODER_calib_start(void)
{
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

}
uint32_t multi_check_flag = 0;
void ENCODER_calib_loop(float dt)
{
    static int count_raw_start;
    
    const float time = 10.24f;
    const float phase_delta = MOTOR_POLE_PAIRS * M_2PI * dt / time;
    const float sample_time_delta = time / (float)(MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR);

    const float t = (float)Encoder.Calib.loop_count * dt;
    const float voltage = ENCODER_CALIB_CURRENT * MOTOR_PHASE_R;
    
    switch(Encoder.Calib.calib_step){
        case 0: // Init
            Encoder.Calib.phase_set = 0;
            Encoder.Calib.loop_count = 0;
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
                for(int i = 0; i<ENCODER_OFFSET_LUT_NUM; i++){
                    moving_avg = 0;
                    for(int j = (-window)/2; j<(window)/2; j++){
                        int index = i*MOTOR_POLE_PAIRS*SAMPLES_PER_PPAIR/ENCODER_OFFSET_LUT_NUM + j;
                        if(index<0){
                            index += (SAMPLES_PER_PPAIR*MOTOR_POLE_PAIRS);
                        }else if(index>(SAMPLES_PER_PPAIR*MOTOR_POLE_PAIRS-1)){
                            index -= (SAMPLES_PER_PPAIR*MOTOR_POLE_PAIRS);
                        }
                        moving_avg += Encoder.Calib.errors[index];
                    }
                    moving_avg = moving_avg/window;
                    int lut_index = lut_offset + i;
                    if(lut_index > (ENCODER_OFFSET_LUT_NUM-1)){
                        lut_index -= ENCODER_OFFSET_LUT_NUM;
                    }
                    Encoder.Config.encoder_offset_lut[lut_index] = moving_avg - Encoder.Config.encoder_offset;
                }

                ERROR_CODE = 0;
                SW_ERROR_RESET();
                Encoder.Config.calib_valid = true;
                
                save_encoder_config();
                
                MC_set_state(MCS_IDLE);
//								multi_check_flag = 1;
                Encoder.Calib.calib_step ++;
            }
            break;

        default:
            break;
    }
    
    Encoder.Calib.loop_count ++;
}

//void ENCODER_calib_loop(float dt)
//{
//    const float time = 10.24f;
//    const float phase_delta = MOTOR_POLE_PAIRS * M_2PI * dt / time;
//    const float sample_time_delta = time / (float)(MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR);

//    const float t = (float)Encoder.Calib.loop_count * dt;
//    const float voltage = ENCODER_CALIB_CURRENT * MOTOR_PHASE_R;

//    switch(Encoder.Calib.calib_step){
//        case 0: // Init
//            Encoder.Calib.phase_set = 0;
//            Encoder.Calib.loop_count = 0;
//            Encoder.Calib.calib_step ++;
//            break;

//        case 1: // Lock
//            MC_modulate((voltage * t / 2.0f), 0, Encoder.Calib.phase_set);
//            if (t >= 2.0f){
//                Encoder.Calib.calib_step ++;
//            }
//            break;
//            
//        case 2: // CW dumy
//            Encoder.Calib.phase_set += phase_delta;
//            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
//            if(Encoder.Calib.phase_set >= M_2PI){
//                Encoder.Calib.phase_set = 0;
//                Encoder.Calib.loop_count = 0;
//                Encoder.Calib.sample_count = 0;
//                Encoder.Calib.next_sample_time = 0;
//                Encoder.Calib.calib_step ++;
//                
//                for(int i=0; i<10; i++){
//                    Encoder.Config.encoder_ex_offset = ENCODER_EX_read();
//                    if(Encoder.Config.encoder_ex_offset != -1){
//                        break;
//                    }
//                }
//            }
//            break;

//        case 3: // CW loop
//            if(Encoder.Calib.sample_count < (MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR)){
//                if(t >= Encoder.Calib.next_sample_time){
//                    Encoder.Calib.next_sample_time += sample_time_delta;
//                    
//                    int count_ref = (Encoder.Calib.phase_set * ENCODER_CPR_F) / (M_2PI * (float)MOTOR_POLE_PAIRS);
//                    int error = Encoder.raw - count_ref;
//                    error += ENCODER_CPR * (error<0);
//                    Encoder.Calib.errors[Encoder.Calib.sample_count] = error;
//                    
//                    Encoder.Calib.sample_count ++;
//                }
//                
//                Encoder.Calib.phase_set += phase_delta;
//            }else{
//                Encoder.Calib.sample_count --;
//                Encoder.Calib.loop_count = 0;
//                Encoder.Calib.calib_step ++;
//                break;
//            }
//            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
//            break;
//            
//        case 4: // CW dumy
//            if(Encoder.Calib.loop_count > (0.5f/dt)){
//                Encoder.Calib.loop_count = 0;
//                Encoder.Calib.calib_step ++;
//                break;
//            }
//            Encoder.Calib.phase_set += phase_delta;
//            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
//            break;
//        
//        case 5: // CCW dumy
//            if(Encoder.Calib.loop_count > (0.5f/dt)){
//                Encoder.Calib.loop_count = 0;
//                Encoder.Calib.next_sample_time = 0;
//                Encoder.Calib.calib_step ++;
//                break;
//            }
//            Encoder.Calib.phase_set -= phase_delta;
//            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
//            break;
//        
//        case 6: // CCW loop
//            if(Encoder.Calib.sample_count >= 0){
//                if(t > Encoder.Calib.next_sample_time){
//                    Encoder.Calib.next_sample_time += sample_time_delta;
//                    
//                    int count_ref = (Encoder.Calib.phase_set * ENCODER_CPR_F) / (M_2PI * (float)MOTOR_POLE_PAIRS);
//                    int error = Encoder.raw - count_ref;
//                    error += ENCODER_CPR * (error<0);
//                    Encoder.Calib.errors[Encoder.Calib.sample_count] = (Encoder.Calib.errors[Encoder.Calib.sample_count] + error) / 2;

//                    Encoder.Calib.sample_count --;
//                }
//                
//                Encoder.Calib.phase_set -= phase_delta;
//            }else{
//                Encoder.Calib.calib_step ++;
//                break;
//            }
//            MC_modulate(voltage, 0, Encoder.Calib.phase_set);
//            break;
//            
//        case 7: // Calculate
//            {
//                // Calculate average offset
//                int64_t moving_avg = 0;
//                for(int i = 0; i<(MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR); i++){
//                    moving_avg += Encoder.Calib.errors[i];
//                }
//                Encoder.Config.encoder_offset = moving_avg/(MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR);
//                
//                //DEBUG("ENCODER_OFFSET: %d\n", Encoder.Config.encoder_offset);
//                
//                // FIR and map measurements to lut
//                int window = SAMPLES_PER_PPAIR;
//                int lut_offset = Encoder.Calib.errors[0] * ENCODER_OFFSET_LUT_NUM / ENCODER_CPR;
//                for(int i = 0; i<ENCODER_OFFSET_LUT_NUM; i++){
//                    moving_avg = 0;
//                    for(int j = (-window)/2; j<(window)/2; j++){
//                        int index = i*MOTOR_POLE_PAIRS*SAMPLES_PER_PPAIR/ENCODER_OFFSET_LUT_NUM + j;
//                        if(index<0){
//                            index += (SAMPLES_PER_PPAIR*MOTOR_POLE_PAIRS);
//                        }else if(index>(SAMPLES_PER_PPAIR*MOTOR_POLE_PAIRS-1)){
//                            index -= (SAMPLES_PER_PPAIR*MOTOR_POLE_PAIRS);
//                        }
//                        moving_avg += Encoder.Calib.errors[index];
//                    }
//                    moving_avg = moving_avg/window;
//                    int lut_index = lut_offset + i;
//                    if(lut_index > (ENCODER_OFFSET_LUT_NUM-1)){
//                        lut_index -= ENCODER_OFFSET_LUT_NUM;
//                    }
//                    Encoder.Config.encoder_offset_lut[lut_index] = moving_avg - Encoder.Config.encoder_offset;
//                }

//                ERROR_CODE = 0;
//                SW_ERROR_RESET();
//                Encoder.Config.calib_valid = true;
//                
//                save_encoder_config();
//                
//                MC_set_state(MCS_IDLE);
//                Encoder.Calib.calib_step ++;
//            }
//            break;

//        default:
//            break;
//    }
//    
//    Encoder.Calib.loop_count ++;
//}

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

int32_t ENCODER_EX_read_rectified(void)
{
    int raw = ENCODER_EX_read();
    if(raw != -1){
        raw = raw - Encoder.Config.encoder_ex_offset;
        if(raw < 0){
            raw += ENCODER_CPR;
        }
    }
    return raw;
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
static const int32_t vel_average_filter_num = 32;
float vel_vec[vel_average_filter_num] = {0.0f};
float Velocity_Filtered;


#include <stdint.h>
#include <math.h>

// --- ?????????? ---
#define ENCODER_RESOLUTION 16384 
#define TWO_PI 6.28318530718f 
#define US_TO_S_FACTOR 1000000.0f 

// --- ??/???? ---
// ????????(????)
#define MAX_PHYSICAL_SPEED_RADS 32.0f*12 

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
int16_t multi_test = 0;

int32_t Get_Multi_Turns(){
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
	return multi_test;
}
int32_t multi_turn_check_111 = 0;
bool init_multi = true;

//void multi_encoder(void){
//	static	int16_t Multi_Turns;

//	static uint16_t Mech_Angle_Old;
//	static float Real_Angle_Old;
//	uint16_t Mech_Angle_Err = IN_ENCODER_OFFSET;
//	uint16_t Mech_Angle_Side_Err = EX_ENCODER_OFFSET;
//	int encoder_one = Encoder.raw;
//	uint16_t Mech_Angle = encoder_one;
//	

//	
//	if(init_multi){
//		Multi_Turns = Get_Multi_Turns();
//	}
//	init_multi = false;
//	// if(init_multi){
//		//  Encoder.shadow_count += Multi_Turns*ENCODER_RESOLUTION;
//		// Encoder.shadow_count -= Mech_Angle_Err;
//	// 					}
//	// 					multi_debug = (Mech_Differ <= 32767 ) ? floor(((uint32_t)Mech_Differ * 31) / 65536) : floor(((uint32_t)Mech_Differ * 31) / 65536) - 31;
//	// //					Multi_Turns = Multi_Turns % 12;

//	if(((Mech_Angle_Old -  Mech_Angle_Err )& 0x3FFF) > 12000 && ((Mech_Angle - Mech_Angle_Err) & 0x3FFF) < 4000) Multi_Turns += 1;
//	if(((Mech_Angle_Old -  Mech_Angle_Err )& 0x3FFF) < 4000 && ((Mech_Angle - Mech_Angle_Err) & 0x3FFF) > 12000) Multi_Turns -= 1;
//	multi_turn_check_111 = Multi_Turns;
//	Mech_Angle_Old = Mech_Angle;
//	float Real_Angle = ((float)((int)Multi_Turns) * M_2PI + (float)((uint16_t)((Mech_Angle << 2) - (Mech_Angle_Err << 2))) / 10430.4f)/GEAR_RATIO; // Real Angle in rad.
//	Real_Velocity = (Real_Angle - Real_Angle_Old) / ENCODER_PLL_DT;
//	Real_Angle_Old = Real_Angle;
//	float vel_sum = Real_Velocity;
//			for (int i = 1; i < vel_average_filter_num; i++)
//			{
//				vel_vec[vel_average_filter_num - i] = vel_vec[vel_average_filter_num - i - 1];
//				vel_sum += vel_vec[vel_average_filter_num - i];
//			}
//			vel_vec[0] = Real_Velocity;
//		float	Real_Velocity_Filtered = vel_sum / (float)vel_average_filter_num;
//		raw_rad_data = Real_Angle;
////Velocity_Filtered = Real_Velocity_Filtered;

//}

	int32_t Multi_Turns;

void multi_encoder(void){

	static uint16_t Mech_Angle_Old;
	static float Real_Angle_Old;
	uint16_t Mech_Angle_Err = IN_ENCODER_OFFSET;
	uint16_t Mech_Angle_Side_Err = EX_ENCODER_OFFSET;
	uint16_t encoder_two = ENCODER_EX_read();
	int encoder_one = Encoder.raw;
	uint16_t Mech_Angle = encoder_one;
	static bool init_multi = true;
	uint16_t  Mech_Differ = -(65535 - (((encoder_one << 2) - (Mech_Angle_Err << 2)) -  ((encoder_two << 2) - (Mech_Angle_Side_Err << 2))));
	if((uint16_t)((encoder_one << 2) - (Mech_Angle_Err << 2)) >= 55535){
		Mech_Differ -= 800;
	}
	if((uint16_t)((encoder_one << 2) - (Mech_Angle_Err << 2)) <= 10000){
		Mech_Differ += 800;
	}					

	int16_t Multi_Turns_1 = (Mech_Differ <= 32767 ) ? floor(((uint32_t)Mech_Differ * 32) / 65536) : floor(((uint32_t)Mech_Differ * 32) / 65536) - 32;
	multi_test = Multi_Turns_1;
	if(init_multi){
		init_in_offset = IN_ENCODER_OFFSET;
		init_ex_offset = EX_ENCODER_OFFSET;
		init_in = Encoder.raw;
		init_ex = encoder_two;
		Multi_Turns = (Mech_Differ <= 32767 ) ? floor(((uint32_t)Mech_Differ * 32) / 65536) : floor(((uint32_t)Mech_Differ * 32) / 65536) - 32;
		Mech_Angle_Old = encoder_one;
	}
	init_multi = false;
	// if(init_multi){
		//  Encoder.shadow_count += Multi_Turns*ENCODER_RESOLUTION;
		// Encoder.shadow_count -= Mech_Angle_Err;
	// 					}
	// 					multi_debug = (Mech_Differ <= 32767 ) ? floor(((uint32_t)Mech_Differ * 31) / 65536) : floor(((uint32_t)Mech_Differ * 31) / 65536) - 31;
	// //					Multi_Turns = Multi_Turns % 12;

	if(((Mech_Angle_Old -  Mech_Angle_Err )& 0x3FFF) > 12000 && ((Mech_Angle - Mech_Angle_Err) & 0x3FFF) < 4000) Multi_Turns += 1;
	if(((Mech_Angle_Old -  Mech_Angle_Err )& 0x3FFF) < 4000 && ((Mech_Angle - Mech_Angle_Err) & 0x3FFF) > 12000) Multi_Turns -= 1;
	Mech_Angle_Old = Mech_Angle;
	multi_turn_check_111 = Multi_Turns;
	float Real_Angle = ((float)((int)Multi_Turns) * M_2PI + (float)((uint16_t)((Mech_Angle << 2) - (Mech_Angle_Err << 2))) / 10430.4f)/GEAR_RATIO; // Real Angle in rad.
	Real_Velocity = (Real_Angle - Real_Angle_Old) / ENCODER_PLL_DT;
	Real_Angle_Old = Real_Angle;
	float vel_sum = Real_Velocity;
			for (int i = 0; i < vel_average_filter_num; i++)
			{
				vel_vec[vel_average_filter_num - i] = vel_vec[vel_average_filter_num - i - 1];
				vel_sum += vel_vec[vel_average_filter_num - i];
			}
			vel_vec[0] = Real_Velocity;
		float	Real_Velocity_Filtered = vel_sum / (float)vel_average_filter_num;
		raw_rad_data = Real_Angle;
//Velocity_Filtered = Real_Velocity_Filtered;

}


bool Multi_Turn_Test(void){
	
}

int encoder_count = 0;



int32_t in_encoder_turns = 0;
int32_t ex_encoder_turns = 0;
void ENCODER_loop(void)
{
	encoder_count ++;
//	if(encoder_count > 10){
//		return;
//	}
	  static int32_t count_init = 0;
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
        
//        // read ex encoder
//        int ex = ENCODER_EX_read_rectified();
//        if(ex != -1){
//            float c0 = 24 * Encoder.count_in_cpr / ENCODER_CPR_F;
//            float c1 = 27 * ex / ENCODER_CPR_F;
//            int diff = (int)((c0 - c1) + 0.5f);
//            if(diff <= 0){
//                diff += 27;
//            }
//            diff = (int)((diff / 3.0f) + 0.5f);
//            if(diff >= 9){
//                diff -= 9;
//            }
//            
//            Encoder.shadow_count = diff * ENCODER_CPR + Encoder.count_in_cpr - (int)HOME_OFFSET;
//            
//            if(Encoder.shadow_count < 0){
//                Encoder.shadow_count += ENCODER_CPR * 9;
//            }
//            
//            Encoder.need_init --;
//        }
        
        Encoder.need_init --;
        
        return;
    }
		multi_encoder();
ex_encoder_turns = Get_Multi_Turns();
in_encoder_turns = multi_turn_check_111;
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
//		Velocity_Filtered = Encoder.vel/2608.917197/12.0f;
	  Velocity_Filtered = get_angular_velocity_rads_v3(Encoder.raw, 50)/12.0f;


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
MotionInput input_data;
TrajectorySetpoint setpoint;
float test_time;
float test_pos;
extern bool no_reset;
//int32_t in_encoder_turns = 0;
//int32_t ex_encoder_turns = 0;
bool check_ex_encoder(void){
	static int32_t step = 0;
	float a =  240*3.14/180.0;
	float f = 0.1;
	float c = 0;
	double PI = 3.1415926;
	float pos = ((a * sin(2 * PI * f * input_data.time_current) + c));
	float vel = (2 * PI * f * a * sin(2 * PI * f * input_data.time_current));
	static uint32_t error_time = 0;
//		float vel = 0;
//float pos = 0;


//	static double Q_START = 0;       // ?????? q0 = PI/2 rad (90?)
//	static double Q_OFFSET = 0;      // ???? q1 = 2*PI rad (360?)
//	const double TOTAL_TIME = 2.5;       // ????? T (?)
//	const double TIME_STEP = 0.0005;       // ??????? (?)
//	static double current_time = 0.0;
//	double desired_q = 0.0;
//	double desired_qd = 0.0;
//	current_time += 0.0005;
//	SmoothTransition(
//            current_time, 
//            TOTAL_TIME, 
//            Q_START, 
//            Q_OFFSET, 
//            &desired_q, 
//            &desired_qd
//        );
//	MotorControl.pos_set = desired_q;
//	MotorControl.velocity_set = desired_qd;
//	MotorControl.Kp = 100;

//	MotorControl.Kd = 1;
//	if(current_time > TOTAL_TIME){
//		multi_check_flag = 0;
//	}
//ex_encoder_turns = Get_Multi_Turns();
//in_encoder_turns = multi_turn_check_111;
	uint8_t data[8];
//	return 0;
no_reset = true;
if(Get_Multi_Turns() - multi_turn_check_111){
	error_time ++;
		if(error_time > 10){
			COM_CAN_report_err(ERR_MULTI_CHECK_ERROR);
			multi_check_flag = 0;
			MotorControl.pos_set = 0;
			MotorControl.velocity_set = 0;
			MotorControl.Kp = 0;
			MotorControl.current_mit = 0;
			MotorControl.Kd = 0;
			input_data.time_current = 0;
			step = 0;
			return true;

	}
}else{
	error_time = 0;
	if(error_time > 10){
	
	}
}


	switch(step){
		case 0:
		*(uint16_t*)data = Encoder.raw;

		OD_write_2(0x2070, data);

		Multi_Turns = 0;
		init_multi = true;
		step = 4;
		input_data.time_current = 0;
		break;
		case 1:
			input_data.time_current += 0.0005;
			MotorControl.pos_set = pos;
			MotorControl.velocity_set = vel;
			MotorControl.Kp = 100;
			MotorControl.current_mit = 0;
			MotorControl.Kd = 1;
			if(input_data.time_current > 31.0){
							step = 2;

			}
			if(input_data.time_current > 30.0){
				MotorControl.Kp = 0;

				MotorControl.Kd = 0;

			}
			/// turn 360
//		Q_OFFSET = 6.2832;
		
		break;
		case 2:
//			input_data.time_current += 0.0005;
//			setpoint = SmoothTransition_Struct(&input_data);
//			MotorControl.pos_set = setpoint.position;
//			MotorControl.velocity_set = setpoint.velocity;
//			MotorControl.Kp = 100;

//			MotorControl.Kd = 1;
//			if(input_data.time_current >= input_data.time_total){
//				input_data.time_current = 0.000;
//				step = 5;
//				input_data.q_start = raw_rad_data;
//				input_data.q_offset = 4*M_PI;
//				input_data.time_total = 3;

//				break;
//			}
		
			multi_check_flag = 0;
			MotorControl.pos_set = 0;
			MotorControl.velocity_set = 0;
			MotorControl.Kp = 0;
			MotorControl.current_mit = 0;
			MotorControl.Kd = 0;
			input_data.time_current = 0;
			step = 0;
			return true;
			/// turn -360
			break;
		case 3:
			/// IDLE
			MotorControl.Kp = 0;
		input_data.time_current = 0.000;

			MotorControl.Kd = 0;
		step = 0;
		break;
		case 4:
						/// motor zero 
			input_data.time_current += 0.0005;
//			MotorControl.pos_set = pos;
//			MotorControl.velocity_set = vel;
//			MotorControl.Kp = 100;

//			MotorControl.Kd = 1;
//			if(input_data.time_current > 10.0){
//							step = 0;

//			}
		if(input_data.time_current > 2.0){
			input_data.q_start = raw_rad_data;
			input_data.q_offset = 2*M_PI;
			input_data.time_current = 0.000;
			input_data.time_total = 2.5;
			step = 1;
		}
		break;
		case 5:
			input_data.time_current += 0.0005;
			setpoint = SmoothTransition_Struct(&input_data);
			MotorControl.pos_set = setpoint.position;
			MotorControl.velocity_set = setpoint.velocity;
			MotorControl.Kp = 100;

			MotorControl.Kd = 1;
			if(input_data.time_current >= input_data.time_total){
				input_data.time_current = 0.000;
				step = 3;
				input_data.q_start = raw_rad_data;
				input_data.q_offset = -2*M_PI;
				input_data.time_total = 3;

				break;
			}
			/// turn -360
			break;
		default: 
			break;
	}

	
}
