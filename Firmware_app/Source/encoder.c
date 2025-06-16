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
extern int16_t Multi_Turns;
extern float out_rad;
extern uint16_t Mech_Differ;


#define ENCODER_RESOLUTION 16384        // 2^14 = 16384
#define HALF_RESOLUTION    (ENCODER_RESOLUTION / 2)
#define TWO_PI             6.2831852f

typedef struct {
    int initialized;
    uint16_t init_position_raw;   // 上电时第一编码器的原始 tick 值
    uint16_t last_position_raw;   // 上一次读取的编码器 tick
    int32_t accum_ticks;          // 从 last_position_raw 开始累计的 tick 差值
    float gear_ratio;             // 减速比
    float init_angle_rad;         // 上电时的“电机侧总角度”，含圈数（rad）
} AbsEncoderState;

// 初始化：传入第一编码器读数、减速比、圈数
void abs_encoder_init(AbsEncoderState* enc, uint16_t raw_val, float gear_ratio, int init_round) {
    enc->initialized = 1;
    enc->init_position_raw = raw_val;
    enc->last_position_raw = raw_val;
    enc->accum_ticks = 0;
    enc->gear_ratio = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;

    float raw_angle_rad = (float)(raw_val) * TWO_PI / ENCODER_RESOLUTION;
    enc->init_angle_rad = ((float)init_round) * TWO_PI + raw_angle_rad;
}

// 更新：每次传入当前第一编码器的值
void abs_encoder_update(AbsEncoderState* enc, uint16_t raw_val) {
    if (!enc->initialized) {
        abs_encoder_init(enc, raw_val, 1.0f, 0); // 默认参数
        return;
    }

    int32_t delta = (int32_t)raw_val - (int32_t)enc->last_position_raw;

    // 编码器跨零处理
    if (delta > HALF_RESOLUTION)
        delta -= ENCODER_RESOLUTION;
    else if (delta < -HALF_RESOLUTION)
        delta += ENCODER_RESOLUTION;

    enc->accum_ticks += delta;
    enc->last_position_raw = raw_val;
}

// 获取当前“关节侧”角度（单位：rad）
float abs_encoder_get_final_angle_rad(const AbsEncoderState* enc) {
    float delta_rad = (float)(enc->accum_ticks) * TWO_PI / ENCODER_RESOLUTION;
    return (enc->init_angle_rad + delta_rad) / enc->gear_ratio;
}

// 获取当前“关节侧”圈数（单位：turn）
float abs_encoder_get_final_angle_turn(const AbsEncoderState* enc) {
    return abs_encoder_get_final_angle_rad(enc) / TWO_PI;
}
uint16_t init_raw = 0;
float abs_raw = 0;
float abs_round = 0;
uint16_t Mech_Angle_Old = 0;
int16_t init_multi_turn = 0;
uint16_t init_ex_encoder = 0;
extern int16_t multi_debug;
void ENCODER_loop(void)
{ 
		static bool init_encoder = true;
		static AbsEncoderState encoder;
    Encoder.raw = ENCODER_read();

    if (Encoder.Config.encoder_reverse) {
        Encoder.raw = ENCODER_CPR - 1 - Encoder.raw;
    }
		
			uint16_t Mech_Angle = Encoder.raw ;
			uint16_t Mech_Angle_Side = ENCODER_EX_read() ;

			uint16_t Mech_Angle_Err = 4460;
			uint16_t Mech_Angle_Side_Err = 8576;
			uint16_t ex_encder = EX_ENCODER_VALUE;
		
			static bool init_multi = true;

			if(init_multi){
				uint16_t Mech_Differ = 65535 - (((Encoder.raw << 2) - (Mech_Angle_Err << 2)) -  ((ex_encder << 2) - (Mech_Angle_Side_Err << 2)));
				if((uint16_t)((Mech_Angle << 2) - (Mech_Angle_Err << 2)) >= 63535)
					Mech_Differ -= 500;
				else if((uint16_t)((Mech_Angle << 2) - (Mech_Angle_Err << 2)) <= 2000)
					Mech_Differ += 500;
				Multi_Turns = (Mech_Differ <= 32767) ? floor(((uint32_t)Mech_Differ * 31) / 65536) : floor(((uint32_t)Mech_Differ * 31) / 65536) - 31;
				init_multi_turn = Multi_Turns;
				init_ex_encoder = Mech_Angle_Side;
				float Real_Angle = - Multi_Turns * TWO_PI + (float)((uint16_t)((Mech_Angle << 2) - (Mech_Angle_Err << 2))) / 10430.4f; //! 这个角度已经是机械角了（磁编的多圈绝对编码还�??要加上offset才行�??

			}
//			multi_debug = (Mech_Differ <= 32767 ) ? floor(((uint32_t)Mech_Differ * 31) / 65536) : floor(((uint32_t)Mech_Differ * 31) / 65536) - 31;
//					Multi_Turns = Multi_Turns % 12;

			init_multi = false;
		
		if(init_encoder){
//			Encoder.shadow_count = Encoder.raw;
			init_raw = Encoder.raw;
			abs_encoder_init(&encoder, init_raw, 12.0f, Multi_Turns);  // 假设开机时读取的是100

			init_encoder = false;
		}
		uint16_t Mech_Differ = 		65535 - (((Encoder.raw << 2) - (Mech_Angle_Err << 2)) -  ((ex_encder << 2) - (Mech_Angle_Side_Err << 2)));
		if((uint16_t)((Mech_Angle << 2) - (Mech_Angle_Err << 2)) >= 65235)
			Mech_Differ -= 300;
		else if((uint16_t)((Mech_Angle << 2) - (Mech_Angle_Err << 2)) <= 300)
			Mech_Differ += 300;
		multi_debug = (Mech_Differ <= 32767) ? floor(((uint32_t)Mech_Differ * 31) / 65536) : floor(((uint32_t)Mech_Differ * 31) / 65536) - 31;
		if(((Mech_Angle_Old -  Mech_Angle_Err )& 0x3FFF) > 12000 && ((Mech_Angle - Mech_Angle_Err) & 0x3FFF) < 4000) Multi_Turns += 1;
		if(((Mech_Angle_Old -  Mech_Angle_Err )& 0x3FFF) < 4000 && ((Mech_Angle - Mech_Angle_Err) & 0x3FFF) > 12000) Multi_Turns -= 1;
		Mech_Angle_Old = Mech_Angle;
		float Real_Angle = ((float)((int)Multi_Turns) * TWO_PI + (float)((uint16_t)((Mech_Angle << 2) - (Mech_Angle_Err << 2))) / 10430.4f)/12.0f; // Real Angle in rad.
   


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
    
    /* Outputs from Encoder for Controller */
    Encoder.shadow_count += delta_count;
    Encoder.phase = Encoder.pll_pos * M_2PI * MOTOR_POLE_PAIRS / ENCODER_CPR_F;
    Encoder.vel = Encoder.pll_vel;
    Encoder.phase_vel = Encoder.vel * M_2PI * MOTOR_POLE_PAIRS / ENCODER_CPR_F;
//abs_encoder_update(&encoder, Encoder.raw);
//    abs_encoder_init(&encoder, init_raw);  // 假设开机时读取的是100
		abs_raw = abs_encoder_get_final_angle_rad(&encoder);
				abs_raw = Real_Angle;
//		abs_round = abs_encoder_get_total_turns(&encoder);
}
