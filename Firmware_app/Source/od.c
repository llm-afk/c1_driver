#include "od.h"
#include "util.h"  
#include "encoder.h"
#include "eeprom_emul.h"
#include "com_can.h"
#include "version.h"


typedef struct {
    uint16_t index;
    void *obj;
    uint8_t datasize;
    uint8_t attribute;
    int (*update_func)(void);
} OD_entry_t;

ODObjs_t ODObjs;
static uint16_t ODObjsCount;
uint16_t g_current_sdo_index = 0;

static eBranchType Parse_Branch_From_SN(void);
static int SN_update_callback(void);

static const OD_entry_t ODList[] = {
    {0x2000, &ODObjs.error_code,                2, ATTR_RAM | ATTR_R,  NULL},
    {0x2001, &ODObjs.status_word,               2, ATTR_RAM | ATTR_R,  NULL},
    {0x2002, &ODObjs.control_word,              2, ATTR_RAM | ATTR_RW, MC_controlword_update},
    {0x2003, &ODObjs.operation_mode,            1, ATTR_RAM | ATTR_RW, NULL},
    
    {0x2004, &ODObjs.sn_s0,                     4, ATTR_ROM | ATTR_RW,  SN_update_callback},
    {0x2005, &ODObjs.sn_s1,                     4, ATTR_ROM | ATTR_RW,  SN_update_callback},
    {0x2006, &ODObjs.sn_s2,                     4, ATTR_ROM | ATTR_RW,  SN_update_callback},
    {0x2007, &ODObjs.sn_s3,                     4, ATTR_ROM | ATTR_RW,  SN_update_callback},
    {0x2008, &ODObjs.sn_s4,                     4, ATTR_ROM | ATTR_RW,  SN_update_callback},
    {0x2009, &ODObjs.sn_s5,                     4, ATTR_ROM | ATTR_RW,  SN_update_callback},
    {0x200A, &ODObjs.sn_s6,                     4, ATTR_ROM | ATTR_RW,  SN_update_callback},
    
    {0x2010, &ODObjs.target_position,           4, ATTR_RAM | ATTR_RW, MC_position_update},
    {0x2011, &ODObjs.target_velocity,           4, ATTR_RAM | ATTR_RW, MC_velocity_update},
    {0x2012, &ODObjs.target_torque,             4, ATTR_RAM | ATTR_RW, MC_torque_update},
    
    {0x2020, &ODObjs.actual_position,           4, ATTR_RAM | ATTR_R,  NULL},
    {0x2021, &ODObjs.actual_velocity,           4, ATTR_RAM | ATTR_R,  NULL},
    {0x2022, &ODObjs.actual_torque,             4, ATTR_RAM | ATTR_R,  NULL},
    {0x2023, &ODObjs.actual_following_error,    4, ATTR_RAM | ATTR_R,  NULL},
    {0x2024, &ODObjs.dc_link_voltage,           4, ATTR_RAM | ATTR_R,  NULL},
    {0x2025, &ODObjs.dc_link_current,           4, ATTR_RAM | ATTR_R,  NULL},
    {0x2026, &ODObjs.electrical_power,          4, ATTR_RAM | ATTR_R,  NULL},
    {0x2027, &ODObjs.mechanical_power,          4, ATTR_RAM | ATTR_R,  NULL},
    {0x2028, &ODObjs.drv_temperature,           4, ATTR_RAM | ATTR_R,  NULL},
    {0x2029, &ODObjs.motor_temperature,         4, ATTR_RAM | ATTR_R,  NULL},
    {0x202A, &ODObjs.in_encoder_value,          4, ATTR_RAM | ATTR_R,  NULL},
    {0x202B, &ODObjs.ex_encoder_value,          4, ATTR_RAM | ATTR_R,  NULL},
    
    {0x2040, &ODObjs.node_id,                   1, ATTR_ROM | ATTR_RW, NULL},    // Restart
    {0x2041, &ODObjs.can_baudrate,              1, ATTR_ROM | ATTR_RW, NULL},    // Restart
    {0x2042, &ODObjs.data_baudrate,             1, ATTR_ROM | ATTR_RW, NULL},    // Restart
    {0x2043, &ODObjs.heartbeat_producer_time,   2, ATTR_ROM | ATTR_RW, NULL},    // Restart
    {0x2044, &ODObjs.heartbeat_consumer_time,   2, ATTR_ROM | ATTR_RW, NULL},    // Restart

    {0x2050, &ODObjs.motor_pp,                  2, ATTR_ROM | ATTR_RW, NULL},
    {0x2051, &ODObjs.motor_r,                   4, ATTR_ROM | ATTR_RW, MC_ctrl_param_update},
    {0x2052, &ODObjs.motor_l_d,                 4, ATTR_ROM | ATTR_RW, MC_ctrl_param_update},
    {0x2053, &ODObjs.motor_l_q,                 4, ATTR_ROM | ATTR_RW, MC_ctrl_param_update},
    {0x2054, &ODObjs.motor_rated_vel,           4, ATTR_ROM | ATTR_RW, NULL},
    {0x2055, &ODObjs.motor_rated_current,       4, ATTR_ROM | ATTR_RW, NULL},
    {0x2056, &ODObjs.motor_torque_constant,     4, ATTR_ROM | ATTR_RW, NULL},
    {0x2057, &ODObjs.motor_inertia,             4, ATTR_ROM | ATTR_RW, MC_ctrl_param_update},
    {0x2058, &ODObjs.polarity,                  1, ATTR_ROM | ATTR_RW, NULL},
    {0x2059, &ODObjs.elec_gear,                 4, ATTR_ROM | ATTR_RW, NULL},    // Re operation
    {0x205A, &ODObjs.load_inertia,              4, ATTR_ROM | ATTR_RW, MC_ctrl_param_update},
    {0x205B, &ODObjs.torque_limit,              4, ATTR_ROM | ATTR_RW, NULL},
    {0x205C, &ODObjs.over_current_level,        4, ATTR_ROM | ATTR_RW, NULL},
    {0x205D, &ODObjs.over_load_dpp_level,       4, ATTR_ROM | ATTR_RW, NULL},
    {0x205E, &ODObjs.over_voltage_level,        4, ATTR_ROM | ATTR_RW, NULL},
    {0x205F, &ODObjs.under_voltage_level,       4, ATTR_ROM | ATTR_RW, NULL},
    {0x2060, &ODObjs.over_temp_drv_level,       4, ATTR_ROM | ATTR_RW, NULL},
    {0x2061, &ODObjs.over_temp_motor_level,     4, ATTR_ROM | ATTR_RW, NULL},
    {0x2062, &ODObjs.position_window,           4, ATTR_ROM | ATTR_RW, NULL},
    {0x2063, &ODObjs.position_window_time,      2, ATTR_ROM | ATTR_RW, NULL},
    {0x2064, &ODObjs.velocity_window,           4, ATTR_ROM | ATTR_RW, NULL},
    {0x2065, &ODObjs.velocity_window_time,      2, ATTR_ROM | ATTR_RW, NULL},
    {0x2066, &ODObjs.velocity_threshold,        4, ATTR_ROM | ATTR_RW, NULL},
    {0x2067, &ODObjs.velocity_threshold_time,   2, ATTR_ROM | ATTR_RW, NULL},    
    {0x2068, &ODObjs.following_error_window,    4, ATTR_ROM | ATTR_RW, NULL},
    {0x2069, &ODObjs.following_error_time,      2, ATTR_ROM | ATTR_RW, NULL},
    {0x206A, &ODObjs.brake_ctrl,                1, ATTR_ROM | ATTR_RW, NULL},
    
    {0x2070, &ODObjs.in_encoder_offset,        2, ATTR_ROM | ATTR_RW, MC_ctrl_param_update},
    {0x2071, &ODObjs.ex_encoder_offset,        2, ATTR_ROM | ATTR_RW, MC_ctrl_param_update},
    
    {0x2080, &ODObjs.profile_velocity,          4, ATTR_ROM | ATTR_RW, MC_profile_update},
    {0x2081, &ODObjs.profile_acceleration,      4, ATTR_ROM | ATTR_RW, MC_profile_update},
    {0x2082, &ODObjs.profile_deceleration,      4, ATTR_ROM | ATTR_RW, MC_profile_update},
    {0x2083, &ODObjs.profile_torque_slope,      4, ATTR_ROM | ATTR_RW, MC_profile_update},
    
    {0x2090, &ODObjs.home_offset,               4, ATTR_ROM | ATTR_RW, NULL},
    {0x2100, &ODObjs.firmware_version,          2, ATTR_RAM | ATTR_R,  NULL},
    {0x2101, &ODObjs.restore_default,           1, ATTR_RAM | ATTR_W,  OD_restore_defalt},
    {0x2102, &ODObjs.plot_ctrl,                 1, ATTR_RAM | ATTR_W,  NULL},
};

static void dictionary_init(void)
{
    ODObjs.error_code = 0;
    ODObjs.status_word = 0;
    ODObjs.control_word = 0;
    ODObjs.operation_mode = 0;
    
    ODObjs.target_position = 0.0f;
    ODObjs.target_velocity = 0.0f;
    ODObjs.target_torque = 0.0f;
    
    ODObjs.actual_position = 0.0f;
    ODObjs.actual_velocity = 0.0f;
    ODObjs.actual_torque = 0.0f;
    ODObjs.actual_following_error = 0.0f;
    ODObjs.dc_link_voltage = 0.0f;
    ODObjs.dc_link_current = 0.0f;
    ODObjs.electrical_power = 0.0f;
    ODObjs.mechanical_power = 0.0f;
    ODObjs.drv_temperature = 0.0f;
    ODObjs.motor_temperature = 0.0f;
    ODObjs.in_encoder_value = 0.0f;
    ODObjs.ex_encoder_value = 0.0f;
    
    ODObjs.node_id = 1;
    ODObjs.can_baudrate = 2;
    ODObjs.data_baudrate = 2;
    ODObjs.heartbeat_producer_time = 0;
    ODObjs.heartbeat_consumer_time = 0;
    
    ODObjs.sn_s0 = 0;
    ODObjs.sn_s1 = 0;
    ODObjs.sn_s2 = 0;
    ODObjs.sn_s3 = 0;
    ODObjs.sn_s4 = 0;
    ODObjs.sn_s5 = 0;
    ODObjs.sn_s6 = 0;
    
    ODObjs.motor_rated_vel = 30.0f;
    ODObjs.motor_rated_current = 160.0f;
    ODObjs.motor_torque_constant = 1.00f;
    ODObjs.motor_inertia = 0.000007f;
    ODObjs.elec_gear = ENCODER_CPR_F;
    ODObjs.load_inertia = 0.0f;
    ODObjs.over_current_level = 160.0f;
    ODObjs.over_load_dpp_level = 99999999.0f;
    ODObjs.over_voltage_level = 40.0f;
    ODObjs.under_voltage_level = 18.0f;
    ODObjs.over_temp_drv_level = 85.0f;
    ODObjs.over_temp_motor_level = 150.0f;
    ODObjs.position_window = 0.01f;
    ODObjs.position_window_time = 100;
    ODObjs.velocity_window = 1.0f;
    ODObjs.velocity_window_time = 100;
    ODObjs.velocity_threshold = 0.1f;
    ODObjs.velocity_threshold_time = 100;
    ODObjs.following_error_window = 0.01f;
    ODObjs.following_error_time = 1000;
    ODObjs.brake_ctrl = 0;
    
    ODObjs.velocity_ctrl_gain = 9500;
    ODObjs.position_ctrl_gain = 1000;
    
    ODObjs.profile_velocity = 10.0f;
    ODObjs.profile_acceleration = 10.0f;
    ODObjs.profile_deceleration = 10.0f;
    ODObjs.profile_torque_slope = 0.1f;
    
    ODObjs.home_offset = 0;
    
    ODObjs.firmware_version = APP_VERSION;
    ODObjs.restore_default = 0;
    ODObjs.plot_ctrl = 0;
}

OD_entry_t *find_entry(uint16_t index)
{
    uint16_t min = 0;
    uint16_t max = ODObjsCount - 1;

    while (min <= max) {
        uint16_t cur = (min + max) >> 1;
        OD_entry_t* entry = (OD_entry_t*)&ODList[cur];

        if (index == entry->index) {
            return entry;
        }

        if (index < entry->index) {
            max = (cur > 0) ? (cur - 1) : cur;
        } else {
            min = cur + 1;
        }
    }

    if (min == max) {
        OD_entry_t* entry = (OD_entry_t*)&ODList[min];
        if (index == entry->index) {
            return entry;
        }
    }

    return NULL;
}

// 解析26位SN码，提取硬件版本段（第13-16字节）的B位和C位
static eBranchType Parse_Branch_From_SN(void)
{
    // 根据26位编码格式：
    // 第1段(2) + 第2段(4) + 第3段(4) + 第4段(2) = 前12个字节，放在 sn_s0, sn_s1, sn_s2 中
    // 第5段 硬件版本(4位ABCD)：放在 sn_s3 中
    // 因为 GD32 是小端序(Little Endian)，所以内存存放顺序如下：
    // sn_s3 的 [7:0]   是 A位
    // sn_s3 的 [15:8]  是 B位
    // sn_s3 的 [23:16] 是 C位
    // sn_s3 的 [31:24] 是 D位
    
    uint8_t b_bit_platform = (ODObjs.sn_s3 >> 8) & 0xFF;  // B位: 芯片平台
    uint8_t c_bit_motor    = (ODObjs.sn_s3 >> 16) & 0xFF; // C位: 电机型号
    
    if(b_bit_platform != '1') return BRANCH_UNKNOWN; // 仅支持 GD平台 
    
    // 根据 C 位解析对应的电机型号和减速比硬件分支
    switch (c_bit_motor) 
    {
        case '1': return BRANCH_C2_NEW;        
        case '3': return BRANCH_C2_PRO;      
        case '4': return BRANCH_C2_PRO_XINZHI;  
        case '5': return BRANCH_A2;    
        case '6': return BRANCH_A2_XINZHI;  
        case '7': return BRANCH_A2_JIEKE_WHEEL;  
        default:  return BRANCH_UNKNOWN;  
    }
}

/**
 * @brief 如果没有写入过sn码，报错始终存在，这样无法使能电机
 */
extern uint8_t  g_need_reboot;
extern uint32_t g_reboot_tick;

static int SN_update_callback(void)
{
    // 只在写完最后一段 SN (sn_s6) 时触发软复位重启 
    if (g_current_sdo_index == 0x200A)
    {
        g_need_reboot = 1;
        g_reboot_tick = get_tick();
    }
    return 0;
}

void OD_check_sn(void)
{
    eBranchType determined_branch = Parse_Branch_From_SN();
    
    if(determined_branch == BRANCH_UNKNOWN) 
    {
        // If unknown or empty SN, lock the motor system by raising a NO_SN error
        ERROR_SET(ERR_NO_SN);
    }
    else 
    {
        // If valid SN, clear the error and initialize the hardware configurations
        ERROR_CLR(ERR_NO_SN);
        HW_Config_Init(determined_branch);
    }
}

void OD_init(void)
{
    ODObjsCount = sizeof(ODList) / sizeof(OD_entry_t);
    
    dictionary_init();

    EE_Init(EE_FORCED_ERASE);

    for(int i=0; i<ODObjsCount; i++){
        if(ODList[i].attribute & ATTR_ROM){
            switch(ODList[i].datasize){
                case 1:
                    EE_ReadVariable8bits(ODList[i].index, (uint8_t*)ODList[i].obj);
                    break;
                case 2:
                    EE_ReadVariable16bits(ODList[i].index, (uint16_t*)ODList[i].obj);
                    break;
                case 4:
                    EE_ReadVariable32bits(ODList[i].index, (uint32_t*)ODList[i].obj);
                    break;
                default:
                    break;
            }
        }
    }
    
    // ---------------------------------------------------------
    // Parse the SN code from EEPROM and apply hardware branch
    // ---------------------------------------------------------
    OD_check_sn();
}

int OD_restore_defalt(void)
{
    if(ODObjs.restore_default == 0xEE){
        // Stop motor
        if(MC_get_state() != MCS_IDLE){
            MC_set_state(MCS_IDLE);
        }
        
        dictionary_init();
        EE_Format(EE_FORCED_ERASE);
        OD_check_sn();

        return 0;
    }
    
    return -1;
}

uint8_t OD_read(uint16_t idx, uint8_t *data)
{
    uint8_t cs = CS_ERR;

    for(int i=0; i<4; i++){
        data[i] = 0;
    }

    // get entry
    OD_entry_t *entry = find_entry(idx);

    if(entry != NULL && entry->attribute & ATTR_R){
        switch(entry->datasize){
            case 1:
                data[0] = *((uint8_t*)entry->obj + 0);
                cs = CS_R_ACK_1;
                break;
            
            case 2:
                data[0] = *((uint8_t*)entry->obj + 0);
                data[1] = *((uint8_t*)entry->obj + 1);
                cs = CS_R_ACK_2;
                break;
            
            case 3:
                data[0] = *((uint8_t*)entry->obj + 0);
                data[1] = *((uint8_t*)entry->obj + 1);
                data[2] = *((uint8_t*)entry->obj + 2);
                cs = CS_R_ACK_3;
                break;
            
            case 4:
                data[0] = *((uint8_t*)entry->obj + 0);
                data[1] = *((uint8_t*)entry->obj + 1);
                data[2] = *((uint8_t*)entry->obj + 2);
                data[3] = *((uint8_t*)entry->obj + 3);
                cs = CS_R_ACK_4;
                break;
        }
    }
    
    return cs;
}

uint8_t OD_write_1(uint16_t idx, uint8_t *data)
{
    uint8_t cs = CS_ERR;
    
    // get entry
    OD_entry_t *entry = find_entry(idx);
    
    if(entry != NULL && entry->attribute & ATTR_W && entry->datasize == 1){
        if(*(uint8_t*)entry->obj != *(uint8_t*)data){
            *(uint8_t*)entry->obj = *(uint8_t*)data;
            if(entry->attribute & ATTR_ROM){
                EE_Status ee_status = EE_WriteVariable8bits(entry->index, *(uint8_t*)entry->obj);
                if((ee_status & EE_STATUSMASK_ERROR) == EE_OK){
                    cs = CS_W_ACK;
                }
                if((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP){
                    EE_CleanUp();
                }
            }else{
                cs = CS_W_ACK;
            }
        }else{
            cs = CS_W_ACK;
        }
    }
    
    if(cs == CS_W_ACK && entry->update_func != NULL){
        if(0 != entry->update_func()){
            cs = CS_ERR;
        }
    }
    
    for(int i=0; i<4; i++){
        data[i] = 0;
    }
    
    return cs;
}
uint8_t flag_zero[2] = {0};
uint8_t OD_write_2(uint16_t idx, uint8_t *data)
{
    uint8_t cs = CS_ERR;
    
    // get entry
    OD_entry_t *entry = find_entry(idx);
		if(idx == 0x2070){
			*(uint16_t*)data = Encoder.raw;
			flag_zero[0] = 1;
			
			
			 if(entry != NULL && entry->attribute & ATTR_W && entry->datasize == 2){
        if(*(uint16_t*)entry->obj != *(uint16_t*)data){
            *(uint16_t*)entry->obj = *(uint16_t*)data;
            if(entry->attribute & ATTR_ROM){
                EE_Status ee_status = EE_WriteVariable16bits(entry->index, *(uint16_t*)entry->obj);
                if((ee_status & EE_STATUSMASK_ERROR) == EE_OK){
                    cs = CS_W_ACK;
                }
                if((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP){
                    EE_CleanUp();
                }
            }else{
                cs = CS_W_ACK;
            }
        }else{
            cs = CS_W_ACK;
        }
			}

			if(cs == CS_W_ACK && entry->update_func != NULL){
					if(0 != entry->update_func()){
							cs = CS_ERR;
					}
			}
			
			for(int i=0; i<4; i++){
					data[i] = 0;
			}
			
			idx = 0x2071;
			OD_entry_t *entry_2071 = find_entry(idx);
			*(uint16_t*)data = EX_ENCODER_VALUE;

			if(entry_2071 != NULL && entry_2071->attribute & ATTR_W && entry_2071->datasize == 2){
        if(*(uint16_t*)entry_2071->obj != *(uint16_t*)data){
            *(uint16_t*)entry_2071->obj = *(uint16_t*)data;
            if(entry_2071->attribute & ATTR_ROM){
                EE_Status ee_status = EE_WriteVariable16bits(entry_2071->index, *(uint16_t*)entry_2071->obj);
                if((ee_status & EE_STATUSMASK_ERROR) == EE_OK){
                    cs = CS_W_ACK;
                }
                if((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP){
                    EE_CleanUp();
                }
            }else{
                cs = CS_W_ACK;
            }
        }else{
            cs = CS_W_ACK;
        }
			}

			if(cs == CS_W_ACK && entry_2071->update_func != NULL){
					if(0 != entry_2071->update_func()){
							cs = CS_ERR;
					}
			}
			
			for(int i=0; i<4; i++){
					data[i] = 0;
			}
						flag_zero[1] = 1;

			return cs;

		}
//		if(idx == 0x2071){
//			*(uint16_t*)data = EX_ENCODER_VALUE;
//			flag_zero[1] = 1;

//		}		
    if(entry != NULL && entry->attribute & ATTR_W && entry->datasize == 2){
        if(*(uint16_t*)entry->obj != *(uint16_t*)data){
            *(uint16_t*)entry->obj = *(uint16_t*)data;
            if(entry->attribute & ATTR_ROM){
                EE_Status ee_status = EE_WriteVariable16bits(entry->index, *(uint16_t*)entry->obj);
                if((ee_status & EE_STATUSMASK_ERROR) == EE_OK){
                    cs = CS_W_ACK;
                }
                if((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP){
                    EE_CleanUp();
                }
            }else{
                cs = CS_W_ACK;
            }
        }else{
            cs = CS_W_ACK;
        }
    }

    if(cs == CS_W_ACK && entry->update_func != NULL){
        if(0 != entry->update_func()){
            cs = CS_ERR;
        }
    }
    
    for(int i=0; i<4; i++){
        data[i] = 0;
    }
    
    return cs;
}

uint8_t OD_write_4(uint16_t idx, uint8_t *data)
{
    uint8_t cs = CS_ERR;
    
    // get entry
    OD_entry_t *entry = find_entry(idx);
    
    if(entry != NULL && entry->attribute & ATTR_W && entry->datasize == 4){
        if(*(uint32_t*)entry->obj != *(uint32_t*)data){
            *(uint32_t*)entry->obj = *(uint32_t*)data;
            if(entry->attribute & ATTR_ROM){
                EE_Status ee_status = EE_WriteVariable32bits(entry->index, *(uint32_t*)entry->obj);
                if((ee_status & EE_STATUSMASK_ERROR) == EE_OK){
                    cs = CS_W_ACK;
                }
                if((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP){
                    EE_CleanUp();
                }
            }else{
                cs = CS_W_ACK;
            }
        }else{
            cs = CS_W_ACK;
        }
    }
    
    if(cs == CS_W_ACK && entry->update_func != NULL){
        g_current_sdo_index = entry->index;
        if(0 != entry->update_func()){
            cs = CS_ERR;
        }
        g_current_sdo_index = 0;
    }
    
    for(int i=0; i<4; i++){
        data[i] = 0;
    }
    
    return cs;    
}


// Global parameters that differ across branches
float g_i_scale = 15.0f;
float g_encoder_calib_current = 2.0f;
float g_gear_ratio = 12.0f;
int g_multi_pri_gear = 17;
int g_multi_sec_gear = 18;
const tTorqueCalibPoint* g_torque_calib_table = NULL;
uint16_t g_torque_calib_table_len = 0;


// Torque calibration tables for different branches
static const tTorqueCalibPoint table_c2_new[] = {
    {0.0f, 0.0f},
    {100.0f, 100.0f} // Placeholder
};

static const tTorqueCalibPoint table_c2_pro[] = {
    {0.0f, 0.0f},
    {100.0f, 100.0f} // Placeholder
};

static const tTorqueCalibPoint table_c2_pro_xinzhi[] = {
    // {0.0f, 0.0f},
    // {100.0f, 100.0f} // Placeholder
    {0.0f, 0.0000f},
    {0.1f, 0.0016f},
    {0.2f, 0.0125f},
    {0.3f, 0.0422f},
    {0.4f, 0.1000f},
    {0.5f, 0.2381f},
    {0.6f, 0.4267f},
    {0.7f, 0.5900f},
    {0.8f, 0.7534f},
    {0.9f, 0.9167f},
    {1.0f, 1.0800f},
    {1.1f, 1.2434f},
    {1.2f, 1.4067f},
    {1.3f, 1.5700f},
    {1.4f, 1.7334f},
    {1.5f, 1.8967f},
    {1.6f, 2.0601f},
    {1.7f, 2.2234f},
    {1.8f, 2.3867f},
    {1.9f, 2.5501f},
    {2.0f, 2.7134f},
    {2.1f, 2.8768f},
    {2.2f, 3.0401f},
    {2.3f, 3.2034f},
    {2.4f, 3.3668f},
    {2.5f, 3.5301f},
    {2.6f, 3.6935f},
    {2.7f, 3.8568f},
    {2.8f, 4.0201f},
    {2.9f, 4.1835f},
    {3.0f, 4.3468f},
    {3.1f, 4.5101f},
    {3.2f, 4.6735f},
    {3.3f, 4.8368f},
    {3.4f, 5.0002f},
    {3.5f, 5.1635f},
    {3.6f, 5.3268f},
    {3.7f, 5.4902f},
    {3.8f, 5.6535f},
    {3.9f, 5.8169f},
    {4.0f, 5.9802f},
    {4.1f, 6.1435f},
    {4.2f, 6.3069f},
    {4.3f, 6.4702f},
    {4.4f, 6.6336f},
    {4.5f, 6.7969f},
    {4.6f, 6.9602f},
    {4.7f, 7.1236f},
    {4.8f, 7.2869f},
    {4.9f, 7.4502f},
    {5.0f, 7.6136f},
    {5.5f, 8.4303f},
    {6.0f, 9.2470f},
    {6.5f, 10.0637f},
    {7.0f, 10.8804f},
    {7.5f, 11.6971f},
    {8.0f, 12.5138f},
    {8.5f, 13.3304f},
    {9.0f, 14.1471f},
    {9.5f, 14.9638f},
    {10.0f, 15.7805f},
    {10.5f, 16.5972f},
    {11.0f, 17.4139f},
    {11.5f, 18.2306f},
    {12.0f, 19.0473f},
    {12.5f, 19.8640f},
    {13.0f, 20.6807f},
    {13.5f, 21.4974f},
    {14.0f, 22.3141f},
    {14.5f, 23.1308f},
    {15.0f, 23.9475f},
    {15.5f, 24.7642f},
    {16.0f, 25.5809f},
    {16.5f, 26.3976f},
    {17.0f, 27.2142f},
    {17.5f, 28.0309f},
    {18.0f, 28.8476f},
    {18.5f, 29.6643f},
    {19.0f, 30.4810f},
    {19.5f, 31.2977f},
    {20.0f, 32.1144f},
    {20.5f, 32.9311f},
    {21.0f, 33.7478f},
    {21.5f, 34.5645f},
    {22.0f, 35.3812f},
    {22.5f, 36.1979f},
    {23.0f, 37.0146f},
    {23.5f, 37.8313f},
    {24.0f, 38.6480f},
    {24.5f, 39.4647f},
    {25.0f, 40.2814f},
    {25.5f, 41.0980f},
    {26.0f, 41.9147f},
    {26.5f, 42.7314f},
    {27.0f, 43.5481f},
    {27.5f, 44.3648f},
    {28.0f, 45.1815f},
    {28.5f, 45.9982f},
    {29.0f, 46.8149f},
    {29.5f, 47.6316f},
    {30.0f, 48.4483f},
    {30.5f, 49.2646f},
    {31.0f, 50.0789f},
    {31.5f, 50.8889f},
    {32.0f, 51.6926f},
    {32.5f, 52.4878f},
    {33.0f, 53.2725f},
    {33.5f, 54.0445f},
    {34.0f, 54.8018f},
    {34.5f, 55.5421f},
    {35.0f, 56.2635f},
    {35.5f, 56.9644f},
    {36.0f, 57.6461f},
    {36.5f, 58.3104f},
    {37.0f, 58.9591f},
    {37.5f, 59.5941f},
    {38.0f, 60.2173f},
};

static const tTorqueCalibPoint table_a2[] = {
    {0.0f, 0.0f},
    {100.0f, 100.0f} // Placeholder
};

static const tTorqueCalibPoint table_a2_xinzhi[] = {
    {0.0f, 0.0000f},
    {0.1f, 0.0016f},
    {0.2f, 0.0125f},
    {0.3f, 0.0422f},
    {0.4f, 0.1000f},
    {0.5f, 0.2381f},
    {0.6f, 0.4267f},
    {0.7f, 0.5900f},
    {0.8f, 0.7534f},
    {0.9f, 0.9167f},
    {1.0f, 1.0800f},
    {1.1f, 1.2434f},
    {1.2f, 1.4067f},
    {1.3f, 1.5700f},
    {1.4f, 1.7334f},
    {1.5f, 1.8967f},
    {1.6f, 2.0601f},
    {1.7f, 2.2234f},
    {1.8f, 2.3867f},
    {1.9f, 2.5501f},
    {2.0f, 2.7134f},
    {2.1f, 2.8768f},
    {2.2f, 3.0401f},
    {2.3f, 3.2034f},
    {2.4f, 3.3668f},
    {2.5f, 3.5301f},
    {2.6f, 3.6935f},
    {2.7f, 3.8568f},
    {2.8f, 4.0201f},
    {2.9f, 4.1835f},
    {3.0f, 4.3468f},
    {3.1f, 4.5101f},
    {3.2f, 4.6735f},
    {3.3f, 4.8368f},
    {3.4f, 5.0002f},
    {3.5f, 5.1635f},
    {3.6f, 5.3268f},
    {3.7f, 5.4902f},
    {3.8f, 5.6535f},
    {3.9f, 5.8169f},
    {4.0f, 5.9802f},
    {4.1f, 6.1435f},
    {4.2f, 6.3069f},
    {4.3f, 6.4702f},
    {4.4f, 6.6336f},
    {4.5f, 6.7969f},
    {4.6f, 6.9602f},
    {4.7f, 7.1236f},
    {4.8f, 7.2869f},
    {4.9f, 7.4502f},
    {5.0f, 7.6136f},
    {5.5f, 8.4303f},
    {6.0f, 9.2470f},
    {6.5f, 10.0637f},
    {7.0f, 10.8804f},
    {7.5f, 11.6971f},
    {8.0f, 12.5138f},
    {8.5f, 13.3304f},
    {9.0f, 14.1471f},
    {9.5f, 14.9638f},
    {10.0f, 15.7805f},
    {10.5f, 16.5972f},
    {11.0f, 17.4139f},
    {11.5f, 18.2306f},
    {12.0f, 19.0473f},
    {12.5f, 19.8640f},
    {13.0f, 20.6807f},
    {13.5f, 21.4974f},
    {14.0f, 22.3141f},
    {14.5f, 23.1308f},
    {15.0f, 23.9475f},
    {15.5f, 24.7642f},
    {16.0f, 25.5809f},
    {16.5f, 26.3976f},
    {17.0f, 27.2142f},
    {17.5f, 28.0309f},
    {18.0f, 28.8476f},
    {18.5f, 29.6643f},
    {19.0f, 30.4810f},
    {19.5f, 31.2977f},
    {20.0f, 32.1144f},
    {20.5f, 32.9311f},
    {21.0f, 33.7478f},
    {21.5f, 34.5645f},
    {22.0f, 35.3812f},
    {22.5f, 36.1979f},
    {23.0f, 37.0146f},
    {23.5f, 37.8313f},
    {24.0f, 38.6480f},
    {24.5f, 39.4647f},
    {25.0f, 40.2814f},
    {25.5f, 41.0980f},
    {26.0f, 41.9147f},
    {26.5f, 42.7314f},
    {27.0f, 43.5481f},
    {27.5f, 44.3648f},
    {28.0f, 45.1815f},
    {28.5f, 45.9982f},
    {29.0f, 46.8149f},
    {29.5f, 47.6316f},
    {30.0f, 48.4483f},
    {30.5f, 49.2646f},
    {31.0f, 50.0789f},
    {31.5f, 50.8889f},
    {32.0f, 51.6926f},
    {32.5f, 52.4878f},
    {33.0f, 53.2725f},
    {33.5f, 54.0445f},
    {34.0f, 54.8018f},
    {34.5f, 55.5421f},
    {35.0f, 56.2635f},
    {35.5f, 56.9644f},
    {36.0f, 57.6461f},
    {36.5f, 58.3104f},
    {37.0f, 58.9591f},
    {37.5f, 59.5941f},
    {38.0f, 60.2173f},
    {38.5f, 60.8305f},
    {39.0f, 61.4355f},
    {39.5f, 62.0342f},
    {40.0f, 62.6285f},
    {40.5f, 63.2199f},
    {41.0f, 63.8080f},
    {41.5f, 64.3924f},
    {42.0f, 64.9723f},
    {42.5f, 65.5474f},
    {43.0f, 66.1168f},
    {43.5f, 66.6801f},
    {44.0f, 67.2367f},
    {44.5f, 67.7859f},
    {45.0f, 68.3271f},
    {45.5f, 68.8598f},
    {46.0f, 69.3834f},
    {46.5f, 69.8973f},
    {47.0f, 70.4008f},
    {47.5f, 70.8935f},
    {48.0f, 71.3746f},
    {48.5f, 71.8436f},
    {49.0f, 72.2999f},
    {49.5f, 72.7429f},
    {50.0f, 73.1720f},
    {50.5f, 73.5870f},
    {51.0f, 73.9890f},
    {51.5f, 74.3793f},
    {52.0f, 74.7594f},
    {52.5f, 75.1308f},
    {53.0f, 75.4950f},
    {53.5f, 75.8532f},
    {54.0f, 76.2071f},
    {54.5f, 76.5580f},
    {55.0f, 76.9073f},
    {55.5f, 77.2565f},
    {56.0f, 77.6071f},
    {56.5f, 77.9604f},
    {57.0f, 78.3180f},
    {57.5f, 78.6812f},
    {58.0f, 79.0515f},
    {58.5f, 79.4303f},
    {59.0f, 79.8191f},
    {59.5f, 80.2193f},
    {60.0f, 80.6324f},
    {60.5f, 81.0591f},
    {61.0f, 81.4983f},
    {61.5f, 81.9481f},
    {62.0f, 82.4065f},
    {62.5f, 82.8718f},
    {63.0f, 83.3420f},
    {63.5f, 83.8154f},
    {64.0f, 84.2900f},
    {64.5f, 84.7639f},
    {65.0f, 85.2354f},
    {65.5f, 85.7025f},
    {66.0f, 86.1635f},
    {66.5f, 86.6163f},
    {67.0f, 87.0593f},
    {67.5f, 87.4904f},
    {68.0f, 87.9078f},
    {68.5f, 88.3098f},
    {69.0f, 88.6943f},
    {69.5f, 89.0596f},
    {70.0f, 89.4038f},
    {70.5f, 89.7257f},
    {71.0f, 90.0265f},
    {71.5f, 90.3083f},
    {72.0f, 90.5730f},
    {72.5f, 90.8225f},
    {73.0f, 91.0589f},
    {73.5f, 91.2841f},
    {74.0f, 91.5000f},
    {74.5f, 91.7086f},
    {75.0f, 91.9118f},
    {75.5f, 92.1117f},
    {76.0f, 92.3102f},
    {76.5f, 92.5092f},
    {77.0f, 92.7106f},
    {77.5f, 92.9166f},
    {78.0f, 93.1290f},
    {78.5f, 93.3497f},
    {79.0f, 93.5808f},
    {79.5f, 93.8241f},
    {80.0f, 94.0818f},
    {80.5f, 94.3550f},
    {81.0f, 94.6424f},
    {81.5f, 94.9421f},
    {82.0f, 95.2520f},
    {82.5f, 95.5703f},
    {83.0f, 95.8950f},
    {83.5f, 96.2240f},
    {84.0f, 96.5554f},
    {84.5f, 96.8872f},
    {85.0f, 97.2174f},
    {85.5f, 97.5442f},
    {86.0f, 97.8654f},
    {86.5f, 98.1791f},
    {87.0f, 98.4834f},
    {87.5f, 98.7763f},
    {88.0f, 99.0557f},
    {88.5f, 99.3198f},
    {89.0f, 99.5665f},
    {89.5f, 99.7939f},
    {90.0f, 100.0000f},
};

static const tTorqueCalibPoint table_a2_jieke_wheel[] = {
    {0.0f, 0.0000f},
    {0.2f, 0.0267f},
    {0.4f, 0.0648f},
    {0.6f, 0.1000f},
    {0.8f, 0.1598f},
    {1.0f, 0.2195f},
    {1.2f, 0.2986f},
    {1.4f, 0.3812f},
    {1.6f, 0.4678f},
    {1.8f, 0.5620f},
    {2.0f, 0.6561f},
    {2.2f, 0.7606f},
    {2.4f, 0.8668f},
    {2.6f, 0.9741f},
    {2.8f, 1.0834f},
    {3.0f, 1.1928f},
    {3.2f, 1.2985f},
    {3.4f, 1.4036f},
    {3.6f, 1.5092f},
    {3.8f, 1.6161f},
    {4.0f, 1.7229f},
    {4.2f, 1.8317f},
    {4.4f, 1.9409f},
    {4.6f, 2.0491f},
    {4.8f, 2.1554f},
    {5.0f, 2.2616f},
    {6.0f, 2.8280f},
    {7.0f, 3.4058f},
    {8.0f, 3.9534f},
    {9.0f, 4.4962f},
    {10.0f, 5.0381f},
    {11.0f, 5.5398f},
    {12.0f, 6.0247f},
    {13.0f, 6.5113f},
    {14.0f, 6.9847f},
    {15.0f, 7.4828f},
    {16.0f, 8.0295f},
    {17.0f, 8.5746f},
    {18.0f, 9.0877f},
    {19.0f, 9.5766f},
    {20.0f, 10.0818f},
    {21.0f, 10.6147f},
    {22.0f, 11.1709f},
    {23.0f, 11.7090f},
    {24.0f, 12.1939f},
    {25.0f, 12.6419f},
    {26.0f, 13.0867f},
    {27.0f, 13.5370f},
    {28.0f, 14.0086f},
    {29.0f, 14.4694f},
    {30.0f, 14.9345f},
    {31.0f, 15.4063f},
    {32.0f, 15.8963f},
    {33.0f, 16.3752f},
    {34.0f, 16.8229f},
    {35.0f, 17.2568f},
    {36.0f, 17.6932f},
    {37.0f, 18.1356f},
    {38.0f, 18.5797f},
    {39.0f, 19.0326f},
    {40.0f, 19.5076f},
    {41.0f, 19.9869f},
    {42.0f, 20.4583f},
    {43.0f, 20.9192f},
    {44.0f, 21.3960f},
    {45.0f, 21.8615f},
    {46.0f, 22.3119f},
    {47.0f, 22.7271f},
    {48.0f, 23.1367f},
    {49.0f, 23.5440f},
    {50.0f, 23.9650f},
    {51.0f, 24.3705f},
    {52.0f, 24.8616f},
    {53.0f, 25.3252f},
    {54.0f, 25.8013f},
    {55.0f, 26.2944f},
    {56.0f, 26.7952f},
    {57.0f, 27.3158f},
    {58.0f, 27.8551f},
    {59.0f, 28.3859f},
    {60.0f, 28.9213f},
    {61.0f, 29.4542f},
    {62.0f, 29.9628f},
    {63.0f, 30.4046f},
    {64.0f, 30.7983f},
    {65.0f, 31.1663f},
    {66.0f, 31.5258f},
    {67.0f, 31.8892f},
    {68.0f, 32.2645f},
    {69.0f, 32.6600f},
    {70.0f, 33.0769f},
    {71.0f, 33.5399f},
    {72.0f, 34.0430f},
    {73.0f, 34.5495f},
    {74.0f, 35.0351f},
    {75.0f, 35.4912f},
    {76.0f, 35.9253f},
    {77.0f, 36.3496f},
    {78.0f, 36.7511f},
    {79.0f, 37.1417f},
    {80.0f, 37.5074f},
    {81.0f, 37.8637f},
    {82.0f, 38.2186f},
    {83.0f, 38.4664f},
    {84.0f, 38.7087f},
    {85.0f, 39.0166f},
    {86.0f, 39.3393f},
    {87.0f, 39.4864f},
    {88.0f, 39.6864f},
    {89.0f, 39.8864f},
    {90.0f, 40.0864f}
};

eBranchType g_current_branch;

void HW_Config_Init(eBranchType branch)
{
    g_current_branch = branch;

    switch (branch) 
    {
        case BRANCH_C2_NEW:
        {
            g_i_scale               = 15.0f;
            g_encoder_calib_current = 8.0f;
            g_gear_ratio            = 12.0f;
            g_multi_pri_gear        = 17;
            g_multi_sec_gear        = 18; // ±0.75 圈  4.7124rad
            g_torque_calib_table    = table_c2_new;
            g_torque_calib_table_len = sizeof(table_c2_new) / sizeof(table_c2_new[0]);
            ODObjs.motor_pp   = 8;
            ODObjs.motor_r    = 0.5629f;
            ODObjs.motor_l_d  = 431e-6f;
            ODObjs.motor_l_q  = 431e-6f;
            ODObjs.polarity   = 0;
            ODObjs.over_temp_drv_level   = 85.0f;
            ODObjs.over_temp_motor_level = 150.0f;
            ODObjs.torque_limit          = 30.0f;
            ODObjs.peak_iq_current       = 30.0f;
            break;
        }
        case BRANCH_C2_PRO:
        {
            g_i_scale               = 15.0f;
            g_encoder_calib_current = 8.0f;
            g_gear_ratio            = 25.0f;
            g_multi_pri_gear        = 32;
            g_multi_sec_gear        = 31; // ±0.62 圈  ±3.8936rad
            g_torque_calib_table    = table_c2_pro;
            g_torque_calib_table_len = sizeof(table_c2_pro) / sizeof(table_c2_pro[0]);
            ODObjs.motor_pp   = 8;
            ODObjs.motor_r    = 0.275f;
            ODObjs.motor_l_d  = 160e-6f;
            ODObjs.motor_l_q  = 160e-6f;
            ODObjs.polarity   = 0;
            ODObjs.over_temp_drv_level   = 85.0f;
            ODObjs.over_temp_motor_level = 150.0f;
            ODObjs.torque_limit          = 50.0f;
            ODObjs.peak_iq_current       = 50.0f;
            break;
        }
        case BRANCH_C2_PRO_XINZHI:
        {
            g_i_scale               = 15.0f;
            g_encoder_calib_current = 8.0f;
            g_gear_ratio            = 25.0f;
            g_multi_pri_gear        = 32;
            g_multi_sec_gear        = 31; // ±0.62 圈  ±3.8936rad
            g_torque_calib_table    = table_c2_pro_xinzhi;
            g_torque_calib_table_len = sizeof(table_c2_pro_xinzhi) / sizeof(table_c2_pro_xinzhi[0]);
            ODObjs.motor_pp   = 10;
            ODObjs.motor_r    = 0.125f;
            ODObjs.motor_l_d  = 160e-6f;        
            ODObjs.motor_l_q  = 160e-6f;
            ODObjs.polarity   = 1;
            ODObjs.over_temp_drv_level   = 85.0f;
            ODObjs.over_temp_motor_level = 150.0f;
            ODObjs.torque_limit          = 50.0f;
            ODObjs.peak_iq_current       = 50.0f;
            break;
        }
        case BRANCH_A2:
        {
            g_i_scale               = 13.0f;
            g_encoder_calib_current = 10.0f;
            g_gear_ratio            = 25.0f;
            g_multi_pri_gear        = 32;
            g_multi_sec_gear        = 31; // ±0.62 圈  ±3.8936rad
            g_torque_calib_table    = table_a2;
            g_torque_calib_table_len = sizeof(table_a2) / sizeof(table_a2[0]);
            ODObjs.motor_pp   = 8;
            ODObjs.motor_r    = 0.275f;
            ODObjs.motor_l_d  = 160e-6f;        
            ODObjs.motor_l_q  = 160e-6f;
            ODObjs.polarity   = 0;
            ODObjs.over_temp_drv_level   = 85.0f;
            ODObjs.over_temp_motor_level = 150.0f;
            ODObjs.torque_limit          = 60.0f;
            ODObjs.peak_iq_current       = 38.0f;
            break;
        }
        case BRANCH_A2_XINZHI:
        {
            g_i_scale               = 7.5f; // 10->82.5A  13->63.4A  7.5->110A
            g_encoder_calib_current = 20.0f;
            g_gear_ratio            = 25.0f;
            g_multi_pri_gear        = 32;  
            g_multi_sec_gear        = 31; // ±0.62 圈  ±3.8936rad
            g_torque_calib_table    = table_a2_xinzhi;
            g_torque_calib_table_len = sizeof(table_a2_xinzhi) / sizeof(table_a2_xinzhi[0]);
            ODObjs.motor_pp   = 10;
            ODObjs.motor_r    = 0.105f; 
            ODObjs.motor_l_d  = 160e-6f;        
            ODObjs.motor_l_q  = 160e-6f;
            ODObjs.polarity   = 1;
            ODObjs.over_temp_drv_level   = 85.0f;
            ODObjs.over_temp_motor_level = 150.0f;
            ODObjs.torque_limit          = 100.0f;
            ODObjs.peak_iq_current       = 90.0f;
            break;
        }
        case BRANCH_A2_JIEKE_WHEEL:
        {
            g_i_scale               = 7.5f;
            g_encoder_calib_current = 10.0f;
            g_gear_ratio            = 5.0f;
            g_multi_pri_gear        = 32;
            g_multi_sec_gear        = 31; // ±3.1 圈  ±19.478rad (负载端)
            g_torque_calib_table    = table_a2_jieke_wheel;
            g_torque_calib_table_len = sizeof(table_a2_jieke_wheel) / sizeof(table_a2_jieke_wheel[0]);
            ODObjs.motor_pp   = 14;
            ODObjs.motor_r    = 0.090f;
            ODObjs.motor_l_d  = 88.5e-6f;        
            ODObjs.motor_l_q  = 88.5e-6f;
            ODObjs.polarity   = 1;
            ODObjs.over_temp_drv_level   = 85.0f;
            ODObjs.over_temp_motor_level = 150.0f;
            ODObjs.torque_limit          = 90.0f;
            ODObjs.peak_iq_current       = 90.0f;
            break;
        }
        default:
        {
            break;
        }
    }
}
