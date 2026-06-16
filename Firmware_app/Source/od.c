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
        
        // Custom feature: A2 series with ID 4 has a gear ratio of 5
        if((g_current_branch == BRANCH_A2 || g_current_branch == BRANCH_A2_XINZHI) && ODObjs.node_id == 4) {
            g_gear_ratio = 5.0f;
        }
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
    {0.0f, 0.0f},
    {100.0f, 100.0f} // Placeholder
};

static const tTorqueCalibPoint table_a2[] = {
    {0.0f, 0.0f},
    {100.0f, 100.0f} // Placeholder
};

static const tTorqueCalibPoint table_a2_xinzhi[] = {
    {0.0f, 0.0f},
    {100.0f, 100.0f} // Placeholder
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
            ODObjs.motor_r    = 0.275f;
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
            g_encoder_calib_current = 8.0f;
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
            ODObjs.torque_limit          = 90.0f;
            ODObjs.peak_iq_current       = 60.0f;
            break;
        }
        case BRANCH_A2_XINZHI:
        {
            g_i_scale               = 13.0f; // 10->82.5A  13->63.4A
            g_encoder_calib_current = 8.0f;
            g_gear_ratio            = 25.0f;
            g_multi_pri_gear        = 32;  
            g_multi_sec_gear        = 31; // ±0.62 圈  ±3.8936rad
            g_torque_calib_table    = table_a2_xinzhi;
            g_torque_calib_table_len = sizeof(table_a2_xinzhi) / sizeof(table_a2_xinzhi[0]);
            ODObjs.motor_pp   = 10;
            ODObjs.motor_r    = 0.275f;
            ODObjs.motor_l_d  = 160e-6f;        
            ODObjs.motor_l_q  = 160e-6f;
            ODObjs.polarity   = 1;
            ODObjs.over_temp_drv_level   = 85.0f;
            ODObjs.over_temp_motor_level = 150.0f;
            ODObjs.torque_limit          = 90.0f;
            ODObjs.peak_iq_current       = 60.0f;
            break;
        }
        default:
        {
            break;
        }
    }
}
