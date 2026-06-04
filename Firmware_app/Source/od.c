#include "od.h"
#include "util.h"
#include "encoder.h"
#include "eeprom_emul.h"
#include "com_can.h"
#include  "version.h"

typedef struct {
    uint16_t index;
    void *obj;
    uint8_t datasize;
    uint8_t attribute;
    int (*update_func)(void);
} OD_entry_t;

ODObjs_t ODObjs;
static uint16_t ODObjsCount;

static const OD_entry_t ODList[] = {
    {0x2000, &ODObjs.error_code,                2, ATTR_RAM | ATTR_R,  NULL},
    {0x2001, &ODObjs.status_word,               2, ATTR_RAM | ATTR_R,  NULL},
    {0x2002, &ODObjs.control_word,              2, ATTR_RAM | ATTR_RW, MC_controlword_update},
    {0x2003, &ODObjs.operation_mode,            1, ATTR_RAM | ATTR_RW, NULL},
    
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
//		{0x2091, &ODObjs.in_encoder_offset,          2, ATTR_ROM | ATTR_RW, NULL},
//		{0x2092, &ODObjs.ex_encoder_offset,          2, ATTR_ROM | ATTR_RW, NULL},
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
    
    ODObjs.motor_pp = 8;
    ODObjs.motor_r = 0.275f;        // 线电阻 550mΩ / 2
    ODObjs.motor_l_d = 160e-6f;     // 线电感 0.32mH / 2
    ODObjs.motor_l_q = 160e-6f;
    ODObjs.motor_rated_vel = 30.0f;
    ODObjs.motor_rated_current = 160.0f;
    ODObjs.motor_torque_constant = 1.00f;
    ODObjs.motor_inertia = 0.000007f;
    
    ODObjs.polarity = 0;
    ODObjs.elec_gear = ENCODER_CPR_F;
    ODObjs.load_inertia = 0.0f;
    ODObjs.torque_limit = 50.0f;
    ODObjs.over_current_level = 160.0f;
    ODObjs.over_load_dpp_level = 99999999.0f;
    ODObjs.over_voltage_level = 40.0f;
    ODObjs.under_voltage_level = 18.0f;
    ODObjs.over_temp_drv_level = 90.0f;
    ODObjs.over_temp_motor_level = 80.0f;
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

    /* Fast search in ordered Object Dictionary. If indexes are mixed,
     * this won't work. If Object Dictionary has up to N entries, then the
     * max number of loop passes is log2(N) */
    while (min < max) {
        /* get entry between min and max */
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
        if(0 != entry->update_func()){
            cs = CS_ERR;
        }
    }
    
    for(int i=0; i<4; i++){
        data[i] = 0;
    }
    
    return cs;    
}
