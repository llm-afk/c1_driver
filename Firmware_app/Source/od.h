#ifndef __OD_H__
#define __OD_H__

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define VERSION_MAJOR       1U
#define VERSION_MINOR       5U

#define ATTR_R      0x01
#define ATTR_W      0x02
#define ATTR_RW     0x03
#define ATTR_ROM    0x04
#define ATTR_RAM    0x08

//*****************************************************************************
//
// OD data declaration
//
//*****************************************************************************
typedef struct {
    uint16_t error_code;
    uint16_t status_word;
    uint16_t control_word;
    uint8_t  operation_mode;
    
    float    target_position;
    float    target_velocity;
    float    target_torque;
    
    float    actual_position;
    float    actual_velocity;
    float    actual_torque;
    float    actual_following_error;
    float    dc_link_voltage;
    float    dc_link_current;
    float    electrical_power;
    float    mechanical_power;
    float    drv_temperature;
    float    motor_temperature;
    float    in_encoder_value;
    float    ex_encoder_value;
    
    uint8_t  node_id;                     // [id]      [UINT8]  (0~127)                                         Update After Power Recycle
    uint8_t  can_baudrate;                // [enum]    [UINT8]  (0~2) 0:500K 1:800K 2:1000K                     Update After Power Recycle
    uint8_t  data_baudrate;               // [enum]    [UINT8]  (0~3) 0:1M 1:2M 2:4M 3:5M                       Update After Power Recycle
    uint16_t heartbeat_producer_time;     // [ms]      [UINT16] (0~65535)                                       Update After Power Recycle
    uint16_t heartbeat_consumer_time;     // [ms]      [UINT16] (0~65535)                                       Update After Power Recycle

    uint16_t motor_pp;
    float    motor_r;
    float    motor_l_d;
    float    motor_l_q;
    float    motor_rated_vel;
    float    motor_rated_current;
    float    motor_torque_constant;
    float    motor_inertia;
    uint8_t  polarity;
    float    elec_gear;
    float    load_inertia;
    float    torque_limit;
    float    over_current_level;
    float    over_load_dpp_level;
    float    over_voltage_level;
    float    under_voltage_level;
    float    over_temp_drv_level;
    float    over_temp_motor_level;
    float    position_window;             // [usr]     [FLOAT]  (0~9999999)
    uint16_t position_window_time;        // [ms]      [UINT16] (0~65535)
    float    velocity_window;             // [usr/s]   [FLOAT]  (0~9999999)
    uint16_t velocity_window_time;        // [ms]      [UINT16] (0~65535)
    float    velocity_threshold;
    uint16_t velocity_threshold_time;
    float    following_error_window;      // [usr]     [FLOAT]  (0~9999999)
    uint16_t following_error_time;        // [ms]      [UINT16] (0~65535)
    uint8_t  brake_ctrl;                  // [enum]    [UINT8]  (0~2) 0:disable 1:enable_H 2:enable_L
    
    uint16_t velocity_ctrl_gain;
    uint16_t position_ctrl_gain;
    
    float    profile_velocity;            // [usr/s]   [FLOAT]  (0~9999999)
    float    profile_acceleration;        // [usr/s²]  [FLOAT]  (0~9999999)
    float    profile_deceleration;        // [usr/s²]  [FLOAT]  (0~9999999)
    float    profile_torque_slope;        // [Nm/s]    [FLOAT]  (0~100)
    
    float    home_offset;
		uint16_t in_encoder_offset;
		uint16_t ex_encoder_offset;
    uint16_t firmware_version;
    uint8_t  restore_default;
    uint8_t  plot_ctrl;
} ODObjs_t;

extern ODObjs_t ODObjs;

void OD_init(void);
int OD_restore_defalt(void);
uint8_t OD_read(uint16_t idx, uint8_t *data);
uint8_t OD_write_1(uint16_t idx, uint8_t *data);
uint8_t OD_write_2(uint16_t idx, uint8_t *data);
uint8_t OD_write_4(uint16_t idx, uint8_t *data);

#endif
