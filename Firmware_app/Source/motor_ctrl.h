#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include "soc.h"
#include "pid.h"
#include "util.h"
#include "od.h"

//*****************************************************************************
//
// OD params
//
//*****************************************************************************
#define ERROR_CODE                          ODObjs.error_code
#define STATUS_WORD                         ODObjs.status_word
#define CONTROL_WORD                        ODObjs.control_word
#define OPERATION_MODE                      ODObjs.operation_mode
#define TARGET_POSITION                     ODObjs.target_position
#define TARGET_VELOCITY                     ODObjs.target_velocity
#define TARGET_TORQUE                       ODObjs.target_torque
#define ACTUAL_POSITION                     ODObjs.actual_position
#define ACTUAL_VELOCITY                     ODObjs.actual_velocity
#define ACTUAL_TORQUE                       ODObjs.actual_torque
#define ACTUAL_FOLLOWING_ERROR              ODObjs.actual_following_error
#define DC_LINK_VOLTAGE                     ODObjs.dc_link_voltage
#define DC_LINK_CURRENT                     ODObjs.dc_link_current
#define ELECTRICAL_POWER                    ODObjs.electrical_power
#define MECHANICAL_POWER                    ODObjs.mechanical_power
#define DRV_TEMPERATURE                     ODObjs.drv_temperature
#define MOTOR_TEMPERATURE                   ODObjs.motor_temperature
#define IN_ENCODER_VALUE                    ODObjs.in_encoder_value
#define EX_ENCODER_VALUE                    ODObjs.ex_encoder_value
#define IN_ENCODER_OFFSET                   ODObjs.in_encoder_offset
#define EX_ENCODER_OFFSET                   ODObjs.ex_encoder_offset

#define MOTOR_POLE_PAIRS                    ODObjs.motor_pp
#define MOTOR_PHASE_R                       ODObjs.motor_r
#define MOTOR_PHASE_L_D                     ODObjs.motor_l_d
#define MOTOR_PHASE_L_Q                     ODObjs.motor_l_q
#define MOTOR_RATED_VELOCITY                (ODObjs.motor_rated_vel * ENCODER_CPR_F)
#define MOTOR_RATED_CURRENT                 ODObjs.motor_rated_current
#define MOTOR_TORQUE_CONSTANT               ODObjs.motor_torque_constant
#define MOTOR_INERTIA                       ODObjs.motor_inertia

#define POLARITY                            ODObjs.polarity
#define ELEC_GEAR                           ODObjs.elec_gear
#define LOAD_INERTIA                        ODObjs.load_inertia
#define TORQUE_LIMIT                        ODObjs.torque_limit
#define OVER_CURRENT_LEVEL                  ODObjs.over_current_level
#define OVER_LOAD_DPP_LEVEL                 ODObjs.over_load_dpp_level
#define OVER_VOLTAGE_LEVEL                  ODObjs.over_voltage_level
#define UNDER_VOLTAGE_LEVEL                 ODObjs.under_voltage_level
#define OVER_TEMP_DRV_LEVEL                 ODObjs.over_temp_drv_level
#define OVER_TEMP_MOTOR_LEVEL               ODObjs.over_temp_motor_level
#define POSITION_WINDOW                     ODObjs.position_window
#define POSITION_WINDOW_TIME                ODObjs.position_window_time
#define VELOCITY_WINDOW                     ODObjs.velocity_window
#define VELOCITY_WINDOW_TIME                ODObjs.velocity_window_time
#define VELOCITY_THRESHOLD_WINDOW           ODObjs.velocity_threshold
#define VELOCITY_THRESHOLD_WINDOW_TIME      ODObjs.velocity_threshold_time
#define FOLLOWING_ERROR_WINDOW              ODObjs.following_error_window
#define FOLLOWING_ERROR_TIME                ODObjs.following_error_time
#define BRAKE_CTRL                          ODObjs.brake_ctrl
#define VELOCITY_CTRL_GAIN                  ODObjs.velocity_ctrl_gain
#define POSITION_CTRL_GAIN                  ODObjs.position_ctrl_gain
#define PROFILE_VELOCITY                    ODObjs.profile_velocity
#define PROFILE_ACCELERATION                ODObjs.profile_acceleration
#define PROFILE_DECELERATION                ODObjs.profile_deceleration
#define PROFILE_TORQUE_SLOPE                ODObjs.profile_torque_slope
#define HOME_OFFSET                         ODObjs.home_offset
#define PLOT_CTRL                           ODObjs.plot_ctrl

typedef enum {
    ERR_OVER_VOLTAGE            = 0x0001,
    ERR_UNDER_VOLTAGE           = 0x0002,
    ERR_FOLLOWING_ERROR         = 0x0004,
    ERR_OVER_TEMP_DRV           = 0x0008,
    ERR_OVER_TEMP_MOTOR         = 0x0010,
    ERR_OVER_CURRENT_SOFT       = 0x0020,
    ERR_OVER_LOAD               = 0x0040,
    ERR_HEARTBEAT_TIMEOUT       = 0x0080,
		ERR_MULTI_CHECK_ERROR       = 0x0100,

    ERR_ENC_CALIB               = 0x4000,
    ERR_ADC_SELFTEST            = 0x8000,
} tErrorCode;


//*****************************************************************************
// CONTROLWORD
// * bit 0-7: CMD
// * bit 8-15: Reserved
//*****************************************************************************
#define CW_CMD_OPERATION_ENABLE             0x01
#define CW_CMD_OPERATION_DISABLE            0x02
#define CW_CMD_RESET_HOME                   0x03
#define CW_CMD_ERROR_RESET                  0xFF
#define CW_CMD_DEV_ENCODER_CALIB            0xF1
#define CW_CMD_DEV_MULTI_CALIB            0xF2


//*****************************************************************************
// STATUSWORD
// * bit 0: Operation enable
// * bit 1: Reserved
// * bit 2: Error
// * bit 3: Target reached
// * bit 4: Zero speed
// * bit 5-15: Reserved
//*****************************************************************************
#define SW_IS_OPERATION_ENABLE              (STATUS_WORD & (1<<0))
#define SW_OPERATION_ENABLE_SET()           (STATUS_WORD |= (1<<0))
#define SW_OPERATION_ENABLE_RESET()         (STATUS_WORD &= ~(1<<0))
#define SW_IS_ERROR                         (STATUS_WORD & (1<<2))
#define SW_ERROR_SET()                      (STATUS_WORD |= (1<<2))
#define SW_ERROR_RESET()                    (STATUS_WORD &= ~(1<<2))
#define SW_IS_TARGET_REACHED                (STATUS_WORD & (1<<3))
#define SW_TARGET_REACHED_SET()             (STATUS_WORD |= (1<<3))
#define SW_TARGET_REACHED_RESET()           (STATUS_WORD &= ~(1<<3))
#define SW_IS_ZERO_SPEED                    (STATUS_WORD & (1<<4))
#define SW_ZERO_SPEED_SET()                 (STATUS_WORD |= (1<<4))
#define SW_ZERO_SPEED_RESET()               (STATUS_WORD &= ~(1<<4))


//*****************************************************************************
//
// Const params
//
//*****************************************************************************
// Motor param
#define MOTOR_RATED_TORQUE         (MOTOR_RATED_CURRENT * MOTOR_TORQUE_CONSTANT)

//#define MOTOR_BACK_EMF_CONSTANT     11.0f     // [Vpk_LL/krpm]
//#define MOTOR_FLUX_LINKAGE          (ONE_BY_SQRT3 * MOTOR_BACK_EMF_CONSTANT * 60.0f / (1000.0f * MOTOR_POLE_PAIRS * M_2PI))     // [Wb]     ¦Ëpm = (1/¡Ì3)(Ke/(1000P))*(60/2¦Ð)
//#define MOTOR_TORQUE_CONSTANT       (1.5f * MOTOR_POLE_PAIRS * MOTOR_FLUX_LINKAGE)                                              // [Nm/A]   ¦Ëpm = (2/3)*(Kt/P)

//#define CURRENT_CTRL_BW_HZ          800
#define CURRENT_CTRL_BW_HZ          350

#define POS_INPUT_FILTER_BW         50.0f  // rad/s

// Control period
#define CURRENT_CTRL_FREQUENCY 		CURRENT_MEASURE_HZ
#define CURRENT_CTRL_PERIOD         (1.0f / CURRENT_CTRL_FREQUENCY)
#define VELOCITY_CTRL_FREQUENCY     (CURRENT_MEASURE_HZ / 5)
#define VELOCITY_CTRL_PERIOD        (1.0f / VELOCITY_CTRL_FREQUENCY)
#define POSITION_CTRL_FREQUENCY     (CURRENT_MEASURE_HZ / 10)
#define POSITION_CTRL_PERIOD        (1.0f / POSITION_CTRL_FREQUENCY)
#define SERVO_CTRL_FREQUENCY        (CURRENT_MEASURE_HZ / 10)
#define SERVO_CTRL_PERIOD           (1.0f / SERVO_CTRL_FREQUENCY)
#define GEAR_RATIO                  25.0f

// Control loop define
#define ENABLED_LOOP_NONE	        0
#define ENABLED_LOOP_POSITION       1
#define ENABLED_LOOP_VELOCITY       2
#define ENABLED_LOOP_CURRENT        4

typedef enum {
    MCS_IDLE                        = 1,
    MCS_OPERATION                   = 2,
    MCS_ENCODER_CALIB               = 4,
		MCS_EX_ENCODER_CHECK            = 8,
} tMCState;

typedef enum {
    OPM_PROFILE_POSITION            = 1,    // Profile position mode
    OPM_PROFILE_VELOCITY            = 2,    // Profile velocity mode
    OPM_PROFILE_TORQUE              = 3,    // Profile torque mode
    OPM_INTERP_POSITION             = 4,    // Interpolated position mode
} tOperationMode;

typedef struct {
    bool is_bootup;
    float BusVoltage;
    float MaxModulateVoltage;
    float Ia, Ib, Ic;
    float Ialpha, Ibeta;
    float Id, Iq;
    float id_filtered, iq_filtered;
    float mod_d, mod_q;
    float i_bus;
    float electrical_power;
    float mechanical_power;

    bool cmd_update;
    float torque_cmd;       // [Nm]
    float velocity_cmd;     // [Usr]
    float position_cmd;     // [Usr]

    float current_set;      // [A]
    float velocity_set;     // [Count/s]
    int64_t position_set;   // [Count]

    float current_ff;       // [A]
    float velocity_ff;      // [Count/s]

    // interpolated Position
    int64_t pos_target;     // [Count]
    float step_pos_add;
    float step_pos_filter_kp;
    float step_pos_filter_ki;

    uint32_t TargeReachedTick;
    uint32_t FollowingErrorTick;

    uint8_t op_mode;
    uint8_t enabled_loop;

    tPID PID_Id;
    tPID PID_Iq;
    tPID PID_vel;
    float PosGain;
		float Kp;
		float Kd;
		float current_mit;      // [A]
		float pos_set;     // [Count/s]
		float raw_pos;
		float raw_vel;
		float raw_tor;


} tMotorControl;

extern tMotorControl MotorControl;

void MC_init(void);
int MC_ctrl_param_update(void);
int MC_controlword_update(void);
int MC_position_update(void);
int MC_velocity_update(void);
int MC_torque_update(void);

int MC_profile_update(void);
void MC_pdo_profile_position(float pos, float vel);
void MC_pdo_profile_velocity(float vel, float acc);
void MC_pdo_profile_torque(float torque, float slope);
void MC_pdo_interp_position(float pos);

extern inline void MC_target_sync(void);

tMCState MC_get_state(void);
int MC_set_state(tMCState state);

void MC_reset_home(void);
void MC_error_reset(void);

void MC_low_priority_task(void);
void MC_high_priority_task(void);

extern inline void MC_modulate(float Vd, float Vq, float pwm_phase);

#endif
