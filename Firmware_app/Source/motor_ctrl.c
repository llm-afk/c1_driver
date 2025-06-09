#include "motor_ctrl.h"
#include "motion_planner.h"
#include "encoder.h"
#include "com_can.h"

static float mElecGear;
static float mElecGearInvers;

static float mProfileVelocity;
static float mProfileAcceleration;
static float mProfileDeceleration;
static float mProfileTorqueSlope;

static bool mIsTPDOReport = false;
static float mInterAcc2CurrFactor;

#define USR_TO_INTER(value)             (value * mElecGear)
#define INTER_TO_USR(value)             (value * mElecGearInvers)
#define INTER_ACC_TO_CURRENT_FF(value)  (value * mInterAcc2CurrFactor)

typedef struct {
    tMCState state;
    tMCState state_next;
    uint8_t state_next_ready;
} tFSM;

static volatile tFSM mFSM = {
    .state = MCS_IDLE,
    .state_next = MCS_IDLE,
    .state_next_ready = 0,
};

tMotorControl MotorControl = {
    .is_bootup = false,
    .BusVoltage = 24.0f,
};

static void operation_mode_reset(void);
static void enter_state(void);
static void exit_state(void);
static void profile_position_execute(void);
static void profile_velocity_execute(void);
static void profile_torque_execute(void);
static void interp_position_execute(void);
static void servo_loop(void);
static inline void position_ctrl_loop(void);
static inline void velocity_ctrl_loop(void);
static inline void current_ctrl_loop(void);
static inline void set_phase_voltage(float Valpha, float Vbeta);

void MC_init(void)
{
    MC_profile_update();
    MC_ctrl_param_update();
    
    // Position input filter
    MotorControl.step_pos_filter_ki = 2.0f * POS_INPUT_FILTER_BW;
    MotorControl.step_pos_filter_kp = 0.25f * POW2(MotorControl.step_pos_filter_ki);
    
    operation_mode_reset();
}

int MC_ctrl_param_update(void)
{
    ENTER_CRITICAL();
    
    // Current ctrl param
    float BW = CURRENT_CTRL_BW_HZ * M_2PI;  // Hz -> Rad/s
    PID_setting(&MotorControl.PID_Id, BW*MOTOR_PHASE_L_D, MOTOR_PHASE_R/MOTOR_PHASE_L_D, 0.0f, CURRENT_CTRL_PERIOD, 0.0f);
    PID_setting(&MotorControl.PID_Iq, BW*MOTOR_PHASE_L_Q, MOTOR_PHASE_R/MOTOR_PHASE_L_Q, 0.0f, CURRENT_CTRL_PERIOD, 0.0f);

    // Velocity ctrl param
    float J = MOTOR_INERTIA + LOAD_INERTIA;
    float damping_factor = 1.0f + 0.01f * (10000 - VELOCITY_CTRL_GAIN);  // damping_factor(1~101)
    float K = MOTOR_TORQUE_CONSTANT / J;
    float ki = BW / POW2(damping_factor);
    float kp = ki * damping_factor / K;
    PID_setting(&MotorControl.PID_vel, kp, ki, 0.0f, VELOCITY_CTRL_PERIOD, 0.0f);

    // Position ctrl param
    MotorControl.PosGain = POSITION_CTRL_GAIN * 0.1f;

    mInterAcc2CurrFactor = (J * M_2PI) / (ENCODER_CPR_F * MOTOR_TORQUE_CONSTANT);

    EXIT_CRITICAL();
    
    return 0;
}

int MC_controlword_update(void)
{
    // CMD
    switch(CONTROL_WORD & 0x00FF){
        case CW_CMD_OPERATION_ENABLE:
            MC_set_state(MCS_OPERATION);
            break;
        
        case CW_CMD_OPERATION_DISABLE:
            MC_set_state(MCS_IDLE);
            break;
        
        case CW_CMD_RESET_HOME:
            MC_reset_home();
            break;
        
        case CW_CMD_ERROR_RESET:
            MC_error_reset();
            break;
        
        case CW_CMD_DEV_ENCODER_CALIB:
            MC_set_state(MCS_ENCODER_CALIB);
            break;
        
        default:
            break;
    }
    
    CONTROL_WORD &= 0xFF00;
    
    return 0;
}

int MC_position_update(void)
{
    float UsrPos = TARGET_POSITION;

    // Dir reverse
    if(POLARITY){
        UsrPos = -UsrPos;
    }
    
    MC_profile_update();

    SW_TARGET_REACHED_RESET();
    MotorControl.position_cmd = UsrPos;
    MotorControl.cmd_update = true;
    
    return 0;
}

int MC_velocity_update(void)
{
    float max = INTER_TO_USR(MOTOR_RATED_VELOCITY);
    TARGET_VELOCITY = CLAMP(TARGET_VELOCITY, -max, +max);
    
    float UsrVel = TARGET_VELOCITY;

    // Dir reverse
    if(POLARITY){
        UsrVel = -UsrVel;
    }
    
    MC_profile_update();

    SW_TARGET_REACHED_RESET();
    MotorControl.velocity_cmd = UsrVel;
    MotorControl.cmd_update = true;
    
    return 0;
}

int MC_torque_update(void)
{
    TARGET_TORQUE = CLAMP(TARGET_TORQUE, -TORQUE_LIMIT, +TORQUE_LIMIT);

    float Torque = TARGET_TORQUE;

    // Dir reverse
    if(POLARITY){
        Torque = -Torque;
    }
    
    MC_profile_update();

    SW_TARGET_REACHED_RESET();
    MotorControl.torque_cmd = Torque;
    MotorControl.cmd_update = true;
    
    return 0;
}

int MC_profile_update(void)
{
    mProfileVelocity = PROFILE_VELOCITY;
    mProfileAcceleration = PROFILE_ACCELERATION;
    mProfileDeceleration = PROFILE_DECELERATION;
    mProfileTorqueSlope = PROFILE_TORQUE_SLOPE;
    
    return 0;
}

void MC_pdo_profile_position(float pos, float vel)
{
    TARGET_POSITION = pos;
    float UsrPos = pos;
    
    // Dir reverse
    if(POLARITY){
        UsrPos = -UsrPos;
    }
    
    mProfileVelocity = vel;
    
    mIsTPDOReport = true;
    MotorControl.TargeReachedTick = 0;
    SW_TARGET_REACHED_RESET();
    MotorControl.position_cmd = UsrPos;
    MotorControl.cmd_update = true;
}

void MC_pdo_profile_velocity(float vel, float acc)
{
    float max = INTER_TO_USR(MOTOR_RATED_VELOCITY);
    vel = CLAMP(vel, -max, +max);
    
    TARGET_VELOCITY = vel;
    float UsrVel = vel;

    // Dir reverse
    if(POLARITY){
        UsrVel = -UsrVel;
    }
    
    mProfileAcceleration = acc;
    mProfileDeceleration = acc;

    mIsTPDOReport = true;
    MotorControl.TargeReachedTick = 0;
    SW_TARGET_REACHED_RESET();
    MotorControl.velocity_cmd = UsrVel;
    MotorControl.cmd_update = true;
}

void MC_pdo_profile_torque(float torque, float slope)
{
    torque = CLAMP(torque, -TORQUE_LIMIT, +TORQUE_LIMIT);
    TARGET_TORQUE = torque;

    // Dir reverse
    if(POLARITY){
        torque = -torque;
    }
    
    mProfileTorqueSlope = slope;

    mIsTPDOReport = true;
    SW_TARGET_REACHED_RESET();
    MotorControl.torque_cmd = torque;
    MotorControl.cmd_update = true;
}

void MC_pdo_interp_position(float pos)
{
    float UsrPos = pos;
    
    // Dir reverse
    if(POLARITY){
        UsrPos = -UsrPos;
    }
    
    MotorControl.position_cmd = UsrPos;
}

inline void MC_target_sync(void)
{
    MotorControl.cmd_update = true;
}

tMCState MC_get_state(void)
{
    return mFSM.state;
}

int MC_set_state(tMCState state)
{
    switch (state) {
        case MCS_IDLE:
            mFSM.state_next = state;
            mFSM.state_next_ready = 0;
            break;
        
        case MCS_OPERATION:
            if(ERROR_CODE){
                return -1;
            }

            if(OPERATION_MODE == 0){
                return -2;
            }

            mFSM.state_next = state;
            mFSM.state_next_ready = 0;
            break;
            
        case MCS_ENCODER_CALIB:
//            if(ERROR_CODE & (~ERR_ENC_CALIB)){
//                return -1;
//            }

            mFSM.state_next = state;
            mFSM.state_next_ready = 0;
            break;

        default:
            return -10;
    }

    return 0;
}

void MC_reset_home(void)
{
//    int ex = ENCODER_EX_read_rectified();
//    if(ex != -1){
//        float c0 = 24 * Encoder.count_in_cpr / ENCODER_CPR_F;
//        float c1 = 27 * ex / ENCODER_CPR_F;
//        int diff = (int)((c0 - c1) + 0.5f);
//        if(diff <= 0){
//            diff += 27;
//        }
//        diff = (int)((diff / 3.0f) + 0.5f);
//        if(diff >= 9){
//            diff -= 9;
//        }
//        
//        float offset = diff * ENCODER_CPR + Encoder.count_in_cpr;
//        
//        OD_write_4(0x2090, (uint8_t*)&offset);
//        
//        ENTER_CRITICAL();
//        Encoder.shadow_count = 0;
//        MotorControl.pos_target = 0;
//        MotorControl.position_set = 0;
//        EXIT_CRITICAL();
//    }

    ENTER_CRITICAL();
    Encoder.shadow_count = 0;
    MotorControl.pos_target = 0;
    MotorControl.position_set = 0;
    EXIT_CRITICAL();
}

void MC_error_reset(void)
{
    if(ERROR_CODE){
        
        ERROR_CODE &= 0xF000;

        if(ERROR_CODE == 0){
            SW_ERROR_RESET();
        }
    }
}

static void operation_mode_reset(void)
{
    int64_t shadow_count = Encoder.shadow_count;
    
    // Param update
    mElecGear = ELEC_GEAR;
    mElecGearInvers = 1.0f / ELEC_GEAR;
    
    MP_reset();
    
    MotorControl.cmd_update = false;

    MotorControl.current_set = 0;
    MotorControl.velocity_set = 0;
    MotorControl.position_set = shadow_count;
    MotorControl.current_ff = 0;
    MotorControl.velocity_ff = 0;

    MotorControl.pos_target = shadow_count;
    MotorControl.step_pos_add = 0;

    MotorControl.TargeReachedTick = 0;
    MotorControl.FollowingErrorTick = 0;

    MotorControl.op_mode = OPERATION_MODE;
    MotorControl.enabled_loop = ENABLED_LOOP_NONE;
    
    MotorControl.id_filtered = 0.0f;
    MotorControl.iq_filtered = 0.0f;
    MotorControl.i_bus = 0.0f;
    MotorControl.electrical_power = 0.0f;
    MotorControl.mechanical_power = 0.0f;

    PID_reset(&MotorControl.PID_vel);
    PID_reset(&MotorControl.PID_Id);
    PID_reset(&MotorControl.PID_Iq);

    // statusword
    SW_TARGET_REACHED_RESET();
}

static void enter_state(void)
{
    switch(mFSM.state_next){
        case MCS_IDLE:
            break;

        case MCS_OPERATION:
            SOC_pwm_enable();
            operation_mode_reset();
            SW_OPERATION_ENABLE_SET();
            if(BRAKE_CTRL == 1){
                BRAKE_SET();
            }else if(BRAKE_CTRL == 2){
                BRAKE_RESET();
            }
            break;
        
        case MCS_ENCODER_CALIB:
            SOC_pwm_enable();
            ENCODER_calib_start();
            break;

        default:
            break;
    }
}

static void exit_state(void)
{
    switch(mFSM.state){
        case MCS_IDLE:
            mFSM.state_next_ready = 1;
            break;

        case MCS_OPERATION:
            SOC_pwm_disable();
            operation_mode_reset();
            SW_OPERATION_ENABLE_RESET();
            if(BRAKE_CTRL == 1){
                BRAKE_RESET();
            }else if(BRAKE_CTRL == 2){
                BRAKE_SET();
            }
            mFSM.state_next_ready = 1;
            break;

        case MCS_ENCODER_CALIB:
            SOC_pwm_disable();
            ENCODER_calib_end();
            mFSM.state_next_ready = 1;
            break;

        default:
            break;
    }
}

static void profile_position_execute(void)
{
    if(MotorControl.cmd_update){
        MotorControl.cmd_update = false;

        float p0 = INTER_TO_USR(Encoder.shadow_count);
        float v0 = INTER_TO_USR(Encoder.vel);

        MP_trapezoid_pos_calculate(p0, MotorControl.position_cmd, v0, 0, mProfileVelocity, mProfileAcceleration, mProfileDeceleration);
    }

    if(MP_is_run()){
        float pos, vel, acc;

        MP_trapezoid_pos_execute(&pos, &vel, &acc);
        MotorControl.position_set = USR_TO_INTER(pos);
        MotorControl.velocity_ff = USR_TO_INTER(vel);
        MotorControl.current_ff = INTER_ACC_TO_CURRENT_FF(USR_TO_INTER(acc));
    }

    MotorControl.enabled_loop = ENABLED_LOOP_POSITION | ENABLED_LOOP_VELOCITY | ENABLED_LOOP_CURRENT;

   // Target reached check
   if(!SW_IS_TARGET_REACHED){
       if(ABS(TARGET_POSITION-ACTUAL_POSITION) < POSITION_WINDOW){
           if(MotorControl.TargeReachedTick == 0){
               MotorControl.TargeReachedTick = get_tick();
           }else{
                if(get_ms_since(MotorControl.TargeReachedTick) >= POSITION_WINDOW_TIME){
                    SW_TARGET_REACHED_SET();
                    
                    if(mIsTPDOReport){
                        mIsTPDOReport = false;
                        
                        CanFrame frame;
                        frame.id = MSG_ID_TPDO_1;
                        frame.dlc = 6;
                        *(float*)&frame.data[0] = ACTUAL_POSITION;
                        *(uint16_t*)&frame.data[4] = STATUS_WORD;
                        COM_CAN_report_frame(&frame);
                    }
               }
           }
       }else{
           MotorControl.TargeReachedTick = 0;
       }
   }
}

static void profile_velocity_execute(void)
{
    if(MotorControl.cmd_update){
        MotorControl.cmd_update = false;

        float v0 = INTER_TO_USR(Encoder.vel);
        float v1 = MotorControl.velocity_cmd;

        MP_trapezoid_vel_calculate(v0, v1, mProfileAcceleration, mProfileDeceleration);
    }

    if(MP_is_run()){
        float vel, acc;

        MP_trapezoid_vel_execute(&vel, &acc);

        MotorControl.velocity_set = USR_TO_INTER(vel);
        MotorControl.current_ff = INTER_ACC_TO_CURRENT_FF(USR_TO_INTER(acc));
    }

    MotorControl.enabled_loop = ENABLED_LOOP_VELOCITY | ENABLED_LOOP_CURRENT;

    // Target reached check
    if(!SW_IS_TARGET_REACHED){
        if(ABS(TARGET_VELOCITY-ACTUAL_VELOCITY) < VELOCITY_WINDOW){
            if(MotorControl.TargeReachedTick == 0){
                MotorControl.TargeReachedTick = get_tick();
            }else{
                if(get_ms_since(MotorControl.TargeReachedTick) >= VELOCITY_WINDOW_TIME){
                    SW_TARGET_REACHED_SET();
                   
                    if(mIsTPDOReport){
                        mIsTPDOReport = false;

                        CanFrame frame;
                        frame.id = MSG_ID_TPDO_2;
                        frame.dlc = 6;
                        *(float*)&frame.data[0] = ACTUAL_VELOCITY;
                        *(uint16_t*)&frame.data[4] = STATUS_WORD;
                        COM_CAN_report_frame(&frame);
                    }
                }
            }
        }else{
            MotorControl.TargeReachedTick = 0;
        }
    }
}

static void profile_torque_execute(void)
{
    if(MotorControl.cmd_update){
        MotorControl.cmd_update = false;

        float t0 = MotorControl.current_set * MOTOR_TORQUE_CONSTANT;
        float t1 = MotorControl.torque_cmd;

        MP_trapezoid_torque_calculate(t0, t1, mProfileTorqueSlope);
    }

    if(MP_is_run()){
        float torque;

        MP_trapezoid_torque_execute(&torque);

        MotorControl.current_set = torque / MOTOR_TORQUE_CONSTANT;
    }else{
        SW_TARGET_REACHED_SET();
        
        if(mIsTPDOReport){
            mIsTPDOReport = false;
            
            CanFrame frame;
            frame.id = MSG_ID_RPDO_3;
            frame.dlc = 6;
            *(float*)&frame.data[0] = ACTUAL_TORQUE;
            *(uint16_t*)&frame.data[4] = STATUS_WORD;
            COM_CAN_report_frame(&frame);
        }
    }

    MotorControl.enabled_loop = ENABLED_LOOP_CURRENT;
}

static void interp_position_execute(void)
{
    if(MotorControl.cmd_update){
        MotorControl.cmd_update = false;
        MotorControl.pos_target = USR_TO_INTER(MotorControl.position_cmd);
    }

    // 2nd order pos tracking filter
    float acc = MotorControl.step_pos_filter_kp * (float)(MotorControl.pos_target - MotorControl.position_set) - MotorControl.step_pos_filter_ki * MotorControl.velocity_ff;
    MotorControl.current_ff = INTER_ACC_TO_CURRENT_FF(acc);
    MotorControl.velocity_ff += acc * SERVO_CTRL_PERIOD;
    MotorControl.step_pos_add += MotorControl.velocity_ff * SERVO_CTRL_PERIOD;
    int integer = (int)MotorControl.step_pos_add;
    MotorControl.step_pos_add -= integer;
    MotorControl.position_set += integer;

    MotorControl.enabled_loop = ENABLED_LOOP_POSITION | ENABLED_LOOP_VELOCITY | ENABLED_LOOP_CURRENT;
}

// 2KHz
static void servo_loop(void)
{
    /* state transition management */
    if(mFSM.state_next != mFSM.state){
        exit_state();
        if(mFSM.state_next_ready){
            enter_state();
            mFSM.state = mFSM.state_next;
        }
    }

    // Update od valaue
    DC_LINK_VOLTAGE = MotorControl.BusVoltage;
    if(POLARITY){
        ACTUAL_TORQUE   = - MotorControl.iq_filtered * MOTOR_TORQUE_CONSTANT;
        ACTUAL_VELOCITY = - INTER_TO_USR(Encoder.vel);
        ACTUAL_POSITION = - INTER_TO_USR(Encoder.shadow_count);
    }else{
        ACTUAL_TORQUE   = + MotorControl.iq_filtered * MOTOR_TORQUE_CONSTANT;
        ACTUAL_VELOCITY = + INTER_TO_USR(Encoder.vel);
        ACTUAL_POSITION = + INTER_TO_USR(Encoder.shadow_count);
    }
    DC_LINK_CURRENT = MotorControl.i_bus;
    ELECTRICAL_POWER = MotorControl.electrical_power;
    MECHANICAL_POWER = MotorControl.mechanical_power;
    
    // Protect check ========================================================
    // Over current check
    if(ABS(MotorControl.Ia) > OVER_CURRENT_LEVEL || ABS(MotorControl.Ib) > OVER_CURRENT_LEVEL || ABS(MotorControl.Ic) > OVER_CURRENT_LEVEL){
        COM_CAN_report_err(ERR_OVER_CURRENT_SOFT);
    }

    // Over voltage check
    if(DC_LINK_VOLTAGE > OVER_VOLTAGE_LEVEL){
        COM_CAN_report_err(ERR_OVER_VOLTAGE);
    }

    // Under voltage check
    if(DC_LINK_VOLTAGE < UNDER_VOLTAGE_LEVEL){
        COM_CAN_report_err(ERR_UNDER_VOLTAGE);
    }
    // Protect check ========================================================

    switch(mFSM.state){
        case MCS_OPERATION:

            MotorControl.i_bus = MotorControl.mod_d * MotorControl.Id + MotorControl.mod_q * MotorControl.iq_filtered;
            MotorControl.electrical_power = MotorControl.BusVoltage * MotorControl.i_bus;
            MotorControl.mechanical_power = MotorControl.iq_filtered * MOTOR_TORQUE_CONSTANT * M_2PI * Encoder.vel / ENCODER_CPR_F;  // P = 2¦Ð * Torque * Vel(r/s)

            // Operation mode switch
            if(MotorControl.op_mode != OPERATION_MODE){
                operation_mode_reset();
            }

            // Operation
            switch(MotorControl.op_mode){
                case OPM_PROFILE_POSITION:
                    profile_position_execute();
                    break;

                case OPM_PROFILE_VELOCITY:
                    profile_velocity_execute();
                    break;

                case OPM_PROFILE_TORQUE:
                    profile_torque_execute();
                    break;

                case OPM_INTERP_POSITION:
                    interp_position_execute();
                    break;

                default:
                    break;
            }
            break;
        
        case MCS_ENCODER_CALIB:
            ENCODER_calib_loop(SERVO_CTRL_PERIOD);
            break;

        default:
            break;
    }
}

// Free loop
void MC_low_priority_task(void)
{
    static uint32_t tick_100 = 0;
    static uint32_t tick_2000 = 0;
    static uint32_t zero_speed_tick = 0;
    
    static bool is_plot = false;
    static float last_value = 0;
    
    static float over_torque_dpp = 0;
    
    // 100Hz
    if(get_ms_since(tick_100) >= 10){
        tick_100 = get_tick();

        // Zero speed check
        if(ABS(INTER_TO_USR(Encoder.vel)) < VELOCITY_THRESHOLD_WINDOW){
            if(zero_speed_tick == 0){
                zero_speed_tick = get_tick();
            }else{
                if(get_ms_since(zero_speed_tick) >= VELOCITY_THRESHOLD_WINDOW_TIME){
                    SW_ZERO_SPEED_SET();
                }
            }
        }else{
            SW_ZERO_SPEED_RESET();
            zero_speed_tick = get_tick();
        }

        // drv over temperature check
        DRV_TEMPERATURE = SOC_read_drv_temp();
        if(DRV_TEMPERATURE > OVER_TEMP_DRV_LEVEL){
            COM_CAN_report_err(ERR_OVER_TEMP_DRV);
        }
        
        // motor over temperature check
        MOTOR_TEMPERATURE = SOC_read_motor_temp();
        if(MOTOR_TEMPERATURE > OVER_TEMP_MOTOR_LEVEL){
            COM_CAN_report_err(ERR_OVER_TEMP_MOTOR);
        }
        
        // Over load check
        float over_torque = ABS(ACTUAL_TORQUE) - MOTOR_RATED_TORQUE;
        if(over_torque > 0){
            over_torque_dpp += over_torque;
            if(over_torque_dpp > OVER_LOAD_DPP_LEVEL){
                COM_CAN_report_err(ERR_OVER_LOAD);
            }
        }else{
            over_torque_dpp = 0;
        }
        
        // in encodedr value update
        IN_ENCODER_VALUE = Encoder.count_in_cpr;
        
        // ex encoder value update
        EX_ENCODER_VALUE = ENCODER_EX_read();
    }
    
    // 2000Hz for plot
    if(PLOT_CTRL){
        if(get_us_since(tick_2000) >= 500){
            tick_2000 = get_timestamp();

            float curr_value = 0;
            
            switch(PLOT_CTRL){
                case 1:
                    curr_value = ACTUAL_TORQUE;
                    break;
                case 2:
                    curr_value = ACTUAL_VELOCITY;
                    break;
                case 3:
                    curr_value = ACTUAL_POSITION;
                    break;
                case 4:
                    curr_value = ACTUAL_FOLLOWING_ERROR;
                    break;
                case 5:
                    curr_value = DC_LINK_VOLTAGE;
                    break;
                case 6:
                    curr_value = DC_LINK_CURRENT;
                    break;
                case 7:
                    curr_value = ELECTRICAL_POWER;
                    break;
                case 8:
                    curr_value = MECHANICAL_POWER;
                    break;
                case 9:
                    curr_value = DRV_TEMPERATURE;
                    break;
                case 10:
                    curr_value = MOTOR_TEMPERATURE;
                    break;
                case 11:
                    curr_value = IN_ENCODER_VALUE;
                    break;
                case 12:
                    curr_value = EX_ENCODER_VALUE;
                    break;
                default:
                    break;
            }
            
            if(!is_plot){
                is_plot = true;
                last_value = curr_value;
            }else{
                is_plot = false;
                COM_CAN_plot_value(last_value, curr_value);
            }
        }
    }
}

// 20KHz
void MC_high_priority_task(void)
{
    if(!MotorControl.is_bootup){
        return;
    }

    static uint8_t tick = 0;
    if(++tick >= 10){
        tick = 0;
    }

    // Lowpass filter FC=3K, rc = 1/(2*pi*FC), alpha = dt / (dt + rc)
    float vbus_new = SOC_read_vbus();
    MotorControl.BusVoltage = 0.4852f * vbus_new + 0.5148f * MotorControl.BusVoltage;
    MotorControl.MaxModulateVoltage = ONE_BY_SQRT3 * MotorControl.BusVoltage * MAX_MODULATION;

    MotorControl.Ia = SOC_read_iphase_a();
    MotorControl.Ib = SOC_read_iphase_b();
    MotorControl.Ic = SOC_read_iphase_c();

    // Clarke transform
    MotorControl.Ialpha = MotorControl.Ia;
    MotorControl.Ibeta  = (MotorControl.Ib - MotorControl.Ic) * ONE_BY_SQRT3;

    ENCODER_loop();

    float sint = arm_sin_f32(Encoder.phase);
    float cost = arm_cos_f32(Encoder.phase);

    // Park transform
    MotorControl.Id =  MotorControl.Ialpha * cost + MotorControl.Ibeta * sint;
    MotorControl.Iq = -MotorControl.Ialpha * sint + MotorControl.Ibeta * cost;
    
    // Lowpass filter FC=800, rc = 1/(2*pi*FC), alpha = dt / (dt + rc)
    MotorControl.id_filtered = 0.2f * MotorControl.Id + 0.8f * MotorControl.id_filtered;
    MotorControl.iq_filtered = 0.2f * MotorControl.Iq + 0.8f * MotorControl.iq_filtered;
    
    // 2KHz
    if(tick == 0){
        servo_loop();
    }

    // 2KHz
    if(tick == 1){
        if(MotorControl.enabled_loop & ENABLED_LOOP_POSITION){
            position_ctrl_loop();
        }
    }

    // 4KHz
    if(tick == 2 || tick == 7){
        if(MotorControl.enabled_loop & ENABLED_LOOP_VELOCITY){
            velocity_ctrl_loop();
        }
    }

    // 20KHz
    if(MotorControl.enabled_loop & ENABLED_LOOP_CURRENT){
        current_ctrl_loop();
    }
}

static inline void position_ctrl_loop(void)
{
    const float pos_error = (float)(MotorControl.position_set - Encoder.shadow_count);
    float out = pos_error * MotorControl.PosGain + MotorControl.velocity_ff;
    MotorControl.velocity_set = CLAMP(out, -MOTOR_RATED_VELOCITY, +MOTOR_RATED_VELOCITY);
    
    // Following error update & check
    ACTUAL_FOLLOWING_ERROR = INTER_TO_USR(pos_error);
    if(POLARITY){
        ACTUAL_FOLLOWING_ERROR = -ACTUAL_FOLLOWING_ERROR;
    }
    if(FOLLOWING_ERROR_TIME){
        if(ABS(ACTUAL_FOLLOWING_ERROR) > FOLLOWING_ERROR_WINDOW){
            if(MotorControl.FollowingErrorTick == 0){
                MotorControl.FollowingErrorTick = get_tick();
            }else{
                if(get_ms_since(MotorControl.FollowingErrorTick) >= FOLLOWING_ERROR_TIME){
                    COM_CAN_report_err(ERR_FOLLOWING_ERROR);
                }
            }
        }else{
            MotorControl.FollowingErrorTick = 0;
        }
    }
}

static inline void velocity_ctrl_loop(void)
{
    const float current_limit = TORQUE_LIMIT / MOTOR_TORQUE_CONSTANT;
    const float vel_error = (MotorControl.velocity_set - Encoder.vel) * M_2PI / ENCODER_CPR_F;    // count/s -> rad/s
    MotorControl.current_set = PI_compute_serial(&MotorControl.PID_vel, vel_error, -current_limit, current_limit, -current_limit, current_limit, MotorControl.current_ff);
}

static inline void current_ctrl_loop(void)
{
    float iq_set = MotorControl.current_set;

    // Current ctrl
    float Vd = PI_compute_serial(&MotorControl.PID_Id,        - MotorControl.id_filtered, -MotorControl.MaxModulateVoltage, +MotorControl.MaxModulateVoltage, -MotorControl.MaxModulateVoltage, +MotorControl.MaxModulateVoltage, 0);
    float Vq = PI_compute_serial(&MotorControl.PID_Iq, iq_set - MotorControl.iq_filtered, -MotorControl.MaxModulateVoltage, +MotorControl.MaxModulateVoltage, -MotorControl.MaxModulateVoltage, +MotorControl.MaxModulateVoltage, 0);

    // Modulate
    MC_modulate(Vd, Vq, Encoder.phase + Encoder.phase_vel * CURRENT_CTRL_PERIOD);
}

static inline void set_phase_voltage(float Valpha, float Vbeta)
{
    uint8_t sector;

    // DQ Limit
    utils_saturate_vector_2d(&Valpha, &Vbeta, SQRT3_BY_2 * MAX_MODULATION);

    if (Vbeta >= 0.0f) {
        if (Valpha >= 0.0f) {
            //quadrant I
            if (ONE_BY_SQRT3 * Vbeta > Valpha)
                sector = 2; //sextant v2-v3
            else
                sector = 1; //sextant v1-v2
        } else {
            //quadrant II
            if (-ONE_BY_SQRT3 * Vbeta > Valpha)
                sector = 3; //sextant v3-v4
            else
                sector = 2; //sextant v2-v3
        }
    } else {
        if (Valpha >= 0.0f) {
            //quadrant IV
            if (-ONE_BY_SQRT3 * Vbeta > Valpha)
                sector = 5; //sextant v5-v6
            else
                sector = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (ONE_BY_SQRT3 * Vbeta > Valpha)
                sector = 4; //sextant v4-v5
            else
                sector = 5; //sextant v5-v6
        }
    }

    uint16_t CntPhA;
    uint16_t CntPhB;
    uint16_t CntPhC;

    switch (sector) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            uint32_t t1 = (Valpha - ONE_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;
            uint32_t t2 = (TWO_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;

            // PWM timings
            CntPhA = (PWM_PERIOD_CYCLES_DIV - t1 - t2) / 2;
            CntPhB = CntPhA + t1;
            CntPhC = CntPhB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            uint32_t t2 = (Valpha + ONE_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;
            uint32_t t3 = (-Valpha + ONE_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;

            // PWM timings
            CntPhB = (PWM_PERIOD_CYCLES_DIV - t2 - t3) / 2;
            CntPhA = CntPhB + t3;
            CntPhC = CntPhA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            uint32_t t3 = (TWO_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;
            uint32_t t4 = (-Valpha - ONE_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;

            // PWM timings
            CntPhB = (PWM_PERIOD_CYCLES_DIV - t3 - t4) / 2;
            CntPhC = CntPhB + t3;
            CntPhA = CntPhC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            uint32_t t4 = (-Valpha + ONE_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;
            uint32_t t5 = (-TWO_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;

            // PWM timings
            CntPhC = (PWM_PERIOD_CYCLES_DIV - t4 - t5) / 2;
            CntPhB = CntPhC + t5;
            CntPhA = CntPhB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            uint32_t t5 = (-Valpha - ONE_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;
            uint32_t t6 = (Valpha - ONE_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;

            // PWM timings
            CntPhC = (PWM_PERIOD_CYCLES_DIV - t5 - t6) / 2;
            CntPhA = CntPhC + t5;
            CntPhB = CntPhA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            uint32_t t6 = (-TWO_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;
            uint32_t t1 = (Valpha + ONE_BY_SQRT3 * Vbeta) * PWM_PERIOD_CYCLES_DIV;

            // PWM timings
            CntPhA = (PWM_PERIOD_CYCLES_DIV - t6 - t1) / 2;
            CntPhC = CntPhA + t1;
            CntPhB = CntPhC + t6;
        } break;
    }

    set_a_duty(CntPhA);
    set_b_duty(CntPhB);
    set_c_duty(CntPhC);
}

inline void MC_modulate(float Vd, float Vq, float phase)
{
    // Voltage_normalize = 1/(2/3*V_bus)
    const float V_to_mod = 1.5f / MotorControl.BusVoltage;
    MotorControl.mod_d = Vd * V_to_mod;
    MotorControl.mod_q = Vq * V_to_mod;

    float sint = arm_sin_f32(phase);
    float cost = arm_cos_f32(phase);
    
    // Inverse park transform
    float Valpha = cost * MotorControl.mod_d - sint * MotorControl.mod_q;
    float Vbeta  = sint * MotorControl.mod_d + cost * MotorControl.mod_q;

    // Modulate
    set_phase_voltage(Valpha, Vbeta);
}
