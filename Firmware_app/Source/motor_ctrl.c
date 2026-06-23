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
uint32_t state_mcs;
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
static inline void current_ctrl_loop(void);
static inline void set_phase_voltage(float Valpha, float Vbeta);

double coef5[] = {1.14787755, -0.0145798348, 0.00117647176, -0.0000548104756, 0.000000739631173};
int n5 = sizeof(coef5)/sizeof(coef5[0]);
double dcoef5[5];
// y = c1*x + c2*x^2 + ... + cn*x^n
double poly_eval(const double *coef, int n, double x) {
    double result = 0.0;
    double xn = x;
    for (int i = 0; i < n; i++) {
        result += coef[i] * xn;
        xn *= x;
    }
    return result;
}

// ========== ?? -> ?? ==========
// coef: ----
// n: ????
// current: ????
double current_to_torque(const double *coef, int n, double current) {
    return poly_eval(coef, n, current);
}

// ========== ?? -> ?? ==========
// ????????:?? f(I) = Torque ? I
// coef: ??->????????
// dcoef: ???????
// n: ??
// torque: ??????
double torque_to_current(const double *coef, const double *dcoef, int n, double torque) {
    double x = torque / coef[0];  // ??(????)
    for (int i = 0; i < 10; i++) {
        double fx = poly_eval(coef, n, x) - torque;
        double dfx = poly_eval(dcoef, n - 1, x);
        if (fabs(dfx) < 1e-10) break; // ???0
        x -= fx / dfx;
    }
    return x;
}

// ========== ???????? ==========
void poly_derivative(const double *coef, double *dcoef, int n) {
    for (int i = 0; i < n - 1; i++) {
        dcoef[i] = coef[i] * (i + 1);
    }
}

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
    poly_derivative(coef5, dcoef5, n5);
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
extern  uint16_t mHeartBeatConsumerTime;
extern  uint32_t mHeartbeatConsumerTick ;
int MC_controlword_update(void)
{
	state_mcs ++;
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
        case CW_CMD_DEV_MULTI_CALIB:
						MC_set_state(MCS_EX_ENCODER_CHECK);
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
uint32_t check_start = 0;
int MC_set_state(tMCState state)
{
//		state_mcs = mFSM.state_next;

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
            if(ERROR_IS_SET(ERR_NO_SN)){
                return -1; // 未授权硬件严禁执行校准
            }
//            if(ERROR_CODE & (~ERR_ENC_CALIB)){
//                return -1;
//            }

            mFSM.state_next = state;
            mFSM.state_next_ready = 0;
            break;
        case MCS_EX_ENCODER_CHECK:
        {
//            if(ERROR_CODE & (~ERR_ENC_CALIB)){
//                return -1;
//            }
							uint16_t idx = 0x2003;
							uint8_t data = 3;
							OD_write_1(idx, &data);
							idx = 0x2002;
							uint8_t enable_data[4] = {0x01, 0x00,0x00,0x00};
							OD_write_2(idx, enable_data);

            break;
        }

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
        // 只清除可恢复错误，保留硬错误（SN锁、编码器未标定、ADC故障）
        ERROR_CODE &= (ERR_NO_SN | ERR_ENC_CALIB | ERR_ADC_SELFTEST);

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

        MP_trapezoid_pos_calculate(p0, MotorControl.position_cmd, v0, 0, mProfileVelocity, 1e6, 1e6);
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

        MP_trapezoid_vel_calculate(v0, v1, 1e6, 1e6);
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
        MP_trapezoid_torque_calculate(t0, t1, 1e6);
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
float raw_pos;
float raw_vel;
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
		float s = (MotorControl.iq_filtered >= 0.0f) ? 1.0f : -1.0f;

    if(POLARITY){

//        ACTUAL_TORQUE   = - s*current_to_torque_5(fabsf(MotorControl.iq_filtered));
                ACTUAL_TORQUE   = - IQ_TO_TORQUE(MotorControl.iq_filtered);
				ACTUAL_VELOCITY = - INTER_TO_USR(Encoder.vel);	
        ACTUAL_POSITION = - INTER_TO_USR(Encoder.shadow_count);
    }else{
//        ACTUAL_TORQUE   = + s*current_to_torque_5(fabsf(MotorControl.iq_filtered));
				ACTUAL_TORQUE   = + IQ_TO_TORQUE(MotorControl.iq_filtered);
				ACTUAL_VELOCITY = + INTER_TO_USR(Encoder.vel);
				ACTUAL_POSITION = + INTER_TO_USR(Encoder.shadow_count);
    }

    DC_LINK_CURRENT = MotorControl.i_bus;
    ELECTRICAL_POWER = MotorControl.electrical_power;
    MECHANICAL_POWER = MotorControl.mechanical_power;
    
    // Protect check ========================================================
    // Over current check
    // if(ABS(MotorControl.Ia) > OVER_CURRENT_LEVEL || ABS(MotorControl.Ib) > OVER_CURRENT_LEVEL || ABS(MotorControl.Ic) > OVER_CURRENT_LEVEL){
    //     COM_CAN_report_err(ERR_OVER_CURRENT_SOFT);
    // }

//    // Over voltage check
//    if(DC_LINK_VOLTAGE > OVER_VOLTAGE_LEVEL){
//        COM_CAN_report_err(ERR_OVER_VOLTAGE);
//    }

    // Under voltage check
//    if(DC_LINK_VOLTAGE < UNDER_VOLTAGE_LEVEL){
//        COM_CAN_report_err(ERR_UNDER_VOLTAGE);
//    }
    // Protect check ========================================================
//				        MC_set_state(MCS_OPERATION);
//								multi_check_flag = 1;
    switch(mFSM.state){
        case MCS_OPERATION:

            MotorControl.i_bus = MotorControl.mod_d * MotorControl.Id + MotorControl.mod_q * MotorControl.iq_filtered;
            MotorControl.electrical_power = MotorControl.BusVoltage * MotorControl.i_bus;
            MotorControl.mechanical_power = IQ_TO_TORQUE(MotorControl.iq_filtered) * M_2PI * Encoder.vel / ENCODER_CPR_F;  // P = 2�� * Torque * Vel(r/s)

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

#define ENCODER_RESOLUTION 16384     // 2^14
#define TWO_PI             6.28318530718f

// 每个编码器计数对应负载端弧度
#define RAD_PER_COUNT  (TWO_PI / (ENCODER_RESOLUTION * g_gear_ratio))

// 编码器位置计数 -> 负载端角度 (rad)
float encoder_position_to_rad(int32_t pos_count) {
    return pos_count * RAD_PER_COUNT;
}

// 编码器速度计数 -> 负载端角速度 (rad/s)
float encoder_speed_to_rad_per_sec(int32_t speed_count) {
    return speed_count * RAD_PER_COUNT;
}

#define VCC         3.3f           // 分压上拉电压
#define ADC_MAX     4095.0f        // 12-bit ADC
#define R_FIXED     10000.0f       // R60 = 10kΩ
#define R_25        10000.0f      // NTC阻值@25℃
#define B_VALUE     3950.0f        // B25/50 值
typedef struct {
    float temperature;   // °C
    float resistance;    // Ω (center value from datasheet)
} ntc_lookup_t;

static const ntc_lookup_t ntc_table[] = {
    {  -55, 749200.000 },
    {  -54, 699508.000 },
    {  -53, 653455.000 },
    {  -52, 610733.000 },
    {  -51, 571067.000 },
    {  -50, 534215.000 },
    {  -49, 499957.000 },
    {  -48, 468095.000 },
    {  -47, 438448.000 },
    {  -46, 410851.000 },
    {  -45, 385154.000 },
    {  -44, 361217.000 },
    {  -43, 338913.000 },
    {  -42, 318123.000 },
    {  -41, 298739.000 },
    {  -40, 280660.000 },
    {  -39, 263791.000 },
    {  -38, 248046.000 },
    {  -37, 233346.000 },
    {  -36, 219615.000 },
    {  -35, 206785.000 },
    {  -34, 194792.000 },
    {  -33, 183576.000 },
    {  -32, 173082.000 },
    {  -31, 163260.000 },
    {  -30, 154062.000 },
    {  -29, 145446.000 },
    {  -28, 137369.000 },
    {  -27, 129795.000 },
    {  -26, 122689.000 },
    {  -25, 116020.000 },
    {  -24, 109756.000 },
    {  -23, 103870.000 },
    {  -22,  98338.000 },
    {  -21,  93135.000 },
    {  -20,  88238.000 },
    {  -19,  83629.000 },
    {  -18,  79288.000 },
    {  -17,  75197.000 },
    {  -16,  71341.000 },
    {  -15,  67704.000 },
    {  -14,  64272.000 },
    {  -13,  61032.000 },
    {  -12,  57972.000 },
    {  -11,  55082.000 },
    {  -10,  52350.000 },
    {   -9,  49766.000 },
    {   -8,  47322.000 },
    {   -7,  45010.000 },
    {   -6,  42821.000 },
    {   -5,  40748.000 },
    {   -4,  38785.000 },
    {   -3,  36925.000 },
    {   -2,  35161.000 },
    {   -1,  33489.000 },
    {    0,  32049.000 },
    {    1,  30399.000 },
    {    2,  28972.000 },
    {    3,  27617.000 },
    {    4,  26330.000 },
    {    5,  25109.000 },
    {    6,  23948.000 },
    {    7,  22846.000 },
    {    8,  21798.000 },
    {    9,  20802.000 },
    {   10,  19856.000 },
    {   11,  18955.000 },
    {   12,  18099.000 },
    {   13,  17285.000 },
    {   14,  16510.000 },
    {   15,  15773.000 },
    {   16,  15071.000 },
    {   17,  14403.000 },
    {   18,  13767.000 },
    {   19,  13162.000 },
    {   20,  12585.000 },
    {   21,  12036.000 },
    {   22,  11513.000 },
    {   23,  11015.000 },
    {   24,  10540.000 },
    {   25,  10000.000 },
    {   26,   9656.000 },
    {   27,   9245.000 },
    {   28,   8853.000 },
    {   29,   8479.000 },
    {   30,   8123.000 },
    {   31,   7783.000 },
    {   32,   7459.000 },
    {   33,   7149.000 },
    {   34,   6854.000 },
    {   35,   6573.000 },
    {   36,   6304.000 },
    {   37,   6047.000 },
    {   38,   5802.000 },
    {   39,   5568.000 },
    {   40,   5345.000 },
    {   41,   5131.000 },
    {   42,   4927.000 },
    {   43,   4733.000 },
    {   44,   4546.000 },
    {   45,   4368.000 },
    {   46,   4198.000 },
    {   47,   4036.000 },
    {   48,   3880.000 },
    {   49,   3732.000 },
    {   50,   3590.000 },
    {   51,   3453.000 },
    {   52,   3323.000 },
    {   53,   3199.000 },
    {   54,   3079.000 },
    {   55,   2965.000 },
    {   56,   2856.000 },
    {   57,   2751.000 },
    {   58,   2651.000 },
    {   59,   2555.000 },
    {   60,   2463.000 },
    {   61,   2375.000 },
    {   62,   2290.000 },
    {   63,   2209.000 },
    {   64,   2131.000 },
    {   65,   2056.000 },
    {   66,   1985.000 },
    {   67,   1916.000 },
    {   68,   1850.000 },
    {   69,   1787.000 },
    {   70,   1726.000 },
    {   71,   1668.000 },
    {   72,   1612.000 },
    {   73,   1558.000 },
    {   74,   1506.000 },
    {   75,   1456.000 },
    {   76,   1408.000 },
    {   77,   1362.000 },
    {   78,   1318.000 },
    {   79,   1276.000 },
    {   80,   1235.000 },
    {   81,   1195.000 },
    {   82,   1157.000 },
    {   83,   1121.000 },
    {   84,   1085.000 },
    {   85,   1052.000 },
    {   86,   1019.000 },
    {   87,    987.000 },
    {   88,    957.000 },
    {   89,    928.000 },
    {   90,    899.000 },
    {   91,    872.000 },
    {   92,    846.000 },
    {   93,    820.000 },
    {   94,    796.000 },
    {   95,    772.000 },
    {   96,    749.000 },
    {   97,    727.000 },
    {   98,    706.000 },
    {   99,    685.000 },
    {  100,    666.000 },
    {  101,    646.000 },
    {  102,    628.000 },
    {  103,    610.000 },
    {  104,    592.000 },
    {  105,    575.000 },
    {  106,    559.000 },
    {  107,    543.000 },
    {  108,    528.000 },
    {  109,    513.000 },
    {  110,    499.000 },
    {  111,    485.000 },
    {  112,    471.000 },
    {  113,    458.000 },
    {  114,    445.000 },
    {  115,    433.000 },
    {  116,    421.000 },
    {  117,    410.000 },
    {  118,    398.000 },
    {  119,    388.000 },
    {  120,    377.000 },
    {  121,    367.000 },
    {  122,    357.000 },
    {  123,    347.000 },
    {  124,    338.000 },
    {  125,    329.000 },
    {  126,    320.000 },
    {  127,    311.000 },
    {  128,    303.000 },
    {  129,    294.000 },
    {  130,    287.000 },
    {  131,    279.000 },
    {  132,    271.000 },
    {  133,    264.000 },
    {  134,    257.000 },
    {  135,    250.000 },
    {  136,    243.000 },
    {  137,    237.000 },
    {  138,    230.000 },
    {  139,    224.000 },
    {  140,    218.000 },
    {  141,    212.000 },
    {  142,    207.000 },
    {  143,    201.000 },
    {  144,    196.000 },
    {  145,    190.000 },
    {  146,    185.000 },
    {  147,    180.000 },
    {  148,    175.000 },
    {  149,    171.000 },
    {  150,    166.000 },
};

#define ntc_table_len (sizeof(ntc_table)/sizeof(ntc_table[0]))


// ⚙️ 由ADC值计算NTC电阻
static inline float calc_ntc_resistance(uint16_t adc_value) {
    float v_out = (float)adc_value / ADC_MAX * VCC;
    return (v_out * R_FIXED) / (VCC - v_out);
}

// ⚙️ 由NTC电阻计算温度（单位：摄氏度）
static inline float calc_temperature_celsius(float r_ntc) {
    float temp_k = 1.0f / (1.0f / (25.0f + 273.15f) + logf(r_ntc / R_25) / B_VALUE);
    return temp_k - 273.15f;
}

static inline float lookup_ntc_temperature(float r_ntc) {
    for (int i = 0; i < ntc_table_len - 1; i++) {
        float r1 = ntc_table[i].resistance;
        float r2 = ntc_table[i + 1].resistance;

        if (r_ntc <= r1 && r_ntc >= r2) {
            float t1 = ntc_table[i].temperature;
            float t2 = ntc_table[i + 1].temperature;
            return t1 + (r_ntc - r1) / (r2 - r1) * (t2 - t1);
        }
    }

    // 越界情况
    if (r_ntc > ntc_table[0].resistance)
        return ntc_table[0].temperature;
    else
        return ntc_table[ntc_table_len - 1].temperature;
}

static inline float get_ntc_temperature(void) {
    uint16_t adc = adc_buff[1];
    float r_ntc = calc_ntc_resistance(adc);
    return lookup_ntc_temperature(r_ntc);  // ← 查表插值计算
}
// ✅ 主函数调用：获取温度值
//static inline float get_ntc_temperature(void) {
//    uint16_t adc = adc_buff[1];
//    float r_ntc = calc_ntc_resistance(adc);
//    return calc_temperature_celsius(r_ntc);
//}
float tau_l;
float pos_err;
float vel_err;
float head_tor;
extern float raw_rad_data;
extern	int init_in;
extern	uint16_t init_ex;
extern	uint16_t init_in_offset;
extern	uint16_t init_ex_offset;
extern int raw_encoder;
extern float Real_Velocity;
extern float Velocity_Filtered;

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

        // // Dual Encoder fault check (0.1 rad tolerance ~ 5.7 deg on inner shaft)
        // uint16_t enc_fault = ENCODER_slip_check(0.1f);
        // if(enc_fault != 0){
        //     COM_CAN_report_err((tErrorCode)enc_fault);
        // }

				if(ERROR_CODE && !(ERROR_CODE & ERR_HEARTBEAT_TIMEOUT)){
					//MC_error_reset();
				}
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
        MOTOR_TEMPERATURE = get_ntc_temperature();
        if(MOTOR_TEMPERATURE > OVER_TEMP_MOTOR_LEVEL){
            COM_CAN_report_err(ERR_OVER_TEMP_MOTOR);
        }
        
        // Over load check
        float over_torque = ABS(ACTUAL_TORQUE) - MOTOR_RATED_TORQUE;
        if(over_torque > 0){
            over_torque_dpp += over_torque;
            if(over_torque_dpp > OVER_LOAD_DPP_LEVEL){
                //COM_CAN_report_err(ERR_OVER_LOAD);
            }
        }else{
            over_torque_dpp = 0;
        }

        /*
        │ 电流 I (A)  │ 净发热 I²−400 │ 保护时间  │                │
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     20     │       0       │    ∞     │ 额定，永远不跳 
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     21     │      41       │  122 s   │ 2 分 02 秒     
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     22     │      84       │   60 s   │ 1 分           
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     23     │      129      │   39 s   │                │
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     24     │      176      │   28 s   │                │
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     25     │      225      │   22 s   │                │
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     26     │      276      │   18 s   │                │
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     27     │      329      │   15 s   │                │
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     28     │      384      │   13 s   │                │
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     29     │      441      │   11 s   │                │
        ├────────────┼───────────────┼──────────┼────────────────┤
        │     30     │      500      │   10 s   │ 标定点      
        ├────────────┼───────────────┼──────────┼────────────────┤
        */

        // I²t overcurrent protection (soft)
        {
            static float i2t_acc = 0.0f;
            const float dt = 0.01f; // 100Hz

            switch(g_current_branch) 
            {
                case BRANCH_C2_NEW: 
                {
                    const float t_trip_test = 30.0f;
                    const float t_rated     = 20.0f;
                    const float t_trip_time = 10.0f;
                    const float i2t_threshold = (t_trip_test * t_trip_test - t_rated * t_rated) * t_trip_time;

                    float current_sq = MotorControl.id_filtered * MotorControl.id_filtered
                                     + MotorControl.iq_filtered * MotorControl.iq_filtered;
                    i2t_acc += (current_sq - t_rated * t_rated) * dt;

                    if(i2t_acc < 0.0f) i2t_acc = 0.0f;

                    if(!ERROR_IS_SET(ERR_OVER_CURRENT_SOFT) && (i2t_acc > i2t_threshold)) 
                    {
                        COM_CAN_report_err(ERR_OVER_CURRENT_SOFT);
                    }
                    break;
                }
                case BRANCH_C2_PRO:
                {
                    break;
                }
                case BRANCH_C2_PRO_XINZHI:
                {
                    break;
                }
                case BRANCH_A2:
                {
                    break;
                }
                case BRANCH_A2_XINZHI:
                {
                    break;
                }
                default: 
                {
                    break;
                }
            }
        }
        // in encodedr value update
        IN_ENCODER_VALUE = Encoder.count_in_cpr;
        
        // ex encoder value update
//        EX_ENCODER_VALUE = ENCODER_EX_read();
			
    }
    
    // 2000Hz for plot
    if(PLOT_CTRL){
        if(get_us_since(tick_2000) >= 500){
            tick_2000 = get_timestamp();

            float curr_value = 0;
            
            switch(PLOT_CTRL){
                case 1:
                    curr_value =ACTUAL_TORQUE ;
                    break;
                case 2:
                    curr_value = MotorControl.current_set;
                    break;
                case 3:
                    curr_value = raw_rad_data;
                    break;
                case 4:
                    curr_value =  init_ex_offset;
                    break;
                case 5:
                    curr_value = raw_encoder;
                    break;
                case 6:
                    curr_value = phase_a_adc_offset;
                    break;
                case 7:
                    curr_value = phase_b_adc_offset;
                    break;
                case 8:
                    curr_value = 0;
                    break;
                case 9:
                    curr_value = IN_ENCODER_OFFSET;
                    break;
                case 10:
                    curr_value = EX_ENCODER_OFFSET;
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


static inline void motor_mit_control(void)
{
    if(ERROR_CODE & ERR_NO_SN) {
        MotorControl.current_set = 0;
        return; // 未授权硬件严禁执行 MIT 运算和发力
    }


	MotorControl.raw_pos = raw_rad_data;
		MotorControl.raw_vel = Velocity_Filtered;

		pos_err = (MotorControl.pos_set - raw_rad_data)/1.0f;
		vel_err = (MotorControl.velocity_set - Velocity_Filtered)/1.0f;
	

	  tau_l =
      MotorControl.Kp * pos_err +
        MotorControl.Kd * vel_err + MotorControl.current_mit;  // ⭐ 前馈力矩
		tau_l = CLAMP(tau_l, -TORQUE_LIMIT, +TORQUE_LIMIT);
		float iq_target = TORQUE_TO_IQ(tau_l);
		iq_target = CLAMP(iq_target, -PEAK_IQ_CURRENT, +PEAK_IQ_CURRENT);
		MotorControl.current_set = iq_target;
		
//		MotorControl.enabled_loop = ENABLED_LOOP_CURRENT;


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
    if(tick == 0)
    {
        EX_ENCODER_VALUE = ENCODER_EX_read();
        motor_mit_control();
        servo_loop();
    }

    // 20KHz
    if(MotorControl.enabled_loop & ENABLED_LOOP_CURRENT){
        current_ctrl_loop();
    }
}

static inline void current_ctrl_loop(void)
{
    float iq_set = MotorControl.current_set;
//    float iq_set = MotorControl.current_mit;
    iq_set = CLAMP(iq_set, -PEAK_IQ_CURRENT, +PEAK_IQ_CURRENT);

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
