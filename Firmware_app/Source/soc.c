#include "soc.h"

uint16_t adc_buff[3];
int16_t  phase_a_adc_offset = 0;
int16_t  phase_b_adc_offset = 0;
int16_t  phase_c_adc_offset = 0;

void SOC_init(void)
{
    // systick
    SysTick->LOAD  = 0xFFFFFFUL;
    SysTick->VAL   = 0;
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Disable ADC interrupt */
    adc_interrupt_disable(ADC0, ADC_INT_EOIC);
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);

    /* enable ADC0 */
    adc_enable(ADC0);
    /* Wait ADC0 startup */
    delay_ms(10);
    /* ADC0 calibration */
    adc_calibration_enable(ADC0);

    /* ADC software trigger enable */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);

    /* ADC0 inject convert complete interrupt */
    adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);

    /* enable ADC1 */
    adc_enable(ADC1);
    /* Wait ADC1 startup */
    delay_ms(10);
    /* ADC1 calibration */
    adc_calibration_enable(ADC1);

    /* Hold TIMER0 counter when core is halted */
    dbg_periph_enable(DBG_TIMER0_HOLD);

    /* Enable TIMER0 counter */
    timer_enable(TIMER0);

    timer_repetition_value_config(TIMER0, 1);

    /* Set all duty to 50% */
    set_a_duty(((uint32_t) PWM_PERIOD_CYCLES_DIV / (uint32_t) 2));
    set_b_duty(((uint32_t) PWM_PERIOD_CYCLES_DIV / (uint32_t) 2));
    set_c_duty(((uint32_t) PWM_PERIOD_CYCLES_DIV / (uint32_t) 2));

    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);

    /* Main PWM Output Enable */
    timer_primary_output_config(TIMER0, ENABLE);
}

void SOC_pwm_enable(void)
{
    /* Set all duty to 50% */
    set_a_duty(((uint32_t) PWM_PERIOD_CYCLES_DIV / (uint32_t) 2));
    set_b_duty(((uint32_t) PWM_PERIOD_CYCLES_DIV / (uint32_t) 2));
    set_c_duty(((uint32_t) PWM_PERIOD_CYCLES_DIV / (uint32_t) 2));

    /* wait for a new PWM period */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    while (RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)) {
    };
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);

    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_ENABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_ENABLE);
}

void SOC_pwm_disable(void)
{
    timer_channel_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCX_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_0, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_1, TIMER_CCXN_DISABLE);
    timer_channel_complementary_output_state_config(TIMER0, TIMER_CH_2, TIMER_CCXN_DISABLE);

    /* wait for a new PWM period */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    while (RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)) {
    };
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
}

#include "motor_ctrl.h"

int SOC_calibration_offset(void)
{
    int i         = 0;
    int adc_sum_a = 0;
    int adc_sum_b = 0;
    int adc_sum_c = 0;

    /* Clear Update Flag */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);
    /* Wait until next update */
    while (RESET == timer_flag_get(TIMER0, TIMER_FLAG_UP)) {
    };
    /* Clear Update Flag */
    timer_flag_clear(TIMER0, TIMER_FLAG_UP);

    while (i < 64) {
        if (timer_flag_get(TIMER0, TIMER_FLAG_UP) == SET) {
            timer_flag_clear(TIMER0, TIMER_FLAG_UP);

            i++;
            adc_sum_a += READ_IPHASE_A_ADC();
            adc_sum_b += READ_IPHASE_B_ADC();
            adc_sum_c += READ_IPHASE_C_ADC();
        }
    }

    phase_a_adc_offset = adc_sum_a / i;
    phase_b_adc_offset = adc_sum_b / i;
    phase_c_adc_offset = adc_sum_c / i;

    // offset check
    i                         = 0;
    const int Vout            = 1861;
    const int check_threshold = 500;
    if (phase_a_adc_offset > (Vout + check_threshold) || phase_a_adc_offset < (Vout - check_threshold)) {
        i = -1;
    }
    if (phase_b_adc_offset > (Vout + check_threshold) || phase_b_adc_offset < (Vout - check_threshold)) {
        i = -1;
    }
    if (phase_c_adc_offset > (Vout + check_threshold) || phase_c_adc_offset < (Vout - check_threshold)) {
        i = -1;
    }

    return i;
}

static void can_enable(uint16_t can_prescaler, uint8_t can_seg1, uint8_t can_seg2, uint8_t can_sjw, uint16_t data_prescaler, uint8_t data_seg1, uint8_t data_seg2, uint8_t data_sjw)
{
    can_parameter_struct can_parameter;
    can_fdframe_struct can_fd_parameter; 
    can_fd_tdc_struct can_fd_tdc_parameter;
    can_filter_parameter_struct can_filter_parameter;
    
    can_interrupt_disable(CAN0, CAN_INTEN_RFNEIE0);
    
    /* initialize CAN register */
    can_deinit(CAN0);
    
    // Reset CAN peripheral
    CAN_CTL(CAN0) |= CAN_CTL_SWRST;
    while ((CAN_CTL(CAN0) & CAN_CTL_SWRST) != 0)
        ; // reset bit is set to zero after reset
    while ((CAN_STAT(CAN0) & CAN_STAT_SLPWS) == 0)
        ; // should be in sleep mode after reset
    
    /* Enter initialize mode */
    can_working_mode_set(CAN0, CAN_MODE_INITIALIZE);
    
    /* initialize CAN parameters */
    can_parameter.working_mode = CAN_NORMAL_MODE;
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = ENABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = ENABLE;
    can_parameter.trans_fifo_order = ENABLE;
    can_parameter.resync_jump_width = can_sjw - 1;
    can_parameter.time_segment_1 = can_seg1 - 1;
    can_parameter.time_segment_2 = can_seg2 - 1;
    can_parameter.prescaler = can_prescaler;
    /* initialize CAN */
    can_init(CAN0, &can_parameter);
    
    can_fd_parameter.fd_frame = ENABLE;
    can_fd_parameter.excp_event_detect = ENABLE;
    can_fd_parameter.delay_compensation = ENABLE;
    can_fd_tdc_parameter.tdc_mode = CAN_TDCMOD_CALC_AND_OFFSET;
    can_fd_tdc_parameter.tdc_offset = 0x04;
    can_fd_tdc_parameter.tdc_filter = 0x04;
    can_fd_parameter.p_delay_compensation = &can_fd_tdc_parameter;
    can_fd_parameter.iso_bosch = CAN_FDMOD_ISO;
    can_fd_parameter.esi_mode = CAN_ESIMOD_HARDWARE;
    can_fd_parameter.data_prescaler = data_prescaler;
    can_fd_parameter.data_time_segment_1 = data_seg1 - 1;
    can_fd_parameter.data_time_segment_2 = data_seg2 - 1;
    can_fd_parameter.data_resync_jump_width = data_sjw - 1;
    /* initialize CAN FD */
    can_fd_init(CAN0, &can_fd_parameter);
    
    /* initialize filter parameters */
    can_filter_parameter.filter_fifo_number = CAN_FIFO0;
    can_filter_parameter.filter_bits = CAN_FILTERBITS_32BIT;
    /* initialize filter */
    can1_filter_start_bank(28);
    
    can_filter_parameter.filter_number    = 10;
    can_filter_parameter.filter_enable    = ENABLE;
    can_filter_parameter.filter_mode      = CAN_FILTERMODE_MASK;
    can_filter_parameter.filter_list_high = 0;
    can_filter_parameter.filter_list_low  = 0;
    can_filter_parameter.filter_mask_high = 0;
    can_filter_parameter.filter_mask_low  = 0;
    can_filter_init(&can_filter_parameter);
    
    // Reset Tx fifo
    can_transmission_stop(CAN0, CAN_MAILBOX0);
    can_transmission_stop(CAN0, CAN_MAILBOX1);
    can_transmission_stop(CAN0, CAN_MAILBOX2);
    
    // Reset Rx fifo
    while ((CAN_RFIFO0(CAN0) & CAN_RFIF_RFL_MASK) != 0) {
        CAN_RFIFO0(CAN0) |= CAN_RFIFO0_RFD0;
    }
    
    /* enable can receive FIFO0 not empty interrupt */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
    nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);
}

int SOC_can_init(int can_idx, int data_idx)
{
    int can_prescaler, can_seg1, can_seg2, can_sjw;
    int data_prescaler, data_seg1, data_seg2, data_sjw;

    switch (can_idx) {
    case 0: // 500K 60%
        can_prescaler  = 8;
        can_seg1 = 8;
        can_seg2 = 6;
        can_sjw  = 4;
        break;

    case 1: // 800K 66.7%
        can_prescaler  = 5;
        can_seg1 = 9;
        can_seg2 = 5;
        can_sjw  = 4;
        break;
    
    case 2: // 1000K 73.3%
        can_prescaler  = 4;
        can_seg1 = 10;
        can_seg2 = 4;
        can_sjw  = 4;
        break;

    default: // 1000K 73.3%
        can_prescaler  = 4;
        can_seg1 = 10;
        can_seg2 = 4;
        can_sjw  = 4;
        break;
    }

    switch (data_idx) {
    case 0: // 1M 73.3%
        data_prescaler  = 4;
        data_seg1 = 10;
        data_seg2 = 4;
        data_sjw  = 4;
        break;

    case 1: // 2M 73.3%
        data_prescaler  = 2;
        data_seg1 = 10;
        data_seg2 = 4;
        data_sjw  = 4;
        break;
    
    case 2: // 4M 73.3%
        data_prescaler  = 1;
        data_seg1 = 10;
        data_seg2 = 4;
        data_sjw  = 4;
        break;
    
    case 3: // 5M 75%
        data_prescaler  = 1;
        data_seg1 = 8;
        data_seg2 = 3;
        data_sjw  = 2;
        break;

    default: // 4M 73.3%
        data_prescaler  = 1;
        data_seg1 = 10;
        data_seg2 = 4;
        data_sjw  = 4;
        break;
    }
    
    can_enable(can_prescaler, can_seg1, can_seg2, can_sjw, data_prescaler, data_seg1, data_seg2, data_sjw);
    
    return 0;
}

int SOC_can_is_tx_busy(void)
{
    uint8_t mailbox_number;

    /* select one empty mailbox */
    if (CAN_TSTAT_TME0 == (CAN_TSTAT(CAN0) & CAN_TSTAT_TME0)) {
        mailbox_number = CAN_MAILBOX0;
    } else if (CAN_TSTAT_TME1 == (CAN_TSTAT(CAN0) & CAN_TSTAT_TME1)) {
        mailbox_number = CAN_MAILBOX1;
    } else if (CAN_TSTAT_TME2 == (CAN_TSTAT(CAN0) & CAN_TSTAT_TME2)) {
        mailbox_number = CAN_MAILBOX2;
    } else {
        mailbox_number = CAN_NOMAILBOX;
    }
    /* return no mailbox empty */
    if (CAN_NOMAILBOX == mailbox_number) {
        return 1;
    }
    
    return 0;
}

int SOC_can_transmit(CanFrame *frame)
{
    uint32_t *p_temp;
    uint32_t reg_temp;
	uint8_t mailbox_number;
	
	// select one empty mailbox
    if(CAN_TSTAT(CAN0) & CAN_TSTAT_TME0){
        mailbox_number = CAN_MAILBOX0;
    }else if(CAN_TSTAT(CAN0) & CAN_TSTAT_TME1){
        mailbox_number = CAN_MAILBOX1;
    }else if(CAN_TSTAT(CAN0) & CAN_TSTAT_TME2){
        mailbox_number = CAN_MAILBOX2;
    }else{
        mailbox_number = CAN_NOMAILBOX;
    }
    
    if(CAN_NOMAILBOX == mailbox_number){
        // no mailbox is empty
        return -1;
    }

    // disable transmission
    CAN_TMI(CAN0, mailbox_number) &= CAN_TMI_TEN;
    
    reg_temp = 0;
    
    // set standard can id
    reg_temp |= TMI_SFID(frame->id & 0x000007FFU);
	
    /* write TMI reg */
    CAN_TMI(CAN0, mailbox_number) = reg_temp;
	
    reg_temp = 0;
    
    // set CAN-FD
    reg_temp |= (1U<<7);
    
    // set bit rate switching
    reg_temp |= (1U<<5);
    
    uint8_t dlc = 0;
    if (frame->dlc <= 8) {
        dlc = frame->dlc;
    } else if (frame->dlc <= 12) {
        dlc = 9;
    } else if (frame->dlc <= 16) {
        dlc = 10;
    } else if (frame->dlc <= 20) {
        dlc = 11;
    } else if (frame->dlc <= 24) {
        dlc = 12;
    } else if (frame->dlc <= 32) {
        dlc = 13;
    } else if (frame->dlc <= 48) {
        dlc = 14;
    } else if (frame->dlc <= 64) {
        dlc = 15;
    }
    
    // set data length
    reg_temp |= dlc;
    
    // write TMP reg
    CAN_TMP(CAN0, mailbox_number) = reg_temp;

    // set data
    uint8_t i = (dlc_to_len_table[dlc] + 3) >> 2;
    p_temp = (uint32_t*)frame->data;
    for(; i>0U; i--){
        CAN_TMDATA0(CAN0, mailbox_number) = *p_temp;
        p_temp ++;
    }
	
	// enable transmission
    CAN_TMI(CAN0, mailbox_number) |= CAN_TMI_TEN;
	
	return 0;
}

int SOC_can_transmit_block(CanFrame *frame)
{
    int ret;
    
    ret = SOC_can_transmit(frame);
    if(ret){
        return ret;
    }
    
    delay_ms(1);
    
    return 0;
}
