#include "com_can.h"
#include "od.h"
#include "dfu.h"
#include "util.h"
#include "fifo_buffer.h"
#include "eeprom_emul.h"

static uint8_t mNodeID;
static uint16_t mHeartBeatProducerTime;
static uint16_t mHeartBeatConsumerTime;

static t_fifo_buffer rx_fifo[1];
static t_fifo_buffer tx_fifo[1];
static t_fifo_buffer report_fifo[1];
static uint8_t rx_buffer[RX_FIFO_BUFFER_SIZE];
static uint8_t tx_buffer[TX_FIFO_BUFFER_SIZE];
static uint8_t report_buffer[REPORT_FIFO_BUFFER_SIZE];

static uint32_t mHeartbeatProducerTick = 0;
static uint32_t mHeartbeatConsumerTick = 0;

static void parse_frame(CanFrame *frame);
static inline void send_to_host_or_enqueue(CanFrame *tx_frame);

void COM_CAN_init(void)
{
    fifoBuf_init(rx_fifo, rx_buffer, RX_FIFO_BUFFER_SIZE);
    fifoBuf_init(tx_fifo, tx_buffer, TX_FIFO_BUFFER_SIZE);
    fifoBuf_init(report_fifo, report_buffer, REPORT_FIFO_BUFFER_SIZE);

    mNodeID = ODObjs.node_id;
    mHeartBeatProducerTime = ODObjs.heartbeat_producer_time;
    mHeartBeatConsumerTime = ODObjs.heartbeat_consumer_time;
    
    SOC_can_init(ODObjs.can_baudrate, ODObjs.data_baudrate);
}

void COM_CAN_plot_value(float v1, float v2)
{
    CanFrame tx_frame;
    tx_frame.id = MSG_ID_PLOT + mNodeID;
    float_to_data(v1, &tx_frame.data[0]);
    float_to_data(v2, &tx_frame.data[4]);
    tx_frame.dlc = 8;
    send_to_host_or_enqueue(&tx_frame);
}

void COM_CAN_report_frame(CanFrame *frame)
{
    frame->id += mNodeID;
    fifoBuf_putData(report_fifo, frame, sizeof(CanFrame));
}

void COM_CAN_report_err(tErrorCode err)
{
    // Stop motor
    if(MC_get_state() != MCS_IDLE){
        SOC_pwm_disable();
        MC_set_state(MCS_IDLE);
    }

    SW_ERROR_SET();

    if(ERROR_CODE & err){
        return;
    }

    ENTER_CRITICAL();

    ERROR_CODE |= err;

    // Report error code
    CanFrame tx_frame;
    tx_frame.id = MSG_ID_EMERGENCY + mNodeID;
    tx_frame.dlc = 8;
    for(int i=0; i<8; i++){
        tx_frame.data[i] = 0;
    }
    *(uint16_t*)&tx_frame.data[0] = ERROR_CODE;
    fifoBuf_putData(report_fifo, &tx_frame, sizeof(CanFrame));

    EXIT_CRITICAL();
}

void COM_CAN_report_bootup(void)
{
    CanFrame tx_frame;
    tx_frame.id = MSG_ID_HEARTBEAT + mNodeID;
    tx_frame.dlc = 1;
    tx_frame.data[0] = 0;
    fifoBuf_putData(report_fifo, &tx_frame, sizeof(CanFrame));
}

void COM_CAN_loop(void)
{
    CanFrame frame;

    // Rx loop
    if(fifoBuf_getData(rx_fifo, &frame, sizeof(CanFrame))){
        parse_frame(&frame);
    }

    // Tx loop
    if(fifoBuf_getUsed(tx_fifo) && !SOC_can_is_tx_busy()){
        fifoBuf_getData(tx_fifo, &frame, sizeof(CanFrame));
        SOC_can_transmit(&frame);
    }

    // Report loop
    if(fifoBuf_getUsed(report_fifo) && !SOC_can_is_tx_busy()){
        fifoBuf_getData(report_fifo, &frame, sizeof(CanFrame));
        SOC_can_transmit(&frame);
    }

    // Heartbeat consumer loop
    if(mHeartBeatConsumerTime && mHeartbeatConsumerTick){
        if(get_ms_since(mHeartbeatConsumerTick) >= mHeartBeatConsumerTime){
            mHeartbeatConsumerTick = 0;
            COM_CAN_report_err(ERR_HEARTBEAT_TIMEOUT);
        }
    }

    // Heartbeat producer loop
    if(mHeartBeatProducerTime){
        if(get_ms_since(mHeartbeatProducerTick) >= mHeartBeatProducerTime){
            mHeartbeatProducerTick = get_tick();

            // Send heartbeat
            CanFrame tx_frame;
            tx_frame.id = MSG_ID_HEARTBEAT + mNodeID;
            tx_frame.dlc = 1;
            tx_frame.data[0] = 0x05;
            send_to_host_or_enqueue(&tx_frame);
        }
    }
}

static inline void send_to_host_or_enqueue(CanFrame *tx_frame)
{
    if(SOC_can_transmit(tx_frame)){
        fifoBuf_putData(tx_fifo, tx_frame, sizeof(CanFrame));
    }
}

static void parse_frame(CanFrame *frame)
{
    switch(GET_MSG_ID(frame->id)){
        case MSG_ID_SYNC:
            if(frame->dlc == 0){
                MC_target_sync();
            }
            break;
        
        case MSG_ID_HEARTBEAT:
            // heartbeat consumer
            if(mHeartBeatConsumerTime){
                if(GET_NODE_ID(frame->id) == 0 && frame->dlc == 1 && frame->data[0]){
                    mHeartbeatConsumerTick = get_tick();
                }
            }
            break;

        case MSG_ID_SDO_CLI:
            if(GET_NODE_ID(frame->id) == mNodeID && frame->dlc == 8) {
                uint8_t cs = frame->data[0];
                uint16_t idx = *(uint16_t*)&frame->data[1];
                uint8_t *data = &frame->data[4];
                
                frame->data[0] = CS_ERR;
                
                if(cs == CS_R){
                    frame->data[0] = OD_read(idx, data);
                }else if(cs == CS_W_1){
                    frame->data[0] = OD_write_1(idx, data);
                }else if(cs == CS_W_2){
                    frame->data[0] = OD_write_2(idx, data);
                }else if(cs == CS_W_4){
                    frame->data[0] = OD_write_4(idx, data);
                }
                
                frame->id = MSG_ID_SDO_SRV + mNodeID;
                
                send_to_host_or_enqueue(frame);
            }
            break;

        case MSG_ID_RPDO_1:
            if(GET_NODE_ID(frame->id) != mNodeID) break;
            MC_pdo_profile_position(*(float*)&frame->data[0], *(float*)&frame->data[4]);
            break;

        case MSG_ID_RPDO_2:
            if(GET_NODE_ID(frame->id) != mNodeID) break;
            MC_pdo_profile_velocity(*(float*)&frame->data[0], *(float*)&frame->data[4]);
            break;

        case MSG_ID_RPDO_3:
            if(GET_NODE_ID(frame->id) != mNodeID) break;
            MC_pdo_profile_torque(*(float*)&frame->data[0], *(float*)&frame->data[4]);
            break;

        case MSG_ID_RPDO_4:
            if(GET_NODE_ID(frame->id) != mNodeID) break;

            MC_pdo_interp_position(*(float*)&frame->data[0]);

            // tx msg
            frame->id = MSG_ID_TPDO_4 + mNodeID;
            frame->dlc = 6;
            *(float*)&frame->data[0] = ACTUAL_POSITION;
            *(uint16_t*)&frame->data[4] = STATUS_WORD;
            send_to_host_or_enqueue(frame);
            break;
        
        case MSG_ID_DFU:
            if(GET_NODE_ID(frame->id) != mNodeID) break;
        
            // Stop motor
            if(MC_get_state() != MCS_IDLE){
                SOC_pwm_disable();
                MC_set_state(MCS_IDLE);
            }

            if(frame->dlc == 4){
                if(0xDDDDDDDD == *(uint32_t*)&frame->data[0]){
                    if(0 == DFU_start()){
                        // ack
                        *(uint32_t*)&frame->data[0] = 0xDDDDDDDD;
                    }else{
                        // nak
                        *(uint32_t*)&frame->data[0] = 0x00000000;
                    }
                    *(uint32_t*)&frame->data[0] = 0xDDDDDDDD;
                }else
                
                if(0xFFFFFFFF == *(uint32_t*)&frame->data[0]){
                    // ack
                    *(uint32_t*)&frame->data[0] = 0xFFFFFFFF;
                    
                    SOC_can_transmit_block(frame);
                    DFU_jump_to_bootloader();
                }
                
                send_to_host_or_enqueue(frame);
            }else{
                DFU_data(&frame->data[0]);
        
                // dfu data ack
                frame->dlc = 0;
                send_to_host_or_enqueue(frame);
            }
            break;

        default:
            break;
    }
}

/******************************************************************************/
static bool can_rx(CanFrame *rx_frame)
{
    if((CAN_RFIFO0(CAN0) & CAN_RFIF_RFL_MASK) != 0){
        rx_frame->id = (0x000007FFU & (uint32_t)(GET_RFIFOMI_SFID(CAN_RFIFOMI(CAN0, 0))));
        
        // set CAN DLC
        uint8_t dlc = GET_RFIFOMP_DLENC(CAN_RFIFOMP(CAN0, 0));
        rx_frame->dlc = dlc_to_len_table[dlc];
        
        // set data
        uint8_t i = (dlc_to_len_table[dlc] + 3) >> 2;
        uint32_t *p_temp = (uint32_t*)rx_frame->data;
        for(; i>0U; i--){
            *p_temp = CAN_RFIFOMDATA0(CAN0, 0);
            p_temp ++;
        }
        
        // release FIFO
        CAN_RFIFO0(CAN0) |= CAN_RFIFO0_RFD0;
        
        return true;
    }
    
    return false;
}

void COM_CAN_rx_callback(void)
{
    CanFrame rxframe;
    while (can_rx(&rxframe)) {
        fifoBuf_putData(rx_fifo, &rxframe, sizeof(CanFrame));
    }
}
