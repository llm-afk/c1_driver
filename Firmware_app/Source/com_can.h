#ifndef __COM_CAN_H__
#define __COM_CAN_H__

#include "soc.h"
#include "motor_ctrl.h"

// msgID | nodeID
// 4 bit  | 7 bit
#define GET_MSG_ID(canid)   (canid & 0x780)
#define GET_NODE_ID(canid)  (canid & 0x07F)

#define MSG_ID_SYNC         0x080   /**< 0x080, Synchronous message                   */
#define MSG_ID_EMERGENCY    0x080   /**< 0x080, Emergency messages          (+nodeID) */
#define MSG_ID_HEARTBEAT    0x700   /**< 0x700, Heartbeat message           (+nodeID) */

#define MSG_ID_SDO_SRV      0x580   /**< 0x580, SDO response from server    (+nodeID) */
#define MSG_ID_SDO_CLI      0x600   /**< 0x600, SDO request from client     (+nodeID) */

#define MSG_ID_RPDO_1       0x200   /**< 0x200, Default RPDO1               (+nodeID) */
#define MSG_ID_RPDO_2       0x300   /**< 0x300, Default RPDO2               (+nodeID) */
#define MSG_ID_RPDO_3       0x400   /**< 0x400, Default RPDO3               (+nodeID) */
#define MSG_ID_RPDO_4       0x500   /**< 0x500, Default RPDO5               (+nodeID) */

#define MSG_ID_TPDO_1       0x180   /**< 0x180, Default TPDO1               (+nodeID) */
#define MSG_ID_TPDO_2       0x280   /**< 0x280, Default TPDO2               (+nodeID) */
#define MSG_ID_TPDO_3       0x380   /**< 0x380, Default TPDO3               (+nodeID) */
#define MSG_ID_TPDO_4       0x480   /**< 0x480, Default TPDO4               (+nodeID) */

#define MSG_ID_DFU          0x680   /**< 0x680,                             (+nodeID) */
#define MSG_ID_PLOT         0x780   /**< 0x780,                             (+nodeID) */

#define CS_R        0x40
#define CS_R_ACK_1  0x4F
#define CS_R_ACK_2  0x4B
#define CS_R_ACK_3  0x47
#define CS_R_ACK_4  0x43
#define CS_W_1      0x2F
#define CS_W_2      0x2B
#define CS_W_3      0x27
#define CS_W_4      0x23
#define CS_W_ACK    0x60
#define CS_ERR      0x80

#define RX_FIFO_BUFFER_SIZE         (64 * sizeof(CanFrame))
#define TX_FIFO_BUFFER_SIZE         (64 * sizeof(CanFrame))
#define REPORT_FIFO_BUFFER_SIZE     (16 * sizeof(CanFrame))

void COM_CAN_init(void);
void COM_CAN_plot_value(float v1, float v2);
void COM_CAN_report_frame(CanFrame *frame);
void COM_CAN_report_err(tErrorCode err);
void COM_CAN_report_bootup(void);
void COM_CAN_loop(void);
void COM_CAN_rx_callback(void);

#endif
