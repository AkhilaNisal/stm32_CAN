#include "fs_ai_protocol.h"
#include <string.h>

volatile int16_t AI_Steer_Request = 0;
volatile uint8_t AI_Brake_Request_F = 0;
volatile uint8_t AI_Brake_Request_R = 0;

void FS_AI_TransmitStatus(VCU_Status_t *status) {
    uCAN_MSG msg;
    msg.frame.id = ID_VCU2AI_STATUS;
    msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg.frame.dlc = 8;

    /* Correct way to clear: access union 'array' or individual data bytes */
    msg.frame.data0 = 0; msg.frame.data1 = 0; msg.frame.data2 = 0; msg.frame.data3 = 0;
    msg.frame.data4 = 0; msg.frame.data5 = 0; msg.frame.data6 = 0; msg.frame.data7 = 0;

    // Byte 0: Handshake(0), Shutdown(1), AS_sw(2), TS_sw(3), Go(4), Steer_status(5)
    msg.frame.data0 = (status->handshake & 0x01) |
                      ((status->shutdown_request & 0x01) << 1) |
                      ((status->as_switch_status & 0x01) << 2) |
                      ((status->ts_switch_status & 0x01) << 3) |
                      ((status->go_signal & 0x01) << 4) |
                      ((status->steering_status & 0x01) << 5);

    // Byte 1: AS_State (Bits 0-2), AMI_State (Bits 4-6)
    msg.frame.data1 = (status->as_state & 0x07) |
                      ((status->ami_state & 0x07) << 4);

    CANSPI_Transmit(&msg);
}

void FS_AI_ProcessIncoming(uCAN_MSG *rxMsg) {
    if (rxMsg->frame.id == ID_AI2VCU_STEER) {
        // ID 0x513: data0 is Low Byte, data1 is High Byte (Little Endian)
        AI_Steer_Request = (int16_t)((rxMsg->frame.data1 << 8) | rxMsg->frame.data0);
    }
    else if (rxMsg->frame.id == ID_AI2VCU_BRAKE) {
        // ID 0x514
        AI_Brake_Request_F = rxMsg->frame.data0;
        AI_Brake_Request_R = rxMsg->frame.data1;
    }
    // Add other cases (0x510, 0x511, 0x512) as needed
}
