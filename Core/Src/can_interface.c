/**
 * @file    can_interface.c
 * @brief   CAN Hardware Abstraction Layer implementation
 */

#include "can_interface.h"
#include <string.h>

/* ============================================================================
   STATIC DATA
   ============================================================================ */

static struct {
    can_interface_stats_t stats;
    uint32_t last_rx_time_ms;
    uint32_t last_watchdog_time_ms;
    fs_ai_vcu_status_t last_status;
    fs_ai_vcu_motor_feedback_t last_motor;
    fs_ai_vcu_steer_feedback_t last_steer;
    fs_ai_vcu_brake_feedback_t last_brake;
    fs_ai_vcu_wheel_speeds_t last_speeds;
    bool handshake_complete;
} can_state = {0};

/* ============================================================================
   INITIALIZATION
   ============================================================================ */

bool can_interface_init(void)
{
    /* Initialize MCP2515 via CANSPI driver */
    if (!CANSPI_Initialize()) {
        return false;
    }

    /* Initialize state */
    memset(&can_state, 0, sizeof(can_state));
    can_state.last_rx_time_ms = HAL_GetTick();
    can_state.last_watchdog_time_ms = HAL_GetTick();

    return true;
}

void can_interface_deinit(void)
{
    CANSPI_Sleep();
}

void can_interface_recover(void)
{
    /* Read error flags */
    uint8_t eflg = CANSPI_ReadEFLG();

    /* If bus-off detected, perform recovery */
    if (eflg & 0x20) {  /* TXBO bit (bit 5) */
        CANSPI_RecoverIfNeeded();
    }
}

/* ============================================================================
   TRANSMISSION
   ============================================================================ */

bool can_interface_send(const uCAN_MSG *msg, uint32_t timeout_ms)
{
    if (msg == NULL) {
        return false;
    }

    /* Use CANSPI_TransmitWait with timeout */
    uint8_t result = CANSPI_TransmitWait((uCAN_MSG *)msg, timeout_ms);

    if (result) {
        can_state.stats.tx_count++;
        return true;
    }

    return false;
}

bool can_interface_send_all(const fs_ai_ai_data_t *ai_data)
{
    if (ai_data == NULL) {
        return false;
    }

    bool all_sent = true;
    uCAN_MSG msg;

    /* Encode and send AI2VCU_Status */
    if (fs_ai_protocol_encode_status(&ai_data->status, &msg)) {
        if (!can_interface_send(&msg, 5)) {
            all_sent = false;
        }
    }

    /* Encode and send AI2VCU_Drive_F */
    if (fs_ai_protocol_encode_drive_f(&ai_data->control, &msg)) {
        if (!can_interface_send(&msg, 5)) {
            all_sent = false;
        }
    }

    /* Encode and send AI2VCU_Drive_R */
    if (fs_ai_protocol_encode_drive_r(&ai_data->control, &msg)) {
        if (!can_interface_send(&msg, 5)) {
            all_sent = false;
        }
    }

    /* Encode and send AI2VCU_Steer */
    if (fs_ai_protocol_encode_steer(&ai_data->control, &msg)) {
        if (!can_interface_send(&msg, 5)) {
            all_sent = false;
        }
    }

    /* Encode and send AI2VCU_Brake */
    if (fs_ai_protocol_encode_brake(&ai_data->control, &msg)) {
        if (!can_interface_send(&msg, 5)) {
            all_sent = false;
        }
    }

    /* Encode and send AI2LOG_Dynamics */
    if (fs_ai_protocol_encode_dynamics(&ai_data->dynamics, &msg)) {
        if (!can_interface_send(&msg, 5)) {
            all_sent = false;
        }
    }

    return all_sent;
}

/* ============================================================================
   RECEPTION
   ============================================================================ */

bool can_interface_receive(fs_ai_vcu_data_t *vcu_data)
{
    uCAN_MSG rxMsg;

    /* Attempt to receive a message */
    if (!CANSPI_Receive(&rxMsg)) {
        return false;  /* No message received */
    }

    /* Update RX timestamp */
    can_state.last_rx_time_ms = HAL_GetTick();
    can_state.stats.rx_count++;

    /* Decode based on CAN ID */
    switch (rxMsg.frame.id) {
        case CAN_ID_VCU2AI_STATUS:
            if (fs_ai_protocol_decode_status(&rxMsg, &can_state.last_status)) {
                can_state.handshake_complete =
                    (can_state.last_status.handshake_bit == HANDSHAKE_ON);
                if (vcu_data) {
                    vcu_data->status = can_state.last_status;
                }
            } else {
                can_state.stats.rx_errors++;
            }
            break;

        case CAN_ID_VCU2AI_DRIVE_F:
            if (fs_ai_protocol_decode_drive_f(&rxMsg, &can_state.last_motor)) {
                if (vcu_data) {
                    vcu_data->motor = can_state.last_motor;
                }
            } else {
                can_state.stats.rx_errors++;
            }
            break;

        case CAN_ID_VCU2AI_DRIVE_R:
            if (fs_ai_protocol_decode_drive_r(&rxMsg, &can_state.last_motor)) {
                if (vcu_data) {
                    vcu_data->motor = can_state.last_motor;
                }
            } else {
                can_state.stats.rx_errors++;
            }
            break;

        case CAN_ID_VCU2AI_STEER:
            if (fs_ai_protocol_decode_steer(&rxMsg, &can_state.last_steer)) {
                if (vcu_data) {
                    vcu_data->steer = can_state.last_steer;
                }
            } else {
                can_state.stats.rx_errors++;
            }
            break;

        case CAN_ID_VCU2AI_BRAKE:
            if (fs_ai_protocol_decode_brake(&rxMsg, &can_state.last_brake)) {
                if (vcu_data) {
                    vcu_data->brake = can_state.last_brake;
                }
            } else {
                can_state.stats.rx_errors++;
            }
            break;

        case CAN_ID_VCU2AI_SPEEDS:
            if (fs_ai_protocol_decode_speeds(&rxMsg, &can_state.last_speeds)) {
                if (vcu_data) {
                    vcu_data->speeds = can_state.last_speeds;
                }
            } else {
                can_state.stats.rx_errors++;
            }
            break;

        default:
            /* Unhandled message ID */
            can_state.stats.rx_errors++;
            break;
    }

    return true;
}

uint8_t can_interface_messages_available(void)
{
    return CANSPI_messagesInBuffer();
}

bool can_interface_process_all(fs_ai_vcu_data_t *vcu_data)
{
    bool any_received = false;

    /* Process all messages in RX buffers */
    while (CANSPI_messagesInBuffer() > 0) {
        if (can_interface_receive(vcu_data)) {
            any_received = true;
        }
    }

    return any_received;
}

/* ============================================================================
   HEALTH MONITORING
   ============================================================================ */

can_health_status_t can_interface_get_health(void)
{
    /* Check for bus-off condition */
    if (CANSPI_isBussOff()) {
        return CAN_HEALTH_BUS_OFF;
    }

    /* Check for error passive */
    if (CANSPI_isRxErrorPassive() || CANSPI_isTxErrorPassive()) {
        return CAN_HEALTH_ERROR;
    }

    /* Check for RX timeout */
    if (can_interface_is_rx_timeout()) {
        return CAN_HEALTH_WARNING;
    }

    return CAN_HEALTH_OK;
}

bool can_interface_is_rx_timeout(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t age_ms = now - can_state.last_rx_time_ms;

    return (age_ms > CAN_INTERFACE_RX_TIMEOUT_MS);
}

void can_interface_update_watchdog(void)
{
    uint32_t now = HAL_GetTick();

    /* Check watchdog interval */
    if ((now - can_state.last_watchdog_time_ms) >= 50) {
        can_state.last_watchdog_time_ms = now;

        /* Increment timeout counter if RX has timed out */
        if (can_interface_is_rx_timeout()) {
            can_state.stats.timeout_errors++;
        }
    }
}

void can_interface_get_stats(can_interface_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    stats->tx_count = can_state.stats.tx_count;
    stats->rx_count = can_state.stats.rx_count;
    stats->rx_errors = can_state.stats.rx_errors;
    stats->timeout_errors = can_state.stats.timeout_errors;
    stats->last_rx_time_ms = can_state.last_rx_time_ms;
}

void can_interface_clear_stats(void)
{
    memset(&can_state.stats, 0, sizeof(can_interface_stats_t));
}

uint32_t can_interface_get_rx_age_ms(void)
{
    return HAL_GetTick() - can_state.last_rx_time_ms;
}
