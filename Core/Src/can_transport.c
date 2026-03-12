/**
 * @file    can_transport.c
 * @brief   CAN Transport Layer implementation
 */

#include "can_transport.h"
#include <string.h>

/* ============================================================================
   CONFIGURATION
   ============================================================================ */

#define TRANSPORT_TX_PERIOD_MS          10      /* 100 Hz */
#define TRANSPORT_HANDSHAKE_TIMEOUT_MS  100
#define TRANSPORT_RX_TIMEOUT_MS         200
#define TRANSPORT_WATCHDOG_INTERVAL_MS  50

/* ============================================================================
   STATIC STATE
   ============================================================================ */

static struct {
    can_transport_state_t state;
    uint32_t last_tx_time_ms;
    uint32_t last_watchdog_time_ms;
    uint32_t handshake_start_time_ms;
    uint32_t error_count;
} transport_state = {
    .state = TRANSPORT_STATE_INIT,
    .last_tx_time_ms = 0,
    .last_watchdog_time_ms = 0,
    .handshake_start_time_ms = 0,
    .error_count = 0
};

/* ============================================================================
   STATE MACHINE FUNCTIONS
   ============================================================================ */

static void transport_state_init(uint32_t current_time_ms)
{
    /* Initialize CAN interface */
    if (can_interface_init()) {
        transport_state.state = TRANSPORT_STATE_HANDSHAKE;
        transport_state.handshake_start_time_ms = current_time_ms;
        transport_state.last_tx_time_ms = current_time_ms;
        transport_state.last_watchdog_time_ms = current_time_ms;
    } else {
        transport_state.state = TRANSPORT_STATE_ERROR;
    }
}

static void transport_state_handshake(uint32_t current_time_ms)
{
    /* Check for handshake timeout */
    uint32_t handshake_elapsed = current_time_ms - transport_state.handshake_start_time_ms;
    if (handshake_elapsed > TRANSPORT_HANDSHAKE_TIMEOUT_MS * 10) {
        /* Give up after ~1 second */
        transport_state.state = TRANSPORT_STATE_ERROR;
        return;
    }

    /* Check if we've received VCU handshake bit */
    if (can_interface_get_rx_age_ms() < TRANSPORT_RX_TIMEOUT_MS) {
        /* Messages are being received */
        transport_state.state = TRANSPORT_STATE_READY;
    }
}

static void transport_state_ready(uint32_t current_time_ms)
{
    /* Check for communication loss */
    if (can_interface_is_rx_timeout()) {
        transport_state.state = TRANSPORT_STATE_ERROR;
        transport_state.error_count++;
        return;
    }

    /* Check for bus-off condition */
    can_health_status_t health = can_interface_get_health();
    if (health == CAN_HEALTH_BUS_OFF) {
        transport_state.state = TRANSPORT_STATE_ERROR;
    }
}

static void transport_state_error(uint32_t current_time_ms)
{
    /* Periodically attempt recovery */
    if ((current_time_ms % 100) == 0) {
        can_interface_recover();

        /* Try to re-init */
        if (can_interface_init()) {
            transport_state.state = TRANSPORT_STATE_HANDSHAKE;
            transport_state.handshake_start_time_ms = current_time_ms;
        }
    }
}

/* ============================================================================
   TIMING FUNCTIONS
   ============================================================================ */

static bool should_transmit(uint32_t current_time_ms)
{
    uint32_t elapsed = current_time_ms - transport_state.last_tx_time_ms;
    return (elapsed >= TRANSPORT_TX_PERIOD_MS);
}

static void update_watchdog(uint32_t current_time_ms)
{
    uint32_t elapsed = current_time_ms - transport_state.last_watchdog_time_ms;
    if (elapsed >= TRANSPORT_WATCHDOG_INTERVAL_MS) {
        transport_state.last_watchdog_time_ms = current_time_ms;
        can_interface_update_watchdog();
    }
}

/* ============================================================================
   PUBLIC API
   ============================================================================ */

bool can_transport_init(void)
{
    /* State machine will handle initialization */
    transport_state.state = TRANSPORT_STATE_INIT;
    return true;
}

void can_transport_deinit(void)
{
    can_interface_deinit();
    transport_state.state = TRANSPORT_STATE_INIT;
}

void can_transport_reset(void)
{
    can_interface_recover();
    transport_state.state = TRANSPORT_STATE_INIT;
}

can_transport_state_t can_transport_update(
    uint32_t current_time_ms,
    const fs_ai_ai_data_t *ai_data,
    fs_ai_vcu_data_t *vcu_data
)
{
    /* Handle state transitions */
    switch (transport_state.state) {
        case TRANSPORT_STATE_INIT:
            transport_state_init(current_time_ms);
            break;

        case TRANSPORT_STATE_HANDSHAKE:
            transport_state_handshake(current_time_ms);
            break;

        case TRANSPORT_STATE_READY:
            transport_state_ready(current_time_ms);
            break;

        case TRANSPORT_STATE_ERROR:
            transport_state_error(current_time_ms);
            break;

        default:
            transport_state.state = TRANSPORT_STATE_ERROR;
            break;
    }

    /* Always process received messages */
    can_interface_process_all(vcu_data);

    /* Transmit on schedule */
    if (should_transmit(current_time_ms) && ai_data != NULL) {
        if (transport_state.state == TRANSPORT_STATE_READY) {
            can_interface_send_all(ai_data);
        }
        transport_state.last_tx_time_ms = current_time_ms;
    }

    /* Update watchdog */
    update_watchdog(current_time_ms);

    return transport_state.state;
}

can_transport_state_t can_transport_get_state(void)
{
    return transport_state.state;
}

bool can_transport_is_ready(void)
{
    return (transport_state.state == TRANSPORT_STATE_READY);
}

bool can_transport_is_error(void)
{
    return (transport_state.state == TRANSPORT_STATE_ERROR);
}

bool can_transport_is_transmit_cycle(uint32_t current_time_ms)
{
    return should_transmit(current_time_ms);
}

uint32_t can_transport_time_to_next_transmit(uint32_t current_time_ms)
{
    uint32_t elapsed = current_time_ms - transport_state.last_tx_time_ms;
    if (elapsed >= TRANSPORT_TX_PERIOD_MS) {
        return 0;
    }
    return TRANSPORT_TX_PERIOD_MS - elapsed;
}

can_interface_stats_t can_transport_get_stats(void)
{
    can_interface_stats_t stats;
    can_interface_get_stats(&stats);
    return stats;
}

can_health_status_t can_transport_get_health(void)
{
    return can_interface_get_health();
}

uint32_t can_transport_get_last_vcu_msg_age_ms(void)
{
    return can_interface_get_rx_age_ms();
}

void can_transport_clear_stats(void)
{
    can_interface_clear_stats();
}
