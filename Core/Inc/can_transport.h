/**
 * @file    can_transport.h
 * @brief   CAN Transport Layer - Message Scheduling
 *
 * Manages the cyclic transmission of FS-AI protocol messages.
 * Handles 10ms transmit cycle, message queueing, and timing coordination.
 */

#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "can_interface.h"
#include "fs_ai_protocol.h"

/* ============================================================================
   TRANSPORT STATE MACHINE
   ============================================================================ */

/** Transport layer state */
typedef enum {
    TRANSPORT_STATE_INIT = 0,           /* Initializing */
    TRANSPORT_STATE_HANDSHAKE = 1,      /* Waiting for VCU handshake */
    TRANSPORT_STATE_READY = 2,          /* Ready to exchange messages */
    TRANSPORT_STATE_ERROR = 3           /* Communication error */
} can_transport_state_t;

/* ============================================================================
   INITIALIZATION
   ============================================================================ */

/**
 * @brief Initialize CAN transport layer
 *
 * Initializes underlying CAN interface and sets up timing.
 *
 * @return true if successful
 */
bool can_transport_init(void);

/**
 * @brief De-initialize CAN transport layer
 */
void can_transport_deinit(void);

/**
 * @brief Reset transport state machine
 */
void can_transport_reset(void);

/* ============================================================================
   MAIN CONTROL LOOP INTERFACE
   ============================================================================ */

/**
 * @brief Update transport layer (call from main loop)
 *
 * Handles message reception, health checks, and transmission scheduling.
 * Should be called as frequently as possible (ideally every 1-2ms).
 *
 * @param current_time_ms Current system time from HAL_GetTick()
 * @param ai_data AI data to transmit (control, status, dynamics)
 * @param vcu_data Output: latest VCU data received
 * @return Current transport state
 */
can_transport_state_t can_transport_update(
    uint32_t current_time_ms,
    const fs_ai_ai_data_t *ai_data,
    fs_ai_vcu_data_t *vcu_data
);

/**
 * @brief Get current transport state
 *
 * @return Current state of the transport layer
 */
can_transport_state_t can_transport_get_state(void);

/**
 * @brief Check if transport is ready for operation
 *
 * @return true if in READY state, false otherwise
 */
bool can_transport_is_ready(void);

/**
 * @brief Check if transport is in error state
 *
 * @return true if in ERROR state
 */
bool can_transport_is_error(void);

/* ============================================================================
   TIMING AND SYNCHRONIZATION
   ============================================================================ */

/**
 * @brief Check if it's time for next transmit cycle
 *
 * Returns true every 10ms. Useful for external timing control.
 *
 * @param current_time_ms Current system time
 * @return true if transmit cycle should run
 */
bool can_transport_is_transmit_cycle(uint32_t current_time_ms);

/**
 * @brief Get time until next transmit cycle
 *
 * @param current_time_ms Current system time
 * @return Milliseconds until next scheduled transmit
 */
uint32_t can_transport_time_to_next_transmit(uint32_t current_time_ms);

/* ============================================================================
   DIAGNOSTICS AND STATUS
   ============================================================================ */

/**
 * @brief Get transport layer statistics
 *
 * @return Statistics from underlying CAN interface
 */
can_interface_stats_t can_transport_get_stats(void);

/**
 * @brief Get health status
 *
 * @return Current CAN bus health status
 */
can_health_status_t can_transport_get_health(void);

/**
 * @brief Get time since last successful VCU message
 *
 * @return Milliseconds elapsed
 */
uint32_t can_transport_get_last_vcu_msg_age_ms(void);

/**
 * @brief Clear all statistics
 */
void can_transport_clear_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_TRANSPORT_H */
