/**
 * @file    can_interface.h
 * @brief   CAN Hardware Abstraction Layer
 *
 * Provides unified interface between protocol codec and MCP2515 driver.
 * Handles message queuing, timeouts, and health monitoring.
 */

#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "CANSPI.h"
#include "fs_ai_protocol.h"

/* ============================================================================
   CONFIGURATION
   ============================================================================ */

#define CAN_INTERFACE_TX_QUEUE_SIZE    6     /* One message per type */
#define CAN_INTERFACE_RX_TIMEOUT_MS    200   /* Message timeout */

/* ============================================================================
   STATUS AND DIAGNOSTICS
   ============================================================================ */

/** CAN bus health status */
typedef enum {
    CAN_HEALTH_OK = 0,
    CAN_HEALTH_WARNING = 1,
    CAN_HEALTH_ERROR = 2,
    CAN_HEALTH_BUS_OFF = 3
} can_health_status_t;

/** Message statistics */
typedef struct {
    uint32_t tx_count;              /* Total messages transmitted */
    uint32_t rx_count;              /* Total messages received */
    uint32_t rx_errors;             /* Malformed/dropped messages */
    uint32_t timeout_errors;        /* RX timeout count */
    uint32_t last_rx_time_ms;       /* Timestamp of last RX message */
} can_interface_stats_t;

/* ============================================================================
   INITIALIZATION AND CONTROL
   ============================================================================ */

/**
 * @brief Initialize CAN interface
 *
 * Initializes MCP2515 hardware and clears all statistics.
 *
 * @return true if successful, false if MCP2515 init failed
 */
bool can_interface_init(void);

/**
 * @brief De-initialize CAN interface
 */
void can_interface_deinit(void);

/**
 * @brief Recover from CAN bus error (bus-off condition)
 *
 * Performs full reset and re-initialization of MCP2515.
 * Should be called periodically or when error detected.
 */
void can_interface_recover(void);

/* ============================================================================
   TRANSMISSION INTERFACE
   ============================================================================ */

/**
 * @brief Queue a message for transmission
 *
 * Non-blocking transmission with timeout. Message is placed in MCP2515
 * TX buffer with configurable timeout waiting.
 *
 * @param msg CAN message to transmit
 * @param timeout_ms Maximum time to wait for TX buffer (5ms typical)
 * @return true if queued successfully, false if TX buffers full
 */
bool can_interface_send(const uCAN_MSG *msg, uint32_t timeout_ms);

/**
 * @brief Send all 6 FS-AI protocol messages
 *
 * Convenience function that encodes and sends all AI→VCU messages.
 * Typically called at 100 Hz (10ms interval).
 *
 * @param ai_data Complete AI data structure
 * @return true if all messages queued successfully
 */
bool can_interface_send_all(const fs_ai_ai_data_t *ai_data);

/* ============================================================================
   RECEPTION INTERFACE
   ============================================================================ */

/**
 * @brief Receive and process one CAN message
 *
 * Non-blocking receive. Should be called in main loop.
 * Internally decodes and stores message data.
 *
 * @param vcu_data Output structure with decoded VCU data
 * @return true if message received and decoded, false if no message
 */
bool can_interface_receive(fs_ai_vcu_data_t *vcu_data);

/**
 * @brief Poll for messages in MCP2515 RX buffers
 *
 * @return Number of messages available in RX buffers
 */
uint8_t can_interface_messages_available(void);

/**
 * @brief Process all pending received messages
 *
 * Calls can_interface_receive() repeatedly until no more messages.
 *
 * @param vcu_data Output structure with latest VCU data
 * @return true if at least one message was processed
 */
bool can_interface_process_all(fs_ai_vcu_data_t *vcu_data);

/* ============================================================================
   HEALTH MONITORING
   ============================================================================ */

/**
 * @brief Get current CAN bus health status
 *
 * @return Health status enumeration
 */
can_health_status_t can_interface_get_health(void);

/**
 * @brief Check if communication has timed out
 *
 * @return true if no RX messages received within timeout window
 */
bool can_interface_is_rx_timeout(void);

/**
 * @brief Update communication watchdog
 *
 * Should be called regularly (50ms) from main loop.
 * Detects communication timeouts and updates error counters.
 */
void can_interface_update_watchdog(void);

/**
 * @brief Get interface statistics
 *
 * @param stats Output statistics structure
 */
void can_interface_get_stats(can_interface_stats_t *stats);

/**
 * @brief Clear all statistics counters
 */
void can_interface_clear_stats(void);

/**
 * @brief Get milliseconds since last RX message
 *
 * Useful for detecting communication loss.
 *
 * @return Milliseconds elapsed since last message
 */
uint32_t can_interface_get_rx_age_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_INTERFACE_H */
