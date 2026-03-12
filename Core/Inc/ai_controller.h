/**
 * @file    ai_controller.h
 * @brief   AI Algorithm Interface
 *
 * Simplified interface for AI algorithms to control the vehicle.
 * Abstracts away all CAN communication details.
 */

#ifndef AI_CONTROLLER_H
#define AI_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "can_transport.h"
#include "fs_ai_protocol.h"

/* ============================================================================
   AI CONTROLLER STATE
   ============================================================================ */

/** AI Controller operational state */
typedef enum {
    AI_CONTROL_STATE_IDLE = 0,
    AI_CONTROL_STATE_READY = 1,
    AI_CONTROL_STATE_RUNNING = 2,
    AI_CONTROL_STATE_FAULT = 3
} ai_control_state_t;

/* ============================================================================
   INITIALIZATION
   ============================================================================ */

/**
 * @brief Initialize AI controller
 *
 * Sets up CAN transport and prepares for operation.
 * Should be called once during startup.
 *
 * @return true if successful
 */
bool ai_controller_init(void);

/**
 * @brief De-initialize AI controller
 */
void ai_controller_deinit(void);

/* ============================================================================
   MAIN LOOP INTERFACE
   ============================================================================ */

/**
 * @brief Update AI controller (call from main loop)
 *
 * Processes received VCU messages, handles state machine,
 * and transmits pending messages.
 *
 * Should be called as frequently as possible, ideally 1-2ms interval.
 * Internally manages 10ms transmit cycle.
 *
 * @param current_time_ms Current system time from HAL_GetTick()
 * @return Current controller state
 */
ai_control_state_t ai_controller_update(uint32_t current_time_ms);

/**
 * @brief Get current AI controller state
 *
 * @return Current operational state
 */
ai_control_state_t ai_controller_get_state(void);

/* ============================================================================
   CONTROL COMMANDS (Called by AI Algorithm)
   ============================================================================ */

/**
 * @brief Set motor and steering control commands
 *
 * This is the main interface for AI algorithms.
 * Call this function with your computed control values.
 *
 * Example usage in AI algorithm:
 * @code
 * // Your AI algorithm computes control values
 * float throttle_nm = 50.0f;          // 50 Nm torque
 * float steering_angle = 5.5f;        // 5.5 degrees
 * float brake_percent = 0.0f;         // No braking
 * float motor_speed_rpm = 2000.0f;    // 2000 RPM
 *
 * ai_set_control(throttle_nm, steering_angle, brake_percent, motor_speed_rpm);
 * @endcode
 *
 * @param torque_nm Motor torque request in Newton-meters (0-195 Nm)
 * @param steering_deg Steering angle in degrees (-21 to +21°)
 * @param brake_pct Brake pressure request in percent (0-100%)
 * @param motor_speed_rpm Motor speed request in RPM (0-4000 RPM)
 */
void ai_set_control(
    float torque_nm,
    float steering_deg,
    float brake_pct,
    float motor_speed_rpm
);

/**
 * @brief Set mission status and safety signals
 *
 * Call this every control cycle to set mission state.
 *
 * @param mission Mission status (MISSION_NOT_SELECTED, MISSION_SELECTED, etc.)
 * @param direction Direction request (DIRECTION_FORWARD or DIRECTION_NEUTRAL)
 * @param estop Emergency stop request (ESTOP_NO or ESTOP_YES)
 */
void ai_set_mission_status(
    fs_ai_mission_status_t mission,
    fs_ai_direction_t direction,
    fs_ai_estop_t estop
);

/**
 * @brief Set lap and cone counters
 *
 * @param lap_count Current lap number
 * @param cone_count Number of cones detected/hit in this lap
 */
void ai_set_mission_progress(
    uint8_t lap_count,
    uint8_t cone_count
);

/**
 * @brief Set dynamics information for logging
 *
 * Optional: send vehicle dynamics to VCU for logging.
 *
 * @param accel_long Longitudinal acceleration in m/s²
 * @param accel_lat Lateral acceleration in m/s²
 * @param yaw_rate Yaw rate in degrees/second
 */
void ai_set_dynamics(
    float accel_long,
    float accel_lat,
    float yaw_rate
);

/* ============================================================================
   VEHICLE STATE FEEDBACK (Data received from VCU)
   ============================================================================ */

/**
 * @brief Get latest vehicle state received from VCU
 *
 * Contains all feedback from the vehicle (wheel speeds, steering, etc.)
 *
 * @param vcu_data Output: pointer to latest VCU data
 * @return true if data is valid and recent
 */
bool ai_get_vehicle_state(fs_ai_vcu_data_t *vcu_data);

/**
 * @brief Get current wheel speeds (RPM)
 *
 * @param fl Front left wheel speed (output)
 * @param fr Front right wheel speed (output)
 * @param rl Rear left wheel speed (output)
 * @param rr Rear right wheel speed (output)
 * @return true if speed data is valid
 */
bool ai_get_wheel_speeds(
    float *fl,
    float *fr,
    float *rl,
    float *rr
);

/**
 * @brief Get average vehicle speed
 *
 * Calculated from wheel speed average.
 *
 * @return Average speed in RPM
 */
float ai_get_average_wheel_speed(void);

/**
 * @brief Get current steering angle feedback
 *
 * @return Steering angle in degrees
 */
float ai_get_steering_feedback(void);

/**
 * @brief Get current brake pressure feedback
 *
 * @param brake_f Output: front brake pressure in %
 * @param brake_r Output: rear brake pressure in %
 * @return true if brake data is valid
 */
bool ai_get_brake_feedback(float *brake_f, float *brake_r);

/**
 * @brief Get VCU status
 *
 * @param status Output: VCU status structure
 * @return true if status is valid
 */
bool ai_get_vcu_status(fs_ai_vcu_status_t *status);

/* ============================================================================
   DIAGNOSTICS AND HEALTH CHECKING
   ============================================================================ */

/**
 * @brief Check if communication is healthy
 *
 * @return true if CAN bus is operational and messages flowing
 */
bool ai_is_healthy(void);

/**
 * @brief Get communication health status
 *
 * @return Health status enum
 */
can_health_status_t ai_get_can_health(void);

/**
 * @brief Check if ready for autonomous operation
 *
 * Requires:
 * - CAN communication established
 * - VCU handshake received
 * - No CAN errors
 *
 * @return true if all conditions met
 */
bool ai_is_ready_for_operation(void);

/**
 * @brief Get diagnostic statistics
 *
 * @return CAN interface statistics
 */
can_interface_stats_t ai_get_diagnostics(void);

/**
 * @brief Clear diagnostic statistics
 */
void ai_clear_diagnostics(void);

/**
 * @brief Get time since last VCU message (milliseconds)
 *
 * Useful for detecting communication timeout.
 *
 * @return Milliseconds elapsed
 */
uint32_t ai_get_vcu_msg_age_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* AI_CONTROLLER_H */
