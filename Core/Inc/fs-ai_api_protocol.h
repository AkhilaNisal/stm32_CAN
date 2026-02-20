#ifndef FS_AI_API_H
#define FS_AI_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "CANSPI.h"

/* ============================================================================
   AS (Autonomous System) States - ADS-DV V4.0 Spec
   ============================================================================ */
typedef enum {
    AS_OFF = 1,
    AS_READY = 2,
    AS_DRIVING = 3,
    AS_EMERGENCY_BRAKE = 4,
    AS_FINISHED = 5
} fs_ai_api_as_state_e;

/* ============================================================================
   AMI (Autonomous Mission Indicator) States
   ============================================================================ */
typedef enum {
    AMI_NOT_SELECTED = 0,
    AMI_ACCELERATION = 1,
    AMI_SKIDPAD = 2,
    AMI_AUTOCROSS = 3,
    AMI_TRACK_DRIVE = 4,
    AMI_STATIC_INSPECTION_A = 5,
    AMI_STATIC_INSPECTION_B = 6,
    AMI_AUTONOMOUS_DEMO = 7
} fs_ai_api_ami_state_e;

/* ============================================================================
   Handshake Bit Enumerations
   ============================================================================ */
typedef enum {
    HANDSHAKE_RECEIVE_BIT_OFF = 0,
    HANDSHAKE_RECEIVE_BIT_ON = 1
} fs_ai_api_handshake_receive_bit_e;

typedef enum {
    HANDSHAKE_SEND_BIT_OFF = 0,
    HANDSHAKE_SEND_BIT_ON = 1
} fs_ai_api_handshake_send_bit_e;

/* ============================================================================
   Remote Emergency Stop (RES) Go Signal
   ============================================================================ */
typedef enum {
    RES_GO_SIGNAL_NO_GO = 0,
    RES_GO_SIGNAL_GO = 1
} fs_ai_api_res_go_signal_bit_e;

/* ============================================================================
   Mission Status Enumeration
   ============================================================================ */
typedef enum {
    MISSION_NOT_SELECTED = 0,
    MISSION_SELECTED = 1,
    MISSION_RUNNING = 2,
    MISSION_FINISHED = 3
} fs_ai_api_mission_status_e;

/* ============================================================================
   Direction Request Enumeration
   ============================================================================ */
typedef enum {
    DIRECTION_NEUTRAL = 0,
    DIRECTION_FORWARD = 1
} fs_ai_api_direction_request_e;

/* ============================================================================
   Emergency Stop Request
   ============================================================================ */
typedef enum {
    ESTOP_NO = 0,
    ESTOP_YES = 1
} fs_ai_api_estop_request_e;

/* ============================================================================
   VCU to AI Data Structure (Reception)
   ============================================================================ */
typedef struct {
    /* From VCU2AI_Status (0x520) */
    fs_ai_api_handshake_receive_bit_e   VCU2AI_HANDSHAKE_RECEIVE_BIT;
    fs_ai_api_res_go_signal_bit_e       VCU2AI_RES_GO_SIGNAL;
    fs_ai_api_as_state_e                VCU2AI_AS_STATE;
    fs_ai_api_ami_state_e               VCU2AI_AMI_STATE;
    uint8_t                             VCU2AI_FAULT_STATUS;
    uint8_t                             VCU2AI_WARNING_STATUS;
    uint8_t                             VCU2AI_WARN_BATT_TEMP_HIGH;
    uint8_t                             VCU2AI_WARN_BATT_SOC_LOW;
    uint8_t                             VCU2AI_AI_ESTOP_REQUEST;
    uint8_t                             VCU2AI_HVIL_OPEN_FAULT;
    uint8_t                             VCU2AI_HVIL_SHORT_FAULT;
    uint8_t                             VCU2AI_EBS_FAULT;
    uint8_t                             VCU2AI_CHARGER_FAULT;
    uint8_t                             VCU2AI_AI_COMMS_LOST;
    uint8_t                             VCU2AI_AUTO_BRAKING_FAULT;
    uint8_t                             VCU2AI_MISSION_STATUS_FAULT;
    uint8_t                             VCU2AI_CHARGE_PROCEDURE_FAULT;
    uint8_t                             VCU2AI_BMS_FAULT;
    uint8_t                             VCU2AI_BRAKE_PLAUSIBILITY_FAULT;

    /* From VCU2AI_Drive_F (0x521) & VCU2AI_Drive_R (0x522) */
    float                               VCU2AI_FRONT_AXLE_TORQUE_MAX_Nm;
    float                               VCU2AI_REAR_AXLE_TORQUE_MAX_Nm;

    /* From VCU2AI_Steer (0x523) */
    float                               VCU2AI_STEER_ANGLE_deg;
    float                               VCU2AI_STEER_ANGLE_MAX_deg;

    /* From VCU2AI_Brake (0x524) */
    float                               VCU2AI_BRAKE_PRESS_F_pct;
    float                               VCU2AI_BRAKE_PRESS_R_pct;

    /* From VCU2AI_Speeds (0x525) */
    float                               VCU2AI_FL_WHEEL_SPEED_rpm;
    float                               VCU2AI_FR_WHEEL_SPEED_rpm;
    float                               VCU2AI_RL_WHEEL_SPEED_rpm;
    float                               VCU2AI_RR_WHEEL_SPEED_rpm;

    /* From VCU2AI_Wheel_counts (0x526) */
    uint16_t                            VCU2AI_FL_PULSE_COUNT;
    uint16_t                            VCU2AI_FR_PULSE_COUNT;
    uint16_t                            VCU2AI_RL_PULSE_COUNT;
    uint16_t                            VCU2AI_RR_PULSE_COUNT;

    /* Timestamp */
    uint32_t                            timestamp_ms;
} fs_ai_api_vcu2ai;

/* ============================================================================
   AI to VCU Data Structure (Transmission)
   ============================================================================ */
typedef struct {
    /* AI2VCU_Status (0x510) */
    fs_ai_api_mission_status_e          AI2VCU_MISSION_STATUS;
    fs_ai_api_direction_request_e       AI2VCU_DIRECTION_REQUEST;
    fs_ai_api_estop_request_e           AI2VCU_ESTOP_REQUEST;
    fs_ai_api_handshake_send_bit_e      AI2VCU_HANDSHAKE_SEND_BIT;
    uint8_t                             AI2VCU_LAP_COUNTER;
    uint8_t                             AI2VCU_CONES_COUNT_ACTUAL;
    uint16_t                            AI2VCU_CONES_COUNT_ALL;
    uint8_t                             AI2VCU_VEH_SPEED_ACTUAL_kmh;
    uint8_t                             AI2VCU_VEH_SPEED_DEMAND_kmh;

    /* AI2VCU_Drive_F (0x511) & AI2VCU_Drive_R (0x512) */
    float                               AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm;
    float                               AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm;
    float                               AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm;
    float                               AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm;

    /* AI2VCU_Steer (0x513) */
    float                               AI2VCU_STEER_ANGLE_REQUEST_deg;

    /* AI2VCU_Brake (0x514) */
    float                               AI2VCU_BRAKE_PRESS_F_REQUEST_pct;
    float                               AI2VCU_BRAKE_PRESS_R_REQUEST_pct;

    /* AI2LOG_Dynamics2 (0x501) */
    int16_t                             AI2LOG_ACCEL_LONGITUDINAL_ms2;
    int16_t                             AI2LOG_ACCEL_LATERAL_ms2;
    int16_t                             AI2LOG_YAW_RATE_degps;
} fs_ai_api_ai2vcu;

/* ============================================================================
   CAN Statistics Structure
   ============================================================================ */
typedef struct {
    uint32_t VCU2AI_Status_count;
    uint32_t VCU2AI_Drive_F_count;
    uint32_t VCU2AI_Drive_R_count;
    uint32_t VCU2AI_Steer_count;
    uint32_t VCU2AI_Brake_count;
    uint32_t VCU2AI_Wheel_speeds_count;
    uint32_t VCU2AI_Wheel_counts_count;
    uint32_t AI2VCU_Status_count;
    uint32_t AI2VCU_Drive_F_count;
    uint32_t AI2VCU_Drive_R_count;
    uint32_t AI2VCU_Steer_count;
    uint32_t AI2VCU_Brake_count;
    uint32_t unhandled_frame_count;
    uint32_t crc_error_count;
    uint32_t timeout_error_count;
} fs_ai_api_can_stats_t;

/* ============================================================================
   Function Prototypes
   ============================================================================ */

/**
 * @brief Initialize the FS-AI API
 * @return True if successful, False otherwise
 */
bool fs_ai_api_init(void);

/**
 * @brief Process incoming CAN messages
 * @param rxMsg Pointer to received CAN message
 */
void fs_ai_api_process_incoming(uCAN_MSG *rxMsg);

/**
 * @brief Get data received from VCU
 * @param data Pointer to VCU2AI data structure
 */
void fs_ai_api_vcu2ai_get_data(fs_ai_api_vcu2ai *data);

/**
 * @brief Set data to be transmitted to VCU
 * @param data Pointer to AI2VCU data structure
 */
void fs_ai_api_ai2vcu_set_data(fs_ai_api_ai2vcu *data);

/**
 * @brief Send all pending AI2VCU messages
 */
void fs_ai_api_send_all_messages(void);

/**
 * @brief Get CAN bus statistics
 * @param stats Pointer to statistics structure
 */
void fs_ai_api_get_can_stats(fs_ai_api_can_stats_t *stats);

/**
 * @brief Clear CAN bus statistics
 */
void fs_ai_api_clear_can_stats(void);

/**
 * @brief Check if handshake is complete
 * @return True if handshake successful, False otherwise
 */
bool fs_ai_api_is_handshake_complete(void);

/**
 * @brief Check if communications are healthy
 * @return True if comms healthy, False otherwise
 */
bool fs_ai_api_is_comms_healthy(void);

#ifdef __cplusplus
}
#endif

#endif /* FS_AI_API_H */
