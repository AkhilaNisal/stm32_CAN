/**
 * @file    fs_ai_protocol.h
 * @brief   FS-AI Protocol encoder/decoder for STM32
 *
 * Clean separation of protocol encoding/decoding from transmission.
 * This module handles all bitfield manipulation and data packing.
 */

#ifndef FS_AI_PROTOCOL_H
#define FS_AI_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "CANSPI.h"

/* ============================================================================
   PROTOCOL CONSTANTS
   ============================================================================ */

/* CAN Message IDs (AI → VCU) */
#define CAN_ID_AI2VCU_STATUS        0x510
#define CAN_ID_AI2VCU_DRIVE_F       0x511
#define CAN_ID_AI2VCU_DRIVE_R       0x512
#define CAN_ID_AI2VCU_STEER         0x513
#define CAN_ID_AI2VCU_BRAKE         0x514
#define CAN_ID_AI2LOG_DYNAMICS      0x501

/* CAN Message IDs (VCU → AI) */
#define CAN_ID_VCU2AI_STATUS        0x520
#define CAN_ID_VCU2AI_DRIVE_F       0x521
#define CAN_ID_VCU2AI_DRIVE_R       0x522
#define CAN_ID_VCU2AI_STEER         0x523
#define CAN_ID_VCU2AI_BRAKE         0x524
#define CAN_ID_VCU2AI_SPEEDS        0x525
#define CAN_ID_VCU2AI_WHEEL_COUNTS  0x526

/* ============================================================================
   ENUMERATION TYPES (from FS-AI V4.0 specification)
   ============================================================================ */

/** AS (Autonomous System) State */
typedef enum {
    AS_OFF = 1,
    AS_READY = 2,
    AS_DRIVING = 3,
    AS_EMERGENCY_BRAKE = 4,
    AS_FINISHED = 5
} fs_ai_as_state_t;

/** AMI (Autonomous Mission Indicator) State */
typedef enum {
    AMI_NOT_SELECTED = 0,
    AMI_ACCELERATION = 1,
    AMI_SKIDPAD = 2,
    AMI_AUTOCROSS = 3,
    AMI_TRACK_DRIVE = 4,
    AMI_STATIC_INSPECTION_A = 5,
    AMI_STATIC_INSPECTION_B = 6,
    AMI_AUTONOMOUS_DEMO = 7
} fs_ai_ami_state_t;

/** Handshake Status */
typedef enum {
    HANDSHAKE_OFF = 0,
    HANDSHAKE_ON = 1
} fs_ai_handshake_t;

/** RES (Remote Emergency Stop) Go Signal */
typedef enum {
    RES_NO_GO = 0,
    RES_GO = 1
} fs_ai_res_go_signal_t;

/** Mission Status */
typedef enum {
    MISSION_NOT_SELECTED = 0,
    MISSION_SELECTED = 1,
    MISSION_RUNNING = 2,
    MISSION_FINISHED = 3
} fs_ai_mission_status_t;

/** Direction Request */
typedef enum {
    DIRECTION_NEUTRAL = 0,
    DIRECTION_FORWARD = 1
} fs_ai_direction_t;

/** Emergency Stop Request */
typedef enum {
    ESTOP_NO = 0,
    ESTOP_YES = 1
} fs_ai_estop_t;

/* ============================================================================
   DATA STRUCTURES - VCU TO AI (RX)
   ============================================================================ */

/** VCU Status data received from vehicle */
typedef struct {
    fs_ai_handshake_t handshake_bit;
    fs_ai_res_go_signal_t go_signal;
    fs_ai_as_state_t as_state;
    fs_ai_ami_state_t ami_state;
    uint8_t fault_status;
    uint8_t warning_status;
    uint32_t timestamp_ms;
} fs_ai_vcu_status_t;

/** VCU Motor feedback */
typedef struct {
    float front_axle_torque_max_nm;
    float rear_axle_torque_max_nm;
} fs_ai_vcu_motor_feedback_t;

/** VCU Steering feedback */
typedef struct {
    float steer_angle_deg;
    float steer_angle_max_deg;
} fs_ai_vcu_steer_feedback_t;

/** VCU Brake feedback */
typedef struct {
    float brake_pressure_f_pct;
    float brake_pressure_r_pct;
} fs_ai_vcu_brake_feedback_t;

/** VCU Wheel speeds */
typedef struct {
    float fl_wheel_speed_rpm;
    float fr_wheel_speed_rpm;
    float rl_wheel_speed_rpm;
    float rr_wheel_speed_rpm;
} fs_ai_vcu_wheel_speeds_t;

/** Combined VCU to AI data */
typedef struct {
    fs_ai_vcu_status_t status;
    fs_ai_vcu_motor_feedback_t motor;
    fs_ai_vcu_steer_feedback_t steer;
    fs_ai_vcu_brake_feedback_t brake;
    fs_ai_vcu_wheel_speeds_t speeds;
} fs_ai_vcu_data_t;

/* ============================================================================
   DATA STRUCTURES - AI TO VCU (TX)
   ============================================================================ */

/** AI Control command */
typedef struct {
    float front_torque_nm;
    float rear_torque_nm;
    float front_motor_speed_rpm;
    float rear_motor_speed_rpm;
    float steer_angle_deg;
    float brake_pressure_f_pct;
    float brake_pressure_r_pct;
} fs_ai_control_t;

/** AI Status to transmit */
typedef struct {
    fs_ai_mission_status_t mission_status;
    fs_ai_direction_t direction;
    fs_ai_estop_t estop;
    uint8_t lap_counter;
    uint8_t cones_count;
    uint8_t vehicle_speed_kmh;
} fs_ai_status_t;

/** AI Dynamics for logging */
typedef struct {
    int16_t accel_longitudinal_ms2;
    int16_t accel_lateral_ms2;
    int16_t yaw_rate_degps;
} fs_ai_dynamics_t;

/** Combined AI to VCU data */
typedef struct {
    fs_ai_control_t control;
    fs_ai_status_t status;
    fs_ai_dynamics_t dynamics;
} fs_ai_ai_data_t;

/* ============================================================================
   PROTOCOL FUNCTIONS - ENCODING (AI → VCU)
   ============================================================================ */

/**
 * @brief Encode AI status message
 *
 * @param data AI status structure
 * @param msg Output CAN message
 * @return true if successful, false if validation failed
 */
bool fs_ai_protocol_encode_status(const fs_ai_status_t *data, uCAN_MSG *msg);

/**
 * @brief Encode AI drive front message
 *
 * @param control Control structure with drive parameters
 * @param msg Output CAN message
 * @return true if successful
 */
bool fs_ai_protocol_encode_drive_f(const fs_ai_control_t *control, uCAN_MSG *msg);

/**
 * @brief Encode AI drive rear message
 *
 * @param control Control structure with drive parameters
 * @param msg Output CAN message
 * @return true if successful
 */
bool fs_ai_protocol_encode_drive_r(const fs_ai_control_t *control, uCAN_MSG *msg);

/**
 * @brief Encode AI steering message
 *
 * @param control Control structure
 * @param msg Output CAN message
 * @return true if successful
 */
bool fs_ai_protocol_encode_steer(const fs_ai_control_t *control, uCAN_MSG *msg);

/**
 * @brief Encode AI brake message
 *
 * @param control Control structure
 * @param msg Output CAN message
 * @return true if successful
 */
bool fs_ai_protocol_encode_brake(const fs_ai_control_t *control, uCAN_MSG *msg);

/**
 * @brief Encode AI dynamics logging message
 *
 * @param dynamics Dynamics structure
 * @param msg Output CAN message
 * @return true if successful
 */
bool fs_ai_protocol_encode_dynamics(const fs_ai_dynamics_t *dynamics, uCAN_MSG *msg);

/* ============================================================================
   PROTOCOL FUNCTIONS - DECODING (VCU → AI)
   ============================================================================ */

/**
 * @brief Decode VCU status message
 *
 * @param msg Input CAN message
 * @param status Output status structure
 * @return true if successful
 */
bool fs_ai_protocol_decode_status(const uCAN_MSG *msg, fs_ai_vcu_status_t *status);

/**
 * @brief Decode VCU drive front message
 *
 * @param msg Input CAN message
 * @param motor Output motor structure
 * @return true if successful
 */
bool fs_ai_protocol_decode_drive_f(const uCAN_MSG *msg, fs_ai_vcu_motor_feedback_t *motor);

/**
 * @brief Decode VCU drive rear message
 *
 * @param msg Input CAN message
 * @param motor Output motor structure (rear axle fields only)
 * @return true if successful
 */
bool fs_ai_protocol_decode_drive_r(const uCAN_MSG *msg, fs_ai_vcu_motor_feedback_t *motor);

/**
 * @brief Decode VCU steering message
 *
 * @param msg Input CAN message
 * @param steer Output steering structure
 * @return true if successful
 */
bool fs_ai_protocol_decode_steer(const uCAN_MSG *msg, fs_ai_vcu_steer_feedback_t *steer);

/**
 * @brief Decode VCU brake message
 *
 * @param msg Input CAN message
 * @param brake Output brake structure
 * @return true if successful
 */
bool fs_ai_protocol_decode_brake(const uCAN_MSG *msg, fs_ai_vcu_brake_feedback_t *brake);

/**
 * @brief Decode VCU wheel speeds message
 *
 * @param msg Input CAN message
 * @param speeds Output wheel speeds structure
 * @return true if successful
 */
bool fs_ai_protocol_decode_speeds(const uCAN_MSG *msg, fs_ai_vcu_wheel_speeds_t *speeds);

#ifdef __cplusplus
}
#endif

#endif /* FS_AI_PROTOCOL_H */
