/**
 * @file    fs_ai_protocol.c
 * @brief   FS-AI Protocol encoder/decoder implementation
 */

#include "fs_ai_protocol.h"
#include <string.h>
#include <math.h>

/* ============================================================================
   CONFIGURATION CONSTANTS
   ============================================================================ */

#define STEER_ANGLE_RES         0.1f        /* 0.1 degrees per unit */
#define BRAKE_PRESS_RES         0.5f        /* 0.5% per unit */
#define TORQUE_RES              0.1f        /* 0.1 Nm per unit */
#define ACCEL_RES               1.0f        /* 1 m/s² per unit */
#define YAW_RATE_RES            1.0f        /* 1 °/s per unit */

/* ============================================================================
   ENCODING FUNCTIONS
   ============================================================================ */

bool fs_ai_protocol_encode_status(const fs_ai_status_t *data, uCAN_MSG *msg)
{
    if (data == NULL || msg == NULL) {
        return false;
    }

    msg->frame.id = CAN_ID_AI2VCU_STATUS;
    msg->frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg->frame.dlc = 8;

    memset(&msg->frame.data0, 0, 8);

    /* Byte 0: Handshake(0), E-stop(1), Mission_status(2-3), Direction(4) */
    msg->frame.data0 = (1 & 0x01) |                           /* Handshake ON */
                       ((data->estop & 0x01) << 1) |
                       ((data->mission_status & 0x03) << 2) |
                       ((data->direction & 0x01) << 4);

    /* Byte 1: Lap counter */
    msg->frame.data1 = data->lap_counter;

    /* Byte 2: Cones count */
    msg->frame.data2 = data->cones_count;

    /* Bytes 3-4: Cones count all (little endian) */
    uint16_t cones_all = data->cones_count;
    msg->frame.data3 = (uint8_t)(cones_all & 0xFF);
    msg->frame.data4 = (uint8_t)((cones_all >> 8) & 0xFF);

    /* Byte 5: Vehicle speed actual */
    msg->frame.data5 = data->vehicle_speed_kmh;

    /* Byte 6: Vehicle speed demand */
    msg->frame.data6 = data->vehicle_speed_kmh;

    /* Byte 7: Reserved */
    msg->frame.data7 = 0;

    return true;
}

bool fs_ai_protocol_encode_drive_f(const fs_ai_control_t *control, uCAN_MSG *msg)
{
    if (control == NULL || msg == NULL) {
        return false;
    }

    msg->frame.id = CAN_ID_AI2VCU_DRIVE_F;
    msg->frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg->frame.dlc = 4;

    memset(&msg->frame.data0, 0, 8);

    /* Validate and clamp torque request */
    float torque = control->front_torque_nm;
    if (torque > 195.0f) torque = 195.0f;
    if (torque < 0.0f) torque = 0.0f;

    /* Validate and clamp motor speed */
    float speed = control->front_motor_speed_rpm;
    if (speed > 4000.0f) speed = 4000.0f;
    if (speed < 0.0f) speed = 0.0f;

    /* Bytes 0-1: Torque request (little endian) */
    uint16_t torque_raw = (uint16_t)(torque / TORQUE_RES);
    msg->frame.data0 = (uint8_t)(torque_raw & 0xFF);
    msg->frame.data1 = (uint8_t)((torque_raw >> 8) & 0xFF);

    /* Bytes 2-3: Motor speed (little endian) */
    uint16_t speed_raw = (uint16_t)speed;
    msg->frame.data2 = (uint8_t)(speed_raw & 0xFF);
    msg->frame.data3 = (uint8_t)((speed_raw >> 8) & 0xFF);

    return true;
}

bool fs_ai_protocol_encode_drive_r(const fs_ai_control_t *control, uCAN_MSG *msg)
{
    if (control == NULL || msg == NULL) {
        return false;
    }

    msg->frame.id = CAN_ID_AI2VCU_DRIVE_R;
    msg->frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg->frame.dlc = 4;

    memset(&msg->frame.data0, 0, 8);

    /* Validate and clamp torque request */
    float torque = control->rear_torque_nm;
    if (torque > 195.0f) torque = 195.0f;
    if (torque < 0.0f) torque = 0.0f;

    /* Validate and clamp motor speed */
    float speed = control->rear_motor_speed_rpm;
    if (speed > 4000.0f) speed = 4000.0f;
    if (speed < 0.0f) speed = 0.0f;

    /* Bytes 0-1: Torque request (little endian) */
    uint16_t torque_raw = (uint16_t)(torque / TORQUE_RES);
    msg->frame.data0 = (uint8_t)(torque_raw & 0xFF);
    msg->frame.data1 = (uint8_t)((torque_raw >> 8) & 0xFF);

    /* Bytes 2-3: Motor speed (little endian) */
    uint16_t speed_raw = (uint16_t)speed;
    msg->frame.data2 = (uint8_t)(speed_raw & 0xFF);
    msg->frame.data3 = (uint8_t)((speed_raw >> 8) & 0xFF);

    return true;
}

bool fs_ai_protocol_encode_steer(const fs_ai_control_t *control, uCAN_MSG *msg)
{
    if (control == NULL || msg == NULL) {
        return false;
    }

    msg->frame.id = CAN_ID_AI2VCU_STEER;
    msg->frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg->frame.dlc = 2;

    memset(&msg->frame.data0, 0, 8);

    /* Validate and clamp steering angle */
    float angle = control->steer_angle_deg;
    if (angle > 21.0f) angle = 21.0f;
    if (angle < -21.0f) angle = -21.0f;

    /* Bytes 0-1: Steering angle (little endian, signed) */
    int16_t angle_raw = (int16_t)(angle / STEER_ANGLE_RES);
    msg->frame.data0 = (uint8_t)(angle_raw & 0xFF);
    msg->frame.data1 = (uint8_t)((angle_raw >> 8) & 0xFF);

    return true;
}

bool fs_ai_protocol_encode_brake(const fs_ai_control_t *control, uCAN_MSG *msg)
{
    if (control == NULL || msg == NULL) {
        return false;
    }

    msg->frame.id = CAN_ID_AI2VCU_BRAKE;
    msg->frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg->frame.dlc = 2;

    memset(&msg->frame.data0, 0, 8);

    /* Validate and clamp brake pressure */
    float brake_f = control->brake_pressure_f_pct;
    float brake_r = control->brake_pressure_r_pct;

    if (brake_f > 100.0f) brake_f = 100.0f;
    if (brake_f < 0.0f) brake_f = 0.0f;
    if (brake_r > 100.0f) brake_r = 100.0f;
    if (brake_r < 0.0f) brake_r = 0.0f;

    /* Brake plausibility: cannot have both torque AND brake */
    if ((brake_f > 0.0f) || (brake_r > 0.0f)) {
        /* Brake is requested - this is handled at higher level */
    }

    /* Byte 0: Front brake pressure */
    msg->frame.data0 = (uint8_t)(brake_f / BRAKE_PRESS_RES);

    /* Byte 1: Rear brake pressure */
    msg->frame.data1 = (uint8_t)(brake_r / BRAKE_PRESS_RES);

    return true;
}

bool fs_ai_protocol_encode_dynamics(const fs_ai_dynamics_t *dynamics, uCAN_MSG *msg)
{
    if (dynamics == NULL || msg == NULL) {
        return false;
    }

    msg->frame.id = CAN_ID_AI2LOG_DYNAMICS;
    msg->frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg->frame.dlc = 6;

    memset(&msg->frame.data0, 0, 8);

    /* Bytes 0-1: Longitudinal acceleration (little endian, signed) */
    int16_t accel_long = (int16_t)(dynamics->accel_longitudinal_ms2 / ACCEL_RES);
    msg->frame.data0 = (uint8_t)(accel_long & 0xFF);
    msg->frame.data1 = (uint8_t)((accel_long >> 8) & 0xFF);

    /* Bytes 2-3: Lateral acceleration (little endian, signed) */
    int16_t accel_lat = (int16_t)(dynamics->accel_lateral_ms2 / ACCEL_RES);
    msg->frame.data2 = (uint8_t)(accel_lat & 0xFF);
    msg->frame.data3 = (uint8_t)((accel_lat >> 8) & 0xFF);

    /* Bytes 4-5: Yaw rate (little endian, signed) */
    int16_t yaw_rate = (int16_t)(dynamics->yaw_rate_degps / YAW_RATE_RES);
    msg->frame.data4 = (uint8_t)(yaw_rate & 0xFF);
    msg->frame.data5 = (uint8_t)((yaw_rate >> 8) & 0xFF);

    return true;
}

/* ============================================================================
   DECODING FUNCTIONS
   ============================================================================ */

bool fs_ai_protocol_decode_status(const uCAN_MSG *msg, fs_ai_vcu_status_t *status)
{
    if (msg == NULL || status == NULL || msg->frame.dlc < 8) {
        return false;
    }

    memset(status, 0, sizeof(fs_ai_vcu_status_t));

    /* Byte 0: Handshake(0), Shutdown_req(1), AS_sw(2), TS_sw(3), GO(4), Steering(5) */
    status->handshake_bit = (msg->frame.data0 >> 0) & 0x01;
    status->go_signal = (msg->frame.data0 >> 4) & 0x01;

    /* Byte 1: AS_state(bits 0-2), AMI_state(bits 4-6) */
    status->as_state = (msg->frame.data1 >> 0) & 0x07;
    status->ami_state = (msg->frame.data1 >> 4) & 0x07;

    /* Byte 2: Various fault/warning flags */
    status->fault_status = (msg->frame.data2 >> 0) & 0x01;
    status->warning_status = (msg->frame.data2 >> 1) & 0x01;

    status->timestamp_ms = HAL_GetTick();

    return true;
}

bool fs_ai_protocol_decode_drive_f(const uCAN_MSG *msg, fs_ai_vcu_motor_feedback_t *motor)
{
    if (msg == NULL || motor == NULL || msg->frame.dlc < 6) {
        return false;
    }

    /* Bytes 4-5: Front axle torque max (little endian) */
    uint16_t torque_raw = ((uint16_t)msg->frame.data5 << 8) | msg->frame.data4;
    motor->front_axle_torque_max_nm = torque_raw * TORQUE_RES;

    return true;
}

bool fs_ai_protocol_decode_drive_r(const uCAN_MSG *msg, fs_ai_vcu_motor_feedback_t *motor)
{
    if (msg == NULL || motor == NULL || msg->frame.dlc < 6) {
        return false;
    }

    /* Bytes 4-5: Rear axle torque max (little endian) */
    uint16_t torque_raw = ((uint16_t)msg->frame.data5 << 8) | msg->frame.data4;
    motor->rear_axle_torque_max_nm = torque_raw * TORQUE_RES;

    return true;
}

bool fs_ai_protocol_decode_steer(const uCAN_MSG *msg, fs_ai_vcu_steer_feedback_t *steer)
{
    if (msg == NULL || steer == NULL || msg->frame.dlc < 6) {
        return false;
    }

    /* Bytes 0-1: Current steering angle (little endian, signed) */
    int16_t angle_raw = ((int16_t)msg->frame.data1 << 8) | msg->frame.data0;
    steer->steer_angle_deg = angle_raw * STEER_ANGLE_RES;

    /* Bytes 2-3: Max steering angle (little endian) */
    uint16_t max_angle_raw = ((uint16_t)msg->frame.data3 << 8) | msg->frame.data2;
    steer->steer_angle_max_deg = max_angle_raw * STEER_ANGLE_RES;

    return true;
}

bool fs_ai_protocol_decode_brake(const uCAN_MSG *msg, fs_ai_vcu_brake_feedback_t *brake)
{
    if (msg == NULL || brake == NULL || msg->frame.dlc < 5) {
        return false;
    }

    /* Byte 0: Front brake pressure */
    brake->brake_pressure_f_pct = msg->frame.data0 * BRAKE_PRESS_RES;

    /* Byte 2: Rear brake pressure */
    brake->brake_pressure_r_pct = msg->frame.data2 * BRAKE_PRESS_RES;

    return true;
}

bool fs_ai_protocol_decode_speeds(const uCAN_MSG *msg, fs_ai_vcu_wheel_speeds_t *speeds)
{
    if (msg == NULL || speeds == NULL || msg->frame.dlc < 8) {
        return false;
    }

    /* Bytes 0-1: FL wheel speed (little endian) */
    uint16_t fl_raw = ((uint16_t)msg->frame.data1 << 8) | msg->frame.data0;
    speeds->fl_wheel_speed_rpm = (float)fl_raw;

    /* Bytes 2-3: FR wheel speed (little endian) */
    uint16_t fr_raw = ((uint16_t)msg->frame.data3 << 8) | msg->frame.data2;
    speeds->fr_wheel_speed_rpm = (float)fr_raw;

    /* Bytes 4-5: RL wheel speed (little endian) */
    uint16_t rl_raw = ((uint16_t)msg->frame.data5 << 8) | msg->frame.data4;
    speeds->rl_wheel_speed_rpm = (float)rl_raw;

    /* Bytes 6-7: RR wheel speed (little endian) */
    uint16_t rr_raw = ((uint16_t)msg->frame.data7 << 8) | msg->frame.data6;
    speeds->rr_wheel_speed_rpm = (float)rr_raw;

    return true;
}
