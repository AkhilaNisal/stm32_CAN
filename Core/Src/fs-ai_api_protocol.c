/**************************************************************************
 * Copyright: Preston EV Limited 2018, Rockfort Engineering Ltd. 2019, 2021
 *
 * File:    fs_ai_api.c (STM32 Embedded Version)
 * Author:  Ian Murphy (Linux), Adapted for STM32
 * Date:    2018-06-25, 2019-05-14, 2021-05-22, 2026-02-17
 *
 * Description: FS-AI API implementation for STM32 microcontroller
 *************************************************************************/

#include "fs-ai_api_protocol.h"
#include "CANSPI.h"
#include <string.h>
#include <stdlib.h>

/* ============================================================================
   Configuration Constants
   ============================================================================ */
#define MOTOR_RATIO             3.5f
#define MOTOR_MAX_RPM           4000.0f
#define STEER_ANGLE_RESOLUTION  0.1f        /* 0.1 degrees per unit */
#define BRAKE_PRESS_RESOLUTION  0.5f        /* 0.5% per unit */
#define TORQUE_RESOLUTION       0.1f        /* 0.1 Nm per unit */
#define ACCEL_RESOLUTION        1.0f        /* 1 m/s² per unit */
#define YAW_RATE_RESOLUTION     1.0f        /* 1 °/s per unit */

#define COMMS_TIMEOUT_MS        100         /* 100ms handshake timeout */
#define MESSAGE_TIMEOUT_MS      100         /* 100ms message timeout */

/* ============================================================================
   Type Definitions
   ============================================================================ */
typedef union {
    uint16_t uword;
    int16_t sword;
    uint8_t bytes[2];
} pack_16_t;

typedef union {
    uint8_t ubytes[8];
    int8_t sbytes[8];
    uint16_t uwords[4];
    int16_t swords[4];
    uint32_t ulongs[2];
    int32_t slongs[2];
    float floats[2];
} can_data_t;

/* ============================================================================
   Static Data - VCU to AI (RX)
   ============================================================================ */
static volatile struct {
    /* VCU2AI_Status (0x520) */
    uint8_t handshake_bit;
    uint8_t shutdown_request;
    uint8_t as_switch_status;
    uint8_t ts_switch_status;
    uint8_t go_signal;
    uint8_t steering_status;
    uint8_t as_state;
    uint8_t ami_state;
    uint8_t fault_status;
    uint8_t warning_status;
    uint8_t warn_batt_temp_high;
    uint8_t warn_batt_soc_low;
    uint8_t ai_estop_request;
    uint8_t hvil_open_fault;
    uint8_t hvil_short_fault;
    uint8_t ebs_fault;
    uint8_t charger_fault;
    uint8_t ai_comms_lost;
    uint8_t auto_braking_fault;
    uint8_t mission_status_fault;
    uint8_t charge_procedure_fault;
    uint8_t bms_fault;
    uint8_t brake_plausibility_fault;
    uint8_t shutdown_cause;
} VCU2AI_Status_raw;

static volatile struct {
    /* VCU2AI_Drive_F (0x521) */
    int16_t  front_axle_torque;
    uint16_t front_axle_torque_request;
    uint16_t front_axle_torque_max;     // <-- was uint8_t
} VCU2AI_Drive_F_raw;

static volatile struct {
    /* VCU2AI_Drive_R (0x522) */
    int16_t  rear_axle_torque;
    uint16_t rear_axle_torque_request;
    uint16_t rear_axle_torque_max;      // <-- was uint8_t
} VCU2AI_Drive_R_raw;

static volatile struct {
    /* VCU2AI_Steer (0x523) */
    int16_t  angle;
    uint16_t angle_max;                 // <-- was uint8_t
    int16_t  angle_request;
} VCU2AI_Steer_raw;

static volatile struct {
    /* VCU2AI_Brake (0x524) */
    uint8_t hyd_press_f;
    uint8_t hyd_press_f_req;
    uint8_t hyd_press_r;
    uint8_t hyd_press_r_req;
    uint8_t status_brk;
    uint8_t status_ebs;
} VCU2AI_Brake_raw;

static volatile struct {
    /* VCU2AI_Speeds (0x525) */
    uint16_t fl_wheel_speed;
    uint16_t fr_wheel_speed;
    uint16_t rl_wheel_speed;
    uint16_t rr_wheel_speed;
} VCU2AI_Speeds_raw;

static volatile struct {
    /* VCU2AI_Wheel_counts (0x526) */
    uint16_t fl_pulse_count;
    uint16_t fr_pulse_count;
    uint16_t rl_pulse_count;
    uint16_t rr_pulse_count;
} VCU2AI_WheelCounts_raw;

/* ============================================================================
   Static Data - AI to VCU (TX)
   ============================================================================ */
static volatile struct {
    /* AI2VCU_Status (0x510) */
    uint8_t handshake_bit;
    uint8_t estop_request;
    uint8_t mission_status;
    uint8_t direction_request;
    uint8_t lap_counter;
    uint8_t cones_count_actual;
    uint16_t cones_count_all;
    uint8_t veh_speed_actual;
    uint8_t veh_speed_demand;
} AI2VCU_Status_raw;

static volatile struct {
    /* AI2VCU_Drive_F (0x511) */
    uint16_t front_axle_torque_request;
    uint16_t front_motor_speed_max;
} AI2VCU_Drive_F_raw;

static volatile struct {
    /* AI2VCU_Drive_R (0x512) */
    uint16_t rear_axle_torque_request;
    uint16_t rear_motor_speed_max;
} AI2VCU_Drive_R_raw;

static volatile struct {
    /* AI2VCU_Steer (0x513) */
    int16_t steer_request;
} AI2VCU_Steer_raw;

static volatile struct {
    /* AI2VCU_Brake (0x514) */
    uint8_t hyd_press_f_req;
    uint8_t hyd_press_r_req;
} AI2VCU_Brake_raw;

static volatile struct {
    /* AI2LOG_Dynamics2 (0x501) */
    int16_t accel_longitudinal;
    int16_t accel_lateral;
    int16_t yaw_rate;
} AI2LOG_Dynamics2_raw;

/* ============================================================================
   Message Fresh Flags
   ============================================================================ */
static volatile bool VCU2AI_Status_fresh = false;
static volatile bool VCU2AI_Drive_F_fresh = false;
static volatile bool VCU2AI_Drive_R_fresh = false;
static volatile bool VCU2AI_Steer_fresh = false;
static volatile bool VCU2AI_Brake_fresh = false;
static volatile bool VCU2AI_Speeds_fresh = false;
static volatile bool VCU2AI_WheelCounts_fresh = false;

/* ============================================================================
   Statistics
   ============================================================================ */
static fs_ai_api_can_stats_t can_stats = {0};

/* ============================================================================
   Message Transmission Functions
   ============================================================================ */

static void send_ai2vcu_status(void) {
    uCAN_MSG msg;
    msg.frame.id = 0x510;
    msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg.frame.dlc = 8;

    /* Clear all data bytes */
    memset(&msg.frame.data0, 0, 8);

    /* Byte 0: Handshake(0), E-stop(1), Mission_status(2-3), Direction(4), Reserved(5-7) */
    msg.frame.data0 = (AI2VCU_Status_raw.handshake_bit & 0x01) |
                      ((AI2VCU_Status_raw.estop_request & 0x01) << 1) |
                      ((AI2VCU_Status_raw.mission_status & 0x03) << 2) |
                      ((AI2VCU_Status_raw.direction_request & 0x01) << 4);

    /* Byte 1: Lap counter */
    msg.frame.data1 = AI2VCU_Status_raw.lap_counter;

    /* Byte 2: Cones count actual */
    msg.frame.data2 = AI2VCU_Status_raw.cones_count_actual;

    /* Byte 3: Cones count all (low byte) */
    msg.frame.data3 = (uint8_t)(AI2VCU_Status_raw.cones_count_all & 0xFF);

    /* Byte 4: Cones count all (high byte) */
    msg.frame.data4 = (uint8_t)((AI2VCU_Status_raw.cones_count_all >> 8) & 0xFF);

    /* Byte 5: Vehicle speed actual */
    msg.frame.data5 = AI2VCU_Status_raw.veh_speed_actual;

    /* Byte 6: Vehicle speed demand */
    msg.frame.data6 = AI2VCU_Status_raw.veh_speed_demand;

    /* Byte 7: Reserved */
    msg.frame.data7 = 0;

    CANSPI_TransmitWait(&msg, 5);
    can_stats.AI2VCU_Status_count++;
}

static void send_ai2vcu_drive_f(void) {
    uCAN_MSG msg;
    msg.frame.id = 0x511;
    msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg.frame.dlc = 4;

    memset(&msg.frame.data0, 0, 8);

    /* Bytes 0-1: Front axle torque request (Little Endian) */
    msg.frame.data0 = (uint8_t)(AI2VCU_Drive_F_raw.front_axle_torque_request & 0xFF);
    msg.frame.data1 = (uint8_t)((AI2VCU_Drive_F_raw.front_axle_torque_request >> 8) & 0xFF);

    /* Bytes 2-3: Front motor speed max (Little Endian) */
    msg.frame.data2 = (uint8_t)(AI2VCU_Drive_F_raw.front_motor_speed_max & 0xFF);
    msg.frame.data3 = (uint8_t)((AI2VCU_Drive_F_raw.front_motor_speed_max >> 8) & 0xFF);

    CANSPI_TransmitWait(&msg, 5);
    can_stats.AI2VCU_Drive_F_count++;
}

static void send_ai2vcu_drive_r(void) {
    uCAN_MSG msg;
    msg.frame.id = 0x512;
    msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg.frame.dlc = 4;

    memset(&msg.frame.data0, 0, 8);

    /* Bytes 0-1: Rear axle torque request (Little Endian) */
    msg.frame.data0 = (uint8_t)(AI2VCU_Drive_R_raw.rear_axle_torque_request & 0xFF);
    msg.frame.data1 = (uint8_t)((AI2VCU_Drive_R_raw.rear_axle_torque_request >> 8) & 0xFF);

    /* Bytes 2-3: Rear motor speed max (Little Endian) */
    msg.frame.data2 = (uint8_t)(AI2VCU_Drive_R_raw.rear_motor_speed_max & 0xFF);
    msg.frame.data3 = (uint8_t)((AI2VCU_Drive_R_raw.rear_motor_speed_max >> 8) & 0xFF);

    CANSPI_TransmitWait(&msg, 5);
    can_stats.AI2VCU_Drive_R_count++;
}

static void send_ai2vcu_steer(void) {
    uCAN_MSG msg;
    msg.frame.id = 0x513;
    msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg.frame.dlc = 2;

    memset(&msg.frame.data0, 0, 8);

    /* Bytes 0-1: Steer request (Little Endian, signed) */
    msg.frame.data0 = (uint8_t)(AI2VCU_Steer_raw.steer_request & 0xFF);
    msg.frame.data1 = (uint8_t)((AI2VCU_Steer_raw.steer_request >> 8) & 0xFF);

    CANSPI_TransmitWait(&msg, 5);
    can_stats.AI2VCU_Steer_count++;
}

static void send_ai2vcu_brake(void) {
    uCAN_MSG msg;
    msg.frame.id = 0x514;
    msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg.frame.dlc = 2;

    memset(&msg.frame.data0, 0, 8);

    /* Byte 0: Front hydraulic pressure request */
    msg.frame.data0 = AI2VCU_Brake_raw.hyd_press_f_req;

    /* Byte 1: Rear hydraulic pressure request */
    msg.frame.data1 = AI2VCU_Brake_raw.hyd_press_r_req;

    CANSPI_TransmitWait(&msg, 5);
    can_stats.AI2VCU_Brake_count++;
}

static void send_ai2log_dynamics2(void) {
    uCAN_MSG msg;
    msg.frame.id = 0x501;
    msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
    msg.frame.dlc = 6;

    memset(&msg.frame.data0, 0, 8);

    /* Bytes 0-1: Longitudinal acceleration (Little Endian, signed) */
    msg.frame.data0 = (uint8_t)(AI2LOG_Dynamics2_raw.accel_longitudinal & 0xFF);
    msg.frame.data1 = (uint8_t)((AI2LOG_Dynamics2_raw.accel_longitudinal >> 8) & 0xFF);

    /* Bytes 2-3: Lateral acceleration (Little Endian, signed) */
    msg.frame.data2 = (uint8_t)(AI2LOG_Dynamics2_raw.accel_lateral & 0xFF);
    msg.frame.data3 = (uint8_t)((AI2LOG_Dynamics2_raw.accel_lateral >> 8) & 0xFF);

    /* Bytes 4-5: Yaw rate (Little Endian, signed) */
    msg.frame.data4 = (uint8_t)(AI2LOG_Dynamics2_raw.yaw_rate & 0xFF);
    msg.frame.data5 = (uint8_t)((AI2LOG_Dynamics2_raw.yaw_rate >> 8) & 0xFF);

    CANSPI_TransmitWait(&msg, 5);
}

/* ============================================================================
   Message Reception and Decoding
   ============================================================================ */

static void decode_vcu2ai_status(uCAN_MSG *rxMsg) {
    if (rxMsg == NULL || rxMsg->frame.dlc < 8) {
        return;
    }

    if (rxMsg == NULL || rxMsg->frame.dlc < 8) return;

    VCU2AI_Status_raw.handshake_bit     = (rxMsg->frame.data0 >> 0) & 0x01;
    VCU2AI_Status_raw.shutdown_request  = (rxMsg->frame.data0 >> 1) & 0x01;
    VCU2AI_Status_raw.as_switch_status  = (rxMsg->frame.data0 >> 2) & 0x01;
    VCU2AI_Status_raw.ts_switch_status  = (rxMsg->frame.data0 >> 3) & 0x01;
    VCU2AI_Status_raw.go_signal         = (rxMsg->frame.data0 >> 4) & 0x01;
    VCU2AI_Status_raw.steering_status   = (rxMsg->frame.data0 >> 5) & 0x01;

    VCU2AI_Status_raw.as_state  =  rxMsg->frame.data1        & 0x07;
    VCU2AI_Status_raw.ami_state = (rxMsg->frame.data1 >> 4)  & 0x07; // <-- FIX


    VCU2AI_Status_raw.fault_status = rxMsg->frame.data2 & 0x01;
    VCU2AI_Status_raw.warning_status = (rxMsg->frame.data2 >> 1) & 0x01;
    VCU2AI_Status_raw.warn_batt_temp_high = (rxMsg->frame.data2 >> 2) & 0x01;
    VCU2AI_Status_raw.warn_batt_soc_low = (rxMsg->frame.data2 >> 3) & 0x01;
    VCU2AI_Status_raw.ai_estop_request = (rxMsg->frame.data2 >> 4) & 0x01;
    VCU2AI_Status_raw.hvil_open_fault = (rxMsg->frame.data2 >> 5) & 0x01;
    VCU2AI_Status_raw.hvil_short_fault = (rxMsg->frame.data2 >> 6) & 0x01;
    VCU2AI_Status_raw.ebs_fault = (rxMsg->frame.data2 >> 7) & 0x01;

    VCU2AI_Status_raw.charger_fault = rxMsg->frame.data3 & 0x01;
    VCU2AI_Status_raw.ai_comms_lost = (rxMsg->frame.data3 >> 1) & 0x01;
    VCU2AI_Status_raw.auto_braking_fault = (rxMsg->frame.data3 >> 2) & 0x01;
    VCU2AI_Status_raw.mission_status_fault = (rxMsg->frame.data3 >> 3) & 0x01;
    VCU2AI_Status_raw.charge_procedure_fault = (rxMsg->frame.data3 >> 4) & 0x01;
    VCU2AI_Status_raw.bms_fault = (rxMsg->frame.data3 >> 5) & 0x01;
    VCU2AI_Status_raw.brake_plausibility_fault = (rxMsg->frame.data3 >> 6) & 0x01;

    VCU2AI_Status_raw.shutdown_cause = rxMsg->frame.data4 & 0x0F;

    VCU2AI_Status_fresh = true;
    can_stats.VCU2AI_Status_count++;
}

static void decode_vcu2ai_drive_f(uCAN_MSG *rxMsg) {
    if (rxMsg == NULL || rxMsg->frame.dlc < 6) return;

    VCU2AI_Drive_F_raw.front_axle_torque =
        (int16_t)((rxMsg->frame.data1 << 8) | rxMsg->frame.data0);

    VCU2AI_Drive_F_raw.front_axle_torque_request =
        (uint16_t)((rxMsg->frame.data3 << 8) | rxMsg->frame.data2);

    VCU2AI_Drive_F_raw.front_axle_torque_max =
        (uint16_t)((rxMsg->frame.data5 << 8) | rxMsg->frame.data4);

    VCU2AI_Drive_F_fresh = true;
    can_stats.VCU2AI_Drive_F_count++;
}

static void decode_vcu2ai_drive_r(uCAN_MSG *rxMsg) {
    if (rxMsg == NULL || rxMsg->frame.dlc < 6) return;

    VCU2AI_Drive_R_raw.rear_axle_torque =
        (int16_t)((rxMsg->frame.data1 << 8) | rxMsg->frame.data0);

    VCU2AI_Drive_R_raw.rear_axle_torque_request =
        (uint16_t)((rxMsg->frame.data3 << 8) | rxMsg->frame.data2);

    VCU2AI_Drive_R_raw.rear_axle_torque_max =
        (uint16_t)((rxMsg->frame.data5 << 8) | rxMsg->frame.data4);

    VCU2AI_Drive_R_fresh = true;
    can_stats.VCU2AI_Drive_R_count++;
}

static void decode_vcu2ai_steer(uCAN_MSG *rxMsg) {
    if (rxMsg == NULL || rxMsg->frame.dlc < 6) return;

    VCU2AI_Steer_raw.angle =
        (int16_t)((rxMsg->frame.data1 << 8) | rxMsg->frame.data0);

    VCU2AI_Steer_raw.angle_max =
        (uint16_t)((rxMsg->frame.data3 << 8) | rxMsg->frame.data2);

    VCU2AI_Steer_raw.angle_request =
        (int16_t)((rxMsg->frame.data5 << 8) | rxMsg->frame.data4);

    VCU2AI_Steer_fresh = true;
    can_stats.VCU2AI_Steer_count++;
}


static void decode_vcu2ai_brake(uCAN_MSG *rxMsg) {
    if (rxMsg == NULL || rxMsg->frame.dlc < 5) {
        return;
    }

    VCU2AI_Brake_raw.hyd_press_f = rxMsg->frame.data0;
    VCU2AI_Brake_raw.hyd_press_f_req = rxMsg->frame.data1;
    VCU2AI_Brake_raw.hyd_press_r = rxMsg->frame.data2;
    VCU2AI_Brake_raw.hyd_press_r_req = rxMsg->frame.data3;
    VCU2AI_Brake_raw.status_brk = rxMsg->frame.data4 & 0x07;
    VCU2AI_Brake_raw.status_ebs = (rxMsg->frame.data4 >> 3) & 0x03;

    VCU2AI_Brake_fresh = true;
    can_stats.VCU2AI_Brake_count++;
}

static void decode_vcu2ai_speeds(uCAN_MSG *rxMsg) {
    if (rxMsg == NULL || rxMsg->frame.dlc < 8) {
        return;
    }

    VCU2AI_Speeds_raw.fl_wheel_speed = (uint16_t)((rxMsg->frame.data1 << 8) | rxMsg->frame.data0);
    VCU2AI_Speeds_raw.fr_wheel_speed = (uint16_t)((rxMsg->frame.data3 << 8) | rxMsg->frame.data2);
    VCU2AI_Speeds_raw.rl_wheel_speed = (uint16_t)((rxMsg->frame.data5 << 8) | rxMsg->frame.data4);
    VCU2AI_Speeds_raw.rr_wheel_speed = (uint16_t)((rxMsg->frame.data7 << 8) | rxMsg->frame.data6);

    VCU2AI_Speeds_fresh = true;
    can_stats.VCU2AI_Wheel_speeds_count++;
}

static void decode_vcu2ai_wheel_counts(uCAN_MSG *rxMsg) {
    if (rxMsg == NULL || rxMsg->frame.dlc < 8) {
        return;
    }

    VCU2AI_WheelCounts_raw.fl_pulse_count = (uint16_t)((rxMsg->frame.data1 << 8) | rxMsg->frame.data0);
    VCU2AI_WheelCounts_raw.fr_pulse_count = (uint16_t)((rxMsg->frame.data3 << 8) | rxMsg->frame.data2);
    VCU2AI_WheelCounts_raw.rl_pulse_count = (uint16_t)((rxMsg->frame.data5 << 8) | rxMsg->frame.data4);
    VCU2AI_WheelCounts_raw.rr_pulse_count = (uint16_t)((rxMsg->frame.data7 << 8) | rxMsg->frame.data6);

    VCU2AI_WheelCounts_fresh = true;
    can_stats.VCU2AI_Wheel_counts_count++;
}

/* ============================================================================
   Public API Functions
   ============================================================================ */

bool fs_ai_api_init(void) {
    /* Initialize CAN bus */
    if (!CANSPI_Initialize()) {
        return false;
    }

    /* Clear all statistics */
    memset(&can_stats, 0, sizeof(fs_ai_api_can_stats_t));

    return true;
}

void fs_ai_api_process_incoming(uCAN_MSG *rxMsg) {
    if (rxMsg == NULL) {
        return;
    }

    switch (rxMsg->frame.id) {
        case 0x520:
            decode_vcu2ai_status(rxMsg);
            break;
        case 0x521:
            decode_vcu2ai_drive_f(rxMsg);
            break;
        case 0x522:
            decode_vcu2ai_drive_r(rxMsg);
            break;
        case 0x523:
            decode_vcu2ai_steer(rxMsg);
            break;
        case 0x524:
            decode_vcu2ai_brake(rxMsg);
            break;
        case 0x525:
            decode_vcu2ai_speeds(rxMsg);
            break;
        case 0x526:
            decode_vcu2ai_wheel_counts(rxMsg);
            break;
        default:
            can_stats.unhandled_frame_count++;
            break;
    }
}

void fs_ai_api_vcu2ai_get_data(fs_ai_api_vcu2ai *data) {
    if (data == NULL) {
        return;
    }

    memset(data, 0, sizeof(fs_ai_api_vcu2ai));

    /* Extract VCU2AI_Status */
    if (VCU2AI_Status_fresh) {
        VCU2AI_Status_fresh = false;
        data->VCU2AI_HANDSHAKE_RECEIVE_BIT = VCU2AI_Status_raw.handshake_bit;
        data->VCU2AI_RES_GO_SIGNAL = VCU2AI_Status_raw.go_signal;
        data->VCU2AI_AS_STATE = VCU2AI_Status_raw.as_state;
        data->VCU2AI_AMI_STATE = VCU2AI_Status_raw.ami_state;
        data->VCU2AI_FAULT_STATUS = VCU2AI_Status_raw.fault_status;
        data->VCU2AI_WARNING_STATUS = VCU2AI_Status_raw.warning_status;
        data->VCU2AI_WARN_BATT_TEMP_HIGH = VCU2AI_Status_raw.warn_batt_temp_high;
        data->VCU2AI_WARN_BATT_SOC_LOW = VCU2AI_Status_raw.warn_batt_soc_low;
        data->VCU2AI_AI_ESTOP_REQUEST = VCU2AI_Status_raw.ai_estop_request;
        data->VCU2AI_HVIL_OPEN_FAULT = VCU2AI_Status_raw.hvil_open_fault;
        data->VCU2AI_HVIL_SHORT_FAULT = VCU2AI_Status_raw.hvil_short_fault;
        data->VCU2AI_EBS_FAULT = VCU2AI_Status_raw.ebs_fault;
        data->VCU2AI_CHARGER_FAULT = VCU2AI_Status_raw.charger_fault;
        data->VCU2AI_AI_COMMS_LOST = VCU2AI_Status_raw.ai_comms_lost;
        data->VCU2AI_AUTO_BRAKING_FAULT = VCU2AI_Status_raw.auto_braking_fault;
        data->VCU2AI_MISSION_STATUS_FAULT = VCU2AI_Status_raw.mission_status_fault;
        data->VCU2AI_CHARGE_PROCEDURE_FAULT = VCU2AI_Status_raw.charge_procedure_fault;
        data->VCU2AI_BMS_FAULT = VCU2AI_Status_raw.bms_fault;
        data->VCU2AI_BRAKE_PLAUSIBILITY_FAULT = VCU2AI_Status_raw.brake_plausibility_fault;
    }

    /* Extract VCU2AI_Drive_F and _Drive_R */
    if (VCU2AI_Drive_F_fresh) {
        VCU2AI_Drive_F_fresh = false;
        data->VCU2AI_FRONT_AXLE_TORQUE_MAX_Nm = TORQUE_RESOLUTION * VCU2AI_Drive_F_raw.front_axle_torque_max;
    }

    if (VCU2AI_Drive_R_fresh) {
        VCU2AI_Drive_R_fresh = false;
        data->VCU2AI_REAR_AXLE_TORQUE_MAX_Nm = TORQUE_RESOLUTION * VCU2AI_Drive_R_raw.rear_axle_torque_max;
    }

    /* Extract VCU2AI_Steer */
    if (VCU2AI_Steer_fresh) {
        VCU2AI_Steer_fresh = false;
        data->VCU2AI_STEER_ANGLE_deg = STEER_ANGLE_RESOLUTION * VCU2AI_Steer_raw.angle;
        data->VCU2AI_STEER_ANGLE_MAX_deg = STEER_ANGLE_RESOLUTION * VCU2AI_Steer_raw.angle_max;
    }

    /* Extract VCU2AI_Brake */
    if (VCU2AI_Brake_fresh) {
        VCU2AI_Brake_fresh = false;
        data->VCU2AI_BRAKE_PRESS_F_pct = BRAKE_PRESS_RESOLUTION * VCU2AI_Brake_raw.hyd_press_f;
        data->VCU2AI_BRAKE_PRESS_R_pct = BRAKE_PRESS_RESOLUTION * VCU2AI_Brake_raw.hyd_press_r;
    }

    /* Extract VCU2AI_Speeds */
    if (VCU2AI_Speeds_fresh) {
        VCU2AI_Speeds_fresh = false;
        data->VCU2AI_FL_WHEEL_SPEED_rpm = (float)VCU2AI_Speeds_raw.fl_wheel_speed;
        data->VCU2AI_FR_WHEEL_SPEED_rpm = (float)VCU2AI_Speeds_raw.fr_wheel_speed;
        data->VCU2AI_RL_WHEEL_SPEED_rpm = (float)VCU2AI_Speeds_raw.rl_wheel_speed;
        data->VCU2AI_RR_WHEEL_SPEED_rpm = (float)VCU2AI_Speeds_raw.rr_wheel_speed;
    }

    /* Extract VCU2AI_Wheel_counts */
    if (VCU2AI_WheelCounts_fresh) {
        VCU2AI_WheelCounts_fresh = false;
        data->VCU2AI_FL_PULSE_COUNT = VCU2AI_WheelCounts_raw.fl_pulse_count;
        data->VCU2AI_FR_PULSE_COUNT = VCU2AI_WheelCounts_raw.fr_pulse_count;
        data->VCU2AI_RL_PULSE_COUNT = VCU2AI_WheelCounts_raw.rl_pulse_count;
        data->VCU2AI_RR_PULSE_COUNT = VCU2AI_WheelCounts_raw.rr_pulse_count;
    }
}

void fs_ai_api_ai2vcu_set_data(fs_ai_api_ai2vcu *data) {
    if (data == NULL) {
        return;
    }

    /* Local validation variables */
    float front_torque_request = data->AI2VCU_FRONT_AXLE_TORQUE_REQUEST_Nm;
    float rear_torque_request = data->AI2VCU_REAR_AXLE_TORQUE_REQUEST_Nm;
    float front_brake_request = data->AI2VCU_BRAKE_PRESS_F_REQUEST_pct;
    float rear_brake_request = data->AI2VCU_BRAKE_PRESS_R_REQUEST_pct;
    float steer_request = data->AI2VCU_STEER_ANGLE_REQUEST_deg;
    float front_motor_speed = data->AI2VCU_FRONT_MOTOR_SPEED_REQUEST_rpm;
    float rear_motor_speed = data->AI2VCU_REAR_MOTOR_SPEED_REQUEST_rpm;

    /* ===== Input Validation ===== */

    /* Validate steering angle */
    if (steer_request > 21.0f) { steer_request = 21.0f; }
    if (steer_request < -21.0f) { steer_request = -21.0f; }

    /* Validate motor speeds */
    if (front_motor_speed > MOTOR_MAX_RPM) { front_motor_speed = MOTOR_MAX_RPM; }
    if (front_motor_speed < 0.0f) { front_motor_speed = 0.0f; }
    if (rear_motor_speed > MOTOR_MAX_RPM) { rear_motor_speed = MOTOR_MAX_RPM; }
    if (rear_motor_speed < 0.0f) { rear_motor_speed = 0.0f; }

    /* Validate torque requests */
    if (front_torque_request > 195.0f) { front_torque_request = 195.0f; }
    if (front_torque_request < 0.0f) { front_torque_request = 0.0f; }
    if (rear_torque_request > 195.0f) { rear_torque_request = 195.0f; }
    if (rear_torque_request < 0.0f) { rear_torque_request = 0.0f; }

    /* Validate brake pressure requests */
    if (front_brake_request > 100.0f) { front_brake_request = 100.0f; }
    if (front_brake_request < 0.0f) { front_brake_request = 0.0f; }
    if (rear_brake_request > 100.0f) { rear_brake_request = 100.0f; }
    if (rear_brake_request < 0.0f) { rear_brake_request = 0.0f; }

    /* Brake plausibility check: Cannot have both torque AND brake simultaneously */
    if ((front_brake_request > 0.0f) || (rear_brake_request > 0.0f)) {
        front_torque_request = 0.0f;
        rear_torque_request = 0.0f;
    }

    /* Validate enum values */
    uint8_t mission_status = data->AI2VCU_MISSION_STATUS;
    uint8_t direction_request = data->AI2VCU_DIRECTION_REQUEST;
    uint8_t estop_request = data->AI2VCU_ESTOP_REQUEST;
    uint8_t handshake_bit = data->AI2VCU_HANDSHAKE_SEND_BIT;

    if (mission_status > MISSION_FINISHED) { mission_status = MISSION_FINISHED; }
    if (direction_request > DIRECTION_FORWARD) { direction_request = DIRECTION_FORWARD; }
    if (estop_request > ESTOP_YES) { estop_request = ESTOP_YES; }
    if (handshake_bit > HANDSHAKE_SEND_BIT_ON) { handshake_bit = HANDSHAKE_SEND_BIT_ON; }

    /* ===== Store in Raw Format ===== */

    AI2VCU_Status_raw.handshake_bit = handshake_bit;
    AI2VCU_Status_raw.estop_request = estop_request;
    AI2VCU_Status_raw.mission_status = mission_status;
    AI2VCU_Status_raw.direction_request = direction_request;
    AI2VCU_Status_raw.lap_counter = data->AI2VCU_LAP_COUNTER;
    AI2VCU_Status_raw.cones_count_actual = data->AI2VCU_CONES_COUNT_ACTUAL;
    AI2VCU_Status_raw.cones_count_all = data->AI2VCU_CONES_COUNT_ALL;
    AI2VCU_Status_raw.veh_speed_actual = data->AI2VCU_VEH_SPEED_ACTUAL_kmh;
    AI2VCU_Status_raw.veh_speed_demand = data->AI2VCU_VEH_SPEED_DEMAND_kmh;

    AI2VCU_Steer_raw.steer_request = (int16_t)(steer_request / STEER_ANGLE_RESOLUTION);

    AI2VCU_Drive_F_raw.front_axle_torque_request = (uint16_t)(front_torque_request / TORQUE_RESOLUTION);
    AI2VCU_Drive_F_raw.front_motor_speed_max = (uint16_t)front_motor_speed;

    AI2VCU_Drive_R_raw.rear_axle_torque_request = (uint16_t)(rear_torque_request / TORQUE_RESOLUTION);
    AI2VCU_Drive_R_raw.rear_motor_speed_max = (uint16_t)rear_motor_speed;

    AI2VCU_Brake_raw.hyd_press_f_req = (uint8_t)(front_brake_request / BRAKE_PRESS_RESOLUTION);
    AI2VCU_Brake_raw.hyd_press_r_req = (uint8_t)(rear_brake_request / BRAKE_PRESS_RESOLUTION);

    /* Store dynamics data for logging */
    AI2LOG_Dynamics2_raw.accel_longitudinal = (int16_t)(data->AI2LOG_ACCEL_LONGITUDINAL_ms2 / ACCEL_RESOLUTION);
    AI2LOG_Dynamics2_raw.accel_lateral = (int16_t)(data->AI2LOG_ACCEL_LATERAL_ms2 / ACCEL_RESOLUTION);
    AI2LOG_Dynamics2_raw.yaw_rate = (int16_t)(data->AI2LOG_YAW_RATE_degps / YAW_RATE_RESOLUTION);
}

void fs_ai_api_send_all_messages(void) {
    /* Send AI to VCU messages in order with controlled timing */
    send_ai2vcu_status();
    send_ai2vcu_drive_f();
    send_ai2vcu_drive_r();
    send_ai2vcu_steer();
    send_ai2vcu_brake();
    send_ai2log_dynamics2();
}

void fs_ai_api_get_can_stats(fs_ai_api_can_stats_t *stats) {
    if (stats == NULL) {
        return;
    }

    memcpy(stats, (void *)&can_stats, sizeof(fs_ai_api_can_stats_t));
}

void fs_ai_api_clear_can_stats(void) {
    memset(&can_stats, 0, sizeof(fs_ai_api_can_stats_t));
}

bool fs_ai_api_is_handshake_complete(void) {
    /* Check if we've received at least one VCU2AI_Status message with handshake */
    return (VCU2AI_Status_raw.handshake_bit == HANDSHAKE_RECEIVE_BIT_ON) &&
           (can_stats.VCU2AI_Status_count > 0);
}

bool fs_ai_api_is_comms_healthy(void) {
    /* Check if we're receiving messages regularly */
    return (can_stats.VCU2AI_Status_count > 0) &&
           (can_stats.VCU2AI_Drive_F_count > 0) &&
           (can_stats.VCU2AI_Drive_R_count > 0) &&
           (can_stats.VCU2AI_Steer_count > 0) &&
           (can_stats.VCU2AI_Brake_count > 0) &&
           (VCU2AI_Status_raw.ai_comms_lost == 0);
}
