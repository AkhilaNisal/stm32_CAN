/**
 * @file    ai_controller.c
 * @brief   AI Algorithm Interface implementation
 */

#include "ai_controller.h"
#include <string.h>

/* ============================================================================
   STATIC STATE
   ============================================================================ */

static struct {
    ai_control_state_t state;
    fs_ai_ai_data_t ai_data;
    fs_ai_vcu_data_t vcu_data;
} ai_state = {
    .state = AI_CONTROL_STATE_IDLE,
    .ai_data = {{0}},
    .vcu_data = {{0}}
};

/* ============================================================================
   INITIALIZATION
   ============================================================================ */

bool ai_controller_init(void)
{
    /* Initialize transport layer */
    if (!can_transport_init()) {
        ai_state.state = AI_CONTROL_STATE_FAULT;
        return false;
    }

    /* Clear all data */
    memset(&ai_state.ai_data, 0, sizeof(fs_ai_ai_data_t));
    memset(&ai_state.vcu_data, 0, sizeof(fs_ai_vcu_data_t));

    ai_state.state = AI_CONTROL_STATE_IDLE;
    return true;
}

void ai_controller_deinit(void)
{
    can_transport_deinit();
    ai_state.state = AI_CONTROL_STATE_IDLE;
}

/* ============================================================================
   MAIN LOOP
   ============================================================================ */

ai_control_state_t ai_controller_update(uint32_t current_time_ms)
{
    /* Update CAN transport (handles all timing) */
    can_transport_state_t transport_state = can_transport_update(
        current_time_ms,
        &ai_state.ai_data,
        &ai_state.vcu_data
    );

    /* Map transport state to AI controller state */
    switch (transport_state) {
        case TRANSPORT_STATE_INIT:
            ai_state.state = AI_CONTROL_STATE_IDLE;
            break;

        case TRANSPORT_STATE_HANDSHAKE:
            ai_state.state = AI_CONTROL_STATE_IDLE;
            break;

        case TRANSPORT_STATE_READY:
            ai_state.state = AI_CONTROL_STATE_READY;
            break;

        case TRANSPORT_STATE_ERROR:
            ai_state.state = AI_CONTROL_STATE_FAULT;
            break;

        default:
            ai_state.state = AI_CONTROL_STATE_FAULT;
            break;
    }

    return ai_state.state;
}

ai_control_state_t ai_controller_get_state(void)
{
    return ai_state.state;
}

/* ============================================================================
   CONTROL COMMANDS
   ============================================================================ */

void ai_set_control(
    float torque_nm,
    float steering_deg,
    float brake_pct,
    float motor_speed_rpm
)
{
    /* Store in AI data structure for next transmit */
    ai_state.ai_data.control.front_torque_nm = torque_nm;
    ai_state.ai_data.control.rear_torque_nm = torque_nm;
    ai_state.ai_data.control.front_motor_speed_rpm = motor_speed_rpm;
    ai_state.ai_data.control.rear_motor_speed_rpm = motor_speed_rpm;
    ai_state.ai_data.control.steer_angle_deg = steering_deg;
    ai_state.ai_data.control.brake_pressure_f_pct = brake_pct;
    ai_state.ai_data.control.brake_pressure_r_pct = brake_pct;
}

void ai_set_mission_status(
    fs_ai_mission_status_t mission,
    fs_ai_direction_t direction,
    fs_ai_estop_t estop
)
{
    ai_state.ai_data.status.mission_status = mission;
    ai_state.ai_data.status.direction = direction;
    ai_state.ai_data.status.estop = estop;
}

void ai_set_mission_progress(
    uint8_t lap_count,
    uint8_t cone_count
)
{
    ai_state.ai_data.status.lap_counter = lap_count;
    ai_state.ai_data.status.cones_count = cone_count;  /* FIXED: cones_count not cone_count */
}

void ai_set_dynamics(
    float accel_long,
    float accel_lat,
    float yaw_rate
)
{
    ai_state.ai_data.dynamics.accel_longitudinal_ms2 = (int16_t)accel_long;
    ai_state.ai_data.dynamics.accel_lateral_ms2 = (int16_t)accel_lat;
    ai_state.ai_data.dynamics.yaw_rate_degps = (int16_t)yaw_rate;
}

/* ============================================================================
   VEHICLE STATE FEEDBACK
   ============================================================================ */

bool ai_get_vehicle_state(fs_ai_vcu_data_t *vcu_data)
{
    if (vcu_data == NULL) {
        return false;
    }

    /* Check if data is recent */
    if (can_transport_get_last_vcu_msg_age_ms() > 200) {
        return false;
    }

    memcpy(vcu_data, &ai_state.vcu_data, sizeof(fs_ai_vcu_data_t));
    return true;
}

bool ai_get_wheel_speeds(
    float *fl,
    float *fr,
    float *rl,
    float *rr
)
{
    if (fl == NULL || fr == NULL || rl == NULL || rr == NULL) {
        return false;
    }

    if (can_transport_get_last_vcu_msg_age_ms() > 200) {
        return false;
    }

    *fl = ai_state.vcu_data.speeds.fl_wheel_speed_rpm;
    *fr = ai_state.vcu_data.speeds.fr_wheel_speed_rpm;
    *rl = ai_state.vcu_data.speeds.rl_wheel_speed_rpm;
    *rr = ai_state.vcu_data.speeds.rr_wheel_speed_rpm;

    return true;
}

float ai_get_average_wheel_speed(void)
{
    float avg = (ai_state.vcu_data.speeds.fl_wheel_speed_rpm +
                 ai_state.vcu_data.speeds.fr_wheel_speed_rpm +
                 ai_state.vcu_data.speeds.rl_wheel_speed_rpm +
                 ai_state.vcu_data.speeds.rr_wheel_speed_rpm) / 4.0f;
    return avg;
}

float ai_get_steering_feedback(void)
{
    return ai_state.vcu_data.steer.steer_angle_deg;
}

bool ai_get_brake_feedback(float *brake_f, float *brake_r)
{
    if (brake_f == NULL || brake_r == NULL) {
        return false;
    }

    *brake_f = ai_state.vcu_data.brake.brake_pressure_f_pct;
    *brake_r = ai_state.vcu_data.brake.brake_pressure_r_pct;

    return true;
}

bool ai_get_vcu_status(fs_ai_vcu_status_t *status)
{
    if (status == NULL) {
        return false;
    }

    if (can_transport_get_last_vcu_msg_age_ms() > 200) {
        return false;
    }

    memcpy(status, &ai_state.vcu_data.status, sizeof(fs_ai_vcu_status_t));
    return true;
}

/* ============================================================================
   DIAGNOSTICS
   ============================================================================ */

bool ai_is_healthy(void)
{
    can_health_status_t health = can_transport_get_health();
    return (health == CAN_HEALTH_OK && !can_transport_is_error());
}

can_health_status_t ai_get_can_health(void)
{
    return can_transport_get_health();
}

bool ai_is_ready_for_operation(void)
{
    /* Check transport is ready */
    if (!can_transport_is_ready()) {
        return false;
    }

    /* Check health */
    if (!ai_is_healthy()) {
        return false;
    }

    /* Check no recent errors */
    if (can_transport_get_last_vcu_msg_age_ms() > 200) {
        return false;
    }

    return true;
}

can_interface_stats_t ai_get_diagnostics(void)
{
    return can_transport_get_stats();
}

void ai_clear_diagnostics(void)
{
    can_transport_clear_stats();
}

uint32_t ai_get_vcu_msg_age_ms(void)
{
    return can_transport_get_last_vcu_msg_age_ms();
}
