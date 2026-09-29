#ifndef FC_POSITION_H
#define FC_POSITION_H

#include <stdbool.h>
#include "fc_types.h"

typedef struct {
    float gain_pos[3];
    float gain_vel_p[3];
    float gain_vel_i[3];
    float gain_vel_d[3];
    float lim_vel_h;
    float lim_vel_up;
    float lim_vel_down;
    float lim_thr_min;
    float lim_thr_max;
    float lim_thr_xy_margin;
    float lim_tilt;
    float lim_acc_h;         /* MPC_ACC_HOR_MAX: max horizontal accel (m/s^2) */
    float hover_thrust;
    float gravity;
    uint32_t decouple_alt;
    uint32_t z_up;
    float vel_int[3];
    float thr_sp_prev[3];
    float vel_prev[3];
    float vel_dot[3];
    float vel_dot_cutoff;
    uint32_t vel_dot_valid;
    uint32_t initialized;
} fc_position_config_t;

typedef struct {
    float position_ned[3];
    float velocity_ned[3];
    float acceleration_ned[3];
    float yaw_ned;
    float yawspeed;
} fc_position_setpoint_t;

typedef struct {
    float acc_sp_ned[3];
    float thrust_ned[3];
    float attitude_sp[4];
    float yaw_rate_sp;
    uint32_t valid;
} fc_position_out_t;

void FC_Position_ConfigDefault(fc_position_config_t *config);
void FC_Position_Reset(fc_position_config_t *config);
bool FC_Position_Update(fc_position_config_t *config, const fc_estimator_state_t *state,
                        const fc_position_setpoint_t *setpoint, float dt, fc_position_out_t *out);

#endif