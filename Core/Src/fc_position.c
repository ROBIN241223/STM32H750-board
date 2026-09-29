#include "fc_position.h"
#include <math.h>
#include <string.h>

#define FLT_EPS 1.0e-7f

static float clampf(float value, float minimum, float maximum)
{
    return fminf(fmaxf(value, minimum), maximum);
}

static float vdot3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static float vnorm2_3(const float a[3])
{
    return vdot3(a, a);
}

static float vnorm3(const float a[3])
{
    return sqrtf(vnorm2_3(a));
}

static void vscale3(float a[3], float scale)
{
    a[0] *= scale;
    a[1] *= scale;
    a[2] *= scale;
}

static void vnormalize3(float a[3])
{
    float n = vnorm3(a);
    if ((n > 0.0f) && isfinite(n)) {
        vscale3(a, 1.0f / n);
    }
}

static void vcross3(float out[3], const float a[3], const float b[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static void set_zero3(float a[3])
{
    a[0] = 0.0f;
    a[1] = 0.0f;
    a[2] = 0.0f;
}

static float v2_norm(const float a[2])
{
    return sqrtf(a[0] * a[0] + a[1] * a[1]);
}

void FC_Position_ConfigDefault(fc_position_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->gain_pos[0] = 0.95f;
    config->gain_pos[1] = 0.95f;
    config->gain_pos[2] = 1.0f;
    config->gain_vel_p[0] = 1.8f;
    config->gain_vel_p[1] = 1.8f;
    config->gain_vel_p[2] = 4.0f;
    config->gain_vel_i[0] = 0.4f;
    config->gain_vel_i[1] = 0.4f;
    config->gain_vel_i[2] = 2.0f;
    config->gain_vel_d[0] = 0.2f;
    config->gain_vel_d[1] = 0.2f;
    config->gain_vel_d[2] = 0.0f;
    config->vel_dot_cutoff = 5.0f;
    config->lim_vel_h = 12.0f;
    config->lim_vel_up = 3.0f;
    config->lim_vel_down = 1.5f;
    config->lim_thr_min = 0.12f;
    config->lim_thr_max = 1.0f;
    config->lim_thr_xy_margin = 0.3f;
    config->lim_tilt = 45.0f * 3.141592653589793f / 180.0f;
    config->lim_acc_h = 3.0f;   /* MPC_ACC_HOR_MAX */
    config->hover_thrust = 0.5f;
    config->gravity = 9.80665f;
    config->decouple_alt = 1U;
    config->z_up = 0U;  /* canonical mode: NED earth frame (z positive downward) */
    config->initialized = 1U;
}

void FC_Position_Reset(fc_position_config_t *config)
{
    if (config == NULL) {
        return;
    }
    set_zero3(config->vel_int);
    set_zero3(config->thr_sp_prev);
    set_zero3(config->vel_prev);
    set_zero3(config->vel_dot);
    config->vel_dot_valid = 0U;
    config->initialized = 1U;
}

static void constrain_xy(const float v0[2], const float v1[2], float max_norm, float out[2])
{
    float sum[2];
    float v0_norm;
    float v1_norm;
    sum[0] = v0[0] + v1[0];
    sum[1] = v0[1] + v1[1];
    if (v2_norm(sum) <= max_norm) {
        out[0] = sum[0];
        out[1] = sum[1];
        return;
    }
    v0_norm = v2_norm(v0);
    if (v0_norm >= max_norm) {
        out[0] = v0[0] / v0_norm * max_norm;
        out[1] = v0[1] / v0_norm * max_norm;
        return;
    }
    {
        float diff[2];
        diff[0] = v1[0] - v0[0];
        diff[1] = v1[1] - v0[1];
        if (v2_norm(diff) < 0.001f) {
            if (v0_norm > 0.0f) {
                out[0] = v0[0] / v0_norm * max_norm;
                out[1] = v0[1] / v0_norm * max_norm;
            } else {
                out[0] = 0.0f;
                out[1] = 0.0f;
            }
            return;
        }
        if (v0_norm < 0.001f) {
            v1_norm = v2_norm(v1);
            if (v1_norm > 0.0f) {
                out[0] = v1[0] / v1_norm * max_norm;
                out[1] = v1[1] / v1_norm * max_norm;
            } else {
                out[0] = 0.0f;
                out[1] = 0.0f;
            }
            return;
        }
        {
            float u1[2];
            float m;
            float c;
            float s;
            v1_norm = v2_norm(v1);
            u1[0] = v1[0] / v1_norm;
            u1[1] = v1[1] / v1_norm;
            m = u1[0] * v0[0] + u1[1] * v0[1];
            c = v0[0] * v0[0] + v0[1] * v0[1] - max_norm * max_norm;
            s = -m + sqrtf(fmaxf(0.0f, m * m - c));
            out[0] = v0[0] + u1[0] * s;
            out[1] = v0[1] + u1[1] * s;
        }
    }
}

static void limit_tilt(float body_unit[3], float max_angle)
{
    const float world_unit[3] = {0.0f, 0.0f, 1.0f};
    float dot;
    float angle;
    float cosine;
    float sine;
    dot = body_unit[2];
    angle = acosf(clampf(dot, -1.0f, 1.0f));
    if (angle > max_angle) {
        float rejection[3];
        float rejection_norm;
        cosine = cosf(max_angle);
        sine = sinf(max_angle);
        rejection[0] = body_unit[0];
        rejection[1] = body_unit[1];
        rejection[2] = body_unit[2] - dot;
        rejection_norm = vnorm3(rejection);
        if (rejection_norm < FLT_EPS) {
            rejection[0] = 1.0f;
            rejection[1] = 0.0f;
            rejection[2] = 0.0f;
            rejection_norm = 1.0f;
        }
        body_unit[0] = cosine * world_unit[0] + sine * rejection[0] / rejection_norm;
        body_unit[1] = cosine * world_unit[1] + sine * rejection[1] / rejection_norm;
        body_unit[2] = cosine * world_unit[2] + sine * rejection[2] / rejection_norm;
    }
}

static void matrix_to_unit_quat(float R[3][3], float q[4])
{
    float trace;
    float two;
    trace = R[0][0] + R[1][1] + R[2][2];
    if (trace > 0.0f) {
        two = sqrtf(trace + 1.0f) * 2.0f;
        q[0] = 0.25f * two;
        q[1] = (R[2][1] - R[1][2]) / two;
        q[2] = (R[0][2] - R[2][0]) / two;
        q[3] = (R[1][0] - R[0][1]) / two;
    } else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
        two = sqrtf(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
        q[0] = (R[2][1] - R[1][2]) / two;
        q[1] = 0.25f * two;
        q[2] = (R[0][1] + R[1][0]) / two;
        q[3] = (R[0][2] + R[2][0]) / two;
    } else if (R[1][1] > R[2][2]) {
        two = sqrtf(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
        q[0] = (R[0][2] - R[2][0]) / two;
        q[1] = (R[0][1] + R[1][0]) / two;
        q[2] = 0.25f * two;
        q[3] = (R[1][2] + R[2][1]) / two;
    } else {
        two = sqrtf(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
        q[0] = (R[1][0] - R[0][1]) / two;
        q[1] = (R[0][2] + R[2][0]) / two;
        q[2] = (R[1][2] + R[2][1]) / two;
        q[3] = 0.25f * two;
    }
    {
        float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if ((n > 0.0f) && isfinite(n)) {
            q[0] /= n;
            q[1] /= n;
            q[2] /= n;
            q[3] /= n;
        }
        if (q[0] < 0.0f) {
            q[0] = -q[0];
            q[1] = -q[1];
            q[2] = -q[2];
            q[3] = -q[3];
        }
    }
}

static void bodyz_to_attitude(const float body_z_in[3], float yaw, float q[4])
{
    float body_z[3];
    float body_x[3];
    float body_y[3];
    float y_c[3];
    float R[3][3];
    body_z[0] = body_z_in[0];
    body_z[1] = body_z_in[1];
    body_z[2] = body_z_in[2];
    if (vnorm2_3(body_z) < FLT_EPS) {
        body_z[0] = 0.0f;
        body_z[1] = 0.0f;
        body_z[2] = 1.0f;
    }
    vnormalize3(body_z);
    y_c[0] = -sinf(yaw);
    y_c[1] = cosf(yaw);
    y_c[2] = 0.0f;
    vcross3(body_x, y_c, body_z);
    if (body_z[2] < 0.0f) {
        body_x[0] = -body_x[0];
        body_x[1] = -body_x[1];
        body_x[2] = -body_x[2];
    }
    if (fabsf(body_z[2]) < 1.0e-6f) {
        body_x[0] = 0.0f;
        body_x[1] = 0.0f;
        body_x[2] = 1.0f;
    }
    vnormalize3(body_x);
    vcross3(body_y, body_z, body_x);
    R[0][0] = body_x[0];
    R[1][0] = body_x[1];
    R[2][0] = body_x[2];
    R[0][1] = body_y[0];
    R[1][1] = body_y[1];
    R[2][1] = body_y[2];
    R[0][2] = body_z[0];
    R[1][2] = body_z[1];
    R[2][2] = body_z[2];
    matrix_to_unit_quat(R, q);
}

bool FC_Position_Update(fc_position_config_t *config, const fc_estimator_state_t *state,
                        const fc_position_setpoint_t *setpoint, float dt, fc_position_out_t *out)
{
    float vel_sp[3];
    float vel_pos[3];
    float vel_ff[3];
    float vel_err[3];
    float acc_sp[3];
    float body_z[3];
    float z_specific_force;
    float thrust_ned_z;
    float cos_ned_body;
    float collective;
    float thrust_sp_xy[2];
    float thrust_sp_xy_norm;
    float thrust_max_squared;
    float allocated_horizontal_thrust;
    float thrust_z_max_squared;
    float thrust_max_xy_squared;
    float thrust_max_xy;
    float acc_sp_xy_produced[2];
    float acc_sp_xy[2];
    float arw_gain;

    if ((config == NULL) || (state == NULL) || (setpoint == NULL) || (out == NULL)) {
        return false;
    }
    if (config->initialized == 0U) {
        return false;
    }
    for (uint32_t i = 0U; i < 3U; i++) {
        vel_ff[i] = setpoint->velocity_ned[i];
        if (!isfinite(vel_ff[i])) {
            vel_ff[i] = 0.0f;
        }
        if (isfinite(setpoint->position_ned[i])) {
            vel_pos[i] = config->gain_pos[i] * (setpoint->position_ned[i] - state->pos_ned[i]);
        } else {
            vel_pos[i] = 0.0f;
        }
        vel_sp[i] = vel_ff[i] + vel_pos[i];
    }
    constrain_xy(vel_pos, vel_ff, config->lim_vel_h, vel_sp);
    vel_sp[2] = clampf(vel_sp[2], -config->lim_vel_up, config->lim_vel_down);

    {
        float alpha = 1.0f;
        if (config->vel_dot_cutoff > 0.0f) {
            alpha = 1.0f - expf(-2.0f * 3.141592653589793f * config->vel_dot_cutoff * dt);
        }
        if (config->vel_dot_valid == 0U) {
            for (uint32_t i = 0U; i < 3U; i++) {
                config->vel_prev[i] = state->vel_ned[i];
                config->vel_dot[i] = 0.0f;
            }
            config->vel_dot_valid = 1U;
        } else {
            for (uint32_t i = 0U; i < 3U; i++) {
                float raw_dot = (state->vel_ned[i] - config->vel_prev[i]) / dt;
                if (!isfinite(raw_dot)) {
                    raw_dot = 0.0f;
                    config->vel_prev[i] = state->vel_ned[i];
                }
                config->vel_dot[i] += alpha * (raw_dot - config->vel_dot[i]);
                config->vel_prev[i] = state->vel_ned[i];
            }
        }
    }

    for (uint32_t i = 0U; i < 3U; i++) {
        vel_err[i] = vel_sp[i] - state->vel_ned[i];
        acc_sp[i] = config->gain_vel_p[i] * vel_err[i] - config->gain_vel_d[i] * config->vel_dot[i] + config->vel_int[i];
        if (isfinite(setpoint->acceleration_ned[i])) {
            acc_sp[i] += setpoint->acceleration_ned[i];
        }
    }

    /* MPC_ACC_HOR_MAX: cap horizontal acceleration demand so a large cross-track
     * position error (e.g. entering a NAV_LOITER) never commands a violent tilt. */
    if (config->lim_acc_h > 0.0f) {
        float acc_h = sqrtf(acc_sp[0] * acc_sp[0] + acc_sp[1] * acc_sp[1]);
        if ((acc_h > config->lim_acc_h) && isfinite(acc_h)) {
            acc_sp[0] *= config->lim_acc_h / acc_h;
            acc_sp[1] *= config->lim_acc_h / acc_h;
        }
    }

    if (config->z_up != 0U) {
        body_z[0] = acc_sp[0];
        body_z[1] = acc_sp[1];
        body_z[2] = config->gravity;
        if (config->decouple_alt != 0U) {
            body_z[2] += acc_sp[2];
        }
        if (vnorm2_3(body_z) < FLT_EPS) {
            body_z[0] = 0.0f;
            body_z[1] = 0.0f;
            body_z[2] = 1.0f;
        }
        vnormalize3(body_z);
        limit_tilt(body_z, config->lim_tilt);
        thrust_ned_z = acc_sp[2] * (config->hover_thrust / config->gravity) + config->hover_thrust;
        cos_ned_body = body_z[2];
        if (cos_ned_body < 0.001f) {
            cos_ned_body = 0.001f;
        }
        collective = fmaxf(thrust_ned_z / cos_ned_body, config->lim_thr_min);
    } else {
        z_specific_force = -config->gravity;
        if (config->decouple_alt != 0U) {
            z_specific_force += acc_sp[2];
        }
        body_z[0] = -acc_sp[0];
        body_z[1] = -acc_sp[1];
        body_z[2] = -z_specific_force;
        if (vnorm2_3(body_z) < FLT_EPS) {
            body_z[0] = 0.0f;
            body_z[1] = 0.0f;
            body_z[2] = 1.0f;
        }
        vnormalize3(body_z);
        limit_tilt(body_z, config->lim_tilt);
        thrust_ned_z = acc_sp[2] * (config->hover_thrust / config->gravity) - config->hover_thrust;
        cos_ned_body = body_z[2];
        if (cos_ned_body < 0.001f) {
            cos_ned_body = 0.001f;
        }
        collective = fminf(thrust_ned_z / cos_ned_body, -config->lim_thr_min);
    }
    for (uint32_t i = 0U; i < 3U; i++) {
        float thr = body_z[i] * collective;
        if (!isfinite(thr)) {
            thr = 0.0f;
        }
        out->thrust_ned[i] = thr;
    }

    thrust_sp_xy[0] = out->thrust_ned[0];
    thrust_sp_xy[1] = out->thrust_ned[1];
    thrust_sp_xy_norm = v2_norm(thrust_sp_xy);
    thrust_max_squared = config->lim_thr_max * config->lim_thr_max;
    allocated_horizontal_thrust = fminf(thrust_sp_xy_norm, config->lim_thr_xy_margin);
    thrust_z_max_squared = thrust_max_squared - allocated_horizontal_thrust * allocated_horizontal_thrust;
    if (config->z_up != 0U) {
        if (out->thrust_ned[2] > sqrtf(fmaxf(0.0f, thrust_z_max_squared))) {
            out->thrust_ned[2] = sqrtf(fmaxf(0.0f, thrust_z_max_squared));
        }
    } else {
        if (out->thrust_ned[2] < -sqrtf(fmaxf(0.0f, thrust_z_max_squared))) {
            out->thrust_ned[2] = -sqrtf(fmaxf(0.0f, thrust_z_max_squared));
        }
    }
    thrust_max_xy_squared = thrust_max_squared - out->thrust_ned[2] * out->thrust_ned[2];
    thrust_max_xy = 0.0f;
    if (thrust_max_xy_squared > 0.0f) {
        thrust_max_xy = sqrtf(thrust_max_xy_squared);
    }
    if (thrust_sp_xy_norm > thrust_max_xy) {
        if (thrust_sp_xy_norm > 0.0f) {
            out->thrust_ned[0] = thrust_sp_xy[0] / thrust_sp_xy_norm * thrust_max_xy;
            out->thrust_ned[1] = thrust_sp_xy[1] / thrust_sp_xy_norm * thrust_max_xy;
        }
    }

    if (config->z_up != 0U) {
        if ((out->thrust_ned[2] <= config->lim_thr_min) && (vel_err[2] >= 0.0f)) {
            vel_err[2] = 0.0f;
        }
        if ((out->thrust_ned[2] >= config->lim_thr_max) && (vel_err[2] <= 0.0f)) {
            vel_err[2] = 0.0f;
        }
    } else {
        if ((out->thrust_ned[2] >= -config->lim_thr_min) && (vel_err[2] >= 0.0f)) {
            vel_err[2] = 0.0f;
        }
        if ((out->thrust_ned[2] <= -config->lim_thr_max) && (vel_err[2] <= 0.0f)) {
            vel_err[2] = 0.0f;
        }
    }

    acc_sp_xy_produced[0] = out->thrust_ned[0] * (config->gravity / config->hover_thrust);
    acc_sp_xy_produced[1] = out->thrust_ned[1] * (config->gravity / config->hover_thrust);
    acc_sp_xy[0] = acc_sp[0];
    acc_sp_xy[1] = acc_sp[1];
    if ((acc_sp[0] * acc_sp[0] + acc_sp[1] * acc_sp[1]) >
        (acc_sp_xy_produced[0] * acc_sp_xy_produced[0] + acc_sp_xy_produced[1] * acc_sp_xy_produced[1])) {
        arw_gain = 2.0f / config->gain_vel_p[0];
        vel_err[0] -= arw_gain * (acc_sp_xy[0] - acc_sp_xy_produced[0]);
        vel_err[1] -= arw_gain * (acc_sp_xy[1] - acc_sp_xy_produced[1]);
    }
    if (!isfinite(vel_err[0])) {
        vel_err[0] = 0.0f;
    }
    if (!isfinite(vel_err[1])) {
        vel_err[1] = 0.0f;
    }
    if (!isfinite(vel_err[2])) {
        vel_err[2] = 0.0f;
    }
    for (uint32_t i = 0U; i < 3U; i++) {
        config->vel_int[i] += vel_err[i] * config->gain_vel_i[i] * dt;
    }
    config->vel_int[2] = clampf(config->vel_int[2], -config->gravity, config->gravity);
    for (uint32_t i = 0U; i < 3U; i++) {
        config->thr_sp_prev[i] = out->thrust_ned[i];
        out->acc_sp_ned[i] = acc_sp[i];
    }

    {
        float body_z_down[3];
        if (config->z_up != 0U) {
            body_z_down[0] = out->thrust_ned[0];
            body_z_down[1] = out->thrust_ned[1];
            body_z_down[2] = out->thrust_ned[2];
        } else {
            body_z_down[0] = -out->thrust_ned[0];
            body_z_down[1] = -out->thrust_ned[1];
            body_z_down[2] = -out->thrust_ned[2];
        }
        bodyz_to_attitude(body_z_down, setpoint->yaw_ned, out->attitude_sp);
    }
    out->yaw_rate_sp = setpoint->yawspeed;
    if (!isfinite(out->attitude_sp[0]) || !isfinite(out->attitude_sp[1]) ||
        !isfinite(out->attitude_sp[2]) || !isfinite(out->attitude_sp[3])) {
        out->valid = 0U;
        return false;
    }
    out->valid = 1U;
    return true;
}