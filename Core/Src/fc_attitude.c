#include "fc_attitude.h"
#include <math.h>
#include <string.h>

#define FLT_EPS_ATT 1.0e-5f

static float clampf_att(float value, float minimum, float maximum)
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

static void qmul(const float a[4], const float b[4], float out[4])
{
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static void qnormalize(float q[4])
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!isfinite(n) || (n < 1.0e-6f)) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
}

static void qcanonical(float q[4])
{
    if (q[0] < 0.0f) {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }
}

static void q_dcm_z(const float q[4], float z[3])
{
    z[0] = 2.0f * (q[1] * q[3] + q[0] * q[2]);
    z[1] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
    z[2] = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
}

static void q_from_two_vecs(const float v1[3], const float v2[3], float out[4])
{
    float cross[3];
    float axis[3];
    float sine;
    float dot;

    vcross3(cross, v1, v2);
    sine = vnorm3(cross);
    dot = vdot3(v1, v2);
    if (sine < 1.0e-6f) {
        if (dot < 0.0f) {
            float ref[3];
            float d;
            ref[0] = 1.0f;
            ref[1] = 0.0f;
            ref[2] = 0.0f;
            d = vdot3(ref, v1);
            if (fabsf(d) > 0.9f) {
                ref[0] = 0.0f;
                ref[1] = 1.0f;
                ref[2] = 0.0f;
            }
            vcross3(axis, ref, v1);
            vnormalize3(axis);
            out[0] = 0.0f;
            out[1] = axis[0];
            out[2] = axis[1];
            out[3] = axis[2];
        } else {
            out[0] = 1.0f;
            out[1] = 0.0f;
            out[2] = 0.0f;
            out[3] = 0.0f;
        }
    } else {
        float half = atan2f(sine, dot) * 0.5f;
        axis[0] = cross[0] / sine;
        axis[1] = cross[1] / sine;
        axis[2] = cross[2] / sine;
        out[0] = cosf(half);
        out[1] = axis[0] * sinf(half);
        out[2] = axis[1] * sinf(half);
        out[3] = axis[2] * sinf(half);
    }
    if (out[0] < 0.0f) {
        out[0] = -out[0];
        out[1] = -out[1];
        out[2] = -out[2];
        out[3] = -out[3];
    }
}

static void q_rotate_vec(const float q[4], const float v[3], float out[3])
{
    float R[3][3];
    R[0][0] = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
    R[1][1] = 1.0f - 2.0f * (q[1] * q[1] + q[3] * q[3]);
    R[2][2] = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
    R[0][1] = 2.0f * (q[1] * q[2] - q[0] * q[3]);
    R[1][0] = 2.0f * (q[1] * q[2] + q[0] * q[3]);
    R[0][2] = 2.0f * (q[1] * q[3] + q[0] * q[2]);
    R[2][0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    R[1][2] = 2.0f * (q[2] * q[3] - q[0] * q[1]);
    R[2][1] = 2.0f * (q[2] * q[3] + q[0] * q[1]);
    out[0] = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    out[1] = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    out[2] = R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2];
}

/* PX4 attitude control: rate_sp = MC_ATT_P * attitude_error (tilt + yaw), plus
   the yaw rate feedforward rotated into body frame.  No rate loop here -- the
   requested body rates are handed to fc_rate.c which owns P/I/D + FF. */
static void attitude_error_to_rate_sp(const fc_attitude_config_t *config, const float q[4],
                                      const float qd_in[4], float yaw_rate_sp, float rate_sp[3])
{
    float qd[4];
    float e_z[3];
    float e_z_d[3];
    float qd_red[4];
    float qd_red_buf[4];
    float qd_dyaw[4];
    float qe[4];
    float eq[3];
    float q_rel[4];
    float yaw_vec_world[3];
    float yaw_vec_body[3];
    float w_dyaw;
    float z_dyaw;

    qd[0] = qd_in[0];
    qd[1] = qd_in[1];
    qd[2] = qd_in[2];
    qd[3] = qd_in[3];
    qnormalize(qd);

    q_dcm_z(q, e_z);
    q_dcm_z(qd, e_z_d);
    q_from_two_vecs(e_z, e_z_d, qd_red);

    if ((fabsf(qd_red[1]) > (1.0f - FLT_EPS_ATT)) || (fabsf(qd_red[2]) > (1.0f - FLT_EPS_ATT))) {
        qd_red[0] = qd[0];
        qd_red[1] = qd[1];
        qd_red[2] = qd[2];
        qd_red[3] = qd[3];
    } else {
        qmul(qd_red, q, qd_red_buf);
        qd_red[0] = qd_red_buf[0];
        qd_red[1] = qd_red_buf[1];
        qd_red[2] = qd_red_buf[2];
        qd_red[3] = qd_red_buf[3];
    }
    qnormalize(qd_red);

    {
        float q_inv[4];
        q_inv[0] = qd_red[0];
        q_inv[1] = -qd_red[1];
        q_inv[2] = -qd_red[2];
        q_inv[3] = -qd_red[3];
        qmul(q_inv, qd, qd_dyaw);
    }
    qcanonical(qd_dyaw);
    w_dyaw = clampf_att(qd_dyaw[0], -1.0f, 1.0f);
    z_dyaw = clampf_att(qd_dyaw[3], -1.0f, 1.0f);

    {
        float dyaw[4];
        dyaw[0] = cosf(config->yaw_w * acosf(w_dyaw));
        dyaw[1] = 0.0f;
        dyaw[2] = 0.0f;
        dyaw[3] = sinf(config->yaw_w * asinf(z_dyaw));
        qmul(qd_red, dyaw, qd);
    }
    qcanonical(qd);

    {
        float q_inv[4];
        q_inv[0] = q[0];
        q_inv[1] = -q[1];
        q_inv[2] = -q[2];
        q_inv[3] = -q[3];
        qmul(q_inv, qd, qe);
    }
    qcanonical(qe);
    eq[0] = 2.0f * qe[1];
    eq[1] = 2.0f * qe[2];
    eq[2] = 2.0f * qe[3];

    for (uint32_t i = 0U; i < 3U; i++) {
        rate_sp[i] = clampf_att(config->gain_att[i] * eq[i], -config->lim_rate[i], config->lim_rate[i]);
    }

    {
        float q_inv[4];
        q_inv[0] = q[0];
        q_inv[1] = -q[1];
        q_inv[2] = -q[2];
        q_inv[3] = -q[3];
        qmul(q_inv, qd, q_rel);
        qnormalize(q_rel);
        yaw_vec_world[0] = 0.0f;
        yaw_vec_world[1] = 0.0f;
        yaw_vec_world[2] = yaw_rate_sp;
        q_rotate_vec(q_rel, yaw_vec_world, yaw_vec_body);
        for (uint32_t i = 0U; i < 3U; i++) {
            rate_sp[i] = clampf_att(rate_sp[i] + yaw_vec_body[i], -config->lim_rate[i], config->lim_rate[i]);
        }
    }
}

void FC_Attitude_ConfigDefault(fc_attitude_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->gain_att[0] = 4.0f;
    config->gain_att[1] = 4.0f;
    config->gain_att[2] = 2.8f / 0.4f;
    config->yaw_w = 0.4f;
    config->lim_rate[0] = 220.0f * 3.141592653589793f / 180.0f;
    config->lim_rate[1] = 220.0f * 3.141592653589793f / 180.0f;
    config->lim_rate[2] = 200.0f * 3.141592653589793f / 180.0f;
    config->initialized = 1U;
}

void FC_Attitude_Reset(fc_attitude_config_t *config)
{
    if (config == NULL) {
        return;
    }
    config->initialized = 1U;
}

bool FC_Attitude_Update(fc_attitude_config_t *config, const fc_estimator_state_t *state,
                        const float attitude_sp[4], float yaw_rate_sp, float rate_sp[3])
{
    float q[4];

    if ((config == NULL) || (state == NULL) || (attitude_sp == NULL) || (rate_sp == NULL)) {
        return false;
    }
    if (config->initialized == 0U) {
        return false;
    }
    if ((state->flags & FC_ESTIMATOR_FLAG_INITIALIZED) == 0U) {
        return false;
    }
    if (!isfinite(state->q_w) || !isfinite(state->q_x) || !isfinite(state->q_y) ||
        !isfinite(state->q_z) || !isfinite(attitude_sp[0]) || !isfinite(attitude_sp[1]) ||
        !isfinite(attitude_sp[2]) || !isfinite(attitude_sp[3]) || !isfinite(yaw_rate_sp)) {
        return false;
    }

    q[0] = state->q_w;
    q[1] = state->q_x;
    q[2] = state->q_y;
    q[3] = state->q_z;
    qnormalize(q);
    qcanonical(q);

    attitude_error_to_rate_sp(config, q, attitude_sp, yaw_rate_sp, rate_sp);

    return isfinite(rate_sp[0]) && isfinite(rate_sp[1]) && isfinite(rate_sp[2]);
}