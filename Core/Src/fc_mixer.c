#include "fc_mixer.h"
#include <math.h>
#include <string.h>

static float clampf_mx(float value, float minimum, float maximum)
{
    return fminf(fmaxf(value, minimum), maximum);
}

void FC_Mixer_ConfigDefault(fc_mixer_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    /* PX4 airframe 4001_quad_x (Generic Quadcopter, Quadrotor x) */
    config->ca_rotor_count = 4U;
    config->ca_rotor_px[0] = 1.0f;
    config->ca_rotor_py[0] = 1.0f;
    config->ca_rotor_px[1] = -1.0f;
    config->ca_rotor_py[1] = -1.0f;
    config->ca_rotor_px[2] = 1.0f;
    config->ca_rotor_py[2] = -1.0f;
    config->ca_rotor_px[3] = -1.0f;
    config->ca_rotor_py[3] = 1.0f;
    config->ca_rotor_ct[0] = 6.5f;
    config->ca_rotor_ct[1] = 6.5f;
    config->ca_rotor_ct[2] = 6.5f;
    config->ca_rotor_ct[3] = 6.5f;
    /* CA_ROTOR_KM: +0.05 = counter-clockwise (0,1), -0.05 = clockwise (2,3) */
    config->ca_rotor_km[0] = 0.05f;
    config->ca_rotor_km[1] = 0.05f;
    config->ca_rotor_km[2] = -0.05f;
    config->ca_rotor_km[3] = -0.05f;
    config->max_slew_norm = 0.08f;
    config->initialized = 1U;
}

/* In-place Gauss-Jordan inverse of an n x n matrix (n <= 4). Returns 0 on
 * success, -1 if singular. */
static int invertn(const float a[4][4], int n, float out[4][4])
{
    float m[4][8];
    float pivot;
    float t;
    int i;
    int j;
    int col;

    memset(out, 0, sizeof(float) * 4 * 4);
    for (i = 0; i < n; i++) {
        for (j = 0; j < n; j++) {
            m[i][j] = a[i][j];
        }
        for (j = 0; j < n; j++) {
            m[i][n + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (col = 0; col < n; col++) {
        int best = col;
        for (i = col + 1; i < n; i++) {
            if (fabsf(m[i][col]) > fabsf(m[best][col])) {
                best = i;
            }
        }
        pivot = m[best][col];
        if (fabsf(pivot) < 1e-12f) {
            return -1;
        }
        if (best != col) {
            for (j = 0; j < 2 * n; j++) {
                t = m[col][j];
                m[col][j] = m[best][j];
                m[best][j] = t;
            }
        }
        for (i = 0; i < n; i++) {
            if (i == col) {
                continue;
            }
            t = m[i][col] / pivot;
            if (fabsf(t) < 1e-12f) {
                continue;
            }
            for (j = 0; j < 2 * n; j++) {
                m[i][j] -= t * m[col][j];
            }
        }
    }

    for (i = 0; i < n; i++) {
        pivot = m[i][i];
        if (fabsf(pivot) < 1e-12f) {
            return -1;
        }
        for (j = 0; j < n; j++) {
            out[i][j] = m[i][n + j] / pivot;
        }
    }
    return 0;
}

/* Build the PX4 effectiveness matrix (rows R,P,Y,T; columns per rotor). */
static bool build_effectiveness(const fc_mixer_config_t *config, float e[4][4], int *n)
{
    uint32_t i;

    *n = (int)config->ca_rotor_count;
    if (*n > (int)FC_MIXER_MAX_ROTORS) {
        *n = (int)FC_MIXER_MAX_ROTORS;
    }
    if (*n < 1) {
        return false;
    }
    memset(e, 0, sizeof(float) * 4 * 4);
    for (i = 0; i < (uint32_t)*n; i++) {
        float ct = config->ca_rotor_ct[i];
        float px = config->ca_rotor_px[i];
        float py = config->ca_rotor_py[i];
        float km = config->ca_rotor_km[i];
        if (!isfinite(ct) || !isfinite(px) || !isfinite(py) || !isfinite(km) ||
            fabsf(ct) < 1e-6f) {
            return false;
        }
        /* axis = (0,0,-1): moment = ct*(r x axis) - ct*km*axis */
        e[0][i] = ct * (-py);
        e[1][i] = ct * px;
        e[2][i] = ct * km;
        e[3][i] = -ct;
    }
    return true;
}

/* PX4 normalize_rpy on the pseudo-inverse columns. */
static void normalize_mix(float m[4][4], int n)
{
    static const float EPS = 1e-3f;
    float sum_sq;
    float scale;
    float yaw_max;
    float thr_sum;
    int nz;
    int i;

    /* roll: same authority scale that roll and pitch share */
    sum_sq = 0.0f;
    nz = 0;
    for (i = 0; i < n; i++) {
        sum_sq += m[i][0] * m[i][0];
        if (fabsf(m[i][0]) > EPS) {
            nz++;
        }
    }
    scale = (nz > 0) ? sqrtf(sum_sq / (nz / 2.0f)) : 1.0f;

    /* pitch */
    sum_sq = 0.0f;
    nz = 0;
    for (i = 0; i < n; i++) {
        sum_sq += m[i][1] * m[i][1];
        if (fabsf(m[i][1]) > EPS) {
            nz++;
        }
    }
    if (nz > 0) {
        scale = fmaxf(scale, sqrtf(sum_sq / (nz / 2.0f)));
    }
    if (fabsf(scale) > 1e-6f) {
        for (i = 0; i < n; i++) {
            m[i][0] /= scale;
            m[i][1] /= scale;
        }
    }

    /* yaw: max column */
    yaw_max = 0.0f;
    for (i = 0; i < n; i++) {
        if (fabsf(m[i][2]) > yaw_max) {
            yaw_max = fabsf(m[i][2]);
        }
    }
    if (yaw_max > EPS) {
        for (i = 0; i < n; i++) {
            m[i][2] /= yaw_max;
        }
    }

    /* thrust: mean |column|, then flip for our +magnitude thrust convention */
    thr_sum = 0.0f;
    for (i = 0; i < n; i++) {
        thr_sum += fabsf(m[i][3]);
    }
    scale = (n > 0) ? (thr_sum / n) : 1.0f;
    if (fabsf(scale) > 1e-6f) {
        for (i = 0; i < n; i++) {
            m[i][3] = -(m[i][3] / scale);
        }
    }
}

bool FC_Mixer_Compute(fc_mixer_config_t *config, const fc_torque_cmd_t *torque,
                      float thrust_norm, bool armed, fc_motor_output_t *output)
{
    float e[4][4];
    float m[4][4];
    float delta[FC_MIXER_MAX_ROTORS];
    float thr;
    float alpha;
    int n;
    uint32_t i;

    if ((config == NULL) || (torque == NULL) || (output == NULL)) {
        return false;
    }
    if (config->initialized == 0U) {
        return false;
    }
    if (!isfinite(thrust_norm) || !isfinite(torque->roll_torque) ||
        !isfinite(torque->pitch_torque) || !isfinite(torque->yaw_torque)) {
        return false;
    }
    memset(output, 0, sizeof(*output));
    output->armed = armed;
    if (!armed) {
        output->valid = true;
        return true;
    }
    if (!build_effectiveness(config, e, &n)) {
        return false;
    }
    if (invertn(e, n, m) != 0) {
        return false;
    }
    normalize_mix(m, n);

    thr = clampf_mx(thrust_norm, 0.0f, 1.0f);
    for (i = 0U; i < (uint32_t)n; i++) {
        delta[i] = m[i][0] * torque->roll_torque + m[i][1] * torque->pitch_torque +
                   m[i][2] * torque->yaw_torque;
    }

    /* airmode-disabled desaturation: scale the deltas so every motor stays
     * within [0, 1]; thrust is never reduced to make room. */
    alpha = 1.0f;
    for (i = 0U; i < (uint32_t)n; i++) {
        float cap;
        if (delta[i] > 1e-9f) {
            cap = (1.0f - thr) / delta[i];
            if (cap < alpha) {
                alpha = cap;
            }
        } else if (delta[i] < -1e-9f) {
            cap = thr / (-delta[i]);
            if (cap < alpha) {
                alpha = cap;
            }
        }
    }
    if (!isfinite(alpha)) {
        alpha = 1.0f;
    }
    alpha = clampf_mx(alpha, 0.0f, 1.0f);

    for (i = 0U; i < (uint32_t)n; i++) {
        float value = thr + alpha * delta[i];
        value = clampf_mx(value, 0.0f, 1.0f);
        if (config->max_slew_norm > 0.0f) {
            value = clampf_mx(value,
                              config->previous_normalized[i] - config->max_slew_norm,
                              config->previous_normalized[i] + config->max_slew_norm);
        }
        config->previous_normalized[i] = value;
        output->normalized[i] = value;
        output->pwm[i] = (uint8_t)lroundf(value * 255.0f);
    }
    output->valid = true;
    return true;
}