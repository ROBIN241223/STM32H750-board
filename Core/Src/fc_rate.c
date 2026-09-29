#include "fc_rate.h"
#include <math.h>
#include <string.h>

#define TWO_PI_RATE 6.283185307179586f

static float clampf_r(float value, float minimum, float maximum)
{
    return fminf(fmaxf(value, minimum), maximum);
}

void FC_RateControl_ConfigDefault(fc_rate_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->gain_rate_k[0] = 1.0f;
    config->gain_rate_k[1] = 1.0f;
    config->gain_rate_k[2] = 1.0f;
    config->gain_rate_p[0] = 0.15f;
    config->gain_rate_p[1] = 0.15f;
    config->gain_rate_p[2] = 0.2f;
    config->gain_rate_i[0] = 0.2f;
    config->gain_rate_i[1] = 0.2f;
    config->gain_rate_i[2] = 0.1f;
    config->gain_rate_d[0] = 0.003f;
    config->gain_rate_d[1] = 0.003f;
    config->gain_rate_d[2] = 0.0f;
    config->gain_rate_ff[0] = 0.0f;
    config->gain_rate_ff[1] = 0.0f;
    config->gain_rate_ff[2] = 0.0f;
    config->lim_rate_int[0] = 0.3f;
    config->lim_rate_int[1] = 0.3f;
    config->lim_rate_int[2] = 0.3f;
    config->yaw_tq_cutoff = 2.0f;
    config->yaw_lpf_enabled = 1U;
    config->initialized = 1U;
}

void FC_RateControl_Reset(fc_rate_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config->rate_int, 0, sizeof(config->rate_int));
    memset(config->gyro_prev, 0, sizeof(config->gyro_prev));
    config->gyro_dt = 0.0f;
    config->initialized = 1U;
}

bool FC_RateControl_Update(fc_rate_config_t *config, const float rate_sp[3],
                           const float rate[3], float dt, bool landed,
                           float torque[3])
{
    float angular_accel[3];
    float rate_error[3];
    float dt_rate;

    if ((config == NULL) || (rate_sp == NULL) || (rate == NULL) || (torque == NULL)) {
        return false;
    }
    if (config->initialized == 0U) {
        return false;
    }
    if (!isfinite(rate_sp[0]) || !isfinite(rate_sp[1]) || !isfinite(rate_sp[2]) ||
        !isfinite(rate[0]) || !isfinite(rate[1]) || !isfinite(rate[2])) {
        return false;
    }

    dt_rate = dt;
    if (!isfinite(dt_rate) || (dt_rate < 0.000125f) || (dt_rate > 0.02f)) {
        dt_rate = 0.002f;
    }

    if ((config->gyro_dt > 0.0f) && isfinite(config->gyro_dt)) {
        for (uint32_t i = 0U; i < 3U; i++) {
            angular_accel[i] = (rate[i] - config->gyro_prev[i]) / config->gyro_dt;
            if (!isfinite(angular_accel[i])) {
                angular_accel[i] = 0.0f;
            }
        }
    } else {
        angular_accel[0] = 0.0f;
        angular_accel[1] = 0.0f;
        angular_accel[2] = 0.0f;
    }
    config->gyro_prev[0] = rate[0];
    config->gyro_prev[1] = rate[1];
    config->gyro_prev[2] = rate[2];
    config->gyro_dt = dt_rate;

    for (uint32_t i = 0U; i < 3U; i++) {
        rate_error[i] = rate_sp[i] - rate[i];
        if (!landed) {
            float i_factor = rate_error[i] / (400.0f * 3.141592653589793f / 180.0f);
            float rate_i;
            i_factor = fmaxf(0.0f, 1.0f - i_factor * i_factor);
            rate_i = config->rate_int[i] +
                i_factor * config->gain_rate_k[i] * config->gain_rate_i[i] * rate_error[i] * dt_rate;
            if (isfinite(rate_i)) {
                config->rate_int[i] = clampf_r(rate_i, -config->lim_rate_int[i], config->lim_rate_int[i]);
            }
        }
        torque[i] = config->gain_rate_k[i] * config->gain_rate_p[i] * rate_error[i] +
            config->rate_int[i] -
            config->gain_rate_k[i] * config->gain_rate_d[i] * angular_accel[i] +
            config->gain_rate_k[i] * config->gain_rate_ff[i] * rate_sp[i];
        if ((config->yaw_lpf_enabled != 0U) && (config->yaw_tq_cutoff > 0.0f)) {
            config->yaw_tq_lpf += (1.0f - expf(-TWO_PI_RATE * config->yaw_tq_cutoff * dt_rate)) *
                (torque[2] - config->yaw_tq_lpf);
            torque[2] = config->yaw_tq_lpf;
        }
    }

    return isfinite(torque[0]) && isfinite(torque[1]) && isfinite(torque[2]);
}