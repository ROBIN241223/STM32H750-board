#ifndef FC_RATE_H
#define FC_RATE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float gain_rate_k[3];
    float gain_rate_p[3];
    float gain_rate_i[3];
    float gain_rate_d[3];
    float gain_rate_ff[3];
    float lim_rate_int[3];
    float yaw_tq_cutoff;
    float rate_int[3];
    float gyro_prev[3];
    float gyro_dt;
    float yaw_tq_lpf;
    uint32_t yaw_lpf_enabled;
    uint32_t initialized;
} fc_rate_config_t;

void FC_RateControl_ConfigDefault(fc_rate_config_t *config);
void FC_RateControl_Reset(fc_rate_config_t *config);
bool FC_RateControl_Update(fc_rate_config_t *config, const float rate_sp[3],
                           const float rate[3], float dt, bool landed,
                           float torque[3]);

#endif