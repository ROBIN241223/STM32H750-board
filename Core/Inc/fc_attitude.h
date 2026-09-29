#ifndef FC_ATTITUDE_H
#define FC_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

typedef struct {
    float gain_att[3];
    float yaw_w;
    float lim_rate[3];
    uint32_t initialized;
} fc_attitude_config_t;

void FC_Attitude_ConfigDefault(fc_attitude_config_t *config);
void FC_Attitude_Reset(fc_attitude_config_t *config);
bool FC_Attitude_Update(fc_attitude_config_t *config, const fc_estimator_state_t *state,
                        const float attitude_sp[4], float yaw_rate_sp, float rate_sp[3]);

#endif