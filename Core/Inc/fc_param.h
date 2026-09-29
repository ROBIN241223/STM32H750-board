#ifndef FC_PARAM_H
#define FC_PARAM_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"
#include "fc_attitude.h"
#include "fc_rate.h"
#include "fc_position.h"
#include "fc_land.h"
#include "fc_guard.h"
#include "fc_mixer.h"
#include "fc_gps.h"
#include "fc_compass.h"

/* Minimal PX4-style parameter server.
 *
 * Mirrors the PX4 param module semantics: every tunable is a named parameter
 * with a type and a default value; the controller modules get their gains from
 * the param table at init (FC_Param_Apply) instead of hard-coded structs.
 * This first revision keeps parameter storage in RAM and seeds the defaults
 * from the same numbers the modules used before, so behaviour is unchanged;
 * an override hook (FC_Param_SetFloat) is what the host/sim or ROS2 layer uses
 * to change a gain at runtime.
 *
 * Parameter names reuse the PX4 conventions:
 *   - MC_*      multicopter attitude / rate control
 *   - MPC_*     multicopter position control
 *   - FD_*      FailureDetector (guard)
 *   - LNDMC_*   multicopter land detector
 *   - FC_*      board-profile additions specific to this airframe
 */

typedef enum {
    FC_PARAM_TYPE_INT32 = 0,
    FC_PARAM_TYPE_FLOAT = 1
} fc_param_type_t;

typedef union {
    int32_t i32;
    float f32;
} fc_param_val_t;

typedef struct {
    const char *name;
    fc_param_type_t type;
    fc_param_val_t default_value;
} fc_param_def_t;

void FC_Param_Init(void);
void FC_Param_ResetAll(void);
uint32_t FC_Param_Count(void);
const char *FC_Param_Name(uint32_t index);
bool FC_Param_Index(const char *name, uint32_t *index);
bool FC_Param_Get(const char *name, fc_param_val_t *value);
bool FC_Param_Set(const char *name, fc_param_val_t value);
bool FC_Param_GetFloat(const char *name, float *value);
bool FC_Param_SetFloat(const char *name, float value);
bool FC_Param_GetInt(const char *name, int32_t *value);
bool FC_Param_SetInt(const char *name, int32_t value);

/* Seed the controller config structs from the current param values.
 * Must run after *_ConfigDefault so every gain is reachable via its PX4 name. */
void FC_Param_Apply(fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg,
                    fc_position_config_t *pos_cfg, fc_land_config_t *land_cfg,
                    fc_guard_config_t *guard_cfg, fc_mixer_config_t *mix_cfg,
                    fc_gps_config_t *gps_cfg, fc_compass_config_t *cmp_cfg);

#endif