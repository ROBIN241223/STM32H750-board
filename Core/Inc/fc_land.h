#ifndef FC_LAND_H
#define FC_LAND_H

#include <stdbool.h>
#include <stdint.h>

/* Phase E: land detector compressed from PX4 MulticopterLandDetector.
 *
 * Four boolean outputs with hysteresis that follow PX4: freefall, then
 * ground_contact -> maybe_landed -> landed.  Feed flags as position mode
 * applies (ground_contact) and to gate the controller integrators (landed).
 */

typedef struct {
    float z_vel_max;        /* LNDMC_Z_VEL_MAX: vertical velocity threshold (m/s) */
    float xy_vel_max;       /* LNDMC_XY_VEL_MAX: horizontal velocity threshold (m/s) */
    float rot_max_rad_s;    /* LNDMC_ROT_MAX: roll/pitch angular rate threshold (rad/s) */
    float alt_gnd_m;        /* close-to-ground altitude (m) */
    float thr_min;          /* MPC_THR_MIN: minimum collective (0..1) */
    float thr_hover;        /* MPC_THR_HOVER: hover collective (0..1) */
    float gc_frac_hi;       /* ground_contact low-thrust fraction of (hover-min) */
    float ml_frac_lo;       /* maybe_landed minimum-thrust fraction of (hover-min) */
    float freefall_accel;   /* specific-force norm below which freefall is assumed (m/s^2) */
    float t_ground_s;       /* ground_contact hysteresis time (s) */
    float t_maybe_s;        /* maybe_landed hysteresis time (s) */
    float t_landed_s;       /* landed hysteresis time (s) */
    float t_freefall_s;     /* freefall hysteresis time (s) */
    uint32_t initialized;
} fc_land_config_t;

typedef struct {
    uint32_t freefall_pending;
    uint32_t ground_contact;
    uint32_t maybe_landed;
    uint32_t landed;
    float t_freefall;
    float t_ground;
    float t_maybe;
    float t_landed;
    uint32_t initialized;
} fc_land_state_t;

typedef struct {
    uint32_t ground_contact;
    uint32_t maybe_landed;
    uint32_t landed;
    uint32_t freefall;
} fc_land_out_t;

void FC_Land_ConfigDefault(fc_land_config_t *config);
void FC_Land_Init(fc_land_config_t *config, fc_land_state_t *state);
bool FC_Land_Update(fc_land_config_t *config, fc_land_state_t *state,
                    float thrust_norm, float vz, float vxy, float rot_xy_rad_s,
                    float accel_norm, float dist_bottom_m, float dt,
                    fc_land_out_t *out);

#endif