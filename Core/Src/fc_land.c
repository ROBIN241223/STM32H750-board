#include "fc_land.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.141592653589793f
#endif

void FC_Land_ConfigDefault(fc_land_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->z_vel_max = 0.25f;        /* LNDMC_Z_VEL_MAX */
    config->xy_vel_max = 1.5f;        /* LNDMC_XY_VEL_MAX */
    config->rot_max_rad_s = 20.0f * M_PI / 180.0f;  /* LNDMC_ROT_MAX (deg/s) */
    config->alt_gnd_m = 1.0f;
    config->thr_min = 0.12f;          /* MPC_THR_MIN */
    config->thr_hover = 0.5f;         /* MPC_THR_HOVER */
    config->gc_frac_hi = 0.6f;
    config->ml_frac_lo = 0.1f;
    config->freefall_accel = 2.0f;
    config->t_ground_s = 0.333f;
    config->t_maybe_s = 0.333f;
    config->t_landed_s = 0.333f;
    config->t_freefall_s = 0.3f;
    config->initialized = 1U;
}

void FC_Land_Init(fc_land_config_t *config, fc_land_state_t *state)
{
    if ((config == NULL) || (state == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->freefall_pending = 0U;
    state->ground_contact = 0U;
    state->maybe_landed = 0U;
    state->landed = 0U;
    state->t_freefall = 0.0f;
    state->t_ground = 0.0f;
    state->t_maybe = 0.0f;
    state->t_landed = 0.0f;
    state->initialized = 1U;
}

static void update_pending(uint32_t condition, uint32_t *state_flag, float *timer,
                           float target_time, float dt)
{
    if (*state_flag) {
        if (!condition) {
            *state_flag = 0U;
            *timer = 0.0f;
        }
        return;
    }
    if (condition) {
        *timer += dt;
        if (*timer >= target_time) {
            *state_flag = 1U;
            *timer = 0.0f;
        }
    } else {
        *timer = 0.0f;
    }
}

bool FC_Land_Update(fc_land_config_t *config, fc_land_state_t *state,
                    float thrust_norm, float vz, float vxy, float rot_xy_rad_s,
                    float accel_norm, float dist_bottom_m, float dt,
                    fc_land_out_t *out)
{
    if ((config == NULL) || (state == NULL) || (out == NULL)) {
        return false;
    }
    if ((config->initialized == 0U) || (state->initialized == 0U)) {
        return false;
    }
    if (!(dt > 0.0f) || !isfinite(dt)) {
        dt = 0.001f;
    }

    /* Freefall: low specific-force norm, PX4 _get_freefall_state(). */
    bool freefall_now = (accel_norm < config->freefall_accel);
    update_pending(freefall_now, &state->freefall_pending, &state->t_freefall,
                   config->t_freefall_s, dt);

    /* Close to ground. */
    bool close_to_ground = (dist_bottom_m < config->alt_gnd_m);

    /* Vertical / horizontal / rotational movement (PX4 thresholds). */
    bool vertical_movement = (fabsf(vz) > config->z_vel_max);
    bool horizontal_movement = (vxy > config->xy_vel_max);
    bool rotational_movement = (fabsf(rot_xy_rad_s) > config->rot_max_rad_s);

    /* Thrust below (hover-min)*fraction above min -> low thrust. */
    float low_thrust = config->thr_min +
        (config->thr_hover - config->thr_min) * config->gc_frac_hi;
    bool has_low_throttle = (thrust_norm <= low_thrust);

    /* ground_contact: close to ground, low thrust, no movement. */
    bool ground_contact_cond = close_to_ground && has_low_throttle &&
        !vertical_movement && !horizontal_movement;
    update_pending(ground_contact_cond, &state->ground_contact, &state->t_ground,
                   config->t_ground_s, dt);

    bool freefall_state = (state->freefall_pending != 0U);
    bool ground_contact_state = (state->ground_contact != 0U);

    /* maybe_landed: minimum thrust, no rotation, no freefall, requires ground_contact. */
    float minimum_thrust = config->thr_min +
        (config->thr_hover - config->thr_min) * config->ml_frac_lo;
    bool has_minimum_thrust = (thrust_norm <= minimum_thrust);
    bool maybe_cond = has_minimum_thrust && ground_contact_state &&
        !freefall_state && !rotational_movement;
    update_pending(maybe_cond, &state->maybe_landed, &state->t_maybe,
                   config->t_maybe_s, dt);

    /* landed: maybe_landed sustained for its own hysteresis time. */
    bool maybe_landed_state = (state->maybe_landed != 0U);
    update_pending(maybe_landed_state, &state->landed, &state->t_landed,
                   config->t_landed_s, dt);

    out->ground_contact = state->ground_contact;
    out->maybe_landed = state->maybe_landed;
    out->landed = state->landed;
    out->freefall = freefall_state;
    return true;
}