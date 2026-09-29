#include "fc_guard.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.141592653589793f
#endif

static void apply_overrides(const fc_guard_config_t *config,
                            fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg,
                            uint32_t mode)
{
    if (mode == FC_GUARD_MODE_STAB) {
        for (uint32_t i = 0U; i < 3U; i++) {
            att_cfg->lim_rate[i] = config->stab_rate_lim;
            rate_cfg->gain_rate_p[i] = config->stab_gain * rate_cfg->gain_rate_p[i];
            rate_cfg->gain_rate_d[i] = config->stab_damp;
            rate_cfg->lim_rate_int[i] = config->stab_rate_int_lim;
        }
    } else {
        for (uint32_t i = 0U; i < 3U; i++) {
            att_cfg->lim_rate[i] = config->rec_rate_lim;
            rate_cfg->gain_rate_p[i] = config->rec_gain * rate_cfg->gain_rate_p[i];
            rate_cfg->lim_rate_int[i] = config->rec_rate_int_lim;
        }
    }
}

static void snapshot_config(fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg,
                            fc_guard_state_t *state)
{
    memcpy(state->base_lim_rate, att_cfg->lim_rate, sizeof(state->base_lim_rate));
    memcpy(state->base_gain_rate_p, rate_cfg->gain_rate_p, sizeof(state->base_gain_rate_p));
    memcpy(state->base_gain_rate_d, rate_cfg->gain_rate_d, sizeof(state->base_gain_rate_d));
    memcpy(state->base_lim_rate_int, rate_cfg->lim_rate_int, sizeof(state->base_lim_rate_int));
}

static void restore_config(fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg,
                           fc_guard_state_t *state)
{
    memcpy(att_cfg->lim_rate, state->base_lim_rate, sizeof(state->base_lim_rate));
    memcpy(rate_cfg->gain_rate_p, state->base_gain_rate_p, sizeof(state->base_gain_rate_p));
    memcpy(rate_cfg->gain_rate_d, state->base_gain_rate_d, sizeof(state->base_gain_rate_d));
    memcpy(rate_cfg->lim_rate_int, state->base_lim_rate_int, sizeof(state->base_lim_rate_int));
}

static bool recover_timed_out(const fc_guard_config_t *config,
                              const fc_guard_state_t *state)
{
    return state->rec_since >= config->rec_timeout_s;
}

void FC_Guard_ConfigDefault(fc_guard_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    /* STAB thresholds (mirror PX4-ish + Gazebo defaults used in px4_x500.py) */
    config->stab_roll_enter_rad = 25.0f * M_PI / 180.0f;
    config->stab_pitch_enter_rad = 25.0f * M_PI / 180.0f;
    config->stab_rate_enter_rad_s = 2.0f;
    config->stab_enter_t_s = 0.3f;                /* FD_FAIL_R/P_TTRI */
    config->stab_roll_exit_rad = 10.0f * M_PI / 180.0f;
    config->stab_pitch_exit_rad = 10.0f * M_PI / 180.0f;
    config->stab_rate_exit_rad_s = 2.0f;
    config->stab_hold_s = 1.5f;
    config->stab_gain = 1.6f;
    config->stab_damp = 0.01f;
    config->stab_rate_lim = 3.0f;
    config->stab_rate_int_lim = 0.16f;
    config->stab_thrust_hold = 0.55f;
    /* RECOVER thresholds */
    config->rec_roll_enter_rad = 100.0f * M_PI / 180.0f;
    config->rec_pitch_enter_rad = 100.0f * M_PI / 180.0f;
    config->rec_air_m = 0.4f;
    config->rec_enter_t_s = 0.3f;                 /* FD_FAIL_R/P_TTRI */
    config->rec_roll_exit_rad = 40.0f * M_PI / 180.0f;
    config->rec_pitch_exit_rad = 40.0f * M_PI / 180.0f;
    config->rec_gain = 3.0f;
    config->rec_rate_lim = 6.28f;
    config->rec_rate_int_lim = 0.3f;
    config->rec_thrust_hold = 0.7f;
    config->rec_timeout_s = 8.0f;
    config->initialized = 1U;
}

void FC_Guard_Init(fc_guard_config_t *config, fc_guard_state_t *state,
                   fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg)
{
    if ((config == NULL) || (state == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->mode = FC_GUARD_MODE_NORMAL;
    state->stable_since = 0.0f;
    state->rec_since = 0.0f;
    state->stab_enter_since = 0.0f;
    state->rec_enter_since = 0.0f;
    state->overridden = 0U;
    if ((att_cfg != NULL) && (rate_cfg != NULL) &&
        (att_cfg->initialized != 0U) && (rate_cfg->initialized != 0U)) {
        snapshot_config(att_cfg, rate_cfg, state);
    }
    state->initialized = 1U;
}

void FC_Guard_Reset(fc_guard_config_t *config, fc_guard_state_t *state,
                    fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg)
{
    if (state->overridden != 0U) {
        restore_config(att_cfg, rate_cfg, state);
    }
    FC_Guard_Init(config, state, att_cfg, rate_cfg);
}

bool FC_Guard_Update(fc_guard_config_t *config, fc_guard_state_t *state,
                     fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg,
                     float roll_rad, float pitch_rad, float body_rate_rad_s,
                     float alt_m, float dt, fc_guard_out_t *out)
{
    uint32_t next;
    uint32_t changed = 0U;
    uint32_t teleport = 0U;
    float roll_abs;
    float pitch_abs;
    bool near_ground;
    bool recover_cond;
    bool stab_cond;
    bool stable_cond;
    bool recovered_cond;

    if ((config == NULL) || (state == NULL) || (out == NULL)) {
        return false;
    }
    if ((config->initialized == 0U) || (state->initialized == 0U)) {
        return false;
    }
    if ((att_cfg == NULL) || (rate_cfg == NULL)) {
        return false;
    }
    if (!(dt > 0.0f) || !isfinite(dt)) {
        dt = 0.001f;
    }
    if (!isfinite(roll_rad)) {
        roll_rad = 0.0f;
    }
    if (!isfinite(pitch_rad)) {
        pitch_rad = 0.0f;
    }
    if (!isfinite(body_rate_rad_s)) {
        body_rate_rad_s = 0.0f;
    }

    roll_abs = fabsf(roll_rad);
    pitch_abs = fabsf(pitch_rad);
    near_ground = (alt_m <= config->rec_air_m);
    /* FD_FAIL_R / FD_FAIL_P style: roll and pitch checked separately (+ rate). */
    recover_cond = near_ground &&
                   ((roll_abs > config->rec_roll_enter_rad) ||
                    (pitch_abs > config->rec_pitch_enter_rad));
    stab_cond = (roll_abs > config->stab_roll_enter_rad) ||
                (pitch_abs > config->stab_pitch_enter_rad) ||
                (body_rate_rad_s > config->stab_rate_enter_rad_s);
    stable_cond = (roll_abs < config->stab_roll_exit_rad) &&
                  (pitch_abs < config->stab_pitch_exit_rad) &&
                  (body_rate_rad_s < config->stab_rate_exit_rad_s);
    recovered_cond = (roll_abs < config->rec_roll_exit_rad) &&
                     (pitch_abs < config->rec_pitch_exit_rad);

    next = state->mode;

    switch (state->mode) {
    case FC_GUARD_MODE_NORMAL:
        /* Trip recovery first if the airframe rolled too far near the ground. */
        if (recover_cond) {
            state->rec_enter_since += dt;
            state->stab_enter_since = 0.0f;
            if (state->rec_enter_since >= config->rec_enter_t_s) {
                next = FC_GUARD_MODE_RECOVER;
                state->rec_since = 0.0f;
                state->rec_enter_since = 0.0f;
            }
        } else if (stab_cond) {
            state->stab_enter_since += dt;
            state->rec_enter_since = 0.0f;
            if (state->stab_enter_since >= config->stab_enter_t_s) {
                next = FC_GUARD_MODE_STAB;
                state->stable_since = 0.0f;
                state->stab_enter_since = 0.0f;
            }
        } else {
            state->stab_enter_since = 0.0f;
            state->rec_enter_since = 0.0f;
        }
        break;

    case FC_GUARD_MODE_STAB:
        if (recover_cond) {
            /* escalate to a full rollover near the ground -> recover */
            state->rec_enter_since += dt;
            state->stab_enter_since = 0.0f;
            if (state->rec_enter_since >= config->rec_enter_t_s) {
                next = FC_GUARD_MODE_RECOVER;
                state->rec_since = 0.0f;
                state->rec_enter_since = 0.0f;
            }
        } else if (stable_cond) {
            state->stable_since += dt;
            if (state->stable_since >= config->stab_hold_s) {
                next = FC_GUARD_MODE_NORMAL;
                state->stable_since = 0.0f;
            }
        } else {
            state->stable_since = 0.0f;
            state->rec_enter_since = 0.0f;
        }
        break;

    case FC_GUARD_MODE_RECOVER:
        state->rec_since += dt;
        if (recovered_cond) {
            /* recovered: back to normal flight */
            next = FC_GUARD_MODE_NORMAL;
            state->rec_since = 0.0f;
            state->stable_since = 0.0f;
            state->rec_enter_since = 0.0f;
        } else if (recover_timed_out(config, state)) {
            /* stuck: cannot right itself, request teleport back to takeoff */
            teleport = 1U;
            next = FC_GUARD_MODE_NORMAL;
            state->rec_since = 0.0f;
            state->stable_since = 0.0f;
            state->rec_enter_since = 0.0f;
        }
        break;

    default:
        next = FC_GUARD_MODE_NORMAL;
        break;
    }

    changed = (next != state->mode) ? 1U : 0U;

    /* apply/restore gain overrides on boundaries */
    if ((state->mode != next) && (state->overridden != 0U)) {
        restore_config(att_cfg, rate_cfg, state);
        state->overridden = 0U;
    }
    if ((next != FC_GUARD_MODE_NORMAL) && (state->overridden == 0U)) {
        snapshot_config(att_cfg, rate_cfg, state);
        apply_overrides(config, att_cfg, rate_cfg, next);
        state->overridden = 1U;
    } else if (next != FC_GUARD_MODE_NORMAL) {
        /* re-apply cleanly on STAB<->RECOVER jump so gains stay consistent */
        if ((state->mode != next) || (next == FC_GUARD_MODE_STAB)) {
            restore_config(att_cfg, rate_cfg, state);
            apply_overrides(config, att_cfg, rate_cfg, next);
        }
    }

    state->mode = next;

    out->mode = next;
    out->mode_changed = changed;
    out->teleport_now = teleport;
    out->valid = 1U;

    switch (next) {
    case FC_GUARD_MODE_RECOVER:
        out->thrust_hold = config->rec_thrust_hold;
        break;
    case FC_GUARD_MODE_STAB:
        out->thrust_hold = config->stab_thrust_hold;
        break;
    default:
        out->thrust_hold = 0.0f;
        break;
    }
    return true;
}