#ifndef FC_GUARD_H
#define FC_GUARD_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_attitude.h"
#include "fc_rate.h"

/* Phase E + A.1/A.3 (25/09/2026): attitude-instability watchdog moved from the
 * Python SIL into C.  Detection now mirrors PX4 FailureDetector:
 *  - roll and pitch are checked SEPARATELY (like FD_FAIL_R / FD_FAIL_P,
 *    each 0 = disabled), for STAB and RECOVER.
 *  - an entrance requires the limit to be exceeded for a sustained trigger
 *    time (like FD_FAIL_R_TTRI / FD_FAIL_P_TTRI), so transient spikes do not
 *    trip the guard.  EXIT thresholds still use hysteresis (STAB holds
 *    stab_hold_s of stable flight before resuming).
 *  - the body-rate criterion is kept as an extra "wobble" trip (not in PX4).
 *
 * Modes:
 *  - NORMAL:   fly the mission.
 *  - STAB:     roll/pitch (or body rate) exceeded -> command level + hold
 *              position, boost rate damping until stable for stab_hold_s.
 *  - RECOVER:  roll/pitch exceeded recovery limit near the ground -> maximum
 *              rate authority to right the vehicle; stuck for rec_timeout_s
 *              -> teleport requested (I/O done by host).
 *
 * The guard owns the gain overrides: FC_Guard_Update writes the per-mode
 * gains/filters directly into the attitude and rate configs and restores the
 * base values on exit, so the caller just marshals the mode + hold thrust.
 */

typedef struct {
    /* --- STAB (rung lac / chao dao) ------------------------------------ */
    float stab_roll_enter_rad;  /* |roll| that trips stabilization (rad)    */
    float stab_pitch_enter_rad; /* |pitch| that trips stabilization (rad)   */
    float stab_rate_enter_rad_s;/* body-rate spike that trips it (rad/s)    */
    float stab_enter_t_s;       /* sustained time to trip STAB (FD_*_TTRI)  */
    float stab_roll_exit_rad;   /* |roll| below which it may resume (rad)   */
    float stab_pitch_exit_rad;  /* |pitch| below which it may resume (rad)  */
    float stab_rate_exit_rad_s; /* rate below which it may resume (rad/s)   */
    float stab_hold_s;          /* stable time required before resume (s)   */
    float stab_gain;            /* rate P multiplier while stabilizing      */
    float stab_damp;            /* rate D added while stabilizing           */
    float stab_rate_lim;        /* attitude lim_rate while stabilizing      */
    float stab_rate_int_lim;    /* rate lim_rate_int while stabilizing      */
    float stab_thrust_hold;     /* throttle (0..1) held while stabilizing   */
    /* --- RECOVER (lat / hoi phuc) --------------------------------------- */
    float rec_roll_enter_rad;   /* |roll| that trips recovery (rad)         */
    float rec_pitch_enter_rad;  /* |pitch| that trips recovery (rad)        */
    float rec_air_m;            /* only recover below this altitude (m)     */
    float rec_enter_t_s;        /* sustained time to trip RECOVER (FD_*_TTRI) */
    float rec_roll_exit_rad;    /* |roll| below which recovery ends (rad)   */
    float rec_pitch_exit_rad;   /* |pitch| below which recovery ends (rad)  */
    float rec_gain;             /* rate P multiplier while recovering       */
    float rec_rate_lim;         /* attitude lim_rate while recovering       */
    float rec_rate_int_lim;     /* rate lim_rate_int while recovering       */
    float rec_thrust_hold;      /* throttle (0..1) held while recovering    */
    float rec_timeout_s;        /* stuck this long -> teleport (s)          */
    /* --- state ----------------------------------------------------------- */
    uint32_t initialized;
} fc_guard_config_t;

typedef struct {
    uint32_t mode;              /* 0 NORMAL, 1 STAB, 2 RECOVER */
    float stable_since;         /* accumulated stable time in STAB (s) */
    float rec_since;            /* time since recovery started (s) */
    float stab_enter_since;     /* accumulated time beyond STAB limits (s) */
    float rec_enter_since;      /* accumulated time beyond RECOVER limits (s) */
    /* base config snapshot restored when leaving override modes */
    float base_lim_rate[3];
    float base_gain_rate_p[3];
    float base_gain_rate_d[3];
    float base_lim_rate_int[3];
    uint32_t overridden;
    uint32_t initialized;
} fc_guard_state_t;

typedef struct {
    uint32_t mode;              /* 0 NORMAL, 1 STAB, 2 RECOVER */
    uint32_t mode_changed;      /* 1 if mode changed this update */
    uint32_t teleport_now;      /* 1 -> host should teleport + reset guard */
    float thrust_hold;          /* throttle to hold while not NORMAL (0..1) */
    uint32_t valid;
} fc_guard_out_t;

#define FC_GUARD_MODE_NORMAL 0U
#define FC_GUARD_MODE_STAB   1U
#define FC_GUARD_MODE_RECOVER 2U

void FC_Guard_ConfigDefault(fc_guard_config_t *config);
void FC_Guard_Init(fc_guard_config_t *config, fc_guard_state_t *state,
                   fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg);
bool FC_Guard_Update(fc_guard_config_t *config, fc_guard_state_t *state,
                     fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg,
                     float roll_rad, float pitch_rad, float body_rate_rad_s,
                     float alt_m, float dt, fc_guard_out_t *out);
void FC_Guard_Reset(fc_guard_config_t *config, fc_guard_state_t *state,
                    fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg);

#endif