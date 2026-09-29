#include "fc_param.h"
#include "fc_attitude.h"
#include "fc_rate.h"
#include "fc_position.h"
#include "fc_land.h"
#include "fc_guard.h"
#include "fc_mixer.h"
#include <string.h>

#define DEG2RAD (3.14159265358979323846f / 180.0f)

#define P_INT32(name, default_value) { name, FC_PARAM_TYPE_INT32, { .i32 = (default_value) } }
#define P_FLOAT(name, default_value) { name, FC_PARAM_TYPE_FLOAT, { .f32 = (default_value) } }

/* Defaults mirror the *_ConfigDefault() values the modules shipped with, so a
 * default config still behaves exactly as validated (sim + acceptance). */
static const fc_param_def_t param_defs[] = {
    /* --- multicopter attitude control ----------------------------------- */
    P_FLOAT("MC_ROLL_P", 4.0f),
    P_FLOAT("MC_PITCH_P", 4.0f),
    P_FLOAT("MC_YAW_P", 2.8f / 0.4f),
    P_FLOAT("MC_YAW_WEIGHT", 0.4f),
    P_FLOAT("MC_ROLLRATE_MAX", 220.0f),   /* deg/s, lim_rate[0] */
    P_FLOAT("MC_PITCHRATE_MAX", 220.0f),  /* deg/s, lim_rate[1] */
    P_FLOAT("MC_YAWRATE_MAX", 200.0f),    /* deg/s, lim_rate[2] */
    /* --- multicopter rate control --------------------------------------- */
    P_FLOAT("MC_ROLLRATE_K", 1.0f),
    P_FLOAT("MC_ROLLRATE_P", 0.15f),
    P_FLOAT("MC_ROLLRATE_I", 0.2f),
    P_FLOAT("MC_ROLLRATE_D", 0.003f),
    P_FLOAT("MC_ROLLRATE_FF", 0.0f),
    P_FLOAT("MC_RR_INT_LIM", 0.3f),
    P_FLOAT("MC_PITCHRATE_K", 1.0f),
    P_FLOAT("MC_PITCHRATE_P", 0.15f),
    P_FLOAT("MC_PITCHRATE_I", 0.2f),
    P_FLOAT("MC_PITCHRATE_D", 0.003f),
    P_FLOAT("MC_PITCHRATE_FF", 0.0f),
    P_FLOAT("MC_PR_INT_LIM", 0.3f),
    P_FLOAT("MC_YAWRATE_K", 1.0f),
    P_FLOAT("MC_YAWRATE_P", 0.2f),
    P_FLOAT("MC_YAWRATE_I", 0.1f),
    P_FLOAT("MC_YAWRATE_D", 0.0f),
    P_FLOAT("MC_YAWRATE_FF", 0.0f),
    P_FLOAT("MC_YR_INT_LIM", 0.3f),
    P_FLOAT("MC_YAW_TQ_CUTOFF", 2.0f),
    /* --- multicopter position control ----------------------------------- */
    P_FLOAT("MPC_XY_P", 0.95f),
    P_FLOAT("MPC_Z_P", 1.0f),
    P_FLOAT("MPC_XY_VEL_P", 1.8f),
    P_FLOAT("MPC_XY_VEL_I", 0.4f),
    P_FLOAT("MPC_XY_VEL_D", 0.2f),
    P_FLOAT("MPC_Z_VEL_P", 4.0f),
    P_FLOAT("MPC_Z_VEL_I", 2.0f),
    P_FLOAT("MPC_Z_VEL_D", 0.0f),
    P_FLOAT("MPC_XY_CRUISE", 12.0f),      /* lim_vel_h */
    P_FLOAT("MPC_Z_VEL_MAX_UP", 3.0f),    /* lim_vel_up */
    P_FLOAT("MPC_Z_VEL_MAX_DN", 1.5f),    /* lim_vel_down */
    P_FLOAT("MPC_THR_MIN", 0.12f),
    P_FLOAT("MPC_THR_MAX", 1.0f),
    P_FLOAT("MPC_THR_HOVER", 0.5f),
    P_FLOAT("MPC_TILTMAX_AIR", 45.0f),    /* deg, lim_tilt */
    P_FLOAT("MPC_ACC_HOR_MAX", 3.0f),     /* m/s^2, horizontal accel limit */
    /* --- FailureDetector (guard), PX4 names + units ---------------------- */
    P_FLOAT("FD_FAIL_R", 25.0f),          /* deg */
    P_FLOAT("FD_FAIL_P", 25.0f),          /* deg */
    P_FLOAT("FD_FAIL_R_TTRI", 0.3f),      /* s */
    P_FLOAT("FD_FAIL_P_TTRI", 0.3f),      /* s */
    P_FLOAT("FC_STAB_GAIN", 1.6f),
    P_FLOAT("FC_STAB_RATELIM", 3.0f),     /* rad/s */
    P_FLOAT("FC_STAB_THR_HOLD", 0.55f),
    P_FLOAT("FC_STAB_HOLD", 1.5f),        /* s stays level before resume */
    P_FLOAT("FC_STAB_EXIT", 10.0f),       /* deg */
    P_FLOAT("FC_REC_RATELIM", 360.0f),    /* deg/s */
    P_FLOAT("FC_REC_THR_HOLD", 0.7f),
    P_FLOAT("FC_REC_EXIT", 40.0f),        /* deg */
    P_FLOAT("FC_REC_TMOUT", 8.0f),        /* s -> teleport */
    P_FLOAT("FC_REC_AIR", 0.4f),          /* m, recovery only near ground */
    /* --- multicopter land detector -------------------------------------- */
    P_FLOAT("LNDMC_Z_VEL_MAX", 0.25f),
    P_FLOAT("LNDMC_XY_VEL_MAX", 1.5f),
    P_FLOAT("LNDMC_ROT_MAX", 20.0f),      /* deg/s */
    P_FLOAT("LNDMC_ALT_MAX", 1.0f),
    P_FLOAT("LNDMC_FFALL_THR", 2.0f),     /* m/s^2 */
    /* --- GNSS receiver (fc_gps) ----------------------------------------- */
    P_INT32("GPS_FUSE", 0),               /* 0 = off (sim ground truth), 1 = fly on the fix */
    P_FLOAT("GPS_POS_GATE", 15.0f),       /* m, max jump accepted between two fixes */
    P_FLOAT("GPS_VMAX", 25.0f),           /* m/s */
    P_FLOAT("GPS_ALT_MAX", 500.0f),       /* m above home */
    P_FLOAT("GPS_HDOP_MAX", 5.0f),
    P_INT32("GPS_SAT_MIN", 6),
    P_INT32("GPS_FIX_MIN", 3),            /* 2 = 2D fix is enough, 3 = need 3D */
    /* --- magnetometer (fc_compass) --------------------------------------- */
    P_FLOAT("CMP_DECLIN", 0.0f),          /* deg, magnetic declination */
    P_INT32("CMP_ROT", 0),                /* deg, mounting rotation 0/90/180/270 */
    P_FLOAT("CMP_CAL_P", 0.05f),          /* envelope outlier margin, fraction of field */
    P_FLOAT("CMP_FILT", 0.2f),            /* LPF on the corrected field */
};

#define FC_PARAM_COUNT (uint32_t)(sizeof(param_defs) / sizeof(param_defs[0]))

static fc_param_val_t param_vals[FC_PARAM_COUNT];
static uint32_t param_initialized;

static uint32_t param_find(const char *name)
{
    if (name == NULL) {
        return FC_PARAM_COUNT;
    }
    for (uint32_t i = 0U; i < FC_PARAM_COUNT; i++) {
        if (strcmp(param_defs[i].name, name) == 0) {
            return i;
        }
    }
    return FC_PARAM_COUNT;
}

void FC_Param_Init(void)
{
    for (uint32_t i = 0U; i < FC_PARAM_COUNT; i++) {
        param_vals[i] = param_defs[i].default_value;
    }
    param_initialized = 1U;
}

void FC_Param_ResetAll(void)
{
    FC_Param_Init();
}

uint32_t FC_Param_Count(void)
{
    return FC_PARAM_COUNT;
}

const char *FC_Param_Name(uint32_t index)
{
    if (index >= FC_PARAM_COUNT) {
        return NULL;
    }
    return param_defs[index].name;
}

bool FC_Param_Index(const char *name, uint32_t *index)
{
    uint32_t idx = param_find(name);
    if (idx >= FC_PARAM_COUNT) {
        return false;
    }
    if (index != NULL) {
        *index = idx;
    }
    return true;
}

bool FC_Param_Get(const char *name, fc_param_val_t *value)
{
    uint32_t idx = param_find(name);
    if ((idx >= FC_PARAM_COUNT) || (value == NULL) || (param_initialized == 0U)) {
        return false;
    }
    *value = param_vals[idx];
    return true;
}

bool FC_Param_Set(const char *name, fc_param_val_t value)
{
    uint32_t idx = param_find(name);
    if ((idx >= FC_PARAM_COUNT) || (param_initialized == 0U)) {
        return false;
    }
    param_vals[idx] = value;
    return true;
}

bool FC_Param_GetFloat(const char *name, float *value)
{
    uint32_t idx = param_find(name);
    if ((idx >= FC_PARAM_COUNT) || (value == NULL) || (param_initialized == 0U) ||
        (param_defs[idx].type != FC_PARAM_TYPE_FLOAT)) {
        return false;
    }
    *value = param_vals[idx].f32;
    return true;
}

bool FC_Param_SetFloat(const char *name, float value)
{
    uint32_t idx = param_find(name);
    if ((idx >= FC_PARAM_COUNT) || (param_initialized == 0U) ||
        (param_defs[idx].type != FC_PARAM_TYPE_FLOAT)) {
        return false;
    }
    param_vals[idx].f32 = value;
    return true;
}

bool FC_Param_GetInt(const char *name, int32_t *value)
{
    uint32_t idx = param_find(name);
    if ((idx >= FC_PARAM_COUNT) || (value == NULL) || (param_initialized == 0U) ||
        (param_defs[idx].type != FC_PARAM_TYPE_INT32)) {
        return false;
    }
    *value = param_vals[idx].i32;
    return true;
}

bool FC_Param_SetInt(const char *name, int32_t value)
{
    uint32_t idx = param_find(name);
    if ((idx >= FC_PARAM_COUNT) || (param_initialized == 0U) ||
        (param_defs[idx].type != FC_PARAM_TYPE_INT32)) {
        return false;
    }
    param_vals[idx].i32 = value;
    return true;
}

static bool set_float_default(const char *name, float fallback, float *field)
{
    float value;
    if (FC_Param_GetFloat(name, &value)) {
        *field = value;
        return true;
    }
    *field = fallback;
    return false;
}

void FC_Param_Apply(fc_attitude_config_t *att_cfg, fc_rate_config_t *rate_cfg,
                    fc_position_config_t *pos_cfg, fc_land_config_t *land_cfg,
                    fc_guard_config_t *guard_cfg, fc_mixer_config_t *mix_cfg,
                    fc_gps_config_t *gps_cfg, fc_compass_config_t *cmp_cfg)
{
    float v;

    if (param_initialized == 0U) {
        return;
    }
    if (att_cfg != NULL) {
        if (FC_Param_GetFloat("MC_ROLL_P", &v)) { att_cfg->gain_att[0] = v; }
        if (FC_Param_GetFloat("MC_PITCH_P", &v)) { att_cfg->gain_att[1] = v; }
        if (FC_Param_GetFloat("MC_YAW_P", &v)) { att_cfg->gain_att[2] = v; }
        if (FC_Param_GetFloat("MC_YAW_WEIGHT", &v)) { att_cfg->yaw_w = v; }
        if (FC_Param_GetFloat("MC_ROLLRATE_MAX", &v)) { att_cfg->lim_rate[0] = v * DEG2RAD; }
        if (FC_Param_GetFloat("MC_PITCHRATE_MAX", &v)) { att_cfg->lim_rate[1] = v * DEG2RAD; }
        if (FC_Param_GetFloat("MC_YAWRATE_MAX", &v)) { att_cfg->lim_rate[2] = v * DEG2RAD; }
    }
    if (rate_cfg != NULL) {
        if (FC_Param_GetFloat("MC_ROLLRATE_K", &v)) { rate_cfg->gain_rate_k[0] = v; }
        if (FC_Param_GetFloat("MC_ROLLRATE_P", &v)) { rate_cfg->gain_rate_p[0] = v; }
        if (FC_Param_GetFloat("MC_ROLLRATE_I", &v)) { rate_cfg->gain_rate_i[0] = v; }
        if (FC_Param_GetFloat("MC_ROLLRATE_D", &v)) { rate_cfg->gain_rate_d[0] = v; }
        if (FC_Param_GetFloat("MC_ROLLRATE_FF", &v)) { rate_cfg->gain_rate_ff[0] = v; }
        if (FC_Param_GetFloat("MC_RR_INT_LIM", &v)) { rate_cfg->lim_rate_int[0] = v; }
        if (FC_Param_GetFloat("MC_PITCHRATE_K", &v)) { rate_cfg->gain_rate_k[1] = v; }
        if (FC_Param_GetFloat("MC_PITCHRATE_P", &v)) { rate_cfg->gain_rate_p[1] = v; }
        if (FC_Param_GetFloat("MC_PITCHRATE_I", &v)) { rate_cfg->gain_rate_i[1] = v; }
        if (FC_Param_GetFloat("MC_PITCHRATE_D", &v)) { rate_cfg->gain_rate_d[1] = v; }
        if (FC_Param_GetFloat("MC_PITCHRATE_FF", &v)) { rate_cfg->gain_rate_ff[1] = v; }
        if (FC_Param_GetFloat("MC_PR_INT_LIM", &v)) { rate_cfg->lim_rate_int[1] = v; }
        if (FC_Param_GetFloat("MC_YAWRATE_K", &v)) { rate_cfg->gain_rate_k[2] = v; }
        if (FC_Param_GetFloat("MC_YAWRATE_P", &v)) { rate_cfg->gain_rate_p[2] = v; }
        if (FC_Param_GetFloat("MC_YAWRATE_I", &v)) { rate_cfg->gain_rate_i[2] = v; }
        if (FC_Param_GetFloat("MC_YAWRATE_D", &v)) { rate_cfg->gain_rate_d[2] = v; }
        if (FC_Param_GetFloat("MC_YAWRATE_FF", &v)) { rate_cfg->gain_rate_ff[2] = v; }
        if (FC_Param_GetFloat("MC_YR_INT_LIM", &v)) { rate_cfg->lim_rate_int[2] = v; }
        if (FC_Param_GetFloat("MC_YAW_TQ_CUTOFF", &v)) { rate_cfg->yaw_tq_cutoff = v; }
    }
    if (pos_cfg != NULL) {
        if (FC_Param_GetFloat("MPC_XY_P", &v)) { pos_cfg->gain_pos[0] = v; pos_cfg->gain_pos[1] = v; }
        if (FC_Param_GetFloat("MPC_Z_P", &v)) { pos_cfg->gain_pos[2] = v; }
        if (FC_Param_GetFloat("MPC_XY_VEL_P", &v)) { pos_cfg->gain_vel_p[0] = v; pos_cfg->gain_vel_p[1] = v; }
        if (FC_Param_GetFloat("MPC_XY_VEL_I", &v)) { pos_cfg->gain_vel_i[0] = v; pos_cfg->gain_vel_i[1] = v; }
        if (FC_Param_GetFloat("MPC_XY_VEL_D", &v)) { pos_cfg->gain_vel_d[0] = v; pos_cfg->gain_vel_d[1] = v; }
        if (FC_Param_GetFloat("MPC_Z_VEL_P", &v)) { pos_cfg->gain_vel_p[2] = v; }
        if (FC_Param_GetFloat("MPC_Z_VEL_I", &v)) { pos_cfg->gain_vel_i[2] = v; }
        if (FC_Param_GetFloat("MPC_Z_VEL_D", &v)) { pos_cfg->gain_vel_d[2] = v; }
        if (FC_Param_GetFloat("MPC_XY_CRUISE", &v)) { pos_cfg->lim_vel_h = v; }
        if (FC_Param_GetFloat("MPC_Z_VEL_MAX_UP", &v)) { pos_cfg->lim_vel_up = v; }
        if (FC_Param_GetFloat("MPC_Z_VEL_MAX_DN", &v)) { pos_cfg->lim_vel_down = v; }
        if (FC_Param_GetFloat("MPC_THR_MIN", &v)) { pos_cfg->lim_thr_min = v; }
        if (FC_Param_GetFloat("MPC_THR_MAX", &v)) { pos_cfg->lim_thr_max = v; }
        if (FC_Param_GetFloat("MPC_THR_HOVER", &v)) { pos_cfg->hover_thrust = v; }
        if (FC_Param_GetFloat("MPC_TILTMAX_AIR", &v)) { pos_cfg->lim_tilt = v * DEG2RAD; }
        if (FC_Param_GetFloat("MPC_ACC_HOR_MAX", &v)) { pos_cfg->lim_acc_h = v; }
    }
    if (guard_cfg != NULL) {
        if (FC_Param_GetFloat("FD_FAIL_R", &v)) { guard_cfg->stab_roll_enter_rad = v * DEG2RAD; }
        if (FC_Param_GetFloat("FD_FAIL_P", &v)) { guard_cfg->stab_pitch_enter_rad = v * DEG2RAD; }
        if (FC_Param_GetFloat("FD_FAIL_R_TTRI", &v)) { guard_cfg->stab_enter_t_s = v; }
        if (FC_Param_GetFloat("FD_FAIL_P_TTRI", &v)) { guard_cfg->rec_enter_t_s = v; }
        if (FC_Param_GetFloat("FC_STAB_GAIN", &v)) { guard_cfg->stab_gain = v; }
        if (FC_Param_GetFloat("FC_STAB_RATELIM", &v)) { guard_cfg->stab_rate_lim = v; }
        if (FC_Param_GetFloat("FC_STAB_THR_HOLD", &v)) { guard_cfg->stab_thrust_hold = v; }
        if (FC_Param_GetFloat("FC_STAB_HOLD", &v)) { guard_cfg->stab_hold_s = v; }
        if (FC_Param_GetFloat("FC_STAB_EXIT", &v)) {
            guard_cfg->stab_roll_exit_rad = v * DEG2RAD;
            guard_cfg->stab_pitch_exit_rad = v * DEG2RAD;
        }
        if (FC_Param_GetFloat("FC_REC_RATELIM", &v)) { guard_cfg->rec_rate_lim = v * DEG2RAD; }
        if (FC_Param_GetFloat("FC_REC_THR_HOLD", &v)) { guard_cfg->rec_thrust_hold = v; }
        if (FC_Param_GetFloat("FC_REC_EXIT", &v)) {
            guard_cfg->rec_roll_exit_rad = v * DEG2RAD;
            guard_cfg->rec_pitch_exit_rad = v * DEG2RAD;
        }
        if (FC_Param_GetFloat("FC_REC_TMOUT", &v)) { guard_cfg->rec_timeout_s = v; }
        if (FC_Param_GetFloat("FC_REC_AIR", &v)) { guard_cfg->rec_air_m = v; }
    }
    if (land_cfg != NULL) {
        if (FC_Param_GetFloat("LNDMC_Z_VEL_MAX", &v)) { land_cfg->z_vel_max = v; }
        if (FC_Param_GetFloat("LNDMC_XY_VEL_MAX", &v)) { land_cfg->xy_vel_max = v; }
        if (FC_Param_GetFloat("LNDMC_ROT_MAX", &v)) { land_cfg->rot_max_rad_s = v * DEG2RAD; }
        if (FC_Param_GetFloat("LNDMC_ALT_MAX", &v)) { land_cfg->alt_gnd_m = v; }
        if (FC_Param_GetFloat("LNDMC_FFALL_THR", &v)) { land_cfg->freefall_accel = v; }
        if (FC_Param_GetFloat("MPC_THR_MIN", &v)) { land_cfg->thr_min = v; }
        if (FC_Param_GetFloat("MPC_THR_HOVER", &v)) { land_cfg->thr_hover = v; }
    }
    /* mixer geometry (CA_ROTOR_*) stays an airframe configuration inside
     * FC_Mixer_ConfigDefault, not user-tunable; mix_cfg kept for symmetry. */
    if (gps_cfg != NULL) {
        set_float_default("GPS_POS_GATE", 15.0f, &gps_cfg->pos_gate_m);
        set_float_default("GPS_VMAX", 25.0f, &gps_cfg->max_speed_m_s);
        set_float_default("GPS_ALT_MAX", 500.0f, &gps_cfg->max_alt_m);
        set_float_default("GPS_HDOP_MAX", 5.0f, &gps_cfg->max_hdop);
        {
            int32_t iv = 0;
            if (FC_Param_GetInt("GPS_SAT_MIN", &iv) && (iv > 0)) {
                gps_cfg->min_sats = (uint32_t)iv;
            }
            if (FC_Param_GetInt("GPS_FIX_MIN", &iv) && (iv >= 0) && (iv <= 3)) {
                gps_cfg->min_fix_type = (uint32_t)iv;
            }
        }
    }
    if (cmp_cfg != NULL) {
        float decl_deg = 0.0f;
        set_float_default("CMP_DECLIN", 0.0f, &decl_deg);
        cmp_cfg->declination_rad = decl_deg * DEG2RAD;
        {
            int32_t iv = 0;
            if (FC_Param_GetInt("CMP_ROT", &iv) && (iv >= 0) && (iv <= 360)) {
                cmp_cfg->rotation_deg = (uint32_t)(iv % 360U);
            }
        }
        set_float_default("CMP_CAL_P", 0.05f, &cmp_cfg->cal_margin_fraction);
        set_float_default("CMP_FILT", 0.2f, &cmp_cfg->filter_alpha);
        /* Both are fractions: a value outside 0..1 either rejects every envelope
         * update or inverts the LPF, so clamp rather than trust the parameter. */
        if ((cmp_cfg->cal_margin_fraction <= 0.0f) || (cmp_cfg->cal_margin_fraction > 1.0f)) {
            cmp_cfg->cal_margin_fraction = 0.05f;
        }
        if ((cmp_cfg->filter_alpha < 0.0f) || (cmp_cfg->filter_alpha > 1.0f)) {
            cmp_cfg->filter_alpha = 0.2f;
        }
    }
    (void)set_float_default;
    (void)mix_cfg;
}