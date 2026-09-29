#ifndef FC_COMPASS_H
#define FC_COMPASS_H

#include <stdbool.h>
#include <stdint.h>

/* Magnetometer front end, split out of fc_estimator so calibration and
 * status live in one place instead of being implicit in the yaw fusion.
 *
 * The module owns three stages and hands fc_estimator a field vector that is
 * already corrected, so the estimator keeps doing nothing but fusion:
 *
 *   raw -> hard iron -> soft iron -> mounting rotation -> declination -> LPF
 *
 * Hard and soft iron are estimated from the min/max envelope the field sweeps
 * as the airframe is turned (PX4's approach, without needing 3D ellipsoid
 * maths). Both are learned only while FC_Compass_StartCalibration has been
 * called, and a calibration is only accepted once every axis has swept a real
 * fraction of the field, so a vehicle that never moves cannot "calibrate" to
 * vibration noise.
 *
 * Units are microtesla, matching the magnetometer convention in fc_types.h
 * and the mag_north_uT/mag_down_uT SDF parameters in the sim model.
 */

typedef struct {
    float declination_rad;    /* CMP_DECLIN: magnetic declination, rad, signed */
    uint32_t rotation_deg;    /* CMP_ROT: mounting rotation 0/90/180/270 (CW) */
    float cal_margin_fraction; /* CMP_CAL_P: outlier margin, as a fraction of the field */
    float min_spread_uT;      /* absolute per-axis (max-min) floor, uT */
    float min_sweep_fraction; /* per-axis (max-min) as a fraction of the field, 0..1 */
    uint32_t min_samples;     /* samples required before a calibration is accepted */
    float field_min_uT;       /* healthy field magnitude range (Earth: 25-65 uT) */
    float field_max_uT;
    float filter_alpha;       /* CMP_FILT: LPF on the corrected field, 0..1 */
    uint32_t initialized;
} fc_compass_config_t;

typedef struct {
    float raw_max_uT[3];      /* running envelope used for calibration */
    float raw_min_uT[3];
    float peak_max_uT[3];     /* margin-free extremes, source of the envelope */
    float peak_min_uT[3];
    float offset_uT[3];       /* hard iron */
    float scale[3];           /* soft iron, 1.0 = no correction */
    float field_uT[3];        /* corrected, rotated, declination applied, filtered */
    float raw_norm_uT;        /* magnitude of the last raw sample, for status */
    float declination_rad;
    uint32_t cal_count;
    uint32_t gate_samples;    /* consecutive samples the sweep gate has held */
    uint32_t calibrating;
    uint32_t calibrated;
    uint32_t valid;           /* corrected field may be fused */
    uint32_t updated;
    uint32_t initialized;
} fc_compass_state_t;

void FC_Compass_ConfigDefault(fc_compass_config_t *config);
void FC_Compass_Init(fc_compass_config_t *config, fc_compass_state_t *state);

/* Restart the envelope calibration. Call it on the ground only: the offsets
 * are only meaningful while the airframe is being rotated. */
void FC_Compass_StartCalibration(fc_compass_state_t *state);

/* True once a calibration has been accepted (every axis spread enough). */
bool FC_Compass_Calibrated(const fc_compass_state_t *state);

/* Feed one raw body-frame sample. dt is used only for the output LPF. */
bool FC_Compass_Update(fc_compass_config_t *config, fc_compass_state_t *state,
                       const float field_uT[3], float dt);

/* Corrected field to hand to the attitude estimator. */
bool FC_Compass_Field(const fc_compass_state_t *state, float out_uT[3]);

/* True when the field is usable: in range, enough spread seen and not a
 * zero/stuck sensor. */
bool FC_Compass_IsValid(const fc_compass_state_t *state);

#endif
