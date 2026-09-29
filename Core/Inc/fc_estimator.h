#ifndef FC_ESTIMATOR_H
#define FC_ESTIMATOR_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

typedef struct {
    float gravity_mps2;
    float accel_correction_gain;
    float mag_correction_gain;
    float pitch_sign;
    /* Horizontal specific force (as a fraction of gravity) above which the
     * accelerometer is treated as loaded rather than as a tilt reference. This
     * only scales the weight of the at-rest tilt correction -- it never latches
     * off, so the correction always recovers once the load clears. */
    float accel_gate_frac;
    uint32_t calibration_samples;
    uint32_t calibration_count;
    float gyro_bias_rad_s[3];
    float gyro_sum_rad_s[3];
    uint64_t previous_timestamp_us;
    bool initialized;
    /* Time constant of the low-pass that folds the at-rest gyro reading into
     * gyro_bias_rad_s. Mirrors PX4 EKF2_GYR_B_NOISE / ZeroGyroUpdate. */
    float gyro_bias_tau_s;
    /* |gyro| below which the vehicle counts as quiescent. */
    float rest_gyro_rad_s;
    /* ||a| - g| tolerance, as a fraction of gravity, for the at-rest test. */
    float rest_accel_frac;
    /* Horizontal speed below which the vehicle counts as quiescent. Only
     * consulted when a position source is available, because that is the only
     * thing that can tell a real bank from cornering load. */
    float rest_speed_m_s;
    /* Consecutive at-rest samples required before the vehicle is declared
     * quiescent, so a single quiet moment cannot flap the filter. */
    uint32_t rest_hold_samples;
    /* Innovation gate on the at-rest tilt correction, radians. A reading that
     * disagrees with the gyro by more than this is rejected outright. */
    float tilt_gate_rad;
} fc_estimator_config_t;

void FC_Estimator_ConfigDefault(fc_estimator_config_t *config);
void FC_Estimator_Init(fc_estimator_config_t *config, fc_estimator_state_t *state);
bool FC_Estimator_Update(fc_estimator_config_t *config, fc_estimator_state_t *state, const fc_imu_sample_t *sample);
void FC_Estimator_FeedPosition(fc_estimator_state_t *state, const float pos_ned[3], const float vel_ned[3]);

#endif
