#include "fc_estimator.h"
#include <math.h>
#include <string.h>

static bool finite3(const float values[3])
{
    return isfinite(values[0]) && isfinite(values[1]) && isfinite(values[2]);
}

static float clampf(float value, float minimum, float maximum)
{
    return fminf(fmaxf(value, minimum), maximum);
}

static void normalize_quaternion(fc_estimator_state_t *state)
{
    float norm = sqrtf(
        state->q_w * state->q_w + state->q_x * state->q_x +
        state->q_y * state->q_y + state->q_z * state->q_z);
    if (!isfinite(norm) || (norm < 1.0e-6f)) {
        state->q_w = 1.0f;
        state->q_x = 0.0f;
        state->q_y = 0.0f;
        state->q_z = 0.0f;
        return;
    }
    state->q_w /= norm;
    state->q_x /= norm;
    state->q_y /= norm;
    state->q_z /= norm;
}

static void update_angles(fc_estimator_state_t *state)
{
    state->roll_rad = atan2f(
        2.0f * (state->q_w * state->q_x + state->q_y * state->q_z),
        1.0f - 2.0f * (state->q_x * state->q_x + state->q_y * state->q_y));
    state->pitch_rad = asinf(clampf(
        2.0f * (state->q_w * state->q_y - state->q_z * state->q_x), -1.0f, 1.0f));
    state->yaw_rad = atan2f(
        2.0f * (state->q_w * state->q_z + state->q_x * state->q_y),
        1.0f - 2.0f * (state->q_y * state->q_y + state->q_z * state->q_z));
}

static void set_from_angles(fc_estimator_state_t *state, float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    state->q_w = cr * cp * cy + sr * sp * sy;
    state->q_x = sr * cp * cy - cr * sp * sy;
    state->q_y = cr * sp * cy + sr * cp * sy;
    state->q_z = cr * cp * sy - sr * sp * cy;
    normalize_quaternion(state);
}

/* Decide whether the vehicle is quiescent enough for the accelerometer to be
 * read as a tilt reference and for the gyro to be read as a bias.
 *
 * This is the PX4 "vehicle at rest" test. It is deliberately strict, because
 * the two corrections it unlocks are only physically meaningful when nothing
 * is moving. The speed term is the part that actually separates a real bank
 * from cornering load: a vehicle sitting still at 20 deg and a vehicle in a
 * steady 20 deg turn produce almost the same specific force, and only the
 * speed tells them apart. Without a position source the speed term is skipped,
 * which simply makes the test more conservative -- the corrections then stay
 * off unless the vehicle is also quiet in the gyro. */
static bool quiescent(
    const fc_estimator_config_t *config,
    const fc_estimator_state_t *state,
    const fc_imu_sample_t *sample,
    float accel_norm)
{
    float gyro_norm = sqrtf(
        sample->gyro_rad_s[0] * sample->gyro_rad_s[0] +
        sample->gyro_rad_s[1] * sample->gyro_rad_s[1] +
        sample->gyro_rad_s[2] * sample->gyro_rad_s[2]);

    if (gyro_norm > config->rest_gyro_rad_s) {
        return false;
    }
    if (fabsf(accel_norm - config->gravity_mps2) > (config->rest_accel_frac * config->gravity_mps2)) {
        return false;
    }
    if (state->pos_valid != 0U) {
        float speed = sqrtf(
            state->vel_ned[0] * state->vel_ned[0] +
            state->vel_ned[1] * state->vel_ned[1]);
        if (speed > config->rest_speed_m_s) {
            return false;
        }
    }
    return true;
}

/* Fold the at-rest gyro reading into the bias estimate. While the vehicle is
 * quiescent the gyro is reading its own bias and nothing else, so this is a
 * direct observation rather than a filter correction. PX4 does the same thing
 * in EKF2's ZeroGyroUpdate; the point is that the bias keeps tracking
 * temperature and ageing instead of being frozen at arm time. */
static void update_gyro_bias(
    fc_estimator_config_t *config,
    const fc_imu_sample_t *sample,
    float dt)
{
    float alpha;

    if (config->gyro_bias_tau_s <= 0.0f) {
        return;
    }
    alpha = 1.0f - expf(-dt / config->gyro_bias_tau_s);
    for (uint32_t i = 0U; i < 3U; i++) {
        config->gyro_bias_rad_s[i] += alpha * (sample->gyro_rad_s[i] - config->gyro_bias_rad_s[i]);
    }
}

void FC_Estimator_ConfigDefault(fc_estimator_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->gravity_mps2 = 9.80665f;
    config->accel_correction_gain = 2.0f;
    config->mag_correction_gain = 3.0f;
    config->pitch_sign = 1.0f;
    config->accel_gate_frac = 0.12f;
    config->calibration_samples = 200U;
    config->gyro_bias_tau_s = 0.5f;
    config->rest_gyro_rad_s = 0.05f;
    config->rest_accel_frac = 0.15f;
    config->rest_speed_m_s = 0.5f;
    config->rest_hold_samples = 25U;
    config->tilt_gate_rad = 0.4363f;   /* 25 deg */
}

void FC_Estimator_Init(fc_estimator_config_t *config, fc_estimator_state_t *state)
{
    if ((config == NULL) || (state == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->q_w = 1.0f;
    state->flags = 0U;
    config->calibration_count = 0U;
    config->previous_timestamp_us = 0U;
    config->initialized = false;
    memset(config->gyro_bias_rad_s, 0, sizeof(config->gyro_bias_rad_s));
    memset(config->gyro_sum_rad_s, 0, sizeof(config->gyro_sum_rad_s));
}

void FC_Estimator_FeedPosition(fc_estimator_state_t *state, const float pos_ned[3], const float vel_ned[3])
{
    if ((state == NULL) || (pos_ned == NULL) || (vel_ned == NULL)) {
        return;
    }
    state->pos_ned[0] = pos_ned[0];
    state->pos_ned[1] = pos_ned[1];
    state->pos_ned[2] = pos_ned[2];
    state->vel_ned[0] = vel_ned[0];
    state->vel_ned[1] = vel_ned[1];
    state->vel_ned[2] = vel_ned[2];
    state->pos_valid = 1U;
}

bool FC_Estimator_Update(fc_estimator_config_t *config, fc_estimator_state_t *state, const fc_imu_sample_t *sample)
{
    float dt = 0.002f;
    float accel_norm;
    float alpha;
    float accel_roll;
    float accel_pitch;
    float gx;
    float gy;
    float gz;

    if ((config == NULL) || (state == NULL) || (sample == NULL) ||
        !finite3(sample->accel_mps2) || !finite3(sample->gyro_rad_s) ||
        ((sample->mag_valid != 0U) && !finite3(sample->mag_ut))) {
        return false;
    }
    if ((sample->timestamp_us != 0U) && (config->previous_timestamp_us != 0U)) {
        uint64_t delta;
        if (sample->timestamp_us < config->previous_timestamp_us) {
            state->flags = FC_ESTIMATOR_FLAG_FAULT;
            return false;
        }
        delta = sample->timestamp_us - config->previous_timestamp_us;
        dt = (float)delta * 1.0e-6f;
    }
    config->previous_timestamp_us = sample->timestamp_us;
    if (!isfinite(dt) || (dt < 0.0002f) || (dt > 0.02f)) {
        state->flags = FC_ESTIMATOR_FLAG_FAULT;
        return false;
    }

    if (!config->initialized) {
        for (uint32_t i = 0U; i < 3U; i++) {
            config->gyro_sum_rad_s[i] += sample->gyro_rad_s[i];
        }
        config->calibration_count++;
        if (config->calibration_count < config->calibration_samples) {
            state->flags = 0U;
            return false;
        }
        for (uint32_t i = 0U; i < 3U; i++) {
            config->gyro_bias_rad_s[i] = config->gyro_sum_rad_s[i] /
                (float)config->calibration_count;
        }
        accel_roll = atan2f(sample->accel_mps2[1], -sample->accel_mps2[2]);
        accel_pitch = config->pitch_sign * atan2f(
            -sample->accel_mps2[0],
            sqrtf(sample->accel_mps2[1] * sample->accel_mps2[1] +
                  sample->accel_mps2[2] * sample->accel_mps2[2]));
        if ((sample->mag_valid != 0U) && finite3(sample->mag_ut)) {
            float phi = accel_roll;
            float theta = accel_pitch;
            float cp = cosf(theta);
            float sp = sinf(theta);
            float cr = cosf(phi);
            float sr = sinf(phi);
            float xh = sample->mag_ut[0] * cp + sample->mag_ut[1] * sr * sp +
                sample->mag_ut[2] * cr * sp;
            float yh = sample->mag_ut[1] * cr - sample->mag_ut[2] * sr;
            set_from_angles(state, accel_roll, accel_pitch, atan2f(-yh, xh));
        } else {
            set_from_angles(state, accel_roll, accel_pitch, 0.0f);
        }
        config->initialized = true;
    }
    state->flags |= FC_ESTIMATOR_FLAG_INITIALIZED;

    gx = sample->gyro_rad_s[0] - config->gyro_bias_rad_s[0];
    gy = sample->gyro_rad_s[1] - config->gyro_bias_rad_s[1];
    gz = sample->gyro_rad_s[2] - config->gyro_bias_rad_s[2];

    {
        float qw = state->q_w;
        float qx = state->q_x;
        float qy = state->q_y;
        float qz = state->q_z;
        state->q_w = qw + 0.5f * (-qx * gx - qy * gy - qz * gz) * dt;
        state->q_x = qx + 0.5f * ( qw * gx + qy * gz - qz * gy) * dt;
        state->q_y = qy + 0.5f * ( qw * gy - qx * gz + qz * gx) * dt;
        state->q_z = qz + 0.5f * ( qw * gz + qx * gy - qy * gx) * dt;
    }
    normalize_quaternion(state);
    update_angles(state);

    accel_norm = sqrtf(
        sample->accel_mps2[0] * sample->accel_mps2[0] +
        sample->accel_mps2[1] * sample->accel_mps2[1] +
        sample->accel_mps2[2] * sample->accel_mps2[2]);

    /* Quiescence has to be re-evaluated every sample and the counter has to be
     * cleared the instant the vehicle moves. The previous code latched the
     * accelerometer off and never restored it: the gate closed for 110 s
     * straight during a circle, which left the attitude frozen at whatever the
     * accelerometer had last claimed. */
    if (quiescent(config, state, sample, accel_norm)) {
        if (state->rest_count < 0xFFFFFFFFU) {
            state->rest_count++;
        }
    } else {
        state->rest_count = 0U;
    }
    if (state->rest_count >= config->rest_hold_samples) {
        state->at_rest = 1U;
        state->flags |= FC_ESTIMATOR_FLAG_AT_REST;
    } else {
        state->at_rest = 0U;
        state->flags &= ~FC_ESTIMATOR_FLAG_AT_REST;
    }

    if (fabsf(accel_norm - config->gravity_mps2) < (0.2f * config->gravity_mps2)) {
        state->flags |= FC_ESTIMATOR_FLAG_ACCEL_VALID;
    } else {
        state->flags &= ~FC_ESTIMATOR_FLAG_ACCEL_VALID;
    }

    if (state->at_rest != 0U) {
        float horizontal = sqrtf(
            sample->accel_mps2[0] * sample->accel_mps2[0] +
            sample->accel_mps2[1] * sample->accel_mps2[1]);
        float load_ratio = (horizontal / config->gravity_mps2) /
            fmaxf(config->accel_gate_frac, 0.05f);
        /* Soft de-weighting. It never reaches zero, so unlike the old hard gate
         * this always recovers as soon as the load goes away. */
        float trust = 1.0f / (1.0f + load_ratio * load_ratio);
        accel_roll = atan2f(sample->accel_mps2[1], -sample->accel_mps2[2]);
        accel_pitch = config->pitch_sign * atan2f(
            -sample->accel_mps2[0],
            sqrtf(sample->accel_mps2[1] * sample->accel_mps2[1] +
                  sample->accel_mps2[2] * sample->accel_mps2[2]));

        /* The gyro is a bias reading here, and that needs no position aid. */
        update_gyro_bias(config, sample, dt);

        /* Re-reading the tilt is a different matter. A vehicle parked at 20 deg
         * and a vehicle cornering at the same bank produce nearly the same
         * specific force, so without a position source there is no way to tell
         * them apart and the reading is simply not a tilt. PX4 draws the line in
         * the same place: the accelerometer sets the tilt once during startup
         * alignment and is not consulted for attitude again in flight. */
        if (state->pos_valid != 0U &&
            (fabsf(accel_roll - state->roll_rad) <= config->tilt_gate_rad) &&
            (fabsf(accel_pitch - state->pitch_rad) <= config->tilt_gate_rad)) {
            alpha = clampf(config->accel_correction_gain * dt, 0.0f, 1.0f) * trust;
            state->roll_rad += alpha * (accel_roll - state->roll_rad);
            state->pitch_rad += alpha * (accel_pitch - state->pitch_rad);
            set_from_angles(state, state->roll_rad, state->pitch_rad, state->yaw_rad);
        }
    }

    if ((sample->mag_valid != 0U) && finite3(sample->mag_ut) &&
        ((state->flags & FC_ESTIMATOR_FLAG_ACCEL_VALID) != 0U)) {
        float mx = sample->mag_ut[0];
        float my = sample->mag_ut[1];
        float mz = sample->mag_ut[2];
        float phi = state->roll_rad;
        float theta = state->pitch_rad;
        float cp = cosf(theta);
        float sp = sinf(theta);
        float cr = cosf(phi);
        float sr = sinf(phi);
        float xh = mx * cp + my * sr * sp + mz * cr * sp;
        float yh = my * cr - mz * sr;
        float mag_yaw = atan2f(-yh, xh);
        float yaw_err = mag_yaw - state->yaw_rad;
        float alpha_mag;
        yaw_err = atan2f(sinf(yaw_err), cosf(yaw_err));
        alpha_mag = clampf(config->mag_correction_gain * dt, 0.0f, 1.0f);
        /* No rate cap here. PX4 limits the magnetometer to 1 deg/s, but it
         * applies that to the yaw *offset* while its yaw angle is propagated by
         * the gyro at the true rate. This estimator integrates the yaw angle
         * itself, so the same cap would fight a real circle and leave the
         * heading pinned while the airframe rotated around it. */
        state->yaw_rad += alpha_mag * yaw_err;
        set_from_angles(state, state->roll_rad, state->pitch_rad, state->yaw_rad);
    }

    memcpy(state->gyro_rad_s, sample->gyro_rad_s, sizeof(state->gyro_rad_s));
    memcpy(state->accel_mps2, sample->accel_mps2, sizeof(state->accel_mps2));
    state->sample_age_us = 0U;
    return true;
}
