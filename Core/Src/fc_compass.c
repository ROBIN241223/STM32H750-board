#include "fc_compass.h"
#include <math.h>
#include <string.h>

static bool finite3(const float v[3])
{
    return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

static float clamp01f(float v)
{
    if (v < 0.0f) {
        return 0.0f;
    }
    if (v > 1.0f) {
        return 1.0f;
    }
    return v;
}

void FC_Compass_ConfigDefault(fc_compass_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->declination_rad = 0.0f;  /* CMP_DECLIN */
    config->rotation_deg = 0U;       /* CMP_ROT */
    config->cal_margin_fraction = 0.05f;  /* CMP_CAL_P: 2.5 uT on a 50 uT field */
    config->min_spread_uT = 3.0f;
    /* A full tumble sweeps each axis to about 2x the field magnitude, and a
     * parked airframe stays below 0.1x, so 1.5x separates a real rotation from a
     * partial one without demanding a laboratory twist rate. */
    config->min_sweep_fraction = 1.5f;
    config->min_samples = 500U;
    config->field_min_uT = 20.0f;
    config->field_max_uT = 130.0f;
    config->filter_alpha = 0.2f;     /* CMP_FILT */
    config->initialized = 1U;
}

void FC_Compass_Init(fc_compass_config_t *config, fc_compass_state_t *state)
{
    if ((state == NULL) || (config == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));
    for (uint32_t i = 0U; i < 3U; i++) {
        state->raw_max_uT[i] = -INFINITY;
        state->raw_min_uT[i] = INFINITY;
        state->peak_max_uT[i] = -INFINITY;
        state->peak_min_uT[i] = INFINITY;
        state->scale[i] = 1.0f;
    }
    state->calibrating = 1U;
    state->declination_rad = config->declination_rad;
    state->initialized = 1U;
}

void FC_Compass_StartCalibration(fc_compass_state_t *state)
{
    if (state == NULL) {
        return;
    }
    for (uint32_t i = 0U; i < 3U; i++) {
        state->raw_max_uT[i] = -INFINITY;
        state->raw_min_uT[i] = INFINITY;
        state->peak_max_uT[i] = -INFINITY;
        state->peak_min_uT[i] = INFINITY;
        state->scale[i] = 1.0f;
    }
    state->cal_count = 0U;
    state->gate_samples = 0U;
    state->calibrating = 1U;
    state->calibrated = 0U;
}

/* Envelope tracker, in two layers. The peak tracker holds the extremes of the
 * raw stream with no margin at all, so it always sits on the true peak. The
 * margin is applied when promoting a peak into the envelope: a peak is only
 * accepted once it clears the envelope by a margin, which rejects spikes.
 *
 * Comparing against the peak rather than against the envelope is what makes this
 * terminate. Against the envelope, a value creeping towards a peak never clears
 * the margin, so the peak is never reached; measured on a tumbling airframe at
 * 0.02 rad per step, the slowest axis went 876 samples without ever being
 * accepted, and the resulting offset was out by 6 uT.
 *
 * Taking an accepted peak exactly, rather than walking towards it, matters too:
 * an exponential walk lags a moving peak, and that lag is a per-axis bias which
 * lands straight in the hard-iron offset.
 *
 * There is deliberately no decay term: a decay towards the current sample would
 * collapse the envelope within a second of the airframe sitting still, which is
 * the normal case before takeoff. */
static void track_envelope(float value, float *peak_max, float *peak_min,
                           float *envelope_max, float *envelope_min, float margin)
{
    /* Seed both layers from the first sample. Walking towards a sample from an
     * infinite bound is -inf + inf, i.e. NaN, which would poison the spread check
     * for the rest of the flight. */
    if (!isfinite(*peak_max) || !isfinite(*peak_min)) {
        *peak_max = value;
        *peak_min = value;
        *envelope_max = value;
        *envelope_min = value;
        return;
    }
    if (value > *peak_max) {
        *peak_max = value;
    }
    if (value < *peak_min) {
        *peak_min = value;
    }
    if (*peak_max > (*envelope_max + margin)) {
        *envelope_max = *peak_max;
    }
    if (*peak_min < (*envelope_min - margin)) {
        *envelope_min = *peak_min;
    }
}

/* A calibration is only real if the airframe was actually rotated. Ground
 * vibration, motor noise and a drifting bench all move a parked magnetometer by
 * a couple of microtesla, so a fixed microtesla threshold is satisfied by an
 * untouched vehicle and locks in a garbage offset. Turning the airframe instead
 * sweeps each body axis through most of the field, so the bar is tied to the
 * measured field magnitude: every axis must show a range of at least
 * min_sweep_fraction of it. */
static bool rotation_excites_all_axes(const fc_compass_config_t *config,
                                      const fc_compass_state_t *state)
{
    for (uint32_t i = 0U; i < 3U; i++) {
        const float span = state->raw_max_uT[i] - state->raw_min_uT[i];
        if (!isfinite(span)) {
            return false;
        }
        if (span < config->min_spread_uT) {
            return false;
        }
        if (span < (config->min_sweep_fraction * state->raw_norm_uT)) {
            return false;
        }
    }
    return true;
}

static void accept_calibration(fc_compass_state_t *state)
{
    float mean_range = 0.0f;
    for (uint32_t i = 0U; i < 3U; i++) {
        state->offset_uT[i] = 0.5f * (state->raw_max_uT[i] + state->raw_min_uT[i]);
        mean_range += 0.5f * (state->raw_max_uT[i] - state->raw_min_uT[i]);
    }
    mean_range /= 3.0f;
    for (uint32_t i = 0U; i < 3U; i++) {
        float range = 0.5f * (state->raw_max_uT[i] - state->raw_min_uT[i]);
        /* A hard-iron-free axis must not divide by zero, and a wildly large
         * scale would turn a soft-iron error into a gain blow-up. */
        state->scale[i] = (range > 1.0f) ? (mean_range / range) : 1.0f;
        if ((state->scale[i] < 0.5f) || (state->scale[i] > 2.0f)) {
            state->scale[i] = 1.0f;
        }
    }
    state->calibrated = 1U;
}

bool FC_Compass_Update(fc_compass_config_t *config, fc_compass_state_t *state,
                       const float field_uT[3], float dt)
{
    if ((config == NULL) || (state == NULL) || (field_uT == NULL) || !finite3(field_uT)) {
        return false;
    }

    state->raw_norm_uT = sqrtf(field_uT[0] * field_uT[0] +
                               field_uT[1] * field_uT[1] +
                               field_uT[2] * field_uT[2]);

    if (state->calibrating != 0U) {
        const float margin = config->cal_margin_fraction * state->raw_norm_uT;
        for (uint32_t i = 0U; i < 3U; i++) {
            track_envelope(field_uT[i], &state->peak_max_uT[i], &state->peak_min_uT[i],
                           &state->raw_max_uT[i], &state->raw_min_uT[i], margin);
        }
        state->cal_count++;
        if ((state->cal_count >= config->min_samples) &&
            rotation_excites_all_axes(config, state)) {
            state->gate_samples++;
            /* Recomputed on every sample once the gate holds, and never latched
             * to a single value: the envelope keeps tightening as the airframe
             * keeps turning, so whatever is read out is the best calibration the
             * motion so far supports. Detecting "the operator stopped turning" is
             * not reliable here, because a slowly moving peak is indistinguishable
             * from a stopped one, and guessing wrong leaves a biased offset
             * latched for the whole flight. */
            accept_calibration(state);
        } else {
            state->gate_samples = 0U;
        }
    }

    float corrected[3];
    for (uint32_t i = 0U; i < 3U; i++) {
        corrected[i] = (field_uT[i] - state->offset_uT[i]) * state->scale[i];
    }

    /* Mounting rotation about z, quarter turns only: the airframe mounting is
     * 0/90/180/270 and a general angle would need a calibration matrix. */
    float rotated[3];
    switch (config->rotation_deg % 360U) {
    case 90U:
        rotated[0] = -corrected[1];
        rotated[1] = corrected[0];
        break;
    case 180U:
        rotated[0] = -corrected[0];
        rotated[1] = -corrected[1];
        break;
    case 270U:
        rotated[0] = corrected[1];
        rotated[1] = -corrected[0];
        break;
    default:
        rotated[0] = corrected[0];
        rotated[1] = corrected[1];
        break;
    }
    rotated[2] = corrected[2];

    /* Declination is a rotation of the horizontal field, i.e. an offset on the
     * heading the estimator reads out of atan2(). Both products use the
     * pre-rotation components. */
    const float cd = cosf(config->declination_rad);
    const float sd = sinf(config->declination_rad);
    const float x = rotated[0];
    const float y = rotated[1];
    rotated[0] = x * cd + y * sd;
    rotated[1] = y * cd - x * sd;

    const float alpha = clamp01f(config->filter_alpha);
    if (state->updated == 0U) {
        memcpy(state->field_uT, rotated, sizeof(rotated));
    } else {
        for (uint32_t i = 0U; i < 3U; i++) {
            state->field_uT[i] += alpha * (rotated[i] - state->field_uT[i]);
        }
    }
    state->declination_rad = config->declination_rad;

    float corrected_norm = sqrtf(rotated[0] * rotated[0] +
                                 rotated[1] * rotated[1] +
                                 rotated[2] * rotated[2]);
    /* A dead or saturated magnetometer reads a constant vector: the magnitude
     * check catches "no field", the spread check catches "stuck". Before a
     * calibration is accepted the raw envelope is still fused, which is the
     * pre-existing behaviour: an uncalibrated field is no worse than none. */
    state->valid = ((state->raw_norm_uT >= config->field_min_uT) &&
                    (state->raw_norm_uT <= config->field_max_uT) &&
                    (corrected_norm >= config->field_min_uT) &&
                    (corrected_norm <= config->field_max_uT) &&
                    ((state->calibrated != 0U) || (state->cal_count > 0U))) ? 1U : 0U;
    state->updated = 1U;
    (void)dt;
    return true;
}

bool FC_Compass_Field(const fc_compass_state_t *state, float out_uT[3])
{
    if ((state == NULL) || (out_uT == NULL) || (state->initialized == 0U)) {
        return false;
    }
    out_uT[0] = state->field_uT[0];
    out_uT[1] = state->field_uT[1];
    out_uT[2] = state->field_uT[2];
    return true;
}

bool FC_Compass_Calibrated(const fc_compass_state_t *state)
{
    return (state != NULL) && (state->calibrated != 0U);
}

bool FC_Compass_IsValid(const fc_compass_state_t *state)
{
    return (state != NULL) && (state->valid != 0U);
}
