#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "fc_gps.h"
#include "fc_compass.h"
#include "gps_parser.h"
#include "gps_port.h"

#define NEAR(a, b, tol) (fabs((a) - (b)) <= (tol))
#define DEG (3.141592653589793 / 180.0)

/* Hanoi, the same origin the sim world uses. */
#define HOME_LAT 21.0285
#define HOME_LON 105.8542
#define HOME_ALT 16.0

/* Metres per degree at this latitude, for checking the local conversion. */
static double m_per_deg_lat(void)
{
    return 6371008.8 * DEG;
}

static void test_gps_config_defaults(void)
{
    fc_gps_config_t config;
    FC_GPS_ConfigDefault(&config);
    assert(config.initialized == 1U);
    assert(config.min_fix_type == 3U);
    assert(config.min_sats == 6U);
    assert(config.max_hdop == 5.0f);
    assert(config.pos_gate_m > 0.0f);
    assert(config.max_speed_m_s > 0.0f);
}

static void fill_good_sample(fc_gps_sample_t *s, double lat, double lon, float alt)
{
    memset(s, 0, sizeof(*s));
    s->lat_deg = lat;
    s->lon_deg = lon;
    s->alt_m = alt;
    s->num_sats = 12U;
    s->fix_type = 3U;
    s->hdop = 0.8f;
    s->vel_valid = 1U;
    s->timestamp_us = 100000U;
}

static void test_gps_needs_home(void)
{
    fc_gps_config_t config;
    fc_gps_state_t state;
    fc_gps_sample_t sample;

    FC_GPS_ConfigDefault(&config);
    FC_GPS_Init(&config, &state);
    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);

    /* A good fix with no origin must be refused, otherwise the local position
     * would be measured against nothing. */
    assert(!FC_GPS_Update(&config, &state, &sample));
    assert(!FC_GPS_IsValid(&state));

    assert(FC_GPS_SetHome(&config, &state, HOME_LAT, HOME_LON, HOME_ALT));
    assert(state.home_set == 1U);
    assert(FC_GPS_Update(&config, &state, &sample));
    assert(FC_GPS_IsValid(&state));
}

static void test_gps_lat_lon_to_ned(void)
{
    fc_gps_config_t config;
    fc_gps_state_t state;
    fc_gps_sample_t sample;
    float pos[3] = {0.0f, 0.0f, 0.0f};
    float vel[3] = {0.0f, 0.0f, 0.0f};

    FC_GPS_ConfigDefault(&config);
    FC_GPS_Init(&config, &state);
    assert(FC_GPS_SetHome(&config, &state, HOME_LAT, HOME_LON, HOME_ALT));

    /* 0.001 deg north and 0.001 deg east, 30 m up. */
    fill_good_sample(&sample, HOME_LAT + 0.001, HOME_LON + 0.001, HOME_ALT + 30.0f);
    sample.vel_ned[0] = 1.5f;
    sample.vel_ned[1] = -0.5f;
    sample.vel_ned[2] = 0.25f;
    assert(FC_GPS_Update(&config, &state, &sample));
    assert(FC_GPS_GetLocal(&state, pos, vel));

    assert(NEAR(pos[0], m_per_deg_lat() * 0.001, 0.05));
    assert(NEAR(pos[1], m_per_deg_lat() * 0.001 * cos(HOME_LAT * DEG), 0.05));
    assert(NEAR(pos[2], -30.0, 0.01));      /* NED down is negative up */
    assert(NEAR(vel[0], 1.5, 1e-6));
    assert(NEAR(vel[1], -0.5, 1e-6));
    assert(NEAR(vel[2], 0.25, 1e-6));
}

static void test_gps_rejects_bad_fixes(void)
{
    fc_gps_config_t config;
    fc_gps_state_t state;
    fc_gps_sample_t sample;
    float before[3] = {0.0f, 0.0f, 0.0f};
    float after[3] = {0.0f, 0.0f, 0.0f};

    FC_GPS_ConfigDefault(&config);
    FC_GPS_Init(&config, &state);
    assert(FC_GPS_SetHome(&config, &state, HOME_LAT, HOME_LON, HOME_ALT));
    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    assert(FC_GPS_Update(&config, &state, &sample));
    assert(FC_GPS_GetLocal(&state, before, NULL));

    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    sample.fix_type = 2U;                    /* only a 2D fix */
    assert(!FC_GPS_Update(&config, &state, &sample));

    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    sample.num_sats = 3U;                    /* too few satellites */
    assert(!FC_GPS_Update(&config, &state, &sample));

    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    sample.hdop = 9.0f;                      /* geometry too poor */
    assert(!FC_GPS_Update(&config, &state, &sample));

    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    sample.lat_deg = 999.0;                  /* impossible latitude */
    assert(!FC_GPS_Update(&config, &state, &sample));

    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    sample.lat_deg = NAN;
    assert(!FC_GPS_Update(&config, &state, &sample));

    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    sample.vel_ned[0] = 500.0f;              /* faster than the vehicle can fly */
    assert(!FC_GPS_Update(&config, &state, &sample));

    /* None of the above may have moved the position. */
    assert(FC_GPS_GetLocal(&state, after, NULL));
    assert(NEAR(after[0], before[0], 1e-6));
    assert(NEAR(after[1], before[1], 1e-6));
    assert(NEAR(after[2], before[2], 1e-6));
    assert(state.rejected >= 6U);
}

static void test_gps_home_needs_a_good_fix(void)
{
    fc_gps_config_t config;
    fc_gps_state_t state;
    fc_gps_sample_t sample;
    float pos[3] = {1.0f, 1.0f, 1.0f};
    float after[3] = {0.0f, 0.0f, 0.0f};

    FC_GPS_ConfigDefault(&config);
    FC_GPS_Init(&config, &state);

    /* Before there is a home point the caller has to decide on one itself, and
     * the only thing it has to go on is the sample itself. */
    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    assert(FC_GPS_SampleAcceptable(&config, &sample));

    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    sample.fix_type = 0U;                     /* receiver has not acquired */
    assert(!FC_GPS_SampleAcceptable(&config, &sample));
    assert(FC_GPS_SetHome(&config, &state, HOME_LAT, HOME_LON, HOME_ALT));
    assert(FC_GPS_GetLocal(&state, pos, NULL));
    assert(NEAR(pos[0], 0.0, 1e-6));

    /* A parked vehicle keeps re-freezing home to each new good fix, so a
     * receiver that drifts while it sits still must be able to move it. Once
     * it has, the new fix has to describe the new origin. */
    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    assert(FC_GPS_SampleAcceptable(&config, &sample));
    assert(FC_GPS_SetHome(&config, &state, HOME_LAT + 0.01, HOME_LON, HOME_ALT));
    fill_good_sample(&sample, HOME_LAT + 0.01, HOME_LON, HOME_ALT);
    assert(FC_GPS_Update(&config, &state, &sample));
    assert(FC_GPS_GetLocal(&state, after, NULL));
    assert(NEAR(after[0], 0.0, 0.05));
    assert(NEAR(after[1], 0.0, 0.05));

    /* ...and a bad sample must never be able to shift the origin. */
    fill_good_sample(&sample, HOME_LAT + 0.02, HOME_LON, HOME_ALT);
    sample.fix_type = 0U;
    assert(!FC_GPS_SampleAcceptable(&config, &sample));
    assert(!FC_GPS_Update(&config, &state, &sample));
    assert(state.home_lat_deg == HOME_LAT + 0.01);
}

static void test_gps_rejects_jump(void)
{
    fc_gps_config_t config;
    fc_gps_state_t state;
    fc_gps_sample_t sample;

    FC_GPS_ConfigDefault(&config);
    FC_GPS_Init(&config, &state);
    assert(FC_GPS_SetHome(&config, &state, HOME_LAT, HOME_LON, HOME_ALT));
    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    assert(FC_GPS_Update(&config, &state, &sample));

    /* A metre away is normal drift. */
    fill_good_sample(&sample, HOME_LAT + (1.0 / m_per_deg_lat()), HOME_LON, HOME_ALT);
    assert(FC_GPS_Update(&config, &state, &sample));

    /* Half a degree is a 55 km jump between two fixes: multipath, not flight. */
    fill_good_sample(&sample, HOME_LAT + 0.5, HOME_LON, HOME_ALT);
    assert(!FC_GPS_Update(&config, &state, &sample));
    assert(NEAR(state.pos_ned[0], 1.0, 0.1));
}

static void test_gps_velocity_held_when_missing(void)
{
    fc_gps_config_t config;
    fc_gps_state_t state;
    fc_gps_sample_t sample;
    float vel[3] = {0.0f, 0.0f, 0.0f};

    FC_GPS_ConfigDefault(&config);
    FC_GPS_Init(&config, &state);
    assert(FC_GPS_SetHome(&config, &state, HOME_LAT, HOME_LON, HOME_ALT));

    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    sample.vel_ned[0] = 2.0f;
    assert(FC_GPS_Update(&config, &state, &sample));

    /* GGA-only stream: no velocity. Differentiating a noisy position would
     * inject the GNSS noise into the velocity loop, so the last value stays. */
    fill_good_sample(&sample, HOME_LAT, HOME_LON, HOME_ALT);
    sample.vel_valid = 0U;
    sample.vel_ned[0] = 99.0f;
    assert(FC_GPS_Update(&config, &state, &sample));
    assert(FC_GPS_GetLocal(&state, NULL, vel));
    assert(NEAR(vel[0], 2.0, 1e-6));
}

static void test_gps_home_moving_resets(void)
{
    fc_gps_config_t config;
    fc_gps_state_t state;
    fc_gps_sample_t sample;
    float pos[3] = {9.0f, 9.0f, 9.0f};

    FC_GPS_ConfigDefault(&config);
    FC_GPS_Init(&config, &state);
    assert(FC_GPS_SetHome(&config, &state, HOME_LAT, HOME_LON, HOME_ALT));
    fill_good_sample(&sample, HOME_LAT + 0.01, HOME_LON, HOME_ALT);
    assert(FC_GPS_Update(&config, &state, &sample));
    assert(state.valid == 1U);

    /* Re-freezing home must clear validity, never leave a stale offset. */
    assert(FC_GPS_SetHome(&config, &state, HOME_LAT + 0.01, HOME_LON, HOME_ALT));
    assert(state.valid == 0U);
    assert(FC_GPS_GetLocal(&state, pos, NULL));
    assert(NEAR(pos[0], 0.0, 1e-6));
    assert(NEAR(pos[1], 0.0, 1e-6));
    assert(NEAR(pos[2], 0.0, 1e-6));
}

/* --------------------------------------------------------------------------
 * Compass
 * -------------------------------------------------------------------------- */

/* Rotate a fixed 50 uT world field into the body frame. Calibration needs all
 * three axes to be excited: a yaw-only sweep leaves the z envelope flat and can
 * never produce a 3-axis solution, so the tumble mixes all three angles. */
static void field_for_attitude(float roll, float pitch, float yaw,
                               const float offset[3], const float scale[3], float out[3])
{
    const double world[3] = {30.0, 40.0, 0.0};
    const double cr = cos(roll), sr = sin(roll);
    const double cp = cos(pitch), sp = sin(pitch);
    const double cy = cos(yaw), sy = sin(yaw);

    const double r1[3] = {world[0], cr * world[1] - sr * world[2], sr * world[1] + cr * world[2]};
    const double r2[3] = {cp * r1[0] + sp * r1[2], r1[1], -sp * r1[0] + cp * r1[2]};
    const double body[3] = {cy * r2[0] - sy * r2[1], sy * r2[0] + cy * r2[1], r2[2]};

    out[0] = (float)(body[0] - offset[0]) * scale[0];
    out[1] = (float)(body[1] - offset[1]) * scale[1];
    out[2] = (float)(body[2] - offset[2]) * scale[2];
}

static void test_compass_config_defaults(void)
{
    fc_compass_config_t config;
    FC_Compass_ConfigDefault(&config);
    assert(config.initialized == 1U);
    assert(config.declination_rad == 0.0f);
    assert(config.rotation_deg == 0U);
    assert(config.cal_margin_fraction > 0.0f);
    assert((config.field_min_uT > 0.0f) && (config.field_max_uT > config.field_min_uT));
}

static void test_compass_rejects_dead_sensor(void)
{
    fc_compass_config_t config;
    fc_compass_state_t state;
    float field[3] = {0.0f, 0.0f, 0.0f};

    FC_Compass_ConfigDefault(&config);
    config.min_samples = 4U;
    FC_Compass_Init(&config, &state);
    /* A flat-zero magnetometer is the classic unplugged-sensor signature. */
    for (int i = 0; i < 20; i++) {
        assert(FC_Compass_Update(&config, &state, field, 0.004f));
    }
    assert(!FC_Compass_IsValid(&state));
}

static void tumble(fc_compass_config_t *config, fc_compass_state_t *state,
                   int steps, const float offset[3], const float scale[3])
{
    for (int step = 0; step < steps; step++) {
        const float t = (float)step * 0.02f;
        float field[3];
        field_for_attitude(t, 0.7f * t, 1.3f * t, offset, scale, field);
        assert(FC_Compass_Update(config, state, field, 0.004f));
    }
}

static void hold_still(fc_compass_config_t *config, fc_compass_state_t *state,
                       int steps, const float offset[3], const float scale[3])
{
    for (int step = 0; step < steps; step++) {
        float field[3];
        field_for_attitude(61.0f, 42.7f, 79.3f, offset, scale, field);
        assert(FC_Compass_Update(config, state, field, 0.004f));
    }
}

static void test_compass_needs_rotation_to_calibrate(void)
{
    fc_compass_config_t config;
    fc_compass_state_t state;
    const float no_offset[3] = {0.0f, 0.0f, 0.0f};
    const float no_scale[3] = {1.0f, 1.0f, 1.0f};

    FC_Compass_ConfigDefault(&config);
    config.min_samples = 8U;
    FC_Compass_Init(&config, &state);

    /* Sitting still: a valid field, but no calibration can be accepted. */
    hold_still(&config, &state, 200, no_offset, no_scale);
    assert(FC_Compass_IsValid(&state));
    assert(!FC_Compass_Calibrated(&state));
    assert(state.calibrating == 1U);

    tumble(&config, &state, 3000, no_offset, no_scale);
    assert(FC_Compass_Calibrated(&state));
    assert(FC_Compass_IsValid(&state));
}

/* A half-hearted wiggle must not be mistaken for a calibration. This is the
 * failure that matters on the ground: motor vibration alone moves an axis by a
 * couple of microtesla, and accepting that produces a hard-iron offset of tens
 * of microtesla and a wrong heading. */
static void test_compass_rejects_partial_rotation(void)
{
    fc_compass_config_t config;
    fc_compass_state_t state;
    float field[3];
    const float no_offset[3] = {0.0f, 0.0f, 0.0f};
    const float no_scale[3] = {1.0f, 1.0f, 1.0f};

    FC_Compass_ConfigDefault(&config);
    config.min_samples = 8U;
    FC_Compass_Init(&config, &state);

    /* Yaw through 90 degrees with roll and pitch pinned: the horizontal axes
     * sweep, the vertical one never does. */
    for (int step = 0; step < 3000; step++) {
        const float t = (float)step * 0.0026f;
        field_for_attitude(0.05f, 0.05f, t, no_offset, no_scale, field);
        assert(FC_Compass_Update(&config, &state, field, 0.004f));
    }
    hold_still(&config, &state, 300, no_offset, no_scale);
    assert(!FC_Compass_Calibrated(&state));
    /* The field is still fused while uncalibrated: refusing to calibrate must
     * not mean refusing to fly. */
    assert(FC_Compass_IsValid(&state));

    /* Completing the tumble in all three axes is accepted. */
    tumble(&config, &state, 3000, no_offset, no_scale);
    assert(FC_Compass_Calibrated(&state));
}

/* The min/max method estimates hard iron from the envelope the field sweeps
 * during whatever motion the operator actually performed, so the reference here
 * is the envelope of this exact tumble. Asserting against the injected offset
 * instead would be asserting that the test's rotation path is symmetric, which
 * it is not: no single-axis or fixed-ratio tumble is. */
static void test_compass_removes_hard_and_soft_iron(void)
{
    fc_compass_config_t config;
    fc_compass_state_t state;
    float field[3];
    float corrected[3];
    const float offset[3] = {6.0f, -4.0f, 2.0f};      /* hard iron, uT */
    const float scale[3] = {1.15f, 0.85f, 1.0f};     /* soft iron, per axis */
    double raw_min[3] = {1e9, 1e9, 1e9};
    double raw_max[3] = {-1e9, -1e9, -1e9};
    double min_mag = 1e9;
    double max_mag = -1e9;

    /* Reference envelope: the same motion, measured directly. */
    for (int step = 0; step < 3000; step++) {
        const float t = (float)step * 0.02f;
        field_for_attitude(t, 0.7f * t, 1.3f * t, offset, scale, field);
        for (int i = 0; i < 3; i++) {
            if (field[i] < raw_min[i]) {
                raw_min[i] = field[i];
            }
            if (field[i] > raw_max[i]) {
                raw_max[i] = field[i];
            }
        }
    }

    FC_Compass_ConfigDefault(&config);
    config.min_samples = 8U;
    FC_Compass_Init(&config, &state);
    tumble(&config, &state, 3000, offset, scale);
    assert(FC_Compass_Calibrated(&state));

    double mean_range = 0.0;
    for (int i = 0; i < 3; i++) {
        assert(NEAR(state.offset_uT[i], 0.5 * (float)(raw_min[i] + raw_max[i]), 2.0));
        mean_range += 0.5 * (raw_max[i] - raw_min[i]);
    }
    mean_range /= 3.0;
    for (int i = 0; i < 3; i++) {
        const double expect = mean_range / (0.5 * (raw_max[i] - raw_min[i]));
        assert(fabs(state.scale[i] - expect) < 0.12);
    }

    /* The point of the whole exercise: the corrected field is rounder than the
     * raw one. The raw magnitudes spanned roughly 99..113 uT per axis because
     * of the injected soft iron; afterwards they sit on one sphere. */
    const int samples = 400;
    for (int step = 0; step < samples; step++) {
        const float t = (float)step * 0.05f;
        double mag;
        field_for_attitude(t, 0.7f * t, 1.3f * t, offset, scale, field);
        assert(FC_Compass_Update(&config, &state, field, 0.004f));
        assert(FC_Compass_Field(&state, corrected));
        mag = sqrt(corrected[0] * corrected[0] +
                   corrected[1] * corrected[1] +
                   corrected[2] * corrected[2]);
        if (mag < min_mag) {
            min_mag = mag;
        }
        if (mag > max_mag) {
            max_mag = mag;
        }
    }
    assert(min_mag > 30.0);
    assert(max_mag < 70.0);
}

static void test_compass_rotation_and_declination(void)
{
    fc_compass_config_t config;
    fc_compass_state_t state;
    float field[3];
    float corrected[3];
    const float no_offset[3] = {0.0f, 0.0f, 0.0f};
    const float no_scale[3] = {1.0f, 1.0f, 1.0f};

    field_for_attitude(0.0, 0.0, 0.0, no_offset, no_scale, field);
    const float raw_x = field[0];
    const float raw_y = field[1];

    /* 90 deg of mounting rotation: the field must come out rotated, and the
     * magnitude must be untouched. */
    FC_Compass_ConfigDefault(&config);
    config.rotation_deg = 90U;
    config.min_samples = 8U;
    FC_Compass_Init(&config, &state);
    for (int i = 0; i < 5; i++) {
        assert(FC_Compass_Update(&config, &state, field, 0.004f));
    }
    assert(FC_Compass_Field(&state, corrected));
    assert(NEAR(corrected[0], -raw_y, 0.01));
    assert(NEAR(corrected[1], raw_x, 0.01));

    /* Declination is a rotation of the horizontal field. */
    FC_Compass_ConfigDefault(&config);
    config.declination_rad = (float)(10.0 * DEG);
    config.min_samples = 8U;
    FC_Compass_Init(&config, &state);
    for (int i = 0; i < 5; i++) {
        assert(FC_Compass_Update(&config, &state, field, 0.004f));
    }
    assert(FC_Compass_Field(&state, corrected));
    {
        const double expect_x = raw_x * cos(10.0 * DEG) + raw_y * sin(10.0 * DEG);
        const double expect_y = raw_y * cos(10.0 * DEG) - raw_x * sin(10.0 * DEG);
        assert(NEAR(corrected[0], expect_x, 0.01));
        assert(NEAR(corrected[1], expect_y, 0.01));
        assert(NEAR(corrected[2], field[2], 0.01));
    }
}

/* --------------------------------------------------------------------------
 * NMEA / UBX parser
 * -------------------------------------------------------------------------- */

/* Feed a whole sentence, CRLF included, and return whether the parser accepted
 * it. Bytes before the line terminator must never report a fix, and a receiver
 * terminates with either CR or LF, so the first terminator ends the sentence and
 * the second is a stray empty line. */
static bool feed_line(gps_parser_t *p, const char *sentence)
{
    bool accepted = false;
    for (const char *c = sentence; *c != '\0'; c++) {
        const bool result = GPS_Parser_Feed(p, (uint8_t)*c);
        if ((*c == '\r') || (*c == '\n')) {
            /* CRLF: the CR completes the sentence and the LF is then an empty
             * line, so the verdict has to be sticky rather than last-byte-wins. */
            accepted = accepted || result;
        } else {
            assert(!result);       /* mid-sentence bytes cannot complete a fix */
        }
    }
    return accepted;
}

/* Append the NMEA XOR checksum so the tests exercise the same verification a
 * real receiver's output goes through, and report whether the sentence parsed.
 * "Parsed" is not the same as "has a fix": a well-formed GGA with quality 0 is
 * parsed and still reports fix_type 0, which is exactly what has to reach the
 * gates. */
static bool feed_nmea(gps_parser_t *p, const char *body)
{
    char sentence[128];
    uint8_t sum = 0U;
    for (const char *c = body; *c != '\0'; c++) {
        sum ^= (uint8_t)*c;
    }
    snprintf(sentence, sizeof(sentence), "%s*%02X\r\n", body, sum);
    return feed_line(p, sentence);
}

static void test_parser_nmea_gga(void)
{
    gps_parser_t parser;
    GPS_Parser_Init(&parser);
    /* 21.0285 N, 105.8542 E, 12 sats, HDOP 0.9, 16 m */
    assert(feed_nmea(&parser,
                     "$GNGGA,081530.000,2101.71000,N,10551.25200,E,1,12,0.90,16.4,M,4.2,M,1.0,0000"));
    assert(NEAR(parser.sample.lat_deg, 21.0285, 1e-4));
    assert(NEAR(parser.sample.lon_deg, 105.8542, 1e-4));
    assert(NEAR(parser.sample.alt_m, 16.4, 0.01));
    assert(parser.sample.num_sats == 12U);
    assert(NEAR(parser.sample.hdop, 0.9, 1e-6));
    assert(parser.sample.fix_type == 3U);
    assert(parser.sample.vel_valid == 0U);   /* GGA has no velocity */
}

static void test_parser_nmea_southern_western(void)
{
    gps_parser_t parser;
    GPS_Parser_Init(&parser);
    /* Southern and western hemispheres must come out negative. */
    assert(feed_nmea(&parser,
                     "$GPGGA,081530.000,3345.00000,S,15112.00000,W,1,09,1.10,12.0,M,0.0,M,,0000"));
    assert(parser.sample.lat_deg < 0.0);
    assert(parser.sample.lon_deg < 0.0);
    assert(NEAR(fabs(parser.sample.lat_deg), 33.75, 1e-4));
    assert(NEAR(fabs(parser.sample.lon_deg), 151.2, 1e-4));
}

static void test_parser_nmea_no_fix(void)
{
    gps_parser_t parser;
    GPS_Parser_Init(&parser);
    /* Fix quality 0 means no fix, and the gates must see it. */
    /* The sentence is well formed, so it parses; the quality field is what
     * reports no fix. */
    assert(feed_nmea(&parser,
                     "$GNGGA,081530.000,2101.71000,N,10551.25200,E,0,00,,,M,,M,,M,,0000"));
    assert(parser.sample.fix_type == 0U);
}

static void test_parser_nmea_checksum_rejected(void)
{
    gps_parser_t parser;
    GPS_Parser_Init(&parser);
    /* Deliberately wrong checksum. */
    feed_line(&parser,
              "$GNGGA,081530.000,2101.71000,N,10551.25200,E,1,12,0.90,16.4,M,4.2,M,1.0,0000*00\n");
    assert(parser.sample.num_sats == 0U);
    assert(GPS_Parser_ChecksumErrors(&parser) == 1U);
}

static void test_parser_nmea_rmc_velocity(void)
{
    gps_parser_t parser;
    GPS_Parser_Init(&parser);
    /* 10 knots over ground on course 090: all of it is east velocity. */
    assert(feed_nmea(&parser,
                     "$GNRMC,081530.000,A,2101.71000,N,10551.25200,E,10.0,90.0,280926,,,A"));
    assert(parser.sample.vel_valid == 1U);
    assert(NEAR(parser.sample.vel_ned[0], 0.0, 1e-6));
    assert(NEAR(parser.sample.vel_ned[1], 10.0 * 0.514444, 0.01));
    /* RMC reports speed over ground, so the components must still add up to it
     * after the course split instead of all landing on north. */
    const float speed = sqrtf((parser.sample.vel_ned[0] * parser.sample.vel_ned[0]) +
                              (parser.sample.vel_ned[1] * parser.sample.vel_ned[1]));
    assert(NEAR(speed, 10.0 * 0.514444, 0.01));
    assert(NEAR(parser.sample.vel_ned[2], 0.0, 1e-6));
}

static void put_i32(uint8_t *p, int32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

/* Build a UBX frame: B5 62 | class id len0 len1 | ck ck | payload | ck ck.
 * The running checksum covers class through len1 for the header, then resumes
 * from those two values across the payload, so the frame carries two checksum
 * pairs, not one. */
#define UBX_NAV_PVT_PAYLOAD 92U
#define UBX_NAV_PVT_FRAME (2U + 4U + 2U + UBX_NAV_PVT_PAYLOAD + 2U)

static uint8_t ubx_ck_a(uint8_t a, uint8_t b)
{
    return (uint8_t)(a + b);
}

static uint8_t ubx_ck_b(uint8_t b, uint8_t ck_a)
{
    return (uint8_t)(b + ck_a);
}

static void ubx_frame(uint8_t *frame, const uint8_t *payload, uint8_t len,
                      bool corrupt_payload_checksum)
{
    uint8_t ck_a = 0U;
    uint8_t ck_b = 0U;
    frame[0] = 0xB5U;
    frame[1] = 0x62U;
    frame[2] = 0x01U;      /* NAV */
    frame[3] = 0x07U;      /* NAV-PVT */
    frame[4] = len;
    frame[5] = 0U;
    ck_a = ubx_ck_a(ck_a, frame[2]);
    ck_b = ubx_ck_b(ck_b, ck_a);
    ck_a = ubx_ck_a(ck_a, frame[3]);
    ck_b = ubx_ck_b(ck_b, ck_a);
    ck_a = ubx_ck_a(ck_a, frame[4]);
    ck_b = ubx_ck_b(ck_b, ck_a);
    ck_a = ubx_ck_a(ck_a, frame[5]);
    ck_b = ubx_ck_b(ck_b, ck_a);
    frame[6] = ck_a;
    frame[7] = ck_b;
    memcpy(&frame[8], payload, len);
    for (uint32_t i = 0U; i < len; i++) {
        ck_a = ubx_ck_a(ck_a, frame[8U + i]);
        ck_b = ubx_ck_b(ck_b, ck_a);
    }
    frame[8U + len] = corrupt_payload_checksum ? (uint8_t)(ck_a ^ 0xFFU) : ck_a;
    frame[9U + len] = ck_b;
}

static void test_parser_ubx_nav_pvt(void)
{
    gps_parser_t parser;
    uint8_t frame[UBX_NAV_PVT_FRAME];
    uint8_t payload[UBX_NAV_PVT_PAYLOAD];

    GPS_Parser_Init(&parser);
    memset(payload, 0, sizeof(payload));
    payload[11] = 0x08U;   /* valid: fix is 3D */
    payload[12] = 0x02U;   /* flags: gnssFixOK */
    payload[14] = 14U;     /* numSV */
    put_i32(&payload[15], (int32_t)(105.8542 * 1e7));
    put_i32(&payload[19], (int32_t)(21.0285 * 1e7));
    put_i32(&payload[27], 16400);        /* hMSL, mm */
    put_i32(&payload[39], 1200);         /* velN, mm/s */
    put_i32(&payload[43], -300);         /* velE */
    put_i32(&payload[47], 50);           /* velD */
    put_u16(&payload[67], 85);           /* pDOP = 0.85 */

    ubx_frame(frame, payload, (uint8_t)UBX_NAV_PVT_PAYLOAD, false);

    bool got = false;
    for (uint32_t i = 0U; i < sizeof(frame); i++) {
        if (GPS_Parser_Feed(&parser, frame[i])) {
            got = true;
        }
    }
    assert(got);
    assert(NEAR(parser.sample.lat_deg, 21.0285, 1e-4));
    assert(NEAR(parser.sample.lon_deg, 105.8542, 1e-4));
    assert(NEAR(parser.sample.alt_m, 16.4, 0.01));
    assert(NEAR(parser.sample.vel_ned[0], 1.2, 1e-3));
    assert(NEAR(parser.sample.vel_ned[1], -0.3, 1e-3));
    assert(parser.sample.num_sats == 14U);
    assert(NEAR(parser.sample.hdop, 0.85, 1e-6));
    assert(parser.sample.fix_type == 3U);
    assert(parser.sample.vel_valid == 1U);
}

static void test_parser_ubx_checksum_rejected(void)
{
    gps_parser_t parser;
    uint8_t frame[UBX_NAV_PVT_FRAME];
    uint8_t payload[UBX_NAV_PVT_PAYLOAD];

    GPS_Parser_Init(&parser);
    memset(payload, 0, sizeof(payload));
    payload[11] = 0x08U;
    payload[12] = 0x02U;
    payload[14] = 14U;
    /* Corrupt the payload checksum pair and leave the header one valid, so the
     * test proves the parser verifies the second pair rather than the first. */
    ubx_frame(frame, payload, (uint8_t)UBX_NAV_PVT_PAYLOAD, true);

    for (uint32_t i = 0U; i < sizeof(frame); i++) {
        assert(!GPS_Parser_Feed(&parser, frame[i]));
    }
    assert(GPS_Parser_ChecksumErrors(&parser) >= 1U);
}

static void test_parser_ubx_resyncs_on_garbage(void)
{
    gps_parser_t parser;
    uint8_t frame[UBX_NAV_PVT_FRAME];
    uint8_t payload[UBX_NAV_PVT_PAYLOAD];

    GPS_Parser_Init(&parser);
    memset(payload, 0, sizeof(payload));
    payload[11] = 0x08U;
    payload[12] = 0x02U;
    payload[14] = 9U;
    put_i32(&payload[15], (int32_t)(105.8542 * 1e7));
    put_i32(&payload[19], (int32_t)(21.0285 * 1e7));
    ubx_frame(frame, payload, (uint8_t)UBX_NAV_PVT_PAYLOAD, false);

    /* A truncated frame, then random bytes, then a good frame. Halfway through
     * the first header the parser is legitimately waiting for more bytes, so
     * the requirement is not that it abandons the frame immediately but that it
     * still latches onto the next valid preamble afterwards, which is what a
     * receiver has to do after a UART overrun. */
    const uint8_t junk[] = {0xB5U, 0x62U, 0x01U, 0x07U, 0x5CU, 0x00U,
                            0xFFU, 0xA3U, 0x5CU, 0x11U};
    for (uint32_t i = 0U; i < sizeof(junk); i++) {
        (void)GPS_Parser_Feed(&parser, junk[i]);
    }
    assert(parser.ubx_state == 0U);

    bool got = false;
    for (uint32_t i = 0U; i < sizeof(frame); i++) {
        if (GPS_Parser_Feed(&parser, frame[i])) {
            got = true;
        }
    }
    assert(got);
    assert(NEAR(parser.sample.lat_deg, 21.0285, 1e-4));
    assert(parser.sample.num_sats == 9U);
}

static void test_null_port_never_fixes(void)
{
    gps_port_t port;
    fc_gps_sample_t sample;
    GPS_Port_Null_Init(&port);
    assert(port.init != NULL);
    assert(port.init(port.context));
    assert(port.read_latest(port.context, &sample) == GPS_PORT_NO_DATA);
    assert(!port.healthy(port.context));
}

/* End to end: a parsed UBX fix must survive the sanity gates and come out as a
 * metric local position, which is exactly the path the firmware runs. */
static void test_parser_into_gps_module(void)
{
    gps_parser_t parser;
    fc_gps_config_t config;
    fc_gps_state_t state;
    uint8_t frame[UBX_NAV_PVT_FRAME];
    uint8_t payload[UBX_NAV_PVT_PAYLOAD];
    float pos[3] = {0.0f, 0.0f, 0.0f};

    FC_GPS_ConfigDefault(&config);
    FC_GPS_Init(&config, &state);
    GPS_Parser_Init(&parser);
    assert(FC_GPS_SetHome(&config, &state, HOME_LAT, HOME_LON, HOME_ALT));

    /* 10 m north, 5 m up of home. */
    memset(payload, 0, sizeof(payload));
    payload[11] = 0x08U;
    payload[12] = 0x02U;
    payload[14] = 16U;
    put_i32(&payload[15], (int32_t)(HOME_LON * 1e7));
    put_i32(&payload[19], (int32_t)((HOME_LAT + (10.0 / m_per_deg_lat())) * 1e7));
    put_i32(&payload[27], (int32_t)((HOME_ALT + 5.0) * 1000.0));
    put_i32(&payload[39], 800);
    put_u16(&payload[67], 70);

    ubx_frame(frame, payload, (uint8_t)UBX_NAV_PVT_PAYLOAD, false);

    bool fed = false;
    for (uint32_t i = 0U; i < sizeof(frame); i++) {
        if (GPS_Parser_Feed(&parser, frame[i])) {
            fed = true;
        }
    }
    assert(fed);
    assert(FC_GPS_Update(&config, &state, &parser.sample));
    assert(FC_GPS_GetLocal(&state, pos, NULL));
    assert(NEAR(pos[0], 10.0, 0.3));
    assert(NEAR(pos[1], 0.0, 0.3));
    assert(NEAR(pos[2], -5.0, 0.05));
}

int main(void)
{
    test_gps_config_defaults();
    test_gps_needs_home();
    test_gps_lat_lon_to_ned();
    test_gps_rejects_bad_fixes();
    test_gps_home_needs_a_good_fix();
    test_gps_rejects_jump();
    test_gps_velocity_held_when_missing();
    test_gps_home_moving_resets();
    test_compass_config_defaults();
    test_compass_rejects_dead_sensor();
    test_compass_needs_rotation_to_calibrate();
    test_compass_rejects_partial_rotation();
    test_compass_removes_hard_and_soft_iron();
    test_compass_rotation_and_declination();
    test_parser_nmea_gga();
    test_parser_nmea_southern_western();
    test_parser_nmea_no_fix();
    test_parser_nmea_checksum_rejected();
    test_parser_nmea_rmc_velocity();
    test_parser_ubx_nav_pvt();
    test_parser_ubx_checksum_rejected();
    test_parser_ubx_resyncs_on_garbage();
    test_null_port_never_fixes();
    test_parser_into_gps_module();
    printf("ALL NAV TESTS PASSED\n");
    return 0;
}
