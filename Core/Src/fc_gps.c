#include "fc_gps.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* WGS84 mean radius. The equirectangular approximation below is the same one
 * PX4 uses for its local tangent plane, and keeps the error below a centimetre
 * over the few hundred metres this vehicle flies. */
#define FC_GPS_EARTH_RADIUS_M 6371008.8
#define FC_GPS_DEG2RAD (M_PI / 180.0)

static bool sample_finite(const fc_gps_sample_t *sample)
{
    if (!isfinite(sample->lat_deg) || !isfinite(sample->lon_deg) ||
        !isfinite(sample->alt_m) || !isfinite(sample->hdop)) {
        return false;
    }
    for (uint32_t i = 0U; i < 3U; i++) {
        if (!isfinite(sample->vel_ned[i])) {
            return false;
        }
    }
    return true;
}

static bool range_ok(const fc_gps_config_t *config, const fc_gps_sample_t *sample)
{
    if ((sample->lat_deg < -90.0) || (sample->lat_deg > 90.0)) {
        return false;
    }
    if ((sample->lon_deg < -180.0) || (sample->lon_deg > 180.0)) {
        return false;
    }
    if (sample->fix_type < config->min_fix_type) {
        return false;
    }
    if (sample->num_sats < config->min_sats) {
        return false;
    }
    if ((sample->hdop <= 0.0f) || (sample->hdop > config->max_hdop)) {
        return false;
    }
    return true;
}

void FC_GPS_ConfigDefault(fc_gps_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->pos_gate_m = 15.0f;      /* GPS_POS_GATE */
    config->max_speed_m_s = 25.0f;   /* GPS_VMAX */
    config->max_alt_m = 500.0f;      /* GPS_ALT_MAX */
    config->max_hdop = 5.0f;         /* GPS_HDOP_MAX */
    config->min_sats = 6U;           /* GPS_SAT_MIN */
    config->min_fix_type = 3U;       /* GPS_FIX_MIN: 3D fix */
    config->initialized = 1U;
}

void FC_GPS_Init(fc_gps_config_t *config, fc_gps_state_t *state)
{
    if ((config == NULL) || (state == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->home_lat_deg = config->home_lat_deg;
    state->home_lon_deg = config->home_lon_deg;
    state->home_alt_m = config->home_alt_m;
    state->valid = 0U;
    state->initialized = 1U;
}

bool FC_GPS_SampleAcceptable(const fc_gps_config_t *config,
                             const fc_gps_sample_t *sample)
{
    if ((config == NULL) || (sample == NULL)) {
        return false;
    }
    return sample_finite(sample) && range_ok(config, sample);
}

bool FC_GPS_SetHome(fc_gps_config_t *config, fc_gps_state_t *state,
                    double lat_deg, double lon_deg, float alt_m)
{
    if ((config == NULL) || (state == NULL) || !isfinite(lat_deg) || !isfinite(lon_deg) ||
        !isfinite(alt_m) || (lat_deg < -90.0) || (lat_deg > 90.0) ||
        (lon_deg < -180.0) || (lon_deg > 180.0)) {
        return false;
    }
    config->home_lat_deg = lat_deg;
    config->home_lon_deg = lon_deg;
    config->home_alt_m = alt_m;
    state->home_lat_deg = lat_deg;
    state->home_lon_deg = lon_deg;
    state->home_alt_m = alt_m;
    state->home_set = 1U;
    /* The origin moved, so the last fix no longer describes where we are. */
    state->pos_ned[0] = 0.0f;
    state->pos_ned[1] = 0.0f;
    state->pos_ned[2] = 0.0f;
    state->vel_ned[0] = 0.0f;
    state->vel_ned[1] = 0.0f;
    state->vel_ned[2] = 0.0f;
    state->valid = 0U;
    return true;
}

/* Equirectangular local tangent plane about home. North and east scale with
 * latitude the same way a real geodesic would, which is what makes the result
 * usable as a metric position. */
static void lat_lon_to_ned(const fc_gps_config_t *config, double lat_deg, double lon_deg,
                           float alt_m, float out_ned[3])
{
    const double m_per_deg_lat = FC_GPS_EARTH_RADIUS_M * FC_GPS_DEG2RAD;
    const double m_per_deg_lon = m_per_deg_lat * cos(config->home_lat_deg * FC_GPS_DEG2RAD);
    out_ned[0] = (float)((lat_deg - config->home_lat_deg) * m_per_deg_lat);
    out_ned[1] = (float)((lon_deg - config->home_lon_deg) * m_per_deg_lon);
    out_ned[2] = -(alt_m - config->home_alt_m);
}

static bool speed_ok(const fc_gps_config_t *config, const float vel_ned[3])
{
    const float speed = sqrtf(vel_ned[0] * vel_ned[0] +
                              vel_ned[1] * vel_ned[1] +
                              vel_ned[2] * vel_ned[2]);
    return speed <= config->max_speed_m_s;
}

bool FC_GPS_Update(fc_gps_config_t *config, fc_gps_state_t *state,
                   const fc_gps_sample_t *sample)
{
    if ((config == NULL) || (state == NULL) || (sample == NULL)) {
        return false;
    }
    if (!sample_finite(sample)) {
        /* A NaN out of a UART receiver is corruption, and it counts against
         * the fix quality the operator sees on the ground. */
        state->rejected++;
        return false;
    }
    state->fix_type = sample->fix_type;
    state->num_sats = sample->num_sats;
    state->hdop = sample->hdop;

    if (!range_ok(config, sample)) {
        state->rejected++;
        return false;
    }

    /* No origin yet: the fix is good but there is nothing to measure it
     * against. The caller must freeze home (FC_GPS_SetHome) while the vehicle
     * is disarmed, so an origin can never shift after takeoff. */
    if (state->home_set == 0U) {
        return false;
    }

    float pos_ned[3];
    lat_lon_to_ned(config, sample->lat_deg, sample->lon_deg, sample->alt_m, pos_ned);

    if (fabsf(pos_ned[2]) > config->max_alt_m) {
        state->rejected++;
        return false;
    }
    if (state->updates > 0U) {
        const float dx = pos_ned[0] - state->pos_ned[0];
        const float dy = pos_ned[1] - state->pos_ned[1];
        const float dz = pos_ned[2] - state->pos_ned[2];
        if ((dx * dx + dy * dy + dz * dz) > (config->pos_gate_m * config->pos_gate_m)) {
            /* A jump this large between two fixes is a multipath or a bad
             * reacquisition, not a vehicle that teleported. */
            state->rejected++;
            return false;
        }
    }

    float vel_ned[3];
    if (sample->vel_valid != 0U) {
        if (!speed_ok(config, sample->vel_ned)) {
            /* A fix claiming to be moving faster than the vehicle can fly is a
             * bad fix, not a missing velocity. Rejecting it keeps the accepted
             * position and velocity from contradicting each other. */
            state->rejected++;
            return false;
        }
        memcpy(vel_ned, sample->vel_ned, sizeof(vel_ned));
    } else {
        /* No velocity in the fix: hold the last one rather than differentiating
         * a noisy position, which would inject the GNSS noise straight into the
         * velocity loop. */
        memcpy(vel_ned, state->vel_ned, sizeof(vel_ned));
    }

    memcpy(state->pos_ned, pos_ned, sizeof(pos_ned));
    memcpy(state->vel_ned, vel_ned, sizeof(vel_ned));
    state->alt_m = sample->alt_m;
    state->last_lat_deg = sample->lat_deg;
    state->last_lon_deg = sample->lon_deg;
    state->speed_m_s = sqrtf(vel_ned[0] * vel_ned[0] +
                             vel_ned[1] * vel_ned[1] +
                             vel_ned[2] * vel_ned[2]);
    state->updates++;
    state->valid = 1U;
    return true;
}

bool FC_GPS_GetLocal(const fc_gps_state_t *state, float pos_ned[3], float vel_ned[3])
{
    if ((state == NULL) || (state->initialized == 0U) || (state->home_set == 0U)) {
        return false;
    }
    if (pos_ned != NULL) {
        memcpy(pos_ned, state->pos_ned, sizeof(state->pos_ned));
    }
    if (vel_ned != NULL) {
        memcpy(vel_ned, state->vel_ned, sizeof(state->vel_ned));
    }
    return true;
}

bool FC_GPS_IsValid(const fc_gps_state_t *state)
{
    return (state != NULL) && (state->valid != 0U) && (state->home_set != 0U);
}
