#ifndef FC_GPS_H
#define FC_GPS_H

#include <stdbool.h>
#include <stdint.h>

/* GNSS receiver front end: turns lat/lon/alt fixes into a local NED position
 * relative to a home point, behind sanity gates that keep a bad fix out of the
 * control loop.
 *
 * There is no Kalman filter here, and that is a deliberate limitation, not an
 * oversight: the estimator in this project has no accelerometer integration to
 * blend a fix with (it integrates attitude only), so fusing would mean
 * averaging a noisy position against nothing. The module therefore does what
 * can be done correctly: convert, gate and expose, and the position loop takes
 * the fix as the position estimate. GPS_FUSE in fc_param selects whether the
 * controller uses it (sim ground truth) or not.
 *
 * The local conversion is equirectangular about home, which is exact enough
 * over the few hundred metres a small airframe flies; the error grows with
 * distance and is not suitable for surveying. The simulation reports absolute
 * lat/lon, so the firmware - not the sim - owns this conversion, exactly as it
 * will have to on real hardware.
 */

#define FC_GPS_NO_FIX 0U

typedef struct {
    double lat_deg;            /* WGS84, degrees north */
    double lon_deg;            /* WGS84, degrees east */
    float alt_m;               /* altitude above MSL (m) */
    float vel_ned[3];          /* velocity from the fix, m/s, NED */
    uint32_t vel_valid;
    uint32_t num_sats;
    uint32_t fix_type;         /* 0 none, 2 2D, 3 3D */
    float hdop;                /* horizontal dilution of precision */
    uint64_t timestamp_us;
} fc_gps_sample_t;

typedef struct {
    double home_lat_deg;
    double home_lon_deg;
    float home_alt_m;
    float pos_gate_m;          /* GPS_POS_GATE: max innovation per fix (m) */
    float max_speed_m_s;       /* GPS_VMAX: sanity limit on reported speed */
    float max_alt_m;           /* GPS_ALT_MAX: reject fixes further than this */
    float max_hdop;            /* GPS_HDOP_MAX */
    uint32_t min_sats;         /* GPS_SAT_MIN */
    uint32_t min_fix_type;     /* GPS_FIX_MIN: 2 = 2D, 3 = 3D */
    uint32_t initialized;
} fc_gps_config_t;

typedef struct {
    double home_lat_deg;
    double home_lon_deg;
    float home_alt_m;
    float pos_ned[3];          /* local NED relative to home, m */
    float vel_ned[3];
    float alt_m;               /* last accepted altitude above MSL */
    double last_lat_deg;        /* last accepted fix, for logging and telemetry */
    double last_lon_deg;
    float hdop;
    float speed_m_s;
    uint32_t num_sats;
    uint32_t fix_type;
    uint32_t home_set;
    uint32_t valid;            /* fix is usable by the controller */
    uint32_t rejected;         /* fixes dropped by the sanity gates */
    uint32_t updates;
    uint32_t initialized;
} fc_gps_state_t;

void FC_GPS_ConfigDefault(fc_gps_config_t *config);
void FC_GPS_Init(fc_gps_config_t *config, fc_gps_state_t *state);

/* Whether a sample clears the quality gates (finite, in range, 3D fix, enough
 * satellites, usable HDOP). This is the test to apply before freezing home: it
 * does not need an origin, which is exactly the situation home is set in, and
 * freezing a no-fix or garbage position would put every later NED reading at
 * the wrong place with no way to notice. */
bool FC_GPS_SampleAcceptable(const fc_gps_config_t *config,
                             const fc_gps_sample_t *sample);

/* Freeze the local origin. Call it while the vehicle is parked at the launch
 * point and the fix is good, before the flight is armed. Until then
 * FC_GPS_Update rejects every fix, so a late-acquiring receiver can never move
 * the origin out from under a flight in progress. */
bool FC_GPS_SetHome(fc_gps_config_t *config, fc_gps_state_t *state,
                    double lat_deg, double lon_deg, float alt_m);

/* Feed one receiver sample. Returns true when the fix was accepted. */
bool FC_GPS_Update(fc_gps_config_t *config, fc_gps_state_t *state,
                   const fc_gps_sample_t *sample);

/* Local NED position and velocity of the last accepted fix. */
bool FC_GPS_GetLocal(const fc_gps_state_t *state, float pos_ned[3], float vel_ned[3]);

bool FC_GPS_IsValid(const fc_gps_state_t *state);

#endif
