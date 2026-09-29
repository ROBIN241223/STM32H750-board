#include "gps_parser.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define GPS_UBX_SYNC1 0xB5U
#define GPS_UBX_SYNC2 0x62U
#define GPS_UBX_CLASS_NAV 0x01U
#define GPS_UBX_ID_NAV_PVT 0x07U

/* UBX framing states. */
#define GPS_UBX_WAIT_SYNC 0U
#define GPS_UBX_HEADER 1U
#define GPS_UBX_PAYLOAD 2U
#define GPS_UBX_CHECKSUM 3U

/* NMEA knots to m/s. */
#define GPS_KNOTS_TO_MS 0.514444

static int32_t rd_i32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void GPS_Parser_Init(gps_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->ubx_state = GPS_UBX_WAIT_SYNC;
    parser->initialized = 1U;
}

uint32_t GPS_Parser_ChecksumErrors(const gps_parser_t *parser)
{
    return (parser == NULL) ? 0U : parser->checksum_errors;
}

/* Split off the next comma-delimited field in place. An empty field is legal in
 * NMEA (that is how a receiver reports "no value"), so an empty field returns a
 * pointer to the terminator rather than NULL; NULL only means the sentence ran
 * out. */
static char *take(char **cursor)
{
    if ((cursor == NULL) || (*cursor == NULL)) {
        return NULL;
    }
    char *start = *cursor;
    char *comma = strchr(start, ',');
    if (comma != NULL) {
        *comma = '\0';
        *cursor = comma + 1;
    } else {
        *cursor = start + strlen(start);
    }
    return start;
}

static double field_double(const char *text)
{
    if ((text == NULL) || (*text == '\0')) {
        return 0.0;
    }
    return strtod(text, NULL);
}

/* ddmm.mmmm / dddmm.mmmm plus a hemisphere into signed degrees. */
static bool field_position(const char *text, const char *hemi, bool is_lat, double *out_deg)
{
    if ((text == NULL) || (hemi == NULL) || (*text == '\0') || (*hemi == '\0')) {
        return false;
    }
    /* ddmm.mmmm: the degrees are the leading integer digits and the minutes are
     * the rest, so the degrees must be floored, not divided by 100. Dividing
     * gives 2101.71 -> 21.0171 degrees instead of 21 degrees 1.71 minutes, which
     * is 21.0285: a 100 m position error, and the southern/western check for
     * "minutes < 60" then misfires on any latitude at all. */
    const double raw = strtod(text, NULL);
    const double whole_deg = floor(raw / 100.0);
    const double minutes = raw - (whole_deg * 100.0);
    if ((minutes < 0.0) || (minutes >= 60.0)) {
        return false;
    }
    double value = whole_deg + (minutes / 60.0);
    if ((*hemi == 'S') || (*hemi == 'W')) {
        value = -value;
    }
    if (is_lat && ((value < -90.0) || (value > 90.0))) {
        return false;
    }
    if (!is_lat && ((value < -180.0) || (value > 180.0))) {
        return false;
    }
    *out_deg = value;
    return true;
}

static bool nmea_checksum_ok(const char *line)
{
    const char *star = strrchr(line, '*');
    if (star == NULL) {
        return false;
    }
    uint8_t sum = 0U;
    for (const char *p = line; p < star; p++) {
        sum ^= (uint8_t)*p;
    }
    return (uint8_t)strtoul(star + 1, NULL, 16) == sum;
}

static bool sentence_is(const char *line, const char *id)
{
    /* "$" + 2-3 char talker + 3 char sentence, so the id always starts at 3. */
    return (strlen(line) >= 7U) &&
           ((line[0] == '$') || (line[0] == '!')) &&
           (strncmp(&line[3], id, 3U) == 0);
}

/* GGA: time, lat, NS, lon, EW, fix quality, sats, HDOP, alt, alt unit, geoid,
 * geoid unit, age, station id. */
static bool handle_gga(gps_parser_t *parser)
{
    /* '$' + 2-char talker + 3-char id occupies indices 0..5, so the first field
     * starts at 7, not 6: index 6 is the separator and taking from there would
     * shift every field by one and hand the time field to the latitude parser. */
    char *cursor = &parser->line[7];
    (void)take(&cursor);                       /* time, not used: the caller stamps it */
    char *lat_text = take(&cursor);
    char *lat_hemi = take(&cursor);
    char *lon_text = take(&cursor);
    char *lon_hemi = take(&cursor);
    char *quality = take(&cursor);
    char *sats = take(&cursor);
    char *hdop = take(&cursor);
    char *alt = take(&cursor);
    if ((lat_text == NULL) || (lat_hemi == NULL) || (lon_text == NULL) ||
        (lon_hemi == NULL) || (quality == NULL) || (sats == NULL) ||
        (hdop == NULL) || (alt == NULL)) {
        return false;
    }

    double lat = 0.0;
    double lon = 0.0;
    if (!field_position(lat_text, lat_hemi, true, &lat) ||
        !field_position(lon_text, lon_hemi, false, &lon)) {
        return false;
    }
    parser->sample.lat_deg = lat;
    parser->sample.lon_deg = lon;
    parser->sample.alt_m = (float)field_double(alt);
    parser->sample.num_sats = (uint32_t)(field_double(sats) + 0.5);
    parser->sample.hdop = (float)field_double(hdop);
    /* GGA reports fix quality, not a 2D/3D flag: 1 = autonomous 3D, 2 =
     * differential, 6 = RTK. Only "no fix" maps to fix_type 0. */
    parser->sample.fix_type = (field_double(quality) >= 1.0) ? 3U : 0U;
    /* GGA carries no velocity, so do not let a previous RMC survive as if it
     * belonged to this fix. */
    parser->sample.vel_valid = 0U;
    parser->have_gga = 1U;
    return true;
}

/* RMC: time, status, lat, NS, lon, EW, speed over ground, course, date,
 * magnetic variation, E/W, mode. */
static bool handle_rmc(gps_parser_t *parser)
{
    char *cursor = &parser->line[7];           /* same offset as GGA */
    (void)take(&cursor);                       /* time */
    char *status = take(&cursor);
    char *lat_text = take(&cursor);
    char *lat_hemi = take(&cursor);
    char *lon_text = take(&cursor);
    char *lon_hemi = take(&cursor);
    char *knots = take(&cursor);
    char *course = take(&cursor);
    if ((status == NULL) || (lat_text == NULL) || (lat_hemi == NULL) ||
        (lon_text == NULL) || (lon_hemi == NULL) || (knots == NULL)) {
        return false;
    }

    double lat = 0.0;
    double lon = 0.0;
    if (!field_position(lat_text, lat_hemi, true, &lat) ||
        !field_position(lon_text, lon_hemi, false, &lon)) {
        return false;
    }
    parser->sample.lat_deg = lat;
    parser->sample.lon_deg = lon;
    /* RMC speed is the magnitude of the horizontal velocity, not its north
     * component, so it has to be split with the course over ground. Putting the
     * whole speed on north makes a 090-degree run report pure northward motion
     * and wrecks every course-dependent consumer downstream. */
    const double speed = field_double(knots) * GPS_KNOTS_TO_MS;
    const double heading = (course == NULL) ? 0.0 : field_double(course);
    const double heading_rad = heading * (3.14159265358979323846 / 180.0);
    parser->sample.vel_ned[0] = (float)(speed * cos(heading_rad));
    parser->sample.vel_ned[1] = (float)(speed * sin(heading_rad));
    parser->sample.vel_ned[2] = 0.0f;   /* no vertical component is reported */
    parser->sample.vel_valid = 1U;
    if (*status == 'V') {
        parser->sample.fix_type = 0U;         /* void: keep position, kill fix */
    }
    parser->have_rmc = 1U;
    return true;
}

static bool nmea_dispatch(gps_parser_t *parser)
{
    if (sentence_is(parser->line, "GGA") || sentence_is(parser->line, "RMC")) {
        if (!nmea_checksum_ok(parser->line)) {
            parser->checksum_errors++;
            return false;
        }
        if (sentence_is(parser->line, "GGA")) {
            return handle_gga(parser);
        }
        return handle_rmc(parser);
    }
    return false;
}

static void ubx_checksum_byte(gps_parser_t *parser, uint8_t byte)
{
    parser->ubx_ck_a = (parser->ubx_ck_a + byte) & 0xFFU;
    parser->ubx_ck_b = (parser->ubx_ck_b + parser->ubx_ck_a) & 0xFFU;
}

static bool handle_nav_pvt(gps_parser_t *parser)
{
    const uint8_t *p = parser->ubx_payload;
    if (parser->ubx_len != GPS_UBX_PAYLOAD_MAX) {
        return false;
    }
    const bool fix_ok = ((p[12] & 0x02U) != 0U);   /* flags: gnssFixOK */
    const bool fix_3d = ((p[11] & 0x08U) != 0U);   /* valid: fix is a 3D fix */

    parser->sample.lat_deg = (double)rd_i32(&p[19]) * 1e-7;
    parser->sample.lon_deg = (double)rd_i32(&p[15]) * 1e-7;
    parser->sample.alt_m = (float)rd_i32(&p[27]) * 1e-3f;   /* hMSL, not ellipsoidal */
    parser->sample.vel_ned[0] = (float)rd_i32(&p[39]) * 1e-3f;
    parser->sample.vel_ned[1] = (float)rd_i32(&p[43]) * 1e-3f;
    parser->sample.vel_ned[2] = (float)rd_i32(&p[47]) * 1e-3f;
    parser->sample.num_sats = p[14];
    parser->sample.hdop = (float)rd_u16(&p[67]) * 0.01f;     /* pDOP */
    parser->sample.fix_type = (fix_ok && fix_3d) ? 3U : ((fix_ok) ? 2U : 0U);
    parser->sample.vel_valid = 1U;
    return true;
}

static bool ubx_dispatch(gps_parser_t *parser)
{
    if ((parser->ubx_class != GPS_UBX_CLASS_NAV) ||
        (parser->ubx_id != GPS_UBX_ID_NAV_PVT)) {
        return false;
    }
    return handle_nav_pvt(parser);
}

/* UBX framing: B5 62 | class id len0 len1 ck ck | payload | ck ck. The header
 * and the payload carry separate checksums, and the payload one resumes from
 * the header values, so the running sums are verified twice. */
static bool ubx_feed(gps_parser_t *parser, uint8_t byte)
{
    switch (parser->ubx_state) {
    case GPS_UBX_WAIT_SYNC:
        if (parser->ubx_fill == 0U) {
            if (byte == GPS_UBX_SYNC1) {
                parser->ubx_fill = 1U;
            }
        } else if (byte == GPS_UBX_SYNC2) {
            parser->ubx_state = GPS_UBX_HEADER;
            parser->ubx_fill = 0U;
            parser->ubx_class = 0U;
            parser->ubx_id = 0U;
            parser->ubx_len = 0U;
            parser->ubx_ck_a = 0U;
            parser->ubx_ck_b = 0U;
        } else {
            /* A second 0xB5 restarts the sync, anything else resyncs. */
            parser->ubx_fill = (byte == GPS_UBX_SYNC1) ? 1U : 0U;
        }
        break;

    case GPS_UBX_HEADER:
        if (parser->ubx_fill < 4U) {
            switch (parser->ubx_fill) {
            case 0U: parser->ubx_class = byte; break;
            case 1U: parser->ubx_id = byte; break;
            case 2U: parser->ubx_len = byte; break;
            default: parser->ubx_len |= ((uint32_t)byte) << 8; break;
            }
            ubx_checksum_byte(parser, byte);
            parser->ubx_fill++;
        } else if (parser->ubx_fill == 4U) {
            /* Header checksum covers class..len1 only; the payload continues
             * the running sum from these two values. */
            parser->ubx_pending_a = byte;    /* header checksum ck_a */
            parser->ubx_fill = 5U;
        } else {
            const bool ok = ((parser->ubx_ck_a == parser->ubx_pending_a) &&
                             (parser->ubx_ck_b == byte));
            if (!ok) {
                parser->checksum_errors++;
                parser->ubx_state = GPS_UBX_WAIT_SYNC;
                parser->ubx_fill = 0U;
            } else if (parser->ubx_len > GPS_UBX_PAYLOAD_MAX) {
                /* A frame this module does not implement and must not buffer;
                 * resync on the next preamble. */
                parser->ubx_state = GPS_UBX_WAIT_SYNC;
                parser->ubx_fill = 0U;
            } else {
                parser->ubx_fill = 0U;       /* payload index starts at zero */
                parser->ubx_state = (parser->ubx_len == 0U)
                                        ? GPS_UBX_WAIT_SYNC : GPS_UBX_PAYLOAD;
            }
        }
        break;

    case GPS_UBX_PAYLOAD:
        parser->ubx_payload[parser->ubx_fill] = byte;
        ubx_checksum_byte(parser, byte);
        parser->ubx_fill++;
        if (parser->ubx_fill >= parser->ubx_len) {
            parser->ubx_fill = 0U;
            parser->ubx_state = GPS_UBX_CHECKSUM;
        }
        break;

    case GPS_UBX_CHECKSUM:
        if (parser->ubx_fill == 0U) {
            parser->ubx_pending_a = byte;
            parser->ubx_fill = 1U;
        } else {
            const bool ok = ((parser->ubx_ck_a == parser->ubx_pending_a) &&
                             (parser->ubx_ck_b == byte));
            parser->ubx_state = GPS_UBX_WAIT_SYNC;
            parser->ubx_fill = 0U;
            if (!ok) {
                parser->checksum_errors++;
            } else if (!ubx_dispatch(parser)) {
                return false;
            } else {
                return true;
            }
        }
        break;

    default:
        parser->ubx_state = GPS_UBX_WAIT_SYNC;
        parser->ubx_fill = 0U;
        break;
    }
    return false;
}

bool GPS_Parser_Feed(gps_parser_t *parser, uint8_t byte)
{
    if ((parser == NULL) || (parser->initialized == 0U)) {
        return false;
    }
    parser->bytes_seen++;

    bool result = false;
    if ((byte == (uint8_t)'\n') || (byte == (uint8_t)'\r')) {
        if (parser->in_line != 0U) {
            parser->line[parser->line_len] = '\0';
            result = nmea_dispatch(parser);
            parser->in_line = 0U;
            parser->line_len = 0U;
        }
    } else if (parser->line_len < (GPS_NMEA_LINE_MAX - 1U)) {
        parser->line[parser->line_len++] = (char)byte;
        parser->in_line = 1U;
    }

    if (ubx_feed(parser, byte)) {
        result = true;
    }
    return result;
}
