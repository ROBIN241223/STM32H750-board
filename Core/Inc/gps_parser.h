#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_gps.h"

/* Byte-stream GNSS parser: turns NMEA sentences or u-blox UBX frames into
 * fc_gps_sample_t. Deliberately free of HAL so the exact code that will run on
 * the flight controller is the code the host tests exercise.
 *
 * Supported:
 *   NMEA  GGA  - position, fix quality, satellite count, HDOP, altitude
 *   NMEA  RMC  - position, status, ground speed, course over ground
 *   UBX   NAV-PVT (class 0x01 id 0x07, 92 byte payload) - everything above in
 *         one frame, including NED velocity and MSL altitude
 *
 * Both protocols are checksummed and an unverified frame is dropped, because a
 * corrupted fix must never reach the position loop. NMEA and UBX arrive
 * interleaved from most receivers, so the parser runs both state machines at
 * once and GGA/RMC fill in a shared sample.
 */

#define GPS_NMEA_LINE_MAX 96U
#define GPS_UBX_PAYLOAD_MAX 92U

typedef struct {
    fc_gps_sample_t sample;
    char line[GPS_NMEA_LINE_MAX];
    uint32_t line_len;
    uint32_t in_line;
    uint32_t have_gga;
    uint32_t have_rmc;
    /* UBX framing: sync, header, payload, checksum. */
    uint32_t ubx_state;
    uint32_t ubx_class;
    uint32_t ubx_id;
    uint32_t ubx_len;
    uint32_t ubx_fill;
    uint32_t ubx_ck_a;
    uint32_t ubx_ck_b;
    uint32_t ubx_pending_a;
    uint32_t ubx_pending_b;
    uint8_t ubx_payload[GPS_UBX_PAYLOAD_MAX];
    uint32_t bytes_seen;
    uint32_t checksum_errors;
    uint32_t initialized;
} gps_parser_t;

void GPS_Parser_Init(gps_parser_t *parser);

/* Feed one received byte. Returns true when this byte completed a sentence or
 * frame that produced a usable fix, so a caller can latch the sample exactly
 * once per update. */
bool GPS_Parser_Feed(gps_parser_t *parser, uint8_t byte);

/* Frames parsed but rejected by a checksum. */
uint32_t GPS_Parser_ChecksumErrors(const gps_parser_t *parser);

#endif
