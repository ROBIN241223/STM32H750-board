#ifndef GPS_PORT_H
#define GPS_PORT_H

#include <stdbool.h>
#include "fc_gps.h"

typedef enum {
    GPS_PORT_OK = 0,
    GPS_PORT_NO_DATA,
    GPS_PORT_NOT_READY,
    GPS_PORT_ERROR,
    GPS_PORT_INVALID_SAMPLE
} gps_port_status_t;

typedef struct {
    bool (*init)(void *context);
    gps_port_status_t (*read_latest)(void *context, fc_gps_sample_t *sample);
    bool (*healthy)(void *context);
    void *context;
} gps_port_t;

/* read_latest must be compared against GPS_PORT_OK, not tested for truth: the
 * failure codes are non-zero, so `if (read_latest(...))` would treat "no data
 * from a receiver that is not fitted" as a successful read and hand the caller
 * a stale sample. */

/* Port that never produces a fix, for boards without a receiver wired up. */
void GPS_Port_Null_Init(gps_port_t *port);

/* USART3 receiver: byte ring fed by the UART interrupt, parsed in task
 * context by gps_parser. */
bool GPS_Port_UartInit(void);
void GPS_Port_UartBind(gps_port_t *port);
void GPS_Port_RxByte(uint8_t byte);   /* called from the ISR */
void GPS_Port_UartRearm(void);        /* called after a UART error */
uint32_t GPS_Port_UartOverruns(void);

#endif
