#include "gps_port.h"
#include <stddef.h>

/* Board with no GNSS receiver fitted: the control loop must see "no fix"
 * forever and fall back to whatever else it has, which is exactly what an
 * absent sensor should look like. */
static bool null_init(void *context)
{
    (void)context;
    return true;
}

static gps_port_status_t null_read(void *context, fc_gps_sample_t *sample)
{
    (void)context;
    (void)sample;
    return GPS_PORT_NO_DATA;
}

static bool null_healthy(void *context)
{
    (void)context;
    return false;
}

void GPS_Port_Null_Init(gps_port_t *port)
{
    if (port == NULL) {
        return;
    }
    port->init = null_init;
    port->read_latest = null_read;
    port->healthy = null_healthy;
    port->context = NULL;
}
