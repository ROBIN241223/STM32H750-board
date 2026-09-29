#include "imu_port.h"
#include <stddef.h>

static bool null_init(void *context)
{
    (void)context;
    return false;
}

static imu_port_status_t null_read(void *context, fc_imu_sample_t *sample)
{
    (void)context;
    (void)sample;
    return IMU_PORT_NOT_READY;
}

static bool null_healthy(void *context)
{
    (void)context;
    return false;
}

void IMU_Port_Null_Init(imu_port_t *port)
{
    if (port == NULL) {
        return;
    }

    port->init = null_init;
    port->read_latest = null_read;
    port->healthy = null_healthy;
    port->context = NULL;
}
