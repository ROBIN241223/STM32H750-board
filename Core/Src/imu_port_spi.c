#include "imu_port_spi.h"
#include <string.h>

/* Two ICM-42688 on SPI1/SCK=PA5 (IMU0) and SPI2/SCK=PB10 (IMU1), polled - the
 * part has its data ready long before the 1 kHz service tick, so there is no
 * DRDY line to wait on. Chip select is driven by the board layer through the
 * bus callbacks, not from here.
 *
 * The two devices are independent, so a failure on one must not stop the other:
 * each keeps its own retry budget and readiness, and ReadLatest reports
 * IMU_PORT_NO_DATA for whichever one has nothing new. */

static bool spi_port_init_single(void *context)
{
    imu_port_spi_t *port = (imu_port_spi_t *)context;

    if ((port == NULL) || (port->devices == NULL) || (port->device_count == 0U)) {
        return false;
    }
    for (uint32_t i = 0U; i < port->device_count; i++) {
        if (!ICM42688_Init(&port->devices[i])) {
            /* Leave ready clear and let the next tick try again; a single flaky
             * bus transaction should not permanently disarm this IMU. */
            port->devices[i].ready = false;
        }
    }

    /* Success means the IMU the controller will actually fuse came up. A dead
     * secondary is degraded but flyable; a dead primary is not. */
    if (port->primary == UINT32_MAX) {
        for (uint32_t i = 0U; i < port->device_count; i++) {
            if (port->devices[i].ready) {
                return true;
            }
        }
        return false;
    }
    return (port->primary < port->device_count) && port->devices[port->primary].ready;
}

static imu_port_status_t spi_port_read_latest(void *context, fc_imu_sample_t *sample)
{
    imu_port_spi_t *port = (imu_port_spi_t *)context;
    imu_port_status_t best = IMU_PORT_NOT_READY;
    bool any = false;

    if ((port == NULL) || (sample == NULL) || (port->devices == NULL)) {
        return IMU_PORT_ERROR;
    }

    /* Prefer the primary, but fall back to any device that answers. Both parts
     * sit on the same airframe and are the same silicon, so a sample from the
     * spare is interchangeable with one from the primary - that redundancy is
     * the reason for two IMUs. A bus level failure is different: it means the
     * device is producing nothing usable at all, so it stops counting. */
    for (uint32_t i = 0U; i < port->device_count; i++) {
        fc_imu_sample_t candidate;
        imu_port_status_t status = ICM42688_ReadLatest(&port->devices[i], &candidate);

        if (status == IMU_PORT_ERROR) {
            port->devices[i].ready = false;
            best = IMU_PORT_ERROR;
            continue;
        }
        if (status != IMU_PORT_OK) {
            continue;
        }
        if ((port->primary < port->device_count) && (i == port->primary)) {
            *sample = candidate;
            return IMU_PORT_OK;
        }
        if (!any) {
            *sample = candidate;
            any = true;
            best = IMU_PORT_OK;
        }
    }

    if (any) {
        return best;
    }
    return (best == IMU_PORT_ERROR) ? IMU_PORT_ERROR : IMU_PORT_NO_DATA;
}

static bool spi_port_healthy(void *context)
{
    const imu_port_spi_t *port = (const imu_port_spi_t *)context;

    if ((port == NULL) || (port->devices == NULL) || (port->device_count == 0U)) {
        return false;
    }
    for (uint32_t i = 0U; i < port->device_count; i++) {
        if (ICM42688_Healthy(&port->devices[i])) {
            return true;
        }
    }
    return false;
}

void IMU_Port_Spi_Init(imu_port_spi_t *port, imu_icm42688_t *devices, uint32_t device_count)
{
    if (port == NULL) {
        return;
    }

    memset(port, 0, sizeof(*port));
    port->devices = devices;
    port->device_count = (devices != NULL) ? device_count : 0U;
    /* Default to the first device; UINT32_MAX would mean "first one that
     * answers", which is only useful once both are known good. */
    port->primary = 0U;
}

void IMU_Port_Spi_AsPort(imu_port_spi_t *port, imu_port_t *interface)
{
    if ((port == NULL) || (interface == NULL)) {
        return;
    }

    interface->init = spi_port_init_single;
    interface->read_latest = spi_port_read_latest;
    interface->healthy = spi_port_healthy;
    interface->context = port;
}
