#ifndef IMU_PORT_SPI_H
#define IMU_PORT_SPI_H

#include <stdint.h>
#include "imu_icm42688.h"
#include "imu_port.h"

/* Polling SPI front end for the board's ICM-42688 pair. The bus plumbing
 * (transfer / CS / delay / timestamp) is injected so this file stays free of
 * HAL dependencies and can be exercised on the host with a fake bus. */
typedef struct {
    imu_icm42688_t *devices;
    uint32_t device_count;
    /* Index of the IMU the controller should fuse; UINT32_MAX falls back to
     * whichever device answers first. */
    uint32_t primary;
} imu_port_spi_t;

void IMU_Port_Spi_Init(imu_port_spi_t *port, imu_icm42688_t *devices, uint32_t device_count);
void IMU_Port_Spi_AsPort(imu_port_spi_t *port, imu_port_t *interface);

#endif
