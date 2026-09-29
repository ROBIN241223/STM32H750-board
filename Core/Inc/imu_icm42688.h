#ifndef IMU_ICM42688_H
#define IMU_ICM42688_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "imu_port.h"

typedef bool (*imu_spi_transfer_fn)(void *context, const uint8_t *tx, uint8_t *rx, size_t length);
typedef void (*imu_chip_select_fn)(void *context, bool active);
typedef void (*imu_delay_ms_fn)(void *context, uint32_t delay_ms);
typedef uint64_t (*imu_timestamp_us_fn)(void *context);

typedef struct {
    imu_spi_transfer_fn transfer;
    imu_chip_select_fn chip_select;
    imu_delay_ms_fn delay_ms;
    imu_timestamp_us_fn timestamp_us;
    void *context;
} imu_spi_bus_t;

/* Register banks. The ICM-42688 keeps most of its configuration in bank 1/2 and
 * only resets to bank 0, so any access to another bank has to be selected first
 * and the driver has to remember where it currently is. Mirrors the bank layout
 * in PX4's src/drivers/imu/invensense/icm42688p. */
typedef enum {
    ICM42688_BANK_0 = 0x00U,
    ICM42688_BANK_1 = 0x01U,
    ICM42688_BANK_2 = 0x02U
} icm42688_bank_t;

typedef struct {
    imu_spi_bus_t bus;
    float gyro_rad_s_per_lsb;
    float accel_mps2_per_lsb;
    uint32_t sequence;
    uint8_t who_am_i;
    /* Bank the part is actually in; bank 0 is the post-reset default. */
    uint8_t register_bank;
    bool ready;
} imu_icm42688_t;

void ICM42688_ConfigDefault(imu_icm42688_t *device, const imu_spi_bus_t *bus);
bool ICM42688_Init(imu_icm42688_t *device);
imu_port_status_t ICM42688_ReadLatest(void *context, fc_imu_sample_t *sample);
bool ICM42688_Healthy(void *context);
void ICM42688_AsPort(imu_icm42688_t *device, imu_port_t *port);

#endif
