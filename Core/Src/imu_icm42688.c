#include "imu_icm42688.h"
#include <math.h>
#include <string.h>

/* Bank 0 registers (the reset default bank). */
#define ICM42688_REG_WHO_AM_I        0x75U
#define ICM42688_REG_PWR_MGMT0       0x4EU
#define ICM42688_REG_GYRO_CONFIG0    0x4FU
#define ICM42688_REG_ACCEL_CONFIG0   0x50U
#define ICM42688_REG_INT_STATUS      0x2DU
#define ICM42688_REG_REG_BANK_SEL    0x2EU
#define ICM42688_REG_DEVICE_CONFIG   0x11U
#define ICM42688_REG_ACCEL_DATA_X1   0x1FU

/* DEVICE_CONFIG */
#define ICM42688_SOFT_RESET_CONFIG   0x01U

/* INT_STATUS */
#define ICM42688_RESET_DONE_INT      0x10U

/* PWR_MGMT0: 0b1100 gyro low-noise | 0b0011 accel low-noise */
#define ICM42688_PWR_MGMT0_LN        0x0FU

/* GYRO_CONFIG0: FS_SEL[7:5] = 0b010 -> +/-500 dps, ODR[3:0] = 0b0110 -> 1 kHz */
#define ICM42688_GYRO_500DPS_1KHZ    0x46U

/* ACCEL_CONFIG0: FS_SEL[7:5] = 0b010 -> +/-4 g, ODR[3:0] = 0b0110 -> 1 kHz */
#define ICM42688_ACCEL_4G_1KHZ       0x46U

#define ICM42688_WHO_AM_I_VALUE      0x47U
#define ICM42688_GYRO_500DPS_LSB     (500.0f / 32768.0f * 0.017453292519943295f)
#define ICM42688_ACCEL_4G_LSB        (4.0f * 9.80665f / 32768.0f)

/* Reset handshake timings, matching PX4's ICM42688P state machine: 1 ms for the
 * soft reset to take effect, poll every 10 ms, give up after 1 s, then pause
 * 100 ms and try again. A bare fixed delay races the part's own reset. */
#define ICM42688_RESET_DELAY_MS      1U
#define ICM42688_RESET_POLL_MS       10U
#define ICM42688_RESET_TIMEOUT_MS    1000U
#define ICM42688_RESET_RETRY_MS      100U
#define ICM42688_RESET_ATTEMPTS      3U
/* Gyro startup time after leaving standby; accel needs ~10 ms on top. */
#define ICM42688_WAKE_DELAY_MS       40U

#define ICM42688_SPI_READ_BIT        0x80U
#define ICM42688_APEX_SAMPLE_BYTES   13U

static bool transfer(imu_icm42688_t *device, const uint8_t *tx, uint8_t *rx, size_t length)
{
    if ((device == NULL) || (device->bus.transfer == NULL) ||
        (device->bus.chip_select == NULL)) {
        return false;
    }

    device->bus.chip_select(device->bus.context, true);
    bool result = device->bus.transfer(device->bus.context, tx, rx, length);
    device->bus.chip_select(device->bus.context, false);
    return result;
}

static bool write_register(imu_icm42688_t *device, uint8_t address, uint8_t value)
{
    const uint8_t tx[2] = {(uint8_t)(address & 0x7FU), value};
    uint8_t rx[2] = {0U, 0U};
    return transfer(device, tx, rx, sizeof(tx));
}

static bool read_register(imu_icm42688_t *device, uint8_t address, uint8_t *value)
{
    const uint8_t tx[2] = {(uint8_t)(address | ICM42688_SPI_READ_BIT), 0U};
    uint8_t rx[2] = {0U, 0U};
    if ((value == NULL) || !transfer(device, tx, rx, sizeof(tx))) {
        return false;
    }

    *value = rx[1];
    return true;
}

/* Switch the part to another register bank. Every bank-1/2 access depends on
 * this, and the driver must not assume it is still in bank 0 after the part or
 * something else has moved it. */
static bool select_register_bank(imu_icm42688_t *device, icm42688_bank_t bank)
{
    if (device->register_bank != (uint8_t)bank) {
        if (!write_register(device, ICM42688_REG_REG_BANK_SEL, (uint8_t)bank)) {
            return false;
        }
        device->register_bank = (uint8_t)bank;
    }
    return true;
}

static void delay_ms(imu_icm42688_t *device, uint32_t ms)
{
    if (device->bus.delay_ms != NULL) {
        device->bus.delay_ms(device->bus.context, ms);
    }
}

static int16_t be_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

void ICM42688_ConfigDefault(imu_icm42688_t *device, const imu_spi_bus_t *bus)
{
    if (device == NULL) {
        return;
    }

    memset(device, 0, sizeof(*device));
    if (bus != NULL) {
        device->bus = *bus;
    }
    device->gyro_rad_s_per_lsb = ICM42688_GYRO_500DPS_LSB;
    device->accel_mps2_per_lsb = ICM42688_ACCEL_4G_LSB;
    device->register_bank = (uint8_t)ICM42688_BANK_0;
}

/* Soft reset, then wait for the part to say it finished rather than guessing:
 * WHO_AM_I readable, the DEVICE_CONFIG reset bit self-cleared, and RESET_DONE
 * latched in INT_STATUS. Retried a bounded number of times, like PX4.
 *
 * A part that never identifies itself is a wiring/part problem, so that case
 * fails immediately instead of burning the retry budget. But the check is not
 * made on the first poll: one millisecond after a soft reset the part may
 * briefly not answer at all, and treating that as a wrong part would kill a
 * healthy sensor. */
static bool reset_and_wait(imu_icm42688_t *device)
{
    bool who_am_i_ok = false;
    uint8_t last_who_am_i = 0U;

    for (uint32_t attempt = 0U; attempt < ICM42688_RESET_ATTEMPTS; attempt++) {
        if (attempt > 0U) {
            delay_ms(device, ICM42688_RESET_RETRY_MS);
        }
        device->register_bank = (uint8_t)ICM42688_BANK_0;
        if (!write_register(device, ICM42688_REG_DEVICE_CONFIG, ICM42688_SOFT_RESET_CONFIG)) {
            continue;
        }
        delay_ms(device, ICM42688_RESET_DELAY_MS);

        for (uint32_t waited = 0U; waited < ICM42688_RESET_TIMEOUT_MS;
                waited += ICM42688_RESET_POLL_MS) {
            uint8_t who_am_i = 0U;
            uint8_t device_config = 0xFFU;
            uint8_t int_status = 0U;

            if (!read_register(device, ICM42688_REG_WHO_AM_I, &who_am_i)) {
                break;
            }
            last_who_am_i = who_am_i;
            if (who_am_i == ICM42688_WHO_AM_I_VALUE) {
                who_am_i_ok = true;
            }
            if (who_am_i_ok &&
                read_register(device, ICM42688_REG_DEVICE_CONFIG, &device_config) &&
                read_register(device, ICM42688_REG_INT_STATUS, &int_status) &&
                (device_config == 0U) &&
                ((int_status & ICM42688_RESET_DONE_INT) != 0U)) {
                device->who_am_i = who_am_i;
                return true;
            }
            delay_ms(device, ICM42688_RESET_POLL_MS);
        }
        if (!who_am_i_ok) {
            device->who_am_i = last_who_am_i;
            return false;
        }
    }
    return false;
}

bool ICM42688_Init(imu_icm42688_t *device)
{
    if ((device == NULL) || (device->bus.delay_ms == NULL)) {
        return false;
    }

    device->ready = false;
    device->sequence = 0U;
    if (!reset_and_wait(device)) {
        return false;
    }

    /* Wake before configuring: the ODR/FS registers are only guaranteed to take
     * once the part is out of standby, and the gyro needs time to start. PX4
     * writes PWR_MGMT0 first, waits 30 ms, and only then runs Configure(). */
    if (!write_register(device, ICM42688_REG_PWR_MGMT0, ICM42688_PWR_MGMT0_LN)) {
        return false;
    }
    delay_ms(device, ICM42688_WAKE_DELAY_MS);

    if (!write_register(device, ICM42688_REG_GYRO_CONFIG0, ICM42688_GYRO_500DPS_1KHZ) ||
        !write_register(device, ICM42688_REG_ACCEL_CONFIG0, ICM42688_ACCEL_4G_1KHZ)) {
        return false;
    }

    device->register_bank = (uint8_t)ICM42688_BANK_0;
    device->ready = true;
    return true;
}

imu_port_status_t ICM42688_ReadLatest(void *context, fc_imu_sample_t *sample)
{
    imu_icm42688_t *device = (imu_icm42688_t *)context;
    uint8_t tx[ICM42688_APEX_SAMPLE_BYTES] = {0U};
    uint8_t rx[ICM42688_APEX_SAMPLE_BYTES] = {0U};

    if ((device == NULL) || (sample == NULL)) {
        return IMU_PORT_ERROR;
    }
    if (!device->ready) {
        return IMU_PORT_NOT_READY;
    }
    if (!select_register_bank(device, ICM42688_BANK_0)) {
        return IMU_PORT_ERROR;
    }

    /* One burst: 0x1F.., accel XYZ then gyro XYZ, big endian, 16 bits each. */
    tx[0] = (uint8_t)(ICM42688_REG_ACCEL_DATA_X1 | ICM42688_SPI_READ_BIT);
    if (!transfer(device, tx, rx, sizeof(tx))) {
        return IMU_PORT_ERROR;
    }

    memset(sample, 0, sizeof(*sample));
    sample->timestamp_us = (device->bus.timestamp_us != NULL) ?
        device->bus.timestamp_us(device->bus.context) : 0U;
    sample->sequence = ++device->sequence;
    sample->accel_mps2[0] = (float)be_i16(&rx[1]) * device->accel_mps2_per_lsb;
    sample->accel_mps2[1] = (float)be_i16(&rx[3]) * device->accel_mps2_per_lsb;
    sample->accel_mps2[2] = (float)be_i16(&rx[5]) * device->accel_mps2_per_lsb;
    sample->gyro_rad_s[0] = (float)be_i16(&rx[7]) * device->gyro_rad_s_per_lsb;
    sample->gyro_rad_s[1] = (float)be_i16(&rx[9]) * device->gyro_rad_s_per_lsb;
    sample->gyro_rad_s[2] = (float)be_i16(&rx[11]) * device->gyro_rad_s_per_lsb;
    if (!isfinite(sample->accel_mps2[2]) || !isfinite(sample->gyro_rad_s[2])) {
        return IMU_PORT_INVALID_SAMPLE;
    }
    sample->status = 0U;
    return IMU_PORT_OK;
}

bool ICM42688_Healthy(void *context)
{
    const imu_icm42688_t *device = (const imu_icm42688_t *)context;
    return (device != NULL) && device->ready && (device->who_am_i == ICM42688_WHO_AM_I_VALUE);
}

static bool icm42688_init_callback(void *context)
{
    return ICM42688_Init((imu_icm42688_t *)context);
}

void ICM42688_AsPort(imu_icm42688_t *device, imu_port_t *port)
{
    if ((device == NULL) || (port == NULL)) {
        return;
    }

    port->init = icm42688_init_callback;
    port->read_latest = ICM42688_ReadLatest;
    port->healthy = ICM42688_Healthy;
    port->context = device;
}
