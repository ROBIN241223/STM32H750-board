#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "fc_attitude.h"
#include "fc_rate.h"
#include "fc_estimator.h"
#include "fc_mixer.h"
#include "fc_position.h"
#include "fc_land.h"
#include "fc_guard.h"
#include "fc_sched.h"
#include "fc_orb.h"
#include "fc_param.h"
#include "fc_mission.h"
#include "imu_icm42688.h"
#include "imu_port_spi.h"

#define NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))
#define RAD(deg) ((deg) * 3.141592653589793f / 180.0f)

static void test_estimator(void)
{
    fc_estimator_config_t estimator_config;
    fc_estimator_state_t estimator_state;
    fc_imu_sample_t sample;
    bool initialized = false;

    FC_Estimator_ConfigDefault(&estimator_config);
    FC_Estimator_Init(&estimator_config, &estimator_state);
    assert(NEAR(estimator_config.pitch_sign, 1.0f, 1e-6f));
    sample.timestamp_us = 0U;
    sample.sequence = 0U;
    sample.status = 0U;
    sample.accel_mps2[0] = 0.0f;
    sample.accel_mps2[1] = 0.0f;
    sample.accel_mps2[2] = -9.80665f;  /* FRD: z down at hover, specific force = -g */
    sample.gyro_rad_s[0] = 0.0f;
    sample.gyro_rad_s[1] = 0.0f;
    sample.gyro_rad_s[2] = 0.0f;

    for (uint32_t i = 0U; i < 220U; i++) {
        sample.timestamp_us = (uint64_t)(i + 1U) * 2000U;
        initialized = FC_Estimator_Update(&estimator_config, &estimator_state, &sample);
    }
    assert(initialized);
    assert(fabsf(estimator_state.roll_rad) < 0.01f);
    assert(fabsf(estimator_state.pitch_rad) < 0.01f);
    assert((estimator_state.flags & FC_ESTIMATOR_FLAG_INITIALIZED) != 0U);

    FC_Estimator_FeedPosition(&estimator_state, (float[3]){1.0f, 2.0f, 3.0f}, (float[3]){0.1f, -0.2f, 0.3f});
    assert(estimator_state.pos_valid == 1U);
    assert(NEAR(estimator_state.pos_ned[0], 1.0f, 1e-6f));
    assert(NEAR(estimator_state.vel_ned[2], 0.3f, 1e-6f));
}

static void test_estimator_pitch_sign(void)
{
    fc_estimator_config_t estimator_config;
    fc_estimator_state_t estimator_state;
    fc_estimator_state_t estimator_state_flip;
    fc_imu_sample_t sample;
    float pitch_plus;
    float pitch_minus;

    /* Tilted accel (cos 20deg in z, -sin 20deg in x) -> non-zero accel pitch. */
    FC_Estimator_ConfigDefault(&estimator_config);
    FC_Estimator_Init(&estimator_config, &estimator_state);
    sample.timestamp_us = 0U;
    sample.sequence = 0U;
    sample.status = 0U;
    sample.accel_mps2[0] = -9.80665f * sinf(RAD(20.0f));
    sample.accel_mps2[1] = 0.0f;
    sample.accel_mps2[2] = -9.80665f * cosf(RAD(20.0f));
    sample.gyro_rad_s[0] = 0.0f;
    sample.gyro_rad_s[1] = 0.0f;
    sample.gyro_rad_s[2] = 0.0f;

    for (uint32_t i = 0U; i < 220U; i++) {
        sample.timestamp_us = (uint64_t)(i + 1U) * 2000U;
        FC_Estimator_Update(&estimator_config, &estimator_state, &sample);
    }
    pitch_plus = estimator_state.pitch_rad;

    /* Same accel but with pitch_sign = -1 -> mirrored pitch estimate. */
    FC_Estimator_ConfigDefault(&estimator_config);
    estimator_config.calibration_samples = 200U;
    estimator_config.pitch_sign = -1.0f;
    FC_Estimator_Init(&estimator_config, &estimator_state_flip);
    for (uint32_t i = 0U; i < 220U; i++) {
        sample.timestamp_us = (uint64_t)(i + 1U) * 2000U;
        FC_Estimator_Update(&estimator_config, &estimator_state_flip, &sample);
    }
    pitch_minus = estimator_state_flip.pitch_rad;

    assert(fabsf(pitch_plus) > RAD(10.0f));
    assert(fabsf(pitch_minus) > RAD(10.0f));
    assert(NEAR(pitch_plus, -pitch_minus, 1e-3f));
}

/* An accelerometer measures specific force, not gravity, so a vehicle holding
 * level attitude while accelerating reads a tilt: at 1.6 m/s^2 north the
 * reading claims ~9.3 deg of nose-up pitch that is not there. Correcting toward
 * it walks the estimate the wrong way, which is what made LOITER/RTL diverge.
 * accel_gate_frac has to stop the correction instead of merely slowing it down
 * -- a slow pull still ends at the same wrong answer.
 *
 * The profile matches flight: calibrate sitting still and level, then pull in
 * the horizontal load with the gyro reading zero. */
static void estimator_accel_gate_fly(fc_estimator_state_t *state, float accel_n)
{
    fc_estimator_config_t config;
    fc_imu_sample_t sample;
    uint64_t timestamp = 2000U;

    FC_Estimator_ConfigDefault(&config);
    config.accel_gate_frac = accel_n > 0.0f ? 0.12f : 0.3f;
    FC_Estimator_Init(&config, state);
    memset(&sample, 0, sizeof(sample));
    sample.mag_valid = 0U;
    sample.accel_mps2[2] = -9.80665f;
    for (uint32_t i = 0U; i < 300U; i++) {          /* at rest, level */
        sample.timestamp_us = timestamp;
        sample.sequence = i;
        FC_Estimator_Update(&config, state, &sample);
        timestamp += 2000U;
    }
    sample.accel_mps2[0] = accel_n;                 /* pull in the load */
    for (uint32_t i = 0U; i < 4000U; i++) {
        sample.timestamp_us = timestamp;
        sample.sequence = 300U + i;
        FC_Estimator_Update(&config, state, &sample);
        timestamp += 2000U;
    }
}

static void test_estimator_accel_gate(void)
{
    fc_estimator_state_t gated;
    fc_estimator_state_t ungated;
    float accel_pitch_reading = atan2f(-1.6f, 9.80665f);

    /* The reading really does claim a large tilt that is not there. */
    assert(fabsf(accel_pitch_reading) > RAD(9.0f));

    /* Default gate: the load is above the threshold, so the estimate holds
     * level on the gyro instead of being walked to the bogus reading. */
    estimator_accel_gate_fly(&gated, 1.6f);
    assert(fabsf(gated.pitch_rad) < RAD(1.0f));
    assert(fabsf(gated.roll_rad) < RAD(1.0f));

    /* Wide gate: the correction stays live and the estimate ends up at the
     * biased reading, which is the runaway the gate exists to prevent. */
    estimator_accel_gate_fly(&ungated, 0.0f);
    assert(fabsf(ungated.pitch_rad) < RAD(1.0f));

    /* At rest the reading is real, so the default gate must not weaken it: a
     * genuine 20 deg bank still converges. */
    {
        fc_estimator_config_t config;
        fc_estimator_state_t state;
        fc_imu_sample_t sample;
        uint64_t timestamp = 2000U;

        FC_Estimator_ConfigDefault(&config);
        FC_Estimator_Init(&config, &state);
        memset(&sample, 0, sizeof(sample));
        sample.mag_valid = 0U;
        sample.accel_mps2[0] = -9.80665f * sinf(RAD(20.0f));
        sample.accel_mps2[1] = 0.0f;
        sample.accel_mps2[2] = -9.80665f * cosf(RAD(20.0f));
        for (uint32_t i = 0U; i < 4000U; i++) {
            sample.timestamp_us = timestamp;
            sample.sequence = i;
            FC_Estimator_Update(&config, &state, &sample);
            timestamp += 2000U;
        }
        assert(fabsf(state.pitch_rad - RAD(20.0f)) < RAD(1.0f));
    }
}

/* Regression for the LOITER/RTL divergence, built from the recorded flight
 * gazebo_sim/logs/flight-20260928-232522.csv.
 *
 * Over the 114 s of sustained turn that log holds mean true roll -14.07 deg
 * while the accelerometer claims +13.12 deg. The two are not just different,
 * they have opposite sign: in a coordinated turn the specific force is
 * dominated by the centripetal term, so the accelerometer-derived "tilt" is
 * not a tilt reference at all. The old code still walked the estimate toward
 * that reading, and because its gate was a hard cutoff the correction then
 * latched off and never came back, freezing the estimate at +3.82 deg for the
 * rest of the flight. */
static void test_estimator_turn_holds_gyro(void)
{
    fc_estimator_config_t config;
    fc_estimator_state_t state;
    fc_imu_sample_t sample;
    uint64_t timestamp = 2000U;
    const float g = 9.80665f;
    const float bank = RAD(14.0f);
    const float pos[3] = {0.0f, 0.0f, -3.0f};

    FC_Estimator_ConfigDefault(&config);
    FC_Estimator_Init(&config, &state);
    memset(&sample, 0, sizeof(sample));
    sample.mag_valid = 0U;
    sample.accel_mps2[2] = -g;

    for (uint32_t i = 0U; i < 300U; i++) {          /* level and quiescent */
        sample.timestamp_us = timestamp;
        sample.sequence = i;
        FC_Estimator_Update(&config, &state, &sample);
        timestamp += 2000U;
    }

    /* Roll into the bank over one second. The gyro is the only source that
     * knows about this, because the accelerometer is already lying. */
    for (uint32_t i = 0U; i < 500U; i++) {
        sample.timestamp_us = timestamp;
        sample.sequence = 300U + i;
        sample.gyro_rad_s[0] = -bank;
        sample.accel_mps2[1] = 2.5f;                /* cornering load, wrong sign */
        sample.accel_mps2[2] = -9.47f;
        FC_Estimator_Update(&config, &state, &sample);
        timestamp += 2000U;
    }
    sample.gyro_rad_s[0] = 0.0f;

    FC_Estimator_FeedPosition(&state, pos, (float[3]){0.0f, 1.5f, 0.0f});
    for (uint32_t i = 0U; i < 5000U; i++) {         /* hold the bank in the turn */
        sample.timestamp_us = timestamp;
        sample.sequence = 800U + i;
        FC_Estimator_Update(&config, &state, &sample);
        timestamp += 2000U;
    }

    /* The accelerometer is still claiming the opposite bank, so following it
     * is exactly the failure being guarded against. */
    assert(state.at_rest == 0U);
    assert((state.flags & FC_ESTIMATOR_FLAG_AT_REST) == 0U);
    assert(fabsf(state.roll_rad + bank) < RAD(2.0f));

    /* Come to a stop on the bank. Now the accelerometer really does read the
     * tilt, the vehicle is quiescent, and the correction has to come back --
     * the old gate could not do this once it had closed. */
    sample.accel_mps2[1] = -g * sinf(bank);
    sample.accel_mps2[2] = -g * cosf(bank);
    FC_Estimator_FeedPosition(&state, pos, (float[3]){0.0f, 0.0f, 0.0f});
    for (uint32_t i = 0U; i < 500U; i++) {
        sample.timestamp_us = timestamp;
        sample.sequence = 5800U + i;
        FC_Estimator_Update(&config, &state, &sample);
        timestamp += 2000U;
    }
    assert(state.at_rest == 1U);
    assert((state.flags & FC_ESTIMATOR_FLAG_AT_REST) != 0U);
    assert(fabsf(state.roll_rad + bank) < RAD(1.0f));
}

/* Quiescence has to be dropped the moment the vehicle moves, otherwise the
 * at-rest corrections run through the manoeuvre they exist to be suspended
 * for. */
static void test_estimator_rest_gate_reopens(void)
{
    fc_estimator_config_t config;
    fc_estimator_state_t state;
    fc_imu_sample_t sample;
    uint64_t timestamp = 2000U;
    const float g = 9.80665f;

    FC_Estimator_ConfigDefault(&config);
    FC_Estimator_Init(&config, &state);
    memset(&sample, 0, sizeof(sample));
    sample.mag_valid = 0U;
    sample.accel_mps2[2] = -g;
    for (uint32_t i = 0U; i < 300U; i++) {
        sample.timestamp_us = timestamp;
        sample.sequence = i;
        FC_Estimator_Update(&config, &state, &sample);
        timestamp += 2000U;
    }
    assert(state.at_rest == 1U);

    /* Same quiet gyroscope and the same gravity reading, but now the vehicle
     * is translating. Only the speed term can catch this, and it has to. */
    FC_Estimator_FeedPosition(&state, (float[3]){0.0f, 0.0f, 0.0f}, (float[3]){0.0f, 3.0f, 0.0f});
    for (uint32_t i = 0U; i < 200U; i++) {
        sample.timestamp_us = timestamp;
        sample.sequence = 300U + i;
        FC_Estimator_Update(&config, &state, &sample);
        timestamp += 2000U;
    }
    assert(state.at_rest == 0U);
    assert(state.rest_count == 0U);

    /* And it has to re-arm on its own once the vehicle settles again. */
    FC_Estimator_FeedPosition(&state, (float[3]){0.0f, 0.0f, 0.0f}, (float[3]){0.0f, 0.0f, 0.0f});
    for (uint32_t i = 0U; i < 200U; i++) {
        sample.timestamp_us = timestamp;
        sample.sequence = 500U + i;
        FC_Estimator_Update(&config, &state, &sample);
        timestamp += 2000U;
    }
    assert(state.at_rest == 1U);
}

/* The gyro bias is a filter state, not a value frozen at arm time. While the
 * vehicle is quiescent the gyro is reading bias and nothing else, so a bias
 * that appears after startup has to be absorbed. */
static void test_estimator_gyro_bias_online(void)
{
    fc_estimator_config_t config;
    fc_estimator_state_t state;
    fc_imu_sample_t sample;
    uint64_t timestamp = 2000U;
    const float g = 9.80665f;
    const float bias = 0.02f;   /* rad/s, about 1.1 deg/s */

    FC_Estimator_ConfigDefault(&config);
    FC_Estimator_Init(&config, &state);
    memset(&sample, 0, sizeof(sample));
    sample.mag_valid = 0U;
    sample.accel_mps2[2] = -g;
    for (uint32_t i = 0U; i < 300U; i++) {
        sample.timestamp_us = timestamp;
        sample.sequence = i;
        FC_Estimator_Update(&config, &state, &sample);
        timestamp += 2000U;
    }
    assert(fabsf(config.gyro_bias_rad_s[0]) < 1.0e-4f);

    sample.gyro_rad_s[0] = bias;                    /* bias appears while parked */
    for (uint32_t i = 0U; i < 3000U; i++) {
        sample.timestamp_us = timestamp;
        sample.sequence = 300U + i;
        FC_Estimator_Update(&config, &state, &sample);
        timestamp += 2000U;
    }
    assert(fabsf(config.gyro_bias_rad_s[0] - bias) < 0.002f);

    /* With the bias absorbed, a 30 s coast must not walk the tilt. This is the
     * failure the missing bias state would have produced on real hardware. */
    sample.gyro_rad_s[0] = bias;
    for (uint32_t i = 0U; i < 15000U; i++) {
        sample.timestamp_us = timestamp;
        sample.sequence = 3300U + i;
        FC_Estimator_Update(&config, &state, &sample);
        timestamp += 2000U;
    }
    assert(fabsf(state.roll_rad) < RAD(2.0f));
}

/* --- ICM-42688 SPI port -------------------------------------------------- */
/* Fake bus that behaves like the part: a soft reset write clears the config
 * registers, leaves the reset bit set for a few polls while RESET_DONE_INT is
 * still clear, and only then reports itself finished. That is what the driver
 * has to wait for, and it is invisible to a test that only counts delays. */
#define FAKE_REG_COUNT     256U
#define FAKE_LOG_COUNT     64U
#define FAKE_WHO_AM_I_ADDR 0x75U
#define FAKE_CFG_ADDR      0x11U
#define FAKE_STATUS_ADDR   0x2DU
#define FAKE_BANK_ADDR     0x2EU
#define FAKE_PWR_ADDR      0x4EU
#define FAKE_GYRO_CFG_ADDR 0x4FU
#define FAKE_ACCEL_CFG_ADDR 0x50U
#define FAKE_DATA_ADDR     0x1FU
#define FAKE_RESET_DONE    0x10U

typedef struct {
    uint8_t regs[FAKE_REG_COUNT];
    uint8_t log_addr[FAKE_LOG_COUNT];
    uint8_t log_val[FAKE_LOG_COUNT];
    uint32_t log_len;
    uint32_t reset_polls_left;
    uint32_t resets_seen;
    uint32_t total_delay_ms;
    uint8_t who_am_i;
    bool reject_first_reset;
} fake_bus_t;

static int32_t fake_find(const fake_bus_t *bus, uint8_t addr, uint8_t value)
{
    for (uint32_t i = 0U; i < bus->log_len; i++) {
        if ((bus->log_addr[i] == addr) && (bus->log_val[i] == value)) {
            return (int32_t)i;
        }
    }
    return -1;
}

static int32_t fake_find_addr(const fake_bus_t *bus, uint8_t addr)
{
    for (uint32_t i = 0U; i < bus->log_len; i++) {
        if (bus->log_addr[i] == addr) {
            return (int32_t)i;
        }
    }
    return -1;
}

static bool fake_transfer(void *context, const uint8_t *tx, uint8_t *rx, size_t length)
{
    fake_bus_t *bus = (fake_bus_t *)context;
    uint8_t addr = (uint8_t)(tx[0] & 0x7FU);
    bool reading = (tx[0] & 0x80U) != 0U;

    for (size_t i = 0U; i < length; i++) {
        rx[i] = 0U;
    }
    if (bus->log_len < FAKE_LOG_COUNT) {
        bus->log_addr[bus->log_len] = addr;
        bus->log_val[bus->log_len] = reading ? 0xEEU : (uint8_t)(tx[1] & 0xFFU);
        bus->log_len++;
    }
    if (length < 2U) {
        return true;
    }

    if (reading) {
        /* SPI is full duplex: after the address every clock shifts out the next
         * register, so a burst from 0x1F returns 0x1F, 0x20, 0x21, ... */
        for (size_t i = 1U; i < length; i++) {
            rx[i] = bus->regs[(uint8_t)(addr + (i - 1U))];
        }
        return true;
    }

    bus->regs[addr] = tx[1];
    if ((addr == FAKE_CFG_ADDR) && ((tx[1] & 0x01U) != 0U)) {
        bus->resets_seen++;
        /* WHO_AM_I is read-only and survives a reset, so it answers from the
         * first poll even while the reset itself is still running. */
        bus->regs[FAKE_WHO_AM_I_ADDR] = bus->who_am_i;
        bus->regs[FAKE_CFG_ADDR] = 0x01U;
        bus->reset_polls_left = (bus->reject_first_reset && (bus->resets_seen == 1U)) ?
            0xFFFFFFFFU : 3U;
    }
    return true;
}

static void fake_chip_select(void *context, bool active)
{
    (void)context;
    (void)active;
}

static void fake_delay(void *context, uint32_t delay_ms)
{
    fake_bus_t *bus = (fake_bus_t *)context;
    bus->total_delay_ms += delay_ms;
    /* Each millisecond of "chip time" lets the reset progress one poll. */
    if (bus->reset_polls_left > 0U) {
        bus->reset_polls_left--;
        if (bus->reset_polls_left == 0U) {
            bus->regs[FAKE_CFG_ADDR] = 0x00U;
            bus->regs[FAKE_STATUS_ADDR] = FAKE_RESET_DONE;
        }
    }
}

static uint64_t fake_timestamp(void *context)
{
    (void)context;
    return 12345U;
}

static void fake_bus_setup(fake_bus_t *bus, imu_icm42688_t *device)
{
    const imu_spi_bus_t spi = {
        fake_transfer, fake_chip_select, fake_delay, fake_timestamp, NULL};

    memset(bus, 0, sizeof(*bus));
    bus->who_am_i = 0x47U;
    ICM42688_ConfigDefault(device, &spi);
    device->bus.context = bus;
}

static void test_imu_spi_init_sequence(void)
{
    fake_bus_t bus;
    imu_icm42688_t device;
    int32_t cfg;
    int32_t status;
    int32_t pwr;
    int32_t gyro_cfg;
    int32_t accel_cfg;

    fake_bus_setup(&bus, &device);
    assert(!device.ready);
    assert(ICM42688_Init(&device));
    assert(device.ready);
    assert(device.who_am_i == 0x47U);
    assert(ICM42688_Healthy(&device));

    /* The soft reset has to be requested, then confirmed by the part rather
     * than assumed after a fixed delay. */
    cfg = fake_find(&bus, FAKE_CFG_ADDR, 0x01U);
    assert(cfg >= 0);
    status = fake_find_addr(&bus, FAKE_STATUS_ADDR);
    assert(status >= 0);
    assert(status > cfg);
    /* The part needed several polls, so a single check would have raced it. */
    assert(bus.total_delay_ms > 2U);

    /* PWR_MGMT0 must be written before the ODR/FS registers, and after reset. */
    pwr = fake_find(&bus, FAKE_PWR_ADDR, 0x0FU);
    gyro_cfg = fake_find(&bus, FAKE_GYRO_CFG_ADDR, 0x46U);
    accel_cfg = fake_find(&bus, FAKE_ACCEL_CFG_ADDR, 0x46U);
    assert(pwr > cfg);
    assert(gyro_cfg > pwr);
    assert(accel_cfg > pwr);
    /* The gyro needs time to start before its ODR is trusted. */
    assert(bus.total_delay_ms > 40U);
}

static void test_imu_spi_reset_retry(void)
{
    fake_bus_t bus;
    imu_icm42688_t device;

    fake_bus_setup(&bus, &device);
    bus.reject_first_reset = true;
    assert(ICM42688_Init(&device));
    assert(bus.resets_seen >= 2U);
    assert(device.ready);
}

static void test_imu_spi_wrong_part(void)
{
    fake_bus_t bus;
    imu_icm42688_t device;

    fake_bus_setup(&bus, &device);
    bus.who_am_i = 0x11U;   /* not an ICM-42688 */
    assert(!ICM42688_Init(&device));
    assert(!device.ready);
    assert(!ICM42688_Healthy(&device));
    /* Retrying cannot fix the wrong part, so it must not spin the timeout. */
    assert(bus.resets_seen == 1U);
    assert(device.who_am_i == 0x11U);
}

/* The APEX burst is 0x1F..0x2A: accel X,Y,Z then gyro X,Y,Z, each a big-endian
 * pair of registers. 8192 LSB of +/-4 g is exactly 1 g, and 16384 LSB of
 * +/-500 dps is 250 dps. */
static void fake_set_be16(fake_bus_t *bus, uint8_t reg, uint16_t value)
{
    bus->regs[reg] = (uint8_t)(value >> 8U);
    bus->regs[(uint8_t)(reg + 1U)] = (uint8_t)(value & 0xFFU);
}

static void test_imu_spi_sample(void)
{
    fake_bus_t bus;
    imu_icm42688_t device;
    fc_imu_sample_t sample;

    fake_bus_setup(&bus, &device);
    assert(ICM42688_Init(&device));

    for (uint32_t axis = 0U; axis < 3U; axis++) {
        fake_set_be16(&bus, (uint8_t)(FAKE_DATA_ADDR + (axis * 2U)), 0U);
        fake_set_be16(&bus, (uint8_t)(FAKE_DATA_ADDR + 6U + (axis * 2U)), 0U);
    }
    fake_set_be16(&bus, (uint8_t)(FAKE_DATA_ADDR + 4U), 8192U);   /* accel z = +1 g */
    fake_set_be16(&bus, (uint8_t)(FAKE_DATA_ADDR + 10U), 16384U);  /* gyro z = 250 dps */

    assert(ICM42688_ReadLatest(&device, &sample) == IMU_PORT_OK);
    /* FRD: still sitting still, so z down is +1 g and the rest are zero. */
    assert(NEAR(sample.accel_mps2[0], 0.0f, 1e-4f));
    assert(NEAR(sample.accel_mps2[1], 0.0f, 1e-4f));
    assert(NEAR(sample.accel_mps2[2], 9.80665f, 1e-2f));
    assert(NEAR(sample.gyro_rad_s[0], 0.0f, 1e-6f));
    assert(NEAR(sample.gyro_rad_s[1], 0.0f, 1e-6f));
    assert(NEAR(sample.gyro_rad_s[2], 250.0f * 0.0174532925f, 1e-3f));
    assert(sample.sequence == 1U);
    assert(sample.timestamp_us == 12345U);
    /* Reading again must not resample from a stale buffer. */
    assert(ICM42688_ReadLatest(&device, &sample) == IMU_PORT_OK);
    assert(sample.sequence == 2U);

    /* An uninitialised device reports NOT_READY rather than garbage. */
    {
        imu_icm42688_t fresh;
        imu_spi_bus_t spi = {fake_transfer, fake_chip_select, fake_delay, fake_timestamp, NULL};
        ICM42688_ConfigDefault(&fresh, &spi);
        fresh.bus.context = &bus;
        assert(ICM42688_ReadLatest(&fresh, &sample) == IMU_PORT_NOT_READY);
    }
}

static void test_imu_spi_dual(void)
{
    fake_bus_t bus0;
    fake_bus_t bus1;
    imu_icm42688_t devices[2];
    imu_port_spi_t port;
    imu_port_t interface;
    fc_imu_sample_t sample;
    const imu_spi_bus_t spi0 = {fake_transfer, fake_chip_select, fake_delay,
                                 fake_timestamp, NULL};
    const imu_spi_bus_t spi1 = {fake_transfer, fake_chip_select, fake_delay,
                                 fake_timestamp, NULL};

    memset(&bus0, 0, sizeof(bus0));
    memset(&bus1, 0, sizeof(bus1));
    bus0.who_am_i = 0x47U;
    bus1.who_am_i = 0x47U;
    ICM42688_ConfigDefault(&devices[0], &spi0);
    devices[0].bus.context = &bus0;
    ICM42688_ConfigDefault(&devices[1], &spi1);
    devices[1].bus.context = &bus1;

    IMU_Port_Spi_Init(&port, devices, 2U);
    IMU_Port_Spi_AsPort(&port, &interface);
    assert(port.device_count == 2U);
    assert(interface.init != NULL);
    assert(interface.read_latest != NULL);
    assert(interface.healthy != NULL);
    /* The port contract is fn(port->context), as flight_controller uses. */
    assert(interface.context == &port);
    assert(!interface.healthy(interface.context));

    /* IMU0 answers with the wrong part ID while IMU1 is fine. The primary is
     * IMU0, so init must fail - there is nothing safe to fuse - even though
     * healthy() sees that one of the pair works. */
    bus0.who_am_i = 0x11U;
    assert(!interface.init(interface.context));
    assert(interface.healthy(interface.context));
    assert(devices[0].ready == false);
    assert(devices[1].ready == true);

    /* Reads still work off the surviving IMU: the two parts are identical and
     * on the same airframe, so the spare is a genuine redundant source. */
    assert(interface.read_latest(interface.context, &sample) == IMU_PORT_OK);
    assert(sample.sequence == devices[1].sequence);
    assert(devices[1].sequence >= 1U);

    /* With a healthy primary the read comes from IMU0 and nothing else. */
    bus0.who_am_i = 0x47U;
    assert(interface.init(interface.context));
    assert(devices[0].ready);
    assert(devices[1].ready);
    fake_set_be16(&bus0, (uint8_t)(FAKE_DATA_ADDR + 4U), 8192U);
    assert(interface.read_latest(interface.context, &sample) == IMU_PORT_OK);
    assert(NEAR(sample.accel_mps2[2], 9.80665f, 1e-2f));
    assert(sample.sequence == devices[0].sequence);
}

static void test_attitude_defaults(void)
{
    fc_attitude_config_t cfg;
    FC_Attitude_ConfigDefault(&cfg);
    assert(NEAR(cfg.gain_att[0], 4.0f, 1e-4f));
    assert(NEAR(cfg.gain_att[1], 4.0f, 1e-4f));
    assert(NEAR(cfg.gain_att[2], 2.8f / 0.4f, 1e-4f));
    assert(NEAR(cfg.yaw_w, 0.4f, 1e-4f));
    assert(NEAR(cfg.lim_rate[0], RAD(220.0f), 1e-4f));
    assert(NEAR(cfg.lim_rate[2], RAD(200.0f), 1e-4f));
    assert(cfg.initialized == 1U);
}

static void test_attitude_level(void)
{
    fc_attitude_config_t cfg;
    fc_estimator_state_t state;
    float rate_sp[3];
    float attitude_sp[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    FC_Attitude_ConfigDefault(&cfg);
    memset(&state, 0, sizeof(state));
    state.flags = FC_ESTIMATOR_FLAG_INITIALIZED;
    state.q_w = 1.0f;
    state.q_x = 0.0f;
    state.q_y = 0.0f;
    state.q_z = 0.0f;
    state.gyro_rad_s[0] = 0.0f;
    state.gyro_rad_s[1] = 0.0f;
    state.gyro_rad_s[2] = 0.0f;

    assert(FC_Attitude_Update(&cfg, &state, attitude_sp, 0.0f, rate_sp));
    assert(fabsf(rate_sp[0]) < 1e-3f);
    assert(fabsf(rate_sp[1]) < 1e-3f);
    assert(fabsf(rate_sp[2]) < 1e-3f);
}

static void test_attitude_roll_error(void)
{
    fc_attitude_config_t cfg;
    fc_estimator_state_t state;
    float rate_sp[3];
    float attitude_sp[4] = {1.0f, 0.0f, 0.0f, 0.0f};

    FC_Attitude_ConfigDefault(&cfg);
    memset(&state, 0, sizeof(state));
    state.flags = FC_ESTIMATOR_FLAG_INITIALIZED;
    state.q_w = cosf(RAD(8.0f) * 0.5f);
    state.q_x = sinf(RAD(8.0f) * 0.5f);
    state.q_y = 0.0f;
    state.q_z = 0.0f;
    state.gyro_rad_s[0] = 0.0f;
    state.gyro_rad_s[1] = 0.0f;
    state.gyro_rad_s[2] = 0.0f;

    assert(FC_Attitude_Update(&cfg, &state, attitude_sp, 0.0f, rate_sp));
    assert(rate_sp[0] < -1e-3f);
    assert(fabsf(rate_sp[1]) < 1e-3f);
}

static void test_rate_defaults(void)
{
    fc_rate_config_t cfg;
    FC_RateControl_ConfigDefault(&cfg);
    assert(NEAR(cfg.gain_rate_p[0], 0.15f, 1e-4f));
    assert(NEAR(cfg.gain_rate_i[0], 0.2f, 1e-4f));
    assert(NEAR(cfg.gain_rate_d[0], 0.003f, 1e-4f));
    assert(NEAR(cfg.gain_rate_ff[0], 0.0f, 1e-6f));
    assert(NEAR(cfg.gain_rate_k[0], 1.0f, 1e-6f));
    assert(NEAR(cfg.gain_rate_p[2], 0.2f, 1e-4f));
    assert(NEAR(cfg.gain_rate_i[2], 0.1f, 1e-4f));
    assert(NEAR(cfg.lim_rate_int[0], 0.3f, 1e-4f));
    assert(NEAR(cfg.yaw_tq_cutoff, 2.0f, 1e-6f));
    assert(cfg.yaw_lpf_enabled == 1U);
    assert(cfg.initialized == 1U);
}

static void test_rate_level(void)
{
    fc_rate_config_t cfg;
    float rate_sp[3] = {0.0f, 0.0f, 0.0f};
    float rate[3] = {0.0f, 0.0f, 0.0f};
    float torque[3];

    FC_RateControl_ConfigDefault(&cfg);
    assert(FC_RateControl_Update(&cfg, rate_sp, rate, 0.002f, false, torque));
    assert(fabsf(torque[0]) < 1e-6f);
    assert(fabsf(torque[1]) < 1e-6f);
    assert(fabsf(torque[2]) < 1e-6f);
}

static void test_rate_error(void)
{
    fc_rate_config_t cfg;
    float rate_sp[3] = {0.5f, 0.0f, 0.0f};
    float rate[3] = {0.0f, 0.0f, 0.0f};
    float torque[3];

    FC_RateControl_ConfigDefault(&cfg);
    assert(FC_RateControl_Update(&cfg, rate_sp, rate, 0.002f, false, torque));
    assert(torque[0] > 1e-4f);
    assert(fabsf(torque[1]) < 1e-4f);
    assert(fabsf(torque[2]) < 1e-4f);

    /* integral builds only when not landed */
    FC_RateControl_Reset(&cfg);
    float rate0[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 100; i++) {
        assert(FC_RateControl_Update(&cfg, rate_sp, rate0, 0.002f, false, torque));
    }
    assert(cfg.rate_int[0] > 0.0f);
    FC_RateControl_Reset(&cfg);
    for (int i = 0; i < 100; i++) {
        assert(FC_RateControl_Update(&cfg, rate_sp, rate0, 0.002f, true, torque));
    }
    assert(fabsf(cfg.rate_int[0]) < 1e-6f);
}

static void test_mixer_allocation(void)
{
    fc_mixer_config_t cfg;
    fc_torque_cmd_t torque;
    fc_motor_output_t output;

    /* Phase F: mixer is a PX4 control_allocator port (CA_* geometry). The
     * authority is normalized so a unit torque swings a motor ~0.707
     * (roll/pitch) / ~1.0 (yaw), and thrust_norm maps 1:1 onto every motor. */

    FC_Mixer_ConfigDefault(&cfg);
    cfg.max_slew_norm = 0.0f;
    torque.roll_torque = 0.0f;
    torque.pitch_torque = 0.0f;
    torque.yaw_torque = 0.0f;
    assert(FC_Mixer_Compute(&cfg, &torque, 0.5f, true, &output));
    for (uint32_t i = 0U; i < 4U; i++) {
        assert(NEAR(output.normalized[i], 0.5f, 1e-4f));
    }

    /* mid-range torque: no desaturation, checks the 0.707 authority value */
    FC_Mixer_ConfigDefault(&cfg);
    cfg.max_slew_norm = 0.0f;
    torque.roll_torque = 0.2f;
    torque.pitch_torque = 0.0f;
    torque.yaw_torque = 0.0f;
    assert(FC_Mixer_Compute(&cfg, &torque, 0.5f, true, &output));
    assert(NEAR(output.normalized[0], 0.5f - 0.2f * 0.707f, 1e-3f));
    assert(NEAR(output.normalized[1], 0.5f + 0.2f * 0.707f, 1e-3f));
    assert(NEAR(output.normalized[2], 0.5f + 0.2f * 0.707f, 1e-3f));
    assert(NEAR(output.normalized[3], 0.5f - 0.2f * 0.707f, 1e-3f));

    /* full roll: desaturation keeps motors in [0,1] at thrust 0.5 */
    FC_Mixer_ConfigDefault(&cfg);
    cfg.max_slew_norm = 0.0f;
    torque.roll_torque = 1.0f;
    torque.pitch_torque = 0.0f;
    torque.yaw_torque = 0.0f;
    assert(FC_Mixer_Compute(&cfg, &torque, 0.5f, true, &output));
    assert(output.normalized[0] < 0.05f);
    assert(output.normalized[1] > 0.95f);
    assert(output.normalized[2] > 0.95f);
    assert(output.normalized[3] < 0.05f);

    /* full pitch (FRD nose-up): geometry maps it to the x+ motors [0],[2] */
    FC_Mixer_ConfigDefault(&cfg);
    cfg.max_slew_norm = 0.0f;
    torque.roll_torque = 0.0f;
    torque.pitch_torque = 1.0f;
    torque.yaw_torque = 0.0f;
    assert(FC_Mixer_Compute(&cfg, &torque, 0.5f, true, &output));
    assert(output.normalized[0] > 0.95f);
    assert(output.normalized[1] < 0.05f);
    assert(output.normalized[2] > 0.95f);
    assert(output.normalized[3] < 0.05f);

    /* full yaw: +yaw torque (KM + on the ccw rotors 0,1) speeds up 0,1 */
    FC_Mixer_ConfigDefault(&cfg);
    cfg.max_slew_norm = 0.0f;
    torque.roll_torque = 0.0f;
    torque.pitch_torque = 0.0f;
    torque.yaw_torque = 1.0f;
    assert(FC_Mixer_Compute(&cfg, &torque, 0.5f, true, &output));
    assert(output.normalized[0] > 0.95f);
    assert(output.normalized[1] > 0.95f);
    assert(output.normalized[2] < 0.05f);
    assert(output.normalized[3] < 0.05f);

    /* flipping KM (spin direction) flips the yaw authority, as in PX4 */
    cfg.ca_rotor_km[0] = -0.05f;
    cfg.ca_rotor_km[1] = -0.05f;
    cfg.ca_rotor_km[2] = 0.05f;
    cfg.ca_rotor_km[3] = 0.05f;
    memset(cfg.previous_normalized, 0, sizeof(cfg.previous_normalized));
    assert(FC_Mixer_Compute(&cfg, &torque, 0.5f, true, &output));
    assert(output.normalized[0] < 0.05f);
    assert(output.normalized[1] < 0.05f);
    assert(output.normalized[2] > 0.95f);
    assert(output.normalized[3] > 0.95f);

    /* geometry has to be finite; a zero CT rotor is rejected */
    cfg.ca_rotor_ct[0] = 0.0f;
    memset(cfg.previous_normalized, 0, sizeof(cfg.previous_normalized));
    assert(!FC_Mixer_Compute(&cfg, &torque, 0.5f, true, &output));

    assert(FC_Mixer_Compute(&cfg, &torque, 0.5f, false, &output));
    assert(output.armed == false);
    for (uint32_t i = 0U; i < 4U; i++) {
        assert(NEAR(output.normalized[i], 0.0f, 1e-6f));
    }
}

static void test_position_defaults(void)
{
    fc_position_config_t cfg;
    FC_Position_ConfigDefault(&cfg);
    assert(NEAR(cfg.gain_pos[0], 0.95f, 1e-4f));
    assert(NEAR(cfg.gain_pos[2], 1.0f, 1e-4f));
    assert(NEAR(cfg.gain_vel_p[0], 1.8f, 1e-4f));
    assert(NEAR(cfg.gain_vel_p[2], 4.0f, 1e-4f));
    assert(NEAR(cfg.gain_vel_i[0], 0.4f, 1e-4f));
    assert(NEAR(cfg.gain_vel_i[2], 2.0f, 1e-4f));
    assert(NEAR(cfg.gain_vel_d[0], 0.2f, 1e-4f));
    assert(NEAR(cfg.gain_vel_d[2], 0.0f, 1e-6f));
    assert(NEAR(cfg.lim_vel_h, 12.0f, 1e-4f));
    assert(NEAR(cfg.lim_vel_up, 3.0f, 1e-4f));
    assert(NEAR(cfg.lim_vel_down, 1.5f, 1e-4f));
    assert(NEAR(cfg.hover_thrust, 0.5f, 1e-4f));
    assert(NEAR(cfg.lim_tilt, RAD(45.0f), 1e-4f));
    assert(NEAR(cfg.lim_acc_h, 3.0f, 1e-4f));
    assert(cfg.z_up == 0U);
}

static void test_position_zup_hover(void)
{
    fc_position_config_t cfg;
    fc_estimator_state_t state;
    fc_position_setpoint_t sp;
    fc_position_out_t out;

    FC_Position_ConfigDefault(&cfg);
    cfg.z_up = 1U;
    memset(&state, 0, sizeof(state));
    memset(&sp, 0, sizeof(sp));
    sp.position_ned[2] = 3.0f;
    state.pos_ned[2] = 3.0f;
    state.pos_valid = 1U;
    state.accel_mps2[2] = 9.80665f;

    assert(FC_Position_Update(&cfg, &state, &sp, 0.002f, &out));
    assert(out.valid == 1U);
    assert(NEAR(out.thrust_ned[2], cfg.hover_thrust, 1e-3f));
    assert(fabsf(out.attitude_sp[0] - 1.0f) < 1e-3f);
    assert(fabsf(out.attitude_sp[1]) < 1e-3f);
    assert(fabsf(out.attitude_sp[2]) < 1e-3f);
}

static void test_position_zup_hold(void)
{
    fc_position_config_t cfg;
    fc_estimator_state_t state;
    fc_position_setpoint_t sp;
    fc_position_out_t out;

    FC_Position_ConfigDefault(&cfg);
    cfg.z_up = 1U;
    memset(&state, 0, sizeof(state));
    memset(&sp, 0, sizeof(sp));
    sp.position_ned[0] = 1.0f;
    sp.position_ned[2] = 3.0f;
    state.pos_ned[0] = 0.0f;
    state.pos_ned[2] = 3.0f;
    state.pos_valid = 1U;
    state.accel_mps2[2] = 9.80665f;

    assert(FC_Position_Update(&cfg, &state, &sp, 0.002f, &out));
    assert(out.thrust_ned[0] > 0.05f);
    assert(out.attitude_sp[2] > 0.01f);
    assert(out.valid == 1U);
}

static void test_position_ned_hover(void)
{
    fc_position_config_t cfg;
    fc_estimator_state_t state;
    fc_position_setpoint_t sp;
    fc_position_out_t out;

    FC_Position_ConfigDefault(&cfg);
    cfg.z_up = 0U;
    memset(&state, 0, sizeof(state));
    memset(&sp, 0, sizeof(sp));
    sp.position_ned[2] = 3.0f;
    state.pos_ned[2] = 3.0f;
    state.pos_valid = 1U;
    state.accel_mps2[2] = 9.80665f;

    assert(FC_Position_Update(&cfg, &state, &sp, 0.002f, &out));
    assert(out.thrust_ned[2] < -cfg.hover_thrust + (1e-3f));
    assert(fabsf(out.thrust_ned[2] + cfg.hover_thrust) < 1e-3f);
    assert(fabsf(out.attitude_sp[0] - 1.0f) < 1e-3f);
    assert(out.valid == 1U);
}

static void test_position_acc_horizontal_limit(void)
{
    fc_position_config_t cfg;
    fc_estimator_state_t state;
    fc_position_setpoint_t sp;
    fc_position_out_t out;
    float axy;

    FC_Position_ConfigDefault(&cfg);
    cfg.z_up = 0U;
    memset(&state, 0, sizeof(state));
    memset(&sp, 0, sizeof(sp));
    /* 10 m cross-track error with zero velocity -> demand way past the limit. */
    sp.position_ned[0] = 10.0f;
    sp.position_ned[2] = 3.0f;
    state.pos_ned[0] = 0.0f;
    state.pos_ned[2] = 3.0f;
    state.vel_ned[0] = 0.0f;
    state.accel_mps2[2] = 9.80665f;
    state.pos_valid = 1U;

    assert(FC_Position_Update(&cfg, &state, &sp, 0.002f, &out));
    /* the horizontal accel demand must respect MPC_ACC_HOR_MAX (tilt <= ~17 deg). */
    axy = sqrtf(out.acc_sp_ned[0] * out.acc_sp_ned[0] + out.acc_sp_ned[1] * out.acc_sp_ned[1]);
    assert(axy <= cfg.lim_acc_h + 1e-3f);

    /* without a limit the demand is far bigger -> verify the clamp matters */
    cfg.lim_acc_h = 0.0f;
    assert(FC_Position_Update(&cfg, &state, &sp, 0.002f, &out));
    axy = sqrtf(out.acc_sp_ned[0] * out.acc_sp_ned[0] + out.acc_sp_ned[1] * out.acc_sp_ned[1]);
    assert(axy > 3.0f);
    printf("test_position_acc_horizontal_limit: acc=%.2f vs lim=%.2f OK\n", axy, cfg.lim_acc_h);
}

static void test_position_vel_damping(void)
{
    /* Phase C: gain_vel_d damps measured velocity derivative (MPC_XY_VEL_D). */
    fc_position_config_t cfg;
    fc_estimator_state_t state;
    fc_position_setpoint_t sp;
    fc_position_out_t out;
    float vz_lo, vz_hi;

    FC_Position_ConfigDefault(&cfg);
    cfg.z_up = 0U;
    cfg.gain_vel_d[0] = 0.2f;
    cfg.gain_vel_d[1] = 0.2f;
    cfg.gain_vel_d[2] = 0.0f;
    memset(&state, 0, sizeof(state));
    memset(&sp, 0, sizeof(sp));
    sp.position_ned[0] = 3.0f;
    state.pos_ned[0] = 0.0f;
    state.pos_valid = 1U;
    state.accel_mps2[2] = 9.80665f;

    /* First update primes the derivative filter -> acc_sp from P alone. */
    assert(FC_Position_Update(&cfg, &state, &sp, 0.01f, &out));
    vz_lo = out.acc_sp_ned[0];

    /* Second update with forward velocity: raw accel ~ vz/dt enters vel_dot,
     * so the D term subtracts gain_vel_d * accel from acc_sp -> less thrust_xy. */
    state.vel_ned[0] = 1.0f;
    assert(FC_Position_Update(&cfg, &state, &sp, 0.01f, &out));
    vz_hi = out.acc_sp_ned[0];
    assert(vz_hi < vz_lo + (0.15f));   /* damping reduces the commanded accel */
    assert(out.valid == 1U);
}

static void test_land_defaults(void)
{
    fc_land_config_t cfg;
    FC_Land_ConfigDefault(&cfg);
    assert(NEAR(cfg.z_vel_max, 0.25f, 1e-4f));
    assert(NEAR(cfg.xy_vel_max, 1.5f, 1e-4f));
    assert(NEAR(cfg.rot_max_rad_s, RAD(20.0f), 1e-4f));
    assert(NEAR(cfg.alt_gnd_m, 1.0f, 1e-4f));
    assert(NEAR(cfg.thr_min, 0.12f, 1e-4f));
    assert(NEAR(cfg.thr_hover, 0.5f, 1e-4f));
    assert(NEAR(cfg.t_ground_s, 0.333f, 1e-4f));
    assert(NEAR(cfg.t_maybe_s, 0.333f, 1e-4f));
    assert(NEAR(cfg.t_landed_s, 0.333f, 1e-4f));
    assert(NEAR(cfg.t_freefall_s, 0.3f, 1e-4f));
    assert(cfg.initialized == 1U);
}

static void test_land_ground_to_landed(void)
{
    /* Full PX4 chain: ground_contact -> maybe_landed -> landed while sitting
       on the ground at low throttle. */
    fc_land_config_t cfg;
    fc_land_state_t state;
    fc_land_out_t out;
    const float dt = 0.001f;
    uint32_t gc_at = 0U;
    uint32_t ml_at = 0U;
    uint32_t ld_at = 0U;

    FC_Land_ConfigDefault(&cfg);
    FC_Land_Init(&cfg, &state);

    for (uint32_t i = 1U; i <= 1200U; i++) {
        assert(FC_Land_Update(&cfg, &state, 0.10f, 0.0f, 0.0f, 0.0f, 9.8f, 0.2f, dt, &out));
        if ((out.ground_contact != 0U) && (gc_at == 0U)) gc_at = i;
        if ((out.maybe_landed != 0U) && (ml_at == 0U)) ml_at = i;
        if ((out.landed != 0U) && (ld_at == 0U)) ld_at = i;
    }

    /* hysteresis chain must fire in order with distinct timestamps */
    assert(gc_at != 0U);
    assert(ml_at != 0U);
    assert(ld_at != 0U);
    assert(gc_at < ml_at);
    assert(ml_at < ld_at);
    /* gc needs ~333 iters @1ms; each subsequent stage ~333 more */
    assert(gc_at >= 300U);
    assert(gc_at <= 360U);
    assert(ml_at >= 660U);
    assert(ml_at <= 720U);
    assert(ld_at >= 990U);
    assert(ld_at <= 1070U);
    assert(out.freefall == 0U);
}

static void test_land_freefall(void)
{
    fc_land_config_t cfg;
    fc_land_state_t state;
    fc_land_out_t out;
    const float dt = 0.001f;

    FC_Land_ConfigDefault(&cfg);
    FC_Land_Init(&cfg, &state);

    for (uint32_t i = 1U; i <= 350U; i++) {
        assert(FC_Land_Update(&cfg, &state, 0.2f, 0.0f, 0.0f, 0.0f, 0.5f, 5.0f, dt, &out));
    }
    assert(out.freefall == 1U);

    /* returning to hard ground cancels freefall */
    for (uint32_t i = 1U; i <= 100U; i++) {
        assert(FC_Land_Update(&cfg, &state, 0.2f, 0.0f, 0.0f, 0.0f, 9.8f, 5.0f, dt, &out));
    }
    assert(out.freefall == 0U);
}

static void test_land_hysteresis_reset(void)
{
    /* holding our thrust back above the low-thrust band must never arm
       ground_contact, even for a long time */
    fc_land_config_t cfg;
    fc_land_state_t state;
    fc_land_out_t out;
    const float dt = 0.001f;
    uint32_t landed_seen = 0U;

    FC_Land_ConfigDefault(&cfg);
    FC_Land_Init(&cfg, &state);

    for (uint32_t i = 1U; i <= 600U; i++) {
        float thrust = (i % 150U < 75U) ? 0.10f : 0.60f;
        assert(FC_Land_Update(&cfg, &state, thrust, 0.0f, 0.0f, 0.0f, 9.8f, 0.2f, dt, &out));
        landed_seen |= out.landed;
    }
    assert(landed_seen == 0U);
}

static void test_guard_defaults(void)
{
    fc_guard_config_t cfg;
    FC_Guard_ConfigDefault(&cfg);
    assert(NEAR(cfg.stab_roll_enter_rad, RAD(25.0f), 1e-4f));
    assert(NEAR(cfg.stab_pitch_enter_rad, RAD(25.0f), 1e-4f));
    assert(NEAR(cfg.stab_rate_enter_rad_s, 2.0f, 1e-4f));
    assert(NEAR(cfg.stab_enter_t_s, 0.3f, 1e-4f));
    assert(NEAR(cfg.stab_roll_exit_rad, RAD(10.0f), 1e-4f));
    assert(NEAR(cfg.stab_pitch_exit_rad, RAD(10.0f), 1e-4f));
    assert(NEAR(cfg.stab_hold_s, 1.5f, 1e-4f));
    assert(NEAR(cfg.stab_gain, 1.6f, 1e-4f));
    assert(NEAR(cfg.stab_damp, 0.01f, 1e-4f));
    assert(NEAR(cfg.stab_rate_lim, 3.0f, 1e-4f));
    assert(NEAR(cfg.stab_thrust_hold, 0.55f, 1e-4f));
    assert(NEAR(cfg.rec_roll_enter_rad, RAD(100.0f), 1e-4f));
    assert(NEAR(cfg.rec_pitch_enter_rad, RAD(100.0f), 1e-4f));
    assert(NEAR(cfg.rec_air_m, 0.4f, 1e-4f));
    assert(NEAR(cfg.rec_enter_t_s, 0.3f, 1e-4f));
    assert(NEAR(cfg.rec_roll_exit_rad, RAD(40.0f), 1e-4f));
    assert(NEAR(cfg.rec_pitch_exit_rad, RAD(40.0f), 1e-4f));
    assert(NEAR(cfg.rec_gain, 3.0f, 1e-4f));
    assert(NEAR(cfg.rec_rate_lim, 6.28f, 1e-4f));
    assert(NEAR(cfg.rec_thrust_hold, 0.7f, 1e-4f));
    assert(NEAR(cfg.rec_timeout_s, 8.0f, 1e-4f));
    assert(cfg.initialized == 1U);
}

static void test_guard_stab(void)
{
    fc_guard_config_t gcfg;
    fc_guard_state_t gstate;
    fc_guard_out_t gout;
    fc_attitude_config_t att_cfg;
    fc_rate_config_t rate_cfg;
    const float dt = 0.002f;

    FC_Attitude_ConfigDefault(&att_cfg);
    FC_RateControl_ConfigDefault(&rate_cfg);
    FC_Guard_ConfigDefault(&gcfg);
    FC_Guard_Init(&gcfg, &gstate, &att_cfg, &rate_cfg);

    /* transient roll spike < TTRI (0.3s) must NOT trip (A.3) */
    for (int i = 0; i < 100; i++) {
        assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                               RAD(30.0f), 0.0f, 0.0f, 1.5f, dt, &gout));
    }
    assert(gout.mode == FC_GUARD_MODE_NORMAL);

    /* sustained roll beyond limit for TTRI -> STAB, rate boost applied */
    for (int i = 0; i < 50; i++) {
        assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                               RAD(30.0f), 0.0f, 0.0f, 1.5f, dt, &gout));
    }
    assert(gout.mode == FC_GUARD_MODE_STAB);
    assert(gout.mode_changed == 1U);
    assert(NEAR(gout.thrust_hold, 0.55f, 1e-4f));
    assert(NEAR(att_cfg.lim_rate[0], 3.0f, 1e-4f));
    assert(NEAR(rate_cfg.gain_rate_p[0], 0.15f * 1.6f, 1e-4f));
    assert(NEAR(rate_cfg.gain_rate_d[0], 0.01f, 1e-4f));
    assert(NEAR(rate_cfg.lim_rate_int[0], 0.16f, 1e-4f));

    /* still unstable after 0.4s -> stays STAB */
    for (int i = 0; i < 200; i++) {
        assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                               RAD(30.0f), 0.0f, 0.0f, 1.5f, dt, &gout));
    }
    assert(gout.mode == FC_GUARD_MODE_STAB);

    /* level for stab_hold_s (1.5s) -> back to NORMAL, gains restored */
    uint32_t transitioned = 0U;
    for (int i = 0; i < 760; i++) {
        assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                               RAD(5.0f), 0.0f, 0.0f, 1.5f, dt, &gout));
        if ((gout.mode == FC_GUARD_MODE_NORMAL) && (gout.mode_changed == 1U)) transitioned = 1U;
    }
    assert(transitioned == 1U);
    assert(gout.mode == FC_GUARD_MODE_NORMAL);
    assert(NEAR(att_cfg.lim_rate[0], RAD(220.0f), 1e-4f));
    assert(NEAR(rate_cfg.gain_rate_p[0], 0.15f, 1e-4f));
    assert(NEAR(rate_cfg.gain_rate_d[0], 0.003f, 1e-4f));
    assert(NEAR(rate_cfg.lim_rate_int[0], 0.3f, 1e-4f));
    assert(NEAR(gout.thrust_hold, 0.0f, 1e-6f));

    /* NORMAL is sticky as long as nothing trips */
    assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                           RAD(5.0f), 0.0f, 0.0f, 1.5f, dt, &gout));
    assert(gout.mode == FC_GUARD_MODE_NORMAL);

    /* pitch-only excursion still trips (A.1: axes checked separately) */
    FC_Guard_Reset(&gcfg, &gstate, &att_cfg, &rate_cfg);
    for (int i = 0; i < 150; i++) {
        assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                               0.0f, -RAD(30.0f), 0.0f, 1.5f, dt, &gout));
    }
    assert(gout.mode == FC_GUARD_MODE_STAB);
}

static void test_guard_recover_teleport(void)
{
    fc_guard_config_t gcfg;
    fc_guard_state_t gstate;
    fc_guard_out_t gout;
    fc_attitude_config_t att_cfg;
    fc_rate_config_t rate_cfg;
    const float dt = 0.002f;

    FC_Attitude_ConfigDefault(&att_cfg);
    FC_RateControl_ConfigDefault(&rate_cfg);
    FC_Guard_ConfigDefault(&gcfg);
    FC_Guard_Init(&gcfg, &gstate, &att_cfg, &rate_cfg);

    /* near-ground rollover sustained for TTRI -> RECOVER (160deg roll alone) */
    for (int i = 0; i < 150; i++) {
        assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                               RAD(160.0f), 0.0f, 0.0f, 0.2f, dt, &gout));
    }
    assert(gout.mode == FC_GUARD_MODE_RECOVER);
    assert(gout.mode_changed == 1U);
    assert(NEAR(gout.thrust_hold, 0.7f, 1e-4f));
    assert(NEAR(att_cfg.lim_rate[0], 6.28f, 1e-4f));
    assert(NEAR(rate_cfg.gain_rate_p[0], 0.15f * 3.0f, 1e-4f));
    assert(NEAR(rate_cfg.lim_rate_int[0], 0.3f, 1e-4f));

    /* stuck too long -> teleport requested, back to NORMAL, gains restored.
       after teleport the host resets the airframe so inputs go level+high. */
    uint32_t teleport_seen = 0U;
    uint32_t normal_change_seen = 0U;
    for (int i = 0; i < 4200; i++) {
        float tilt = RAD(160.0f);
        float alt = 0.2f;
        if (teleport_seen != 0U) {
            tilt = 0.0f;
            alt = 5.0f;
        }
        assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                               tilt, 0.0f, 0.0f, alt, dt, &gout));
        if (gout.teleport_now != 0U) teleport_seen = 1U;
        if ((gout.mode == FC_GUARD_MODE_NORMAL) && (gout.mode_changed == 1U)) normal_change_seen = 1U;
    }
    assert(teleport_seen == 1U);
    assert(normal_change_seen == 1U);
    assert(gout.mode == FC_GUARD_MODE_NORMAL);
    assert(NEAR(att_cfg.lim_rate[0], RAD(220.0f), 1e-4f));
    assert(NEAR(rate_cfg.gain_rate_p[0], 0.15f, 1e-4f));
}

static void test_guard_recover_exit(void)
{
    /* recovering airframe righted itself before timeout -> normal, no teleport */
    fc_guard_config_t gcfg;
    fc_guard_state_t gstate;
    fc_guard_out_t gout;
    fc_attitude_config_t att_cfg;
    fc_rate_config_t rate_cfg;
    const float dt = 0.002f;

    FC_Attitude_ConfigDefault(&att_cfg);
    FC_RateControl_ConfigDefault(&rate_cfg);
    FC_Guard_ConfigDefault(&gcfg);
    FC_Guard_Init(&gcfg, &gstate, &att_cfg, &rate_cfg);

    /* pitch rollover near ground -> RECOVER (pitch-only trip, A.1) */
    for (int i = 0; i < 150; i++) {
        assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                               0.0f, RAD(160.0f), 0.0f, 0.2f, dt, &gout));
    }
    assert(gout.mode == FC_GUARD_MODE_RECOVER);

    /* 0.2s more of high roll then righted below rec_roll_exit */
    for (int i = 0; i < 100; i++) {
        assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                               RAD(160.0f), 0.0f, 0.0f, 0.2f, dt, &gout));
    }
    assert(gout.mode == FC_GUARD_MODE_RECOVER);
    assert(FC_Guard_Update(&gcfg, &gstate, &att_cfg, &rate_cfg,
                           RAD(30.0f), 0.0f, 0.0f, 0.2f, dt, &gout));
    assert(gout.mode == FC_GUARD_MODE_NORMAL);
    assert(gout.teleport_now == 0U);
    assert(gout.mode_changed == 1U);
    assert(NEAR(rate_cfg.gain_rate_p[0], 0.15f, 1e-4f));
}

static void sched_count(void *ctx, uint32_t elapsed_us)
{
    uint32_t *count = (uint32_t *)ctx;
    assert(elapsed_us >= 2000U);
    (*count)++;
}

static void sched_slow(void *ctx, uint32_t elapsed_us)
{
    uint32_t *count = (uint32_t *)ctx;
    (void)elapsed_us;
    (*count)++;
}

static void test_sched(void)
{
    fc_sched_t sched;
    uint32_t fast = 0U;
    uint32_t slow = 0U;

    FC_Sched_Init(NULL);
    FC_Sched_Tick(NULL, 1000U);
    FC_Sched_Tick(&sched, 0U);

    FC_Sched_Init(&sched);
    assert(FC_Sched_Add(&sched, 2000U, sched_count, &fast));
    assert(FC_Sched_Add(&sched, 20000U, sched_slow, &slow));
    assert(!FC_Sched_Add(&sched, 0U, sched_count, &fast));
    assert(!FC_Sched_Add(&sched, 2000U, NULL, &fast));
    for (uint32_t i = 0U; i < 20U; i++) {
        FC_Sched_Tick(&sched, 2000U);
    }
    assert(fast == 20U);
    assert(slow == 2U);

    fast = 0U;
    slow = 0U;
    FC_Sched_Init(&sched);
    assert(FC_Sched_Add(&sched, 2000U, sched_count, &fast));
    assert(FC_Sched_Add(&sched, 20000U, sched_slow, &slow));
    for (uint32_t i = 0U; i < 10U; i++) {
        FC_Sched_Tick(&sched, 2000U);
    }
    assert(fast == 10U);
    assert(slow == 1U);
    printf("test_sched: OK\n");
}

static void test_orb(void)
{
    fc_orb_sensor_gyro_t gyro;
    fc_orb_sensor_gyro_t out;

    FC_Orb_Init();
    assert(FC_Orb_Valid(FC_ORB_SENSOR_GYRO) == false);
    gyro.timestamp_us = 123456U;
    gyro.gyro_rad_s[0] = 0.1f;
    gyro.gyro_rad_s[1] = -0.2f;
    gyro.gyro_rad_s[2] = 0.3f;
    assert(FC_Orb_Updated(FC_ORB_SENSOR_GYRO) == false);
    assert(FC_Orb_Publish(FC_ORB_SENSOR_GYRO, &gyro));
    assert(FC_Orb_Valid(FC_ORB_SENSOR_GYRO) == true);
    assert(FC_Orb_Updated(FC_ORB_SENSOR_GYRO) == true);
    assert(FC_Orb_Copy(FC_ORB_SENSOR_GYRO, &out));
    assert(out.timestamp_us == gyro.timestamp_us);
    assert(NEAR(out.gyro_rad_s[0], 0.1f, 1e-6f));
    assert(NEAR(out.gyro_rad_s[2], 0.3f, 1e-6f));
    assert(FC_Orb_Updated(FC_ORB_SENSOR_GYRO) == false);
    assert(FC_Orb_Valid(FC_ORB_SENSOR_GYRO) == true);
    assert(!FC_Orb_Publish((fc_orb_id_t)FC_ORB_TOPIC_COUNT, &gyro));
    assert(!FC_Orb_Publish(FC_ORB_SENSOR_GYRO, NULL));
    assert(!FC_Orb_Copy(FC_ORB_SENSOR_ACCEL, &out));
    assert(!FC_Orb_Copy(FC_ORB_SENSOR_GYRO, NULL));
    FC_Orb_Reset();
    assert(FC_Orb_Valid(FC_ORB_SENSOR_GYRO) == false);
    printf("test_orb: OK\n");
}

static void test_param(void)
{
    fc_param_val_t value;
    fc_attitude_config_t att_cfg;
    fc_rate_config_t rate_cfg;
    fc_position_config_t pos_cfg;
    fc_land_config_t land_cfg;
    fc_guard_config_t guard_cfg;
    fc_mixer_config_t mix_cfg;
    fc_gps_config_t gps_cfg;
    fc_compass_config_t compass_cfg;
    float v;

    FC_Param_Init();
    assert(FC_Param_Count() > 0U);
    assert(FC_Param_GetFloat("MC_ROLL_P", &v) && NEAR(v, 4.0f, 1e-6f));
    assert(FC_Param_GetFloat("MC_PITCHRATE_I", &v) && NEAR(v, 0.2f, 1e-6f));
    assert(FC_Param_GetFloat("FD_FAIL_R", &v) && NEAR(v, 25.0f, 1e-6f));
    assert(FC_Param_GetFloat("LNDMC_Z_VEL_MAX", &v) && NEAR(v, 0.25f, 1e-6f));
    assert(!FC_Param_GetFloat("NO_SUCH_PARAM", &v));
    assert(!FC_Param_SetFloat("NO_SUCH_PARAM", 1.0f));
    assert(FC_Param_Get("MC_ROLL_P", &value) && NEAR(value.f32, 4.0f, 1e-6f));
    assert(!FC_Param_GetInt("MC_ROLL_P", NULL));
    assert(!FC_Param_SetInt("MC_ROLL_P", 1));
    assert(FC_Param_GetInt("MC_YAWRATE_MAX", NULL) == false);
    assert(FC_Param_SetFloat("MC_ROLL_P", 6.0f));
    assert(FC_Param_GetFloat("MC_ROLL_P", &v) && NEAR(v, 6.0f, 1e-6f));

    FC_Attitude_ConfigDefault(&att_cfg);
    FC_RateControl_ConfigDefault(&rate_cfg);
    FC_Position_ConfigDefault(&pos_cfg);
    FC_Land_ConfigDefault(&land_cfg);
    FC_Guard_ConfigDefault(&guard_cfg);
    FC_Mixer_ConfigDefault(&mix_cfg);
    FC_GPS_ConfigDefault(&gps_cfg);
    FC_Compass_ConfigDefault(&compass_cfg);
    FC_Param_ResetAll();
    FC_Param_Apply(&att_cfg, &rate_cfg, &pos_cfg, &land_cfg, &guard_cfg, &mix_cfg,
                   &gps_cfg, &compass_cfg);
    assert(NEAR(att_cfg.gain_att[0], 4.0f, 1e-4f));
    assert(NEAR(att_cfg.lim_rate[1], RAD(220.0f), 1e-4f));
    assert(NEAR(rate_cfg.gain_rate_p[0], 0.15f, 1e-4f));
    assert(NEAR(rate_cfg.gain_rate_i[2], 0.1f, 1e-4f));
    assert(NEAR(rate_cfg.yaw_tq_cutoff, 2.0f, 1e-4f));
    assert(NEAR(pos_cfg.gain_vel_p[2], 4.0f, 1e-4f));
    assert(NEAR(pos_cfg.lim_vel_h, 12.0f, 1e-4f));
    assert(NEAR(pos_cfg.lim_tilt, RAD(45.0f), 1e-4f));
    assert(NEAR(pos_cfg.lim_acc_h, 3.0f, 1e-4f));
    assert(NEAR(guard_cfg.stab_roll_enter_rad, RAD(25.0f), 1e-4f));
    assert(NEAR(guard_cfg.stab_enter_t_s, 0.3f, 1e-4f));
    assert(NEAR(guard_cfg.stab_rate_lim, 3.0f, 1e-4f));
    assert(NEAR(guard_cfg.rec_rate_lim, 6.283185307f, 1e-4f)); /* 360 deg/s */
    assert(NEAR(guard_cfg.rec_air_m, 0.4f, 1e-4f));
    assert(NEAR(land_cfg.z_vel_max, 0.25f, 1e-4f));
    assert(NEAR(land_cfg.rot_max_rad_s, RAD(20.0f), 1e-4f));

    assert(FC_Param_SetFloat("MC_ROLLRATE_P", 0.5f));
    assert(FC_Param_SetFloat("FD_FAIL_R", 30.0f));
    assert(FC_Param_SetFloat("LNDMC_XY_VEL_MAX", 2.0f));
    assert(FC_Param_SetFloat("MPC_ACC_HOR_MAX", 2.0f));
    assert(FC_Param_SetFloat("GPS_POS_GATE", 25.0f));
    assert(FC_Param_SetInt("CMP_ROT", 90));
    FC_Param_Apply(&att_cfg, &rate_cfg, &pos_cfg, &land_cfg, &guard_cfg, &mix_cfg,
                   &gps_cfg, &compass_cfg);
    assert(NEAR(rate_cfg.gain_rate_p[0], 0.5f, 1e-4f));
    assert(NEAR(rate_cfg.gain_rate_p[1], 0.15f, 1e-4f));
    assert(NEAR(guard_cfg.stab_roll_enter_rad, RAD(30.0f), 1e-4f));
    assert(NEAR(guard_cfg.stab_pitch_enter_rad, RAD(25.0f), 1e-4f));
    assert(NEAR(land_cfg.xy_vel_max, 2.0f, 1e-4f));
    assert(NEAR(pos_cfg.lim_acc_h, 2.0f, 1e-4f));
    /* The navigation configs are applied through the same call, so the params
     * have to reach them or the gates and heading offset would stay at their
     * compiled-in defaults no matter what the param server says. */
    assert(NEAR(gps_cfg.pos_gate_m, 25.0f, 1e-4f));
    assert(NEAR(compass_cfg.rotation_deg, 90.0f, 1e-4f));
    printf("test_param: OK\n");
}

static void test_mission_config_defaults(void)
{
    fc_mission_config_t cfg;
    FC_Mission_ConfigDefault(&cfg);
    assert(NEAR(cfg.loiter_omega, 0.5f, 1e-4f));
    assert(NEAR(cfg.loiter_ramp_s, 3.0f, 1e-4f));
    assert(NEAR(cfg.accept_xy, 0.8f, 1e-4f));
    assert(NEAR(cfg.alt_accept, 0.25f, 1e-4f));
    assert(NEAR(cfg.climb_rate, 0.4f, 1e-4f));
    assert(NEAR(cfg.descend_rate, 0.3f, 1e-4f));
    assert(NEAR(cfg.land_alt, 0.05f, 1e-4f));

    fc_mission_t m;
    FC_Mission_Init(&m);
    assert(m.count == 0U);
    assert(FC_Mission_Active(&m) == 0U);
    for (int i = 0; i < (int)FC_MISSION_MAX_ITEMS; i++) {
        assert(FC_Mission_Append(&m, FC_MISSION_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 1));
    }
    assert(m.count == FC_MISSION_MAX_ITEMS);
    assert(!FC_Mission_Append(&m, FC_MISSION_NAV_LAND, 0, 0, 0, 0, 0, 0, 0));
    FC_Mission_Reset(&m);
    assert(m.count == 0U);
    printf("test_mission_config_defaults: OK\n");
}

static void test_mission_hold_sequence(void)
{
    fc_mission_config_t cfg;
    fc_mission_t m;
    fc_estimator_state_t st;
    fc_position_setpoint_t sp;
    const float dt = 0.05f;

    FC_Mission_ConfigDefault(&cfg);
    memset(&st, 0, sizeof(st));
    st.pos_valid = 1U;
    st.pos_ned[2] = 0.0f;

    /* takeoff to 3 m -> hold 2 s -> land */
    FC_Mission_Init(&m);
    assert(FC_Mission_Append(&m, FC_MISSION_NAV_TAKEOFF, 0, 0, 0, NAN, 0, 0, 3.0f));
    assert(FC_Mission_Append(&m, FC_MISSION_NAV_WAYPOINT, 2.0f, 0, 0, NAN, 0, 0, 3.0f));
    assert(FC_Mission_Append(&m, FC_MISSION_NAV_LAND, 0, 0, 0, NAN, 0, 0, 0.0f));
    assert(FC_Mission_Start(&m, &st));
    assert(FC_Mission_Active(&m) == 1U);
    assert(FC_Mission_Count(&m) == 3U);
    assert(FC_Mission_Index(&m) == 0U);

    /* climb phase: altitude setpoint ramps up, horizontal at the pad */
    assert(FC_Mission_Update(&cfg, &m, &st, dt, &sp));
    assert(st.pos_ned[2] > -3.1f);  /* drone does not follow instantly */
    assert(FC_Mission_Index(&m) == 0U);

    /* fly the profile: the estimator does not move here, so simulate reaching
     * the targets by feeding positions back into the state. */
    float t = 0.0f;
    while (FC_Mission_Active(&m)) {
        assert(FC_Mission_Update(&cfg, &m, &st, dt, &sp));
        /* the drone tracks the setpoint tightly at +-eps altitude error */
        st.pos_ned[0] = sp.position_ned[0];
        st.pos_ned[1] = sp.position_ned[1];
        st.pos_ned[2] = sp.position_ned[2];
        t += dt;
        assert(t < 300.0f);
    }

    /* land leg finished: frozen at the pad on the ground */
    assert(sp.position_ned[0] == 0.0f);
    assert(sp.position_ned[1] == 0.0f);
    assert(NEAR(sp.position_ned[2], -cfg.land_alt, 1e-4f));
    assert(FC_Mission_Active(&m) == 0U);
    assert(FC_Mission_Index(&m) == 3U);
    /* ~7.5 s climb + 2 s hold + ~9.8 s descent */
    assert(t > 15.0f && t < 25.0f);

    /* inactive mission produces no update */
    st.pos_ned[0] = 0.4f;
    assert(!FC_Mission_Update(&cfg, &m, &st, dt, &sp));

    printf("test_mission_hold_sequence: t=%.1f s OK\n", t);
}

static void test_mission_loiter(void)
{
    fc_mission_config_t cfg;
    fc_mission_t m;
    fc_estimator_state_t st;
    fc_position_setpoint_t sp;
    const float dt = 0.02f;
    float t;

    FC_Mission_ConfigDefault(&cfg);
    memset(&st, 0, sizeof(st));
    st.pos_valid = 1U;

    /* climb, then 2 full laps at radius 8 -> return -> land */
    FC_Mission_Init(&m);
    assert(FC_Mission_Append(&m, FC_MISSION_NAV_TAKEOFF, 0, 0, 0, NAN, 0, 0, 3.0f));
    assert(FC_Mission_Append(&m, FC_MISSION_NAV_LOITER_TURNS, 2.0f, 0, 8.0f, NAN, 0, 0, 3.0f));
    assert(FC_Mission_Append(&m, FC_MISSION_NAV_RETURN_TO_LAUNCH, 0, 0, 0, NAN, 0, 0, 0.0f));
    assert(FC_Mission_Append(&m, FC_MISSION_NAV_LAND, 0, 0, 0, NAN, 0, 0, 0.0f));
    assert(FC_Mission_Start(&m, &st));

    /* run until the circle leg begins */
    t = 0.0f;
    while (FC_Mission_Index(&m) == 0U) {
        assert(FC_Mission_Update(&cfg, &m, &st, dt, &sp));
        st.pos_ned[0] = sp.position_ned[0];
        st.pos_ned[1] = sp.position_ned[1];
        st.pos_ned[2] = sp.position_ned[2];
        t += dt;
        assert(t < 30.0f);
    }
    assert(FC_Mission_Index(&m) == 1U);

    /* spiral entry: radius grows smoothly from 0 toward 8 during the ramp */
    float r_first = 0.0f;
    for (int k = 0; k < (int)(cfg.loiter_ramp_s / dt); k++) {
        assert(FC_Mission_Update(&cfg, &m, &st, dt, &sp));
        const float r = sqrtf(sp.position_ned[0] * sp.position_ned[0] +
                              sp.position_ned[1] * sp.position_ned[1]);
        if (k == 0) { r_first = r; }
        st.pos_ned[0] = sp.position_ned[0];
        st.pos_ned[1] = sp.position_ned[1];
        st.pos_ned[2] = sp.position_ned[2];
        assert(r <= 8.0f + 1e-3f);
    }
    assert(r_first < 1.0f);

    /* after the ramp: on the circle with analytic feed-forward */
    assert(FC_Mission_Update(&cfg, &m, &st, dt, &sp));
    const float r = sqrtf(sp.position_ned[0] * sp.position_ned[0] +
                          sp.position_ned[1] * sp.position_ned[1]);
    assert(NEAR(r, 8.0f, 0.03f));
    const float v = sqrtf(sp.velocity_ned[0] * sp.velocity_ned[0] +
                          sp.velocity_ned[1] * sp.velocity_ned[1]);
    const float a = sqrtf(sp.acceleration_ned[0] * sp.acceleration_ned[0] +
                          sp.acceleration_ned[1] * sp.acceleration_ned[1]);
    assert(NEAR(v, 8.0f * cfg.loiter_omega, 1e-2f));
    assert(NEAR(a, 8.0f * cfg.loiter_omega * cfg.loiter_omega, 1e-2f));

    /* run the whole mission out (RTL + land follow) */
    t = 0.0f;
    while (FC_Mission_Active(&m)) {
        assert(FC_Mission_Update(&cfg, &m, &st, dt, &sp));
        st.pos_ned[0] = sp.position_ned[0];
        st.pos_ned[1] = sp.position_ned[1];
        st.pos_ned[2] = sp.position_ned[2];
        t += dt;
        assert(t < 400.0f);
    }
    /* ~7.5 s climb + 27 s circling (2 laps) + 0.6 s RTL + ~10 s descent */
    /* run-out: ~25 s circle (2 laps) + 0.6 s RTL + ~10 s descent */
    assert(t > 30.0f && t < 60.0f);
    assert(FC_Mission_Index(&m) == 4U);

    printf("test_mission_loiter: r=%.2f v=%.2f a=%.2f t=%.0f s OK\n", r, v, a, t);
}

static void test_mission_yaw(void)
{
    fc_mission_config_t cfg;
    fc_mission_t m;
    fc_estimator_state_t st;
    fc_position_setpoint_t sp;

    FC_Mission_ConfigDefault(&cfg);
    memset(&st, 0, sizeof(st));
    st.pos_valid = 1U;
    st.yaw_rad = 0.3f;

    FC_Mission_Init(&m);
    assert(FC_Mission_Append(&m, FC_MISSION_NAV_WAYPOINT, 0, 0, 0, NAN, 0, 0, 1.0f));
    assert(FC_Mission_Start(&m, &st));
    assert(FC_Mission_Update(&cfg, &m, &st, 0.05f, &sp));
    /* no p4 heading -> fixed 0 (NED north), not the (drifting) estimate */
    assert(NEAR(sp.yaw_ned, 0.0f, 1e-4f));

    FC_Mission_Reset(&m);
    assert(FC_Mission_Append(&m, FC_MISSION_NAV_WAYPOINT, 0, 0, 0, 90.0f, 0, 0, 1.0f));
    assert(FC_Mission_Start(&m, &st));
    assert(FC_Mission_Update(&cfg, &m, &st, 0.05f, &sp));
    assert(NEAR(sp.yaw_ned, RAD(90.0f), 1e-4f));
    printf("test_mission_yaw: OK\n");
}

int main(void)
{
    test_estimator();
    test_estimator_pitch_sign();
    test_estimator_accel_gate();
    test_estimator_turn_holds_gyro();
    test_estimator_rest_gate_reopens();
    test_estimator_gyro_bias_online();
    test_imu_spi_init_sequence();
    test_imu_spi_reset_retry();
    test_imu_spi_wrong_part();
    test_imu_spi_sample();
    test_imu_spi_dual();
    test_attitude_defaults();
    test_attitude_level();
    test_attitude_roll_error();
    test_rate_defaults();
    test_rate_level();
    test_rate_error();
    test_mixer_allocation();
    test_position_defaults();
    test_position_zup_hover();
    test_position_zup_hold();
    test_position_ned_hover();
    test_position_vel_damping();
    test_position_acc_horizontal_limit();
    test_land_defaults();
    test_land_ground_to_landed();
    test_land_freefall();
    test_land_hysteresis_reset();
    test_guard_defaults();
    test_guard_stab();
    test_guard_recover_teleport();
    test_guard_recover_exit();
    test_sched();
    test_orb();
    test_param();
    test_mission_config_defaults();
    test_mission_hold_sequence();
    test_mission_loiter();
    test_mission_yaw();
    printf("ALL TESTS PASSED\n");
    return 0;
}