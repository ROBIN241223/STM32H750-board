#ifndef FC_TYPES_H
#define FC_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint64_t timestamp_us;
    uint32_t sequence;
    float accel_mps2[3];
    float gyro_rad_s[3];
    float mag_ut[3];
    uint32_t mag_valid;
    uint32_t status;
} fc_imu_sample_t;

typedef struct {
    float q_w;
    float q_x;
    float q_y;
    float q_z;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float gyro_rad_s[3];
    float accel_mps2[3];
    uint32_t sample_age_us;
    uint32_t flags;
    float pos_ned[3];
    float vel_ned[3];
    uint32_t pos_valid;
    /* Consecutive samples that have satisfied the quiescence test. Reset to
     * zero the moment the vehicle moves, so the at-rest corrections shut off
     * immediately and cannot be left latched on through a turn. */
    uint32_t rest_count;
    uint8_t at_rest;
} fc_estimator_state_t;

typedef struct {
    bool armed;
    float thrust_norm;
    float roll_rad;
    float pitch_rad;
    float yaw_rate_rad_s;
    uint32_t timestamp_ms;
} fc_setpoint_t;

typedef struct {
    float roll_torque;
    float pitch_torque;
    float yaw_torque;
} fc_torque_cmd_t;

typedef struct {
    float normalized[4];
    uint8_t pwm[4];
    bool valid;
    bool armed;
    uint32_t timestamp_ms;
} fc_motor_output_t;

#define FC_ESTIMATOR_FLAG_INITIALIZED   (1UL << 0)
#define FC_ESTIMATOR_FLAG_ACCEL_VALID   (1UL << 1)
#define FC_ESTIMATOR_FLAG_DUAL_VALID    (1UL << 2)
#define FC_ESTIMATOR_FLAG_FAULT         (1UL << 3)
#define FC_ESTIMATOR_FLAG_AT_REST       (1UL << 4)

#endif
