/**
 * @file ros2_motor.h
 * @brief Motor control module - TIM2 PWM 4-channel ESC control
 */

#ifndef ROS2_MOTOR_H
#define ROS2_MOTOR_H

#include "main.h"
#include "ros2_comm.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * ESC PWM CONFIGURATION
 * =========================================================================== */

#define MOTOR_PWM_FREQ_HZ       50          /* ESC standard: 50Hz (20ms period) */
#define MOTOR_PWM_PERIOD_US     20000       /* 20ms = 20000us */
#define MOTOR_PWM_MIN_US        1000        /* min pulse: 1ms = 1000us (idle) */
#define MOTOR_PWM_MAX_US        2000        /* max pulse: 2ms = 2000us (full) */
#define MOTOR_PWM_ARM_US        500         /* arming pulse: 0.5ms */
#define MOTOR_PWM_DEADBAND_US   20          /* deadband around center */

#define MOTOR_COUNT             4

/* Motor mapping to TIM2 channels */
#define MOTOR1_CHANNEL          TIM_CHANNEL_1   /* PA0 */
#define MOTOR2_CHANNEL          TIM_CHANNEL_2   /* PA1 */
#define MOTOR3_CHANNEL          TIM_CHANNEL_3   /* PA2 */
#define MOTOR4_CHANNEL          TIM_CHANNEL_4   /* PB11 */

/* ===========================================================================
 * STATE
 * =========================================================================== */

typedef struct {
    uint16_t pwm_us[MOTOR_COUNT];           /* current PWM pulse width (us) */
    bool armed;
    bool failsafe;
    uint32_t last_cmd_tick;                 /* last command received tick */
} motor_state_t;

/* ===========================================================================
 * FUNCTIONS
 * =========================================================================== */

/**
 * @brief Initialize motor control (start PWM, set to disarmed)
 */
void ROS2_Motor_Init(void);

/**
 * @brief Set motor PWM values (1000-2000 us range)
 * @param motor Index 0-3
 * @param pulse_us Pulse width in microseconds
 */
void ROS2_Motor_SetPWM(uint8_t motor, uint16_t pulse_us);

/**
 * @brief Set all motors from ROS2 message (0-255 mapped to PWM range)
 * @param values Array of 4 values (0-255)
 */
void ROS2_Motor_SetAll(uint8_t values[MOTOR_COUNT]);

/**
 * @brief Arm all ESCs (sends low pulse for 1 second)
 */
void ROS2_Motor_Arm(void);

/**
 * @brief Disarm all ESCs (sends 0 pulse)
 */
void ROS2_Motor_Disarm(void);

/**
 * @brief Emergency stop - immediately set all motors to minimum
 */
void ROS2_Motor_EmergencyStop(void);

/**
 * @brief Failsafe check - if no command received for >500ms, stop motors
 */
void ROS2_Motor_FailsafeCheck(void);

/**
 * @brief Get current motor state
 */
const motor_state_t* ROS2_Motor_GetState(void);

/**
 * @brief FreeRTOS task: listen for motor commands from ROS2 queue
 */
void ROS2_Motor_Task(void *argument);

/**
 * @brief Enqueue a motor command from ROS2 communication task
 * @param cmd Motor command to enqueue
 */
void ROS2_Motor_EnqueueCmd(ros2_motor_msg_t *cmd);

#endif /* ROS2_MOTOR_H */
