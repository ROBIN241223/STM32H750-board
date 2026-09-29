/**
 * @file ros2_motor.c
 * @brief Motor control - TIM2 4-channel PWM for ESC
 *
 * TIM2 config: 240MHz APB1 clock, Prescaler=59 (->4MHz), Period=19999 (->50Hz)
 * Pulse: 1000-2000 us -> compare 4000-8000
 */

#include "ros2_motor.h"
#include "ros2_comm.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "queue.h"
#include <string.h>

/* ===========================================================================
 * EXTERNAL HANDLES
 * =========================================================================== */

extern TIM_HandleTypeDef htim2;

/* ===========================================================================
 * PRIVATE VARIABLES
 * =========================================================================== */

static motor_state_t motor_state;
static QueueHandle_t motor_cmd_queue = NULL;
static volatile motor_owner_t motor_owner = MOTOR_OWNER_MANUAL;
static volatile bool flight_arm_request = false;
static volatile bool flight_output_valid = false;
static fc_motor_output_t flight_output;
static volatile uint32_t flight_output_tick = 0U;

/* TIM2 tick rate: 240MHz / (Prescaler+1) = 240MHz/60 = 4MHz -> 0.25us per tick */
#define TIM2_TICK_HZ        4000000
#define US_TO_TICKS(us)     ((uint32_t)(us) * TIM2_TICK_HZ / 1000000)

/* ===========================================================================
 * INITIALIZATION
 * =========================================================================== */

void ROS2_Motor_Init(void)
{
    memset(&motor_state, 0, sizeof(motor_state));
    motor_state.armed = false;
    motor_state.failsafe = false;
    motor_owner = MOTOR_OWNER_MANUAL;
    flight_arm_request = false;
    flight_output_valid = false;
    memset(&flight_output, 0, sizeof(flight_output));
    flight_output_tick = 0U;

    /* Reconfigure TIM2 for ESC PWM: 50Hz */
    /* Stop timer first */
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

    /* Reconfigure for 50Hz ESC PWM */
    __HAL_TIM_SET_PRESCALER(&htim2, 60 - 1);      /* 240MHz/60 = 4MHz */
    __HAL_TIM_SET_AUTORELOAD(&htim2, 20000 - 1);  /* 4MHz/20000 = 50Hz */

    /* Set all channels to minimum (disarmed) */
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR1_CHANNEL, US_TO_TICKS(0));
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR2_CHANNEL, US_TO_TICKS(0));
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR3_CHANNEL, US_TO_TICKS(0));
    __HAL_TIM_SET_COMPARE(&htim2, MOTOR4_CHANNEL, US_TO_TICKS(0));

    /* Start PWM */
    HAL_TIM_PWM_Start(&htim2, MOTOR1_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, MOTOR2_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, MOTOR3_CHANNEL);
    HAL_TIM_PWM_Start(&htim2, MOTOR4_CHANNEL);

    motor_cmd_queue = xQueueCreate(4, sizeof(ros2_motor_msg_t));

    DEBUG_INFO("[Motor] TIM2 reconfigured for 50Hz ESC PWM");
    DEBUG_INFO("[Motor] Channels: CH1(PA0) CH2(PA1) CH3(PA2) CH4(PB11)");
    DEBUG_INFO("[Motor] PWM range: %d-%d us", MOTOR_PWM_MIN_US, MOTOR_PWM_MAX_US);
}

/* ===========================================================================
 * PWM CONTROL
 * =========================================================================== */

static void set_pwm_channel(uint32_t channel, uint16_t pulse_us)
{
    if (pulse_us == 0) {
        __HAL_TIM_SET_COMPARE(&htim2, channel, 0);
    } else {
        if (pulse_us < MOTOR_PWM_MIN_US) pulse_us = MOTOR_PWM_MIN_US;
        if (pulse_us > MOTOR_PWM_MAX_US) pulse_us = MOTOR_PWM_MAX_US;
        __HAL_TIM_SET_COMPARE(&htim2, channel, US_TO_TICKS(pulse_us));
    }
}

void ROS2_Motor_SetPWM(uint8_t motor, uint16_t pulse_us)
{
    if (motor >= MOTOR_COUNT) return;
    if (!motor_state.armed && pulse_us > 0) return;

    motor_state.pwm_us[motor] = pulse_us;

    switch (motor) {
        case 0: set_pwm_channel(MOTOR1_CHANNEL, pulse_us); break;
        case 1: set_pwm_channel(MOTOR2_CHANNEL, pulse_us); break;
        case 2: set_pwm_channel(MOTOR3_CHANNEL, pulse_us); break;
        case 3: set_pwm_channel(MOTOR4_CHANNEL, pulse_us); break;
    }
}

void ROS2_Motor_SetAll(uint8_t values[MOTOR_COUNT])
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        /* Map 0-255 to MOTOR_PWM_MIN_US - MOTOR_PWM_MAX_US */
        uint16_t pulse_us;
        if (values[i] == 0) {
            pulse_us = 0;
        } else {
            pulse_us = MOTOR_PWM_MIN_US +
                (uint16_t)((uint32_t)values[i] * (MOTOR_PWM_MAX_US - MOTOR_PWM_MIN_US) / 255);
        }
        ROS2_Motor_SetPWM(i, pulse_us);
    }
    motor_state.last_cmd_tick = osKernelGetTickCount();
}

/* ===========================================================================
 * ARM / DISARM
 * =========================================================================== */

void ROS2_Motor_Arm(void)
{
    if (motor_state.armed) return;

    DEBUG_INFO("[Motor] Arming ESCs...");

    /* ESC arming: send minimum pulse for 1 second */
    for (uint8_t ch = 0; ch < MOTOR_COUNT; ch++) {
        set_pwm_channel(
            (ch == 0) ? MOTOR1_CHANNEL :
            (ch == 1) ? MOTOR2_CHANNEL :
            (ch == 2) ? MOTOR3_CHANNEL : MOTOR4_CHANNEL,
            MOTOR_PWM_ARM_US);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Set to idle */
    for (uint8_t ch = 0; ch < MOTOR_COUNT; ch++) {
        set_pwm_channel(
            (ch == 0) ? MOTOR1_CHANNEL :
            (ch == 1) ? MOTOR2_CHANNEL :
            (ch == 2) ? MOTOR3_CHANNEL : MOTOR4_CHANNEL,
            MOTOR_PWM_MIN_US);
    }

    motor_state.armed = true;
    motor_state.failsafe = false;
    motor_state.last_cmd_tick = osKernelGetTickCount();
    DEBUG_INFO("[Motor] ESCs armed successfully");
}

void ROS2_Motor_Disarm(void)
{
    DEBUG_INFO("[Motor] Disarming ESCs...");
    ROS2_Motor_EmergencyStop();
    motor_state.armed = false;
    DEBUG_INFO("[Motor] ESCs disarmed");
}

void ROS2_Motor_EmergencyStop(void)
{
    for (uint8_t ch = 0; ch < MOTOR_COUNT; ch++) {
        set_pwm_channel(
            (ch == 0) ? MOTOR1_CHANNEL :
            (ch == 1) ? MOTOR2_CHANNEL :
            (ch == 2) ? MOTOR3_CHANNEL : MOTOR4_CHANNEL,
            0);
        motor_state.pwm_us[ch] = 0;
    }
    motor_state.failsafe = true;
    DEBUG_WARN("[Motor] EMERGENCY STOP - all motors killed");
}

void ROS2_Motor_FailsafeCheck(void)
{
    if (!motor_state.armed) return;

    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed = now - motor_state.last_cmd_tick;

    if (elapsed > 500) {
        DEBUG_WARN("[Motor] Failsafe triggered! No command for %lu ms", (unsigned long)elapsed);
        ROS2_Motor_EmergencyStop();
    }
}

const motor_state_t* ROS2_Motor_GetState(void)
{
    return &motor_state;
}

/* ===========================================================================
 * FREERTOS TASK
 * =========================================================================== */

void ROS2_Motor_Task(void *argument)
{
    ros2_motor_msg_t cmd;

    (void)argument;
    DEBUG_INFO("[Motor] Task started");

    while (1) {
        if (xQueueReceive(motor_cmd_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (motor_owner == MOTOR_OWNER_MANUAL) {
                if (cmd.armed && !motor_state.armed) {
                    ROS2_Motor_Arm();
                } else if (!cmd.armed && motor_state.armed) {
                    ROS2_Motor_Disarm();
                }
                if (motor_state.armed) {
                    ROS2_Motor_SetAll(cmd.motor);
                }
            }
        }

        if (motor_owner == MOTOR_OWNER_FLIGHT) {
            uint32_t now = osKernelGetTickCount();
            uint32_t age = now - flight_output_tick;
            if (flight_arm_request && !motor_state.armed) {
                ROS2_Motor_Arm();
            }
            if (!flight_arm_request && motor_state.armed) {
                ROS2_Motor_Disarm();
            }
            if (motor_state.armed && flight_output_valid && (age <= 100U)) {
                taskENTER_CRITICAL();
                for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
                    motor_state.pwm_us[i] = (uint16_t)(
                        MOTOR_PWM_MIN_US +
                        ((uint32_t)flight_output.pwm[i] *
                         (MOTOR_PWM_MAX_US - MOTOR_PWM_MIN_US) / 255U));
                    set_pwm_channel(
                        (i == 0U) ? MOTOR1_CHANNEL :
                        (i == 1U) ? MOTOR2_CHANNEL :
                        (i == 2U) ? MOTOR3_CHANNEL : MOTOR4_CHANNEL,
                        motor_state.pwm_us[i]);
                }
                taskEXIT_CRITICAL();
                motor_state.last_cmd_tick = now;
            } else if (motor_state.armed && (age > 100U)) {
                ROS2_Motor_FlightFault();
            }
        } else {
            ROS2_Motor_FailsafeCheck();
        }
    }
}

/* ===========================================================================
 * QUEUE ACCESS (called from ros2_comm task)
 * =========================================================================== */

void ROS2_Motor_EnqueueCmd(ros2_motor_msg_t *cmd)
{
    if (motor_cmd_queue && cmd) {
        xQueueOverwrite(motor_cmd_queue, cmd);
    }
}

void ROS2_Motor_RequestFlightArm(void)
{
    motor_owner = MOTOR_OWNER_FLIGHT;
    flight_arm_request = true;
    motor_state.failsafe = false;
}

void ROS2_Motor_FlightDisarm(void)
{
    flight_arm_request = false;
    flight_output_valid = false;
    if (motor_state.armed) {
        ROS2_Motor_Disarm();
    }
    motor_owner = MOTOR_OWNER_NONE;
}

void ROS2_Motor_FlightFault(void)
{
    flight_arm_request = false;
    flight_output_valid = false;
    if (motor_state.armed || !motor_state.failsafe) {
        ROS2_Motor_EmergencyStop();
    }
    motor_state.armed = false;
    motor_owner = MOTOR_OWNER_NONE;
}

void ROS2_Motor_SubmitFlightOutput(const fc_motor_output_t *output)
{
    if (output == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    flight_output = *output;
    flight_output_tick = osKernelGetTickCount();
    flight_output_valid = output->valid && output->armed;
    taskEXIT_CRITICAL();
}

motor_owner_t ROS2_Motor_GetOwner(void)
{
    return motor_owner;
}
