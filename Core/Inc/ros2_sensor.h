/**
 * @file ros2_sensor.h
 * @brief Sensor reading module - ADC3 4-channel, battery voltage, temperature
 */

#ifndef ROS2_SENSOR_H
#define ROS2_SENSOR_H

#include "main.h"
#include <stdint.h>

/* ===========================================================================
 * CONFIGURATION
 * =========================================================================== */

#define SENSOR_UPDATE_INTERVAL_MS   20      /* 50 Hz publish rate */
#define SENSOR_ADC_CHANNELS        4
#define SENSOR_VBAT_DIVIDER_RATIO  11.0f   /* voltage divider ratio */
#define SENSOR_VBAT_REF_MV         3300.0f /* ADC reference voltage mV */
#define SENSOR_ADC_RESOLUTION      65535.0f /* 16-bit ADC */

/* ===========================================================================
 * DATA STRUCTURES
 * =========================================================================== */

typedef struct {
    float adc_raw[SENSOR_ADC_CHANNELS];    /* raw ADC values (0.0 - 1.0) */
    float adc_mv[SENSOR_ADC_CHANNELS];     /* ADC in millivolts */
    float vbat_mv;                         /* battery voltage mV */
    float temp_c;                          /* MCU internal temperature */
    uint32_t timestamp_ms;                 /* FreeRTOS tick count */
} sensor_data_t;

/* ===========================================================================
 * FUNCTIONS
 * =========================================================================== */

/**
 * @brief Initialize sensor module (ADC3 calibration)
 */
void ROS2_Sensor_Init(void);

/**
 * @brief Read all sensors and update data structure
 * @param data Pointer to sensor data structure to fill
 */
void ROS2_Sensor_Read(sensor_data_t *data);

/**
 * @brief FreeRTOS task: reads sensors and sends to ROS2 at 50Hz
 */
void ROS2_Sensor_Task(void *argument);

#endif /* ROS2_SENSOR_H */
