/**
 * @file ros2_sensor.c
 * @brief Sensor reading module - ADC3 4-channel scanning
 */

#include "ros2_sensor.h"
#include "ros2_comm.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <string.h>

/* ===========================================================================
 * EXTERNAL HANDLES
 * =========================================================================== */

extern ADC_HandleTypeDef hadc3;

/* ===========================================================================
 * INITIALIZATION
 * =========================================================================== */

void ROS2_Sensor_Init(void)
{
    /* ADC3 is already initialized by CubeMX in main.c */
    /* Calibrate ADC if supported */
    if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        DEBUG_WARN("[Sensor] ADC3 calibration failed");
    }
    DEBUG_INFO("[Sensor] ADC3 initialized (16-bit, 4 channels)");
}

/* ===========================================================================
 * READ SENSORS
 * =========================================================================== */

void ROS2_Sensor_Read(sensor_data_t *data)
{
    if (!data) return;
    memset(data, 0, sizeof(sensor_data_t));
    data->timestamp_ms = osKernelGetTickCount();

    /* Read 4 ADC channels sequentially (scan mode, single conversion) */
    for (uint8_t ch = 0; ch < SENSOR_ADC_CHANNELS; ch++) {
        if (HAL_ADC_Start(&hadc3) != HAL_OK) continue;

        if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
            uint32_t raw = HAL_ADC_GetValue(&hadc3);
            data->adc_raw[ch] = (float)raw / SENSOR_ADC_RESOLUTION;
            data->adc_mv[ch] = data->adc_raw[ch] * SENSOR_VBAT_REF_MV;
        }

        HAL_ADC_Stop(&hadc3);
    }

    /* Calculate battery voltage (assume channel 0 is voltage divider) */
    data->vbat_mv = data->adc_mv[0] * SENSOR_VBAT_DIVIDER_RATIO;

    /* Read MCU internal temperature (STM32H750: TS_DATA register) */
    /* Temperature formula: T = ((TS_CODE - TS_CAL1) / (TS_CAL2 - TS_CAL1)) * (130 - 30) + 30 */
    /* Simplified: use a linear approximation */
    data->temp_c = 25.0f; /* TODO: implement actual temp sensor read */
}

/* ===========================================================================
 * FREERTOS TASK
 * =========================================================================== */

void ROS2_Sensor_Task(void *argument)
{
    sensor_data_t sensor_data;

    DEBUG_INFO("[Sensor] Task started (interval: %dms)", SENSOR_UPDATE_INTERVAL_MS);

    while (1) {
        ROS2_Sensor_Read(&sensor_data);

        /* Send to ROS2 via UART2 */
        ROS2_SendSensorData(
            sensor_data.adc_mv,
            sensor_data.vbat_mv,
            sensor_data.temp_c,
            sensor_data.timestamp_ms
        );

        vTaskDelay(pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL_MS));
    }
}
