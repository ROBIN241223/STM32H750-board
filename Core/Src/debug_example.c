/**
 * @file debug_example.c
 * @brief Example usage of Debug UART for Flight Controller
 * 
 * This file shows how to use the debug UART interface to:
 * - Print status messages
 * - Log sensor data
 * - Debug system performance
 * - Communicate with PC
 */

#include "main.h"
#include "debug_uart.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <math.h>

/* ===========================================================================
 * EXAMPLE 1: System Status Task
 * =========================================================================== */

/**
 * Print system status every 1 second
 * This is just an example - copy and modify for your needs
 */
void SystemStatus_Task(void *argument)
{
    static uint32_t loop_count = 0;
    
    while(1)
    {
        DEBUG_INFO("=== System Status (Loop %lu) ===", loop_count++);
        
        /* Print heap usage */
        #if (configUSE_TRACE_FACILITY == 1)
        size_t free_heap = xPortGetFreeHeapSize();
        DEBUG_INFO("Free Heap: %lu bytes", free_heap);
        #endif
        
        /* Print system clock */
        DEBUG_INFO("System Clock: %lu MHz", SystemCoreClock / 1000000);
        
        /* Print FreeRTOS tick count */
        uint32_t tick_count = osKernelGetTickCount();
        uint32_t tick_freq = osKernelGetTickFreq();
        DEBUG_INFO("Uptime: %.2f seconds (Ticks: %lu @ %lu Hz)", 
                   (float)tick_count / tick_freq, tick_count, tick_freq);
        
        /* Print time of day */
        static int counter = 0;
        DEBUG_INFO("Counter: %d", counter++);
        
        /* Wait 1 second */
        osDelay(1000);
    }
}

/* ===========================================================================
 * EXAMPLE 2: Sensor Data Logging Task
 * =========================================================================== */

/**
 * Simulate reading sensor data and logging via UART
 * Replace with actual SPI/I2C sensor reads
 */
void SensorData_Task(void *argument)
{
    DEBUG_INFO("[SensorData] Starting sensor logging task");
    
    while(1)
    {
        /* Simulate IMU data (replace with actual SPI1 reads) */
        float accel_x = 0.5f;   // Real: read from IMU via SPI1
        float accel_y = -0.2f;
        float accel_z = 9.8f;
        float gyro_x = 0.1f;    // deg/s
        float gyro_y = -0.05f;
        float gyro_z = 0.0f;
        
        DEBUG_INFO("IMU - Accel: X=%.3f Y=%.3f Z=%.3f (m/s²)", 
                   accel_x, accel_y, accel_z);
        DEBUG_INFO("IMU - Gyro:  X=%.3f Y=%.3f Z=%.3f (°/s)", 
                   gyro_x, gyro_y, gyro_z);
        
        /* Simulate barometer data (replace with actual I2C reads) */
        float pressure = 101325.0f;  // Pa
        float altitude = 0.0f;       // m
        float temperature = 25.0f;   // °C
        
        DEBUG_INFO("Baro - Pressure: %.2f Pa, Alt: %.2f m, Temp: %.2f °C", 
                   pressure, altitude, temperature);
        
        /* Print raw sensor data as hex dump */
        uint8_t raw_imu[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
        DEBUG_INFO("IMU Raw Data (first 6 bytes):");
        Debug_PrintHex(raw_imu, 6);
        
        /* Wait 500ms before next read */
        osDelay(500);
    }
}

/* ===========================================================================
 * EXAMPLE 3: Motor Control Task
 * =========================================================================== */

/**
 * Example PWM motor control with debug output
 * Actual code would control TIM2 PWM channels
 */
void MotorControl_Task(void *argument)
{
    DEBUG_INFO("[MotorControl] Starting motor control task");
    
    uint16_t pwm_value = 1000;  /* 0-1000 for ESC */
    
    while(1)
    {
        /* Simulate motor PWM values */
        uint16_t motor1 = pwm_value;
        uint16_t motor2 = pwm_value;
        uint16_t motor3 = pwm_value;
        uint16_t motor4 = pwm_value;
        
        /* In real code, set PWM:
         * __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, motor1);
         * __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor2);
         * etc...
         */
        
        DEBUG_INFO("Motor PWM: M1=%u M2=%u M3=%u M4=%u", 
                   motor1, motor2, motor3, motor4);
        
        /* Simulate throttle increase */
        pwm_value += 10;
        if (pwm_value > 1000)
            pwm_value = 1000;
        
        osDelay(100);
    }
}

/* ===========================================================================
 * EXAMPLE 4: Command Processing Task
 * =========================================================================== */

/**
 * Process commands received from PC via debug UART
 */
void CommandProcessor_Task(void *argument)
{
    DEBUG_INFO("[Command] Starting command processor task");
    
    uint8_t rx_buffer[64];
    
    while(1)
    {
        /* Check if any data is available */
        if (Debug_Available() > 0)
        {
            uint32_t rx_len = Debug_Receive(rx_buffer, sizeof(rx_buffer) - 1);
            rx_buffer[rx_len] = '\0';  /* Null terminate */
            
            DEBUG_INFO("Received %lu bytes: %s", rx_len, (char *)rx_buffer);
            
            /* Parse commands */
            if (strstr((char *)rx_buffer, "STATUS") != NULL)
            {
                DEBUG_INFO("RESPONSE: OK - System Running");
            }
            else if (strstr((char *)rx_buffer, "RESET") != NULL)
            {
                DEBUG_WARN("RESPONSE: Resetting system...");
                osDelay(100);
                NVIC_SystemReset();
            }
            else if (strstr((char *)rx_buffer, "INFO") != NULL)
            {
                DEBUG_INFO("=== Flight Controller Info ===");
                DEBUG_INFO("MCU: STM32H750VBTx");
                DEBUG_INFO("Clock: 480 MHz");
                DEBUG_INFO("RAM: 1 MB");
                DEBUG_INFO("Flash: 128 KB");
                DEBUG_INFO("FreeRTOS: Yes");
            }
            else
            {
                DEBUG_ERR("Unknown command");
            }
        }
        
        osDelay(50);
    }
}

/* ===========================================================================
 * EXAMPLE 5: Performance Monitoring Task
 * =========================================================================== */

/**
 * Monitor task performance and print statistics
 */
void PerformanceMonitor_Task(void *argument)
{
    DEBUG_INFO("[Performance] Starting performance monitor");
    
    uint32_t last_tick = 0;
    uint32_t last_heap = 0;
    
    while(1)
    {
        uint32_t current_tick = osKernelGetTickCount();
        uint32_t elapsed_ms = (current_tick - last_tick);
        
        #if (configUSE_TRACE_FACILITY == 1)
        size_t free_heap = xPortGetFreeHeapSize();
        if (free_heap != last_heap)
        {
            DEBUG_WARN("Heap changed: %lu → %lu bytes", last_heap, free_heap);
            last_heap = free_heap;
        }
        #endif
        
        if (elapsed_ms >= 5000)  /* Every 5 seconds */
        {
            DEBUG_INFO("Uptime: %u seconds, Tick: %u", 
                       current_tick / 1000, current_tick);
            last_tick = current_tick;
        }
        
        osDelay(1000);
    }
}

/* ===========================================================================
 * HELPER FUNCTIONS FOR DEBUGGING
 * =========================================================================== */

/**
 * Print floating point register (for comparing values)
 */
void Debug_PrintFloat(const char *name, float value)
{
    DEBUG_INFO("%s = %.6f", name, value);
}

/**
 * Print memory address and contents
 */
void Debug_DumpMemory(void *address, uint32_t size)
{
    uint8_t *ptr = (uint8_t *)address;
    DEBUG_INFO("Memory dump @ %p (%lu bytes):", address, size);
    Debug_PrintHex(ptr, size);
}

/**
 * Print error code and message
 */
void Debug_PrintError(const char *source, HAL_StatusTypeDef status)
{
    const char *status_str;
    
    switch(status)
    {
        case HAL_OK:      status_str = "OK"; break;
        case HAL_ERROR:   status_str = "ERROR"; break;
        case HAL_BUSY:    status_str = "BUSY"; break;
        case HAL_TIMEOUT: status_str = "TIMEOUT"; break;
        default:          status_str = "UNKNOWN"; break;
    }
    
    if (status != HAL_OK)
        DEBUG_ERR("%s returned: %s (%d)", source, status_str, status);
}

/* ===========================================================================
 * TO USE THESE EXAMPLES:
 * 
 * 1. Include this file in your freertos.c or main.c:
 *    #include "debug_example.c"
 * 
 * 2. In main.c, create the tasks:
 *    osThreadNew(SystemStatus_Task, NULL, NULL);
 *    osThreadNew(SensorData_Task, NULL, NULL);
 *    osThreadNew(MotorControl_Task, NULL, NULL);
 *    osThreadNew(CommandProcessor_Task, NULL, NULL);
 *    osThreadNew(PerformanceMonitor_Task, NULL, NULL);
 * 
 * 3. Build, flash, and open terminal at 115200 bps
 * 
 * 4. Send commands like:
 *    - "STATUS" → Get system status
 *    - "INFO" → Get system info
 *    - "RESET" → Reset system
 * =========================================================================== */
