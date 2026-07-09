/**
 * @file sd_logger.h
 * @brief SD Card logging module - binary log format via SDMMC1
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * CONFIGURATION
 * =========================================================================== */

#define SD_LOG_RECORD_SIZE      64          /* bytes per log record */
#define SD_LOG_BUFFER_RECORDS   16          /* records buffered before flush */
#define SD_LOG_SECTOR_SIZE      512
#define SD_LOG_FILENAME         "FLIGHT01.LOG"
#define SD_LOG_UPDATE_INTERVAL_MS 100       /* 10 Hz log rate */

/* ===========================================================================
 * LOG RECORD FORMAT (64 bytes, packed)
 * =========================================================================== */

#pragma pack(push, 1)
typedef struct {
    uint8_t sync[2];            /* 0xAA 0x55 sync bytes */
    uint32_t timestamp_ms;      /* FreeRTOS tick */
    uint16_t adc[4];            /* raw ADC values */
    uint16_t vbat_mv;           /* battery voltage */
    int16_t temp_c10;           /* temperature * 10 */
    uint32_t fdcan_id;          /* last FDCAN ID (0 if none) */
    uint8_t fdcan_data[8];      /* last FDCAN data */
    uint8_t fdcan_dlc;          /* last FDCAN DLC */
    uint8_t motor_pwm[4];       /* motor PWM values */
    uint8_t reserved[24];       /* reserved for future use */
    uint16_t crc16;             /* CRC16 of record */
} sd_log_record_t;
#pragma pack(pop)

/* ===========================================================================
 * FUNCTIONS
 * =========================================================================== */

/**
 * @brief Initialize SD card logger
 * @return true if SD card mounted successfully
 */
bool SD_Logger_Init(void);

/**
 * @brief Start logging to SD card
 * @return true if started
 */
bool SD_Logger_Start(void);

/**
 * @brief Stop logging
 */
void SD_Logger_Stop(void);

/**
 * @brief Check if logging is active
 */
bool SD_Logger_IsActive(void);

/**
 * @brief Write a log record
 * @param record Pointer to log record
 * @return true if written
 */
bool SD_Logger_Write(sd_log_record_t *record);

/**
 * @brief Get statistics
 * @param bytes_written Total bytes written
 * @param records_written Total records written
 * @param errors Total write errors
 */
void SD_Logger_Stats(uint32_t *bytes_written, uint32_t *records_written, uint32_t *errors);

/**
 * @brief FreeRTOS task: periodic log flush, manage logging state
 */
void SD_Logger_Task(void *argument);

#endif /* SD_LOGGER_H */
