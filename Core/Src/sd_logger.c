/**
 * @file sd_logger.c
 * @brief SD Card logging - binary record format via SDMMC1
 */

#include "sd_logger.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

/* ===========================================================================
 * EXTERNAL HANDLES
 * =========================================================================== */

extern SD_HandleTypeDef hsd1;

/* ===========================================================================
 * PRIVATE VARIABLES
 * =========================================================================== */

static bool sd_initialized = false;
static bool sd_logging_active = false;
static uint32_t sd_write_sector = 0;
static uint32_t sd_total_bytes = 0;
static uint32_t sd_total_records = 0;
static uint32_t sd_total_errors = 0;

/* Buffer for batching writes */
static sd_log_record_t sd_buffer[SD_LOG_BUFFER_RECORDS];
static uint32_t sd_buffer_count = 0;

static SemaphoreHandle_t sd_mutex = NULL;

/* ===========================================================================
 * CRC16 for log records
 * =========================================================================== */

static uint16_t log_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

/* ===========================================================================
 * SD CARD INIT
 * =========================================================================== */

bool SD_Logger_Init(void)
{
    sd_mutex = xSemaphoreCreateMutex();

    /* SDMMC1 is already initialized by CubeMX in main.c */
    /* Check if card is present */
    if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
        /* Try to re-initialize */
        if (HAL_SD_Init(&hsd1) != HAL_OK) {
            DEBUG_ERR("[SD] Card init failed");
            return false;
        }

        if (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
            DEBUG_ERR("[SD] Card not ready");
            return false;
        }
    }

    HAL_SD_CardInfoTypeDef card_info;
    HAL_SD_GetCardInfo(&hsd1, &card_info);
    DEBUG_INFO("[SD] Card detected: %lu MB", (unsigned long)(card_info.BlockSize * card_info.BlockNbr / 1024 / 1024));
    DEBUG_INFO("[SD] Block size: %lu bytes", (unsigned long)card_info.BlockSize);

    sd_write_sector = 0;
    sd_initialized = true;

    DEBUG_INFO("[SD] Logger initialized");
    return true;
}

/* ===========================================================================
 * LOGGING CONTROL
 * =========================================================================== */

bool SD_Logger_Start(void)
{
    if (!sd_initialized) {
        DEBUG_ERR("[SD] Not initialized");
        return false;
    }

    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        sd_logging_active = true;
        sd_buffer_count = 0;
        sd_total_bytes = 0;
        sd_total_records = 0;
        sd_total_errors = 0;
        xSemaphoreGive(sd_mutex);

        DEBUG_INFO("[SD] Logging STARTED (sector %lu)", (unsigned long)sd_write_sector);
        return true;
    }
    return false;
}

void SD_Logger_Stop(void)
{
    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        /* Flush remaining buffer */
        if (sd_buffer_count > 0) {
            uint32_t bytes = sd_buffer_count * SD_LOG_RECORD_SIZE;
            uint32_t sectors = (bytes + SD_LOG_SECTOR_SIZE - 1) / SD_LOG_SECTOR_SIZE;

            uint8_t sector_buf[SD_LOG_SECTOR_SIZE];
            memset(sector_buf, 0, sizeof(sector_buf));
            memcpy(sector_buf, sd_buffer, bytes);

            if (HAL_SD_WriteBlocks(&hsd1, sector_buf, sd_write_sector, sectors, 1000) == HAL_OK) {
                sd_write_sector += sectors;
                sd_total_bytes += bytes;
                sd_total_records += sd_buffer_count;
            } else {
                sd_total_errors++;
            }
            sd_buffer_count = 0;
        }

        sd_logging_active = false;
        xSemaphoreGive(sd_mutex);

        DEBUG_INFO("[SD] Logging STOPPED (%lu records, %lu bytes)",
                   (unsigned long)sd_total_records, (unsigned long)sd_total_bytes);
    }
}

bool SD_Logger_IsActive(void)
{
    return sd_logging_active;
}

/* ===========================================================================
 * WRITE LOG RECORD
 * =========================================================================== */

bool SD_Logger_Write(sd_log_record_t *record)
{
    if (!sd_logging_active || !record) return false;

    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        /* Set sync bytes and calculate CRC */
        record->sync[0] = 0xAA;
        record->sync[1] = 0x55;
        record->crc16 = log_crc16((const uint8_t *)record, SD_LOG_RECORD_SIZE - 2);

        /* Add to buffer */
        memcpy(&sd_buffer[sd_buffer_count], record, sizeof(sd_log_record_t));
        sd_buffer_count++;

        /* Flush when buffer full */
        if (sd_buffer_count >= SD_LOG_BUFFER_RECORDS) {
            uint32_t bytes = sd_buffer_count * SD_LOG_RECORD_SIZE;
            uint32_t sectors = (bytes + SD_LOG_SECTOR_SIZE - 1) / SD_LOG_SECTOR_SIZE;

            uint8_t sector_buf[SD_LOG_SECTOR_SIZE];
            memset(sector_buf, 0, sizeof(sector_buf));
            memcpy(sector_buf, sd_buffer, bytes);

            if (HAL_SD_WriteBlocks(&hsd1, sector_buf, sd_write_sector, sectors, 1000) == HAL_OK) {
                sd_write_sector += sectors;
                sd_total_bytes += bytes;
                sd_total_records += sd_buffer_count;
            } else {
                sd_total_errors++;
                DEBUG_ERR("[SD] Write failed at sector %lu", (unsigned long)sd_write_sector);
            }
            sd_buffer_count = 0;
        }

        xSemaphoreGive(sd_mutex);
        return true;
    }

    return false;
}

void SD_Logger_Stats(uint32_t *bytes_written, uint32_t *records_written, uint32_t *errors)
{
    if (bytes_written) *bytes_written = sd_total_bytes;
    if (records_written) *records_written = sd_total_records;
    if (errors) *errors = sd_total_errors;
}

/* ===========================================================================
 * FREERTOS TASK
 * =========================================================================== */

void SD_Logger_Task(void *argument)
{
    DEBUG_INFO("[SD] Task started");

    /* Wait for SD card to be ready */
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        if (sd_logging_active) {
            /* Periodic flush if data in buffer but no new writes */
            if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (sd_buffer_count > 0) {
                    /* Flush partial buffer every SD_LOG_UPDATE_INTERVAL_MS */
                    uint32_t bytes = sd_buffer_count * SD_LOG_RECORD_SIZE;
                    uint32_t sectors = (bytes + SD_LOG_SECTOR_SIZE - 1) / SD_LOG_SECTOR_SIZE;

                    uint8_t sector_buf[SD_LOG_SECTOR_SIZE];
                    memset(sector_buf, 0, sizeof(sector_buf));
                    memcpy(sector_buf, sd_buffer, bytes);

                    if (HAL_SD_WriteBlocks(&hsd1, sector_buf, sd_write_sector, sectors, 1000) == HAL_OK) {
                        sd_write_sector += sectors;
                        sd_total_bytes += bytes;
                        sd_total_records += sd_buffer_count;
                    } else {
                        sd_total_errors++;
                    }
                    sd_buffer_count = 0;
                }
                xSemaphoreGive(sd_mutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(SD_LOG_UPDATE_INTERVAL_MS));
    }
}
