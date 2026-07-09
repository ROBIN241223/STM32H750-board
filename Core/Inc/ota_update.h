/**
 * @file ota_update.h
 * @brief OTA firmware update module via QSPI Flash (W25Q64)
 *
 * Flash Layout (8MB W25Q64):
 *   0x000000 - 0x003FFF : Config/Status (16KB)
 *   0x004000 - 0x0FBFFF : App A - current firmware (1008KB)
 *   0x0FC000 - 0x1F7FFF : App B - OTA target firmware (1008KB)
 *   0x1F8000 - 0x1FFFFF : Bootloader metadata (32KB)
 *   0x200000 - 0x7FFFFF : Reserved
 */

#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#include "main.h"
#include "ros2_comm.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * QSPI FLASH LAYOUT
 * =========================================================================== */

#define OTA_QSPI_FLASH_SIZE        (8 * 1024 * 1024)   /* 8MB W25Q64 */

#define OTA_ADDR_CONFIG            0x000000
#define OTA_SIZE_CONFIG            (16 * 1024)          /* 16KB */

#define OTA_ADDR_APP_A             0x004000
#define OTA_SIZE_APP_A             (1008 * 1024)        /* 1008KB */

#define OTA_ADDR_APP_B             0x0FC000
#define OTA_SIZE_APP_B             (1008 * 1024)        /* 1008KB */

#define OTA_ADDR_BOOT_META         0x1F8000
#define OTA_SIZE_BOOT_META         (32 * 1024)          /* 32KB */

#define OTA_CHUNK_SIZE             256                   /* W25Q64 page size */
#define OTA_ERASE_SECTOR_SIZE      4096                  /* W25Q64 sector size */

/* ===========================================================================
 * W25Q64 COMMANDS
 * =========================================================================== */

#define W25Q64_CMD_WRITE_ENABLE        0x06
#define W25Q64_CMD_WRITE_DISABLE       0x04
#define W25Q64_CMD_READ_STATUS_REG1    0x05
#define W25Q64_CMD_READ_DATA           0x03
#define W25Q64_CMD_PAGE_PROGRAM        0x02
#define W25Q64_CMD_SECTOR_ERASE        0x20
#define W25Q64_CMD_BLOCK_ERASE_32K     0x52
#define W25Q64_CMD_BLOCK_ERASE_64K     0xD8
#define W25Q64_CMD_CHIP_ERASE          0xC7
#define W25Q64_CMD_JEDEC_ID            0x9F
#define W25Q64_CMD_POWER_DOWN          0xB9
#define W25Q64_CMD_RELEASE_PD          0xAB

#define W25Q64_STATUS_BUSY             0x01
#define W25Q64_STATUS_WEL              0x02

/* ===========================================================================
 * CONFIG STRUCT IN FLASH
 * =========================================================================== */

#define OTA_CONFIG_MAGIC             0x4F544131  /* "OTA1" */

typedef struct {
    uint32_t magic;                 /* OTA_CONFIG_MAGIC if valid */
    uint32_t version;               /* config version */
    uint8_t active_bank;            /* 0=A, 1=B */
    uint32_t app_a_size;            /* firmware size in App A */
    uint32_t app_a_crc32;           /* CRC32 of App A */
    uint32_t app_b_size;            /* firmware size in App B */
    uint32_t app_b_crc32;           /* CRC32 of App B */
    uint32_t boot_count;            /* consecutive boot attempts */
    uint32_t max_boot_count;        /* rollback threshold */
    uint8_t app_a_valid;            /* 1=valid firmware */
    uint8_t app_b_valid;            /* 1=valid firmware */
    uint8_t reserved[16];           /* padding to 64 bytes */
} ota_config_t;

/* ===========================================================================
 * OTA STATE MACHINE
 * =========================================================================== */

typedef enum {
    OTA_STATE_IDLE,
    OTA_STATE_ERASING,
    OTA_STATE_RECEIVING,
    OTA_STATE_VERIFYING,
    OTA_STATE_REBOOTING,
    OTA_STATE_ERROR,
} ota_state_t;

typedef struct {
    ota_state_t state;
    uint32_t total_size;            /* expected firmware size */
    uint32_t received_size;         /* bytes received so far */
    uint32_t write_addr;            /* current write address in QSPI */
    uint32_t expected_crc32;        /* expected CRC32 */
    uint32_t calculated_crc32;      /* running CRC32 */
    uint32_t seq;                   /* last received sequence number */
    ota_config_t config;            /* copy of flash config */
    bool initialized;
} ota_context_t;

/* ===========================================================================
 * FUNCTIONS
 * =========================================================================== */

/**
 * @brief Initialize OTA module, read config from QSPI
 * @return true if QSPI flash detected and config valid
 */
bool OTA_Init(void);

/**
 * @brief Get current OTA state
 */
ota_state_t OTA_GetState(void);

/**
 * @brief Get OTA context (progress info)
 */
const ota_context_t* OTA_GetContext(void);

/**
 * @brief Handle OTA command from ROS2
 * @param msg Parsed OTA message
 */
void OTA_HandleCommand(ros2_ota_msg_t *msg);

/**
 * @brief QSPI Flash low-level functions
 */
bool OTA_QSPI_ReadJedecID(uint8_t *manufacturer, uint8_t *memory_type, uint8_t *capacity);
bool OTA_QSPI_Read(uint32_t addr, uint8_t *data, uint32_t len);
bool OTA_QSPI_WritePage(uint32_t addr, const uint8_t *data, uint32_t len);
bool OTA_QSPI_EraseSector(uint32_t addr);
bool OTA_QSPI_EraseBlock64K(uint32_t addr);
bool OTA_QSPI_EraseChip(void);

/**
 * @brief QSPI XIP mode management
 *
 * The app runs from QSPI in memory-mapped (XIP) mode.
 * Before writing/erasing QSPI, we must exit XIP mode.
 * After OTA completes, we reboot (bootloader re-enters XIP).
 */
void OTA_QSPI_ExitXIP(void);
void OTA_QSPI_EnterXIP(void);

/**
 * @brief CRC32 calculation
 */
uint32_t OTA_CalcCRC32(const uint8_t *data, uint32_t len);
uint32_t OTA_CalcCRC32_File(uint32_t addr, uint32_t len);

/**
 * @brief Reboot into new firmware
 */
void OTA_Reboot(void);

/**
 * @brief Mark current bank as valid (called after successful boot)
 */
void OTA_MarkValid(void);

#endif /* OTA_UPDATE_H */
