/**
 * @file ota_update.c
 * @brief OTA firmware update via QSPI Flash (W25Q64)
 */

#include "ota_update.h"
#include "ros2_comm.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* ===========================================================================
 * EXTERNAL HANDLES
 * =========================================================================== */

extern QSPI_HandleTypeDef hqspi;

/* ===========================================================================
 * PRIVATE VARIABLES
 * =========================================================================== */

static ota_context_t ota_ctx;
static uint8_t qspi_buf[256];
static bool qspi_in_xip = false;

/* Forward declaration */
static void qspi_build_cmd(QSPI_CommandTypeDef *cmd, uint32_t instruction,
                            uint32_t addr_mode, uint32_t addr, uint32_t data_mode, uint32_t dummy);

/* ===========================================================================
 * QSPI XIP MODE MANAGEMENT
 *
 * When running from QSPI (memory-mapped), we must exit XIP mode
 * before performing any writes/erases, since memory-mapped mode
 * only supports read operations.
 * =========================================================================== */

void OTA_QSPI_ExitXIP(void)
{
    if (qspi_in_xip) {
        /* Exit memory-mapped mode by aborting any ongoing transfer */
        HAL_QSPI_Abort(&hqspi);
        qspi_in_xip = false;
    }
}

void OTA_QSPI_EnterXIP(void)
{
    if (!qspi_in_xip) {
        /* Re-enter memory-mapped mode */
        QSPI_CommandTypeDef cmd;
        qspi_build_cmd(&cmd, W25Q64_CMD_READ_DATA, QSPI_ADDRESS_1_LINE, 0, QSPI_DATA_1_LINE, 0);
        cmd.AddressSize = QSPI_ADDRESS_24_BITS;
        cmd.NbData = 0;

        QSPI_MemoryMappedTypeDef mem_mapped;
        mem_mapped.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
        HAL_QSPI_MemoryMapped(&hqspi, &cmd, &mem_mapped);
        qspi_in_xip = true;
    }
}

/* ===========================================================================
 * W25Q64 QSPI LOW-LEVEL DRIVERS
 * =========================================================================== */

static void qspi_build_cmd(QSPI_CommandTypeDef *cmd, uint32_t instruction,
                            uint32_t addr_mode, uint32_t addr, uint32_t data_mode, uint32_t dummy)
{
    cmd->Instruction = instruction;
    cmd->Address = addr;
    cmd->AlternateBytes = 0;
    cmd->AddressSize = QSPI_ADDRESS_24_BITS;
    cmd->AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    cmd->DummyCycles = dummy;
    cmd->InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd->AddressMode = addr_mode;
    cmd->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd->DataMode = data_mode;
    cmd->NbData = 0;
    cmd->DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd->DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd->SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
}

static bool qspi_write_enable(void)
{
    QSPI_CommandTypeDef cmd;
    qspi_build_cmd(&cmd, W25Q64_CMD_WRITE_ENABLE, QSPI_ADDRESS_NONE, 0, QSPI_DATA_NONE, 0);

    if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK)
        return false;

    /* Verify WEL bit */
    qspi_build_cmd(&cmd, W25Q64_CMD_READ_STATUS_REG1, QSPI_ADDRESS_NONE, 0, QSPI_DATA_1_LINE, 0);
    cmd.NbData = 1;
    if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK)
        return false;

    uint8_t status;
    if (HAL_QSPI_Receive(&hqspi, &status, 100) != HAL_OK)
        return false;

    return (status & W25Q64_STATUS_WEL) != 0;
}

static bool qspi_wait_busy(void)
{
    uint32_t timeout = 10000;
    uint8_t status;

    while (timeout--) {
        QSPI_CommandTypeDef cmd;
        qspi_build_cmd(&cmd, W25Q64_CMD_READ_STATUS_REG1, QSPI_ADDRESS_NONE, 0, QSPI_DATA_1_LINE, 0);
        cmd.NbData = 1;

        if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK)
            return false;
        if (HAL_QSPI_Receive(&hqspi, &status, 100) != HAL_OK)
            return false;

        if (!(status & W25Q64_STATUS_BUSY))
            return true;

        vTaskDelay(1);
    }

    return false;
}

/* ===========================================================================
 * QSPI PUBLIC FUNCTIONS
 * =========================================================================== */

bool OTA_QSPI_ReadJedecID(uint8_t *manufacturer, uint8_t *memory_type, uint8_t *capacity)
{
    QSPI_CommandTypeDef cmd;
    qspi_build_cmd(&cmd, W25Q64_CMD_JEDEC_ID, QSPI_ADDRESS_NONE, 0, QSPI_DATA_1_LINE, 0);
    cmd.NbData = 3;

    if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK)
        return false;

    uint8_t id_data[3];
    if (HAL_QSPI_Receive(&hqspi, id_data, 100) != HAL_OK)
        return false;

    *manufacturer = id_data[0];
    *memory_type = id_data[1];
    *capacity = id_data[2];

    return true;
}

bool OTA_QSPI_Read(uint32_t addr, uint8_t *data, uint32_t len)
{
    QSPI_CommandTypeDef cmd;
    qspi_build_cmd(&cmd, W25Q64_CMD_READ_DATA, QSPI_ADDRESS_1_LINE, addr, QSPI_DATA_1_LINE, 0);
    cmd.NbData = len;

    if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK)
        return false;

    return (HAL_QSPI_Receive(&hqspi, data, 500) == HAL_OK);
}

bool OTA_QSPI_WritePage(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (len > OTA_CHUNK_SIZE) len = OTA_CHUNK_SIZE;
    if (len == 0) return true;

    if (!qspi_write_enable()) return false;

    QSPI_CommandTypeDef cmd;
    qspi_build_cmd(&cmd, W25Q64_CMD_PAGE_PROGRAM, QSPI_ADDRESS_1_LINE, addr, QSPI_DATA_1_LINE, 0);
    cmd.NbData = len;

    if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK)
        return false;

    if (HAL_QSPI_Transmit(&hqspi, (uint8_t *)data, 100) != HAL_OK)
        return false;

    return qspi_wait_busy();
}

bool OTA_QSPI_EraseSector(uint32_t addr)
{
    addr &= ~(OTA_ERASE_SECTOR_SIZE - 1);

    if (!qspi_write_enable()) return false;

    QSPI_CommandTypeDef cmd;
    qspi_build_cmd(&cmd, W25Q64_CMD_SECTOR_ERASE, QSPI_ADDRESS_1_LINE, addr, QSPI_DATA_NONE, 0);

    if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK)
        return false;

    return qspi_wait_busy();
}

bool OTA_QSPI_EraseBlock64K(uint32_t addr)
{
    addr &= ~0xFFFF;

    if (!qspi_write_enable()) return false;

    QSPI_CommandTypeDef cmd;
    qspi_build_cmd(&cmd, W25Q64_CMD_BLOCK_ERASE_64K, QSPI_ADDRESS_1_LINE, addr, QSPI_DATA_NONE, 0);

    if (HAL_QSPI_Command(&hqspi, &cmd, 100) != HAL_OK)
        return false;

    return qspi_wait_busy();
}

/* ===========================================================================
 * CRC32
 * =========================================================================== */

uint32_t OTA_CalcCRC32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

uint32_t OTA_CalcCRC32_File(uint32_t addr, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    uint32_t remaining = len;
    uint32_t offset = 0;

    while (remaining > 0) {
        uint32_t chunk = (remaining > 256) ? 256 : remaining;
        if (!OTA_QSPI_Read(addr + offset, qspi_buf, chunk))
            return 0;

        for (uint32_t i = 0; i < chunk; i++) {
            crc ^= qspi_buf[i];
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 1)
                    crc = (crc >> 1) ^ 0xEDB88320;
                else
                    crc >>= 1;
            }
        }

        offset += chunk;
        remaining -= chunk;
    }

    return ~crc;
}

/* ===========================================================================
 * OTA INIT
 * =========================================================================== */

bool OTA_Init(void)
{
    memset(&ota_ctx, 0, sizeof(ota_ctx));

    uint8_t mfr, type, cap;
    if (!OTA_QSPI_ReadJedecID(&mfr, &type, &cap)) {
        DEBUG_ERR("[OTA] QSPI: Failed to read JEDEC ID");
        return false;
    }

    DEBUG_INFO("[OTA] QSPI Flash: MFR=0x%02X TYPE=0x%02X CAP=0x%02X", mfr, type, cap);

    if (mfr != 0xEF) {
        DEBUG_WARN("[OTA] Expected Winbond (0xEF), got 0x%02X", mfr);
    }

    if (!OTA_QSPI_Read(OTA_ADDR_CONFIG, (uint8_t *)&ota_ctx.config, sizeof(ota_config_t))) {
        DEBUG_ERR("[OTA] Failed to read config");
        return false;
    }

    if (ota_ctx.config.magic != OTA_CONFIG_MAGIC) {
        DEBUG_INFO("[OTA] No valid config found, initializing...");
        memset(&ota_ctx.config, 0, sizeof(ota_config_t));
        ota_ctx.config.magic = OTA_CONFIG_MAGIC;
        ota_ctx.config.version = 1;
        ota_ctx.config.active_bank = 0;
        ota_ctx.config.app_a_valid = 1;
        ota_ctx.config.app_b_valid = 0;
        ota_ctx.config.max_boot_count = 3;
        ota_ctx.config.boot_count = 0;
    }

    ota_ctx.state = OTA_STATE_IDLE;
    ota_ctx.initialized = true;

    DEBUG_INFO("[OTA] Active bank: %c", ota_ctx.config.active_bank == 0 ? 'A' : 'B');
    DEBUG_INFO("[OTA] App A: %s (%lu bytes, CRC=0x%08lX)",
               ota_ctx.config.app_a_valid ? "valid" : "invalid",
               (unsigned long)ota_ctx.config.app_a_size,
               (unsigned long)ota_ctx.config.app_a_crc32);
    DEBUG_INFO("[OTA] App B: %s (%lu bytes, CRC=0x%08lX)",
               ota_ctx.config.app_b_valid ? "valid" : "invalid",
               (unsigned long)ota_ctx.config.app_b_size,
               (unsigned long)ota_ctx.config.app_b_crc32);

    return true;
}

/* ===========================================================================
 * OTA COMMAND HANDLER
 * =========================================================================== */

static void ota_write_config(void)
{
    OTA_QSPI_EraseSector(OTA_ADDR_CONFIG);
    OTA_QSPI_WritePage(OTA_ADDR_CONFIG, (const uint8_t *)&ota_ctx.config, sizeof(ota_config_t));
}

void OTA_HandleCommand(ros2_ota_msg_t *msg)
{
    if (!msg) return;

    switch (msg->cmd) {
        case OTA_CMD_BEGIN: {
            DEBUG_INFO("[OTA] BEGIN: size=%lu, crc=0x%08lX",
                       (unsigned long)msg->size, (unsigned long)msg->crc32);

            if (msg->size == 0 || msg->size > OTA_SIZE_APP_B) {
                ROS2_SendOTAResponse("error_size", msg->size);
                break;
            }

            /* Exit XIP mode - we need to write to QSPI */
            OTA_QSPI_ExitXIP();

            ota_ctx.state = OTA_STATE_ERASING;
            ota_ctx.total_size = msg->size;
            ota_ctx.received_size = 0;
            ota_ctx.write_addr = OTA_ADDR_APP_B;
            ota_ctx.expected_crc32 = msg->crc32;
            ota_ctx.calculated_crc32 = 0xFFFFFFFF;
            ota_ctx.seq = 0;

            DEBUG_INFO("[OTA] Erasing App B region (%lu KB)...", (unsigned long)(msg->size / 1024 + 1));

            uint32_t sectors = (msg->size + OTA_ERASE_SECTOR_SIZE - 1) / OTA_ERASE_SECTOR_SIZE;
            for (uint32_t i = 0; i < sectors; i++) {
                if (!OTA_QSPI_EraseSector(OTA_ADDR_APP_B + i * OTA_ERASE_SECTOR_SIZE)) {
                    ota_ctx.state = OTA_STATE_ERROR;
                    ROS2_SendOTAResponse("error_erase", i);
                    return;
                }
                if (i % 16 == 0) vTaskDelay(1);
            }

            ota_ctx.state = OTA_STATE_RECEIVING;
            ota_ctx.config.app_b_valid = 0;
            ota_write_config();

            ROS2_SendOTAResponse("erased", msg->size);
            DEBUG_INFO("[OTA] Erase complete, ready to receive");
            break;
        }

        case OTA_CMD_DATA: {
            if (ota_ctx.state != OTA_STATE_RECEIVING) {
                ROS2_SendOTAResponse("error_state", 0);
                break;
            }

            if (msg->seq != ota_ctx.seq) {
                ROS2_SendOTAResponse("error_seq", msg->seq);
                break;
            }

            if (!OTA_QSPI_WritePage(ota_ctx.write_addr, msg->data, msg->data_len)) {
                ota_ctx.state = OTA_STATE_ERROR;
                ROS2_SendOTAResponse("error_write", msg->seq);
                break;
            }

            ota_ctx.write_addr += msg->data_len;
            ota_ctx.received_size += msg->data_len;
            ota_ctx.seq++;

            ROS2_SendOTAResponse("ok", ota_ctx.received_size);
            break;
        }

        case OTA_CMD_VERIFY: {
            if (ota_ctx.state != OTA_STATE_RECEIVING) {
                ROS2_SendOTAResponse("error_state", 0);
                break;
            }

            ota_ctx.state = OTA_STATE_VERIFYING;
            DEBUG_INFO("[OTA] Verifying firmware (CRC32)...");

            uint32_t actual_crc = OTA_CalcCRC32_File(OTA_ADDR_APP_B, ota_ctx.total_size);

            if (actual_crc == ota_ctx.expected_crc32) {
                ota_ctx.config.app_b_crc32 = actual_crc;
                ota_ctx.config.app_b_size = ota_ctx.total_size;
                ota_ctx.config.app_b_valid = 1;
                ota_ctx.config.boot_count = 0;
                ota_write_config();

                ota_ctx.state = OTA_STATE_IDLE;
                ROS2_SendOTAResponse("verified", actual_crc);
                DEBUG_INFO("[OTA] CRC verified: 0x%08lX OK", (unsigned long)actual_crc);
            } else {
                ota_ctx.state = OTA_STATE_ERROR;
                ROS2_SendOTAResponse("error_crc", actual_crc);
                DEBUG_ERR("[OTA] CRC mismatch: expected 0x%08lX, got 0x%08lX",
                          (unsigned long)ota_ctx.expected_crc32, (unsigned long)actual_crc);
            }
            break;
        }

        case OTA_CMD_REBOOT: {
            if (!ota_ctx.config.app_b_valid) {
                ROS2_SendOTAResponse("error_nofirmware", 0);
                break;
            }

            ota_ctx.config.active_bank = 1;
            ota_ctx.config.boot_count = 0;
            ota_write_config();

            ROS2_SendOTAResponse("rebooting", 0);
            DEBUG_INFO("[OTA] Rebooting into App B...");
            vTaskDelay(pdMS_TO_TICKS(200));

            OTA_Reboot();
            break;
        }

        case OTA_CMD_ABORT: {
            ota_ctx.state = OTA_STATE_IDLE;
            ota_ctx.config.app_b_valid = 0;
            ota_write_config();
            ROS2_SendOTAResponse("aborted", 0);
            DEBUG_INFO("[OTA] Update aborted");
            break;
        }

        case OTA_CMD_STATUS: {
            uint32_t pct = 0;
            if (ota_ctx.total_size > 0)
                pct = (ota_ctx.received_size * 100) / ota_ctx.total_size;

            const char *state_str;
            switch (ota_ctx.state) {
                case OTA_STATE_IDLE:      state_str = "idle"; break;
                case OTA_STATE_ERASING:   state_str = "erasing"; break;
                case OTA_STATE_RECEIVING: state_str = "receiving"; break;
                case OTA_STATE_VERIFYING: state_str = "verifying"; break;
                case OTA_STATE_REBOOTING: state_str = "rebooting"; break;
                case OTA_STATE_ERROR:     state_str = "error"; break;
                default:                  state_str = "unknown"; break;
            }

            char json[128];
            snprintf(json, sizeof(json),
                "{\"t\":\"o\",\"s\":\"%s\",\"recv\":%lu,\"tot\":%lu,\"pct\":%lu}",
                state_str,
                (unsigned long)ota_ctx.received_size,
                (unsigned long)ota_ctx.total_size,
                (unsigned long)pct);
            ROS2_SendRaw(json);
            break;
        }
    }
}

/* ===========================================================================
 * REBOOT
 * =========================================================================== */

void OTA_Reboot(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void OTA_MarkValid(void)
{
    if (!ota_ctx.initialized) return;

    if (ota_ctx.config.active_bank == 0) {
        ota_ctx.config.app_a_valid = 1;
    } else {
        ota_ctx.config.app_b_valid = 1;
    }
    ota_ctx.config.boot_count = 0;
    ota_write_config();
}

ota_state_t OTA_GetState(void)
{
    return ota_ctx.state;
}

const ota_context_t* OTA_GetContext(void)
{
    return &ota_ctx;
}
