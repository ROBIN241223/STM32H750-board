/**
 * @file ros2_comm.c
 * @brief ROS2 UART Communication - JSON protocol over USART2
 */

#include "ros2_comm.h"
#include "ros2_motor.h"
#include "flight_controller.h"
#include "fdcan_comm.h"
#include "ota_update.h"
#include "sd_logger.h"
#include "debug_uart.h"
#include "gps_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ===========================================================================
 * PRIVATE VARIABLES
 * =========================================================================== */

extern UART_HandleTypeDef huart2;

static uint8_t rx_buffer[ROS2_RX_BUFFER_SIZE];
static volatile uint32_t rx_head = 0;
static volatile uint32_t rx_count = 0;

static ros2_tx_msg_t tx_msg;

/* Message queue for parsed messages (from ISR context to task) */
static QueueHandle_t rx_msg_queue = NULL;

/* ===========================================================================
 * CRC16 (CRC-CCITT)
 * =========================================================================== */

uint16_t ros2_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ===========================================================================
 * MINIMAL JSON PARSER
 * =========================================================================== */

static const char* json_find_key(const char *json, const char *key)
{
    char search[32];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *p = strstr(json, search);
    if (!p) return NULL;
    p += strlen(search);
    while (*p == ' ' || *p == ':') p++;
    return p;
}

static int json_get_int(const char *json, const char *key, int default_val)
{
    const char *p = json_find_key(json, key);
    if (!p) return default_val;
    return atoi(p);
}

static float json_get_float(const char *json, const char *key, float default_val)
{
    const char *p = json_find_key(json, key);
    if (!p) return default_val;
    return strtof(p, NULL);
}

static const char* json_get_string(const char *json, const char *key, char *buf, uint32_t buf_size)
{
    const char *p = json_find_key(json, key);
    if (!p) return NULL;
    if (*p != '"') return NULL;
    p++;
    uint32_t i = 0;
    while (*p && *p != '"' && i < buf_size - 1) {
        buf[i++] = *p++;
    }
    buf[i] = '\0';
    return buf;
}

static void json_get_int_array(const char *json, const char *key, int *arr, uint32_t max_count)
{
    const char *p = json_find_key(json, key);
    if (!p || *p != '[') return;
    p++;
    for (uint32_t i = 0; i < max_count; i++) {
        while (*p == ' ') p++;
        if (*p == ']' || *p == '\0') break;
        arr[i] = atoi(p);
        while (*p && *p != ',' && *p != ']') p++;
        if (*p == ',') p++;
    }
}

/* ===========================================================================
 * PARSE INCOMING JSON MESSAGE
 * =========================================================================== */

static bool parse_message(const char *json, ros2_parsed_msg_t *msg)
{
    char type_str[8] = {0};
    if (!json_get_string(json, "t", type_str, sizeof(type_str)))
        return false;

    if (strcmp(type_str, "m") == 0) {
        msg->type = MSG_TYPE_CMD_MOTOR;
        msg->motor.armed = (json_get_int(json, "a", 0) == 1);
        msg->motor.motor[0] = (uint8_t)json_get_int(json, "m1", 0);
        msg->motor.motor[1] = (uint8_t)json_get_int(json, "m2", 0);
        msg->motor.motor[2] = (uint8_t)json_get_int(json, "m3", 0);
        msg->motor.motor[3] = (uint8_t)json_get_int(json, "m4", 0);
        return true;
    }
    else if (strcmp(type_str, "fc") == 0) {
        msg->type = MSG_TYPE_CMD_FLIGHT;
        msg->flight.armed = (json_get_int(json, "a", 0) == 1);
        msg->flight.thrust_norm = json_get_float(json, "th", 0.0f);
        msg->flight.roll_rad = json_get_float(json, "r", 0.0f);
        msg->flight.pitch_rad = json_get_float(json, "p", 0.0f);
        msg->flight.yaw_rate_rad_s = json_get_float(json, "y", 0.0f);
        if (!isfinite(msg->flight.thrust_norm) ||
            !isfinite(msg->flight.roll_rad) ||
            !isfinite(msg->flight.pitch_rad) ||
            !isfinite(msg->flight.yaw_rate_rad_s)) {
            return false;
        }
        if (msg->flight.thrust_norm < 0.0f) msg->flight.thrust_norm = 0.0f;
        if (msg->flight.thrust_norm > 1.0f) msg->flight.thrust_norm = 1.0f;
        return true;
    }
    else if (strcmp(type_str, "g") == 0) {
        msg->type = MSG_TYPE_CMD_GPIO;
        msg->gpio.pin = json_get_int(json, "pin", -1);
        msg->gpio.value = json_get_int(json, "val", 0);
        return true;
    }
    else if (strcmp(type_str, "c") == 0) {
        msg->type = MSG_TYPE_CMD_FDCAN_TX;
        msg->fdcan.id = (uint32_t)json_get_int(json, "id", 0x100);
        msg->fdcan.dlc = (uint8_t)json_get_int(json, "dlc", 8);
        int data[8] = {0};
        json_get_int_array(json, "data", data, 8);
        for (int i = 0; i < 8; i++) msg->fdcan.data[i] = (uint8_t)data[i];
        return true;
    }
    else if (strcmp(type_str, "l") == 0) {
        msg->type = MSG_TYPE_CMD_SDLOG;
        msg->sdlog.start = (json_get_int(json, "start", 0) == 1);
        return true;
    }
    else if (strcmp(type_str, "o") == 0) {
        msg->type = MSG_TYPE_CMD_OTA;
        char cmd_str[16] = {0};
        json_get_string(json, "cmd", cmd_str, sizeof(cmd_str));
        if (strcmp(cmd_str, "begin") == 0) {
            msg->ota.cmd = OTA_CMD_BEGIN;
            msg->ota.size = (uint32_t)json_get_int(json, "size", 0);
            msg->ota.crc32 = (uint32_t)json_get_int(json, "crc", 0);
        } else if (strcmp(cmd_str, "data") == 0) {
            msg->ota.cmd = OTA_CMD_DATA;
            msg->ota.seq = (uint32_t)json_get_int(json, "seq", 0);
            /* Parse hex string data */
            const char *hex = json_find_key(json, "data");
            if (hex && *hex == '"') {
                hex++;
                uint8_t i = 0;
                while (*hex && *hex != '"' && i < 254) {
                    uint8_t hi = 0, lo = 0;
                    if (*hex >= '0' && *hex <= '9') hi = *hex - '0';
                    else if (*hex >= 'a' && *hex <= 'f') hi = *hex - 'a' + 10;
                    else if (*hex >= 'A' && *hex <= 'F') hi = *hex - 'A' + 10;
                    hex++;
                    if (*hex >= '0' && *hex <= '9') lo = *hex - '0';
                    else if (*hex >= 'a' && *hex <= 'f') lo = *hex - 'a' + 10;
                    else if (*hex >= 'A' && *hex <= 'F') lo = *hex - 'A' + 10;
                    hex++;
                    msg->ota.data[i++] = (hi << 4) | lo;
                }
                msg->ota.data_len = i;
            }
        } else if (strcmp(cmd_str, "verify") == 0) {
            msg->ota.cmd = OTA_CMD_VERIFY;
        } else if (strcmp(cmd_str, "reboot") == 0) {
            msg->ota.cmd = OTA_CMD_REBOOT;
        } else if (strcmp(cmd_str, "abort") == 0) {
            msg->ota.cmd = OTA_CMD_ABORT;
        } else if (strcmp(cmd_str, "status") == 0) {
            msg->ota.cmd = OTA_CMD_STATUS;
        } else {
            return false;
        }
        return true;
    }
    else if (strcmp(type_str, "r") == 0) {
        msg->type = MSG_TYPE_CMD_REQUEST;
        return true;
    }

    return false;
}

/* ===========================================================================
 * PUBLIC FUNCTIONS
 * =========================================================================== */

HAL_StatusTypeDef ROS2_Comm_Init(void)
{
    rx_head = 0;
    rx_count = 0;
    memset(&tx_msg, 0, sizeof(tx_msg));

    rx_msg_queue = xQueueCreate(8, sizeof(ros2_parsed_msg_t));

    /* Start interrupt-based RX on USART2 */
    HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_head], 1);

    DEBUG_INFO("[ROS2] USART2 comm initialized (115200 8N1)");
    DEBUG_INFO("[ROS2] TX: PD5, RX: PA3");

    return ret;
}

uint32_t ROS2_Comm_Process(void)
{
    uint32_t processed = 0;
    char line[ROS2_JSON_MAX_LEN];

    /* Process received bytes into lines */
    while (rx_count > 0 && processed < ROS2_MAX_MSG_PER_TICK) {
        /* Extract one line from ring buffer */
        uint32_t line_len = 0;
        bool line_complete = false;

        uint32_t tail = (rx_head - rx_count + ROS2_RX_BUFFER_SIZE) % ROS2_RX_BUFFER_SIZE;
        uint32_t remaining = rx_count;

        while (remaining > 0 && line_len < ROS2_JSON_MAX_LEN - 1) {
            uint8_t c = rx_buffer[tail];
            tail = (tail + 1) % ROS2_RX_BUFFER_SIZE;
            remaining--;

            if (c == '\n' || c == '\r') {
                if (line_len > 0) {
                    line_complete = true;
                    rx_count -= (line_len + 1);
                    break;
                }
                rx_count--;
                continue;
            }
            line[line_len++] = c;
        }

        if (!line_complete) break;

        line[line_len] = '\0';

        /* Parse JSON */
        ros2_parsed_msg_t msg;
        if (parse_message(line, &msg)) {
            /* Dispatch to appropriate module */
            switch (msg.type) {
                case MSG_TYPE_CMD_MOTOR:
                    ROS2_Motor_EnqueueCmd(&msg.motor);
                    break;

                case MSG_TYPE_CMD_FLIGHT: {
                    fc_setpoint_t setpoint;
                    setpoint.armed = msg.flight.armed;
                    setpoint.thrust_norm = msg.flight.thrust_norm;
                    setpoint.roll_rad = msg.flight.roll_rad;
                    setpoint.pitch_rad = msg.flight.pitch_rad;
                    setpoint.yaw_rate_rad_s = msg.flight.yaw_rate_rad_s;
                    setpoint.timestamp_ms = osKernelGetTickCount();
                    FC_Setpoint_Update(&setpoint);
                    break;
                }

                case MSG_TYPE_CMD_FDCAN_TX: {
                    fdcan_tx_msg_t tx;
                    tx.id = msg.fdcan.id;
                    tx.dlc = msg.fdcan.dlc;
                    memcpy(tx.data, msg.fdcan.data, 8);
                    FDCAN_Comm_EnqueueTx(&tx);
                    break;
                }

                case MSG_TYPE_CMD_SDLOG:
                    if (msg.sdlog.start)
                        SD_Logger_Start();
                    else
                        SD_Logger_Stop();
                    break;

                case MSG_TYPE_CMD_OTA:
                    OTA_HandleCommand(&msg.ota);
                    break;

                case MSG_TYPE_CMD_GPIO:
                    /* TODO: GPIO control */
                    break;

                case MSG_TYPE_CMD_REQUEST:
                    /* TODO: on-demand sensor read */
                    break;

                default:
                    break;
            }
            processed++;
        }
    }

    return processed;
}

/* ===========================================================================
 * TX FUNCTIONS
 * =========================================================================== */

static void send_json(const char *json)
{
    uint32_t len = strlen(json);
    if (len + 2 > ROS2_TX_BUFFER_SIZE) return;

    memcpy(tx_msg.buffer, json, len);
    tx_msg.buffer[len] = '\n';
    tx_msg.buffer[len + 1] = '\0';

    HAL_UART_Transmit(&huart2, (uint8_t *)tx_msg.buffer, len + 1, 100);
}

void ROS2_SendSensorData(float adc[4], float vbat, float temp, uint32_t timestamp)
{
    char json[256];
    snprintf(json, sizeof(json),
        "{\"t\":\"s\",\"ts\":%lu,\"a\":[%.0f,%.0f,%.0f,%.0f],\"v\":%.0f,\"tp\":%.1f}",
        (unsigned long)timestamp,
        adc[0], adc[1], adc[2], adc[3],
        vbat, temp);
    send_json(json);
}

void ROS2_SendStatus(uint32_t uptime_ms, uint32_t free_heap, uint8_t boot_bank)
{
    char json[192];
    snprintf(json, sizeof(json),
        "{\"t\":\"h\",\"up\":%lu,\"heap\":%lu,\"bank\":%d,\"fw\":\"1.0.0\"}",
        (unsigned long)uptime_ms,
        (unsigned long)free_heap,
        boot_bank);
    send_json(json);
}

void ROS2_SendFDCANRx(uint32_t id, uint8_t *data, uint8_t dlc)
{
    char json[192];
    char hex_data[20] = {0};
    uint8_t hex_idx = 0;
    for (uint8_t i = 0; i < dlc && i < 8; i++) {
        hex_idx += snprintf(hex_data + hex_idx, sizeof(hex_data) - hex_idx, "%02X", data[i]);
    }
    snprintf(json, sizeof(json),
        "{\"t\":\"f\",\"id\":%lu,\"dlc\":%d,\"d\":\"%s\"}",
        (unsigned long)id, dlc, hex_data);
    send_json(json);
}

void ROS2_SendOTAResponse(const char *status, uint32_t param)
{
    char json[128];
    snprintf(json, sizeof(json),
        "{\"t\":\"o\",\"s\":\"%s\",\"p\":%lu}",
        status, (unsigned long)param);
    send_json(json);
}

void ROS2_SendRaw(const char *json)
{
    send_json(json);
}

/* ===========================================================================
 * RX QUEUE ACCESS
 * =========================================================================== */

int ROS2_DequeueMessage(ros2_parsed_msg_t *msg)
{
    if (!rx_msg_queue) return 0;
    return (xQueueReceive(rx_msg_queue, msg, 0) == pdTRUE) ? 1 : 0;
}

/* ===========================================================================
 * MERGED HAL CALLBACKS (USART1 debug + USART2 ROS2)
 * HAL only allows one callback function, so we dispatch here.
 * =========================================================================== */

extern UART_HandleTypeDef huart1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        /* Debug UART: read byte and feed into debug ring buffer */
        uint8_t byte = huart->pRxBuffPtr[0];
        Debug_RxByte(byte);
        Debug_RearmRx();
    }
    else if (huart->Instance == USART2) {
        /* ROS2 UART: read byte and feed into ROS2 ring buffer */
        uint8_t byte = huart->pRxBuffPtr[0];
        rx_buffer[rx_head] = byte;
        rx_head = (rx_head + 1) % ROS2_RX_BUFFER_SIZE;
        rx_count++;
        if (rx_count > ROS2_RX_BUFFER_SIZE)
            rx_count = ROS2_RX_BUFFER_SIZE;
        HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_head], 1);
    }
    else if (huart->Instance == USART3) {
        /* GNSS UART: the GPS port owns the ring and the re-arm */
        uint8_t byte = huart->pRxBuffPtr[0];
        GPS_Port_RxByte(byte);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* TX complete - placeholder for both UARTs */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        Debug_RearmRx();
    }
    else if (huart->Instance == USART2) {
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_head], 1);
    }
    else if (huart->Instance == USART3) {
        GPS_Port_UartRearm();
    }
}
