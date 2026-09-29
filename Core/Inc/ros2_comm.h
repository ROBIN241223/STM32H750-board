/**
 * @file ros2_comm.h
 * @brief ROS2 UART Communication Protocol for STM32 <-> Raspberry Pi
 * @note  Uses USART2 (PD5-TX, PA3-RX) at 115200 baud
 *        Protocol: JSON lines terminated by \n, CRC16 appended
 */

#ifndef ROS2_COMM_H
#define ROS2_COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * CONFIGURATION
 * =========================================================================== */

#define ROS2_UART               huart2
#define ROS2_BAUDRATE           115200
#define ROS2_RX_BUFFER_SIZE     512
#define ROS2_TX_BUFFER_SIZE     1024
#define ROS2_JSON_MAX_LEN       256
#define ROS2_MAX_MSG_PER_TICK   4

/* ===========================================================================
 * MESSAGE TYPES
 * =========================================================================== */

typedef enum {
    MSG_TYPE_UNKNOWN = 0,
    MSG_TYPE_SENSOR,        /* {"t":"s",...} STM32 -> Pi */
    MSG_TYPE_STATUS,        /* {"t":"h",...} STM32 -> Pi (heartbeat) */
    MSG_TYPE_FDCAN_RX,      /* {"t":"f",...} STM32 -> Pi */
    MSG_TYPE_CMD_MOTOR,     /* {"t":"m",...} Pi -> STM32 */
    MSG_TYPE_CMD_FLIGHT,    /* {"t":"fc",...} Pi -> STM32 */
    MSG_TYPE_CMD_GPIO,      /* {"t":"g",...} Pi -> STM32 */
    MSG_TYPE_CMD_FDCAN_TX,  /* {"t":"c",...} Pi -> STM32 */
    MSG_TYPE_CMD_SDLOG,     /* {"t":"l",...} Pi -> STM32 */
    MSG_TYPE_CMD_OTA,       /* {"t":"o",...} Pi -> STM32 */
    MSG_TYPE_CMD_REQUEST,   /* {"t":"r",...} Pi -> STM32 */
} ros2_msg_type_t;

/* ===========================================================================
 * PARSED MESSAGE STRUCTURES
 * =========================================================================== */

typedef struct {
    float adc[4];
    float vbat;
    float temp;
    uint32_t timestamp;
} ros2_sensor_msg_t;

typedef struct {
    uint8_t motor[4];       /* PWM values 0-255 -> mapped to timer */
    bool armed;
} ros2_motor_msg_t;

typedef struct {
    bool armed;
    float thrust_norm;
    float roll_rad;
    float pitch_rad;
    float yaw_rate_rad_s;
} ros2_fc_cmd_t;

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
} ros2_fdcan_msg_t;

typedef struct {
    int pin;
    int value;
} ros2_gpio_msg_t;

typedef struct {
    bool start;             /* true=start logging, false=stop */
} ros2_sdlog_msg_t;

typedef struct {
    enum {
        OTA_CMD_BEGIN,
        OTA_CMD_DATA,
        OTA_CMD_VERIFY,
        OTA_CMD_REBOOT,
        OTA_CMD_ABORT,
        OTA_CMD_STATUS,
    } cmd;
    uint32_t size;          /* firmware size in bytes (BEGIN) */
    uint32_t crc32;         /* expected CRC32 (BEGIN) */
    uint32_t seq;           /* sequence number (DATA) */
    uint8_t data[256];      /* firmware chunk (DATA) */
    uint8_t data_len;       /* actual data length in chunk */
} ros2_ota_msg_t;

typedef struct {
    ros2_msg_type_t type;
    union {
        ros2_motor_msg_t motor;
        ros2_fc_cmd_t flight;
        ros2_fdcan_msg_t fdcan;
        ros2_gpio_msg_t gpio;
        ros2_sdlog_msg_t sdlog;
        ros2_ota_msg_t ota;
    };
} ros2_parsed_msg_t;

/* ===========================================================================
 * TX MESSAGE BUILDER
 * =========================================================================== */

typedef struct {
    char buffer[ROS2_TX_BUFFER_SIZE];
    uint16_t len;
} ros2_tx_msg_t;

/* ===========================================================================
 * INITIALIZATION
 * =========================================================================== */

/**
 * @brief Initialize ROS2 communication on USART2
 * @return HAL status
 */
HAL_StatusTypeDef ROS2_Comm_Init(void);

/**
 * @brief Main processing call - call from FreeRTOS task
 *        Reads UART, parses JSON, dispatches to appropriate handler
 * @return Number of messages processed
 */
uint32_t ROS2_Comm_Process(void);

/* ===========================================================================
 * TX FUNCTIONS (STM32 -> Pi)
 * =========================================================================== */

/**
 * @brief Send sensor data message
 */
void ROS2_SendSensorData(float adc[4], float vbat, float temp, uint32_t timestamp);

/**
 * @brief Send heartbeat/status message
 */
void ROS2_SendStatus(uint32_t uptime_ms, uint32_t free_heap, uint8_t boot_bank);

/**
 * @brief Send FDCAN received message
 */
void ROS2_SendFDCANRx(uint32_t id, uint8_t *data, uint8_t dlc);

/**
 * @brief Send OTA status response
 */
void ROS2_SendOTAResponse(const char *status, uint32_t param);

/**
 * @brief Raw send a JSON string (adds \n terminator)
 */
void ROS2_SendRaw(const char *json);

/**
 * @brief Dequeue a parsed message from the RX queue (called from main task)
 * @param msg Pointer to message struct to fill
 * @return 1 if message dequeued, 0 if empty
 */
int ROS2_DequeueMessage(ros2_parsed_msg_t *msg);

/* ===========================================================================
 * CRC16 UTILITIES
 * =========================================================================== */

uint16_t ros2_crc16(const uint8_t *data, uint16_t len);

/* ===========================================================================
 * UART CALLBACKS (called from HAL)
 * =========================================================================== */

void ROS2_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void ROS2_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* ROS2_COMM_H */
