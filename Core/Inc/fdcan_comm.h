/**
 * @file fdcan_comm.h
 * @brief FDCAN2 communication module - TX/RX with ROS2 bridge
 */

#ifndef FDCAN_COMM_H
#define FDCAN_COMM_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===========================================================================
 * CONFIGURATION
 * =========================================================================== */

#define FDCAN_RX_QUEUE_SIZE     32
#define FDCAN_TX_QUEUE_SIZE     16
#define FDCAN_DEFAULT_ID        0x100

/* ===========================================================================
 * DATA STRUCTURES
 * =========================================================================== */

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
    uint32_t timestamp_ms;
} fdcan_rx_msg_t;

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t dlc;
} fdcan_tx_msg_t;

/* ===========================================================================
 * FUNCTIONS
 * =========================================================================== */

/**
 * @brief Initialize FDCAN2 with filter and start RX
 */
void FDCAN_Comm_Init(void);

/**
 * @brief Send a FDCAN message
 * @param msg Pointer to TX message
 * @return true if sent, false if busy
 */
bool FDCAN_Comm_Transmit(fdcan_tx_msg_t *msg);

/**
 * @brief Check if RX message available
 * @return Number of messages in RX queue
 */
uint32_t FDCAN_Comm_Available(void);

/**
 * @brief Dequeue a received FDCAN message
 * @param msg Pointer to RX message struct to fill
 * @return true if message dequeued, false if empty
 */
bool FDCAN_Comm_Receive(fdcan_rx_msg_t *msg);

/**
 * @brief FDCAN RX callback (HAL weak override, called from HAL ISR)
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

/**
 * @brief FreeRTOS task: process FDCAN RX queue, forward to ROS2
 */
void FDCAN_Comm_Task(void *argument);

/**
 * @brief Enqueue a FDCAN TX message from ROS2 communication task
 * @param msg TX message to enqueue
 */
void FDCAN_Comm_EnqueueTx(fdcan_tx_msg_t *msg);

#endif /* FDCAN_COMM_H */
