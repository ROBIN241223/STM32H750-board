/**
 * @file debug_uart.h
 * @brief Debug UART Interface for PC Communication
 * @author Custom Flight Controller
 * @date 2026
 */

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ===========================================================================
 * CONFIGURATION
 * =========================================================================== */

/* Select which UART to use for debug output */
#define DEBUG_UART_DEVICE       USART1_Handle    /* USART1: PA9(TX), PA10(RX) @ 115200 */
#define DEBUG_UART_BAUDRATE     115200

/* Buffer sizes */
#define DEBUG_RX_BUFFER_SIZE    256
#define DEBUG_TX_BUFFER_SIZE    512

/* ===========================================================================
 * EXPORTED FUNCTIONS
 * =========================================================================== */

/**
 * @brief Initialize debug UART interface
 * @return HAL_OK if successful
 */
HAL_StatusTypeDef Debug_UART_Init(void);

/**
 * @brief Print formatted string via debug UART (like printf)
 * @param format Format string
 * @param ... Arguments
 */
void Debug_Printf(const char *format, ...);

/**
 * @brief Send raw data via debug UART
 * @param data Pointer to data buffer
 * @param size Size in bytes
 * @return Number of bytes sent
 */
uint32_t Debug_Send(const uint8_t *data, uint32_t size);

/**
 * @brief Receive data from debug UART (non-blocking)
 * @param data Pointer to receive buffer
 * @param max_size Maximum size to read
 * @return Number of bytes received
 */
uint32_t Debug_Receive(uint8_t *data, uint32_t max_size);

/**
 * @brief Check if data is available to read
 * @return Number of bytes available
 */
uint32_t Debug_Available(void);

/**
 * @brief Simple test routine for debug output
 */
void Debug_Test(void);

/* ===========================================================================
 * REDIRECT PRINTF TO DEBUG UART
 * =========================================================================== */

/* Macros for easy debug output */
#define DEBUG_LOG(fmt, ...) Debug_Printf("[LOG] " fmt "\r\n", ##__VA_ARGS__)
#define DEBUG_INFO(fmt, ...) Debug_Printf("[INF] " fmt "\r\n", ##__VA_ARGS__)
#define DEBUG_WARN(fmt, ...) Debug_Printf("[WRN] " fmt "\r\n", ##__VA_ARGS__)
#define DEBUG_ERR(fmt, ...) Debug_Printf("[ERR] " fmt "\r\n", ##__VA_ARGS__)
#define DEBUG_HEX(data, size) Debug_PrintHex(data, size)

/**
 * @brief Print hex dump
 * @param data Pointer to data
 * @param size Size in bytes
 */
void Debug_PrintHex(const uint8_t *data, uint32_t size);

#endif /* DEBUG_UART_H */
