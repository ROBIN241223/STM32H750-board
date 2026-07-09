/**
 * @file debug_uart.c
 * @brief Debug UART Interface Implementation
 */

#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"

/* ===========================================================================
 * PRIVATE VARIABLES
 * =========================================================================== */

static uint8_t debug_rx_buffer[DEBUG_RX_BUFFER_SIZE];
static uint8_t debug_tx_buffer[DEBUG_TX_BUFFER_SIZE];
static uint32_t debug_rx_head = 0;
static uint32_t debug_rx_tail = 0;
static volatile uint32_t debug_rx_count = 0;

extern UART_HandleTypeDef huart1;  /* USART1 handle from generated code */

/* ===========================================================================
 * IMPLEMENTATION
 * =========================================================================== */

/**
 * @brief Initialize debug UART interface
 */
HAL_StatusTypeDef Debug_UART_Init(void)
{
    /* USART1 is already initialized by HAL_Init() in main.c */
    /* Start receiving data in interrupt mode */
    HAL_UART_Receive_IT(&huart1, debug_rx_buffer, 1);
    
    DEBUG_LOG("=== STM32H750 Flight Controller Debug UART Initialized ===");
    DEBUG_INFO("USART1: 115200 8N1");
    DEBUG_INFO("TX: PA9, RX: PA10");
    
    return HAL_OK;
}

/**
 * @brief Printf-style formatted output
 */
void Debug_Printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    
    int len = vsnprintf((char *)debug_tx_buffer, DEBUG_TX_BUFFER_SIZE, format, args);
    va_end(args);
    
    if (len > 0 && len < DEBUG_TX_BUFFER_SIZE)
    {
        HAL_UART_Transmit(&huart1, debug_tx_buffer, len, HAL_MAX_DELAY);
    }
}

/**
 * @brief Send raw data via UART
 */
uint32_t Debug_Send(const uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0)
        return 0;
    
    if (HAL_UART_Transmit(&huart1, (uint8_t *)data, size, HAL_MAX_DELAY) == HAL_OK)
        return size;
    
    return 0;
}

/**
 * @brief Receive data (non-blocking)
 */
uint32_t Debug_Receive(uint8_t *data, uint32_t max_size)
{
    if (data == NULL || max_size == 0)
        return 0;
    
    uint32_t available = debug_rx_count;
    if (available > max_size)
        available = max_size;
    
    uint32_t i = 0;
    while (available > 0 && debug_rx_count > 0)
    {
        data[i++] = debug_rx_buffer[debug_rx_tail];
        debug_rx_tail = (debug_rx_tail + 1) % DEBUG_RX_BUFFER_SIZE;
        debug_rx_count--;
        available--;
    }
    
    return i;
}

/**
 * @brief Check available data
 */
uint32_t Debug_Available(void)
{
    return debug_rx_count;
}

/**
 * @brief Print hex dump
 */
void Debug_PrintHex(const uint8_t *data, uint32_t size)
{
    if (data == NULL || size == 0)
        return;

    for (uint32_t i = 0; i < size; i++)
    {
        if (i % 16 == 0 && i > 0)
            Debug_Printf("\r\n");

        Debug_Printf("%02X ", data[i]);
    }
    Debug_Printf("\r\n");
}

/**
 * @brief Feed a received byte into debug UART ring buffer
 */
void Debug_RxByte(uint8_t byte)
{
    debug_rx_buffer[debug_rx_head] = byte;
    debug_rx_head = (debug_rx_head + 1) % DEBUG_RX_BUFFER_SIZE;
    debug_rx_count++;
    if (debug_rx_count > DEBUG_RX_BUFFER_SIZE)
        debug_rx_count = DEBUG_RX_BUFFER_SIZE;
}

/**
 * @brief Re-arm debug UART RX interrupt
 */
void Debug_RearmRx(void)
{
    HAL_UART_Receive_IT(&huart1, &debug_rx_buffer[debug_rx_head], 1);
}

/**
 * @brief Test debug output
 */
void Debug_Test(void)
{
    DEBUG_INFO("=== Debug UART Test ===");
    DEBUG_INFO("System Clock: %lu MHz", SystemCoreClock / 1000000);
    DEBUG_INFO("FreeRTOS Heap: %lu bytes", configTOTAL_HEAP_SIZE);
    DEBUG_INFO("Tick Rate: %lu Hz", configTICK_RATE_HZ);
    
    DEBUG_WARN("This is a warning message");
    DEBUG_ERR("This is an error message");
    
    uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    DEBUG_INFO("Hex dump test:");
    Debug_PrintHex(test_data, sizeof(test_data));
    
    DEBUG_INFO("=== Test Complete ===\r\n");
}

/* ===========================================================================
 * HAL CALLBACKS
 * Note: HAL_UART_RxCpltCallback, HAL_UART_TxCpltCallback, HAL_UART_ErrorCallback
 * are implemented in ros2_comm.c as a merged handler for both USART1 and USART2.
 * =========================================================================== */

/* ===========================================================================
 * LIBC INTEGRATION - Redirect printf() to Debug UART
 * =========================================================================== */

int _write(int file, char *ptr, int len)
{
    if (file == 1 || file == 2)  /* stdout or stderr */
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
        return len;
    }
    return -1;
}

int _read(int file, char *ptr, int len)
{
    if (file == 0)  /* stdin */
    {
        uint32_t available = Debug_Available();
        if (available > 0)
        {
            len = (available < len) ? available : len;
            Debug_Receive((uint8_t *)ptr, len);
            return len;
        }
    }
    return 0;
}
