#include "gps_port.h"
#include "gps_parser.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* USART3 carries the GNSS receiver (USART1 is the debug console, USART2 the
 * ROS2 link). The interrupt only pushes bytes into a ring; the parser runs in
 * task context so the control loop never pays for framing inside an ISR and
 * so a partially received sentence can never be read half-parsed.
 */

#define GPS_UART_RX_RING 512U

extern UART_HandleTypeDef huart3;

static uint8_t rx_ring[GPS_UART_RX_RING];
static volatile uint32_t rx_head;
static volatile uint32_t rx_tail;
static volatile uint32_t rx_overruns;
static uint8_t rx_byte;
static gps_parser_t parser;
static fc_gps_sample_t latched;
static uint32_t latched_valid;
static uint32_t uart_started;

bool GPS_Port_UartInit(void)
{
    GPS_Parser_Init(&parser);
    memset(&latched, 0, sizeof(latched));
    latched_valid = 0U;
    rx_head = 0U;
    rx_tail = 0U;
    rx_overruns = 0U;
    if (uart_started != 0U) {
        return true;
    }
    if (HAL_UART_Receive_IT(&huart3, &rx_byte, 1U) != HAL_OK) {
        return false;
    }
    uart_started = 1U;
    return true;
}

void GPS_Port_RxByte(uint8_t byte)
{
    const uint32_t next = (rx_head + 1U) % GPS_UART_RX_RING;
    if (next == rx_tail) {
        /* Full: drop the byte rather than corrupt the stream. A receiver that
         * outruns us is a hardware or baud problem, and a truncated sentence
         * fails its checksum anyway. */
        rx_overruns++;
    } else {
        rx_ring[rx_head] = byte;
        rx_head = next;
    }
    /* Re-arm here so the receive buffer stays private to this module. */
    if (uart_started != 0U) {
        (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
    }
}

void GPS_Port_UartRearm(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    if (uart_started != 0U) {
        (void)HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
    }
}

uint32_t GPS_Port_UartOverruns(void)
{
    return rx_overruns;
}

static bool uart_init(void *context)
{
    (void)context;
    return GPS_Port_UartInit();
}

static gps_port_status_t uart_read(void *context, fc_gps_sample_t *sample)
{
    (void)context;
    uint32_t drained = 0U;
    bool got_fix = false;
    while (rx_tail != rx_head) {
        const uint8_t byte = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1U) % GPS_UART_RX_RING;
        if (GPS_Parser_Feed(&parser, byte)) {
            got_fix = true;
        }
        /* Bound the work per call: a 10 Hz receiver never needs more, and a
         * burst of traffic must not stall the control loop. */
        if (++drained >= 128U) {
            break;
        }
    }
    if (got_fix != 0U) {
        latched = parser.sample;
        latched_valid = 1U;
    }
    if (latched_valid == 0U) {
        return GPS_PORT_NO_DATA;
    }
    if (sample != NULL) {
        *sample = latched;
    }
    return GPS_PORT_OK;
}

static bool uart_healthy(void *context)
{
    (void)context;
    return (latched_valid != 0U) && (uart_started != 0U);
}

void GPS_Port_UartBind(gps_port_t *port)
{
    if (port == NULL) {
        return;
    }
    port->init = uart_init;
    port->read_latest = uart_read;
    port->healthy = uart_healthy;
    port->context = NULL;
}
