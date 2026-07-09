/**
 * @file fdcan_comm.c
 * @brief FDCAN2 communication - TX/RX with ROS2 bridge forwarding
 */

#include "fdcan_comm.h"
#include "ros2_comm.h"
#include "debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

/* ===========================================================================
 * EXTERNAL HANDLES
 * =========================================================================== */

extern FDCAN_HandleTypeDef hfdcan2;

/* ===========================================================================
 * PRIVATE VARIABLES
 * =========================================================================== */

static QueueHandle_t fdcan_rx_queue = NULL;
static QueueHandle_t fdcan_tx_queue = NULL;

/* ===========================================================================
 * INITIALIZATION
 * =========================================================================== */

void FDCAN_Comm_Init(void)
{
    fdcan_rx_queue = xQueueCreate(FDCAN_RX_QUEUE_SIZE, sizeof(fdcan_rx_msg_t));
    fdcan_tx_queue = xQueueCreate(FDCAN_TX_QUEUE_SIZE, sizeof(fdcan_tx_msg_t));

    /* Configure FDCAN2 RX filter to accept all standard IDs */
    FDCAN_FilterTypeDef filter;
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;  /* mask=0 accepts all IDs */
    HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);

    /* Enable FIFO0 new message interrupt */
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* Start FDCAN */
    HAL_FDCAN_Start(&hfdcan2);

    DEBUG_INFO("[FDCAN] FDCAN2 initialized (1.333 Mbit/s)");
    DEBUG_INFO("[FDCAN] TX: PB13, RX: PB5");
    DEBUG_INFO("[FDCAN] Filter: accept all standard IDs");
}

/* ===========================================================================
 * TRANSMIT
 * =========================================================================== */

bool FDCAN_Comm_Transmit(fdcan_tx_msg_t *msg)
{
    if (!msg) return false;

    FDCAN_TxHeaderTypeDef header;
    header.Identifier = msg->id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = msg->dlc;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, msg->data) == HAL_OK) {
        return true;
    }

    return false;
}

/* ===========================================================================
 * RECEIVE
 * =========================================================================== */

uint32_t FDCAN_Comm_Available(void)
{
    if (!fdcan_rx_queue) return 0;
    return (uint32_t)uxQueueMessagesWaiting(fdcan_rx_queue);
}

bool FDCAN_Comm_Receive(fdcan_rx_msg_t *msg)
{
    if (!fdcan_rx_queue || !msg) return false;
    return (xQueueReceive(fdcan_rx_queue, msg, 0) == pdTRUE);
}

/* ===========================================================================
 * HAL CALLBACK
 * =========================================================================== */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == FDCAN2) {
        FDCAN_RxHeaderTypeDef rx_header;
        fdcan_rx_msg_t rx_msg;

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_msg.data) == HAL_OK) {
            rx_msg.id = rx_header.Identifier;
            rx_msg.dlc = rx_header.DataLength;
            rx_msg.timestamp_ms = HAL_GetTick();

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(fdcan_rx_queue, &rx_msg, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* ===========================================================================
 * FREERTOS TASK
 * =========================================================================== */

void FDCAN_Comm_Task(void *argument)
{
    fdcan_rx_msg_t rx_msg;
    fdcan_tx_msg_t tx_msg;

    DEBUG_INFO("[FDCAN] Task started");

    while (1) {
        /* Forward FDCAN RX to ROS2 */
        while (FDCAN_Comm_Receive(&rx_msg)) {
            ROS2_SendFDCANRx(rx_msg.id, rx_msg.data, rx_msg.dlc);
        }

        /* Process FDCAN TX queue */
        if (xQueueReceive(fdcan_tx_queue, &tx_msg, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (!FDCAN_Comm_Transmit(&tx_msg)) {
                DEBUG_WARN("[FDCAN] TX failed for ID 0x%03lX", (unsigned long)tx_msg.id);
            }
        }
    }
}

/* ===========================================================================
 * QUEUE ACCESS (called from ros2_comm task)
 * =========================================================================== */

void FDCAN_Comm_EnqueueTx(fdcan_tx_msg_t *msg)
{
    if (fdcan_tx_queue && msg) {
        xQueueSend(fdcan_tx_queue, msg, 0);
    }
}
