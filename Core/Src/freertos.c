/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS application tasks for ROS2 flight controller
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug_uart.h"
#include "ros2_comm.h"
#include "ros2_sensor.h"
#include "ros2_motor.h"
#include "flight_controller.h"
#include "imu_port.h"
#include "fdcan_comm.h"
#include "sd_logger.h"
#include "ota_update.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Task stack sizes (in words, multiply by 4 for bytes) */
#define TASK_STACK_SENSOR        (256)
#define TASK_STACK_MOTOR         (256)
#define TASK_STACK_FDCAN         (256)
#define TASK_STACK_SDLOG         (512)
#define TASK_STACK_ROS2_COMM     (512)
#define TASK_STACK_FLIGHT        (1024)

/* Task priorities */
#define TASK_PRIO_SENSOR         (osPriorityAboveNormal)
#define TASK_PRIO_MOTOR          (osPriorityHigh)
#define TASK_PRIO_ROS2_COMM      (osPriorityNormal)
#define TASK_PRIO_FDCAN          (osPriorityNormal)
#define TASK_PRIO_SDLOG          (osPriorityBelowNormal)

/* Heartbeat interval */
#define HEARTBEAT_INTERVAL_MS    1000

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static osThreadId_t ros2CommTaskHandle;
static osThreadId_t sensorTaskHandle;
static osThreadId_t motorTaskHandle;
static osThreadId_t fdcanTaskHandle;
static osThreadId_t sdlogTaskHandle;
static osThreadId_t flightTaskHandle;
static imu_port_t imu0_port;
static imu_port_t imu1_port;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void ROS2_Comm_Task(void *argument);
static void System_Heartbeat_Task(void *argument);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief ROS2 Communication main task
 *        Processes incoming UART messages and dispatches to modules
 */
static void ROS2_Comm_Task(void *argument)
{
    DEBUG_INFO("[ROS2] Communication task started");

    while (1) {
        ROS2_Comm_Process();
        vTaskDelay(pdMS_TO_TICKS(5));  /* ~200Hz processing rate */
    }
}

/**
 * @brief System heartbeat task
 *        Sends periodic status to ROS2 and checks system health
 */
static void System_Heartbeat_Task(void *argument)
{
    DEBUG_INFO("[Heartbeat] Task started (interval: %dms)", HEARTBEAT_INTERVAL_MS);

    /* Wait a bit before first heartbeat */
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        uint32_t now = osKernelGetTickCount();
        uint32_t uptime = now;

        /* Get free heap */
        uint32_t free_heap = (uint32_t)xPortGetFreeHeapSize();

        /* Get active bank from OTA config */
        const ota_context_t *ota = OTA_GetContext();
        uint8_t bank = (ota && ota->initialized) ? ota->config.active_bank : 0;

        /* Send heartbeat to ROS2 */
        ROS2_SendStatus(uptime, free_heap, bank);

        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
    }
}

/* USER CODE END Application */

/* =========================================================================== */
/* CubeMX-generated code below - DO NOT MODIFY                                 */
/* =========================================================================== */

extern osThreadId_t defaultTaskHandle;
extern const osThreadAttr_t defaultTask_attributes;

extern void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* Initialize modules */
    ROS2_Comm_Init();
    ROS2_Sensor_Init();
    ROS2_Motor_Init();
    IMU_Port_Null_Init(&imu0_port);
    IMU_Port_Null_Init(&imu1_port);
    (void)FC_Init(&imu0_port, &imu1_port);
    FDCAN_Comm_Init();
    OTA_Init();
    SD_Logger_Init();

    /* USER CODE END Init */

    /* Create the thread(s) */
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */

    /* Create application tasks */
    ros2CommTaskHandle = osThreadNew(
        ROS2_Comm_Task, NULL,
        &(const osThreadAttr_t){
            .name = "ros2Comm",
            .stack_size = TASK_STACK_ROS2_COMM * 4,
            .priority = TASK_PRIO_ROS2_COMM,
        });

    sensorTaskHandle = osThreadNew(
        ROS2_Sensor_Task, NULL,
        &(const osThreadAttr_t){
            .name = "sensor",
            .stack_size = TASK_STACK_SENSOR * 4,
            .priority = TASK_PRIO_SENSOR,
        });

    motorTaskHandle = osThreadNew(
        ROS2_Motor_Task, NULL,
        &(const osThreadAttr_t){
            .name = "motor",
            .stack_size = TASK_STACK_MOTOR * 4,
            .priority = TASK_PRIO_MOTOR,
        });

    flightTaskHandle = osThreadNew(
        FC_Task, NULL,
        &(const osThreadAttr_t){
            .name = "flight",
            .stack_size = TASK_STACK_FLIGHT * 4,
            .priority = TASK_PRIO_MOTOR,
        });

    fdcanTaskHandle = osThreadNew(
        FDCAN_Comm_Task, NULL,
        &(const osThreadAttr_t){
            .name = "fdcan",
            .stack_size = TASK_STACK_FDCAN * 4,
            .priority = TASK_PRIO_FDCAN,
        });

    sdlogTaskHandle = osThreadNew(
        SD_Logger_Task, NULL,
        &(const osThreadAttr_t){
            .name = "sdlog",
            .stack_size = TASK_STACK_SDLOG * 4,
            .priority = TASK_PRIO_SDLOG,
        });

    /* Start heartbeat as a bare FreeRTOS task (uses DEBUG UART) */
    xTaskCreate(System_Heartbeat_Task, "heartbeat", 256 * 4, NULL,
                 osPriorityBelowNormal, NULL);

    DEBUG_INFO("[RTOS] All tasks created");
    DEBUG_INFO("[RTOS] Tasks: ros2Comm, sensor, motor, flight, fdcan, sdlog, heartbeat, default");

    /* USER CODE END RTOS_THREADS */
}
