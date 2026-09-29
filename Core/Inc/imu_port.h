#ifndef IMU_PORT_H
#define IMU_PORT_H

#include <stdbool.h>
#include "fc_types.h"

typedef enum {
    IMU_PORT_OK = 0,
    IMU_PORT_NO_DATA,
    IMU_PORT_NOT_READY,
    IMU_PORT_ERROR,
    IMU_PORT_INVALID_SAMPLE
} imu_port_status_t;

typedef struct {
    bool (*init)(void *context);
    imu_port_status_t (*read_latest)(void *context, fc_imu_sample_t *sample);
    bool (*healthy)(void *context);
    void *context;
} imu_port_t;

void IMU_Port_Null_Init(imu_port_t *port);

#endif
