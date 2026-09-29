#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include "imu_port.h"
#include "fc_attitude.h"
#include "fc_estimator.h"
#include "fc_mixer.h"
#include "fc_position.h"
#include "fc_land.h"
#include "fc_guard.h"

typedef struct {
    bool imu_ready;
    bool estimator_ready;
    bool dual_imu_consistent;
    bool output_valid;
    uint32_t fault_flags;
    uint32_t last_update_tick;
    uint32_t guard_mode;      /* FC_GUARD_MODE_NORMAL/STAB/RECOVER */
    uint32_t guard_teleport;  /* 1 -> host should teleport + reset guard */
    uint32_t land_landed;     /* 1 -> land detector fired */
} fc_health_t;

#define FC_FAULT_IMU_NOT_READY       (1UL << 0)
#define FC_FAULT_DUAL_DISAGREEMENT   (1UL << 1)
#define FC_FAULT_ESTIMATOR          (1UL << 2)
#define FC_FAULT_CONTROLLER         (1UL << 3)
#define FC_FAULT_OUTPUT             (1UL << 4)

bool FC_Init(const imu_port_t *primary, const imu_port_t *secondary);
void FC_Setpoint_Update(const fc_setpoint_t *setpoint);
void FC_PositionSetpoint_Update(const fc_position_setpoint_t *setpoint);
void FC_Request_Disarm(void);
void FC_Task(void *argument);
const fc_health_t *FC_GetHealth(void);

#endif
