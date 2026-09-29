#include "fc_orb.h"
#include <string.h>

#define FC_ORB_MAX_TOPIC_SIZE 96U

typedef struct {
    uint8_t storage[FC_ORB_MAX_TOPIC_SIZE];
    uint32_t size;
    uint32_t published;  /* data has been published at least once    */
    uint32_t updated;    /* publish happened since last copy()       */
} fc_orb_topic_t;

static fc_orb_topic_t topics[FC_ORB_TOPIC_COUNT];

static uint32_t topic_size(fc_orb_id_t id)
{
    switch (id) {
    case FC_ORB_SENSOR_ACCEL:
        return (uint32_t)sizeof(fc_orb_sensor_accel_t);
    case FC_ORB_SENSOR_GYRO:
        return (uint32_t)sizeof(fc_orb_sensor_gyro_t);
    case FC_ORB_VEHICLE_ATTITUDE:
        return (uint32_t)sizeof(fc_orb_vehicle_attitude_t);
    case FC_ORB_VEHICLE_LOCAL_POSITION:
        return (uint32_t)sizeof(fc_orb_vehicle_local_position_t);
    case FC_ORB_VEHICLE_ATTITUDE_SETPOINT:
        return (uint32_t)sizeof(fc_orb_attitude_setpoint_t);
    case FC_ORB_VEHICLE_RATES_SETPOINT:
        return (uint32_t)sizeof(fc_orb_rates_setpoint_t);
    case FC_ORB_VEHICLE_TORQUE_SETPOINT:
        return (uint32_t)sizeof(fc_orb_torque_setpoint_t);
    case FC_ORB_VEHICLE_THRUST_SETPOINT:
        return (uint32_t)sizeof(fc_orb_thrust_setpoint_t);
    case FC_ORB_VEHICLE_STATUS:
        return (uint32_t)sizeof(fc_orb_vehicle_status_t);
    case FC_ORB_ACTUATOR_OUTPUTS:
        return (uint32_t)sizeof(fc_motor_output_t);
    default:
        return 0U;
    }
}

void FC_Orb_Init(void)
{
    memset(topics, 0, sizeof(topics));
}

void FC_Orb_Reset(void)
{
    memset(topics, 0, sizeof(topics));
}

bool FC_Orb_Publish(fc_orb_id_t id, const void *data)
{
    if ((id >= FC_ORB_TOPIC_COUNT) || (data == NULL)) {
        return false;
    }
    uint32_t size = topic_size(id);
    if ((size == 0U) || (size > FC_ORB_MAX_TOPIC_SIZE)) {
        return false;
    }
    memcpy(topics[id].storage, data, size);
    topics[id].size = size;
    topics[id].published = 1U;
    topics[id].updated = 1U;
    return true;
}

bool FC_Orb_Copy(fc_orb_id_t id, void *data)
{
    if ((id >= FC_ORB_TOPIC_COUNT) || (data == NULL) ||
        (topics[id].published == 0U)) {
        return false;
    }
    memcpy(data, topics[id].storage, topics[id].size);
    topics[id].updated = 0U;
    return true;
}

bool FC_Orb_Updated(fc_orb_id_t id)
{
    return (id < FC_ORB_TOPIC_COUNT) && (topics[id].updated != 0U);
}

bool FC_Orb_Valid(fc_orb_id_t id)
{
    return (id < FC_ORB_TOPIC_COUNT) && (topics[id].published != 0U);
}