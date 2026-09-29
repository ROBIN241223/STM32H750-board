#ifndef FC_ORB_H
#define FC_ORB_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

/* Minimal uORB-style publish/subscribe bus (single writer, single reader,
 * latest-value-kept) mirroring the PX4 uORB semantics used by the control
 * pipeline: a module publishes a topic, others copy the latest message.
 * Storage is statically allocated; every topic has a fixed layout so no
 * dynamic memory / queues are needed for the 500 Hz control loop.
 *
 * Topic names follow the PX4 conventions (sensor_*, vehicle_*).
 */

typedef enum {
    FC_ORB_SENSOR_ACCEL,           /* fc_orb_sensor_accel_t        */
    FC_ORB_SENSOR_GYRO,            /* fc_orb_sensor_gyro_t         */
    FC_ORB_VEHICLE_ATTITUDE,       /* fc_orb_vehicle_attitude_t    */
    FC_ORB_VEHICLE_LOCAL_POSITION, /* fc_orb_vehicle_local_position_t */
    FC_ORB_VEHICLE_ATTITUDE_SETPOINT, /* fc_orb_attitude_setpoint_t */
    FC_ORB_VEHICLE_RATES_SETPOINT, /* fc_orb_rates_setpoint_t      */
    FC_ORB_VEHICLE_TORQUE_SETPOINT,   /* fc_orb_torque_setpoint_t   */
    FC_ORB_VEHICLE_THRUST_SETPOINT,   /* fc_orb_thrust_setpoint_t   */
    FC_ORB_VEHICLE_STATUS,         /* fc_orb_vehicle_status_t      */
    FC_ORB_ACTUATOR_OUTPUTS,       /* fc_motor_output_t            */
    FC_ORB_SENSOR_GPS,             /* fc_orb_sensor_gps_t          */
    FC_ORB_TOPIC_COUNT
} fc_orb_id_t;

typedef struct {
    uint64_t timestamp_us;
    float accel_mps2[3];
} fc_orb_sensor_accel_t;

typedef struct {
    uint64_t timestamp_us;
    float gyro_rad_s[3];
} fc_orb_sensor_gyro_t;

typedef struct {
    uint64_t timestamp_us;
    float q[4];
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float gyro_rad_s[3];
} fc_orb_vehicle_attitude_t;

typedef struct {
    uint64_t timestamp_us;
    float pos_ned[3];
    float vel_ned[3];
    uint32_t pos_valid;
} fc_orb_vehicle_local_position_t;

typedef struct {
    uint64_t timestamp_us;
    float q[4];
    float yaw_rate_sp;
} fc_orb_attitude_setpoint_t;

typedef struct {
    uint64_t timestamp_us;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
} fc_orb_rates_setpoint_t;

typedef struct {
    uint64_t timestamp_us;
    float x;
    float y;
    float z;
} fc_orb_torque_setpoint_t;

typedef struct {
    uint64_t timestamp_us;
    float x;
    float y;
    float z;
} fc_orb_thrust_setpoint_t;

typedef struct {
    uint64_t timestamp_us;
    uint32_t armed;
    uint32_t guard_mode;
    uint32_t landed;
    uint32_t nav_state;
} fc_orb_vehicle_status_t;

/* Raw receiver fix plus the local NED fc_gps derived from it, so a subscriber
 * gets both the geodetic data (logging) and the metric one (control) without
 * repeating the conversion. */
typedef struct {
    uint64_t timestamp_us;
    double lat_deg;
    double lon_deg;
    float alt_m;
    float vel_ned[3];
    float pos_ned[3];
    float hdop;
    uint32_t num_sats;
    uint32_t fix_type;
    uint32_t pos_valid;
} fc_orb_sensor_gps_t;

void FC_Orb_Init(void);
bool FC_Orb_Publish(fc_orb_id_t id, const void *data);
bool FC_Orb_Copy(fc_orb_id_t id, void *data);
bool FC_Orb_Updated(fc_orb_id_t id);
bool FC_Orb_Valid(fc_orb_id_t id);
void FC_Orb_Reset(void);

#endif