#include "flight_controller.h"
#include "ros2_motor.h"
#include "fc_position.h"
#include "fc_rate.h"
#include "fc_sched.h"
#include "fc_orb.h"
#include "fc_param.h"
#include "fc_gps.h"
#include "fc_compass.h"
#include "gps_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

static imu_port_t primary_imu;
static imu_port_t secondary_imu;
static fc_estimator_config_t estimator_config;
static fc_estimator_state_t estimator_state;
static fc_gps_config_t gps_config;
static fc_gps_state_t gps_state;
static fc_compass_config_t compass_config;
static fc_compass_state_t compass_state;
static gps_port_t gps_port;
static uint32_t gps_fuse_enabled;
static fc_attitude_config_t attitude_config;
static fc_rate_config_t rate_config;
static fc_mixer_config_t mixer_config;
static fc_position_config_t position_config;
static fc_position_setpoint_t position_setpoint;
static fc_position_out_t position_out;
static fc_land_config_t land_config;
static fc_land_state_t land_state;
static fc_land_out_t land_out;
static fc_guard_config_t guard_config;
static fc_guard_state_t guard_state;
static fc_guard_out_t guard_out;
static fc_setpoint_t setpoint;
static fc_health_t health;
static bool position_mode;
static TickType_t last_control_tick;
static fc_sched_t control_sched;

/* Latest state snapshot shared with the telemetry scheduler callbacks.      */
typedef struct {
    fc_estimator_state_t estimator;
    fc_gps_state_t gps;
    fc_motor_output_t output;
    uint32_t output_valid;
    uint32_t armed;
    uint32_t guard_mode;
    uint32_t landed;
    uint32_t nav_state;
} fc_telemetry_ctx_t;
static fc_telemetry_ctx_t telemetry;

/* PX4 work-queue style callbacks, run cooperatively by FC_Sched_Tick.       */
static void publish_sensors(void *ctx, uint32_t elapsed_us)
{
    fc_telemetry_ctx_t *t = (fc_telemetry_ctx_t *)ctx;
    fc_orb_sensor_gyro_t gyro;
    fc_orb_sensor_accel_t accel;
    uint64_t ts_us = (uint64_t)xTaskGetTickCount() * 1000U;
    (void)elapsed_us;

    memset(&gyro, 0, sizeof(gyro));
    gyro.timestamp_us = ts_us;
    memcpy(gyro.gyro_rad_s, t->estimator.gyro_rad_s, sizeof(gyro.gyro_rad_s));
    FC_Orb_Publish(FC_ORB_SENSOR_GYRO, &gyro);

    memset(&accel, 0, sizeof(accel));
    accel.timestamp_us = ts_us;
    memcpy(accel.accel_mps2, t->estimator.accel_mps2, sizeof(accel.accel_mps2));
    FC_Orb_Publish(FC_ORB_SENSOR_ACCEL, &accel);
}

/* Published on its own slower slot: a receiver runs at 1-10 Hz, so republishing
 * the same fix at 500 Hz would tell subscribers nothing new while flooding the
 * bus. Without this the topic stays invalid forever and a ground station has no
 * way to tell "no receiver" from "topic never wired up". */
static void publish_gps(void *ctx, uint32_t elapsed_us)
{
    fc_telemetry_ctx_t *t = (fc_telemetry_ctx_t *)ctx;
    fc_orb_sensor_gps_t gps;
    (void)elapsed_us;

    memset(&gps, 0, sizeof(gps));
    gps.timestamp_us = (uint64_t)xTaskGetTickCount() * 1000U;
    gps.lat_deg = t->gps.last_lat_deg;
    gps.lon_deg = t->gps.last_lon_deg;
    gps.alt_m = t->gps.alt_m;
    gps.hdop = t->gps.hdop;
    gps.num_sats = t->gps.num_sats;
    gps.fix_type = t->gps.fix_type;
    memcpy(gps.vel_ned, t->gps.vel_ned, sizeof(gps.vel_ned));
    if (FC_GPS_GetLocal(&t->gps, gps.pos_ned, NULL)) {
        gps.pos_valid = 1U;
    }
    FC_Orb_Publish(FC_ORB_SENSOR_GPS, &gps);
}

static void publish_attitude(void *ctx, uint32_t elapsed_us)
{
    fc_telemetry_ctx_t *t = (fc_telemetry_ctx_t *)ctx;
    fc_orb_vehicle_attitude_t att;
    (void)elapsed_us;

    memset(&att, 0, sizeof(att));
    att.timestamp_us = (uint64_t)xTaskGetTickCount() * 1000U;
    att.q[0] = t->estimator.q_w;
    att.q[1] = t->estimator.q_x;
    att.q[2] = t->estimator.q_y;
    att.q[3] = t->estimator.q_z;
    att.roll_rad = t->estimator.roll_rad;
    att.pitch_rad = t->estimator.pitch_rad;
    att.yaw_rad = t->estimator.yaw_rad;
    memcpy(att.gyro_rad_s, t->estimator.gyro_rad_s, sizeof(att.gyro_rad_s));
    FC_Orb_Publish(FC_ORB_VEHICLE_ATTITUDE, &att);
}

static void publish_actuators(void *ctx, uint32_t elapsed_us)
{
    fc_telemetry_ctx_t *t = (fc_telemetry_ctx_t *)ctx;
    (void)elapsed_us;
    if (t->output_valid != 0U) {
        FC_Orb_Publish(FC_ORB_ACTUATOR_OUTPUTS, &t->output);
    }
}

static void publish_status(void *ctx, uint32_t elapsed_us)
{
    fc_telemetry_ctx_t *t = (fc_telemetry_ctx_t *)ctx;
    fc_orb_vehicle_status_t status;
    (void)elapsed_us;

    memset(&status, 0, sizeof(status));
    status.timestamp_us = (uint64_t)xTaskGetTickCount() * 1000U;
    status.armed = t->armed;
    status.guard_mode = t->guard_mode;
    status.landed = t->landed;
    status.nav_state = t->nav_state;
    FC_Orb_Publish(FC_ORB_VEHICLE_STATUS, &status);
}

static float absolute_difference(float a, float b)
{
    return fabsf(a - b);
}

static bool samples_consistent(const fc_imu_sample_t *primary, const fc_imu_sample_t *secondary)
{
    float gyro_delta = 0.0f;
    float accel_delta = 0.0f;
    for (uint32_t i = 0U; i < 3U; i++) {
        gyro_delta += absolute_difference(primary->gyro_rad_s[i], secondary->gyro_rad_s[i]);
        accel_delta += absolute_difference(primary->accel_mps2[i], secondary->accel_mps2[i]);
    }
    return (gyro_delta < 2.0f) && (accel_delta < 5.0f);
}

static void setpoint_attitude_from_angles(float roll, float pitch, float yaw, float attitude_sp[4])
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    attitude_sp[0] = cr * cp * cy + sr * sp * sy;
    attitude_sp[1] = sr * cp * cy - cr * sp * sy;
    attitude_sp[2] = cr * sp * cy + sr * cp * sy;
    attitude_sp[3] = cr * cp * sy - sr * sp * cy;
}

bool FC_Init(const imu_port_t *primary, const imu_port_t *secondary)
{
    if ((primary == NULL) || (secondary == NULL)) {
        return false;
    }
    primary_imu = *primary;
    secondary_imu = *secondary;
    if (primary_imu.init != NULL) {
        (void)primary_imu.init(primary_imu.context);
    }
    if (secondary_imu.init != NULL) {
        (void)secondary_imu.init(secondary_imu.context);
    }
    FC_Estimator_ConfigDefault(&estimator_config);
    FC_Estimator_Init(&estimator_config, &estimator_state);
    FC_GPS_ConfigDefault(&gps_config);
    FC_GPS_Init(&gps_config, &gps_state);
    FC_Compass_ConfigDefault(&compass_config);
    FC_Compass_Init(&compass_config, &compass_state);
    GPS_Port_UartBind(&gps_port);
    if ((gps_port.init != NULL) && !gps_port.init(gps_port.context)) {
        /* No receiver: keep the null port so the loop sees "no fix" instead of
         * dereferencing a port that was never bound. */
        GPS_Port_Null_Init(&gps_port);
    }
    FC_Position_ConfigDefault(&position_config);
    FC_Attitude_ConfigDefault(&attitude_config);
    FC_RateControl_ConfigDefault(&rate_config);
    FC_Mixer_ConfigDefault(&mixer_config);
    FC_Land_ConfigDefault(&land_config);
    FC_Guard_ConfigDefault(&guard_config);
    /* Params are the single source of tuning; Apply runs before *_Init so the
     * guard's base-config snapshot contains the parameter-applied gains.     */
    FC_Param_Init();
    FC_Param_Apply(&attitude_config, &rate_config, &position_config,
                   &land_config, &guard_config, &mixer_config,
                   &gps_config, &compass_config);
    {
        int32_t fuse = 0;
        gps_fuse_enabled = (FC_Param_GetInt("GPS_FUSE", &fuse) && (fuse != 0)) ? 1U : 0U;
    }
    FC_Orb_Init();
    FC_Sched_Init(&control_sched);
    FC_Sched_Add(&control_sched, 2000U, publish_sensors, &telemetry);
    FC_Sched_Add(&control_sched, 200000U, publish_gps, &telemetry);
    FC_Sched_Add(&control_sched, 20000U, publish_attitude, &telemetry);
    FC_Sched_Add(&control_sched, 20000U, publish_actuators, &telemetry);
    FC_Sched_Add(&control_sched, 100000U, publish_status, &telemetry);
    memset(&telemetry, 0, sizeof(telemetry));
    FC_Estimator_Init(&estimator_config, &estimator_state);
    FC_Land_Init(&land_config, &land_state);
    memset(&land_out, 0, sizeof(land_out));
    FC_Guard_Init(&guard_config, &guard_state, &attitude_config, &rate_config);
    memset(&guard_out, 0, sizeof(guard_out));
    memset(&setpoint, 0, sizeof(setpoint));
    memset(&position_setpoint, 0, sizeof(position_setpoint));
    memset(&position_out, 0, sizeof(position_out));
    memset(&health, 0, sizeof(health));
    position_mode = false;
    last_control_tick = xTaskGetTickCount();
    return true;
}

void FC_Setpoint_Update(const fc_setpoint_t *new_setpoint)
{
    if (new_setpoint == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    setpoint = *new_setpoint;
    setpoint.thrust_norm = fminf(fmaxf(setpoint.thrust_norm, 0.0f), 1.0f);
    if (!setpoint.armed) {
        position_mode = false;
    }
    taskEXIT_CRITICAL();
}

void FC_PositionSetpoint_Update(const fc_position_setpoint_t *new_setpoint)
{
    if ((new_setpoint == NULL) || !setpoint.armed) {
        return;
    }
    taskENTER_CRITICAL();
    position_setpoint = *new_setpoint;
    position_mode = true;
    taskEXIT_CRITICAL();
}

void FC_Request_Disarm(void)
{
    taskENTER_CRITICAL();
    setpoint.armed = false;
    position_mode = false;
    taskEXIT_CRITICAL();
    FC_Attitude_Reset(&attitude_config);
    FC_RateControl_Reset(&rate_config);
    FC_Position_Reset(&position_config);
    FC_Guard_Reset(&guard_config, &guard_state, &attitude_config, &rate_config);
    FC_Land_Init(&land_config, &land_state);
    ROS2_Motor_FlightFault();
}

const fc_health_t *FC_GetHealth(void)
{
    return &health;
}

void FC_Task(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2U);
    fc_imu_sample_t primary_sample;
    fc_imu_sample_t secondary_sample;
    fc_imu_sample_t corrected_sample;
    fc_imu_sample_t *selected_sample;
    fc_gps_sample_t gps_sample;
    double gps_last_lat = 0.0;
    double gps_last_lon = 0.0;
    float gps_last_alt = 0.0f;
    float gps_pos_ned[3] = {0.0f, 0.0f, 0.0f};
    float gps_vel_ned[3] = {0.0f, 0.0f, 0.0f};
    uint32_t gps_have_sample = 0U;
    uint32_t gps_fix_count = 0U;
    fc_torque_cmd_t torque;
    fc_motor_output_t output;

    (void)argument;
    for (;;) {
        /* Single cooperative tick source; callbacks publish orb topics.     */
        FC_Sched_Tick(&control_sched, 2000U);
        telemetry.armed = setpoint.armed ? 1U : 0U;
        telemetry.guard_mode = health.guard_mode;
        telemetry.landed = health.land_landed;
        telemetry.nav_state = position_mode ? 1U : 0U;
        bool primary_ok = (primary_imu.read_latest != NULL) &&
            (primary_imu.read_latest(primary_imu.context, &primary_sample) == IMU_PORT_OK);
        bool secondary_ok = (secondary_imu.read_latest != NULL) &&
            (secondary_imu.read_latest(secondary_imu.context, &secondary_sample) == IMU_PORT_OK);
        float dt;
        health.last_update_tick = xTaskGetTickCount();
        health.imu_ready = primary_ok || secondary_ok;
        health.dual_imu_consistent = true;

        if (primary_ok && secondary_ok) {
            health.dual_imu_consistent = samples_consistent(&primary_sample, &secondary_sample);
        }
        if (!health.imu_ready || !health.dual_imu_consistent) {
            health.fault_flags = !health.imu_ready ? FC_FAULT_IMU_NOT_READY : FC_FAULT_DUAL_DISAGREEMENT;
            health.estimator_ready = false;
            health.output_valid = false;
            ROS2_Motor_FlightFault();
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        if (setpoint.armed &&
            ((xTaskGetTickCount() - setpoint.timestamp_ms) > pdMS_TO_TICKS(100U))) {
            health.fault_flags = FC_FAULT_OUTPUT;
            health.estimator_ready = false;
            health.output_valid = false;
            ROS2_Motor_FlightFault();
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        selected_sample = primary_ok ? &primary_sample : &secondary_sample;

        /* --- GNSS: convert and gate the fix before the estimator sees it --- */
        if ((gps_port.read_latest != NULL) &&
            (gps_port.read_latest(gps_port.context, &gps_sample) == GPS_PORT_OK)) {
            gps_sample.timestamp_us = (uint64_t)xTaskGetTickCount() *
                                     (1000000U / configTICK_RATE_HZ);
            gps_last_lat = gps_sample.lat_deg;
            gps_last_lon = gps_sample.lon_deg;
            gps_last_alt = gps_sample.alt_m;
            gps_have_sample = 1U;
            if (FC_GPS_Update(&gps_config, &gps_state, &gps_sample)) {
                gps_fix_count++;
            }
        }
        /* Keep re-freezing home to the newest good fix while the vehicle is
         * parked, so arming always starts from the position we are actually
         * standing in. Once armed, home is frozen for the rest of the flight.
         * The quality gate matters here: a receiver that has not acquired yet
         * still reports a position, and freezing that would place home
         * hundreds of metres from the launch point. */
        if ((gps_have_sample != 0U) && !setpoint.armed &&
            FC_GPS_SampleAcceptable(&gps_config, &gps_sample)) {
            (void)FC_GPS_SetHome(&gps_config, &gps_state,
                                 gps_last_lat, gps_last_lon, gps_last_alt);
        }
        /* With GPS_FUSE the fix *is* the position estimate; without it the
         * position comes from wherever the caller feeds it (sim ground truth),
         * so existing behaviour is untouched. */
        if ((gps_fuse_enabled != 0U) && FC_GPS_IsValid(&gps_state)) {
            (void)FC_GPS_GetLocal(&gps_state, gps_pos_ned, gps_vel_ned);
            FC_Estimator_FeedPosition(&estimator_state, gps_pos_ned, gps_vel_ned);
        }
        /* Published here rather than next to the estimator snapshot so the topic
         * still reports fix quality when an unrelated fault cuts the loop short:
         * a dead GPS is exactly the case an operator needs to see. */
        telemetry.gps = gps_state;

        /* --- magnetometer: correct, rotate, then let the estimator fuse ---- */
        corrected_sample = *selected_sample;
        if (FC_Compass_Update(&compass_config, &compass_state,
                              selected_sample->mag_ut, 0.0f) &&
            FC_Compass_Field(&compass_state, corrected_sample.mag_ut)) {
            corrected_sample.mag_valid = FC_Compass_IsValid(&compass_state) ? 1U : 0U;
        } else {
            corrected_sample.mag_valid = 0U;
        }
        selected_sample = &corrected_sample;

        if (!FC_Estimator_Update(&estimator_config, &estimator_state, selected_sample)) {
            health.fault_flags = FC_FAULT_ESTIMATOR;
            health.estimator_ready = false;
            health.output_valid = false;
            ROS2_Motor_FlightFault();
            vTaskDelayUntil(&last_wake, period);
            continue;
        }
        health.estimator_ready = true;
        telemetry.estimator = estimator_state;
        if (!setpoint.armed) {
            health.output_valid = false;
            ROS2_Motor_FlightDisarm();
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        dt = (float)(xTaskGetTickCount() - last_control_tick) / (float)configTICK_RATE_HZ;
        last_control_tick = xTaskGetTickCount();
        if (!isfinite(dt) || (dt < 0.000125f) || (dt > 0.02f)) {
            dt = 0.002f;
        }

        {
            /* A.1: guard checks |roll| and |pitch| separately (FD_FAIL_R/P) */
            float roll_rad = estimator_state.roll_rad;
            float pitch_rad = estimator_state.pitch_rad;
            float gx = estimator_state.gyro_rad_s[0];
            float gy = estimator_state.gyro_rad_s[1];
            float gz = estimator_state.gyro_rad_s[2];
            float body_rate_rad_s = sqrtf(gx * gx + gy * gy + gz * gz);
            float alt_m = (estimator_state.pos_valid != 0U) ? -estimator_state.pos_ned[2] : 1.0e6f;
            bool ok_g = FC_Guard_Update(&guard_config, &guard_state, &attitude_config,
                                        &rate_config, roll_rad, pitch_rad,
                                        body_rate_rad_s, alt_m, dt, &guard_out);
            health.guard_mode = ok_g ? guard_out.mode : FC_GUARD_MODE_NORMAL;
            health.guard_teleport = ok_g ? guard_out.teleport_now : 0U;
        }
        bool normal_flight = (guard_out.mode == FC_GUARD_MODE_NORMAL);
        float attitude_sp[4];
        float thrust_norm;
        float yaw_rate_sp;

        if (normal_flight && position_mode && (estimator_state.pos_valid != 0U) &&
            FC_Position_Update(&position_config, &estimator_state, &position_setpoint,
                               dt, &position_out)) {
            if (position_config.z_up != 0U) {
                thrust_norm = position_out.thrust_ned[2];
            } else {
                thrust_norm = -position_out.thrust_ned[2];
            }
            thrust_norm = fminf(fmaxf(thrust_norm, 0.0f), 1.0f);
            memcpy(attitude_sp, position_out.attitude_sp, sizeof(attitude_sp));
            yaw_rate_sp = position_out.yaw_rate_sp;
        } else if (normal_flight) {
            setpoint_attitude_from_angles(setpoint.roll_rad, setpoint.pitch_rad,
                                          estimator_state.yaw_rad, attitude_sp);
            thrust_norm = fminf(fmaxf(setpoint.thrust_norm, 0.0f), 1.0f);
            yaw_rate_sp = setpoint.yaw_rate_rad_s;
        } else {
            attitude_sp[0] = 1.0f;
            attitude_sp[1] = 0.0f;
            attitude_sp[2] = 0.0f;
            attitude_sp[3] = 0.0f;
            thrust_norm = fminf(fmaxf(guard_out.thrust_hold, 0.0f), 1.0f);
            yaw_rate_sp = 0.0f;
        }

        {
            float vz = estimator_state.vel_ned[2];
            float vx = estimator_state.vel_ned[0];
            float vy = estimator_state.vel_ned[1];
            float vxy = sqrtf(vx * vx + vy * vy);
            float rot_xy_rad_s = sqrtf(estimator_state.gyro_rad_s[0] * estimator_state.gyro_rad_s[0] +
                                       estimator_state.gyro_rad_s[1] * estimator_state.gyro_rad_s[1]);
            float ax = estimator_state.accel_mps2[0];
            float ay = estimator_state.accel_mps2[1];
            float az = estimator_state.accel_mps2[2];
            float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
            float alt_m = (estimator_state.pos_valid != 0U) ? -estimator_state.pos_ned[2] : 1.0e6f;
            if (!FC_Land_Update(&land_config, &land_state, thrust_norm, vz, vxy,
                                rot_xy_rad_s, accel_norm, alt_m, dt, &land_out)) {
                health.fault_flags = FC_FAULT_CONTROLLER;
                health.output_valid = false;
                ROS2_Motor_FlightFault();
                vTaskDelayUntil(&last_wake, period);
                continue;
            }
        }
        bool landed = (land_out.landed != 0U);
        health.land_landed = landed ? 1U : 0U;

        if (landed) {
            health.output_valid = false;
            ROS2_Motor_FlightDisarm();
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        {
            float rate_sp[3];
            float torque_axis[3];
            bool ok = FC_Attitude_Update(&attitude_config, &estimator_state, attitude_sp,
                                         yaw_rate_sp, rate_sp) &&
                FC_RateControl_Update(&rate_config, rate_sp, estimator_state.gyro_rad_s, dt,
                                      landed, torque_axis);
            torque.roll_torque = torque_axis[0];
            torque.pitch_torque = torque_axis[1];
            torque.yaw_torque = torque_axis[2];
            ok = ok &&
                FC_Mixer_Compute(&mixer_config, &torque, thrust_norm, true, &output);
            if (!ok) {
                health.fault_flags = FC_FAULT_CONTROLLER;
                health.output_valid = false;
                ROS2_Motor_FlightFault();
                vTaskDelayUntil(&last_wake, period);
                continue;
            }
        }

        ROS2_Motor_RequestFlightArm();
        ROS2_Motor_SubmitFlightOutput(&output);
        telemetry.output = output;
        telemetry.output_valid = 1U;
        health.fault_flags = 0U;
        health.output_valid = true;
        vTaskDelayUntil(&last_wake, period);
    }
}
