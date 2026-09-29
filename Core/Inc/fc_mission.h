/**
 * @file    fc_mission.h
 * @brief   MAVLink-style NAV_* mission navigator (flight side).
 *
 * The navigator consumes a sequence of NAV_* commands (the same list a GCS
 * uploads as a MAVLink mission) and computes the desired trajectory setpoint
 * (position + velocity/acceleration feed-forward) every control cycle, exactly
 * like PX4 navigator -> vehicle_trajectory_setpoint.  The position controller
 * (fc_position) simply tracks the setpoint.
 *
 * Items are appended through FC_Mission_Append() over whatever transport the
 * system exposes (MAVLink MISSION_ITEM in the field, the simulator binding in
 * the SITL harness).  All mission coordinates are NED offsets (east/north) from
 * the takeoff pad; the altitude field z is positive UP (m).
 */
#ifndef FC_MISSION_H
#define FC_MISSION_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_estimator.h"
#include "fc_position.h"

#define FC_MISSION_MAX_ITEMS 32U
#define FC_MISSION_NAV_WAYPOINT      16
#define FC_MISSION_NAV_LOITER_TURNS  18
#define FC_MISSION_NAV_LOITER_TIME   19
#define FC_MISSION_NAV_RETURN_TO_LAUNCH 20
#define FC_MISSION_NAV_LAND          21
#define FC_MISSION_NAV_TAKEOFF       22

typedef struct {
    /* NAV_WAYPOINT: p1=hold(s) p2=accept(m) p4=yaw(deg)
     * NAV_LOITER_TURNS: p1=#laps p2=heading_req p3=radius(m)
     * NAV_LOITER_TIME: p1=seconds p3=radius(m)
     * NAV_TAKEOFF: p4=yaw(deg); NAV_LAND: p3=abort-alt (unused)
     * p4=NAN means "keep current heading". */
    uint32_t cmd;
    float p1;
    float p2;
    float p3;
    float p4;
    float x;   /* NED east offset (m) */
    float y;   /* NED north offset (m) */
    float z;   /* altitude, positive UP (m) */
} fc_mission_item_t;

typedef struct {
    float loiter_omega;    /* angular rate while circling (rad/s) */
    float loiter_ramp_s;   /* seconds to spiral from 0 to full radius */
    float accept_xy;       /* horizontal acceptance radius (m) */
    float alt_accept;      /* vertical acceptance window (m) */
    float climb_rate;      /* max altitude climb (m/s, up) */
    float descend_rate;    /* max altitude descend (m/s, down) */
    float land_alt;        /* altitude where LAND phase finishes (m, up) */
} fc_mission_config_t;

typedef struct {
    fc_mission_item_t items[FC_MISSION_MAX_ITEMS];
    uint32_t count;
    uint32_t index;        /* current item */
    uint32_t active;       /* 1 while navigating */
    float mission_t;       /* seconds since FC_Mission_Start */
    float leg_t;           /* seconds in the current leg */
    float hold_t;          /* seconds inside the acceptance radius */
    float ref_alt;         /* commanded altitude (m, up) */
    float tgt_x;           /* current point-leg target (NED east) */
    float tgt_y;           /* current point-leg target (NED north) */
    float phi0;            /* loiter entry phase angle (rad) */
    float laps_full;       /* completed full laps on the circle */
    float ref_alt_prev;
} fc_mission_t;

void FC_Mission_ConfigDefault(fc_mission_config_t *config);
void FC_Mission_Init(fc_mission_t *mission);
bool FC_Mission_Append(fc_mission_t *mission, uint32_t cmd, float p1, float p2,
                       float p3, float p4, float x, float y, float z);
void FC_Mission_Reset(fc_mission_t *mission);
bool FC_Mission_Start(fc_mission_t *mission, const fc_estimator_state_t *state);
bool FC_Mission_Active(const fc_mission_t *mission);
uint32_t FC_Mission_Index(const fc_mission_t *mission);
uint32_t FC_Mission_Count(const fc_mission_t *mission);
bool FC_Mission_Update(const fc_mission_config_t *config, fc_mission_t *mission,
                       const fc_estimator_state_t *state, float dt, fc_position_setpoint_t *sp);

#endif