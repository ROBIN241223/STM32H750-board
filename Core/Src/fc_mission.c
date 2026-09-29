/**
 * @file    fc_mission.c
 * @brief   MAVLink-style NAV_* mission navigator (see fc_mission.h).
 */
#include "fc_mission.h"

#include <math.h>
#include <string.h>

#define FC_MISSION_PI 3.14159265358979323846f
#define FC_MISSION_TWO_PI 6.28318530717958647692f

static float deg2rad(float d)
{
    return d * FC_MISSION_PI / 180.0f;
}

static float approach_alt(float alt, float target, float climb_rate, float descend_rate, float dt)
{
    if (target > alt + 1e-6f) {
        alt += climb_rate * dt;
        if (alt > target) { alt = target; }
    } else if (target < alt - 1e-6f) {
        alt -= descend_rate * dt;
        if (alt < target) { alt = target; }
    }
    return alt;
}

void FC_Mission_ConfigDefault(fc_mission_config_t *config)
{
    config->loiter_omega = 0.5f;
    config->loiter_ramp_s = 3.0f;
    config->accept_xy = 0.8f;
    config->alt_accept = 0.25f;
    config->climb_rate = 0.4f;
    config->descend_rate = 0.3f;
    config->land_alt = 0.05f;
}

void FC_Mission_Init(fc_mission_t *mission)
{
    memset(mission, 0, sizeof(*mission));
}

bool FC_Mission_Append(fc_mission_t *mission, uint32_t cmd, float p1, float p2,
                       float p3, float p4, float x, float y, float z)
{
    if ((mission == NULL) || (mission->count >= FC_MISSION_MAX_ITEMS)) {
        return false;
    }
    fc_mission_item_t *item = &mission->items[mission->count];
    item->cmd = cmd;
    item->p1 = p1;
    item->p2 = p2;
    item->p3 = p3;
    item->p4 = p4;
    item->x = x;
    item->y = y;
    item->z = z;
    mission->count++;
    return true;
}

void FC_Mission_Reset(fc_mission_t *mission)
{
    FC_Mission_Init(mission);
}

bool FC_Mission_Start(fc_mission_t *mission, const fc_estimator_state_t *state)
{
    if ((mission == NULL) || (mission->count == 0U)) {
        return false;
    }
    mission->index = 0U;
    mission->active = 1U;
    mission->mission_t = 0.0f;
    mission->leg_t = 0.0f;
    mission->hold_t = 0.0f;
    mission->phi0 = 0.0f;
    mission->laps_full = 0.0f;
    if (state != NULL) {
        mission->ref_alt = -state->pos_ned[2];
        if (mission->ref_alt < 0.0f) { mission->ref_alt = 0.0f; }
    } else {
        mission->ref_alt = 0.0f;
    }
    mission->ref_alt_prev = mission->ref_alt;
    return true;
}

bool FC_Mission_Active(const fc_mission_t *mission)
{
    return (mission != NULL) && (mission->active != 0U);
}

uint32_t FC_Mission_Index(const fc_mission_t *mission)
{
    return (mission != NULL) ? mission->index : 0U;
}

uint32_t FC_Mission_Count(const fc_mission_t *mission)
{
    return (mission != NULL) ? mission->count : 0U;
}

static void advance(fc_mission_t *mission)
{
    mission->index++;
    mission->leg_t = 0.0f;
    mission->hold_t = 0.0f;
    mission->phi0 = 0.0f;
    mission->laps_full = 0.0f;
    if (mission->index >= mission->count) {
        mission->active = 0U;
    }
}

bool FC_Mission_Update(const fc_mission_config_t *config, fc_mission_t *mission,
                       const fc_estimator_state_t *state, float dt, fc_position_setpoint_t *sp)
{
    if ((mission == NULL) || (sp == NULL) || (mission->active == 0U)) {
        return false;
    }

    const fc_mission_item_t *it = &mission->items[mission->index];
    const float xeast = state->pos_ned[0];
    const float ynorth = state->pos_ned[1];
    const float alt_now = -state->pos_ned[2];

    memset(sp, 0, sizeof(*sp));

    mission->mission_t += dt;
    mission->leg_t += dt;

    float tgt_x = 0.0f;
    float tgt_y = 0.0f;
    float tgt_alt = 0.0f;
    bool reachable_leg = false;
    bool loiter_leg = false;
    float radius = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;

    switch (it->cmd) {
    case FC_MISSION_NAV_TAKEOFF:
    case FC_MISSION_NAV_WAYPOINT:
        tgt_x = it->x;
        tgt_y = it->y;
        tgt_alt = it->z;
        reachable_leg = true;
        break;

    case FC_MISSION_NAV_RETURN_TO_LAUNCH:
        tgt_x = 0.0f;
        tgt_y = 0.0f;
        tgt_alt = mission->ref_alt;   /* keep current altitude */
        reachable_leg = true;
        break;

    case FC_MISSION_NAV_LOITER_TURNS:
    case FC_MISSION_NAV_LOITER_TIME:
        cx = it->x;
        cy = it->y;
        radius = it->p3;
        tgt_alt = it->z;
        loiter_leg = true;
        break;

    case FC_MISSION_NAV_LAND:
        tgt_x = it->x;
        tgt_y = it->y;
        tgt_alt = config->land_alt;
        reachable_leg = true;
        break;

    default:
        return false;
    }

    float s = 1.0f;      /* radius scaling for the spiral entry */
    float phi = 0.0f;
    float cos_a = 1.0f;
    float sin_a = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float ax = 0.0f;
    float ay = 0.0f;

    if (loiter_leg) {
        if (mission->leg_t < 1e-4f) {
            mission->phi0 = atan2f(ynorth - cy, xeast - cx);
        }
        phi = mission->phi0 + config->loiter_omega * mission->leg_t;
        cos_a = cosf(phi);
        sin_a = sinf(phi);
        /* smooth spiral ramp: s(t) = (1-cos(pi t/tau))/2 gives s(0)=1, s'(0)=0,
         * s'(tau)=0 so the drone enters the circle with zero velocity shock. */
        if (config->loiter_ramp_s > 1e-3f) {
            const float u = mission->leg_t / config->loiter_ramp_s;
            if (u <= 1.0f) {
                /* smoothstep S-curve: s(0)=s'(0)=s''(0)=0, s(1)=1, s'(1)=s''(1)=0.
                 * No acceleration feed-forward during the ramp; a spiral entry
                 * with a raw axial acc_ff (R*s`` ~ 6 m/s2) at t=0 tips the
                 * vehicle over. Velocity ff builds smoothly from zero. */
                const float u2 = u * u;
                const float u3 = u2 * u;
                s = 10.0f * u3 - 15.0f * u2 * u2 + 6.0f * u2 * u3;
                const float sd = (30.0f * u2 - 60.0f * u3 + 30.0f * u2 * u2) / config->loiter_ramp_s;
                const float om = config->loiter_omega;
                vx = radius * (sd * cos_a - s * om * sin_a);
                vy = radius * (sd * sin_a + s * om * cos_a);
                ax = 0.0f;
                ay = 0.0f;
            } else {
                s = 1.0f;
            }
        }
        mission->tgt_x = cx + radius * s * cos_a;
        mission->tgt_y = cy + radius * s * sin_a;
        sp->position_ned[0] = mission->tgt_x;
        sp->position_ned[1] = mission->tgt_y;
        if (s == 1.0f) {
            const float rv = radius * config->loiter_omega;
            const float ra = radius * config->loiter_omega * config->loiter_omega;
            vx = -rv * sin_a;
            vy = rv * cos_a;
            ax = -ra * cos_a;
            ay = -ra * sin_a;
        }
        sp->velocity_ned[0] = vx;
        sp->velocity_ned[1] = vy;
        sp->acceleration_ned[0] = ax;
        sp->acceleration_ned[1] = ay;

        mission->ref_alt = approach_alt(mission->ref_alt, tgt_alt,
                                        config->climb_rate, config->descend_rate, dt);
    } else {
        mission->tgt_x = tgt_x;
        mission->tgt_y = tgt_y;
        sp->position_ned[0] = tgt_x;
        sp->position_ned[1] = tgt_y;

        if (it->cmd == FC_MISSION_NAV_LAND) {
            /* descend toward the pad; never below land_alt */
            mission->ref_alt -= config->descend_rate * dt;
            if (mission->ref_alt < config->land_alt) {
                mission->ref_alt = config->land_alt;
            }
        } else {
            mission->ref_alt = approach_alt(mission->ref_alt, tgt_alt,
                                            config->climb_rate, config->descend_rate, dt);
        }
    }

    /* vertical feed-forward keeps the climb/descent profile tight */
    if (dt > 0.0f) {
        const float vz = (mission->ref_alt - mission->ref_alt_prev) / dt;
        const float vz_clamped = fminf(fmaxf(vz, -0.8f), 0.8f);
        sp->velocity_ned[2] = -vz_clamped;
    }
    mission->ref_alt_prev = mission->ref_alt;

    sp->position_ned[2] = -mission->ref_alt;    /* NED: up is negative */
    sp->acceleration_ned[2] = 0.0f;

    /* heading: p4 (deg) if given, else hold 0 (NED north). The sim lacks a
     * reliable absolute heading source, so trailing the drifting estimate
     * would excite the yaw loop; a fixed yaw keeps the circle clean. */
    if (isfinite((double)it->p4) && !loiter_leg) {
        sp->yaw_ned = deg2rad(it->p4);
        sp->yawspeed = 0.0f;
    } else {
        sp->yaw_ned = 0.0f;
        sp->yawspeed = 0.0f;
    }

    /* --- leg completion -------------------------------------------------- */
    if (loiter_leg) {
        if (it->cmd == FC_MISSION_NAV_LOITER_TURNS) {
            if (mission->leg_t >= config->loiter_ramp_s) {
                mission->laps_full = config->loiter_omega * (mission->leg_t - config->loiter_ramp_s);
                mission->laps_full /= FC_MISSION_TWO_PI;
            }
            if (mission->laps_full >= it->p1) {
                advance(mission);
            }
        } else if (mission->leg_t >= (config->loiter_ramp_s + it->p1)) {
            advance(mission);
        }
    } else if (reachable_leg) {
        const float dxy = sqrtf((mission->tgt_x - xeast) * (mission->tgt_x - xeast) +
                                (mission->tgt_y - ynorth) * (mission->tgt_y - ynorth));
        const float dz = fabsf(tgt_alt - alt_now);
        const bool xy_ok = dxy <= config->accept_xy;
        const bool z_ok = dz <= config->alt_accept;
        float hold_target = 0.0f;

        if (it->cmd == FC_MISSION_NAV_WAYPOINT) {
            hold_target = it->p1;
        } else if (it->cmd == FC_MISSION_NAV_RETURN_TO_LAUNCH) {
            hold_target = 0.6f;   /* settle before the LAND leg */
        }
        /* always settle briefly before moving on */
        if (hold_target < 0.1f) { hold_target = 0.1f; }

        if (xy_ok && z_ok) {
            mission->hold_t += dt;
        } else {
            mission->hold_t = 0.0f;
        }
        if (mission->hold_t >= hold_target) {
            advance(mission);
        }
    }

    if (mission->active == 0U) {
        /* land leg finished: freeze at the pad */
        sp->position_ned[0] = 0.0f;
        sp->position_ned[1] = 0.0f;
        sp->position_ned[2] = -config->land_alt;
        sp->velocity_ned[2] = 0.0f;
        sp->yaw_ned = state->yaw_rad;
    }

    return true;
}