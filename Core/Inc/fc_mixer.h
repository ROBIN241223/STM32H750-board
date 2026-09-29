#ifndef FC_MIXER_H
#define FC_MIXER_H

#include <stdbool.h>
#include <stdint.h>
#include "fc_types.h"

#define FC_MIXER_MAX_ROTORS 4U

/* Phase F: control_allocator port from PX4 (ActuatorEffectivenessRotors +
 * ControlAllocationPseudoInverse + normalize_rpy).
 *
 * Geometry comes from CA_* params exactly like PX4 airframe 4001_quad_x:
 * rotor position (PX/PY/PZ), spin direction (KM, + = counter-clockwise seen
 * from above), thrust coefficient (CT), axis fixed vertical (0, 0, -1).
 * The nxn effectiveness matrix rows are [roll, pitch, yaw, thrust_z]:
 *
 *   roll_i   = ct * (-py_i)
 *   pitch_i  = ct *  px_i
 *   yaw_i    = ct *  km_i
 *   thrust_i = -ct              (axis_z = -1 => upward thrust is -z)
 *
 * The mix is the pseudo-inverse of E, normalized the PX4 normalize_rpy way:
 * roll/pitch share the same authority scale, yaw = max column, thrust =
 * mean |column|.  With this an input torque of -1..1 maps to the same
 * authority on every axis (a unit torque swings each participating motor by
 * ~0.707) and thrust_norm 0..1 maps to every motor directly.
 *
 * Saturation is resolved like PX4 airmode-disabled: the torque deltas are
 * scaled down (thrust is preserved) so all motors stay within [0, 1].  The
 * old signed params (roll_scale / pitch_sign / yaw_scale / yaw_sign /
 * max_delta_norm) are gone: signal direction falls out of the geometry
 * (PX/PY/KM) exactly like PX4.
 *
 * Note: our thrust_norm is a positive magnitude (0..1); PX4's thrust
 * setpoint is a signed NED vector, so the thrust column is flipped inside
 * FC_Mixer_Compute.
 */

typedef struct {
    uint32_t ca_rotor_count;                /* CA_ROTOR_COUNT (default 4) */
    float ca_rotor_px[FC_MIXER_MAX_ROTORS]; /* CA_ROTOR{i}_PX */
    float ca_rotor_py[FC_MIXER_MAX_ROTORS]; /* CA_ROTOR{i}_PY */
    float ca_rotor_pz[FC_MIXER_MAX_ROTORS]; /* CA_ROTOR{i}_PZ (not used for vertical axis) */
    float ca_rotor_km[FC_MIXER_MAX_ROTORS]; /* CA_ROTOR{i}_KM: +ccw / -cw */
    float ca_rotor_ct[FC_MIXER_MAX_ROTORS]; /* CA_ROTOR{i}_CT */
    float max_slew_norm;                    /* per-motor ramp limit (0 = off) */
    float previous_normalized[FC_MIXER_MAX_ROTORS];
    uint32_t initialized;
} fc_mixer_config_t;

void FC_Mixer_ConfigDefault(fc_mixer_config_t *config);
bool FC_Mixer_Compute(fc_mixer_config_t *config, const fc_torque_cmd_t *torque,
                      float thrust_norm, bool armed, fc_motor_output_t *output);

#endif