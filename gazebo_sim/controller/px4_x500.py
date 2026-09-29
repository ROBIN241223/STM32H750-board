#!/usr/bin/env python3
# gz-sim SIL for the STM32H750 PX4-style controller.
#
# Loads the EXACT C modules (fc_position.c, fc_attitude.c, fc_mixer.c) built as
# a shared library (controller/build/px4_ctl.so) and drives the x500 Gazebo
# model with them via ctypes. Nothing important is re-implemented in Python:
# the Python side only marshals Gazebo sensors into fc_estimator_state_t and
# the resulting fc_motor_output_t normalized values into rotor speeds.
#
# Mission is a MAVLink-style command sequence (a QGC-style list of NAV_* items).
# The navigator executes one item after another and streams the trajectory
# setpoint (position + velocity/acceleration feed-forward) to the C position
# controller, exactly like PX4 navigator -> vehicle_trajectory_setpoint.
#
# Item format (MAV_CMD semantics, NED offsets from HOME, altitude positive up):
#   (cmd, p1, p2, p3, p4, x, y, z)
#   NAV_TAKEOFF       (22) p4=yaw(deg,nan=keep)               z=altitude
#   NAV_WAYPOINT      (16) p1=hold(s) p2=acceptance(m) p4=yaw z=altitude
#   NAV_LOITER_TURNS  (18) p1=#laps p2=heading_req p3=radius  z=altitude
#   NAV_LOITER_TIME   (19) p1=seconds p3=radius               z=altitude
#   NAV_RETURN_TO_LAUNCH (20)                                  -> home pad
#   NAV_LAND          (21) p3=abort alt                        z=ground
#
# Missions load from GZ_MISSION_FILE (text: `cmd p1..p4 x y z`, '#' = comment)
# or fall back to a built-in preset for MISSION ('hold' | 'circle').
#
# Usage:
#   python3 -u controller/px4_x500.py [ALT] [DUR] [CLIMB] [LAND_RATE] [MISSION]
# Env: GZ_K GZ_MG GZ_TELEPORT GZ_DBG GZ_MISSION_FILE GZ_CIRCLE_R GZ_CIRCLE_OMEGA
#      GZ_CIRCLE_LAPS GZ_ACC_HOR GZ_LOITER_RAMP GZ_WP_ACC_R

import ctypes, importlib, math, os, subprocess, sys, time

os.environ.setdefault('GZ_PARTITION', 'stm32_h750_sim')
os.environ.setdefault('GZ_IP', '127.0.0.1')

_msgs = importlib.import_module('gz.msgs')
_pose_pb = importlib.import_module('gz.msgs.pose_v_pb2')
_imu_pb = importlib.import_module('gz.msgs.imu_pb2')
_mag_pb = importlib.import_module('gz.msgs.magnetometer_pb2')
_act_pb = importlib.import_module('gz.msgs.actuators_pb2')
_tr = importlib.import_module('gz.transport')

MODEL = 'x500'
POSE = '/world/quadcopter_world/pose/info'
IMU = '/x500/imu'
MAG = '/x500/mag'
MOTOR = '/x500/command/motor_speed'

ALT = float(sys.argv[1] if len(sys.argv) > 1 else 3.0)
DUR = float(sys.argv[2] if len(sys.argv) > 2 else 60.0)
CLIMB = max(0.01, float(sys.argv[3] if len(sys.argv) > 3 else 0.3))
LAND_RATE = float(sys.argv[4] if len(sys.argv) > 4 else 0.3)
MISSION = sys.argv[5] if len(sys.argv) > 5 else 'hold'

# --- navigator / trajectory knobs (firmware-side, fc_mission_config_t) -----
ACC_H = float(os.environ.get('GZ_ACC_HOR', '3.0'))            # MPC_ACC_HOR_MAX (m/s^2)
LOIT_OMEGA = float(os.environ.get('GZ_LOITER_OMEGA', '0.5'))  # loiter angular rate (rad/s)
LOIT_RAMP = float(os.environ.get('GZ_LOITER_RAMP', '3.0'))    # s to spiral onto the circle
CLIMB_R = float(os.environ.get('GZ_CLIMB_RATE', '0.4'))       # climb rate (m/s up)
DESC_R = float(os.environ.get('GZ_DESC_RATE', '0.3'))         # descend rate (m/s down)
XY_ACC_R = float(os.environ.get('GZ_WP_ACC_R', '0.8'))        # waypoint acceptance radius (m)

# --- flight-path logging (GZ_LOG / GZ_LOG_HZ / GZ_LOG_NOHEADER) ------------
# Every controller iteration can be appended to a CSV so a flight can be
# replayed and analysed offline (tools/analyze_flight.py). Without it, tilt
# during LOITER/RTL/LAND can only be guessed at from the 2 Hz stdout line.
LOG_HZ = float(os.environ.get('GZ_LOG_HZ', '25.0'))
LOG_PATH = os.environ.get('GZ_LOG', '')
LOG_QUIET = os.environ.get('GZ_LOG_NOHEADER', '') != ''
LOG_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       os.pardir, 'logs'))
LOG_FIELDS = (
    't', 'leg', 'x', 'y', 'z', 'sp_x', 'sp_y', 'sp_z', 'sp_vx', 'sp_vy', 'sp_vz',
    'vx', 'vy', 'vz', 'roll_deg', 'pitch_deg', 'yaw_pose_deg', 'yaw_est_deg',
    'roll_est_deg', 'pitch_est_deg', 'gyr_x', 'gyr_y', 'gyr_z',
    'acc_x', 'acc_y', 'acc_z', 'thr', 'tq_roll', 'tq_pitch', 'tq_yaw',
    'm0', 'm1', 'm2', 'm3', 'acc_sp_x', 'acc_sp_y', 'acc_sp_z',
    'guard', 'gc', 'ml', 'ld', 'launched',
)

def _default_log_path():
    """logs/flight-<YYYYmmdd-HHMMSS>.csv so consecutive runs never overwrite."""
    stamp = time.strftime('%Y%m%d-%H%M%S')
    return os.path.join(LOG_DIR, 'flight-%s.csv' % stamp)


class FlightLog:
    """Buffered CSV writer for the flight path (telemetry + setpoints + guard)."""

    def __init__(self, path, meta):
        self.path = path
        self.events = []
        self.rows = 0
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.fh = open(path, 'w', encoding='utf-8', buffering=1 << 16)
        if not LOG_QUIET:
            for key, val in meta:
                self.fh.write('# %s %s\n' % (key, val))
            self.fh.write('# columns ' + ' '.join(LOG_FIELDS) + '\n')
        self.fh.write(','.join(LOG_FIELDS) + '\n')

    def row(self, values):
        self.fh.write(','.join(('%.6g' % v) if isinstance(v, float) else str(v)
                               for v in values) + '\n')
        self.rows += 1

    def event(self, t_rel, name, detail=''):
        # collapse repeats (the takeoff kick fires every loop until handover)
        if self.events and self.events[-1][1] == name and self.events[-1][2] == detail:
            return
        self.events.append((round(t_rel, 3), name, detail))

    def close(self, summary=None):
        if summary is not None:
            self.fh.write('# summary ' + ' '.join('%s=%s' % (k, v)
                                                  for k, v in summary.items()) + '\n')
        self.fh.close()


def wrap_deg(rad):
    return (math.degrees(rad) + 180.0) % 360.0 - 180.0


# --- MAVLink-style mission commands (MAV_CMD_NAV_*) ------------------------
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_TURNS = 18
MAV_CMD_NAV_LOITER_TIME = 19
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
FC_MISSION_MAX_ITEMS = 32
_MAV_NAME = {MAV_CMD_NAV_WAYPOINT: 'WAYPOINT', MAV_CMD_NAV_LOITER_TURNS: 'LOITER',
             MAV_CMD_NAV_LOITER_TIME: 'LOITERTIME', MAV_CMD_NAV_RETURN_TO_LAUNCH: 'RTL',
             MAV_CMD_NAV_LAND: 'LAND', MAV_CMD_NAV_TAKEOFF: 'TAKEOFF'}


def parse_mission_text(text):
    """Parse 'cmd p1 p2 p3 p4 x y z; ...' into (cmd, p1..p4, x, y, z) tuples."""
    items = []
    for tok in text.replace(',', ' ').split(';'):
        tok = tok.split('#', 1)[0].strip()
        if not tok:
            continue
        parts = tok.split()
        cmd = int(parts[0])
        vals = []
        for p in parts[1:]:
            if p.lower() in ('n', 'nan'):
                vals.append(float('nan'))
            else:
                vals.append(float(p))
        if len(vals) != 7:
            raise ValueError(f'mot lenh can 8 truong, co {len(vals) + 1}: {tok!r}')
        items.append((cmd, vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6]))
    return items


def default_mission(mode, alt, dur):
    """Minimal fallback mission (takeoff -> waypoint hold -> land) so that bare
    CLI invocations keep working; the demo missions are always SHIPPED as a
    MAVLink-style command string (GZ_MISSION / GZ_MISSION_FILE), never edited
    into the controller."""
    del mode
    return [
        (MAV_CMD_NAV_TAKEOFF, 0.0, 0.0, 0.0, float('nan'), 0.0, 0.0, alt),
        (MAV_CMD_NAV_WAYPOINT, dur, XY_ACC_R, 0.0, float('nan'), 0.0, 0.0, alt),
        (MAV_CMD_NAV_LAND, 0.0, 0.0, 0.0, float('nan'), 0.0, 0.0, 0.0),
    ]


def load_mission():
    """Build the command sequence to SHIP into the C navigator (fc_mission.c).

    The mission travels as a MAVLink-style string, like a GCS uploading
    MISSION_ITEMs:  GZ_MISSION='cmd p1..p4 x y z ; cmd ...'  or a file via
    GZ_MISSION_FILE (one item per line).  With neither, a minimal hold mission
    is sent so bare CLI invocations (acceptance tests/README) work unchanged.
    """
    text = os.environ.get('GZ_MISSION')
    if text:
        items = parse_mission_text(text)
        print('-> NAN mission GZ_MISSION (%d lenh): %s' %
              (len(items), ' > '.join(_MAV_NAME.get(it[0], str(it[0])) for it in items)), flush=True)
        return items
    path = os.environ.get('GZ_MISSION_FILE')
    if path:
        items = parse_mission_text('\n'.join(open(path).readlines()))
        print('-> NAN mission GZ_MISSION_FILE (%d lenh): %s' %
              (len(items), ' > '.join(_MAV_NAME.get(it[0], str(it[0])) for it in items)), flush=True)
        return items
    mission = default_mission(MISSION, ALT, DUR)
    print('-> mission mac dinh: ' + ' > '.join(_MAV_NAME.get(it[0], str(it[0])) for it in mission), flush=True)
    return mission

STAR = float(os.environ.get('GZ_START_ALT', '3.0'))
K = float(os.environ.get('GZ_K', '8.54858e-06'))
MG = float(os.environ.get('GZ_MG', '20.25'))
DBG = os.environ.get('GZ_DBG', '0') == '1'
WMAX = 1000.0

# --- ctypes bindings (mirror fc_types.h / fc_position.h / fc_attitude.h / fc_mixer.h)
class EstimatorState(ctypes.Structure):
    _fields_ = [
        ('q_w', ctypes.c_float), ('q_x', ctypes.c_float), ('q_y', ctypes.c_float), ('q_z', ctypes.c_float),
        ('roll_rad', ctypes.c_float), ('pitch_rad', ctypes.c_float), ('yaw_rad', ctypes.c_float),
        ('gyro_rad_s', ctypes.c_float * 3),
        ('accel_mps2', ctypes.c_float * 3),
        ('sample_age_us', ctypes.c_uint32), ('flags', ctypes.c_uint32),
        ('pos_ned', ctypes.c_float * 3), ('vel_ned', ctypes.c_float * 3),
        ('pos_valid', ctypes.c_uint32),
        ('rest_count', ctypes.c_uint32),
        ('at_rest', ctypes.c_uint8),
    ]

class PositionConfig(ctypes.Structure):
    _fields_ = [
        ('gain_pos', ctypes.c_float * 3),
        ('gain_vel_p', ctypes.c_float * 3),
        ('gain_vel_i', ctypes.c_float * 3),
        ('gain_vel_d', ctypes.c_float * 3),
        ('lim_vel_h', ctypes.c_float), ('lim_vel_up', ctypes.c_float), ('lim_vel_down', ctypes.c_float),
        ('lim_thr_min', ctypes.c_float), ('lim_thr_max', ctypes.c_float), ('lim_thr_xy_margin', ctypes.c_float),
        ('lim_tilt', ctypes.c_float), ('lim_acc_h', ctypes.c_float), ('hover_thrust', ctypes.c_float), ('gravity', ctypes.c_float),
        ('decouple_alt', ctypes.c_uint32), ('z_up', ctypes.c_uint32),
        ('vel_int', ctypes.c_float * 3), ('thr_sp_prev', ctypes.c_float * 3),
        ('vel_prev', ctypes.c_float * 3), ('vel_dot', ctypes.c_float * 3),
        ('vel_dot_cutoff', ctypes.c_float), ('vel_dot_valid', ctypes.c_uint32),
        ('initialized', ctypes.c_uint32),
    ]

class PositionSetpoint(ctypes.Structure):
    _fields_ = [
        ('position_ned', ctypes.c_float * 3),
        ('velocity_ned', ctypes.c_float * 3),
        ('acceleration_ned', ctypes.c_float * 3),
        ('yaw_ned', ctypes.c_float), ('yawspeed', ctypes.c_float),
    ]

class PositionOut(ctypes.Structure):
    _fields_ = [
        ('acc_sp_ned', ctypes.c_float * 3),
        ('thrust_ned', ctypes.c_float * 3),
        ('attitude_sp', ctypes.c_float * 4),
        ('yaw_rate_sp', ctypes.c_float),
        ('valid', ctypes.c_uint32),
    ]

class AttitudeConfig(ctypes.Structure):
    _fields_ = [
        ('gain_att', ctypes.c_float * 3),
        ('yaw_w', ctypes.c_float),
        ('lim_rate', ctypes.c_float * 3),
        ('initialized', ctypes.c_uint32),
    ]

class RateConfig(ctypes.Structure):
    _fields_ = [
        ('gain_rate_k', ctypes.c_float * 3),
        ('gain_rate_p', ctypes.c_float * 3),
        ('gain_rate_i', ctypes.c_float * 3),
        ('gain_rate_d', ctypes.c_float * 3),
        ('gain_rate_ff', ctypes.c_float * 3),
        ('lim_rate_int', ctypes.c_float * 3),
        ('yaw_tq_cutoff', ctypes.c_float),
        ('rate_int', ctypes.c_float * 3),
        ('gyro_prev', ctypes.c_float * 3),
        ('gyro_dt', ctypes.c_float),
        ('yaw_tq_lpf', ctypes.c_float),
        ('yaw_lpf_enabled', ctypes.c_uint32), ('initialized', ctypes.c_uint32),
    ]

class MixerConfig(ctypes.Structure):
    _fields_ = [
        # Phase F: PX4 control_allocator (CA_* geometry, 4001_quad_x defaults)
        ('ca_rotor_count', ctypes.c_uint32),
        ('ca_rotor_px', ctypes.c_float * 4),
        ('ca_rotor_py', ctypes.c_float * 4),
        ('ca_rotor_pz', ctypes.c_float * 4),
        ('ca_rotor_km', ctypes.c_float * 4),
        ('ca_rotor_ct', ctypes.c_float * 4),
        ('max_slew_norm', ctypes.c_float),
        ('previous_normalized', ctypes.c_float * 4),
        ('initialized', ctypes.c_uint32),
    ]

class TorqueCmd(ctypes.Structure):
    _fields_ = [('roll_torque', ctypes.c_float), ('pitch_torque', ctypes.c_float), ('yaw_torque', ctypes.c_float)]

# fc_guard_config_t / _state_t / _out_t (fc_guard.h)
# A.1/A.3: roll + pitch checked separately (FD_FAIL_R/P) with a sustained
# trigger time (FD_FAIL_*_TTRI) before STAB/RECOVER trip.
class GuardConfig(ctypes.Structure):
    _fields_ = [
        ('stab_roll_enter_rad', ctypes.c_float), ('stab_pitch_enter_rad', ctypes.c_float),
        ('stab_rate_enter_rad_s', ctypes.c_float), ('stab_enter_t_s', ctypes.c_float),
        ('stab_roll_exit_rad', ctypes.c_float), ('stab_pitch_exit_rad', ctypes.c_float),
        ('stab_rate_exit_rad_s', ctypes.c_float), ('stab_hold_s', ctypes.c_float),
        ('stab_gain', ctypes.c_float), ('stab_damp', ctypes.c_float),
        ('stab_rate_lim', ctypes.c_float), ('stab_rate_int_lim', ctypes.c_float),
        ('stab_thrust_hold', ctypes.c_float),
        ('rec_roll_enter_rad', ctypes.c_float), ('rec_pitch_enter_rad', ctypes.c_float),
        ('rec_air_m', ctypes.c_float), ('rec_enter_t_s', ctypes.c_float),
        ('rec_roll_exit_rad', ctypes.c_float), ('rec_pitch_exit_rad', ctypes.c_float),
        ('rec_gain', ctypes.c_float), ('rec_rate_lim', ctypes.c_float),
        ('rec_rate_int_lim', ctypes.c_float), ('rec_thrust_hold', ctypes.c_float),
        ('rec_timeout_s', ctypes.c_float),
        ('initialized', ctypes.c_uint32),
    ]

class GuardState(ctypes.Structure):
    _fields_ = [
        ('mode', ctypes.c_uint32),
        ('stable_since', ctypes.c_float), ('rec_since', ctypes.c_float),
        ('stab_enter_since', ctypes.c_float), ('rec_enter_since', ctypes.c_float),
        ('base_lim_rate', ctypes.c_float * 3),
        ('base_gain_rate_p', ctypes.c_float * 3),
        ('base_gain_rate_d', ctypes.c_float * 3),
        ('base_lim_rate_int', ctypes.c_float * 3),
        ('overridden', ctypes.c_uint32), ('initialized', ctypes.c_uint32),
    ]

class GuardOut(ctypes.Structure):
    _fields_ = [
        ('mode', ctypes.c_uint32), ('mode_changed', ctypes.c_uint32),
        ('teleport_now', ctypes.c_uint32), ('thrust_hold', ctypes.c_float),
        ('valid', ctypes.c_uint32),
    ]

# fc_land_config_t / _state_t / _out_t (fc_land.h)
class LandConfig(ctypes.Structure):
    _fields_ = [
        ('z_vel_max', ctypes.c_float), ('xy_vel_max', ctypes.c_float),
        ('rot_max_rad_s', ctypes.c_float), ('alt_gnd_m', ctypes.c_float),
        ('thr_min', ctypes.c_float), ('thr_hover', ctypes.c_float),
        ('gc_frac_hi', ctypes.c_float), ('ml_frac_lo', ctypes.c_float),
        ('freefall_accel', ctypes.c_float),
        ('t_ground_s', ctypes.c_float), ('t_maybe_s', ctypes.c_float),
        ('t_landed_s', ctypes.c_float), ('t_freefall_s', ctypes.c_float),
        ('initialized', ctypes.c_uint32),
    ]

class LandState(ctypes.Structure):
    _fields_ = [
        ('freefall_pending', ctypes.c_uint32), ('ground_contact', ctypes.c_uint32),
        ('maybe_landed', ctypes.c_uint32), ('landed', ctypes.c_uint32),
        ('t_freefall', ctypes.c_float), ('t_ground', ctypes.c_float),
        ('t_maybe', ctypes.c_float), ('t_landed', ctypes.c_float),
        ('initialized', ctypes.c_uint32),
    ]

class LandOut(ctypes.Structure):
    _fields_ = [
        ('ground_contact', ctypes.c_uint32), ('maybe_landed', ctypes.c_uint32),
        ('landed', ctypes.c_uint32), ('freefall', ctypes.c_uint32),
    ]

class MotorOutput(ctypes.Structure):
    _fields_ = [
        ('normalized', ctypes.c_float * 4),
        ('pwm', ctypes.c_uint8 * 4),
        ('valid', ctypes.c_bool), ('armed', ctypes.c_bool),
        ('timestamp_ms', ctypes.c_uint32),
    ]

# fc_estimator_config_t (fc_estimator.h)
class EstimatorConfig(ctypes.Structure):
    _fields_ = [
        ('gravity_mps2', ctypes.c_float),
        ('accel_correction_gain', ctypes.c_float),
        ('mag_correction_gain', ctypes.c_float),
        ('pitch_sign', ctypes.c_float),
        ('accel_gate_frac', ctypes.c_float),
        ('calibration_samples', ctypes.c_uint32),
        ('calibration_count', ctypes.c_uint32),
        ('gyro_bias_rad_s', ctypes.c_float * 3),
        ('gyro_sum_rad_s', ctypes.c_float * 3),
        ('previous_timestamp_us', ctypes.c_uint64),
        ('initialized', ctypes.c_bool),
        ('gyro_bias_tau_s', ctypes.c_float),
        ('rest_gyro_rad_s', ctypes.c_float),
        ('rest_accel_frac', ctypes.c_float),
        ('rest_speed_m_s', ctypes.c_float),
        ('rest_hold_samples', ctypes.c_uint32),
        ('tilt_gate_rad', ctypes.c_float),
    ]

# fc_imu_sample_t (fc_types.h)
class ImuSample(ctypes.Structure):
    _fields_ = [
        ('timestamp_us', ctypes.c_uint64),
        ('sequence', ctypes.c_uint32),
        ('accel_mps2', ctypes.c_float * 3),
        ('gyro_rad_s', ctypes.c_float * 3),
        ('mag_ut', ctypes.c_float * 3),
        ('mag_valid', ctypes.c_uint32),
        ('status', ctypes.c_uint32),
    ]

# fc_mission_config_t / fc_mission_item_t / fc_mission_t (fc_mission.h)
class MissionConfig(ctypes.Structure):
    _fields_ = [
        ('loiter_omega', ctypes.c_float),
        ('loiter_ramp_s', ctypes.c_float),
        ('accept_xy', ctypes.c_float),
        ('alt_accept', ctypes.c_float),
        ('climb_rate', ctypes.c_float),
        ('descend_rate', ctypes.c_float),
        ('land_alt', ctypes.c_float),
    ]


class MissionItem(ctypes.Structure):
    _fields_ = [
        ('cmd', ctypes.c_uint32),
        ('p1', ctypes.c_float), ('p2', ctypes.c_float), ('p3', ctypes.c_float), ('p4', ctypes.c_float),
        ('x', ctypes.c_float), ('y', ctypes.c_float), ('z', ctypes.c_float),
    ]


class Mission(ctypes.Structure):
    _fields_ = [
        ('items', MissionItem * FC_MISSION_MAX_ITEMS),
        ('count', ctypes.c_uint32),
        ('index', ctypes.c_uint32),
        ('active', ctypes.c_uint32),
        ('mission_t', ctypes.c_float),
        ('leg_t', ctypes.c_float),
        ('hold_t', ctypes.c_float),
        ('ref_alt', ctypes.c_float),
        ('tgt_x', ctypes.c_float),
        ('tgt_y', ctypes.c_float),
        ('phi0', ctypes.c_float),
        ('laps_full', ctypes.c_float),
        ('ref_alt_prev', ctypes.c_float),
    ]


_LIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build', 'px4_ctl.so')
if not os.path.exists(_LIB_PATH):
    sys.exit('missing ' + _LIB_PATH + ' (run controller/build with gcc first)')
_lib = ctypes.CDLL(_LIB_PATH)

_lib.FC_Position_ConfigDefault.argtypes = [ctypes.POINTER(PositionConfig)]
_lib.FC_Position_ConfigDefault.restype = None
_lib.FC_Position_Update.argtypes = [ctypes.POINTER(PositionConfig), ctypes.POINTER(EstimatorState),
                                    ctypes.POINTER(PositionSetpoint), ctypes.c_float, ctypes.POINTER(PositionOut)]
_lib.FC_Position_Update.restype = ctypes.c_bool
_lib.FC_Mission_ConfigDefault.argtypes = [ctypes.POINTER(MissionConfig)]
_lib.FC_Mission_ConfigDefault.restype = None
_lib.FC_Mission_Init.argtypes = [ctypes.POINTER(Mission)]
_lib.FC_Mission_Init.restype = None
_lib.FC_Mission_Append.argtypes = [ctypes.POINTER(Mission), ctypes.c_uint32,
                                   ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                   ctypes.c_float, ctypes.c_float, ctypes.c_float]
_lib.FC_Mission_Append.restype = ctypes.c_bool
_lib.FC_Mission_Start.argtypes = [ctypes.POINTER(Mission), ctypes.POINTER(EstimatorState)]
_lib.FC_Mission_Start.restype = ctypes.c_bool
_lib.FC_Mission_Active.argtypes = [ctypes.POINTER(Mission)]
_lib.FC_Mission_Active.restype = ctypes.c_uint32
_lib.FC_Mission_Index.argtypes = [ctypes.POINTER(Mission)]
_lib.FC_Mission_Index.restype = ctypes.c_uint32
_lib.FC_Mission_Update.argtypes = [ctypes.POINTER(MissionConfig), ctypes.POINTER(Mission),
                                   ctypes.POINTER(EstimatorState), ctypes.c_float,
                                   ctypes.POINTER(PositionSetpoint)]
_lib.FC_Mission_Update.restype = ctypes.c_bool
_lib.FC_Attitude_ConfigDefault.argtypes = [ctypes.POINTER(AttitudeConfig)]
_lib.FC_Attitude_ConfigDefault.restype = None
_lib.FC_Attitude_Update.argtypes = [ctypes.POINTER(AttitudeConfig), ctypes.POINTER(EstimatorState),
                                    ctypes.POINTER(ctypes.c_float), ctypes.c_float,
                                    ctypes.POINTER(ctypes.c_float)]
_lib.FC_Attitude_Update.restype = ctypes.c_bool
_lib.FC_RateControl_ConfigDefault.argtypes = [ctypes.POINTER(RateConfig)]
_lib.FC_RateControl_ConfigDefault.restype = None
_lib.FC_RateControl_Reset.argtypes = [ctypes.POINTER(RateConfig)]
_lib.FC_RateControl_Reset.restype = None
_lib.FC_RateControl_Update.argtypes = [ctypes.POINTER(RateConfig), ctypes.POINTER(ctypes.c_float),
                                       ctypes.POINTER(ctypes.c_float), ctypes.c_float,
                                       ctypes.c_bool, ctypes.POINTER(ctypes.c_float)]
_lib.FC_RateControl_Update.restype = ctypes.c_bool
_lib.FC_Mixer_ConfigDefault.argtypes = [ctypes.POINTER(MixerConfig)]
_lib.FC_Mixer_ConfigDefault.restype = None
_lib.FC_Mixer_Compute.argtypes = [ctypes.POINTER(MixerConfig), ctypes.POINTER(TorqueCmd),
                                  ctypes.c_float, ctypes.c_bool, ctypes.POINTER(MotorOutput)]
_lib.FC_Mixer_Compute.restype = ctypes.c_bool
_lib.FC_Estimator_ConfigDefault.argtypes = [ctypes.POINTER(EstimatorConfig)]
_lib.FC_Estimator_ConfigDefault.restype = None
_lib.FC_Estimator_Init.argtypes = [ctypes.POINTER(EstimatorConfig), ctypes.POINTER(EstimatorState)]
_lib.FC_Estimator_Init.restype = None
_lib.FC_Estimator_Update.argtypes = [ctypes.POINTER(EstimatorConfig), ctypes.POINTER(EstimatorState),
                                     ctypes.POINTER(ImuSample)]
_lib.FC_Estimator_Update.restype = ctypes.c_bool
_lib.FC_Estimator_FeedPosition.argtypes = [
    ctypes.POINTER(EstimatorState),
    ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
_lib.FC_Estimator_FeedPosition.restype = None
_lib.FC_Guard_ConfigDefault.argtypes = [ctypes.POINTER(GuardConfig)]
_lib.FC_Guard_ConfigDefault.restype = None
_lib.FC_Guard_Init.argtypes = [ctypes.POINTER(GuardConfig), ctypes.POINTER(GuardState),
                               ctypes.POINTER(AttitudeConfig), ctypes.POINTER(RateConfig)]
_lib.FC_Guard_Init.restype = None
_lib.FC_Guard_Update.argtypes = [ctypes.POINTER(GuardConfig), ctypes.POINTER(GuardState),
                                 ctypes.POINTER(AttitudeConfig), ctypes.POINTER(RateConfig),
                                 ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                 ctypes.c_float, ctypes.POINTER(GuardOut)]
_lib.FC_Guard_Update.restype = ctypes.c_bool
_lib.FC_Guard_Reset.argtypes = [ctypes.POINTER(GuardConfig), ctypes.POINTER(GuardState),
                                ctypes.POINTER(AttitudeConfig), ctypes.POINTER(RateConfig)]
_lib.FC_Guard_Reset.restype = None
_lib.FC_Land_ConfigDefault.argtypes = [ctypes.POINTER(LandConfig)]
_lib.FC_Land_ConfigDefault.restype = None
_lib.FC_Land_Init.argtypes = [ctypes.POINTER(LandConfig), ctypes.POINTER(LandState)]
_lib.FC_Land_Init.restype = None
_lib.FC_Land_Update.argtypes = [ctypes.POINTER(LandConfig), ctypes.POINTER(LandState),
                                ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                ctypes.POINTER(LandOut)]
_lib.FC_Land_Update.restype = ctypes.c_bool

pos_cfg = PositionConfig()
att_cfg = AttitudeConfig()
rate_cfg = RateConfig()
mix_cfg = MixerConfig()
est_cfg = EstimatorConfig()
_lib.FC_Position_ConfigDefault(ctypes.byref(pos_cfg))
_lib.FC_Attitude_ConfigDefault(ctypes.byref(att_cfg))
_lib.FC_RateControl_ConfigDefault(ctypes.byref(rate_cfg))
_lib.FC_Mixer_ConfigDefault(ctypes.byref(mix_cfg))
_lib.FC_Estimator_ConfigDefault(ctypes.byref(est_cfg))
# PX4-style: feed NED (pos/vel + setpoint z negative). ConfigDefault already sets z_up=0.
pos_cfg.gain_pos[2] = 0.9
pos_cfg.gain_vel_p[2] = 3.0
pos_cfg.gain_vel_i[2] = 0.0
pos_cfg.gain_vel_d[2] = 1.0
pos_cfg.vel_dot_cutoff = 1.2
pos_cfg.lim_vel_up = 0.3
pos_cfg.lim_vel_down = 0.3
pos_cfg.lim_acc_h = ACC_H
rate_cfg.gain_rate_p[0] = 0.24
rate_cfg.gain_rate_p[1] = 0.24
rate_cfg.gain_rate_i[0] = 0.04
rate_cfg.gain_rate_i[1] = 0.04
rate_cfg.gain_rate_d[0] = 0.0
rate_cfg.gain_rate_d[1] = 0.0
att_cfg.lim_rate[0] = 2.0
att_cfg.lim_rate[1] = 2.0
rate_cfg.lim_rate_int[0] = 0.16
rate_cfg.lim_rate_int[1] = 0.16
# Phase F: the mixer now normalizes torque -1..1 per PX4 control_allocator
# (unit torque swings a motor by ~0.707 for roll/pitch, ~1.0 for yaw), so the
# old roll/pitch_scale=0.284 / yaw_scale=0.25 authority moved into gain_rate_k:
#    k_roll/pitch = 0.284/0.707 = 0.402,  k_yaw = 0.25/1.0 = 0.25.
# The integral clamps are scaled the same way to keep the validated authority.
rate_cfg.gain_rate_k[0] = 0.402
rate_cfg.gain_rate_k[1] = 0.402
rate_cfg.gain_rate_k[2] = 0.25
rate_cfg.lim_rate_int[0] = 0.064
rate_cfg.lim_rate_int[1] = 0.064
rate_cfg.lim_rate_int[2] = 0.04

# --- C land detector + attitude guard (Phase E) --------------------------
# FC_Guard_Init snapshots the base att/rate gains above; from now on the C
# guard owns every STAB/RECOVER gain override and restore.
guard_cfg = GuardConfig()
guard_state = GuardState()
guard_out = GuardOut()
land_cfg = LandConfig()
land_state = LandState()
land_out = LandOut()
_lib.FC_Guard_ConfigDefault(ctypes.byref(guard_cfg))
_lib.FC_Guard_Init(ctypes.byref(guard_cfg), ctypes.byref(guard_state),
                    ctypes.byref(att_cfg), ctypes.byref(rate_cfg))
_lib.FC_Land_ConfigDefault(ctypes.byref(land_cfg))
_lib.FC_Land_Init(ctypes.byref(land_cfg), ctypes.byref(land_state))

# --- state (GIL-locked floats only) ---------------------------------------
st = {'z': None, 'x': None, 'y': None,
      'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0, 'has_q': False,
      'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
      'ax': 0.0, 'ay': 0.0, 'az': 0.0,
      'mx': 0.0, 'my': 0.0, 'mz': 0.0, 'has_mag': False}


def on_pose(msg):
    for p in msg.pose:
        if p.name == MODEL:
            st['x'] = p.position.x
            st['y'] = p.position.y
            st['z'] = p.position.z
            st['qx'], st['qy'], st['qz'], st['qw'] = (
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            st['has_q'] = True
            return


GYRO_LPF = float(os.environ.get('GZ_GYRO_LPF', '0.3'))  # EMA alpha for sim gyro noise
ACCEL_LPF = float(os.environ.get('GZ_ACCEL_LPF', '0.3'))
MAG_LPF = float(os.environ.get('GZ_MAG_LPF', '0.3'))
GYRO_MAX = float(os.environ.get('GZ_GYRO_MAX', '40.0'))  # rad/s sanity gate (mirrors firmware validation)
ACCEL_MAX = float(os.environ.get('GZ_ACCEL_MAX', '80.0'))  # m/s^2 sanity gate
_gp = [0.0, 0.0, 0.0]
_ap = [0.0, 0.0, 0.0]
_mp = [0.0, 0.0, 0.0]


def gyro_valid(x, y, z):
    return (abs(x) <= GYRO_MAX and abs(y) <= GYRO_MAX and abs(z) <= GYRO_MAX
            and math.isfinite(x) and math.isfinite(y) and math.isfinite(z))


def accel_valid(x, y, z):
    return (abs(x) <= ACCEL_MAX and abs(y) <= ACCEL_MAX and abs(z) <= ACCEL_MAX
            and math.isfinite(x) and math.isfinite(y) and math.isfinite(z))


def on_imu(msg):
    x, y, z = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
    if not gyro_valid(x, y, z):
        return  # drop out-of-range / NaN sample like firmware does
    _gp[0] += GYRO_LPF * (x - _gp[0])
    _gp[1] += GYRO_LPF * (y - _gp[1])
    _gp[2] += GYRO_LPF * (z - _gp[2])
    st['gx'], st['gy'], st['gz'] = _gp[0], _gp[1], _gp[2]

    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z
    if not accel_valid(ax, ay, az):
        return
    _ap[0] += ACCEL_LPF * (ax - _ap[0])
    _ap[1] += ACCEL_LPF * (ay - _ap[1])
    _ap[2] += ACCEL_LPF * (az - _ap[2])
    st['ax'], st['ay'], st['az'] = _ap[0], _ap[1], _ap[2]


def on_mag(msg):
    f = msg.field_tesla
    mx = f.x * 1.0e6
    my = f.y * 1.0e6
    mz = f.z * 1.0e6
    if not math.isfinite(mx) or not math.isfinite(my) or not math.isfinite(mz):
        return
    _mp[0] += MAG_LPF * (mx - _mp[0])
    _mp[1] += MAG_LPF * (my - _mp[1])
    _mp[2] += MAG_LPF * (mz - _mp[2])
    st['mx'], st['my'], st['mz'] = _mp[0], _mp[1], _mp[2]
    st['has_mag'] = True


node = _tr.Node()
node.subscribe(_pose_pb.Pose_V, POSE, on_pose)
node.subscribe(_imu_pb.IMU, IMU, on_imu)
node.subscribe(_mag_pb.Magnetometer, MAG, on_mag)
_opts = _tr.AdvertiseMessageOptions()
motor_pub = node.advertise(MOTOR, _act_pb.Actuators, _opts)
_m = _act_pb.Actuators()
_m.velocity.extend([0.0, 0.0, 0.0, 0.0, 0.0])


def publish(w0, w1, w2, w3):
    _m.velocity[:] = [w0, w1, w2, w3, 0.0]
    motor_pub.publish(_m)


def set_pose(x, y, z):
    subprocess.run(['gz', 'service', '-s', '/world/quadcopter_world/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '6000',
                    '--req', f'name: "{MODEL}" position {{ x: {x} y: {y} z: {z} }} '
                             'orientation { x: 0 y: 0 z: 0 w: 1 }'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def alloc_loss(vals):
    # total thrust F = 4*K*w_hover^2; normalized=hover_thrust -> hover.
    # w(n) = sqrt(n / hover_thrust) * w_hover
    hover_w = math.sqrt(MG / (4.0 * K))
    out = []
    for n in vals:
        w = hover_w * math.sqrt(max(0.0, n) / pos_cfg.hover_thrust)
        out.append(max(0.0, min(WMAX, w)))
    return out


def vel_history():
    if not hasattr(vel_history, 'prev'):
        vel_history.prev = (None, None, None, None)
    return vel_history.prev


RECOVER_TILT = float(os.environ.get('GZ_REC_TILT', '100.0'))
EXIT_TILT = float(os.environ.get('GZ_REC_EXIT', '40.0'))
RECOVER_AIR = float(os.environ.get('GZ_REC_AIR', '0.4'))
GROUND_REC_TQ = float(os.environ.get('GZ_REC_GAIN', '3.0'))
REC_RATE = float(os.environ.get('GZ_REC_RATE', '6.28'))
# Airwobble stabilization: tripped as soon as the airframe starts to lean
# (roll OR pitch OR body angular rate, sustained for a trigger time like
# PX4 FD_FAIL_*_TTRI), holds it level and stops mission flight.
STAB_TILT = float(os.environ.get('GZ_STAB_TILT', '25.0'))     # deg, enter
STAB_TILT_EXIT = float(os.environ.get('GZ_STAB_EXIT', '10.0'))  # deg, resume
STAB_RATE = float(os.environ.get('GZ_STAB_RATE', '2.0'))      # rad/s, enter
STAB_TTRI = float(os.environ.get('GZ_STAB_TTRI', '0.3'))      # s sustained to trip
STAB_HOLD = float(os.environ.get('GZ_STAB_HOLD', '1.5'))      # s stable to resume
STAB_GAIN = float(os.environ.get('GZ_STAB_GAIN', '1.6'))      # rate P boost
STAB_DAMP = float(os.environ.get('GZ_STAB_DAMP', '0.01'))     # rate D added
STAB_RATE_LIM = float(os.environ.get('GZ_STAB_RATELIM', '3.0'))  # rad/s
# Teleport-to-takeoff fallback: if recovery cannot right the drone this long,
# put it back at the original takeoff pose and restart the mission cleanly.
RECOVER_TMOUT = float(os.environ.get('GZ_REC_TMOUT', '8.0'))     # s stuck before teleport
RECOVER_TTRI = float(os.environ.get('GZ_REC_TTRI', '0.3'))       # s sustained to trip
TELEPORT_HALT = float(os.environ.get('GZ_TELE_HALT', '2.0'))     # s motor-off at teleport

HOME_X = 0.0
HOME_Y = 0.0
HOME_Z = 0.0
TELE_LOG = []  # (time, reason) of every teleport event for the acceptance record

_GUARD_NORMAL = 0
_GUARD_STAB = 1
_GUARD_RECOVER = 2

_D2R = math.pi / 180.0
# Feed the env-tunable thresholds into the C guard config (defaults match fc_guard.c).
guard_cfg.stab_roll_enter_rad = STAB_TILT * _D2R
guard_cfg.stab_pitch_enter_rad = STAB_TILT * _D2R
guard_cfg.stab_rate_enter_rad_s = STAB_RATE
guard_cfg.stab_enter_t_s = STAB_TTRI
guard_cfg.stab_roll_exit_rad = STAB_TILT_EXIT * _D2R
guard_cfg.stab_pitch_exit_rad = STAB_TILT_EXIT * _D2R
guard_cfg.stab_rate_exit_rad_s = STAB_RATE
guard_cfg.stab_hold_s = STAB_HOLD
guard_cfg.stab_gain = STAB_GAIN
guard_cfg.stab_damp = STAB_DAMP
guard_cfg.stab_rate_lim = STAB_RATE_LIM
guard_cfg.rec_roll_enter_rad = RECOVER_TILT * _D2R
guard_cfg.rec_pitch_enter_rad = RECOVER_TILT * _D2R
guard_cfg.rec_air_m = RECOVER_AIR
guard_cfg.rec_enter_t_s = RECOVER_TTRI
guard_cfg.rec_roll_exit_rad = EXIT_TILT * _D2R
guard_cfg.rec_pitch_exit_rad = EXIT_TILT * _D2R
guard_cfg.rec_gain = GROUND_REC_TQ
guard_cfg.rec_rate_lim = REC_RATE
guard_cfg.rec_timeout_s = RECOVER_TMOUT


def tilt_deg():
    qw = st['qw']; qx = st['qx']; qy = st['qy']; qz = st['qz']
    cos = max(-1.0, min(1.0, 1.0 - 2.0 * (qx * qx + qy * qy)))
    return math.degrees(math.acos(cos))


def guard_mode_name(mode):
    return {_GUARD_NORMAL: 'NORM', _GUARD_STAB: 'STAB', _GUARD_RECOVER: 'RECOV'}.get(mode, '??')


def teleport_to_takeoff(reason, est_cfg, state):
    print(f'=> TELEPORT ve vi tri cat canh ({HOME_X:.1f},{HOME_Y:.1f},{HOME_Z:.2f}) do: {reason}', flush=True)
    publish(0.0, 0.0, 0.0, 0.0)
    time.sleep(0.2)
    set_pose(HOME_X, HOME_Y, HOME_Z)
    time.sleep(TELEPORT_HALT)
    # restart estimator calibration so attitude re-fuses from the level pose
    _lib.FC_Estimator_Init(ctypes.byref(est_cfg), ctypes.byref(state))
    # restore base gains + re-snapshot, and clear the land detector hysteresis
    _lib.FC_Guard_Reset(ctypes.byref(guard_cfg), ctypes.byref(guard_state),
                        ctypes.byref(att_cfg), ctypes.byref(rate_cfg))
    _lib.FC_Land_Init(ctypes.byref(land_cfg), ctypes.byref(land_state))
    state.pos_valid = 1
    print('-> da reset: doi estimator calibrate lai truoc khi cat canh', flush=True)


def main():
    global HOME_X, HOME_Y, HOME_Z
    if os.environ.get('GZ_TELEPORT', '0') == '1':
        set_pose(0.0, 0.0, STAR)  # optional: spawn at STAR altitude
    else:
        # Always start from a known pad pose. Without this the vehicle keeps
        # whatever attitude the previous run left it in, and a run that ended in
        # a guard recovery begins tilted over. The estimator then aligns to that
        # tilt, the controller drives it further, and it tumbles on takeoff --
        # which looks like an estimator fault but is only a dirty initial state.
        set_pose(HOME_X, HOME_Y, HOME_Z)
        time.sleep(1.5)
    print(f'PLAN: clim={CLIMB:+} m/s to {ALT:+.2f} m, giu {DUR}s, land={LAND_RATE} m/s', flush=True)

    state = EstimatorState()
    sp = PositionSetpoint()
    out = PositionOut()
    torque = TorqueCmd()
    motor = MotorOutput()
    sample = ImuSample()
    est_cfg2 = EstimatorConfig()
    _lib.FC_Estimator_ConfigDefault(ctypes.byref(est_cfg2))
    _lib.FC_Estimator_Init(ctypes.byref(est_cfg2), ctypes.byref(state))
    state.pos_valid = 1
    est_ready = False
    guard_mode = _GUARD_NORMAL
    last_guard_mode = _GUARD_NORMAL

    # --- firmware navigator (fc_mission.c) ---------------------------------
    # The Python side only SHIPS the mission to the C navigator (like a GCS
    # uploading MISSION_ITEMs); all path planning/trajectory computation stays
    # in the flight controller.
    msn_cfg = MissionConfig()
    _lib.FC_Mission_ConfigDefault(ctypes.byref(msn_cfg))
    msn_cfg.loiter_omega = LOIT_OMEGA
    msn_cfg.loiter_ramp_s = LOIT_RAMP
    msn_cfg.accept_xy = XY_ACC_R
    msn_cfg.climb_rate = CLIMB_R
    msn_cfg.descend_rate = DESC_R
    mission = Mission()
    _lib.FC_Mission_Init(ctypes.byref(mission))
    mission_items = load_mission()
    sent = 0
    for it in mission_items:
        if _lib.FC_Mission_Append(ctypes.byref(mission), ctypes.c_uint32(it[0]),
                                  ctypes.c_float(it[1]), ctypes.c_float(it[2]),
                                  ctypes.c_float(it[3]), ctypes.c_float(it[4]),
                                  ctypes.c_float(it[5]), ctypes.c_float(it[6]), ctypes.c_float(it[7])):
            sent += 1
    print(f'-> da gui {sent}/{len(mission_items)} lenh vao navigator (C firmware)', flush=True)
    mission_started = False
    prev_idx = 0
    t_at_tgt = None
    z_prev = None
    t_prev = None
    px_prev = py_prev = None
    t_start = time.time()
    last_print = 0.0
    fade = 1.0
    launched = False
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    max_tilt_deg = 0.0
    max_tilt_t = 0.0
    leg_stats = {}
    log_next = 0.0
    stop_reason = 'timeout'
    log = FlightLog(LOG_PATH or _default_log_path(), [
        ('format', 'stm32h750-gaz-sil-csv'),
        ('time', time.strftime('%Y-%m-%dT%H:%M:%S')),
        ('mission', ' | '.join('%d(%s)' % (it[0], _MAV_NAME.get(it[0], '?'))
                               for it in mission_items)),
        ('mission_src', os.environ.get('GZ_MISSION') or os.environ.get('GZ_MISSION_FILE') or 'default'),
        ('log_hz', LOG_HZ),
        ('thr_hover', pos_cfg.hover_thrust),
        ('acc_hor_max', ACC_H),
        ('loiter_omega', LOIT_OMEGA),
        ('loiter_ramp', LOIT_RAMP),
        ('climb_rate', CLIMB_R),
        ('descend_rate', DESC_R),
        ('accept_xy', XY_ACC_R),
        ('guard_stab_deg', STAB_TILT),
        ('guard_recover_deg', RECOVER_TILT),
        ('guard_stab_ttri', STAB_TTRI),
    ])
    print(f'-> flight log: {log.path}', flush=True)

    def log_event(t_now, name, detail=''):
        log.event(t_now - t_start, name, detail)

    def log_row(t_now, leg_cmd, sp_, vels, thr_, guard_, gc, ml, ld):
        log.row((
            t_now - t_start, leg_cmd, st['x'], st['y'], st['z'],
            -sp_.position_ned[0], -sp_.position_ned[1], -sp_.position_ned[2],
            -sp_.velocity_ned[0], -sp_.velocity_ned[1], -sp_.velocity_ned[2],
            vx, vy, vz,
            math.degrees(roll), math.degrees(pitch), yaw_pose_deg, math.degrees(state.yaw_rad),
            math.degrees(state.roll_rad), math.degrees(state.pitch_rad),
            st['gx'], st['gy'], st['gz'], st['ax'], st['ay'], st['az'],
            thr_, torque.roll_torque, torque.pitch_torque, torque.yaw_torque,
            vels[0], vels[1], vels[2], vels[3],
            out.acc_sp_ned[0], out.acc_sp_ned[1], out.acc_sp_ned[2],
            guard_, gc, ml, ld, int(launched),
        ))

    def finish_mission(t_now):
        publish(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.5)
        hold_s = 0.0 if t_at_tgt is None else max(0.0, t_now - t_at_tgt)
        print(f'catxinacnhan: dung tren mat dat z={st["z"]:.3f} '
              f'max_abs_roll={max_abs_roll:.2f}deg max_abs_pitch={max_abs_pitch:.2f}deg '
              f'hold={hold_s:.1f}s', flush=True)
        if TELE_LOG:
            print('teleports_xacnhan:', flush=True)
            for t0, why in TELE_LOG:
                print(f'  +{t0 - t_start:7.1f}s  {why}', flush=True)

    try:
        while True:
            now = time.time()
            z = st['z']
            if z is None or not st['has_q']:
                time.sleep(0.02)
                continue

            # --- feed C estimator with real IMU (gyro + accel + mag) -----------
            sample.timestamp_us = int(time.monotonic() * 1.0e6)
            sample.sequence += 1
            sample.gyro_rad_s[0] = st['gx']
            sample.gyro_rad_s[1] = st['gy']
            sample.gyro_rad_s[2] = st['gz']
            sample.accel_mps2[0] = st['ax']
            sample.accel_mps2[1] = st['ay']
            sample.accel_mps2[2] = st['az']
            sample.mag_ut[0] = st['mx']
            sample.mag_ut[1] = st['my']
            sample.mag_ut[2] = st['mz']
            sample.mag_valid = 1 if st['has_mag'] else 0
            est_ok = _lib.FC_Estimator_Update(ctypes.byref(est_cfg2), ctypes.byref(state), ctypes.byref(sample))
            if not est_ready:
                if not est_cfg2.initialized:
                    # still calibrating gyro bias at rest -> hold motors off
                    publish(0.0, 0.0, 0.0, 0.0)
                    time.sleep(0.01)
                    continue
                est_ready = True
                print('-> estimator C da khoi tao (gyro + accel + mag)', flush=True)
                t_start = now  # restart the takeoff-kick window now that we have a fused attitude
            elif not est_ok:
                # transient sample out of range: hold, do not disarm the mission
                publish(0.0, 0.0, 0.0, 0.0)
                time.sleep(0.005)
                continue

            if z_prev is None:
                z_prev, t_prev = z, now
                px_prev, py_prev = st['x'], st['y']
                HOME_X, HOME_Y, HOME_Z = st['x'], st['y'], z
                print(f'-> HOME = ({HOME_X:.2f},{HOME_Y:.2f},{HOME_Z:.2f}), chuan bi cat canh', flush=True)

            dt = max(min(now - t_prev, 0.02), 0.000125)
            vz = 0.0 if (now - t_prev) > 2.0 else (z - z_prev) / dt
            vx = 0.0 if (now - t_prev) > 2.0 else (st['x'] - px_prev) / dt
            vy = 0.0 if (now - t_prev) > 2.0 else (st['y'] - py_prev) / dt
            if abs(vz) > 10.0:
                vz = 0.0
            if abs(vx) > 10.0:
                vx = 0.0
            if abs(vy) > 10.0:
                vy = 0.0
            z_prev, t_prev = z, now
            px_prev, py_prev = st['x'], st['y']

            # arm the navigator as soon as we know the pad (HOME) and the pose
            if not mission_started:
                _lib.FC_Mission_Start(ctypes.byref(mission), ctypes.byref(state))
                mission_started = True
                t_start = now
                print('-> navigator (C firmware) bat dau, quy dao quy dinh boi firmware', flush=True)

            # current mission leg (from the C navigator) for reporting/disarming
            idx = int(_lib.FC_Mission_Index(ctypes.byref(mission)))
            if idx != prev_idx:
                prev_idx = idx
                if idx < len(mission_items):
                    cur_name = _MAV_NAME.get(mission_items[idx][0], str(mission_items[idx][0]))
                    print(f'-> [{cur_name}] bat dau (lenh #{idx + 1}/{len(mission_items)})', flush=True)
                    if mission_items[idx][0] in (MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_LOITER_TURNS, MAV_CMD_NAV_LOITER_TIME):
                        t_at_tgt = now
                elif idx >= len(mission_items) and not _lib.FC_Mission_Active(ctypes.byref(mission)):
                    print('-> mission hoan tat, cho dat xuong', flush=True)
            if os.environ.get('GZ_YAW_DEBUG'):
                pass  # reserved: estimator yaw diagnostics
            cur_cmd = mission_items[idx][0] if 0 <= idx < len(mission_items) else None
            leg_cmd = cur_cmd if cur_cmd is not None else -1
            leg_name = _MAV_NAME.get(cur_cmd, 'DONE') if cur_cmd is not None else 'DONE'

            # --- mission state machine -----------------------------------------
            # REMOVED: the sequence/trajectory logic now lives entirely in the C
            # navigator (fc_mission.c). The Python side only ships the commands and
            # executes the controller outputs (see the FC_Mission_Update call below).

            # navigator parked at the pad after the LAND leg: cut motors
            if mission_started and not _lib.FC_Mission_Active(ctypes.byref(mission)) and z <= 0.12:
                print('navigator da ha canh xong -> dung dong co', flush=True)
                stop_reason = 'landed_navigator_parked'
                log_event(now, stop_reason, 'z=%.3f' % z)
                finish_mission(now)
                break

            # Takeoff kick: use the validated equal-thrust launch while near the ground.
            # Gentle + short + only when the drone sits level, so an imperfect spawn
            # (sim is nondeterministic run to run) cannot tip the frame into a flip.
            roll = math.atan2(2 * (st['qw'] * st['qx'] + st['qy'] * st['qz']),
                              1 - 2 * (st['qx'] ** 2 + st['qy'] ** 2))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (st['qw'] * st['qy'] - st['qx'] * st['qz']))))
            qw_, qx_, qy_, qz_ = st['qw'], st['qx'], st['qy'], st['qz']
            yaw_pose_deg = wrap_deg(math.atan2(2 * (qw_ * qz_ + qx_ * qy_),
                                               1 - 2 * (qy_ * qy_ + qz_ * qz_)))
            kick = 860.0
            if (idx == 0 and cur_cmd == MAV_CMD_NAV_TAKEOFF and z < 0.30 and
                    (now - t_start) < 0.40 and abs(roll) < 0.10 and abs(pitch) < 0.10):
                # Feedback takeoff: no blind kick. Ramp thrust gently and hand over
                # to the feedback controller the moment the frame starts moving up
                # or begins to tilt, so an imperfect spawn cannot tip into a flip.
                kick += 30.0
                if kick > 905.0 or (now - t_start) > 0.40:
                    launched = True
                else:
                    publish(kick, kick, kick, kick)
                    log_event(now, 'takeoff_kick', 'w=%.0f' % kick)
                    if now >= log_next:
                        log_next = now + 1.0 / max(LOG_HZ, 0.1)
                        log_row(now, leg_cmd, sp, (kick, kick, kick, kick),
                                0.0, guard_mode, 0, 0, 0)
                    continue
            launched = True

            # --- attitude instability watchdog (C guard: STAB / RECOVER / teleport)
            # Every STAB/RECOVER gain override + restore now lives in fc_guard.c; the
            # Python side feeds |roll| and |pitch| separately (A.1, FD_FAIL_R/P style)
            # + body rate + altitude, then acts on the returned mode / hold-thrust /
            # teleport request.
            pose_tilt = tilt_deg()
            roll = math.atan2(2 * (st['qw'] * st['qx'] + st['qy'] * st['qz']),
                              1 - 2 * (st['qx'] ** 2 + st['qy'] ** 2))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (st['qw'] * st['qy'] - st['qx'] * st['qz']))))
            body_rate = math.sqrt(st['gx'] ** 2 + st['gy'] ** 2 + st['gz'] ** 2)
            ok_g = _lib.FC_Guard_Update(ctypes.byref(guard_cfg), ctypes.byref(guard_state),
                                        ctypes.byref(att_cfg), ctypes.byref(rate_cfg),
                                        ctypes.c_float(roll), ctypes.c_float(pitch),
                                        ctypes.c_float(body_rate), ctypes.c_float(z),
                                        ctypes.c_float(dt), ctypes.byref(guard_out))
            if not ok_g or guard_out.valid == 0:
                print('! guard update invalid, disarming', flush=True)
                publish(0.0, 0.0, 0.0, 0.0)
                break
            guard_mode = guard_out.mode
            if guard_out.mode_changed:
                print(f'[{guard_mode_name(guard_mode)}] bat dau: tilt={pose_tilt:.1f}deg '
                      f'roll={math.degrees(roll):.1f} pitch={math.degrees(pitch):.1f} '
                      f'rate={body_rate:.2f}rad/s', flush=True)
                log_event(now, 'guard_' + guard_mode_name(guard_mode).lower(),
                          'tilt=%.1f roll=%.1f pitch=%.1f rate=%.2f'
                          % (pose_tilt, math.degrees(roll), math.degrees(pitch), body_rate))
            if guard_out.teleport_now:
                # stuck: cannot right itself, bring the drone back to the takeoff pad
                log_event(now, 'teleport', 'recovery_timeout')
                teleport_to_takeoff('khong tu dung thang duoc (C guard timeout)', est_cfg2, state)
                TELE_LOG.append((now, 'recovery_timeout'))
                z_prev = None
                t_prev = None
                px_prev = py_prev = None
                _lib.FC_Mission_Start(ctypes.byref(mission), ctypes.byref(state))
                prev_idx = int(_lib.FC_Mission_Index(ctypes.byref(mission)))
                t_at_tgt = None
                t_start = now
                fade = 1.0
                launched = False
                continue

            # --- estimator ground-truth pose feed (NED frame) ------------------
            # Attitude comes from the C estimator (gyro+accel+mag fusion), NOT the
            # pose ground-truth. Position/velocity are still taken from the sim pose
            # (Gazebo +z up = NED -z) and converted to NED for the position loop.
            pos_in = (ctypes.c_float * 3)(st['x'], -st['y'], -z)
            vel_in = (ctypes.c_float * 3)(vx, -vy, -vz)
            _lib.FC_Estimator_FeedPosition(ctypes.byref(state), pos_in, vel_in)
            state.pos_ned[0] = st['x']
            state.pos_ned[1] = -st['y']
            state.pos_ned[2] = -z
            state.vel_ned[0] = vx
            state.vel_ned[1] = -vy
            state.vel_ned[2] = -vz

            # --- position loop --------------------------------------------------
            if guard_mode != _GUARD_NORMAL:
                # Level-hold override commanded by the C guard: drone upright, held in
                # place at the guard's hold thrust. All gain overrides are already
                # applied/restored by fc_guard.c inside att_cfg/rate_cfg.
                att_sp = (ctypes.c_float * 4)(1.0, 0.0, 0.0, 0.0)
                rate_sp = (ctypes.c_float * 3)(0.0, 0.0, 0.0)
                torque_axis = (ctypes.c_float * 3)(0.0, 0.0, 0.0)
                if not _lib.FC_Attitude_Update(ctypes.byref(att_cfg), ctypes.byref(state), att_sp,
                                               ctypes.c_float(0.0), rate_sp):
                    print('! attitude update invalid during hold, disarming', flush=True)
                    publish(0.0, 0.0, 0.0, 0.0)
                    break
                if not _lib.FC_RateControl_Update(ctypes.byref(rate_cfg), rate_sp, state.gyro_rad_s,
                                                  ctypes.c_float(dt), ctypes.c_bool(False), torque_axis):
                    print('! rate update invalid during hold, disarming', flush=True)
                    publish(0.0, 0.0, 0.0, 0.0)
                    break
                torque.roll_torque = torque_axis[0]
                torque.pitch_torque = torque_axis[1]
                torque.yaw_torque = torque_axis[2]
                thr = max(0.0, min(1.0, guard_out.thrust_hold))
                if not _lib.FC_Mixer_Compute(ctypes.byref(mix_cfg), ctypes.byref(torque), ctypes.c_float(thr),
                                             ctypes.c_bool(True), ctypes.byref(motor)):
                    print('! mixer invalid during hold, disarming', flush=True)
                    publish(0.0, 0.0, 0.0, 0.0)
                    break
                vels = alloc_loss(list(motor.normalized))
                publish(*vels)
                tag = guard_mode_name(guard_mode)
                print(f'[{tag}] tilt={pose_tilt:5.1f} r={body_rate:4.2f} z={z:+.2f} '
                      f'tq={torque.roll_torque:+.2f},{torque.pitch_torque:+.2f},{torque.yaw_torque:+.2f} '
                      f'w={(vels[0]+vels[1]+vels[2]+vels[3])/4:6.0f}', flush=True)
                if now >= log_next:
                    log_next = now + 1.0 / max(LOG_HZ, 0.1)
                    log_row(now, leg_cmd, sp, vels, thr, guard_mode, 0, 0, 0)
                time.sleep(0.01)
                continue

            # --- trajectory setpoint from the C mission navigator -------------------
            # The firmware plans the whole flight (climb profile, loiter spiral with
            # analytic velocity/accel feed-forward, RTL, descent) and streams a
            # vehicle_trajectory_setpoint; Python simply tracks it.
            ok_m = _lib.FC_Mission_Update(ctypes.byref(msn_cfg), ctypes.byref(mission),
                                          ctypes.byref(state), ctypes.c_float(dt), ctypes.byref(sp))
            if not ok_m or not mission_started:
                # no active mission: park at the pad on the ground (land_alt) and
                # let the fade below settle it onto the ground for the disarm
                sp.position_ned[0] = 0.0
                sp.position_ned[1] = 0.0
                sp.position_ned[2] = -0.05
                sp.velocity_ned[0] = 0.0
                sp.velocity_ned[1] = 0.0
                sp.velocity_ned[2] = 0.0
                sp.acceleration_ned[0] = 0.0
                sp.acceleration_ned[1] = 0.0
                sp.acceleration_ned[2] = 0.0
                sp.yaw_ned = state.yaw_rad
                sp.yawspeed = 0.0
            ok = _lib.FC_Position_Update(ctypes.byref(pos_cfg), ctypes.byref(state),
                                         ctypes.byref(sp), ctypes.c_float(dt), ctypes.byref(out))
            if not ok or not out.valid:
                print('! position update invalid, disarming', flush=True)
                publish(0.0, 0.0, 0.0, 0.0)
                break

            # --- attitude + rate loop ------------------------------------------
            # PX4 split: mc_att_control (attitude error -> body rate setpoint), then
            # mc_rate_control (rate error -> torque). FC_RateControl_Update is the
            # exact C rate loop; the Python side only marshals the two setpoints.
            att_sp = (ctypes.c_float * 4).from_address(ctypes.addressof(out) + 24)
            yaw_r = out.yaw_rate_sp
            rate_sp = (ctypes.c_float * 3)(0.0, 0.0, 0.0)
            torque_axis = (ctypes.c_float * 3)(0.0, 0.0, 0.0)
            if not _lib.FC_Attitude_Update(ctypes.byref(att_cfg), ctypes.byref(state), att_sp,
                                           ctypes.c_float(yaw_r), rate_sp):
                print('! attitude update invalid, disarming', flush=True)
                publish(0.0, 0.0, 0.0, 0.0)
                break

            # rate loop: feed C rate counters with actual body rates (gyro)
            if not _lib.FC_RateControl_Update(ctypes.byref(rate_cfg), rate_sp, state.gyro_rad_s,
                                              ctypes.c_float(dt), ctypes.c_bool(False), torque_axis):
                print('! rate update invalid, disarming', flush=True)
                publish(0.0, 0.0, 0.0, 0.0)
                break
            torque.roll_torque = torque_axis[0]
            torque.pitch_torque = torque_axis[1]
            torque.yaw_torque = torque_axis[2]

            # --- vertical thrust -------------------------------------------------
            # Phase C: vertical thrust comes from the C position controller output
            # (which now includes gain_vel_d damping + accel feed-forward), instead
            # of the old Python SIL altitude loop. out.thrust_ned is the NED thrust
            # vector; its magnitude equals the normalized collective for the mixer.
            thrust_vec = (out.thrust_ned[0], out.thrust_ned[1], out.thrust_ned[2])
            thr = math.sqrt(thrust_vec[0] ** 2 + thrust_vec[1] ** 2 + thrust_vec[2] ** 2)
            if not math.isfinite(thr):
                thr = 0.0
            thr = max(0.12, min(1.0, thr))

            # --- C land detector (phase E): same feed as firmware, gated by armed.
            # Only finalize when the mission is actually launched (active thrust);
            # during the initial ground calibration the motors are off and the
            # detector rightfully sees a "landed" state we must ignore.
            if launched:
                rot_xy = math.sqrt(st['gx'] ** 2 + st['gy'] ** 2)
                accel_norm = math.sqrt(st['ax'] ** 2 + st['ay'] ** 2 + st['az'] ** 2)
                ok_l = _lib.FC_Land_Update(ctypes.byref(land_cfg), ctypes.byref(land_state),
                                           ctypes.c_float(thr), ctypes.c_float(vz),
                                           ctypes.c_float(math.hypot(vx, vy)),
                                           ctypes.c_float(rot_xy), ctypes.c_float(accel_norm),
                                           ctypes.c_float(z), ctypes.c_float(dt), ctypes.byref(land_out))
                if not ok_l:
                    print('! land update invalid, disarming', flush=True)
                    publish(0.0, 0.0, 0.0, 0.0)
                    break
                if land_out.landed:
                    print('phat hien da cham dat (C land detector) -> dung dong co', flush=True)
                    stop_reason = 'landed_land_detector'
                    log_event(now, stop_reason, 'z=%.3f' % z)
                    finish_mission(now)
                    break

            # --- mixer ----------------------------------------------------------
            if not _lib.FC_Mixer_Compute(ctypes.byref(mix_cfg), ctypes.byref(torque), ctypes.c_float(thr),
                                         ctypes.c_bool(True), ctypes.byref(motor)):
                print('! mixer compute invalid, disarming', flush=True)
                publish(0.0, 0.0, 0.0, 0.0)
                break
            if not motor.valid:
                print('! mixer output invalid, disarming', flush=True)
                publish(0.0, 0.0, 0.0, 0.0)
                break
            vels = alloc_loss(list(motor.normalized))
            if z < 0.25 and (cur_cmd == MAV_CMD_NAV_LAND or not _lib.FC_Mission_Active(ctypes.byref(mission))):
                vels = [v * fade for v in vels]
            publish(*vels)

            roll = math.atan2(2 * (st['qw'] * st['qx'] + st['qy'] * st['qz']),
                              1 - 2 * (st['qx'] ** 2 + st['qy'] ** 2))
            pitch = math.asin(max(-1.0, min(1.0, 2 * (st['qw'] * st['qy'] - st['qx'] * st['qz']))))
            max_abs_roll = max(max_abs_roll, abs(math.degrees(roll)))
            max_abs_pitch = max(max_abs_pitch, abs(math.degrees(pitch)))
            tilt_now = pose_tilt
            if tilt_now > max_tilt_deg:
                max_tilt_deg, max_tilt_t = tilt_now, now - t_start
            ls = leg_stats.setdefault(leg_name, {'t0': now - t_start, 'max_tilt': 0.0,
                                                 'max_roll': 0.0, 'max_pitch': 0.0,
                                                 'min_z': z, 'max_yaw_err': 0.0, 'n': 0})
            ls['max_tilt'] = max(ls['max_tilt'], tilt_now)
            ls['max_roll'] = max(ls['max_roll'], abs(math.degrees(roll)))
            ls['max_pitch'] = max(ls['max_pitch'], abs(math.degrees(pitch)))
            ls['min_z'] = min(ls['min_z'], z)
            ls['max_yaw_err'] = max(ls['max_yaw_err'],
                                    abs(wrap_deg(math.radians(yaw_pose_deg) - state.yaw_rad)))
            ls['n'] += 1
            if now >= log_next:
                log_next = now + 1.0 / max(LOG_HZ, 0.1)
                log_row(now, leg_cmd, sp, vels, thr, guard_mode,
                        land_out.ground_contact, land_out.maybe_landed, land_out.landed)
            if DBG:
                print(f'F: z={z:+.2f} ref={-sp.position_ned[2]:+.2f} vz={vz:+.2f} r={math.degrees(roll):+.1f} '
                      f'p={math.degrees(pitch):+.1f} thr={thr:.3f} tq=({torque.roll_torque:+.2f},'
                      f'{torque.pitch_torque:+.2f},{torque.yaw_torque:+.2f}) w=({"%03d" % vels[0]},'
                      f'{"%03d" % vels[1]},{"%03d" % vels[2]},{"%03d" % vels[3]})', flush=True)
            if now - last_print >= 0.5:
                last_print = now
                hold_s = 0.0 if t_at_tgt is None else max(0.0, now - t_at_tgt)
                print(f'[{leg_name:9s}] z={z:+6.3f} ref={-sp.position_ned[2]:+5.2f} vz={vz:+5.2f} '
                        f'r={math.degrees(roll):+6.1f} p={math.degrees(pitch):+6.1f} '
                        f'yaw_pose={yaw_pose_deg:+7.1f} yaw_est={math.degrees(state.yaw_rad):+7.1f} '
                        f'gz={st["gz"]:+6.3f} tq_y={torque.yaw_torque:+5.2f} '
                        f'w={(vels[0] + vels[1] + vels[2] + vels[3]) / 4:6.0f} hold={hold_s:6.1f}s', flush=True)
            time.sleep(0.01)
    finally:
        close_log(log, t_start, leg_stats, max_tilt_deg, max_tilt_t,
                  max_abs_roll, max_abs_pitch, stop_reason)


def close_log(log, t_start, leg_stats, max_tilt_deg, max_tilt_t,
              max_abs_roll, max_abs_pitch, stop_reason):
    """Flush the flight log and print the per-leg summary used to compare runs."""
    if leg_stats:
        print('%-10s %8s %8s %8s %8s %8s' %
              ('leg', 't0[s]', 'maxtilt', 'maxroll', 'maxpitch', 'maxyawE'), flush=True)
        for name, s in leg_stats.items():
            print('%-10s %8.1f %8.1f %8.1f %8.1f %8.1f' %
                  (name, s['t0'], s['max_tilt'], s['max_roll'], s['max_pitch'],
                   s['max_yaw_err']), flush=True)
    log.close(summary={
        'rows': log.rows,
        'stop': stop_reason,
        'max_tilt_deg': round(max_tilt_deg, 2),
        'max_tilt_t': round(max_tilt_t, 2),
        'max_abs_roll': round(max_abs_roll, 2),
        'max_abs_pitch': round(max_abs_pitch, 2),
        'events': len(log.events),
    })
    print('-> log: %s (%d rows, %d events, stop=%s, max_tilt=%.1fdeg @%.1fs)' %
          (log.path, log.rows, len(log.events), stop_reason, max_tilt_deg, max_tilt_t), flush=True)
    for t0, name, detail in log.events:
        print('   +%7.1fs  %-16s %s' % (t0, name, detail), flush=True)


if __name__ == '__main__':
    try:
        main()
    finally:
        publish(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.2)
        print('done', flush=True)