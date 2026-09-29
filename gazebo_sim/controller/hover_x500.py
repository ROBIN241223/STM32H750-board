#!/usr/bin/env python3
# gz-sim controller for x500: PX4-style attitude (MC_ATT_P) + rate PID
# (rate_control.cpp) + multirotor mixer, with a vertical velocity/altitude
# loop for climb / hold / controlled landing.
#
# Uses gz python bindings only: one transport Node subscribes to
# pose/info + imu and advertises the motor commands at ~30 Hz, so there is
# no per-update process spawning (that was dropping/interleaving commands).

import importlib, math, os, re, subprocess, sys, time

os.environ.setdefault('GZ_PARTITION', 'stm32_h750_sim')
os.environ.setdefault('GZ_IP', '127.0.0.1')

_msgs = importlib.import_module('gz.msgs')
_pose_pb = importlib.import_module('gz.msgs.pose_v_pb2')
_imu_pb = importlib.import_module('gz.msgs.imu_pb2')
_act_pb = importlib.import_module('gz.msgs.actuators_pb2')
_tr = importlib.import_module('gz.transport')

MODEL = 'x500'
POSE = '/world/quadcopter_world/pose/info'
IMU = '/x500/imu'
MOTOR = '/x500/command/motor_speed'

Z_TGT = float(sys.argv[1] if len(sys.argv) > 1 else 2.0)
DUR = float(sys.argv[2] if len(sys.argv) > 2 else 3.0)
CLIMB = max(0.01, float(sys.argv[3] if len(sys.argv) > 3 else 0.15))
LAND_RATE = float(sys.argv[4] if len(sys.argv) > 4 else 0.12)

STAR = float(os.environ.get('GZ_START_ALT', '3.0'))

K = float(os.environ.get('GZ_K', '8.54858e-06'))
MG = float(os.environ.get('GZ_MG', '20.25'))
KPV, KD = float(os.environ.get('GZ_KPV', '0.9')), 3.0
CB = float(os.environ.get('GZ_CB', '1.0'))   # extra thrust while climbing
KAP = float(os.environ.get('GZ_KAP', '4.0'))
KR = float(os.environ.get('GZ_KR', '6.0'))
KR_I = float(os.environ.get('GZ_RI', '1.0'))
DBG = os.environ.get('GZ_DBG', '0') == '1'
INT_LIM = float(os.environ.get('GZ_IL', '4.0'))
RATE_SP_MAX = float(os.environ.get('GZ_RSP', '2.0'))
SIG = float(os.environ.get('GZ_SIG', '1.0'))
SIGA = float(os.environ.get('GZ_SIGA', '1.0'))
GZ_AX = os.environ.get('GZ_AX', 'RP').upper()
J = 0.04
WMIN, WMAX = 0.0, 1000.0
SMIN, SMAX = MG - 4.0, MG + 14.0
KICK_W = float(os.environ.get('GZ_KICK', '880.0'))

# --- state (locked implicitly by the GIL; floats only) ---------------------
st = {'z': None, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0, 'p': 0.0, 'q': 0.0}


def rpy(qx, qy, qz, qw):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qx * qz))))
    return roll, pitch


def on_pose(msg):
    for p in msg.pose:
        if p.name == MODEL:
            st['z'] = p.position.z
            st['qx'], st['qy'], st['qz'], st['qw'] = (
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            return


def on_imu(msg):
    st['p'] = msg.angular_velocity.x
    st['q'] = msg.angular_velocity.y


node = _tr.Node()
node.subscribe(_pose_pb.Pose_V, POSE, on_pose)
node.subscribe(_imu_pb.IMU, IMU, on_imu)
_opts = _tr.AdvertiseMessageOptions()
motor_pub = node.advertise(MOTOR, _act_pb.Actuators, _opts)
_m = _act_pb.Actuators()
_m.velocity.extend([0.0, 0.0, 0.0, 0.0, 0.0])


def publish(v0, v1, v2, v3):
    _m.velocity[:] = [v0, v1, v2, v3, 0.0]
    motor_pub.publish(_m)


def set_pose(x, y, z):
    subprocess.run(['gz', 'service', '-s', '/world/quadcopter_world/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '6000',
                    '--req', f'name: "{MODEL}" position {{ x: {x} y: {y} z: {z} }} '
                             'orientation { x: 0 y: 0 z: 0 w: 1 }'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    if os.environ.get('GZ_TELEPORT', '1') == '1':
        set_pose(2.0, 0.0, STAR)   # optional spawn point; set 0 to see real takeoff
    # hold at hover while subscribers connect
    hvr = math.sqrt(MG / (4.0 * K))
    publish(hvr, hvr, hvr, hvr)

    print(f'PLAN: clim={CLIMB} m/s u {Z_TGT} m, giu {DUR}s, land={LAND_RATE} m/s', flush=True)
    phase = 'climb'
    fade = 1.0
    ref = None
    z_prev, t_prev = None, time.time()
    t_start = time.time()
    t_at_tgt = None
    int_x = int_y = 0.0
    last_print = 0.0
    launched = False

    while True:
        now = time.time()
        z = st['z']
        if z is None:
            time.sleep(0.02)
            continue
        if z_prev is None:
            z_prev, t_prev, ref = z, now, z
        dt = max(now - t_prev, 1e-3)
        vz = (z - z_prev) / dt
        z_prev, t_prev = z, now
        # clamp vz reporting for big teleports
        if abs(vz) > 10.0:
            vz = 0.0

        if abs(st['qw']) > 1.0:
            continue
        roll, pitch = rpy(st['qx'], st['qy'], st['qz'], st['qw'])
        if abs(roll) > 2.0:
            print(f'! INVERTED r={math.degrees(roll):+.1f} p={math.degrees(pitch):+.1f}', flush=True)

        if phase == 'climb':
            ref = min(ref + CLIMB * dt, Z_TGT)
            if ref >= Z_TGT - 1e-6:
                if t_at_tgt is None:
                    t_at_tgt = now
                if now - t_at_tgt >= DUR:
                    phase = 'land'
                    print('-> ha canh tu tu', flush=True)
        elif phase == 'land':
            ref = max(ref - LAND_RATE * dt, 0.0)
            if z < 0.25:
                fade = max(0.0, fade - 0.20 * dt)
                if z <= -0.004 or (fade <= 0.05 and z < 0.04):
                    print('da cham dat -> dung dong co', flush=True)
                    publish(0.0, 0.0, 0.0, 0.0)
                    time.sleep(0.5)
                    print(f'catxinacnhan: da dung tren mat dat z={st["z"]:.3f}', flush=True)
                    break
        else:
            break

        # takeoff kick: only while still near the ground, roughly level
        kick = (phase == 'climb' and z < 0.9 and abs(vz) < 3.0
                and (now - t_start) < 4.0 and abs(roll) < 0.3)
        if kick and not launched:
            publish(KICK_W, KICK_W, KICK_W, KICK_W)
            launched = True
            continue
        launched = True

        # PX4 AttitudeControl.cpp: angular-rate setpoint from the attitude
        # error in BODY frame, eq = 2 * imag(qd o conj(q)). Level target:
        # qd = identity, so eq = 2*imag(conj(q)) = -(2qx, 2qy, 2qz).
        e_roll = -2.0 * st['qx']
        e_pitch = -2.0 * st['qy']
        rd_x = max(-RATE_SP_MAX, min(RATE_SP_MAX, KAP * e_roll))
        rd_y = max(-RATE_SP_MAX, min(RATE_SP_MAX, KAP * e_pitch))
        ex = rd_x - st['p']
        ey = rd_y - st['q']
        int_x += KR_I * ex * dt
        int_y += KR_I * ey * dt
        int_x = max(-INT_LIM, min(INT_LIM, int_x))
        int_y = max(-INT_LIM, min(INT_LIM, int_y))
        aa_x = max(-12.0, min(12.0, KR * ex + int_x))
        aa_y = max(-12.0, min(12.0, KR * ey + int_y))
        if 'R' not in GZ_AX:
            aa_x = 0.0
        if 'P' not in GZ_AX:
            aa_y = 0.0
        tx = SIG * J * aa_x
        ty = SIG * J * aa_y
        L = 0.174
        # x500_base rotor positions: r0(+L,-L) r1(-L,+L) r2(+L,+L) r3(-L,-L).
        # roll pair (y+ = r1,r2 vs y- = r0,r3) -> [-,+,+,-]; pitch pair
        # (x+ = r0,r2 vs x- = r1,r3) -> [-,+,-,+]  (crossed arms). This is
        # NOT the standard PX4 ordering, so the shares are built explicitly.
        cr = tx / (2.0 * L)
        cp = ty / (2.0 * L)
        d0 = -cr - cp
        d1 = +cr + cp
        d2 = +cr - cp
        d3 = -cr + cp
        vt = max(-0.3, min(0.3, KPV * (ref - z)))
        S = MG + KD * (vt - vz)
        if phase == 'climb' and ref < Z_TGT - 1e-6:
            S += 1.0 + CB
        S = max(SMIN, min(SMAX, S))
        if phase == 'land' and z < 0.25:
            S = min(S, MG + 3.0)
        dd = (d0, d1, d2, d3)
        ddm = max(abs(min(dd)), abs(max(dd)))
        if ddm > 1.0:
            dd = tuple(v / ddm for v in dd)
        vels = []
        for d in dd:
            f = (S / 4.0) + d
            w = math.sqrt(max(f, 0.0) / K) if f > 0 else 0.0
            vels.append(max(WMIN, min(WMAX, w)))
        if phase == 'land' and z < 0.25:
            vels = [v * fade for v in vels]
        publish(*vels)
        if DBG:
            print(f'RO: z={z:+.2f} r={math.degrees(roll):+.1f} p={math.degrees(pitch):+.1f} '
                  f'pgyr={st["p"]:+.2f} qgyr={st["q"]:+.2f} aa=({aa_x:+.1f},{aa_y:+.1f}) '
                  f'd=({d0:+.2f},{d1:+.2f},{d2:+.2f},{d3:+.2f}) w=({"%.0f"%vels[0]},{"%.0f"%vels[1]},{"%.0f"%vels[2]},{"%.0f"%vels[3]})',
                  flush=True)
        if now - last_print >= 0.5:
            last_print = now
            print(f'[{phase:5s}] z={z:+.3f} ref={ref:+.2f} vz={vz:+.2f} '
                  f'r={math.degrees(roll):+.1f} p={math.degrees(pitch):+.1f} '
                  f'pgyr={st["p"]:+.2f} qgyr={st["q"]:+.2f} w={(vels[0]+vels[2])/2:.0f}',
                  flush=True)
        time.sleep(0.033)


if __name__ == '__main__':
    try:
        main()
    finally:
        publish(0.0, 0.0, 0.0, 0.0)
        time.sleep(0.2)
        print('done', flush=True)