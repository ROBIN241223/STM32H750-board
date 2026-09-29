import importlib
import math
import os
import time

_msgs = importlib.import_module('gz.msgs')
_pose_pb = importlib.import_module('gz.msgs.pose_v_pb2')
_imu_pb = importlib.import_module('gz.msgs.imu_pb2')
_act_pb = importlib.import_module('gz.msgs.actuators_pb2')
_tr = importlib.import_module('gz.transport')

POSE = '/world/quadcopter_world/pose/info'
IMU = '/x500/imu'
MOTOR = '/x500/command/motor_speed'

st = {'gx': 0.0, 'gy': 0.0, 'gz': 0.0, 'qw': 1.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0,
      'x': None, 'y': None, 'z': None, 'got': False, 'n': 0}


def on_imu(msg):
    av = msg.angular_velocity
    st['gx'], st['gy'], st['gz'] = av.x, av.y, av.z
    st['got'] = True


def on_pose(msg):
    p = msg.pose[0]
    o = p.orientation
    st['qw'], st['qx'], st['qy'], st['qz'] = o.w, o.x, o.y, o.z
    st['x'], st['y'], st['z'] = p.position.x, p.position.y, p.position.z
    st['n'] += 1


node = _tr.Node()
node.subscribe(_imu_pb.IMU, IMU, on_imu)
node.subscribe(_pose_pb.Pose_V, POSE, on_pose)
_opts = _tr.AdvertiseMessageOptions()
motor_pub = node.advertise(MOTOR, _act_pb.Actuators, _opts)
_m = _act_pb.Actuators()
_m.velocity.extend([0.0, 0.0, 0.0, 0.0, 0.0])


def publish(w0, w1, w2, w3):
    _m.velocity[:] = [w0, w1, w2, w3, 0.0]
    motor_pub.publish(_m)


def math_deg(y, x):
    return math.degrees(math.atan2(y, x))


# AHRS mag field
W = float(os.environ.get('GZ_YAW_W', '790.0'))  # base hover-ish speed
S = float(os.environ.get('GZ_YAW_DELTA', '60.0'))  # yaw differential added to (0,1) minus (2,3)
T = float(os.environ.get('GZ_YAW_T', '6.0'))  # seconds of forced differential

time.sleep(1.0)  # let subscribers warm up
print(f'BASELINE spin-up all {W:.0f}', flush=True)
publish(float(W), float(W), float(W), float(W))
time.sleep(2.0)
print(f'GYRO start gz={st["gz"]:+.3f}', flush=True)

# Positive: +S to motors 0(FR ccw),1(BL ccw); -S to 2(FL cw),3(BR cw)
# This matches current mixer "+yaw torque" allocation? delta: m0,m1 get +yaw; m2,m3 get -yaw
publish(float(W) + S, float(W) + S, float(W) - S, float(W) - S)
t0 = time.time()
while time.time() - t0 < T:
    time.sleep(0.2)
    qw, qx, qy, qz = st['qw'], st['qx'], st['qy'], st['qz']
    yaw_pose = math_deg(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    print(f'  t={time.time()-t0:4.1f} gz={st["gz"]:+.3f} z={st["z"] if st["z"] is not None else -99:5.2f} yaw_pose={yaw_pose:+6.1f}', flush=True)

publish(0.0, 0.0, 0.0, 0.0)
time.sleep(0.5)
print('done', flush=True)