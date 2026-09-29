#!/usr/bin/env python3
# Persistent motor-speed publisher used by hover_x500.py.
# Reads lines "w0 w1 w2 w3" from stdin and broadcasts them to
# /x500/command/motor_speed at ~50 Hz from ONE long-lived transport node
# (per-update gz topic -p processes were dropping/interleaving commands).
import importlib, os, sys, time

os.environ.setdefault('GZ_PARTITION', 'stm32_h750_sim')
os.environ.setdefault('GZ_IP', '127.0.0.1')

act_pb = importlib.import_module('gz.msgs.actuators_pb2')
tr = importlib.import_module('gz.transport')


def main():
    node = tr.Node()
    opts = tr.AdvertiseMessageOptions()
    pub = node.advertise('/x500/command/motor_speed', act_pb.Actuators, opts)
    msg = act_pb.Actuators()
    msg.velocity.extend([0.0, 0.0, 0.0, 0.0, 0.0])
    try:
        pub.publish(msg)
    except Exception:
        pass
    vals = [0.0] * 4
    buf = ''
    deadline = 0.0
    while True:
        line = sys.stdin.readline()
        if line == '':
            # controller exited: spin motors down before leaving
            vals = [0.0] * 4
            for _ in range(3):
                try:
                    msg.velocity[:] = vals + [0.0]
                    pub.publish(msg)
                except Exception:
                    pass
                time.sleep(0.05)
            return
        toks = line.split()
        if len(toks) < 4:
            continue
        try:
            vals = [float(x) for x in toks[:4]]
        except ValueError:
            continue
        # broadcast latest at high rate while waiting for the next update
        while time.time() - deadline < 0.02:
            try:
                msg.velocity[:] = vals + [0.0]
                pub.publish(msg)
            except Exception:
                pass
            time.sleep(0.02)


if __name__ == '__main__':
    main()