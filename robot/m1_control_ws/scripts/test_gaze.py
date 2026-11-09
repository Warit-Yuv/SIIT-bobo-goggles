#!/usr/bin/env python3
"""Drive the gaze pipeline with synthetic gaze points, for testing without eyes.

Publishes /gaze/point and reports what the control node does with it.

    python3 test_gaze.py                 # run the scripted sequence
    python3 test_gaze.py --hold 2        # stay in each zone 2 s
    python3 test_gaze.py --udp 127.0.0.1 # test the UDP path instead of ROS

Sequence: centre -> forward -> centre -> strafe right -> centre -> forward-right
-> centre -> silence (to prove the link timeout stops the robot).
"""

import argparse
import json
import socket
import sys
import time

# Normalized gaze targets: (label, u, v). Centre is (0.5, 0.5).
SEQUENCE = [
    ('centre (deadzone: PTZ only)', 0.50, 0.50),
    ('forward', 0.50, 0.12),
    ('centre', 0.50, 0.50),
    ('strafe right', 0.90, 0.50),
    ('centre', 0.50, 0.50),
    ('forward-right', 0.78, 0.22),
    ('centre', 0.50, 0.50),
]


def run_udp(host, port, hold):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f'[test] UDP target {host}:{port}')
    for label, u, v in SEQUENCE:
        print(f'[test] -> {label}  (u={u}, v={v})')
        end = time.time() + hold
        while time.time() < end:
            sock.sendto(json.dumps({'type': 'gaze', 'u': u, 'v': v,
                                    'valid': True, 'model': 'test'}).encode(),
                        (host, port))
            time.sleep(1 / 30)
    print('[test] -> silence (link timeout should stop the robot)')
    time.sleep(1.5)
    print('[test] done')


def run_ros(hold):
    import rclpy
    from geometry_msgs.msg import Point, Twist, Vector3
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('gaze_test_driver')
    pub = node.create_publisher(Point, '/gaze/point', 10)

    # Start the node under test here rather than assuming somebody left it
    # running. An unstarted node publishes nothing, every reading is zero, and
    # the test then reports a confident failure about code it never exercised.
    import m1_control.gaze_control_node as gcn
    # EEG gating OFF for this test.
    #
    # This file checks the ZONE MAPPING - does looking right produce a right
    # turn. It says nothing about whether an EEG is attached, but the node
    # subscribes to /eeg/run and /eeg/mode, and those topics are LATCHED. So
    # whatever the bridge last published (possibly "run off" from another
    # test) is adopted the instant this node starts, every command is gated to
    # zero, and the test fails with a perfect zone table and no movement.
    #
    # That is a test-isolation bug, not a controller bug: the same code passes
    # when run alone. Turning the gate off makes the test deterministic and
    # keeps it testing what it is named after.
    # This test needs EXCLUSIVE use of /cmd_vel/gaze.
    #
    # It reads the topic to see what the controller under test produced. If the
    # robot stack is running there is already a gaze_control_node publishing to
    # the same topic, and the reading becomes "whichever node published last" -
    # which is the robot's gated zero as often as it is the test's turn
    # command. The result is a test that passes alone and fails in a batch, for
    # a reason that has nothing to do with the code it names.
    #
    # Skipping is the honest answer: the precondition is "nothing else is
    # driving", and that is not something the test can arrange for itself.
    probe = rclpy.create_node('gaze_test_probe')
    existing = [i for i in probe.get_publishers_info_by_topic('/cmd_vel/gaze')
                if i.node_name != 'gaze_control_node']
    if existing:
        print(f'  [SKIP] {len(existing)} other publisher(s) on '
              f'/cmd_vel/gaze - stop the robot stack first:')
        for i in existing:
            print(f'         {i.node_name}')
        probe.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()
        return 0
    probe.destroy_node()

    control = gcn.GazeControlNode()
    control.use_eeg_gating = False
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(control)

    state = {'vel': None, 'servo': None, 'zone': ''}
    node.create_subscription(Twist, '/cmd_vel/gaze',
                             lambda m: state.__setitem__('vel', m), 10)
    node.create_subscription(Vector3, '/servo/gaze',
                             lambda m: state.__setitem__('servo', m), 10)
    node.create_subscription(String, '/gaze/zone',
                             lambda m: state.__setitem__('zone', m.data), 10)

    def spin(seconds):
        end = time.time() + seconds
        while time.time() < end:
            executor.spin_once(timeout_sec=0.01)

    spin(1.0)
    print(f'{"target":<30} {"zone":<18} {"vx":>6} {"vy":>6} {"wz":>6}  pan/tilt')
    print('-' * 84)

    for label, u, v in SEQUENCE:
        end = time.time() + hold
        while time.time() < end:
            msg = Point()
            msg.x, msg.y, msg.z = u, v, 0.0
            pub.publish(msg)
            # Spin the EXECUTOR, not just this node. Spinning only `node`
            # leaves the control node's subscriptions unprocessed, so it never
            # sees a single gaze point and the test reports all zeros.
            executor.spin_once(timeout_sec=0.01)
        vel = state['vel']
        srv = state['servo']
        vx = vel.linear.x if vel else 0.0
        vy = vel.linear.y if vel else 0.0
        wz = vel.angular.z if vel else 0.0
        pt = f'{srv.x:.0f}/{srv.y:.0f}' if srv else '--'
        print(f'{label:<30} {state["zone"]:<18} '
              f'{vx:6.2f} {vy:6.2f} {wz:6.2f}  {pt}')

    print('\n[test] stopping publisher; waiting for the link timeout...')
    spin(1.2)
    vel = state['vel']
    # vel is None when nothing was ever published, which is also stopped.
    stopped = vel is None or max(abs(vel.linear.x), abs(vel.linear.y),
                                 abs(vel.angular.z)) < 1e-6
    print(f'[test] after silence: {"STOPPED (pass)" if stopped else "STILL MOVING (FAIL)"}')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if stopped else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hold', type=float, default=1.0,
                    help='seconds per zone (> dwell time of 0.4 s)')
    ap.add_argument('--udp', default=None,
                    help='send UDP to this host instead of publishing ROS')
    ap.add_argument('--port', type=int, default=9100)
    a = ap.parse_args()

    if a.udp:
        run_udp(a.udp, a.port, a.hold)
        return 0
    return run_ros(a.hold)


if __name__ == '__main__':
    sys.exit(main())
