#!/usr/bin/env python3
"""Does a gesture reach the console's node, and does the pad light for it?

Two separate links, and either can break silently:

    /eeg/gesture  ->  ConsoleNode._eeg_gesture_cb  ->  state  ->  pad.set_gesture

The pad drawing is already covered by test_gesture_pad.py. This covers the
first link, which needs real ROS and a real message - the part that cannot be
checked by reading source.

Run on the operator PC, with the robot stack up:
    python3 scripts/test_eeg_live.py
    python3 scripts/test_eeg_live.py --host 10.0.0.54
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))

import rclpy                                                    # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    # The bridge runs on the ROBOT, so the robot is the sensible default.
    # 127.0.0.1 would only be right when this is run on the Pi itself.
    ap.add_argument('--host', default='10.0.0.54',
                    help='where the EEG bridge listens (the robot)')
    ap.add_argument('--port', type=int, default=9000)
    args = ap.parse_args()

    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        detail = str(detail).encode('ascii', 'replace').decode('ascii')
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    rclpy.init()
    from m1_console import ConsoleNode                          # noqa: PLC0415

    node = ConsoleNode()
    print('node up')

    # Spin briefly so subscriptions are established and the latched topics
    # replay. Without this the first gesture races the connection.
    end = time.time() + 1.5
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    def spin(seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def send(gesture, seq):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.5)
        payload = {'v': 1, 'seq': seq, 'ts': time.time(),
                   'event': {'gesture': gesture}}
        try:
            sock.sendto(json.dumps(payload).encode(),
                        (args.host, args.port))
            sock.recvfrom(2048)
        except (socket.timeout, OSError):
            return False
        finally:
            sock.close()
        return True

    base = 2_000_000
    print('\nthe subscription:')
    if not send('JAW_CLENCH', base):
        # SKIP, not FAIL. This test needs the robot stack running, which is an
        # external precondition - and a suite that goes red whenever the robot
        # is switched off is a suite people stop reading. The distinction
        # matters: "robot not running" is not the same as "the EEG is broken".
        print(f'  [SKIP] nothing answered on {args.host}:{args.port}')
        print('         the robot stack must be up for this test:')
        print('           ssh pi@10.0.0.54 "cd /home/pi && ./m1.sh status"')
        node.destroy_node()
        rclpy.try_shutdown()
        return 0
    check('the EEG bridge is reachable', True)

    spin(1.0)
    got = node.state.get('eeg_gesture')
    check('the gesture reaches the node', got is not None,
          f'state[eeg_gesture] = {got!r}')
    check('it is the gesture that was sent',
          got and got.startswith('JAW_CLENCH'), str(got))
    stamp = node.state.get('eeg_gesture_at') or 0.0
    check('the arrival time was recorded', stamp > 0.0,
          f'{time.monotonic() - stamp:.2f}s ago' if stamp else 'missing')
    check('the timestamp is recent',
          stamp and (time.monotonic() - stamp) < 5.0,
          'a stale stamp would never light the pad')

    print('\na second, different gesture:')
    send('DOUBLE_BLINK', base + 1)
    spin(1.0)
    got2 = node.state.get('eeg_gesture')
    check('the state updates to the new gesture',
          got2 and got2.startswith('DOUBLE_BLINK'), str(got2))
    check('the timestamp moves forward',
          (node.state.get('eeg_gesture_at') or 0.0) > stamp)

    print('\nthe pad consumes it:')
    # Exactly what _refresh_gesture does, so the split/upper logic is checked
    # against a REAL payload rather than a hand-written string.
    text = node.state.get('eeg_gesture') or ''
    at = node.state.get('eeg_gesture_at') or 0.0
    gesture = text.split('/')[0].strip().upper()
    check('the payload reduces to a key name',
          gesture == 'DOUBLE_BLINK', gesture)

    from PyQt5 import QtCore, QtGui, QtWidgets                  # noqa: PLC0415
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    import m1_console                                            # noqa: PLC0415
    GesturePad = m1_console.make_gesture_pad(QtCore, QtGui, QtWidgets)
    pad = GesturePad()
    pad.set_gesture(gesture, at)
    check('the pad lights that key', pad._lit_now() == 'DOUBLE_BLINK',
          str(pad._lit_now()))

    print('\nthe log line:')
    # The console queues a log line for every gesture. If that queue is never
    # drained the list grows without bound, which is a slow leak rather than a
    # visible bug - so check something actually queued it.
    check('a log line was queued', len(node._log_queue) > 0,
          f'{len(node._log_queue)} pending')

    node.destroy_node()
    rclpy.try_shutdown()

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
