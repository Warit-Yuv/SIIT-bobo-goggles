#!/usr/bin/env python3
"""Simulate the EEG laptop so the robot side can be tested without a headset.

Sends the same UDP/JSON packets that `muse_robot` sends from Windows:

    {"type": "heartbeat"}
    {"command": "GO", "gesture": "DOUBLE_BLINK"}

Usage (from the container, while `start_robot.sh` is running):

    python3 /root/m1_control_ws/scripts/test_eeg_udp.py --ip 127.0.0.1
    python3 .../test_eeg_udp.py --ip 127.0.0.1 --cmd GO --hold 3
    python3 .../test_eeg_udp.py --ip 127.0.0.1 --sequence

`--sequence` runs GO -> LEFT -> RIGHT -> STOP with heartbeats in between, which
exercises the majority filter, the turn pulse and the STOP path.
"""

import argparse
import json
import socket
import time

GESTURE = {
    'GO': 'DOUBLE_BLINK',
    'STOP': 'JAW_CLENCH',
    'LEFT': 'LOOK_LEFT',
    'RIGHT': 'LOOK_RIGHT',
}


class Sender:

    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)

    def _send(self, payload, expect_ack=True):
        self.sock.sendto(json.dumps(payload).encode(), self.addr)
        if not expect_ack:
            return None
        try:
            data, _ = self.sock.recvfrom(2048)
            return json.loads(data.decode())
        except (socket.timeout, ValueError):
            return None

    def heartbeat(self):
        return self._send({'type': 'heartbeat'})

    def command(self, cmd):
        return self._send({'command': cmd, 'gesture': GESTURE.get(cmd, '')})


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ip', default='127.0.0.1', help='robot IP')
    ap.add_argument('--port', type=int, default=9000)
    ap.add_argument('--cmd', default=None, help='single command to send')
    ap.add_argument('--hold', type=float, default=0.0,
                    help='seconds to keep heartbeating after the command')
    ap.add_argument('--sequence', action='store_true')
    a = ap.parse_args()

    s = Sender(a.ip, a.port)

    print(f'[test] target {a.ip}:{a.port}')
    ack = s.heartbeat()
    if ack is None:
        print('[test] NO REPLY - is the robot bridge running and the port open?')
        return 1
    print(f'[test] robot replied: {ack}')

    if a.sequence:
        plan = [('GO', 2.0), ('LEFT', 1.5), ('RIGHT', 1.5), ('STOP', 1.0)]
        for cmd, hold in plan:
            print(f'[test] -> {cmd}')
            s.command(cmd)
            end = time.time() + hold
            while time.time() < end:
                s.heartbeat()
                time.sleep(0.5)
        print('[test] sequence complete')
        return 0

    if a.cmd:
        cmd = a.cmd.upper()
        # Send three times: the majority filter needs a few samples.
        for _ in range(3):
            ack = s.command(cmd)
            print(f'[test] -> {cmd}  ack={ack}')
            time.sleep(0.1)
        end = time.time() + a.hold
        while time.time() < end:
            s.heartbeat()
            time.sleep(0.5)
        print('[test] done')
        return 0

    print('[test] heartbeat only (Ctrl+C to stop)')
    try:
        while True:
            print(f'[test] ack={s.heartbeat()}')
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
