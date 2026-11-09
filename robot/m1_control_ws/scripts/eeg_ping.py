#!/usr/bin/env python3
"""Send one protocol-v1 packet to the bridge and read back the ROS state.

This is the "is the EEG actually connected?" check that does not need a
headset. It answers three questions at once:

  * is anything listening on the UDP port
  * does the bridge accept the packet version I am sending
  * did /eeg/run and /eeg/mode actually change

Run from the robot or anywhere that can reach it:
    python3 scripts/eeg_ping.py                    # localhost
    python3 scripts/eeg_ping.py --host 10.0.0.54   # the robot
    python3 scripts/eeg_ping.py --run on           # leave run ON
    python3 scripts/eeg_ping.py --watch            # print state for 10 s
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time

PROTOCOL_VERSION = 1


def send(host, port, seq, state=None, gesture=None, timeout=1.5):
    """Send one packet. Returns the reply dict, or None."""
    msg = {'v': PROTOCOL_VERSION, 'seq': seq, 'ts': time.time()}
    if state is not None:
        msg['state'] = state
    if gesture is not None:
        msg['event'] = {'gesture': gesture}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(json.dumps(msg).encode(), (host, port))
        data, _addr = sock.recvfrom(4096)
        return json.loads(data.decode())
    except socket.timeout:
        return None
    except (OSError, ValueError) as exc:
        print(f'  send failed: {exc}')
        return None
    finally:
        sock.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=9000)
    ap.add_argument('--run', choices=('on', 'off', 'toggle'), default=None)
    ap.add_argument('--mode', choices=('robot', 'camera'), default=None)
    ap.add_argument('--gesture', default=None,
                    help='JAW_CLENCH / DOUBLE_BLINK / EYEBROW_RAISE')
    ap.add_argument('--watch', action='store_true',
                    help='keep heartbeating for 10 s so the link stays up')
    ap.add_argument('--seq', type=int, default=1_000_000,
                    help='starting sequence number (default high on purpose, '
                         'so the bridge cannot mistake it for a stale packet)')
    args = ap.parse_args()

    print(f'[eeg] target {args.host}:{args.port}  protocol v{PROTOCOL_VERSION}')

    state = {}
    if args.run == 'on':
        state['run'] = True
    elif args.run == 'off':
        state['run'] = False
    if args.mode:
        state['mode'] = args.mode

    # Start at a HIGH sequence number, not 1.
    #
    # The bridge ignores any seq lower than one it has already seen, because a
    # lower seq normally means a late or duplicated datagram. A real sender has
    # usually been running for a while, so seq 1 looks stale and is dropped -
    # the bridge reports `dropped: 1` and nothing happens. Starting high means
    # this tool can never be mistaken for a stale packet.
    base = args.seq
    reply = send(args.host, args.port, base, state=state or None,
                 gesture=args.gesture)
    if reply is None:
        print('[eeg] NO REPLY')
        print('      Nothing is listening, or a firewall is dropping it.')
        print('      On the robot:  ss -ulnp | grep 9000')
        print('      In the container: ros2 run m1_control eeg_udp_bridge')
        return 1

    print(f'[eeg] reply: {json.dumps(reply)}')
    # `link` lives INSIDE the state block - the bridge reports what the sender
    # should believe, and the link flag is part of that.
    st = reply.get('state', reply)
    print(f'[eeg] run={st.get("run")}  mode={st.get("mode")}  '
          f'music={st.get("music")}  link={st.get("link")}  '
          f'dropped={st.get("dropped")}')

    if args.watch:
        print('[eeg] heartbeating for 10 s - Ctrl-C to stop')
        seq = base + 1
        end = time.time() + 10
        try:
            while time.time() < end:
                send(args.host, args.port, seq, state=state or None)
                seq += 1
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass

    print('\n[eeg] what to check next, on the robot:')
    print('      ros2 topic echo /eeg/run  --once')
    print('      ros2 topic echo /eeg/mode --once')
    print('      ros2 topic echo /eeg/link --once')
    return 0


if __name__ == '__main__':
    sys.exit(main())
