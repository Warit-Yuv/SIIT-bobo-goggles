#!/usr/bin/env python3
"""Raw YDLidar protocol probe - identifies model/baudrate without ROS.

Sends the standard YDLidar command frames and prints the raw reply so the
real model can be identified when a driver profile mismatches.

    python3 lidar_probe.py --port /dev/rplidar
    python3 lidar_probe.py --port /dev/rplidar --baud 115200

Command frames (header 0xA5, checksum = low byte of header+cmd):
    GET_DEVICE_INFO   A5 90
    GET_HEALTH        A5 52
    STOP_SCAN         A5 25
    SCAN              A5 20
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit('pyserial not installed: pip3 install pyserial')

BAUDS = [230400, 115200, 128000, 153600, 256000, 460800, 512000]

CMDS = {
    'GET_DEVICE_INFO': bytes([0xA5, 0x90, 0x35]),
    'GET_HEALTH':      bytes([0xA5, 0x52, 0xF7]),
    'STOP_SCAN':       bytes([0xA5, 0x25, 0xCA]),
}


def probe(port, baud, cmd_name, wait=1.2):
    """Send one command, return (bytes_read, decoded_summary)."""
    try:
        ser = serial.Serial(port, baud, timeout=0.15, write_timeout=0.5)
    except serial.SerialException as exc:
        return None, f'open failed: {exc}'

    try:
        ser.reset_input_buffer()
        ser.write(CMDS[cmd_name])
        ser.flush()
        time.sleep(wait)
        data = ser.read(64)
    finally:
        ser.close()

    if not data:
        return b'', 'no reply'

    head = data[0]
    summary = f'{len(data)}B head=0x{head:02X}'
    # Device info reply: A5 5A <type> <model:1> <fw_major> <fw_minor> <hw:1> <serial:16> <crc:2>
    if cmd_name == 'GET_DEVICE_INFO' and len(data) >= 8 and head == 0xA5:
        summary += (f' | model=0x{data[2]:02X} fw={data[3]}.{data[4]}'
                    f' hw=0x{data[5]:02X}')
    if cmd_name == 'GET_HEALTH' and len(data) >= 3 and head == 0xA5:
        summary += f' | status=0x{data[2]:02X}'
    return data, summary


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/rplidar')
    ap.add_argument('--baud', type=int, default=None,
                    help='test only this baudrate')
    a = ap.parse_args()

    bauds = [a.baud] if a.baud else BAUDS
    best = None

    print(f'probe {a.port}\n' + '-' * 62)
    for baud in bauds:
        print(f'\nbaud {baud}')
        alive = False
        for cmd_name in ('GET_DEVICE_INFO', 'GET_HEALTH'):
            data, summary = probe(a.port, baud, cmd_name)
            mark = 'OK ' if data else '-- '
            print(f'  [{mark}] {cmd_name:<16} {summary}')
            if data:
                alive = True
        if alive and best is None:
            best = baud

    print('\n' + '-' * 62)
    if best:
        print(f'RESPONSIVE baudrate: {best}')
        print('-> set this as `baudrate:` in the driver params file')
    else:
        print('No response at any baudrate.')
        print('-> check wiring, that the LiDAR motor spins, and permissions')
        print('-> the LiDAR may need a moment after power-up; retry once')
    return 0 if best else 1


if __name__ == '__main__':
    raise SystemExit(main())
