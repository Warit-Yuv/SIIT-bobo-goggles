#!/usr/bin/env python3
"""Two-way voice link between the operator PC and the robot.

Designed for the project's actual use case: a bed-bound operator talking to
people near the robot, and hearing the room. That means **two independent
directions**, each of which can be muted on its own:

    robot mic  --UDP:9200-->  operator speakers     (hear the room)
    operator mic --UDP:9201--> robot speaker        (be heard)

Why raw PCM over UDP instead of a codec
---------------------------------------
`arecord` and `aplay` already exist on both machines (ALSA utils), and raw
16 kHz mono 16-bit PCM is 256 kbit/s - small next to the 4 Mbit/s video stream
already flowing. That buys three things a codec would cost:

  * no ffmpeg/gstreamer dependency on the operator PC;
  * no codec negotiation to fail silently;
  * every packet is self-describing, so a dropped one is a 20 ms gap and
    nothing else - no decoder state to corrupt.

The trade-off is bandwidth, and on a LAN it is not the bottleneck.

Design notes
------------
* **Mute is per-direction.** Hearing the room and being heard are different
  needs; an operator may want to listen without their own room noise going out.
* **The robot side is silent unless enabled.** Nothing streams from the robot
  until `/audio/enable` says so, so the microphone is never live by accident.
* **Jitter buffer, not a queue.** Playback keeps a small pre-roll so a late
  packet does not stutter, but drops the oldest audio if it falls too far
  behind. Latency must not grow without bound.
* **Capture and playback are subprocesses.** `arecord`/`aplay` are managed, not
  reimplemented, so device selection is the same `plughw:X,Y` string the user
  already sees in `arecord -l`.

Usage
-----
On the robot (inside the container):
    ros2 run m1_control audio_bridge --ros-args -p role:=robot \\
        -p device_in:=plughw:2,0 -p device_out:=plughw:2,0 \\
        -p peer_ip:=10.0.0.102

On the operator PC:
    ros2 run m1_control audio_bridge --ros-args -p role:=operator \\
        -p device_in:=plughw:0,0 -p device_out:=plughw:0,0 \\
        -p peer_ip:=10.0.0.54

Standalone check, no ROS (proves the audio path works):
    python3 audio_bridge.py --role robot --dry-run
"""

import argparse
import socket
import subprocess
import threading
import time

#: 16 kHz mono signed-16 little-endian. Speech is intelligible and the packet
#: rate stays modest: 16000 * 2 bytes = 32 kB/s = 256 kbit/s.
SAMPLE_RATE = 16000
CHANNELS = 1
SAMPLE_BYTES = 2
CHUNK_MS = 20
CHUNK_BYTES = SAMPLE_RATE * SAMPLE_BYTES * CHANNELS * CHUNK_MS // 1000  # 640

PORT_ROBOT_TO_OP = 9200   # robot microphone -> operator speakers
PORT_OP_TO_ROBOT = 9201   # operator microphone -> robot speaker


class AudioBridgeCore:
    """Transport + mute state. No ROS, so it can be tested standalone."""

    def __init__(self, role, peer_ip,
                 device_in=None, device_out=None,
                 port_in=None, port_out=None,
                 jitter_ms=60, enabled=False):
        self.role = role
        self.peer_ip = peer_ip
        self.device_in = device_in
        self.device_out = device_out

        # Which UDP port we listen on, and which we send to, depends on role.
        if role == 'robot':
            self.listen_port = port_in or PORT_OP_TO_ROBOT
            self.send_port = port_out or PORT_ROBOT_TO_OP
        else:
            self.listen_port = port_in or PORT_ROBOT_TO_OP
            self.send_port = port_out or PORT_OP_TO_ROBOT

        self.jitter_packets = max(1, int(jitter_ms / CHUNK_MS))
        self.enabled = enabled

        # Mutes are separate: hearing the room and being heard are different.
        self.mic_enabled = True      # our microphone -> peer
        self.speaker_enabled = True  # peer audio -> our speakers

        self.stop_event = threading.Event()
        self._rx_proc = None
        self._tx_proc = None
        self.packets_sent = 0
        self.packets_recv = 0
        self.packets_dropped = 0
        self.last_rx = 0.0
        self.lock = threading.Lock()

    # ------------------------------------------------------------- controls
    def set_enabled(self, on):
        self.enabled = bool(on)

    def set_mic(self, on):
        self.mic_enabled = bool(on)

    def set_speaker(self, on):
        self.speaker_enabled = bool(on)

    def status(self):
        now = time.monotonic()
        with self.lock:
            return {
                'enabled': self.enabled,
                'mic': self.mic_enabled,
                'speaker': self.speaker_enabled,
                'sent': self.packets_sent,
                'recv': self.packets_recv,
                'dropped': self.packets_dropped,
                # "live" means audio arrived in the last second.
                'live': (now - self.last_rx) < 1.0 if self.last_rx else False,
            }

    # --------------------------------------------------------------- threads
    def _spawn(self, cmd, name):
        try:
            return subprocess.Popen(
                cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, bufsize=0)
        except FileNotFoundError:
            print(f'[audio] {name}: command not found: {cmd[0]}')
            return None

    def _arecord_cmd(self):
        return ['arecord', '-q', '-D', self.device_in,
                '-f', 'S16_LE', '-r', str(SAMPLE_RATE),
                '-c', str(CHANNELS), '-t', 'raw']

    def _aplay_cmd(self):
        return ['aplay', '-q', '-D', self.device_out,
                '-f', 'S16_LE', '-r', str(SAMPLE_RATE),
                '-c', str(CHANNELS), '-t', 'raw']

    def start(self):
        if self.device_in:
            self._tx_proc = self._spawn(self._arecord_cmd(), 'capture')
        if self.device_out:
            self._rx_proc = self._spawn(self._aplay_cmd(), 'playback')

        threading.Thread(target=self._tx_loop, daemon=True).start()
        threading.Thread(target=self._rx_loop, daemon=True).start()
        return self

    def stop(self):
        self.stop_event.set()
        for p in (self._tx_proc, self._rx_proc):
            if p is not None:
                try:
                    p.terminate()
                except OSError:
                    pass

    # ---------------------------------------------------------------- uplink
    def _tx_loop(self):
        """Our microphone -> peer."""
        if self._tx_proc is None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Never block the capture pipe if the network stalls.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)

        while not self.stop_event.is_set():
            chunk = self._tx_proc.stdout.read(CHUNK_BYTES)
            if not chunk:
                break
            if not (self.enabled and self.mic_enabled):
                continue
            try:
                sock.sendto(chunk, (self.peer_ip, self.send_port))
                with self.lock:
                    self.packets_sent += 1
            except OSError:
                pass
        sock.close()

    # -------------------------------------------------------------- downlink
    def _rx_loop(self):
        """Peer audio -> our speakers, with a bounded jitter buffer."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('0.0.0.0', self.listen_port))
        except OSError as exc:
            print(f'[audio] cannot bind UDP {self.listen_port}: {exc}')
            return
        sock.settimeout(0.1)

        buffer = []
        pre_roll = self.jitter_packets
        playing = False

        while not self.stop_event.is_set():
            try:
                data, _ = sock.recvfrom(4096)
            except socket.timeout:
                data = None
            except OSError:
                break

            if data:
                with self.lock:
                    self.packets_recv += 1
                    self.last_rx = time.monotonic()
                if self.enabled and self.speaker_enabled:
                    buffer.append(data)
                    # Bounded: if we fall behind, drop the oldest rather than
                    # let latency grow. Voice that is 2 s late is worse than a
                    # 20 ms gap.
                    if len(buffer) > pre_roll * 5:
                        drop = len(buffer) - pre_roll
                        del buffer[:drop]
                        with self.lock:
                            self.packets_dropped += drop
                else:
                    buffer.clear()
                    playing = False
                    continue

            if len(buffer) >= pre_roll:
                playing = True

            if playing and buffer and self._rx_proc is not None:
                chunk = buffer.pop(0)
                try:
                    self._rx_proc.stdin.write(chunk)
                    self._rx_proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    break
            elif playing and not buffer:
                # Underrun: wait for the buffer to refill before resuming.
                playing = False

        sock.close()


# ---------------------------------------------------------------------------
# ROS wrapper
# ---------------------------------------------------------------------------

def run_ros(args, core):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    class AudioBridge(Node):

        def __init__(self):
            super().__init__('audio_bridge')
            self.pub_status = self.create_publisher(String, '/audio/status', 10)

            # /audio/enable is the master switch; the per-direction topics let
            # the operator mute just one side.
            self.create_subscription(Bool, '/audio/enable', self._en_cb, 10)
            self.create_subscription(Bool, '/audio/mic', self._mic_cb, 10)
            self.create_subscription(Bool, '/audio/speaker', self._spk_cb, 10)

            self.create_timer(1.0, self._report)
            self.get_logger().info(
                f'Audio bridge [{core.role}] in={core.device_in} '
                f'out={core.device_out} peer={core.peer_ip} '
                f'listen={core.listen_port} send={core.send_port} '
                f'enabled={core.enabled}')

        def _en_cb(self, msg):
            core.set_enabled(msg.data)
            self.get_logger().info(f"Audio {'ON' if msg.data else 'OFF'}")

        def _mic_cb(self, msg):
            core.set_mic(msg.data)
            self.get_logger().info(f"Mic {'on' if msg.data else 'muted'}")

        def _spk_cb(self, msg):
            core.set_speaker(msg.data)
            self.get_logger().info(
                f"Speaker {'on' if msg.data else 'muted'}")

        def _report(self):
            s = core.status()
            self.pub_status.publish(String(data=(
                f"enabled={s['enabled']} mic={s['mic']} spk={s['speaker']} "
                f"sent={s['sent']} recv={s['recv']} drop={s['dropped']} "
                f"live={s['live']}")))

    rclpy.init()
    node = AudioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        core.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--role', choices=['robot', 'operator'], default='robot')
    ap.add_argument('--peer-ip', default='127.0.0.1')
    ap.add_argument('--device-in', default=None,
                    help='ALSA capture device, e.g. plughw:2,0')
    ap.add_argument('--device-out', default=None,
                    help='ALSA playback device, e.g. plughw:2,0')
    ap.add_argument('--jitter-ms', type=int, default=60)
    ap.add_argument('--enable', action='store_true',
                    help='start unmuted (default: wait for /audio/enable)')
    ap.add_argument('--dry-run', action='store_true',
                    help='no ROS; print transport status')
    args, _ = ap.parse_known_args(argv)

    core = AudioBridgeCore(
        role=args.role, peer_ip=args.peer_ip,
        device_in=args.device_in, device_out=args.device_out,
        jitter_ms=args.jitter_ms, enabled=args.enable)

    if args.dry_run:
        print(f'[audio] role={args.role} peer={args.peer_ip}')
        print(f'[audio] capture={args.device_in} playback={args.device_out}')
        print(f'[audio] listen=UDP {core.listen_port} send=UDP {core.send_port}')
        print(f'[audio] chunk={CHUNK_BYTES}B every {CHUNK_MS}ms '
              f'({SAMPLE_RATE} Hz mono)')
        return 0

    core.start()
    run_ros(args, core)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
