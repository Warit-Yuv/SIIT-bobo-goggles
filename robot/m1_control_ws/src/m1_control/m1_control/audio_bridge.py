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
  reimplemented, so device selection is the same string the user already sees in
  `arecord -l`.
* **On a VMware guest, use `--device-in default`, not `plughw:0,0`.** VMware
  forwards the host microphone to the PulseAudio sound server, not to the raw
  ALSA card. Reading the card directly returns a valid stream of pure silence -
  it opens fine and produces nothing, which looks exactly like a broken
  microphone. Measured on this VM:

      arecord -d 2 -f cd /tmp/a.wav          # -> "default", via PulseAudio
      arecord -D plughw:0,0 -d 2 /tmp/b.wav # -> opens, delivers zeros

  On the robot the opposite is true: there is no PulseAudio in the container, so
  `plughw:2,0` (the C-Media USB headset) is correct.

Usage
-----
On the robot (inside the container):
    ros2 run m1_control audio_bridge --ros-args -p role:=robot \\
        -p device_in:=plughw:2,0 -p device_out:=plughw:2,0 \\
        -p peer_ip:=10.0.0.102

On the operator PC:
    ros2 run m1_control audio_bridge --ros-args -p role:=operator \\
        -p device_in:=default -p device_out:=default \\
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
        # Subprocess health, reported so a dead arecord/aplay is visible in the
        # GUI instead of looking like a quiet room.
        self.capture_ok = False
        self.playback_ok = False
        self.capture_restarts = 0
        self.playback_restarts = 0
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
                # Subprocess health. A dead arecord/aplay is the difference
                # between "nobody spoke" and "this end cannot hear anything",
                # and the two look identical from the packet counters alone.
                'cap_ok': self.capture_ok,
                'play_ok': self.playback_ok,
                'cap_restarts': self.capture_restarts,
                'play_restarts': self.playback_restarts,
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
            with self.lock:
                self.capture_ok = self._tx_proc is not None
        if self.device_out:
            self._rx_proc = self._spawn(self._aplay_cmd(), 'playback')
            with self.lock:
                self.playback_ok = self._rx_proc is not None

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
        """Our microphone -> peer.

        Restarts `arecord` if it dies. The first version `break`ed out of the
        loop on EOF, which silently killed the whole uplink for the rest of the
        session: the process stayed alive, the ROS node stayed registered, the
        GUI kept showing a packet counter, and nothing was ever transmitted
        again. Measured on the robot after a few hours: both `arecord` and
        `aplay` were zombies and no UDP port was bound at all.

        An audio device that disappears (USB re-enumeration, a suspend, an
        ALSA underrun) is a normal event, not a reason to give up.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Never block the capture pipe if the network stalls.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)

        while not self.stop_event.is_set():
            proc = self._tx_proc
            if proc is None or proc.poll() is not None:
                if not self._restart_capture():
                    # Device genuinely unavailable - back off rather than spin.
                    self.stop_event.wait(2.0)
                    continue
                proc = self._tx_proc

            try:
                chunk = proc.stdout.read(CHUNK_BYTES)
            except (ValueError, OSError):
                # Pipe closed under us; treat like EOF and restart.
                chunk = b''

            if not chunk:
                # arecord exited. Reap it so it does not linger as a zombie,
                # then let the top of the loop restart it.
                self._reap(self._tx_proc)
                self._tx_proc = None
                continue

            if not (self.enabled and self.mic_enabled):
                continue
            try:
                sock.sendto(chunk, (self.peer_ip, self.send_port))
                with self.lock:
                    self.packets_sent += 1
            except OSError:
                pass
        sock.close()

    def _restart_capture(self):
        """(Re)spawn arecord. Returns True if a live process is now attached."""
        self._reap(self._tx_proc)
        proc = self._spawn(self._arecord_cmd(), 'capture')
        self._tx_proc = proc
        with self.lock:
            self.capture_restarts += 1
            self.capture_ok = proc is not None
        if proc is not None:
            print(f'[audio] capture (re)started: {self.device_in}')
        return proc is not None

    @staticmethod
    def _reap(proc):
        """Collect a dead child so it does not stay a zombie."""
        if proc is None:
            return
        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=0.5)
            else:
                proc.wait(timeout=0.5)
        except (OSError, subprocess.TimeoutExpired):
            pass

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

            # Notice a dead aplay even when nothing is arriving - otherwise
            # the downlink stays broken until the next packet happens to
            # arrive, which may be never if the other side is also down.
            if self._rx_proc is not None and self._rx_proc.poll() is not None:
                self._restart_playback()

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
                    # aplay died (device unplugged, ALSA error). Restart it
                    # rather than abandoning the downlink - see _tx_loop.
                    self._restart_playback()
                    playing = False
            elif playing and not buffer:
                # Underrun: wait for the buffer to refill before resuming.
                playing = False

        sock.close()

    def _restart_playback(self):
        """(Re)spawn aplay. Returns True if a live process is now attached."""
        self._reap(self._rx_proc)
        proc = self._spawn(self._aplay_cmd(), 'playback')
        self._rx_proc = proc
        with self.lock:
            self.playback_restarts += 1
            self.playback_ok = proc is not None
        if proc is not None:
            print(f'[audio] playback (re)started: {self.device_out}')
        return proc is not None


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
                f"live={s['live']} cap_ok={s['cap_ok']} play_ok={s['play_ok']}"
                f" cap_restarts={s['cap_restarts']}"
                f" play_restarts={s['play_restarts']}")))

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
                    help='ALSA capture device. "default" routes through '
                         'PulseAudio, which is what a VMware guest needs - see '
                         'the note below. Use plughw:N,0 only on real hardware.')
    ap.add_argument('--device-out', default=None,
                    help='ALSA playback device. "default" routes through '
                         'PulseAudio.')
    ap.add_argument('--jitter-ms', type=int, default=60)
    ap.add_argument('--enable', action='store_true',
                    help='start unmuted (default: wait for /audio/enable)')
    ap.add_argument('--dry-run', action='store_true',
                    help='no ROS; print transport status')
    ap.add_argument('--no-ros', action='store_true',
                    help='run the relay without ROS at all (needs --enable). '
                         'For machines with a real microphone but no ROS - '
                         'e.g. the Windows host, where the physical mic lives '
                         'and VMware does not forward it.')
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

    if args.no_ros:
        # Escape hatch for the operator side.
        #
        # VMware's virtual sound card captures digital silence unless the host
        # explicitly forwards its microphone, and getting a USB headset into
        # the VM is a click-through in the VMware menus. Neither is a problem
        # with the relay - the physical microphone is on the host. This mode
        # runs the same transport with no ROS so it can live where the
        # hardware actually is.
        #
        # There is no /audio/enable to wait for here, so it starts unmuted.
        if not core.enabled:
            print('[audio] --no-ros implies --enable; starting unmuted')
            core.set_enabled(True)
        print(f'[audio] no-ROS mode: role={args.role} peer={args.peer_ip}')
        print(f'[audio] capture={args.device_in} playback={args.device_out}')
        print(f'[audio] listen=UDP {core.listen_port} send=UDP {core.send_port}')
        core.start()
        try:
            while True:
                s = core.status()
                dev = 'ok' if (s['cap_ok'] and s['play_ok']) else 'DEVICE DOWN'
                print(f"\r[audio] sent={s['sent']} recv={s['recv']} "
                      f"drop={s['dropped']} live={s['live']} {dev}   ",
                      end='', flush=True)
                time.sleep(1.0)
        except KeyboardInterrupt:
            print()
        finally:
            core.stop()
        return 0

    core.start()
    run_ros(args, core)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
