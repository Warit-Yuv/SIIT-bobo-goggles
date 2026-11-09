#!/usr/bin/env python3
"""Play a random sound from a folder when the EEG eyebrow-raise fires.

    /eeg/music  ->  sound_player_node  ->  ffplay  ->  robot speaker

WHY THIS IS ITS OWN NODE
------------------------
The EEG classifier on this headset confuses jaw clench with eyebrow raise often
enough that the classifier itself reports UNCLEAR_MUSCLE and refuses to guess.
A gesture that unreliable cannot be trusted to move a robot, so it gets the one
job where a misfire costs nothing: making a noise.

It also means the sound is an INDEPENDENT output. A misfire plays the wrong
sound and nothing else happens - the robot does not move, the camera does not
move, and no state is corrupted.

TRIGGERING
----------
The protocol sends a toggle, so `music` flips on every raise. Reacting to the
LEVEL would mean the first raise plays and the second one silently stops - the
operator would have to raise an eyebrow twice per sound. This reacts to the
EDGE instead, so every raise plays exactly one sound, and raising again while
it is still playing replaces it.

The level is still published on /eeg/music for anything that wants it.

Run on the robot:
    ros2 run m1_control sound_player_node
    ros2 run m1_control sound_player_node --ros-args -p sound_dir:=/home/pi/sound
"""

from __future__ import annotations

import glob
import os
import random
import shutil
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

#: Extensions the player is known to handle. Deliberately short: a folder full
#: of odd formats that half-play is worse than a folder of mp3s that all play.
AUDIO_EXT = ('.mp3', '.wav', '.ogg', '.m4a', '.flac', '.opus')


class SoundPlayerNode(Node):
    """One random sound per eyebrow raise."""

    def __init__(self):
        super().__init__('sound_player_node')

        # Inside the robot container the host's /home/pi is NOT visible - the
        # container has its own filesystem, and only the folders named in
        # m1_docker.sh are mounted. /root/sound is that mount. The other two
        # entries let the same node run outside the container (during a
        # `--dry-run`, or on a plain install) without a different parameter.
        self.declare_parameter('sound_dir', '')
        # ffplay is present on this image and needs no service to be running,
        # unlike PulseAudio. It also exits cleanly on SIGTERM, which matters
        # because a stuck ffplay holds the ALSA device and the next sound is
        # silent with no error anywhere.
        self.declare_parameter('player', 'ffplay')
        self.declare_parameter('volume', 80)        # ffplay uses 0..100
        self.declare_parameter('topic', '/eeg/music')
        self.declare_parameter('enabled', True)

        self.sound_dir = self._resolve_dir(
            str(self.get_parameter('sound_dir').value))
        self.player = str(self.get_parameter('player').value)
        self.volume = int(self.get_parameter('volume').value)
        self.enabled = bool(self.get_parameter('enabled').value)

        self._proc = None
        self._last_state = None
        self._played = 0

        if shutil.which(self.player) is None:
            self.get_logger().error(
                f'"{self.player}" is not installed - no sound will play. '
                f'Install it, or set the "player" parameter to something that '
                f'exists (ffplay, cvlc, paplay).')

        files = self._scan()
        self.get_logger().info(f'sound folder: {self.sound_dir}')
        if files:
            self.get_logger().info(
                f'{len(files)} sound(s) in {self.sound_dir}: '
                + ', '.join(os.path.basename(f) for f in files[:5])
                + ('...' if len(files) > 5 else ''))
        else:
            self.get_logger().warn(
                f'No audio files in {self.sound_dir}. Eyebrow raises will be '
                f'silent. Put .mp3/.wav files there, or set sound_dir.')

        self.create_subscription(
            Bool, str(self.get_parameter('topic').value),
            self._music_cb, 10)
        # Check on the player every second: a crashed ffplay must be noticed,
        # or the next raise finds the ALSA device held and fails silently.
        self.create_timer(1.0, self._reap)

    # ------------------------------------------------------------------ files
    #: Where to look, in order, when `sound_dir` is left empty.
    #:
    #: A list rather than one path because the SAME node runs in two places: the
    #: container (where the host's /home/pi is not mounted) and a plain install
    #: (where it is). Picking the first that exists is not a guess - each entry
    #: is a real deployment, and the choice is logged.
    CANDIDATE_DIRS = ('/root/sound', '/home/pi/sound',
                      os.path.expanduser('~/sound'))

    def _resolve_dir(self, requested):
        if requested:
            return requested
        for path in self.CANDIDATE_DIRS:
            if os.path.isdir(path):
                return path
        return self.CANDIDATE_DIRS[0]

    def _scan(self):
        """Every playable file in the folder, sorted for reproducibility."""
        try:
            names = os.listdir(self.sound_dir)
        except OSError as exc:
            self.get_logger().warn(f'Cannot read {self.sound_dir}: {exc}')
            return []
        out = []
        for name in names:
            if name.startswith('.'):
                continue
            if os.path.splitext(name)[1].lower() in AUDIO_EXT:
                path = os.path.join(self.sound_dir, name)
                if os.path.isfile(path):
                    out.append(path)
        return sorted(out)

    # ------------------------------------------------------------------ input
    def _music_cb(self, msg: Bool):
        """Play on the EDGE, not the level.

        `music` is a toggle, so it flips on every raise. If this played on
        `True` only, the operator would need two raises per sound - one to arm
        it and one to actually hear something. Reacting to a change means every
        raise makes a noise, which is what "raise an eyebrow, get a sound"
        means.
        """
        state = bool(msg.data)
        if state == self._last_state:
            return
        first = self._last_state is None
        self._last_state = state
        if first:
            # The latched topic replays its value on connect. That is not a
            # gesture, and playing a sound because the node just started would
            # be a surprise noise from an empty room.
            self.get_logger().info(
                f'music state is {"on" if state else "off"} (initial)')
            return
        self.get_logger().info(
            f'eyebrow raise -> {"on" if state else "off"}')
        self.play()

    # ---------------------------------------------------------------- playing
    def play(self):
        if not self.enabled:
            return
        files = self._scan()
        if not files:
            self.get_logger().warn('nothing to play')
            return
        path = random.choice(files)

        # Replace rather than overlap. Two ffplay processes fight over the ALSA
        # device and the result is usually silence, which looks like a bug in
        # the gesture rather than in the mixer.
        self._kill()
        cmd = [self.player, '-nodisp', '-autoexit', '-loglevel', 'error',
               '-volume', str(self.volume), path]
        try:
            self._proc = subprocess.Popen(
                cmd, stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            self._played += 1
            self.get_logger().info(
                f'playing {os.path.basename(path)} (#{self._played})')
        except OSError as exc:
            self.get_logger().error(f'could not start {self.player}: {exc}')

    def _kill(self):
        proc, self._proc = self._proc, None
        if proc is None:
            return
        try:
            proc.terminate()
            proc.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            proc.kill()
        except Exception:                                    # noqa: BLE001
            pass

    def _reap(self):
        """Notice a player that died, so the next sound is not silent."""
        proc = self._proc
        if proc is None:
            return
        code = proc.poll()
        if code is None:
            return
        self._proc = None
        if code not in (0, -15, -9):
            err = b''
            try:
                err = proc.stderr.read() or b''
            except Exception:                                # noqa: BLE001
                pass
            self.get_logger().warn(
                f'{self.player} exited with {code}'
                + (f': {err.decode(errors="replace").strip()[:200]}'
                   if err else ''))

    def destroy_node(self):
        self._kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SoundPlayerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
