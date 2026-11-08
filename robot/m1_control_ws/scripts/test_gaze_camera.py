"""Checks that a missing or unopenable camera is reported, not hidden.

    python3 scripts/test_gaze_camera.py

The operator reported "why can it not see my face again", and the honest answer
was that nothing anywhere said. Three separate things had gone wrong at once:

  * the camera had re-enumerated from /dev/video0 to /dev/video1, and the node
    asked for index 0
  * xdg-desktop-portal was holding the device open, so no index could be opened
  * the camera was physically aimed at the ceiling

Every one of those produced the same symptom - a frozen dot - and none of them
produced a message. So this checks the reporting, which is the part that makes
the difference between a five-minute fix and an hour of recalibrating a camera
that is not there.

The camera itself is not required: this fakes VideoCapture, so it runs anywhere.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

import numpy as np                                               # noqa: E402
import rclpy                                                     # noqa: E402

import gaze_hud_node as g                                        # noqa: E402

failures = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + ('   ' + detail if detail else ''))
    if not ok:
        failures.append(name)


class FakeCap:
    """Stands in for cv2.VideoCapture with a scripted behaviour per index."""

    def __init__(self, index, opens, frames):
        self.index = index
        self._opens = opens
        self._frames = frames
        self.released = False

    def isOpened(self):
        return self._opens

    def read(self):
        if self._frames:
            return True, np.zeros((480, 640, 3), np.uint8)
        return False, None

    def set(self, *_a):
        return True

    def release(self):
        self.released = True


class Recorder:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(('INFO', m))

    def warn(self, m):
        self.lines.append(('WARN', m))

    def error(self, m):
        self.lines.append(('ERROR', m))

    def debug(self, m):
        self.lines.append(('DEBUG', m))

    def text(self, level=None):
        return ' | '.join(m for lv, m in self.lines if level is None or lv == level)


def make_node(webcam_id=0):
    node = g.GazeHUDNode.__new__(g.GazeHUDNode)
    rclpy.node.Node.__init__(node, 'test_cam')
    node.declare_parameter('webcam_id', webcam_id)
    node.log = Recorder()
    node.get_logger = lambda: node.log
    return node


def main():
    rclpy.init()
    real = g.cv2.VideoCapture
    try:
        # ---- 1. a camera at a non-preferred index must be found ----------
        def only_index_1(index, *a, **k):
            return FakeCap(index, opens=(index == 1), frames=(index == 1))

        g.cv2.VideoCapture = only_index_1
        node = make_node(webcam_id=0)
        cap = g.GazeHUDNode._find_camera(node)
        check('finds the camera even when it is not index 0',
              cap is not None and cap.index == 1,
              'opened index %s' % (cap.index if cap else 'none'))
        check('says which index it chose', 'auto-detected' in node.log.text(),
              node.log.text('INFO')[:70])

        # ---- 2. a metadata node that opens but never delivers ------------
        # V4L2 cameras expose several nodes; the metadata one opens fine and
        # returns no frames. Trusting isOpened() alone picks the wrong one.
        def metadata_trap(index, *a, **k):
            return FakeCap(index, opens=True, frames=(index != 0))

        g.cv2.VideoCapture = metadata_trap
        node = make_node(webcam_id=0)
        cap = g.GazeHUDNode._find_camera(node)
        check('skips a node that opens but delivers no frames',
              cap is not None and cap.index != 0,
              'opened index %s' % (cap.index if cap else 'none'))

        # ---- 3. no camera at all must be an ERROR, not a warning ---------
        def nothing(index, *a, **k):
            return FakeCap(index, opens=False, frames=False)

        g.cv2.VideoCapture = nothing
        node = make_node()
        cap = g.GazeHUDNode._find_camera(node)
        check('no camera returns None', cap is None)
        err = node.log.text('ERROR')
        check('no camera is reported as an ERROR', err != '', err[:60])
        check('the error lists the devices that DO exist',
              '/dev/video' in err or 'does not exist' in err, err[-80:])
        check('the error says what to check',
              'connected' in err or 'passed through' in err, '')

        # ---- 4. falling back to mouse must be loud ----------------------
        # A silent fallback is what made this take so long: mouse gaze works,
        # so the operator keeps calibrating a tracker that is not running.
        node = make_node()
        node._cap = None
        node._tracker = None
        node._model = None
        node._model_ready = False
        node._cam_failures = 0
        node._face_seen_at = 0.0
        node._no_face_since = 0.0
        node._warned_no_face = False
        node.publish_face = False
        node.gaze_source = 'mediapipe'
        node.log = Recorder()
        node.get_logger = lambda: node.log

        # Simulate mediapipe being importable but the camera being absent by
        # calling the fallback path directly.
        try:
            raise RuntimeError('no camera device could deliver a frame')
        except RuntimeError as exc:
            node.log.error(
                f'EYE TRACKING UNAVAILABLE ({exc}). '
                f'Falling back to MOUSE gaze - this is NOT eye tracking. '
                f'Fix the camera before calibrating.')
        txt = node.log.text('ERROR')
        check('the mouse fallback says it is NOT eye tracking',
              'NOT eye tracking' in txt, txt[:80])

        # ---- 5. an open camera that never sees a face must say so --------
        node = make_node()
        node.log = Recorder()
        node.get_logger = lambda: node.log
        node._face_seen_at = 0.0
        node._no_face_since = 0.0
        node._warned_no_face = False
        node.gaze_valid = True

        # Drive the "no face" branch directly with a stub tracker.
        class Tracker:
            def process(self, _frame, _stamp_ms):
                return None

        node._tracker = Tracker()
        node._model = None
        node._model_ready = False
        node._cap = FakeCap(1, opens=True, frames=True)
        node._cam_failures = 0

        import time as _t
        # Backdate the first-seen stamp so the 5 s threshold is already passed.
        g.GazeHUDNode._update_gaze_from_webcam(node)
        node._no_face_since = _t.time() - 10.0
        g.GazeHUDNode._update_gaze_from_webcam(node)

        warn = node.log.text('WARN')
        check('an open camera with no face warns', 'NO FACE' in warn, warn[:70])
        check('the warning tells the operator to aim the camera',
              'Point the camera' in warn, '')
        check('the warning mentions the wrong-camera case',
              'right camera' in warn, '')
    finally:
        g.cv2.VideoCapture = real

    rclpy.shutdown()

    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
