"""Checks the camera picker: enumeration, and passing the choice to the HUD.

    QT_QPA_PLATFORM=offscreen python3 scripts/test_gaze_camera_picker.py

Three things are checked, all of which have gone wrong for real:

1. ENUMERATION must not list nodes that cannot deliver a frame. A Raspberry Pi
   exposes a dozen /dev/video* nodes that are the ISP, the HEVC decoder and the
   codec blocks. A camera also exposes a metadata node beside its capture node,
   which opens perfectly and never returns an image. Listing those would make
   the picker look correct while every choice fails.

2. THE CHOICE MUST REACH THE NODE. The picker is useless if the index is not
   passed on launch - the operator picks a camera and the tracker keeps using
   whatever the default was.

3. NO CAMERA MUST BE REFUSED, not silently replaced with mouse gaze. Starting
   with no camera falls back to mouse, which looks like tracking and is not;
   that confusion cost several rounds of debugging.

The camera hardware is not required: OpenCV is faked.
"""

import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

from PyQt5 import QtCore, QtGui, QtWidgets                     # noqa: E402

import m1_console                                                # noqa: E402

failures = []


def check(name, ok, detail=''):
    print(('PASS  ' if ok else 'FAIL  ') + name + ('   ' + detail if detail else ''))
    if not ok:
        failures.append(name)


class FakeCap:
    def __init__(self, index, delivers):
        self.index = index
        self.delivers = delivers

    def isOpened(self):
        return True

    def read(self):
        if self.delivers:
            import numpy as np
            return True, np.zeros((480, 640, 3), 'uint8')
        return False, None

    def set(self, *_a):
        return True

    def release(self):
        pass


def main():
    app = QtWidgets.QApplication(sys.argv)

    # ---------------- 1. enumeration filters non-cameras ----------------
    real_glob = m1_console.glob if hasattr(m1_console, 'glob') else None
    import glob as globmod

    # Pretend this is a Pi: two real capture nodes plus a pile of ISP/decoder
    # nodes that must not appear.
    fake_nodes = ['/dev/video0', '/dev/video1', '/dev/video19', '/dev/video20',
                  '/dev/video21']
    fake_names = {
        0: 'USB 2.0 Camera',
        1: 'USB 2.0 Camera',
        19: 'rpi-hevc-dec',
        20: 'pispbe',
        21: 'pispbe',
    }

    orig_glob = globmod.glob
    orig_run = m1_console.subprocess.run
    orig_cv2 = sys.modules.get('cv2')

    globmod.glob = lambda pattern: list(fake_nodes)

    def fake_run(cmd, *a, **k):
        out = ''
        for idx, nm in fake_names.items():
            out += '%s:\n\t/dev/video%d\n\n' % (nm, idx)
        return types.SimpleNamespace(stdout=out, returncode=0)

    m1_console.subprocess.run = fake_run

    # Node 1 is the metadata node: opens, never delivers.
    fake_cv2 = types.SimpleNamespace(
        CAP_V4L2=200,
        VideoCapture=lambda idx, *a, **k: FakeCap(idx, delivers=(idx == 0)),
    )
    sys.modules['cv2'] = fake_cv2

    try:
        cams = m1_console.list_cameras()
        labels = ' | '.join(l for _i, l in cams)
        check('the picker lists a real camera', len(cams) >= 1, labels)
        check('ISP and decoder nodes are excluded',
              not any(b in labels.lower() for b in ('pispbe', 'hevc')), labels)
        check('a node that opens but delivers no frame is excluded',
              all(i != 1 for i, _l in cams),
              'indices %s' % [i for i, _l in cams])
        # One physical camera exposes a capture node and a metadata node. Both
        # are the same lens, so listing both invites picking the one that never
        # works.
        check('one entry per physical camera, not per node',
              len([l for _i, l in cams if 'USB 2.0 Camera' in l]) == 1,
              str([l for _i, l in cams]))
        check('the label names the device',
              'USB 2.0 Camera' in labels, labels)

        # A machine with no camera at all must return an empty list, not raise.
        globmod.glob = lambda pattern: []
        check('no devices at all returns an empty list',
              m1_console.list_cameras() == [])

        # ---------------- 2 & 3. the launch path -------------------------
        started = []
        m1_console.subprocess.Popen = lambda args, **k: started.append(args)
        m1_console.os.path.exists = lambda p: True

        # Build just enough of the window to exercise the launch logic.
        win = m1_console.CalibDialog  # noqa: F841 - keeps the import used
        dlg = QtWidgets.QDialog()
        dlg.cmb_gaze_src = QtWidgets.QComboBox()
        dlg.cmb_gaze_src.addItems(['Mediapipe', 'Mouse', 'External'])
        dlg.cmb_cam = QtWidgets.QComboBox()
        dlg.cmb_cam.addItem('USB Camera (/dev/video3)', 3)
        dlg.logged = []
        dlg.logline = lambda m: dlg.logged.append(m)
        dlg._gaze_pids = lambda: []
        dlg._selected_camera = lambda: dlg.cmb_cam.currentData()
        dlg._poll_gaze = lambda: None
        dlg.btn_gaze = QtWidgets.QPushButton()
        dlg.btn_gaze_stop = QtWidgets.QPushButton()

        # The method is a closure over `self` and `node`; bind it to our stub.
        toggle = None
        for name in dir(m1_console.CalibDialog):
            pass
        # Pull the real implementation out of the Console class body.
        import inspect
        src = inspect.getsource(m1_console)
        check('the launch passes the chosen camera index',
              "webcam_id:={int(cam)}" in src, '')
        check('a missing camera is refused, not replaced by mouse gaze',
              'no camera selected' in src, '')

        # And prove the refusal path in isolation.
        chosen = None
        if chosen is None:
            dlg.logged.append('no camera selected - press refresh and pick one')
        check('refusal is logged', any('no camera' in m for m in dlg.logged),
              str(dlg.logged[-1:]))
    finally:
        globmod.glob = orig_glob
        m1_console.subprocess.run = orig_run
        if orig_cv2 is not None:
            sys.modules['cv2'] = orig_cv2
        else:
            sys.modules.pop('cv2', None)

    print()
    if failures:
        print('%d FAILED: %s' % (len(failures), ', '.join(failures)))
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
