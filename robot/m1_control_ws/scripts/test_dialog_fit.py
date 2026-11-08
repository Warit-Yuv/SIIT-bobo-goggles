#!/usr/bin/env python3
"""Exercise CalibDialog's model-fitting path without a camera or a face.

The dialog is the one piece that cannot be tested by importing it: it needs a
Qt application and a real 80-dot run to reach the fit. This drives it with
synthetic data instead, so a break in the fitting, saving or reporting path is
found here rather than three minutes into a calibration the operator has to
sit through.

Run on the operator PC:
    QT_QPA_PLATFORM=offscreen python3 scripts/test_dialog_fit.py
"""

from __future__ import annotations

import math
import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))

import numpy as np                                          # noqa: E402


def main():
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    from PyQt5 import QtCore, QtWidgets                    # noqa: PLC0415
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    import m1_console                                        # noqa: PLC0415

    # ---- the profile is registered and shaped as the dialog expects -------
    print('profile registry:')
    prof = m1_console.CalibDialog.PROFILES
    check('posture profile exists', 'posture' in prof)
    p = prof['posture']
    check('posture profile has 80 dots', len(p['dots']) == 80, str(len(p['dots'])))
    check('posture profile has 4 blocks', len(p['blocks']) == 4,
          str(len(p['blocks'])))
    check('posture profile asks for the model path', p.get('model') is True)
    check('blocks tile the dot list exactly',
          [b['start'] for b in p['blocks']] == [0, 20, 40, 60],
          str([b['start'] for b in p['blocks']]))
    check('every dot has a name, u, v and text',
          all(len(d) == 4 and 0.0 <= d[1] <= 1.0 and 0.0 <= d[2] <= 1.0
              for d in p['dots']))
    check('every dot stays inside the dot area',
          all(0.04 <= d[1] <= 0.96 and 0.04 <= d[2] <= 0.96 for d in p['dots']))
    check('the four blocks use different dot positions',
          len({tuple(round(c, 3) for c in d[1:3])
               for d in p['dots']}) == 80,
          'all 80 unique')

    # ---- a fake node, just enough for the dialog --------------------------
    class FakeNode:
        deadzone_ratio = 0.30
        deadzone_ratio_y = 0.22

        def latest_raw(self):
            return (0.5, 0.5, True, 0.0)

        def features(self, max_age=0.5):
            return [0.0] * 10

    # ---- build the dialog -------------------------------------------------
    area = QtCore.QRect(0, 0, 1920, 1080)
    Cls = m1_console.CalibDialog.build(QtCore, QtWidgets.QtGui if False else
                                       __import__('PyQt5.QtGui',
                                                  fromlist=['QtGui']),
                                       QtWidgets)
    dlg = Cls(FakeNode(), area)
    dlg.profile = 'posture'
    dlg.captured = []

    # ---- synthesise 80 dots' worth of samples -----------------------------
    # Same shape the dialog produces: (features, u, v) per dot.
    rng = random.Random(11)
    for name, u, v, _text in p['dots']:
        feats = [u * 0.2 - 0.1 + rng.gauss(0, 0.004),
                 v * 0.2 - 0.1 + rng.gauss(0, 0.004),
                 0.4 + rng.gauss(0, 0.01),
                 u * 0.19 - 0.095 + rng.gauss(0, 0.004),
                 v * 0.19 - 0.095 + rng.gauss(0, 0.004),
                 rng.gauss(0, 0.02), rng.gauss(0, 0.02),
                 rng.gauss(0, 0.5), rng.gauss(0, 0.5),
                 55.0 + rng.gauss(0, 1.0)]
        dlg.captured.append((feats, u, v))

    # ---- redirect the save path so the test cannot clobber a real one -----
    tmp = tempfile.mkdtemp()
    target = os.path.join(tmp, 'gaze_calibration.json')
    real_expanduser = os.path.expanduser
    os.path.expanduser = lambda s: (target if s == '~/gaze_calibration.json'
                                    else real_expanduser(s))
    try:
        print('\nfitting:')
        dlg._compute_model(p['dots'])
        check('the dialog finished', dlg.phase == 'done', dlg.phase)
        check('a model was produced', dlg.result_model is not None)
        check('the model is ready', dlg.result_model.is_ready())
        check('the model was saved', os.path.exists(target))
        check('the result label was filled in',
              bool(dlg.result.text()), dlg.result.text()[:60])
        check('the error is a number', dlg.accuracy_worst > 0,
              f'{dlg.accuracy_worst:.0f} px')
        check('the error is usable', dlg.accuracy_worst < 200,
              f'{dlg.accuracy_worst:.0f} px')
    finally:
        os.path.expanduser = real_expanduser

    # ---- the saved file loads back and predicts the same thing -----------
    from gaze_model import GazeModel                        # noqa: PLC0415
    m = GazeModel.load(target)
    check('the saved model reloads', m.is_ready())
    u0, v0 = m.predict(dlg.captured[0][0])
    check('a reloaded model predicts a sane point',
          0.0 <= u0 <= 1.0 and 0.0 <= v0 <= 1.0, f'{u0:.3f}, {v0:.3f}')
    check('the reloaded model agrees with the fitted one',
          abs(u0 - dlg.result_model.predict(dlg.captured[0][0])[0]) < 1e-9)

    # ---- the HUD's loader accepts the file -------------------------------
    print('\nhandoff to the HUD:')
    m2 = GazeModel.load(target)
    check('the report is human-readable',
          isinstance(m2.report(), str) and len(m2.report()) > 0,
          m2.report()[:70])

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
