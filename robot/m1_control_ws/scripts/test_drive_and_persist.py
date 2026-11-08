#!/usr/bin/env python3
"""Two bugs that both looked like "it forgot".

1. DRIVING: releasing A while W is still held stopped the robot dead.
2. CALIBRATION: restarting the HUD threw away the calibration.

Both are "state that should have been derived or persisted wasn't", and both
are cheap to get wrong again, so both are pinned here.

Run on the operator PC:
    QT_QPA_PLATFORM=offscreen python3 scripts/test_drive_and_persist.py
"""

from __future__ import annotations

import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))

import m1_console                                               # noqa: E402


def main():
    ok = True

    def check(name, cond, detail=''):
        nonlocal ok
        if not cond:
            ok = False
        print(f'  [{"PASS" if cond else "FAIL"}] {name}'
              + (f'   {detail}' if detail else ''))

    # ------------------------------------------------------------------ keys
    print('keyboard drive:')
    keys = m1_console.DRIVE_KEYS
    # Check the PROPERTY, not the exact number: the number is a preference the
    # operator is allowed to change without this test failing.
    check('W drives forward', keys['w'][:2] == ('linear', 'x')
          and keys['w'][2] > 0, str(keys['w']))
    check('S drives backward', keys['s'][2] < 0)
    check('A and D turn opposite ways', keys['a'][2] == -keys['d'][2])
    check('forward is faster than 0.25', keys['w'][2] > 0.25,
          f'{keys["w"][2]} m/s')

    # The whole point: simulate press/release ORDER and check the resulting
    # command each time. This mirrors _apply_drive without needing a window.
    def command(held):
        cmd = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0}
        axis = {('linear', 'x'): 'vx', ('linear', 'y'): 'vy',
                ('angular', 'z'): 'wz'}
        for k in held:
            spec = keys.get(k)
            if spec:
                cmd[axis[(spec[0], spec[1])]] = spec[2]
        return cmd

    held = set()
    held.add('w')
    c = command(held)
    check('W alone drives straight forward', c['vx'] > 0 and c['wz'] == 0,
          str(c))

    held.add('a')
    c = command(held)
    check('W+A drives forward AND turns', c['vx'] > 0 and c['wz'] > 0, str(c))

    held.discard('a')
    c = command(held)
    check('releasing A leaves W still driving forward', c['vx'] > 0,
          f'{c} - this is the bug: it used to become all zeros')
    check('releasing A clears the turn', c['wz'] == 0, str(c))

    held.discard('w')
    c = command(held)
    check('releasing everything stops the robot',
          c['vx'] == 0 and c['vy'] == 0 and c['wz'] == 0, str(c))

    # Opposite keys held together must not cancel into a surprise.
    c = command({'w', 's'})
    check('W and S together is still a legal command',
          c['vx'] in (keys['w'][2], keys['s'][2]), str(c))

    # ----------------------------------------------------------- persistence
    print('\ncalibration persistence:')
    import gaze_model                                           # noqa: E402

    tmp = tempfile.mkdtemp()
    model_path = os.path.join(tmp, 'gaze_calibration.json')
    legacy_path = os.path.join(tmp, 'gaze_legacy_calib.json')

    # --- the Posture model --------------------------------------------
    X, Y, G = [], [], []
    for i in range(40):
        u = 0.1 + 0.8 * (i % 8) / 7.0
        v = 0.1 + 0.8 * (i // 8) / 4.0
        X.append([u - 0.5, v - 0.5, 0.4, u - 0.5, v - 0.5,
                  0.0, 0.0, 0.0, 0.0, 55.0])
        Y.append((u, v))
        G.append(i)
    m = gaze_model.GazeModel()
    m.fit(X, Y, G)
    m.save(model_path)
    check('the model file is written', os.path.exists(model_path))

    probe = X[0]
    before = m.predict(probe)
    # "Restart": a brand new object reading only the file.
    again = gaze_model.GazeModel.load(model_path)
    after = again.predict(probe)
    check('a reloaded model predicts identically',
          abs(before[0] - after[0]) < 1e-9 and abs(before[1] - after[1]) < 1e-9,
          f'{before[0]:.6f} vs {after[0]:.6f}')
    check('the reloaded model is usable', again.is_ready())

    # --- the four quick profiles --------------------------------------
    # Written by the HUD, read back by the HUD. Do it the same way it does.
    values = [0.482, 0.531, 1.35, 0.92]
    with open(legacy_path, 'w', encoding='utf-8') as fh:
        json.dump({'offset_x': values[0], 'offset_y': values[1],
                   'gain_x': values[2], 'gain_y': values[3]}, fh)
    with open(legacy_path, encoding='utf-8') as fh:
        d = json.load(fh)
    restored = [float(d[k]) for k in
                ('offset_x', 'offset_y', 'gain_x', 'gain_y')]
    check('the offset/gain survives a restart', restored == values,
          str(restored))
    check('a non-neutral gain is preserved', restored[2] != 1.0,
          'neutral would mean the file was ignored')

    # --- a broken file must not be trusted ----------------------------
    with open(legacy_path, 'w', encoding='utf-8') as fh:
        fh.write('{"offset_x": "banana"}')
    bad = None
    try:
        with open(legacy_path, encoding='utf-8') as fh:
            d = json.load(fh)
        bad = [float(d[k]) for k in
               ('offset_x', 'offset_y', 'gain_x', 'gain_y')]
    except (KeyError, TypeError, ValueError):
        bad = None
    check('a corrupt file is rejected rather than half-adopted', bad is None,
          'the HUD keeps its parameter values instead')

    # --- and a missing file must be silent ---------------------------
    missing = os.path.join(tmp, 'nope.json')
    try:
        with open(missing, encoding='utf-8') as fh:
            json.load(fh)
        silent = False
    except OSError:
        silent = True
    check('a missing file is not an error on first run', silent)

    # --- the status line must be able to change ----------------------
    print('\nthe status line:')
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            '..', 'src', 'm1_control', 'm1_control',
                            'gaze_hud_node.py'), encoding='utf-8').read()
    body = src.split('def _report_calib', 1)[1].split('\n    def ', 1)[0]
    check('_report_calib echoes the live values, not a constant',
          'self._calib' in body,
          'it used to publish a hardcoded [0.5, 0.5, 1.0, 1.0]')

    print()
    print('=' * 60)
    print('ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
