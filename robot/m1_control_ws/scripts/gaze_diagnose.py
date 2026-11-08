#!/usr/bin/env python3
"""Standalone gaze diagnostic. No ROS, no robot, no console.

    python3 scripts/gaze_diagnose.py                 # check the signal
    python3 scripts/gaze_diagnose.py --quick         # + a short calibration
    python3 scripts/gaze_diagnose.py --camera 1      # pick a camera
    python3 scripts/gaze_diagnose.py --record out.npz

WHY THIS EXISTS
---------------
Every calibration so far has ended with "it does not work" and no way to tell
WHY. The dialog could only say "the fit is loose", which is the same message
whether the camera is aimed at a wall, the head barely moved, the eye signal is
flat, or the model is too small. This prints the numbers instead.

WHAT IT CHECKS
--------------
  face        is a face found at all, and in what fraction of frames
  eye signal  does eye_h / eye_v actually MOVE
  head signal does yaw / pitch / head_z move, or is the operator frozen
  spread      per-feature standard deviation, which is what the model needs
  accuracy    with --quick, a real fit on nine dots and the resulting error

The `spread` numbers are the ones that matter. A feature with near-zero spread
cannot be learned from - the model correctly ignores it - so if eye_h and eye_v
are both flat, no calibration can succeed, and this says so before you spend
three minutes finding out.

The window shows the face outline, eye contours, irises and direction arrows,
so it is obvious whether the camera sees you properly.
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                       / 'src' / 'm1_control' / 'm1_control'))

import cv2                                                       # noqa: E402
import numpy as np                                               # noqa: E402

from gaze_model import HEAD_COLUMNS, GazeModel                   # noqa: E402
from gaze_tracker import (FEATURE_NAMES, GazeTracker, describe,   # noqa: E402
                          draw_face)

#: Nine dots: centre first, then the four cardinals, then the corners. The same
#: order the console uses, so a good result here predicts a good result there.
QUICK_DOTS = [
    ('CENTRE', 0.50, 0.50),
    ('TOP', 0.50, 0.10),
    ('RIGHT', 0.90, 0.50),
    ('BOTTOM', 0.50, 0.90),
    ('LEFT', 0.10, 0.50),
    ('TOP-LEFT', 0.10, 0.10),
    ('TOP-RIGHT', 0.90, 0.10),
    ('BOTTOM-RIGHT', 0.90, 0.90),
    ('BOTTOM-LEFT', 0.10, 0.90),
]


def find_camera(preferred=0, log=print):
    """First /dev/videoN that delivers a frame. The index is not stable."""
    for idx in [preferred] + [i for i in range(10) if i != preferred]:
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        ok, _ = cap.read()
        if not ok:
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        log('[diag] camera %d opened' % idx)
        return cap, idx
    log('[diag] NO USABLE CAMERA. Check the webcam is plugged in and not held '
        'by another program (systemctl --user restart xdg-desktop-portal).')
    return None, None


def capture_phase(tracker, cap, seconds, label, show=True, collect=None):
    """Grab frames for `seconds`, drawing and optionally collecting features."""
    t0 = time.time()
    seen = total = 0
    while time.time() - t0 < seconds:
        ok, frame = cap.read()
        if not ok:
            continue
        total += 1
        face = tracker.process(frame, int(time.time() * 1000))
        if face is not None:
            seen += 1
            if collect is not None and not face.blinking:
                collect.append(face.features.copy())
            if show:
                draw_face(frame, face, 1.0)
                cv2.putText(frame, '%s  %s' % (label, describe(face)), (8, 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 255, 200), 1)
        elif show:
            cv2.putText(frame, '%s  NO FACE' % label, (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        if show:
            cv2.imshow('Gaze diagnostic', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return seen, total, True
    return seen, total, False


def report_signal(samples, seconds):
    """Per-feature spread, which is what the model can actually learn from."""
    X = np.array(samples) if samples else np.zeros((0, len(FEATURE_NAMES)))
    print()
    print('  %-9s %8s %8s %8s   %s' % ('feature', 'mean', 'std', 'range', 'verdict'))
    print('  ' + '-' * 70)
    problems = []
    for i, name in enumerate(FEATURE_NAMES):
        if len(X) == 0:
            print('  %-9s %8s %8s %8s   NO DATA' % (name, '-', '-', '-'))
            problems.append(name)
            continue
        col = X[:, i]
        mean, std = float(col.mean()), float(col.std())
        spread = float(col.max() - col.min())
        # Head columns use the same minimum spread the model uses, so this
        # report and the fit agree about what counts as "did not move".
        if i in HEAD_COLUMNS:
            ok = std >= HEAD_COLUMNS[i]
            note = 'ok' if ok else 'FROZEN (model will ignore)'
        else:
            ok = std >= (0.004 if i in (0, 1) else 0.02)
            note = 'ok' if ok else 'TOO STILL'
        if not ok:
            problems.append(name)
        print('  %-9s %8.4f %8.4f %8.4f   %s' % (name, mean, std, spread, note))

    print()
    if len(X) < 20:
        print('  Only %d usable samples in %.0f s - the tracker is not seeing you.'
              % (len(X), seconds))
        return problems
    print('  %d usable samples in %.0f s (%.0f Hz)'
          % (len(X), seconds, len(X) / max(1e-6, seconds)))

    if 'eye_h' in problems and 'look_h' in problems:
        print('  >>> BOTH horizontal eye signals are flat. No calibration can work.')
    if 'eye_v' in problems and 'look_v' in problems:
        print('  >>> BOTH vertical eye signals are flat. Vertical gaze is unusable.')
    if all(n in problems for n in ('yaw', 'pitch', 'head_z')):
        print('  >>> The head never moved. Look at the corners with your head,')
        print('      not just your eyes - the model needs that variation.')
    return problems


def quick_calibration(tracker, cap, per_dot=1.3, settle=0.8):
    """Fit on nine dots and report the real error. Returns (model, data)."""
    win = 'Gaze calibration'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    try:
        rect = cv2.getWindowImageRect(win)
        if rect and rect[2] > 100:
            sw, sh = rect[2], rect[3]
        else:
            sw, sh = 1280, 720

        X, Y, G = [], [], []
        for di, (name, u, v) in enumerate(QUICK_DOTS):
            start = time.time()
            sample_from = start + settle
            end = sample_from + per_dot
            while time.time() < end:
                ok, frame = cap.read()
                if not ok:
                    continue
                face = tracker.process(frame, int(time.time() * 1000))
                if time.time() >= sample_from and face is not None \
                        and not face.blinking:
                    X.append(face.features.copy())
                    Y.append([u, v])
                    G.append(di)

                canvas = np.zeros((sh, sw, 3), np.uint8)
                draw_face(canvas, face, 1.0)
                # Shrink over the settle phase: the cue to start looking.
                frac = min(1.0, max(0.0, (time.time() - start) / max(1e-3, settle)))
                r = int(34 - 25 * frac)
                cv2.circle(canvas, (int(u * sw), int(v * sh)), r,
                           (60, 60, 235), -1, cv2.LINE_AA)
                cv2.putText(canvas, '%s  %d/%d' % (name, di + 1, len(QUICK_DOTS)),
                            (20, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (230, 230, 230), 2)
                cv2.imshow(win, canvas)
                if cv2.waitKey(1) & 0xFF == 27:
                    return None, (X, Y, G)
        return GazeModel(), (X, Y, G)
    finally:
        cv2.destroyWindow(win)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera', type=int, default=0)
    ap.add_argument('--seconds', type=float, default=12.0,
                    help='how long to sample the signal')
    ap.add_argument('--quick', action='store_true',
                    help='also run a short calibration and report the error')
    ap.add_argument('--record', help='save the samples to an .npz for offline fitting')
    ap.add_argument('--no-window', action='store_true')
    args = ap.parse_args()

    print('[diag] gaze diagnostic - no ROS, no robot')
    cap, idx = find_camera(args.camera)
    if cap is None:
        return 1

    try:
        tracker = GazeTracker()
    except Exception as exc:  # noqa: BLE001
        print('[diag] tracker failed to start: %s' % exc)
        cap.release()
        return 1

    show = not args.no_window
    samples = []
    print('[diag] sampling for %.0f s - look around, and move your head a little'
          % args.seconds)
    seen, total, quit_early = capture_phase(tracker, cap, args.seconds, 'SIGNAL',
                                            show=show, collect=samples)
    print('[diag] face found in %d of %d frames' % (seen, total))
    problems = report_signal(samples, args.seconds)

    if args.quick and not quit_early and len(samples) > 50:
        print('\n[diag] quick calibration: %d dots, about %.0f s'
              % (len(QUICK_DOTS), len(QUICK_DOTS) * 2.1))
        model, (X, Y, G) = quick_calibration(tracker, cap)
        if model is not None and len(X) >= 40:
            model.fit(np.array(X), np.array(Y), np.array(G))
            print('  ' + model.report())
            err, _dots = model.validate(np.array(X), np.array(Y), np.array(G))
            print('  error per dot (px at 1920x1080):')
            for (name, _u, _v), e in zip(QUICK_DOTS, err):
                print('    %-14s %4.0f' % (name, e * 1920))
            print('  mean %.0f px, worst %.0f px'
                  % (err.mean() * 1920, err.max() * 1920))
            if err.mean() * 1920 < 120:
                print('  >>> This signal is usable. Run the console calibration.')
            else:
                print('  >>> Still poor. The limiting factor is the signal, not')
                print('      the calibration - see the FROZEN/TOO STILL rows above.')

    if args.record and samples:
        np.savez(args.record, features=np.array(samples),
                 names=np.array(FEATURE_NAMES))
        print('\n[diag] saved %d samples to %s' % (len(samples), args.record))

    if show:
        cv2.destroyAllWindows()
    tracker.close()
    cap.release()

    print()
    if problems:
        print('[diag] features with no usable variation: %s' % ', '.join(problems))
        return 1
    print('[diag] every feature shows variation - the signal is good')
    return 0


if __name__ == '__main__':
    sys.exit(main())
