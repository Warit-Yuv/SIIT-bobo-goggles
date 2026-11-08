#!/usr/bin/env python3
"""Render the face overlay with a synthetic face, to check the drawing.

The overlay cannot be checked against a real face without somebody sitting in
front of the camera. This builds a plausible landmark set instead, so the
question "do the eye outlines and arrows land in the right places?" can be
answered from the image rather than by hoping.

Writes /tmp/annotated.jpg.
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'm1_control', 'm1_control'))

import cv2                                                  # noqa: E402
import numpy as np                                          # noqa: E402

from gaze_tracker import EYE_CONTOURS, FaceResult, draw_face  # noqa: E402


def main():
    pts = np.zeros((478, 2), np.float32)

    # A head-shaped blob, so the face box has something to wrap.
    ang = np.linspace(0, 2 * np.pi, 200, endpoint=False)
    pts[:200, 0] = 320 + 110 * np.cos(ang)
    pts[:200, 1] = 240 + 140 * np.sin(ang)

    # Two eye contours, at the right landmark indices.
    for contour in EYE_CONTOURS:
        c = np.array(contour)
        cx, cy = (250, 215) if c[0] == 33 else (395, 215)
        a = np.linspace(0, 2 * np.pi, len(c), endpoint=False)
        pts[c, 0] = cx + 32 * np.cos(a)
        pts[c, 1] = cy + 16 * np.sin(a)

    # Irises looking up and to the right, so the arrows have a direction that
    # is obvious in the image.
    eyes = []
    for cx, cy in ((250, 215), (395, 215)):
        eyes.append((np.array([cx, cy], np.float32),
                     np.array([cx + 12, cy - 7], np.float32), 8.0))

    face = FaceResult(features=np.zeros(10), blinking=False, points=pts,
                      eyes=eyes)

    img = np.full((480, 640, 3), 40, np.uint8)
    draw_face(img, face)
    cv2.rectangle(img, (210, 100), (430, 380), (60, 160, 255), 1)

    out = '/tmp/annotated.jpg'
    cv2.imwrite(out, img)
    print(f'wrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
