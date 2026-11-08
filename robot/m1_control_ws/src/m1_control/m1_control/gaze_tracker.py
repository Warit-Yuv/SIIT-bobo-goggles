#!/usr/bin/env python3
"""Face + iris tracking with MediaPipe FaceLandmarker.

Port of the feature extraction from github.com/lANXEZ/GazeModel, adapted to run
headless inside a ROS node.

WHY THIS REPLACES THE OLD TRACKER
---------------------------------
The previous tracker used `mp.solutions.face_mesh` and produced two numbers:
iris position along the eye-corner line, and iris position off it. Two numbers
cannot describe gaze, because where you are looking depends on all of:

    * where the iris sits inside the eye
    * which way the head is turned
    * how far the head is from the screen
    * where the head is, sideways and vertically

Fitting a two-parameter line through two numbers therefore only works in the
exact posture the calibration was done in. Move your head, and it is wrong.
That is why four rewrites of the calibration maths never helped: the maths was
fine, the INPUT was insufficient.

FaceLandmarker gives three things `face_mesh` does not:

  blendshapes                 52 face-expression scores, including MediaPipe's
                              own eye-direction estimate (eyeLookUpLeft,
                              eyeLookOutRight, ...). This is a trained model's
                              opinion of where the eye points, and it is much
                              less noisy than a hand-rolled iris ratio.
  facial_transformation_matrix
                              head rotation AND position in cm, from MediaPipe
                              rather than our own solvePnP on six landmarks.
  a real blink score          eyeBlinkLeft/Right, so blinking frames can be
                              excluded instead of poisoning the average.

THE TEN FEATURES
----------------
    eye_h    iris left/right inside the eye, averaged over both eyes
    eye_v    iris up/down inside the eye
    eye_open eyelid opening (they close a little when looking down)
    look_h   MediaPipe's eye-direction score, horizontal
    look_v   MediaPipe's eye-direction score, vertical
    yaw      head turned left/right (radians)
    pitch    head tilted up/down (radians)
    head_x   head position relative to the camera (cm)
    head_y
    head_z

The model in gaze_model.py learns how those ten map to a screen position.

USAGE
-----
    tracker = GazeTracker(model_path='~/gaze-models/face_landmarker.task')
    face = tracker.process(frame_bgr, timestamp_ms)
    if face is not None and not face.blinking:
        u, v = model.predict(face.features)

`draw_face()` draws the eye outlines, irises and direction arrows the operator
asked for, and is deliberately separate from tracking so a rendering problem
can never stop the tracker.
"""

from __future__ import annotations

import math
import os
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

#: Where the model lives by default. Downloaded on first use.
DEFAULT_MODEL = os.path.expanduser('~/gaze-models/face_landmarker.task')
MODEL_URL = ('https://storage.googleapis.com/mediapipe-models/'
             'face_landmarker/face_landmarker/float16/1/face_landmarker.task')

# ---------------------------------------------------------------------------
# MediaPipe landmark indices
# ---------------------------------------------------------------------------

#: Eye corners and eyelids, listed image-left first so both eyes measure in the
#: same direction. Using the WRONG order makes one eye's horizontal signal run
#: backwards, and averaging then cancels the very thing being measured.
EYES = (
    {'corners': (33, 133), 'lids': (159, 145)},     # person's right eye
    {'corners': (362, 263), 'lids': (386, 374)},    # person's left eye
)
IRISES = (range(468, 473), range(473, 478))         # centre, then 4 edge points

#: Eye outlines, for drawing. These are the FaceMesh contour loops.
EYE_CONTOURS = (
    [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246],
    [362, 382, 381, 380, 374, 373, 390, 249, 263, 466, 388, 387, 386, 385, 384, 398],
)

#: A blink above this score means the frame should not be used for calibration.
BLINK_THRESHOLD = 0.6

FEATURE_NAMES = (
    'eye_h', 'eye_v', 'eye_open',
    'look_h', 'look_v',
    'yaw', 'pitch',
    'head_x', 'head_y', 'head_z',
)
N_FEATURES = len(FEATURE_NAMES)


@dataclass
class FaceResult:
    """One frame's worth of measurements."""

    features: np.ndarray                 # (N_FEATURES,) - what the model uses
    blinking: bool
    points: np.ndarray                   # (478, 2) landmarks in pixels
    eyes: list = field(default_factory=list)   # (centre, iris, radius) per eye
    blendshapes: dict = field(default_factory=dict)
    head_matrix: np.ndarray | None = None


def ensure_model(path: str | None = None, log=print) -> str:
    """Return a usable model path, downloading it once if needed.

    Raises rather than silently continuing: without the model there is no
    tracking, and a tracker that quietly reports nothing is worse than one that
    says why.
    """
    p = Path(path or DEFAULT_MODEL).expanduser()
    if p.exists() and p.stat().st_size > 100_000:
        return str(p)
    p.parent.mkdir(parents=True, exist_ok=True)
    partial = p.with_suffix('.part')
    log(f'[gaze] downloading face model to {p} ...')
    urllib.request.urlretrieve(MODEL_URL, partial)
    partial.replace(p)
    log(f'[gaze] model ready ({p.stat().st_size // 1024} KB)')
    return str(p)


def _measure_eye(points, eye, iris_centre):
    """Iris position relative to the eye, normalised by eye width.

    Returns (along, down, open) where `along` runs image-left to image-right
    and `down` points down the image, both as fractions of the eye's width.

    Measuring from the eye's CENTRE rather than from one corner matters: a
    corner-based ratio changes when the eye rotates about its own centre, which
    adds a spurious signal that has nothing to do with where the person is
    looking.
    """
    a, b = points[eye['corners'][0]], points[eye['corners'][1]]
    width = np.linalg.norm(b - a)
    if width < 1e-6:
        return 0.0, 0.0, 0.0
    along = (b - a) / width
    down = np.array([-along[1], along[0]])
    offset = iris_centre - (a + b) / 2.0
    top, bottom = points[eye['lids'][0]], points[eye['lids'][1]]
    return (float(offset @ along / width),
            float(offset @ down / width),
            float(np.linalg.norm(bottom - top) / width))


class GazeTracker:
    """Wraps FaceLandmarker and turns a frame into a FaceResult."""

    def __init__(self, model_path: str | None = None, log=print):
        import mediapipe as mp
        from mediapipe.tasks import python as mp_tasks
        from mediapipe.tasks.python import vision

        self._mp = mp
        self.log = log
        path = ensure_model(model_path, log=log)

        options = vision.FaceLandmarkerOptions(
            base_options=mp_tasks.BaseOptions(model_asset_path=path),
            running_mode=vision.RunningMode.VIDEO,
            num_faces=1,
            output_face_blendshapes=True,
            output_facial_transformation_matrixes=True,
        )
        self._landmarker = vision.FaceLandmarker.create_from_options(options)
        self._last_ms = -1

    # ------------------------------------------------------------------ core
    def process(self, frame_bgr, timestamp_ms):
        """Frame -> FaceResult, or None when no face is found."""
        import mediapipe as mp

        # VIDEO mode requires strictly increasing timestamps.
        timestamp_ms = max(int(timestamp_ms), self._last_ms + 1)
        self._last_ms = timestamp_ms

        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        result = self._landmarker.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), timestamp_ms)

        if not result.face_landmarks or len(result.face_landmarks[0]) < 478:
            return None

        h, w = frame_bgr.shape[:2]
        points = np.array([(lm.x * w, lm.y * h) for lm in result.face_landmarks[0]],
                          dtype=np.float64)

        # Iris radius, so the drawing can size the circle to the actual eye.
        irises = [
            (points[r.start],
             float(np.mean(np.linalg.norm(
                 points[r.start + 1:r.stop] - points[r.start], axis=1))))
            for r in IRISES
        ]

        measurements, eyes = [], []
        for eye in EYES:
            centre = points[list(eye['corners'])].mean(axis=0)
            # Match iris to eye by NEAREST rather than by index: the iris
            # ranges are in a fixed order, but a mirrored or rotated frame can
            # put the left iris in the right slot, and swapping them silently
            # inverts the horizontal signal.
            iris_centre, radius = min(
                irises, key=lambda ir: np.linalg.norm(ir[0] - centre))
            measurements.append(_measure_eye(points, eye, iris_centre))
            eyes.append((centre, iris_centre, radius))

        eye_h, eye_v, eye_open = np.mean(measurements, axis=0)

        blend = {}
        if result.face_blendshapes:
            blend = {c.category_name: float(c.score)
                     for c in result.face_blendshapes[0]}

        # MediaPipe's own eye-direction estimate. Positive look_h means looking
        # to the image's right; positive look_v means looking up.
        look_h = ((blend.get('eyeLookOutLeft', 0.0)
                   + blend.get('eyeLookInRight', 0.0))
                  - (blend.get('eyeLookInLeft', 0.0)
                     + blend.get('eyeLookOutRight', 0.0)))
        look_v = ((blend.get('eyeLookUpLeft', 0.0)
                   + blend.get('eyeLookUpRight', 0.0))
                  - (blend.get('eyeLookDownLeft', 0.0)
                     + blend.get('eyeLookDownRight', 0.0)))

        blinking = max(blend.get('eyeBlinkLeft', 0.0),
                       blend.get('eyeBlinkRight', 0.0)) > BLINK_THRESHOLD

        matrix = None
        yaw = pitch = 0.0
        head_x = head_y = head_z = 0.0
        if result.facial_transformation_matrixes:
            matrix = np.asarray(result.facial_transformation_matrixes[0],
                                dtype=np.float64)
            rot = matrix[:3, :3]
            # Same convention as the rest of this project: +yaw is turned to
            # the image's right, +pitch is tilted up.
            yaw = math.atan2(rot[0, 2], rot[2, 2])
            pitch = math.asin(float(np.clip(-rot[1, 2], -1.0, 1.0)))
            head_x, head_y, head_z = (float(matrix[0, 3]), float(matrix[1, 3]),
                                      float(matrix[2, 3]))

        features = np.array([eye_h, eye_v, eye_open, look_h, look_v,
                             yaw, pitch, head_x, head_y, head_z],
                            dtype=np.float64)

        return FaceResult(features=features, blinking=blinking, points=points,
                          eyes=eyes, blendshapes=blend, head_matrix=matrix)

    def close(self):
        try:
            self._landmarker.close()
        except Exception:  # noqa: BLE001
            pass


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

def draw_face(frame, face, scale=1.0):
    """Eye outlines, irises and eye-direction arrows, drawn in place.

    Separate from tracking on purpose: a drawing problem must never be able to
    stop the tracker, and this runs at display rate while tracking runs at
    camera rate.

    The arrow length is exaggerated 6x so a small eye movement is visible. It
    points from the eye centre through the iris, which is the direction the eye
    is actually turned - the same thing the model is being asked to learn.
    """
    if face is None:
        return frame

    for contour in EYE_CONTOURS:
        pts = (face.points[contour] * scale).astype(np.int32)
        cv2.polylines(frame, [pts], True, (80, 220, 80), 1, cv2.LINE_AA)

    for eye_centre, iris_centre, radius in face.eyes:
        c = tuple(int(round(v)) for v in iris_centre * scale)
        tip = tuple(int(round(v)) for v in
                    (iris_centre + (iris_centre - eye_centre) * 6.0) * scale)
        cv2.circle(frame, c, max(int(round(radius * scale)), 1),
                   (255, 200, 0), 1, cv2.LINE_AA)
        cv2.arrowedLine(frame, c, tip, (0, 120, 255), 2, cv2.LINE_AA,
                        tipLength=0.25)
        cv2.circle(frame, c, 2, (0, 0, 255), -1, cv2.LINE_AA)

    # Overall head direction, from the transformation matrix, so a turned head
    # is obvious at a glance even when the eyes are still.
    if face.head_matrix is not None and len(face.eyes) == 2:
        mid = (face.eyes[0][1] + face.eyes[1][1]) / 2.0
        yaw, pitch = face.features[5], face.features[6]
        dx = math.sin(yaw) * 90.0
        dy = -math.sin(pitch) * 90.0
        start = tuple(int(round(v)) for v in mid * scale)
        end = (int(round(start[0] + dx)), int(round(start[1] + dy)))
        cv2.arrowedLine(frame, start, end, (120, 200, 255), 2, cv2.LINE_AA,
                        tipLength=0.2)
    return frame


def describe(face):
    """One-line summary for logs and diagnostics."""
    if face is None:
        return 'no face'
    f = face.features
    return ('eye %+.3f/%+.3f  open %.2f  look %+.2f/%+.2f  '
            'head %+.0f/%+.0f deg  z %.0f cm%s'
            % (f[0], f[1], f[2], f[3], f[4],
               math.degrees(f[5]), math.degrees(f[6]), f[9],
               '  BLINK' if face.blinking else ''))
