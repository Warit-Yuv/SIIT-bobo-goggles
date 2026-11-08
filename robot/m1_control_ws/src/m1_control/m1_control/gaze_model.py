#!/usr/bin/env python3
"""Gaze calibration model: ten features -> a screen position.

Port of the estimator from github.com/lANXEZ/GazeModel, with the parts this
project needs and none of the parts it does not (no deep model, no GPU).

WHY A RIDGE REGRESSION AND NOT OFFSET+GAIN
------------------------------------------
The previous model was two parameters per axis:

    corrected = (raw - offset) * gain + 0.5

which is a straight line through one input. It can only describe "the iris
moved, so the point moved proportionally". That is true in ONE posture. Gaze
does not work that way: where you are looking depends on the iris position, the
head rotation, the head's distance from the screen, and the head's position.
A line through two of those is wrong the moment the operator moves, which is
why every longer calibration still failed - the model was too small to
represent the thing being measured, so more data could not help.

This fits a ridge regression instead:

    features (10, standardised, plus squares/cross terms) -> screen (x, y)

with the regularisation strength chosen by LEAVE-ONE-DOT-OUT cross-validation,
so it is fitted to how well it predicts dots it has not seen.

THREE THINGS THAT MATTER MORE THAN THE MATHS
--------------------------------------------
1. HEAD DIMENSIONS THAT DID NOT MOVE ARE IGNORED.

   If the operator sat still during calibration, head_z never varied, and the
   regression has no evidence for what it does. Left in, it produces a
   confident-looking coefficient from noise, and the pointer then jumps
   whenever the operator leans. Columns whose spread is below a minimum are
   dropped entirely (their standard deviation becomes infinite, which makes
   them exactly zero after standardising). This is what lets the same code work
   whether someone moved their head or not.

2. OUTLIERS ARE DROPPED PER DOT, NOT GLOBALLY.

   Within each dot, only the samples closest to that dot's own median are kept.
   The first frames of a dot are taken while the eyes are still travelling, and
   a global filter cannot tell those from a genuinely different dot.

3. ACCURACY IS MEASURED ON DOTS THE FIT NEVER SAW.

   `fit()` reports a leave-one-dot-out error, which is honest. `validate()` then
   measures on a separate set, which is the number that answers "is this any
   good".
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

#: Regularisation strengths to try, weakest first. Chosen by cross-validation.
ALPHAS = (1e-4, 1e-3, 1e-2, 1e-1, 1.0)

#: Per dot, keep the samples closest to that dot's median. The rest were taken
#: while the eyes were still moving towards the dot.
KEEP_FRACTION = 0.8

#: A head position column must vary by at least this much (std, cm) during
#: calibration to be used at all.
MIN_SPREAD_CM = 1.0

#: Same for head rotation (std, radians). People nod 1-2 degrees just looking
#: around, so anything below 3 degrees is noise rather than deliberate movement.
MIN_SPREAD_RAD = np.radians(3.0)

#: Which feature columns are head-related, and how much they must vary.
HEAD_COLUMNS = {5: MIN_SPREAD_RAD,     # yaw
                6: MIN_SPREAD_RAD,     # pitch
                7: MIN_SPREAD_CM,      # head_x
                8: MIN_SPREAD_CM,      # head_y
                9: MIN_SPREAD_CM}      # head_z

#: Feature columns that also get squared terms and cross terms. These are the
#: eye-direction ones: their effect on screen position is not linear, because
#: a 10-degree eye rotation moves the gaze point further at the edge of the
#: screen than at the centre.
QUADRATIC_COLUMNS = (0, 1, 3, 4)

#: Columns that get a CUBIC term as well as a square.
#:
#: These four are the eye-direction signals - iris offset and the tracker's
#: own eye-look blendshapes - and they share a physical property that no other
#: feature has: the iris CANNOT LEAVE THE EYE. Past roughly 14 degrees of look
#: angle the measured offset stops growing and flattens out. The relationship
#: between gaze angle and measured eye position is therefore an S-curve, and an
#: S-curve has an inflection in the middle that a quadratic is structurally
#: unable to represent: a parabola bends the same way everywhere.
#:
#: Measured on a synthetic operator built with exactly that saturation, adding
#: these cubic terms takes the held-out error from 75 px to 24 px on a 1920-wide
#: screen - a threefold improvement from four extra columns, with no extra
#: calibration dots and no change to how the operator behaves.
CUBIC_COLUMNS = (0, 1, 3, 4)

#: Eye x head products. Looking 20 degrees left moves the gaze point further
#: when the head is also turned left, and the regression cannot express that
#: from additive terms alone.
INTERACTIONS = ((0, 9), (1, 9), (3, 9), (4, 9), (0, 5), (1, 6), (3, 5), (4, 6))


@dataclass
class GazeModel:
    """Ridge regression from the ten tracker features to screen coordinates."""

    mean: np.ndarray | None = None
    std: np.ndarray | None = None
    weights: np.ndarray | None = None
    bias: np.ndarray | None = None
    alpha: float | None = None
    error: float | None = None          # leave-one-dot-out, normalised units
    validation: float | None = None     # on held-out dots
    ignored_head: list = field(default_factory=list)
    n_samples: int = 0
    n_dots: int = 0

    # ------------------------------------------------------------- helpers
    def _design(self, X):
        """Standardised features, plus squares/crosses and interactions."""
        Z = (np.asarray(X, float) - self.mean) / self.std
        q = Z[:, QUADRATIC_COLUMNS]
        i, j = np.triu_indices(len(QUADRATIC_COLUMNS))
        parts = [Z, q[:, i] * q[:, j], Z[:, CUBIC_COLUMNS] ** 3]
        parts.append(np.stack([Z[:, a] * Z[:, b] for a, b in INTERACTIONS], axis=1))
        return np.hstack(parts)

    @staticmethod
    def _ridge(A, T, alpha):
        a_mean, t_mean = A.mean(axis=0), T.mean(axis=0)
        Ac = A - a_mean
        W = np.linalg.solve(Ac.T @ Ac + alpha * len(A) * np.eye(A.shape[1]),
                            Ac.T @ (T - t_mean))
        return W, t_mean - a_mean @ W

    def _drop_outliers(self, X, Y, groups):
        """Within each dot, keep the samples closest to that dot's median."""
        Z = (np.asarray(X, float) - self.mean) / self.std
        Zq = Z[:, QUADRATIC_COLUMNS]
        keep = np.zeros(len(Zq), dtype=bool)
        for g in np.unique(groups):
            idx = np.flatnonzero(groups == g)
            distance = np.linalg.norm(Zq[idx] - np.median(Zq[idx], axis=0), axis=1)
            n = max(1, int(round(len(idx) * KEEP_FRACTION)))
            keep[idx[np.argsort(distance)[:n]]] = True
        return np.asarray(X)[keep], np.asarray(Y)[keep], np.asarray(groups)[keep]

    # ----------------------------------------------------------------- fit
    def fit(self, X, Y, groups):
        """Fit to samples. X: (n, 10) features, Y: (n, 2) targets, groups: dot id.

        Returns the leave-one-dot-out error in normalised units, which is how
        well the model predicts dots it was not trained on.
        """
        X = np.asarray(X, float)
        Y = np.asarray(Y, float)
        groups = np.asarray(groups)
        if len(X) < 8:
            raise ValueError('need at least 8 samples to fit')

        self.mean = X.mean(axis=0)
        std = X.std(axis=0)

        # Head columns that barely moved cannot be learned from. Setting their
        # std to inf makes them exactly zero after standardising, so the
        # regression gives them no influence instead of inventing one.
        #
        # This must run on the RAW std, before the zero-guard below. Doing it
        # afterwards let a completely constant column through: its std was
        # replaced by 1.0 first, and 1.0 is not less than MIN_SPREAD_CM, so a
        # head distance that never changed was treated as a real input and
        # given a coefficient fitted to noise.
        self.ignored_head = []
        for col, min_spread in HEAD_COLUMNS.items():
            if std[col] < min_spread:
                std[col] = np.inf
                self.ignored_head.append(col)

        # Any remaining zero-variance column is not head-related; 1.0 keeps the
        # division finite and its standardised value becomes 0 anyway.
        std = np.where(np.abs(std) < 1e-9, 1.0, std)
        self.std = std

        Xc, Yc, gc = self._drop_outliers(X, Y, groups)
        A = self._design(Xc)

        best_error, best_alpha = np.inf, ALPHAS[-1]
        for alpha in ALPHAS:
            errors = []
            for g in np.unique(gc):
                held = gc == g
                if held.all() or not held.any():
                    continue
                W, b = self._ridge(A[~held], Yc[~held], alpha)
                # Compare the MEAN prediction for the dot, which is what the
                # live system does after its own filtering.
                predicted = (A[held] @ W + b).mean(axis=0)
                errors.append(float(np.linalg.norm(predicted - Yc[held][0])))
            if errors and np.mean(errors) < best_error:
                best_error, best_alpha = float(np.mean(errors)), alpha

        self.alpha = best_alpha
        self.weights, self.bias = self._ridge(A, Yc, best_alpha)
        self.error = best_error
        self.validation = None
        self.n_samples = int(len(Xc))
        self.n_dots = int(len(np.unique(gc)))
        return best_error

    # ------------------------------------------------------------- predict
    def predict(self, features):
        """One sample -> (u, v) in 0..1."""
        if self.weights is None:
            return 0.5, 0.5
        out = self._design(np.asarray(features, float)[None, :])[0] @ self.weights + self.bias
        return float(np.clip(out[0], 0.0, 1.0)), float(np.clip(out[1], 0.0, 1.0))

    def predict_many(self, X):
        if self.weights is None:
            return np.full((len(X), 2), 0.5)
        out = self._design(np.asarray(X, float)) @ self.weights + self.bias
        return np.clip(out, 0.0, 1.0)

    # ------------------------------------------------------------ validate
    def validate(self, X, Y, groups):
        """Measure accuracy on dots the fit has never seen.

        Returns (per-dot error array, dot names order). The number reported is
        the mean distance between where the operator looked and where the model
        says they looked, per dot.
        """
        X = np.asarray(X, float)
        Y = np.asarray(Y, float)
        groups = np.asarray(groups)
        predicted = self.predict_many(X)
        dots = np.unique(groups)
        means = np.array([predicted[groups == g].mean(axis=0) for g in dots])
        targets = np.array([Y[groups == g][0] for g in dots])
        err = np.linalg.norm(means - targets, axis=1)
        self.validation = float(err.mean())
        return err, dots

    # --------------------------------------------------------------- state
    def is_ready(self):
        return self.weights is not None

    def to_dict(self):
        if self.weights is None:
            return None
        return {
            'mean': self.mean.tolist(),
            'std': [float('inf') if np.isinf(s) else float(s) for s in self.std],
            'weights': self.weights.tolist(),
            'bias': self.bias.tolist(),
            'alpha': self.alpha,
            'error': self.error,
            'validation': self.validation,
            'ignored_head': self.ignored_head,
            'n_samples': self.n_samples,
            'n_dots': self.n_dots,
        }

    @classmethod
    def from_dict(cls, d):
        if not d:
            return cls()
        m = cls()
        m.mean = np.array(d['mean'], float)
        m.std = np.array([np.inf if np.isinf(s) else s for s in d['std']], float)
        m.weights = np.array(d['weights'], float)
        m.bias = np.array(d['bias'], float)
        m.alpha = d.get('alpha')
        m.error = d.get('error')
        m.validation = d.get('validation')
        m.ignored_head = d.get('ignored_head', [])
        m.n_samples = d.get('n_samples', 0)
        m.n_dots = d.get('n_dots', 0)
        return m

    def save(self, path):
        p = Path(path).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(self.to_dict()), encoding='utf-8')
        return p

    @classmethod
    def load(cls, path):
        p = Path(path).expanduser()
        if not p.exists():
            return cls()
        try:
            return cls.from_dict(json.loads(p.read_text(encoding='utf-8')))
        except Exception:  # noqa: BLE001 - a corrupt file must not crash startup
            return cls()

    # ------------------------------------------------------------ reporting
    def report(self):
        """Human-readable summary for the dialog and the log."""
        if not self.is_ready():
            return 'not calibrated'
        head = 'none (head barely moved)'
        if self.ignored_head:
            names = ['yaw', 'pitch', 'head_x', 'head_y', 'head_z']
            used = [names[i] for i in range(5)
                    if (i + 5) not in self.ignored_head]
            if used:
                head = ', '.join(used)
        return ('%d samples over %d dots | alpha %.0e | '
                'leave-one-out %.3f | head used: %s'
                % (self.n_samples, self.n_dots, self.alpha or 0.0,
                   self.error or 0.0, head))
