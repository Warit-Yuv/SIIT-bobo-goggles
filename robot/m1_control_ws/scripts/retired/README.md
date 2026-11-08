# Retired tests

These four tests exercised the **two-parameter offset+gain** gaze model:

```
corrected = (raw - offset) * gain + 0.5
```

That model is gone. It was replaced by a ten-feature ridge regression in
`gaze_model.py`, fed by `gaze_tracker.py`. These files call methods that no
longer exist (`GazeHUDNode._blend_head`, `GazeHUDNode._head_rotation`) and
cannot be made to pass without reimplementing the design they were written for.

| File | What it tested | Where that lives now |
|---|---|---|
| `test_gaze_bias.py` | a constant bias shifts the point instead of pinning it | `scripts/test_gaze_new_arch.py` — *bias absorbed, not saturated* |
| `test_gaze_blend.py` | eye and head blend without clipping | `scripts/test_gaze_new_arch.py` — *head pose is not a confound* |
| `test_gaze_noclip.py` | nothing in the acquisition path is clamped | `scripts/test_gaze_new_arch.py` — *no clipping* |
| `test_gaze_pose.py` | the PnP forward vector is `-z`, not `+z` | head pose now comes from MediaPipe's own transformation matrix, so the sign convention is theirs, not ours |

## Why they were not simply fixed

They tested the maths of a model that could not work. Gaze depends on where the
eyes point, where the head points, how far away it is, and where it sits
laterally. Two parameters per axis draw a straight line through one of those
and call it the answer.

Every one of these tests passed while the operator could not drive the robot.
That is the lesson worth keeping: **a test that passes is not the same as a
system that works.** The replacement file asserts the same properties against
the design that replaced it, and `test_posture_calib.py` measures whether the
result is actually accurate.
