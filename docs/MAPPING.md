# Mapping, localising, and the kidnapped robot

What to do when the robot is picked up, bumped, or shoved into a wall.

---

## The two problems, and why they are different

| Problem | What happens | Can SLAM fix it? |
|---|---|---|
| **Pushed or rotated in place** | Odometry disagrees with the scan; the map skews | Usually — if odometry is honest |
| **Picked up and put down elsewhere** | The robot has no idea where it is | **No.** Not a bug |

The second one is called the **kidnapped robot problem**, and it is a
mathematical limitation, not an implementation flaw. A mapper accumulates a pose
graph; it has no way to know the graph's origin has moved. Feed it a scan from
the other side of the room and it will confidently match it against the wrong
place, then tear the map apart trying to reconcile the two.

No amount of tuning fixes this. The answer is a different **mode**.

---

## Mode 1 — Mapping (build the map, once)

```bash
cd ~/m1_control_ws
./scripts/m1_map.sh start        # start slam_toolbox
# drive the robot around the whole space
./scripts/m1_map.sh save office  # writes ~/maps/office.pgm + .yaml
```

Do this **once per space**. Drive slowly, loop back through rooms so the mapper
can close loops, and avoid fast rotations — every fast turn is a chance for the
scan matcher to lose lock.

### If the map starts drifting

Drift during mapping almost always comes from odometry lying about heading.
Check it directly:

```bash
./scripts/probe_rotation.py 40
# rotate the robot by hand while it records
```

The summary tells you which source is wrong:

```
  /imu/data  (raw sensor)                       net  -81.5 deg
  /odom      (EKF output -> SLAM uses this)     net   -7.4 deg   <- BUG
  /odom_raw  (wheel encoders only)              net   +0.0 deg
```

If `/imu` moved and `/odom` did not, the EKF is trusting the wheels over the
gyro. That is `config/ekf_m1.yaml` — see `robot/docs/report.md` §7.17.

### Resetting a bad map mid-session

There is no `/reset` service in Humble's slam_toolbox, so the pose graph is
cleared by restarting the node:

```bash
./scripts/m1_map.sh reset     # empty pose graph, same session, keeps everything else
```

Save first if any of it is worth keeping:

```bash
./scripts/m1_map.sh save attempt1
./scripts/m1_map.sh reset
```

---

## Mode 2 — Localising (use the map, every visit after)

Once a map exists, **stop mapping** and localise against it instead. Now the
robot can be moved freely inside the map and recover.

```bash
./scripts/m1_map.sh stop
./scripts/m1_map.sh localize office
```

This starts `map_server` (publishes the saved map) and `amcl` (a particle
filter that works out where the robot is from the laser scan).

### After picking the robot up

```bash
./scripts/m1_map.sh relocalize 1.5 -0.5 0.0
#                             x    y   yaw(rad)
```

Or in RViz, click **2D Pose Estimate** (the toolbar arrow) and drag in the
direction the robot faces.

AMCL spreads particles around that guess, compares each particle's predicted
scan against the real one, and converges on the true pose. The particle cloud
visibly collapses in RViz over about a second.

### Why a human has to supply that starting pose

A kidnapped robot, by definition, has no idea where it is. One laser scan often
cannot distinguish two similar corridors, and guessing wrong is worse than
asking — a confident wrong pose drags the whole map with it.

So the workflow splits the work honestly:

| Who | Does what |
|---|---|
| **You** | "It's roughly here, facing this way" |
| **AMCL** | Everything after that, to centimetre accuracy |

You only need to be within about a metre and 45°, which is easy to eyeball on a
map. The default covariance in `m1_map.sh relocalize` is 0.25 m², which tells
the filter exactly how much to trust your click.

---

## Which mode am I in?

```bash
./scripts/m1_map.sh status
```

```
=== modes ===
  mapping  : off
  localise : RUNNING (amcl)

=== /map ===
  publishers: 1

=== robot pose ===
  Translation: [1.42, -0.61, 0.00]

=== saved maps ===
  office
  attempt1
```

**Never run both modes at once.** Both publish `map`→`odom`, so the robot's pose
jumps between two competing transforms and RViz shows a robot that teleports.
`status` warns you if it detects both.

---

## Recommended workflow

```bash
# ---- First time in a new space ----
./scripts/m1_map.sh start
#   drive the whole area slowly, loop back, return to the start
./scripts/m1_map.sh save office
./scripts/m1_map.sh stop

# ---- Every session after ----
./scripts/m1_map.sh localize office
./scripts/m1_map.sh relocalize 0 0 0        # or click 2D Pose Estimate

# ---- Robot got knocked / moved ----
./scripts/m1_map.sh relocalize 2.0 1.0 1.57
#   watch the particle cloud in RViz; if it converges, done

# ---- Localisation refuses to converge ----
./scripts/m1_map.sh relocalize X Y YAW      # try a different starting area
#   still bad? the space probably changed - map it again
```

---

## Command reference

| Command | What it does |
|---|---|
| `m1_map.sh start` | Start mapping (slam_toolbox) |
| `m1_map.sh save NAME` | Save to `~/maps/NAME.{pgm,yaml}` |
| `m1_map.sh list` | List saved maps with resolution and date |
| `m1_map.sh localize NAME` | Localise against a saved map (AMCL) |
| `m1_map.sh relocalize X Y YAW` | Tell AMCL roughly where the robot is |
| `m1_map.sh reset` | Restart the mapper with an empty pose graph |
| `m1_map.sh status` | Which mode is running, robot pose, saved maps |
| `m1_map.sh stop` | Stop mapping or localisation |

---

## Tuning, if convergence is poor

`config/amcl_m1.yaml` is tuned for this robot. The knobs that matter:

| Parameter | Value | Why |
|---|---|---|
| `robot_model_type` | `OmniMotionModel` | Mecanum wheels strafe; a differential model mispredicts sideways motion |
| `max_particles` | 5000 | Enough to keep several hypotheses alive in a furnished room |
| `update_min_d` / `update_min_a` | 0.05 | Small, so a nudge triggers an update instead of waiting for 25 cm of travel |
| `laser_max_range` | 12.0 | Matches the Tmini Plus hardware maximum |
| `set_initial_pose` | true | Otherwise a `2D Pose Estimate` click can be silently rejected |

If the robot localises confidently but in the **wrong** place, raise
`max_particles` and lower `update_min_d` further — the filter is converging too
fast on one hypothesis.

---

## Known limitations

1. **A map only covers what was mapped.** Putting the robot down in an unmapped
   room gives AMCL nothing to match against. It will not converge; map that area
   first.
2. **Furniture that moves** degrades localisation. A chair shifted 30 cm is
   fine; a whole room rearranged is not.
3. **Gyro-only heading drifts** slowly, because this IMU has no magnetometer.
   Over a long session expect a few degrees of yaw error. Localisation against a
   map corrects this continuously; pure mapping does not.
4. **AMCL needs motion to converge.** A robot sitting still produces one scan and
   no new information. Nudge it forward and back a few centimetres if the cloud
   stays wide.
