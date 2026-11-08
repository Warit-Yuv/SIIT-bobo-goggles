# Starting and stopping the system

The one-page answer to "how do I close this properly?".

---

## The short version

| I want to… | Do this |
|---|---|
| Stop just SLAM + RViz | Click **Stop view** in the console |
| Stop everything on the VM | `~/m1_control_ws/scripts/kill_vm.sh --all` |
| Close the console | Click the window's ✕, or **Ctrl+Q** |
| Stop the robot | `./m1.sh stop --hard` on the Pi |
| See what is running | `~/m1_control_ws/scripts/kill_vm.sh --status` |

You do **not** need the command line for normal use. The console cleans up after
itself when you close it.

---

## Why "2 maps are fighting"

Two `slam_toolbox` instances were running at the same time. Each publishes its
own `/map`, so the map appeared to jump around between two different versions of
the room. It looked like the map was resetting; it was two mappers disagreeing.

That happens for two reasons, and both are now fixed:

1. **"Launch SLAM + RViz" had no guard.** Every click started another full
   stack. It now takes a `flock`, so a second launch fails immediately instead
   of adding a second mapper.
2. **Closing the console left SLAM and RViz running.** The next launch then
   added a second one. Closing the window now stops the view too (there is a
   checkbox if you would rather it did not).

### Checking for duplicates

```bash
~/m1_control_ws/scripts/kill_vm.sh --status
```

The important line is the last one:

```
=== /map publishers (must be 0 or 1) ===
  1 publisher(s)
```

**More than one publisher means duplicates are running.** The script says so
explicitly:

```
  2 publisher(s)
  WARNING: more than one mapper is publishing /map.
           Run ./kill_vm.sh, then start the view again ONCE.
```

---

## What each shutdown path does

### `kill_vm.sh` — stop the view

```bash
~/m1_control_ws/scripts/kill_vm.sh
```

Stops SLAM, RViz, the gaze HUD and any `ros2 launch m1_control` parents. Leaves
the console running.

This is what the console's **Stop view** button runs.

### `kill_vm.sh --all` — stop everything

```bash
~/m1_control_ws/scripts/kill_vm.sh --all
```

Also stops the console itself. This is the "I am done for the day" command.

The console is deliberately **not** in the default kill list: if it were,
pressing "Stop view" would destroy the window reporting the result, and the app
would vanish for no visible reason.

### Closing the console

Click ✕, or press **Ctrl+Q**. Both route through the same handler, which:

1. Sends E-STOP to the robot.
2. Stops the view stack, if you started it from this window.
3. Exits.

Ctrl+Q is worth remembering: calling `app.quit()` directly would skip the
cleanup and leave SLAM running, which is exactly how the duplicates started.

### Stopping the robot

On the Pi, or over SSH:

```bash
ssh pi@10.0.0.54 'cd /home/pi && ./m1.sh stop --hard'
```

`--hard` matters. Plain `stop` was once observed to report success while the
stack kept running — see `robot/docs/report.md` §7.9.

---

## Full shutdown order

Order matters: stop the consumer before the producer, so nothing logs a stream of
"connection lost" errors on the way down.

```bash
# 1. Operator PC — stop the view and the console
~/m1_control_ws/scripts/kill_vm.sh --all

# 2. Robot — stop the stack
ssh pi@10.0.0.54 'cd /home/pi && ./m1.sh stop --hard'

# 3. Optional: the container itself
ssh pi@10.0.0.54 'docker stop ros-humble-dev'
```

To start again, reverse it: robot first, then the operator.

---

## Starting

```bash
# 1. Robot
ssh pi@10.0.0.54 'cd /home/pi && ./m1.sh bg'      # ~20 s to settle

# 2. Operator PC
~/m1_console.sh
```

Then use the console: **Start** brings the stack up over SSH, and
**Launch SLAM + RViz** brings up the view. Or tick *Also launch SLAM + RViz* on
the Start button and it does both.

> Always launch the console with `~/m1_console.sh`, never `ros2 run` directly.
> The wrapper sets `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI`.
> Missing those produces no error at all — just an empty GUI
> (`robot/docs/report.md` §7.13).

---

## If something is stuck

```bash
# What is actually alive?
~/m1_control_ws/scripts/kill_vm.sh --status

# Nuke the view and try once more
~/m1_control_ws/scripts/kill_vm.sh

# ROS's node list is cached and can report dead nodes; clear it
ros2 daemon stop && ros2 daemon start
```

If the console window is gone but SLAM is still running, the lock file at
`/tmp/m1_operator.lock` is what prevents a second launch. `flock` releases it
automatically when the process dies, so there is no stale lock to delete.

---

## Quick reference

```bash
# VM
~/m1_console.sh                              # start the console
~/m1_control_ws/scripts/kill_vm.sh           # stop the view
~/m1_control_ws/scripts/kill_vm.sh --all     # stop everything
~/m1_control_ws/scripts/kill_vm.sh --status  # what is running?

# Pi
./m1.sh bg           # start, background
./m1.sh start        # start, foreground
./m1.sh stop --hard  # stop
./m1.sh diag         # health check
./m1.sh logs         # follow the log

# Console keys
Ctrl+Q               # quit cleanly (stops the view too)
Space                # E-STOP
0–4                  # select mode
```
