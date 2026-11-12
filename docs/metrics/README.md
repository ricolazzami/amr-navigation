# Measured evidence

Nothing here is hand-written: every number is the output of
`scripts/eval_metrics.py` (or a `ros2` dump) on a recorded run.

## Single measured run per scenario

`run_<scenario>.csv` is the 10 Hz log from `scripts/log_run.py`:
`t`, ground truth `gt_x/gt_y/gt_yaw` (from `/world/maze/dynamic_pose/info`),
raw wheel odometry, EKF output, `/cmd_vel_nav`, `/cmd_vel`, the reflex distance,
and the commanded/output yaw rate. `run_<scenario>.csv.map.npy` +
`.map.json` save the exact `/map` grid the planner saw, so a re-evaluation reads
the same occupancy instead of the map file on disk.

`eval_<scenario>.txt` is `eval_metrics.py` on that CSV: JSON block + the
markdown row. Re-running the numbers:

```bash
PYTHONPATH=src/amr_planning python3 scripts/eval_metrics.py \
    docs/metrics/run_static.csv --goal 4 4 --name "Static maze"
```

| Scenario | Success | t (s) | Path / A* optimal | Clearance (m) | Decel (m/s²) | RMSE raw / EKF (m) |
|---|---|---|---|---|---|---|
| Static maze | yes | 89.1 | 15.99 / 16.5 (0.97) | 0.11 | 1.06 | 0.041 / 0.022 |
| Dynamic obstacle | yes | 105.6 | 16.30 / 16.5 (0.99) | 0.11 | 1.59 | 0.023 / 0.021 |
| Dead-end goal | yes | 95.2 | 18.59 / 19.45 (0.96) | 0.10 | 0.90 | 0.025 / 0.024 |

Definitions (snapped goal, 0.30 m arrival radius, median-filtered decel, lateral
and oscillation) are in [`../report.md`](../report.md) §5.1.

## Which clock the times are on

`t` and `time_to_goal_s` are **simulation** seconds: `scripts/log_run.py` runs
with `use_sim_time`, so the column is the Gazebo `/clock`, and the same robot
behaviour measures the same on an idle host and a loaded one. That is not how
it started. Until this was fixed, `log_run.py` never set `use_sim_time`, so `t`
was Unix epoch and every duration was wall-clock. Measured on this host:

| Configuration | Real-time factor |
|---|---|
| Gazebo GUI + RViz | 0.28 |
| Gazebo GUI + RViz + CSV recorder | 0.28 |
| Headless (`rviz:=false`, what the batches use) | 0.64-0.67 |

So a run that was wall-clock 95 s could be 80 s of actual simulated motion, and
the 20 s smoke test took 56 s of wall time at RTF 0.355. `repeat_wallclock/`
holds the eval files measured under the old definition; `repeat/` holds the
re-measured ones. They are kept apart rather than blended.

## Checkpoint A

| File | What it proves |
|---|---|
| `checkA_tf.txt` | TF tree: 8 frames, one publisher per frame (no `map→odom` double owner) |
| `checkA_map_reload.txt` | `map_server` serves the map standalone, without SLAM |
| `checkA_bag_info.txt` | `ros2 bag info` of a full run (5 topics, 9504 msgs, 161 s) |

## `repeat/` — the 10/10 acceptance evidence

`static_01…10.txt` and `dynamic_01…10.txt` are the per-run eval output of
`scripts/repeat.sh` (one fresh container and one fresh launch per run, no code
change in between). `scripts/summarize_repeat.py "<glob>"` counts the pass
conditions.

| Batch | Result | Detail |
|---|---|---|
| static | **10/10** | 0 contact (clearance 0.09-0.12 m), lateral 0.09-0.12 m, 77.9-83.8 s |
| dynamic | **10/10** | 0 contact (clearance 0.08-0.11 m), 0 runs with >= 2 yaw reversals per approach, 10/10 marked the obstacle, 79.7-119.5 s |

`repeat_wallclock/verify_01.txt` is one static run **after** the chassis was
skinned with the iRobot Create body: 90.7 s, ratio 0.96, clearance 0.11 m,
lateral 0.10 m, RMSE 0.016/0.011 m — inside the 10-run static range, which is
the point of a visual-only swap. It sits in `repeat_wallclock/` because it was
measured before the clock fix, so its 90.7 s is wall time.

Raw CSVs for the repeat batches stay in `log/repeat/` (gitignored); the eval
files here are the evidence.

### The dynamic scenario has to test the same thing ten times

In the first dynamic batch the encounter was not reproducible, and the cause was
the mover. Its sweep phase was `t % period` on the absolute simulation clock
while the goal was triggered by a wall-clock `sleep`, so where the box sat when
the robot reached the junction depended on how fast the host happened to run the
simulator: 3 of 10 runs never met the box at all, one was pinned for 44 s, and
travel times ranged 102-197 s.

Five things were wrong; all five are fixed, and each fix was measured:

1. **Phase.** Anchored to the first `/plan` instead of the clock, because the
   wall-clock wait moved the sweeper by seconds between runs. That removed the
   gross drift (a whole 9 s period, 22.0-23.4 s arrival, became a 0.2 s spread
   on three probe runs) but not the encounter: over the ten final runs the
   arrival at x = 0.5 after the plan still spans **22.0-36.7 s** (14.7 s), since
   the box's own sweep sits in the same corridor and delays the robot by seconds
   run to run. Phase anchoring alone was therefore not enough - which is why the
   encounter is staged (item 5).
2. **Anchor topic.** The anchor first hung off `/goal_pose`. That topic comes
   from a one-shot `ros2 topic pub --once`, which exits as soon as it has
   matched a subscriber; with the mover also subscribed, 2 of 3 runs delivered
   the goal to the mover and not to the A* planner, and the robot never moved.
3. **The freeze.** The guard was purely radial (0.75 m, 2-D), so the box froze
   wherever that distance first dropped - the bottom of the sweep, ~0.6 m off
   the lane and outside the reflex's 0.18 m forward corridor. The robot drove
   past an obstacle it could not see: `obs_d` stayed infinite for the whole
   encounter, 0 approaches, decel 0.90 m/s2 (no braking at all). The guard is
   now lateral as well (0.18 m, the same number as `corridor_half_width`), and
   the box stays where it stopped instead of snapping back to the schedule -
   without that the hold lapsed in under a second and the reflex never reached
   its 1 s hold, so nothing was ever marked or replanned.
4. **The hold timer.** The 1 s hold was re-armed by a single tick in which the
   obstacle was not pinching. The box is only *just* inside the pinch threshold
   at a shallow standoff (measured `obs_d` oscillating 0.334-0.36 against a
   0.350 m edge), and the local costmap drops the box for a tick at control rate,
   so the timer kept resetting and `elapsed` never left 0.60 s - while a slightly
   deeper 0.22 m standoff marked fine. `pinch_grace = 0.5 s` absorbs a short gap
   without restarting the clock; the 1 s requirement is unchanged.
5. **The encounter itself.** Phase alone was still not enough. Two consecutive
   probes caught the box crossing in front of the robot for 0.6 s - shorter than
   the reflex's 1 s hold, so nothing was marked and the box swept on - and in
   another run the box reached the lane only after the robot had driven past it:
   `obstacle resumes its sweep after 1.89 sim s: robot (+1.26, -4.19) box
   (0.5, -4.22)`, i.e. the box yielded 0.76 m *behind* the robot. The first time
   the robot comes within `guard_lead = 1.2 m` of the crossing line the box is now
   placed in the robot's own lane and holds until the robot is `release_margin`
   past (capped at 6 s). Once per run, so the hold → mark → replan event is
   present in every dynamic run rather than in whichever runs the sweep happened
   to line up with.

A second, short proximity hold (0.75 m / 0.18 m, capped at 2 s) stays in place
only so the teleported box cannot overlap the robot on the later crossings; it is
not meant to stage a second encounter.

`src/amr_bringup/test/test_obstacle_phase.py` pins the staging (`lead_triggered`,
`stage_target`), that `guard_lateral` is the same number the reflex uses, that the
anchor has not gone back to `/goal_pose`, that the shipped YAML still holds those
values, and that no `self._x = ...` shadows a method. `test_reflex.py` pins the
speed law and the pinch gate.

## `rerun/` — a live re-check, kept separate on purpose

`scripts/repeat.sh static 3` was run again after the Create body swap, for a
demonstration. It writes to the same `repeat/static_0N.txt` names, so the three
acceptance files were restored from git and the new output moved here as
`rerun_static_01...03.txt`.

| Run | Success | t (s) | Path / optimal | Clearance (m) | Decel (m/s2) | RMSE raw / EKF (m) | Lateral (m) |
|---|---|---|---|---|---|---|---|
| rerun_static_01 | yes | 95.8 | 15.84 / 16.5 (0.96) | 0.11 | 0.93 | 0.039 / 0.017 | 0.13 |
| rerun_static_02 | yes | 85.4 | 15.8 / 16.5 (0.96) | 0.12 | 0.80 | 0.036 / 0.017 | 0.12 |
| rerun_static_03 | yes | 104.1 | 15.89 / 16.5 (0.96) | 0.10 | 0.90 | 0.014 / 0.020 | 0.09 |

All three pass every gate, but two sit above the 10-run range (79.9–94.8 s):
95.8 s and 104.1 s. These were measured while the host was also playing the
demo video and extracting frames, which is the obvious candidate but was not
instrumented, so it is not claimed. The numbers are reported as measured, and
they are **not** mixed into the acceptance batch — that batch is the 10 runs
above, unchanged. Note that these three were also measured before the clock fix,
so their times are wall-clock seconds and the 10-run range they are compared
against is the wall-clock one (79.9–94.8 s above). Under simulation time the
same acceptance range is 77.9–83.8 s.

## Demo video segments

The two recorded segments are built by `scripts/record_segment.sh` and glued by
`scripts/compose_video.sh` into `docs/video/demo.mp4`; the segments themselves
are not kept in the repo (the composed film is).