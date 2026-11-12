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
| Static maze | yes | 91.2 | 15.95 / 16.5 (0.97) | 0.11 | 0.90 | 0.028 / 0.027 |
| Dynamic obstacle | yes | 132.0 | 16.11 / 16.5 (0.98) | 0.10 | 2.42 | 0.106 / 0.030 |
| Dead-end goal | yes | 105.3 | 18.5 / 19.45 (0.95) | 0.09 | 0.90 | 0.030 / 0.034 |

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
| static | **10/10** | 0 contact (clearance 0.10–0.13 m), lateral 0.09–0.12 m, 79.9–94.8 s |
| dynamic | **10/10** | 0 contact (min clearance 0.07 m), 0 runs with ≥ 2 yaw reversals per approach, 102.0–197.4 s |

`verify_01.txt` is one static run **after** the chassis was skinned with the
iRobot Create body: 90.7 s, ratio 0.96, clearance 0.11 m, lateral 0.10 m,
RMSE 0.016/0.011 m — inside the 10-run static range, which is the point of a
visual-only swap.

Raw CSVs for the repeat batches stay in `log/repeat/` (gitignored); the eval
files here are the evidence.

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
above, unchanged.

## Demo video segments

The two recorded segments are built by `scripts/record_segment.sh` and glued by
`scripts/compose_video.sh` into `docs/video/demo.mp4`; the segments themselves
are not kept in the repo (the composed film is).