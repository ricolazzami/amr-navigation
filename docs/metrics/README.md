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

## Demo video segments

The two recorded segments are built by `scripts/record_segment.sh` and glued by
`scripts/compose_video.sh` into `docs/video/demo.mp4`; the segments themselves
are not kept in the repo (the composed film is).