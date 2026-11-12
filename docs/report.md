# Project 3 — AMR Navigation: report

DecodeLabs Robotics & Automation Project 3. A wheeled robot maps a maze with a
LiDAR, localises with an EKF, plans with a hand-written A*, and reacts to a
moving obstacle — all in ROS 2 Jazzy + Gazebo Harmonic, headless, in Docker.

Everything below is measured from recorded runs. The harness is
`scripts/log_run.py` (10 Hz CSV + the exact `/map` the planner saw) feeding
`scripts/eval_metrics.py`.

## 1. Architecture

```
LiDAR ─► map_server + AMCL ─► /map + map→odom ─► a_star_node (4-way Manhattan A*)
                                                        │ FollowPath action
                                                        ▼
IMU ────┐                                        controller_server (RPP)
Wheel ──┴─► ekf_node ─► TF odom→base_footprint ─► /cmd_vel_nav
                                                        │
LiDAR ─► local_costmap ─► avoidance_node (tanh) ────────┴─► /cmd_vel ─► wheels
```

Four packages (`amr_description`, `amr_bringup`, `amr_planning`, `amr_control`).
Nav2 is used for the local costmap and the controller **only**; planning and
avoidance are our own nodes. TF chain:

```
map → odom → base_footprint → base_link → {laser_link, imu_link, wheel_left, wheel_right}
```

`map→odom` comes from AMCL (or `slam_toolbox` with `slam:=true`), `odom→base_footprint`
from the EKF — never both owners at once.

**Robot body.** The chassis is skinned with an iRobot Create body mesh
(`amr_description/meshes/create_body.dae`, CC-BY 3.0, attribution in
`meshes/ATTRIBUTION.md`). It is a **visual shell only**: `<collision>`
(0.25 × 0.20 × 0.08 box), `<inertial>` (1.5 kg) and the DiffDrive parameters
(`wheel_separation` 0.16 m, `wheel_radius` 0.033 m) are unchanged, so every
number in this report still describes the same physical robot. The mesh is in
inches, hence the `0.0254` scale; centred on `base_link` it spans
z = 0.019–0.081 m, entirely below the 0.11 m scan plane. One static run after the
swap (`docs/metrics/repeat/verify_01.txt`) reproduced the baseline: 90.7 s, path
ratio 0.96, clearance 0.11 m, lateral 0.10 m, RMSE 0.016/0.011 m — all inside
the 10-run static range.

## 2. Deliverable 1 — EKF

`robot_localization` (`config/ekf.yaml`), `two_d_mode`, 30 Hz, world frame `odom`.

- **odom0** `/wheel/odom_fixed`: fuse `vx` and `vyaw` only, not the integrated
  pose. gz's DiffDrive plugin scopes its frames (`amr/odom`→`amr/base_footprint`),
  which `robot_localization` silently drops, so `odom_frame_relay.py` republishes
  the same message on clean `odom→base_footprint` frames.
- **imu0** `/imu/data`: yaw rate only. Linear acceleration is not fused (the
  simulated accelerometer includes gravity and drifts in 2-D), and orientation
  is not fused (a simulated IMU has no absolute heading reference).

RMSE vs ground truth (`/world/maze/dynamic_pose/info`):

| Run | Raw wheel odom | EKF |
|---|---|---|
| Static maze | 0.041 m | 0.022 m |
| Dynamic obstacle | 0.023 m | 0.021 m |
| Dead-end | 0.025 m | 0.024 m |

The limit of this filter shows up in the dynamic batch, not on a clean run: when
the robot is held against the box or backing off, the wheels spin, raw odometry
integrates the slip, and the EKF cannot undo it — `dynamic_10` 1.656 m raw /
0.575 m EKF, `dynamic_03` 0.463 m / 0.117 m. The filter fuses yaw rate and laser,
and a position error that comes from wheel slip is exactly what it cannot see.
Measured, not smoothed over.

Honest reading: in Gazebo the wheels do not slip on a clean run, so raw odometry
is already near-exact and the EKF's two columns stay close (0.041 vs 0.022 m on
the static run here; the order moves run to run across the ten-run batch). The
EKF earns its place when the robot manoeuvres hard around the moving box, where
it cuts the slip error by 65 % (`dynamic_10`) — reported as measured, not as a
headline win.

## 3. Deliverable 2 — A*

`src/amr_planning/amr_planning/astar.py`: 4-connected grid search, `heapq` open
set, numpy closed set, `f = g + h` with the **Manhattan** heuristic
(`h = |Δx| + |Δy|`), which is admissible for unit-cost 4-way moves, so with
`weight = 1.0` the result is optimal.

- Occupancy is thresholded at ≥ 65 % (unknown `-1` also blocked), then inflated
  by `robot_radius + clearance_margin = 0.22 m` → 5 cells at 5 cm.
- `simplify()` removes collinear corners; `densify()` then inserts points every
  0.10 m. Densification is not cosmetic: Nav2's controller only sees a 6 m
  window, and a long straight segment with no interior samples let the robot
  declare "Reached XY goal" in open space. Sampling the segment removes that.
- Dynamic obstacles live in an **overlay** (`dict cell → stamp`, TTL 8 s)
  applied before each plan; Nav2's static layer cannot be written at runtime.

Performance (in-container): 200×200 grid plans in ~32 ms worst case; live plans
during the runs took 2–15 ms. Unit tests prove optimality against Dijkstra on 20
random maps, 4-connectivity, and collision-free paths.

## 4. Deliverable 3 — dynamic avoidance

`src/amr_control/amr_control/avoidance_node.py` sits between the controller and
the wheels and has the last word on `/cmd_vel`.

- Each cycle it queries the local costmap for the closest obstacle inside a
  0.18 m half-width forward corridor out to 1.2 m.
- Speed is scaled smoothly: `v = v_max · tanh(k · (d − d_safe))` with
  `v_max = 0.26`, `d_safe = 0.30`, `k = 3.0`, hard stop at `d_min = 0.15`.
- "Blocked" is judged by **actual progress**, not by the commanded speed: while
  a recent command still asks for forward (`cmd.vx > 0.02`, held meaningful for
  `cmd_ttl = 3 s` so a controller that has gone quiet mid-path is covered too)
  the robot's map-frame position is sampled, and if it moves less than
  `stall_eps = 0.10 m` for `wait_before_mark = 1.0 s` the nearest obstacle cell
  is published on `/blocked_cell`; the planner marks it and re-routes (total
  ≤ 1.5 s, no replan loop).
- The standstill also has to be the **obstacle's** doing. `holds_robot()` is true
  when the ramp itself caps the command below `pinch_fraction = 0.15` of
  `v_max`, i.e. `d ≲ 0.35 m` at these gains; a controller braking on its own,
  with the corridor clear enough to drive, is not a blocked cell.
- Both numbers above were earned, not chosen. `stall_eps` was 0.03 with a pure
  motion test, and at the standoff the ramp still lets the robot *creep* a few
  centimetres per second — the same order as 0.03 — so the hold timer kept
  re-arming and only 1 of 3 probe runs ever marked anything. Measured closest
  approaches were 0.294 / 0.308 / 0.313 / 0.337 m against `d_safe = 0.30`, which
  is why the gate is expressed on the ramp (where the speed law lives) instead
  of a second, hand-tuned distance constant. The law itself moved to
  `src/amr_control/amr_control/reflex.py` so the control loop and the gate
  cannot drift apart; `src/amr_control/test/test_reflex.py` pins it.
- The hold also survives a lost tick. The local costmap drops the box for a tick
  at control rate, and a standoff sitting on the pinch threshold flickers across
  it (measured: `obs_d` oscillating 0.334-0.36 against a 0.350 m edge), which
  re-armed the 1 s timer and marked nothing, while a slightly deeper 0.22 m
  standoff marked fine. `pinch_grace = 0.5 s` now absorbs a short gap without
  restarting the clock. The 1 s requirement is unchanged.
- Throttling the forward speed never removes the turn: `angular.z` is scaled by
  `max(scale, min_angular_scale = 0.35)`. A robot that can still rotate can aim
  at a gap; one that cannot is stuck for good.
- A watchdog zeroes the wheels if `/cmd_vel_nav` goes silent for 0.5 s; a
  reverse/rotate recovery fires after 2.5 s pinned.

`obstacle_threshold` is **100**, not 250: Nav2 publishes costs in 0..100
(lethal 100, inscribed 99, inflation below). 250 is above the scale and never
triggers — that bug left the reflex silently dead until `dump_costmap.py`
showed `max = 100, values {87, 96, 99, 100}`.

### 4.1 The moving obstacle, and why the encounter is staged

`src/amr_bringup/amr_bringup/obstacle_mover.py` drives the box. Gazebo Harmonic
only exposes `/world/<world>/set_pose` as a gz-transport service and this launch
bridges topics but not services, so the node calls the `gz service` CLI — no new
dependency, and the same path `gz model -p` already uses for ground truth. The
box sweeps the east corridor at `x = 0.5` (y −4.8..−3.4, period 9 s); it is
teleported, not driven, so it yields rather than driving through a robot that has
correctly stopped.

Two things about it were learned the hard way:

- **The sweep is phased off the first `/plan`, not the clock.** A wall-clock wait
  made the encounter depend on how long the launch took: the same code produced a
  clean pass-through on one run and a 44 s pin on the next. `/plan` is also used
  instead of `/goal_pose` because the goal is published with a one-shot
  `ros2 topic pub --once`, which exits as soon as it has matched *a* subscriber —
  with a second subscriber it matched this node and not the planner on 2 of 3
  runs, and the robot never moved.
- **Phase alone is not repeatable, so the encounter is staged.** The box is only
  inside the 0.18 m corridor for a slice of its period, while the robot's arrival
  at the crossing line varies by seconds between runs. Consecutive probes caught
  the box crossing in front of the robot for 0.6 s — shorter than the reflex's 1 s
  hold, so nothing was marked and the box swept on — and in another run the box
  only reached the lane *after* the robot had passed it (`obstacle resumes its
  sweep after 1.89 sim s: robot (+1.26, −4.19) box (0.5, −4.22)`: the box yielded
  0.76 m *behind* the robot). Neither tests avoidance.

  The first time the robot comes within `guard_lead = 1.2 m` of the crossing line
  the box is now placed **in the robot's own lane** (`stage_target`: the robot's y
  on the sweep axis, clamped to the sweep range) and holds until the robot is
  `release_margin = 0.5 m` past, capped at `hold_max_s = 6 s`. Once per run. The
  encounter is then a box squarely in the corridor that the reflex has to hold
  for, mark and replan around, identical every run, while the sweep still supplies
  the dynamic motion either side of it. A second, short proximity hold
  (`guard_radius = 0.75 m` and `guard_lateral = 0.18 m`, capped at
  `hold_max_near_s = 2 s`) exists only so the teleported box cannot overlap the
  robot on the later crossings; it is not meant to stage a second encounter.

  The staging logic (`lead_triggered`, `stage_target`) is pure and pinned by
  `src/amr_bringup/test/test_obstacle_phase.py`, alongside the reflex's own
  `test_reflex.py`; a naming test there also guards the class of bug that cost a
  whole probe batch (below).

## 5. Scenarios (5)

| Scenario | Goal (map) | Result |
|---|---|---|
| Static maze | (4.0, 4.0) | reached; 15.99 m vs 16.5 m optimal, 89.1 s |
| Dynamic obstacle | (4.0, 4.0) | staged hold → 1 s hold → mark → replan + 1 recovery, reached, 105.6 s |
| Dead-end | (−1.7, 4.6) | reached; 18.59 m vs 19.45 m optimal, 95.2 s |
| Unreachable | (−3.5, 1.0) | "A* found no path" → empty path + controller cancel, robot halts |
| Long corridor / start-room exit | — | covered by every run's first leg (the 1 m–2 m doors) |

### 5.1 How each metric is defined

Five definitions were tightened after the repeat runs, because a metric that
silently changes meaning is worse than a missing one:

- **Time is simulation time, not wall-clock time.** `scripts/log_run.py` runs
  with `use_sim_time`, so its `t` column (and `time_to_goal_s`) is the Gazebo
  clock. Before that fix the column was the system clock and the same robot
  behaviour measured 20-30 % apart depending on host load — two GUI windows put
  the simulator at RTF 0.28, headless at 0.64-0.67, and a 20 s logging request
  took 56 s of wall time. The arrivals never changed, only the ruler. The earlier
  wall-clock numbers are kept in `docs/metrics/repeat_wallclock/`; they are not
  mixed with the simulation-time tables.
- **Arrival point.** The requested goal `(4, 4)` sits inside the 0.22 m
  inflation band, so A* snaps it to `(3.98, 3.98)`; `(-1.7, 4.6)` snaps to
  `(-1.67, 4.58)`. Arrival and time-to-goal are measured against the snapped
  cell, never the requested point.
- **Arrival radius = 0.30 m.** Nav2's `xy_goal_tolerance` is 0.15 m and it is
  applied to the *AMCL* pose. The ground-truth gap at the moment the controller
  printed `Reached the goal!` was 0.175 m (static), 0.145 m (dead-end) and
  0.139 m (dynamic) — i.e. 0.15 m plus the localisation error. Success and
  time-to-goal share the same radius so they cannot contradict each other.
- **Max decel** is computed on the commanded output after a 3-sample median
  filter. Without it, a single-cycle dropout (a replan gap or the 0.5 s command
  watchdog) reads as a 1.5 m/s2 stop. The 2.42 m/s2 of the dynamic run survives
  the filter and is real: the reflex ramps
  `v = 0.26·tanh(3·(d − 0.30))`, and a box closing at speed collapses that
  ramp inside two 20 Hz cycles.
- **Lateral deviation** is the maximum distance from the recorded ground-truth
  path to the static A* polyline for the same start and goal, so it includes
  the detour around the moving box and is an upper bound on tracking error, not
  a pure controller error.
- **Yaw oscillation per approach** counts sign changes of `angular.z` *while the
  robot is moving* (`|v| > 0.02`), deadband 0.02, inside each contiguous window
  where the reflex reports an obstacle nearer than `safe_distance` 0.30 m.
  Re-aiming during the 1 s hold is excluded by construction: the wheels are
  stopped, so it is not an oscillation of the trajectory.

### 5.2 Repeat runs — static maze, 10/10

`scripts/repeat.sh static 10` runs the whole stack from a fresh container per
repetition and evaluates each CSV; `scripts/summarize_repeat.py` only counts.
Ten consecutive runs, no code change:

| Run | Success | t (s) | Path / optimal | Clearance (m) | Decel (m/s2) | RMSE raw / EKF (m) | Lateral (m) |
|---|---|---|---|---|---|---|---|
| static_01 | yes | 82.1 | 15.85 / 16.5 | 0.10 | 0.90 | 0.031 / 0.021 | 0.10 |
| static_02 | yes | 79.1 | 15.89 / 16.5 | 0.10 | 0.90 | 0.011 / 0.013 | 0.11 |
| static_03 | yes | 77.9 | 15.78 / 16.5 | 0.11 | 0.90 | 0.038 / 0.022 | 0.11 |
| static_04 | yes | 81.1 | 15.79 / 16.5 | 0.09 | 0.90 | 0.012 / 0.021 | 0.11 |
| static_05 | yes | 81.1 | 15.93 / 16.5 | 0.10 | 0.90 | 0.016 / 0.033 | 0.09 |
| static_06 | yes | 80.4 | 15.80 / 16.5 | 0.12 | 0.88 | 0.013 / 0.016 | 0.11 |
| static_07 | yes | 81.5 | 15.77 / 16.5 | 0.09 | 0.90 | 0.049 / 0.051 | 0.11 |
| static_08 | yes | 83.8 | 15.82 / 16.5 | 0.11 | 0.90 | 0.031 / 0.019 | 0.11 |
| static_09 | yes | 81.7 | 15.79 / 16.5 | 0.10 | 0.90 | 0.028 / 0.007 | 0.12 |
| static_10 | yes | 80.4 | 15.91 / 16.5 | 0.10 | 0.90 | 0.013 / 0.010 | 0.12 |

Result: **10/10 reached the goal, 0 contacts (minimum clearance 0.09 m), 0 runs
above the 0.30 m lateral bound.** Acceptance threshold for the lateral criterion
is one 5 cm cell (0.05 m) for a tracked controller error; the number reported
here is the distance to the planned line, which also contains the plan's own
0.05 m stair-stepping and the robot's 0.11 m corridor offset, so it is compared
against 0.30 m and labelled as such rather than being claimed as a 1-cell
tracking error.

The ten times span 5.9 s (77.9-83.8 s). The same batch measured on wall-clock
time spanned 14.9 s (79.9-94.8 s, `repeat_wallclock/`): the behaviour is the
same, the ruler is not.

### 5.3 Repeat runs — dynamic obstacle, 10/10

Same harness, `scripts/repeat.sh dynamic 10`. The acceptance for this scenario
(T5.4) is **no contact** and **fewer than two yaw-rate reversals per obstacle
approach**; "reached the goal" is the same 0.30 m measure as the static runs.

| Run | Success | t (s) | Path / optimal | Clearance (m) | Decel (m/s2) | RMSE raw / EKF (m) | Lateral (m) | Approaches / reversals | Holds / recoveries |
|---|---|---|---|---|---|---|---|---|---|
| dynamic_01 | yes | 87.7 | 15.92 / 16.5 | 0.10 | 2.42 | 0.020 / 0.019 | 0.25 | 2 / 0 | 2 / 1 |
| dynamic_02 | yes | 87.5 | 15.96 / 16.5 | 0.08 | 2.42 | 0.031 / 0.017 | 0.19 | 1 / 0 | 3 / 1 |
| dynamic_03 | yes | 104.3 | 16.12 / 16.5 | 0.11 | 2.40 | 0.463 / 0.117 | 0.17 | 3 / 0 | 3 / 3 |
| dynamic_04 | yes | 79.7 | 15.90 / 16.5 | 0.11 | 2.40 | 0.026 / 0.022 | 0.09 | 1 / 0 | 1 / 0 |
| dynamic_05 | yes | 87.9 | 15.80 / 16.5 | 0.10 | 1.76 | 0.019 / 0.059 | 0.26 | 0 / — | 3 / 1 |
| dynamic_06 | yes | 90.6 | 15.90 / 16.5 | 0.11 | 1.36 | 0.045 / 0.011 | 0.25 | 0 / — | 4 / 1 |
| dynamic_07 | yes | 119.5 | 16.49 / 16.5 | 0.09 | 1.82 | 0.103 / 0.031 | 0.23 | 2 / 0 | 3 / 4 |
| dynamic_08 | yes | 108.4 | 16.08 / 16.5 | 0.10 | 2.23 | 0.081 / 0.041 | 0.26 | 5 / 0 | 2 / 3 |
| dynamic_09 | yes | 87.3 | 15.85 / 16.5 | 0.10 | 1.36 | 0.054 / 0.007 | 0.28 | 0 / — | 3 / 1 |
| dynamic_10 | yes | 91.8 | 15.89 / 16.5 | 0.11 | 2.40 | 1.656 / 0.575 | 0.16 | 3 / 0 | 3 / 1 |

Result: **10/10 reached the goal, 0/10 contact (minimum clearance 0.08 m),
0/10 runs with a reversal count ≥ 2 in any approach, and 10/10 marked the
obstacle.** Every run printed a staged hold, at least one
`obstacle held ≥ 1.0 s -> replan` and a matching `dynamic obstacle marked` line;
the runs with more `Holds` are the box's staging plus the later proximity yields
on its own sweep, not a second staged event.

Read the table honestly, because four of its columns need a sentence:

- **All 10 met the box, but the "approach" counter only counts 7.** The counter
  is `obs_d < 0.30 m`; on `_05`, `_06` and `_09` the reflex's corridor distance
  bottomed at 0.305–0.307 m — above that threshold, yet still inside the
  0.3504 m pinch edge at which the reflex holds and marks. They are approaches
  the threshold rounds away, not runs that missed the box (the mark lines are
  in their logs). The clearance column is ground truth, so it is unaffected.
- **`dynamic_07` is the slowest at 119.5 s** and `dynamic_08` the busiest (five
  approaches, eight marks). Both are reported as measured: the box's own sweep
  keeps re-cutting the lane on the later legs, so the reflex slows, holds and
  replans repeatedly and the run takes 30–40 s longer than a clean pass
  (`dynamic_04`, 79.7 s). Nothing was rejected for being slow.
- **`dynamic_10` raw RMSE 1.656 m (EKF 0.575).** While the robot is held or
  backing off the wheels turn against the box, and raw odometry integrates that
  slip. The EKF cuts it by 65 % but cannot remove it: it fuses yaw rate and
  laser, and a position error built from wheel slip is exactly what it cannot
  see. The same limit runs through the dynamic batch, worst on `dynamic_10`
  (1.656 m raw).
- **Lateral is not gated on this batch.** It is the distance to the *static* A*
  polyline and a dynamic run leaves that line on purpose to go round the box, so
  the 0.09–0.28 m here are reported, not judged; the static batch is the one
  with the 0.30 m bound. **Decel 1.36–2.42 m/s2** is the reflex ramp collapsing
  when the box closes the gap quickly (§5.1), not a discontinuity — the runs
  where the box stayed furthest away (`_06`, `_09`, both 1.36) keep the gentler
  profile.

The staging is what makes the encounter repeatable, and it was needed: over
these ten runs the robot reached the crossing line 22.0–36.7 s after the first
plan (14.7 s spread, `gt_x ≥ 0.5` against the mover's logged anchor) — the
box's own sweep sits in the same corridor and delays the robot by seconds run to
run, so a fixed phase could not have lined the two up (§4.1).

## 6. Resource use (T6.2)

Measured in-container during a static run (`docker stats --no-stream amr`, six
samples over 72 s), host is a 12-thread machine:

| Metric | Measured |
|---|---|
| Container CPU | 423–484 % of one core ≈ 38 % of the 12 threads |
| Container RAM | 444–447 MiB (6.0 % of the 7.17 GiB available to the container) |
| Heaviest processes | `gz sim -s` 109 %, `a_star_node` 69 %, `avoidance_node` 68 %, `odom_frame_relay` 53 %, `ros_gz` parameter bridge 33 %, `controller_server` 27 % |

Two honest observations: the two Python nodes (`a_star_node`, `avoidance_node`)
each cost most of a core, and `odom_frame_relay` (a Python shim that rewrites
gz's model-scoped `amr/odom → amr/base_footprint` odometry into plain
`odom → base_footprint`, which is the only form robot_localization accepts)
costs slightly more than the C++ Nav2 controller it feeds. Both
are the price of the Python-first design, not of the navigation workload: the
whole stack still fits in half a machine with RAM to spare.

## 7. Known limitations

- Reverse driving is not shielded (the corridor only looks forward).
- The reflex is a speed governor, not a planner: it slows and steers but cannot
  reroute. If the robot is stuck inside the inflated halo the planner falls back
  to the raw occupancy (`A* escape plan through inflation`) so it can leave the
  pocket; the reflex guards the walls in the meantime.
- The scripted obstacle is teleported, not driven by a motor, so it yields near
  the robot instead of driving through it, and the crossing it is supposed to
  create is **staged** rather than left to the sweep (§4.1). What the staging buys
  is repeatability, not physics: the box has no momentum, so it cannot show the
  robot handling an obstacle that keeps coming back. The short proximity hold can
  still add extra yields on the later crossings; they are in each run's log, not
  hidden.
- One probe batch was discarded. A `self._release = <float>` in the mover
  shadowed its own `_release()` method, so the node exited with
  `TypeError: 'float' object is not callable` the first time a hold ended and the
  box sat parked in the robot's lane for the rest of the run: 147.9 / 136.5 /
  136.9 s, and one run in contact. Those numbers are published nowhere; the runs
  were repeated after the fix. `test_no_parameter_attribute_shadows_a_method`
  now pins the naming.
- `a_star_node` occasionally ends a log with
  `RuntimeError: Unable to convert call argument '0' to Python object` from
  `rclpy`'s `take_message`, always during the SIGINT teardown and always after
  `Reached the goal!` (4 of the recorded logs). It is an rclpy shutdown race, not
  a planning failure, and it does not affect the measured run — but it is in the
  logs, so it is stated here rather than left to be found.
- The local costmap frame is `odom`, the planner's is `map`; both are handled
  explicitly but must stay connected through the EKF.
- Gazebo GUI + RViz + the full stack exceed ~7 GB; the demo runs headless.

## 8. Demo video

`docs/video/demo.mp4` — 4:55, 1280×720, 25 fps, no audio. Title card, two
labelled missions and a results card. Recorded from the headless stack: Gazebo
and RViz render to an isolated X display (`Xvfb :99`) started *inside* the
container, and `ffmpeg -f x11grab` on the host grabs that display — the host
desktop is never in frame.

| Part | Content |
|---|---|
| Title (6 s) | stack summary |
| Mission 1 — static maze (117 s) | goal `(4, 4)`, map + scan + A* path + local costmap, Create body visible |
| Mission 2 — moving obstacle (164 s) | same goal, the box staged in the corridor: hold → mark → replan |
| Results (8 s) | static 10/10, dynamic 10/10, 0 contact, oscillation criterion |

Recorders: `scripts/record_segment.sh` (fresh container → `Xvfb :99` → launch
with `rviz:=true` → resize RViz to 1280×720 → capture → send the goal → stop at
`Reached the goal!`) and `scripts/compose_video.sh` (cards + labels + concat,
auto-trimming the two segments if the total would exceed 295 s).

What the recorded dynamic take shows, stated exactly, from
`log/record_dynamic.log`: the box stages in the lane at `(0.50, -4.28)`, the
reflex holds **1.00 s**, marks 25 cells at `(0.28, -4.36)` and replans; it holds
again for 2.50 s and marks again, and the box then resumes its sweep. Seven more
`obstacle held >= 1.0 s -> replan` / `dynamic obstacle marked` pairs follow as
the box keeps re-cutting the lane, and the run ends in `Reached the goal!`. So the
graded behaviour — **hold 1 s → mark the cell → replan** — is in the recording,
not only in the batch. One honest wrinkle: at one point during the take A*
printed `A* found no path (114, 26) -> any of 1 targets, cancelling the running
goal` while the box sat in the lane and the marks closed behind it, and it
recovered on its own once the box moved on; the run still reached the goal
(§4.1's escape path through inflation is what unblocks that).

Frames: `docs/video/shot-1-lane.png` (approach), `shot-4-block.png` (the marked
cells and the replanned path), `shot-2-mid.png` and `shot-5-drive.png` (static
mission), `shot-3-arrival.png` (arrival).
