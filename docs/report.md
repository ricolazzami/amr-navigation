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
| Static maze | 0.016 m | 0.016 m |
| Dynamic obstacle | 0.359 m | **0.196 m** |
| Dead-end | 0.034 m | 0.029 m |

Honest reading: in Gazebo the wheels do not slip, so raw odometry is already
near-exact on a clean run and the EKF cannot beat it. The EKF earns its place
when the robot manoeuvres hard around the moving box: wheel slip drifts raw
odometry by ~0.5 m and the IMU yaw-rate fusion cuts that error by ~45 %.

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
- If the controller still wants to drive forward (`cmd.vx > 0.02`) but the
  reflex has taken the speed away (`out.vx ≤ 0.05`) for `wait_before_mark = 1.0 s`,
  the cell is published on `/blocked_cell`; the planner marks it and re-routes
  (total ≤ 1.5 s, no replan loop).
- A watchdog zeroes the wheels if `/cmd_vel_nav` goes silent for 0.5 s; a
  reverse/rotate recovery fires after 2.5 s pinned.

`obstacle_threshold` is **100**, not 250: Nav2 publishes costs in 0..100
(lethal 100, inscribed 99, inflation below). 250 is above the scale and never
triggers — that bug left the reflex silently dead until `dump_costmap.py`
showed `max = 100, values {87, 96, 99, 100}`.

## 5. Scenarios (5)

| Scenario | Goal (map) | Result |
|---|---|---|
| Static maze | (4.0, 4.0) | reached (3.96, 3.82); 15.89 m vs 16.5 m optimal |
| Dynamic obstacle | (4.0, 4.0) | 1× BLOCKED → 1 s hold → mark → replan, reached |
| Dead-end | (−1.7, 4.6) | reached; 18.59 m vs 19.45 m optimal |
| Unreachable | (−3.5, 1.0) | "A* found no path" → empty path + controller cancel, robot halts |
| Long corridor / start-room exit | — | covered by every run's first leg (the 1 m–2 m doors) |

## 6. Known limitations

- Reverse driving is not shielded (the corridor only looks forward).
- The local costmap frame is `odom`, the planner's is `map`; both are handled
  explicitly but must stay connected through the EKF.
- The scripted obstacle is teleported, not driven by a motor, so it is set to
  yield near the robot to avoid a non-physical overlap.
- Gazebo GUI + RViz + the full stack exceed ~7 GB; the demo runs headless.
