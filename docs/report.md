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
| Static maze | 0.028 m | 0.027 m |
| Dynamic obstacle | 0.106 m | **0.030 m** |
| Dead-end | 0.030 m | 0.034 m |

Honest reading: in Gazebo the wheels do not slip, so raw odometry is already
near-exact on a clean run and the EKF cannot beat it. The EKF earns its place
when the robot manoeuvres hard around the moving box: wheel slip drifts raw
odometry and the IMU yaw-rate fusion cuts that error from 0.106 m to 0.030 m
(down 72 %). On the dead-end run the EKF is marginally worse (0.030 → 0.034 m),
inside the noise of a short, clean run — reported as measured.

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
  the controller asks for forward (`cmd.vx > 0.02`) the robot's map-frame
  position is sampled, and if it moves less than `stall_eps = 0.03 m` for
  `wait_before_mark = 1.0 s` the nearest obstacle cell is published on
  `/blocked_cell`; the planner marks it and re-routes (total ≤ 1.5 s, no replan
  loop). Progress-based detection also fires when the controller has gone quiet
  mid-path, which a pure speed test always missed.
- Throttling the forward speed never removes the turn: `angular.z` is scaled by
  `max(scale, min_angular_scale = 0.35)`. A robot that can still rotate can aim
  at a gap; one that cannot is stuck for good.
- A watchdog zeroes the wheels if `/cmd_vel_nav` goes silent for 0.5 s; a
  reverse/rotate recovery fires after 2.5 s pinned.

`obstacle_threshold` is **100**, not 250: Nav2 publishes costs in 0..100
(lethal 100, inscribed 99, inflation below). 250 is above the scale and never
triggers — that bug left the reflex silently dead until `dump_costmap.py`
showed `max = 100, values {87, 96, 99, 100}`.

## 5. Scenarios (5)

| Scenario | Goal (map) | Result |
|---|---|---|
| Static maze | (4.0, 4.0) | reached; 15.95 m vs 16.5 m optimal, 92.8 s |
| Dynamic obstacle | (4.0, 4.0) | 3× BLOCKED → 1 s hold → mark → replan + 1 recovery, reached, 134.2 s |
| Dead-end | (−1.7, 4.6) | reached; 18.5 m vs 19.45 m optimal, 106.7 s |
| Unreachable | (−3.5, 1.0) | "A* found no path" → empty path + controller cancel, robot halts |
| Long corridor / start-room exit | — | covered by every run's first leg (the 1 m–2 m doors) |

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
- The scripted obstacle is teleported, not driven by a motor, so it is set to
  yield near the robot to avoid a non-physical overlap. Because it can freeze at
  a sweep extreme, outside the forward corridor, a given crossing may not block:
  holds are timing-dependent (three events on the measured run).
- The local costmap frame is `odom`, the planner's is `map`; both are handled
  explicitly but must stay connected through the EKF.
- Gazebo GUI + RViz + the full stack exceed ~7 GB; the demo runs headless.
