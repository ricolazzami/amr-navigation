# Project 3 — AMR Navigation

Autonomous mobile robot navigation in simulation: LiDAR mapping, EKF state
estimation, a hand-written A* planner, and reflex-level dynamic obstacle
avoidance.

Stack: **ROS 2 Jazzy + Gazebo Harmonic (gz sim) + Nav2 (controller only) +
robot_localization**, all inside Docker on Arch Linux. Navigation localises
against a pre-built map (`map_server` + `nav2_amcl`); online `slam_toolbox`
mapping is still selectable with `slam:=true`.

## The three graded deliverables

| # | Deliverable | Where |
|---|---|---|
| 1 | EKF fusing IMU + wheel odometry into `odom/filtered` | `config/ekf.yaml`, `ekf_filter_node` |
| 2 | Custom A* on a discrete occupancy array, Manhattan heuristic | `src/amr_planning/amr_planning/astar.py` |
| 3 | Dynamic avoidance: local costmap → proximity error → smooth `Twist` | `src/amr_control/amr_control/avoidance_node.py` |

## Architecture

```
LiDAR ──► map_server + AMCL ──► /map + map→odom ─► a_star_node (4-way Manhattan A*)
                                                          │ FollowPath action
                                                          ▼
IMU ────┐                                           controller_server
Wheel ──┴─► ekf_node ──► TF odom→base_footprint ──► /cmd_vel_nav
                                                          │
LiDAR ──► local_costmap ──► avoidance_node (tanh) ────────┴─► /cmd_vel ──► robot
```

TF: `map → odom → base_footprint → base_link → {laser_link, imu_link, wheel_*}`

Design notes:

- **Localisation uses a pre-built map + AMCL by default.** Online SLAM over the
  sparse box maze drifts ~0.8–1.6 m against the (accurate) EKF odometry, which
  put the A* start/obstacle cells in the wrong place; localising against a known
  map removes that failure mode. `slam:=true` restores online mapping with
  `slam_toolbox` (and never runs AMCL at the same time — both would publish
  `map→odom`).
- **The map is authored in the world frame** (origin `-5.5,-5.5`, 11 m at
  0.05 m), so world `(4,4)` is map `(4,4)` and the goal below needs no offset.
  It is generated from the SDF: `python3 scripts/make_map.py`.
- **Nav2 is used for the local costmap and the controller only.** Planning is
  our own node, executing Nav2's `FollowPath` action with the A* path.
- **The avoidance node has the last word on `/cmd_vel`.** The controller
  publishes `/cmd_vel_nav`; the avoidance node scales it and republishes.
- **Dynamic obstacles are marked in the planner's overlay**, not in Nav2's
  static layer, which cannot be written at runtime.
- **1-second hold, then replan.** When the robot is pinned, the cell is
  reported after 1 s and A* re-routes.

## Build and run

```bash
# host, once
sudo pacman -S --needed xorg-xhost
xhost +local:docker

# build image + shell
./scripts/run.sh

# inside the container
colcon build --symlink-install
source install/setup.bash
ros2 launch amr_bringup demo.launch.py
```

Send a goal from RViz with **2D Goal Pose**, or:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 4.0, y: 4.0}, orientation: {w: 1.0}}}"
```

Useful flags:

```bash
ros2 launch amr_bringup demo.launch.py headless:=false   # Gazebo GUI (needs more RAM)
ros2 launch amr_bringup demo.launch.py dynamic:=false    # park the moving obstacle
ros2 launch amr_bringup demo.launch.py rviz:=false       # server only
ros2 launch amr_bringup demo.launch.py slam:=true        # online SLAM instead of AMCL
```

The dynamic obstacle is driven with the `gz service` CLI (`/world/maze/set_pose`)
because Harmonic exposes that only as a gz-transport service and the launch
bridges topics, not services.

## Tests

Pure Python core, no ROS runtime needed:

```bash
PYTHONPATH=src/amr_planning:src/amr_control python3 -m pytest \
  src/amr_planning/test src/amr_control/test -q
```

Covers: A* optimality against Dijkstra on 20 random maps, 4-connectivity,
collision-free paths, inflation, nearest-free snapping, path densification
(no gap above `step`, never leaves free cells), corridor geometry, and the
200×200 performance budget.

## Layout

```
src/amr_description/   URDF/xacro, LiDAR + IMU sensors
src/amr_bringup/       launch, world, params, maps, RViz, obstacle mover
src/amr_planning/      A* core + ROS node + tests
src/amr_control/       corridor geometry + avoidance node + tests
docker/                Dockerfile + compose
scripts/               run.sh, make_map.py, dump_map.py, diag_drive.py
docs/                  report, metrics, bags
```

`scripts/make_map.py` regenerates `maps/maze.{pgm,yaml}` from the world.
`scripts/dump_map.py` and `scripts/diag_drive.py` are diagnostic tools
(occupancy dump around the robot; live controller/pose table).

## Known limitations

- Reverse driving is not shielded by the avoidance node (the corridor only
  looks forward).
- The local costmap frame is `odom`; the planner works in `map`. Both are
  handled explicitly, but the two must stay connected through the EKF.
- Gazebo GUI plus RViz plus the whole stack does not fit comfortably in 7 GB;
  the demo runs headless and visualises in RViz.
