#!/usr/bin/env python3
"""A* planner node.

Owns the path planning deliverable:

* subscribes to the 2-D occupancy grid produced by SLAM / map_server,
* plans with the in-house 4-way Manhattan A* (``amr_planning.astar``),
* keeps an overlay of cells an obstacle was seen in, so a dynamic obstacle can
  be routed around without touching Nav2's static layer,
* publishes the path for RViz and hands it to Nav2's ``controller_server``
  through the ``FollowPath`` action.
"""

from __future__ import annotations

import math
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

from amr_planning.astar import astar, densify, path_cost, simplify
from amr_planning.grid_utils import (
    GridSpec,
    blocked_mask,
    inflate,
    nearest_free,
    occupancy_to_numpy,
)


class AStarPlanner(Node):
    def __init__(self) -> None:
        super().__init__("a_star_planner")

        # ---- parameters -------------------------------------------------
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("blocked_topic", "/blocked_cell")
        self.declare_parameter("action_name", "follow_path")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("occupied_threshold", 65)
        self.declare_parameter("unknown_is_blocked", True)
        self.declare_parameter("robot_radius", 0.22)
        self.declare_parameter("clearance_margin", 0.05)
        self.declare_parameter("heuristic_weight", 1.0)
        self.declare_parameter("overlay_ttl", 8.0)
        self.declare_parameter("mark_radius", 0.35)
        self.declare_parameter("replan_min_period", 0.4)
        self.declare_parameter("snap_radius_cells", 20)

        gp = self.get_parameter
        self._map_topic = gp("map_topic").value
        self._goal_topic = gp("goal_topic").value
        self._plan_topic = gp("plan_topic").value
        self._blocked_topic = gp("blocked_topic").value
        self._action_name = gp("action_name").value
        self._map_frame = gp("map_frame").value
        self._base_frame = gp("base_frame").value
        self._threshold = int(gp("occupied_threshold").value)
        self._unknown_blocked = bool(gp("unknown_is_blocked").value)
        self._robot_radius = float(gp("robot_radius").value)
        self._margin = float(gp("clearance_margin").value)
        self._weight = float(gp("heuristic_weight").value)
        self._overlay_ttl = float(gp("overlay_ttl").value)
        self._mark_radius = float(gp("mark_radius").value)
        self._min_period = float(gp("replan_min_period").value)
        self._snap_cells = int(gp("snap_radius_cells").value)

        # ---- state ------------------------------------------------------
        self._grid: np.ndarray | None = None
        self._spec: GridSpec | None = None
        self._base_mask: np.ndarray | None = None
        self._raw_mask: np.ndarray | None = None
        self._goal: PoseStamped | None = None
        self._overlay: dict[tuple[int, int], float] = {}
        self._last_plan_s = -1e9
        self._hover = 0

        # FollowPath bookkeeping: avoid preempting the controller 2x/s with an
        # identical path, but resend immediately if the goal died.
        self._active_handle = None
        self._last_sent_cell: tuple[int, int] | None = None
        self._last_sent_s = -1e9

        # ---- interfaces -------------------------------------------------
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, self._map_topic, self._on_map, map_qos
        )
        self.create_subscription(PoseStamped, self._goal_topic, self._on_goal, 10)
        self.create_subscription(
            PointStamped, self._blocked_topic, self._on_blocked, 10
        )

        self._path_pub = self.create_publisher(Path, self._plan_topic, map_qos)
        self._marker_pub = self.create_publisher(Marker, "plan_marker", 10)

        self._client = ActionClient(self, FollowPath, self._action_name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Overlay entries expire; a slow timer replans when that happens.
        self.create_timer(1.0, self._on_housekeeping)

        self.get_logger().info(
            f"A* planner up: {self._map_topic} -> {self._plan_topic}, "
            f"robot_radius={self._robot_radius:.2f} m, weight={self._weight:.2f}"
        )

    # ------------------------------------------------------------------
    # time helpers (sim time aware)
    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid) -> None:
        spec = GridSpec(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
        )
        grid = occupancy_to_numpy(msg.data, spec.width, spec.height)

        changed = self._spec is None or spec != self._spec
        self._spec = spec
        self._grid = grid

        mask = blocked_mask(
            grid, threshold=self._threshold, unknown_is_blocked=self._unknown_blocked
        )
        radius_cells = int(
            math.ceil((self._robot_radius + self._margin) / spec.resolution)
        )
        self._base_mask = inflate(mask, radius_cells)
        self._raw_mask = mask

        if changed:
            self.get_logger().info(
                f"map {spec.width}x{spec.height} @ {spec.resolution:.3f} m, "
                f"origin ({spec.origin_x:.2f}, {spec.origin_y:.2f}), "
                f"inflated by {radius_cells} cells"
            )
            if self._goal is not None:
                self._request_plan()

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = msg
        self.get_logger().info(
            f"goal ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
        self._request_plan(force=True)

    def _on_blocked(self, msg: PointStamped) -> None:
        """Obstacle seen by the avoidance node: mark it and re-route.

        Only cells the static map still believes are free get an overlay entry.
        Walls, crates and unknown space are already blocked by the occupancy
        grid; re-marking them would carve a 0.7 m hole out of the free corridor
        next to every wall the robot drives past.
        """
        if self._spec is None or self._grid is None:
            return
        cx, cy = self._spec.world_to_grid(msg.point.x, msg.point.y)
        r = max(1, int(round(self._mark_radius / self._spec.resolution)))
        now = self._now()
        marked = 0
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                x, y = cx + dx, cy + dy
                if not self._spec.in_bounds((x, y)):
                    continue
                if self._grid[y, x] >= self._threshold:
                    continue  # static obstacle already known
                self._overlay[(x, y)] = now + self._overlay_ttl
                marked += 1
        if marked == 0:
            self.get_logger().info(
                f"blocked cell ({msg.point.x:.2f}, {msg.point.y:.2f}) is already "
                "static, no overlay change"
            )
            return
        self.get_logger().warn(
            f"dynamic obstacle marked at ({msg.point.x:.2f}, {msg.point.y:.2f}), "
            f"{marked} cells, replanning"
        )
        self._request_plan(force=True)

    def _on_housekeeping(self) -> None:
        """Expire overlay cells, then keep the controller fed.

        One timer, two jobs. Expiring dynamic-obstacle cells lets the planner
        re-open a route that was blocked earlier. Re-issuing the plan every
        second is the deadlock breaker: if the controller aborted its goal
        (patience, preemption) the robot would otherwise sit still and SLAM
        would stop changing the map, so nothing would ever call _request_plan
        again. A steady heartbeat guarantees a fresh FollowPath goal.
        """
        if self._overlay:
            now = self._now()
            expired = [cell for cell, t in self._overlay.items() if t <= now]
            for cell in expired:
                del self._overlay[cell]
            if expired and self._goal is not None:
                self._request_plan()
        self._request_plan()

    # ------------------------------------------------------------------
    # planning
    # ------------------------------------------------------------------
    def _request_plan(self, force: bool = False) -> bool:
        if self._grid is None or self._spec is None or self._base_mask is None:
            return False
        if self._goal is None:
            return False

        now = self._now()
        if not force and (now - self._last_plan_s) < self._min_period:
            return False
        self._last_plan_s = now

        start = self._robot_cell()
        if start is None:
            self.get_logger().warn(
                f"no TF {self._map_frame}->{self._base_frame}, cannot plan"
            )
            return False

        mask = self._base_mask.copy()
        mask |= self._overlay_mask()

        # The robot is standing on its own cell; SLAM may leave it unknown
        # (nothing observed under the chassis), which would look blocked.
        if self._spec.in_bounds(start):
            mask[start[1], start[0]] = False

        start_free = nearest_free(mask, start, self._snap_cells)
        if start_free is None:
            self.get_logger().error("robot cell is blocked and no free cell nearby")
            return False
        if start_free != start:
            self.get_logger().warn(
                f"start {start} blocked (inflation/overlay), snapping to {start_free}"
            )

        goal_cell = self._spec.world_to_grid(
            self._goal.pose.position.x, self._goal.pose.position.y
        )
        goal_free = nearest_free(mask, goal_cell, self._snap_cells)

        if goal_free is not None:
            if goal_free != goal_cell:
                self.get_logger().warn(
                    f"goal {goal_cell} blocked, snapping to {goal_free}"
                )
            candidates = [goal_free]
        else:
            # Online SLAM only knows what the LiDAR has seen. A goal outside the
            # current map is not "unreachable", it is "not mapped yet": drive to
            # the frontier (known-free next to unknown) nearest the goal; the map
            # callback replans as the frontier advances.
            candidates = self._frontier_candidates(goal_cell, mask)
            if not candidates:
                self.get_logger().error(
                    f"goal {goal_cell} is outside the map and no frontier found"
                )
                self._publish_empty()
                return False
            self.get_logger().info(
                f"goal {goal_cell} not mapped yet, heading for frontier {candidates[0]}"
            )

        # A frontier candidate can sit across a wall from the robot; try the
        # nearest few instead of giving up on the first unreachable one.
        t0 = time.perf_counter()
        cells = None
        goal_free = candidates[0]
        for candidate in candidates[:8]:
            cells = astar(mask, start_free, candidate, weight=self._weight)
            if cells is not None:
                goal_free = candidate
                break
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        if cells is None and self._raw_mask is not None:
            # The robot can end up inside the inflated halo (tight doorway plus a
            # recovery nudge): every neighbour reads "blocked" and no safe plan
            # exists, so it freezes for good. Plan on the raw occupancy instead —
            # the reflex still guards the walls — so the robot can leave the pocket.
            esc = self._raw_mask.copy() | self._overlay_mask()
            if self._spec.in_bounds(start):
                esc[start[1], start[0]] = False
            for candidate in candidates[:8]:
                cells = astar(esc, start_free, candidate, weight=self._weight)
                if cells is not None:
                    goal_free = candidate
                    break
            if cells is not None:
                self.get_logger().warn(
                    "A* escape plan through inflation (robot boxed in)",
                    throttle_duration_sec=5.0,
                )

        if cells is None:
            self.get_logger().error(
                f"A* found no path {start_free} -> any of {len(candidates[:8])} "
                f"targets ({elapsed_ms:.1f} ms); cancelling the running goal",
                throttle_duration_sec=5.0,
            )
            self._publish_empty()
            return False

        corners = simplify(cells)
        # Dense enough that Nav2's costmap-window truncation never leaves a
        # final pose inside the goal tolerance (see densify docstring).
        path_cells = densify(corners, 2)  # 2 cells = 0.10 m
        path_msg = self._to_path(path_cells)
        self._path_pub.publish(path_msg)
        self._publish_marker(corners)
        self.get_logger().info(
            f"A* {len(corners)} corners, {path_cost(cells, self._spec.resolution):.2f} m, "
            f"{elapsed_ms:.1f} ms, overlay={len(self._overlay)} cells"
        )
        self._send_follow_path(path_msg, goal_free, force)
        return True

    def _robot_cell(self) -> tuple[int, int] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception:  # tf2 raises several unrelated exception types
            return None
        assert self._spec is not None
        return self._spec.world_to_grid(
            tf.transform.translation.x, tf.transform.translation.y
        )

    def _frontier_candidates(
        self, goal_cell: tuple[int, int], mask: np.ndarray
    ) -> list[tuple[int, int]]:
        """Frontier cells ordered by how close they get us to the goal.

        Online SLAM only knows what the LiDAR has seen, so a goal 8 m away is
        initially outside the map. The frontier is the boundary between
        known-free and unknown: driving at the frontier nearest the goal grows
        the map in that direction, and the map callback replans. This is a
        one-function version of frontier exploration, deliberately not a
        separate node.
        """
        assert self._spec is not None and self._grid is not None
        grid = self._grid
        unknown = grid < 0

        # Frontier = known-free cell touching unknown. It must be computed on the
        # raw occupancy grid: the inflated mask has no free cell next to unknown
        # by construction (inflation swallows exactly that band).
        raw_free = (grid >= 0) & (grid < self._threshold)
        frontier = np.zeros_like(raw_free)
        if unknown.any():
            frontier[1:, :] |= unknown[:-1, :]
            frontier[:-1, :] |= unknown[1:, :]
            frontier[:, 1:] |= unknown[:, :-1]
            frontier[:, :-1] |= unknown[:, 1:]
        # The map is cropped to the observed region, so the crop edge is frontier
        # too: a free cell on the border is a place we have not looked past.
        # Without this a fully-cropped map has no frontier and the robot stalls.
        border = np.zeros_like(raw_free)
        border[0, :] = True
        border[-1, :] = True
        border[:, 0] = True
        border[:, -1] = True
        frontier |= border
        frontier &= raw_free

        ys, xs = np.nonzero(frontier)
        if xs.size == 0:
            self.get_logger().warn(
                f"frontier empty: unknown={int(unknown.sum())}, "
                f"free={int(raw_free.sum())}, border_free="
                f"{int((raw_free & border).sum())}"
            )
            return []

        gx, gy = goal_cell
        distance = np.abs(xs - gx) + np.abs(ys - gy)
        order = np.argsort(distance)

        # Only cells the planner is allowed to occupy are useful: keep the
        # inflation band out, then fall back to snapping a raw frontier cell.
        inflated_free = ~mask
        clear = np.flatnonzero(inflated_free[ys, xs])
        if clear.size:
            clear = clear[np.argsort(distance[clear])]
            return [(int(xs[i]), int(ys[i])) for i in clear[:24]]

        out: list[tuple[int, int]] = []
        snap = max(self._snap_cells, 60)
        for i in order[:24]:
            snapped = nearest_free(mask, (int(xs[i]), int(ys[i])), snap)
            if snapped is not None:
                out.append(snapped)
        if not out:
            self.get_logger().warn(
                f"frontier candidates unusable: {xs.size} cells, "
                f"clear={clear.size}, no free cell within {snap} cells"
            )
        return out

    def _overlay_mask(self) -> np.ndarray:
        assert self._spec is not None
        mask = np.zeros((self._spec.height, self._spec.width), dtype=bool)
        for x, y in self._overlay:
            mask[y, x] = True
        return mask

    def _to_path(self, cells: list[tuple[int, int]]) -> Path:
        assert self._spec is not None
        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for i, (gx, gy) in enumerate(cells):
            wx, wy = self._spec.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            if i < len(cells) - 1:
                nx, ny = self._spec.grid_to_world(*cells[i + 1])
                yaw = math.atan2(ny - wy, nx - wx)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def _publish_empty(self) -> None:
        """No route: clear the published path and stop the controller.

        Publishing an empty path is not enough on its own: the FollowPath goal
        from the last good plan is still executing, so the robot would keep
        driving its stale route. Cancel it, and the reflector watchdog takes the
        wheels to zero.
        """
        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(path)

        handle = self._active_handle
        self._active_handle = None
        if handle is not None:
            handle.cancel_goal_async()
        self.get_logger().warn(
            "no route: path cleared, controller goal cancelled",
            throttle_duration_sec=5.0,
        )

    def _publish_marker(self, cells: list[tuple[int, int]]) -> None:
        assert self._spec is not None
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "a_star_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.3
        marker.color.a = 0.9
        for gx, gy in cells:
            wx, wy = self._spec.grid_to_world(gx, gy)
            marker.points.append(Point(x=wx, y=wy, z=0.05))
        self._marker_pub.publish(marker)

    def _send_follow_path(
        self, path_msg: Path, goal_cell: tuple[int, int], force: bool
    ) -> None:
        # Sending a new FollowPath goal preempts the running one, and every
        # preemption leaves a moment where the controller has no goal at all.
        # Do it only when the endpoint actually moved, the sent goal is old, or
        # a caller explicitly forces it (new user goal, obstacle marked).
        now = self._now()
        if (
            not force
            and goal_cell == self._last_sent_cell
            and (now - self._last_sent_s) < 5.0
        ):
            return

        if not self._client.server_is_ready():
            if not self._client.wait_for_server(timeout_sec=0.5):
                self.get_logger().warn(
                    f"action '{self._action_name}' not available; path published only"
                )
                return
        goal = FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = ""
        goal.goal_checker_id = ""
        goal.progress_checker_id = ""
        self._last_sent_cell = goal_cell
        self._last_sent_s = now
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("FollowPath goal rejected (controller busy?)")
            self._active_handle = None
            return
        self._active_handle = handle
        # Only clear the handle if it is still the current one: an older,
        # preempted goal finishing must not erase the goal that replaced it.
        handle.get_result_async().add_done_callback(
            lambda fut, h=handle: self._on_goal_done(fut, h)
        )
        self.get_logger().debug("FollowPath goal accepted")

    def _on_goal_done(self, future, handle=None) -> None:
        # The goal finished, aborted or was preempted: forget the handle so the
        # next heartbeat sends a fresh one instead of believing one is active.
        if handle is None or self._active_handle is handle:
            self._active_handle = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SIGINT can arrive after the context is already gone (e.g. launch
        # shutting the group down); destroying twice must not mask the exit.
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
