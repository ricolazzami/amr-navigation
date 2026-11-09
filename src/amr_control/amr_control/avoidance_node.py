#!/usr/bin/env python3
"""Dynamic obstacle avoidance (the "reflex" layer).

Sits between Nav2's controller and the wheels:

    controller_server --/cmd_vel_nav--> avoidance_node --/cmd_vel--> robot

It measures the closest obstacle inside a forward corridor of the local
costmap and scales the commanded speed with a smooth tanh ramp
(``v = v_max * tanh(k * (d - d_safe))``), so the robot eases to a stop instead
of slamming the brakes. If the obstacle stays for ``wait_before_mark`` seconds
the cell is reported on ``/blocked_cell``; the planner marks it and replans.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_geometry_msgs import do_transform_point

from amr_control.grid_view import CostmapView


class AvoidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("avoidance_node")

        # ---- parameters -------------------------------------------------
        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("cmd_in_topic", "/cmd_vel_nav")
        self.declare_parameter("cmd_out_topic", "/cmd_vel")
        self.declare_parameter("blocked_topic", "/blocked_cell")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        self.declare_parameter("max_speed", 0.26)
        self.declare_parameter("safe_distance", 0.55)
        self.declare_parameter("min_distance", 0.28)
        self.declare_parameter("tanh_gain", 3.0)
        self.declare_parameter("scan_radius", 1.5)
        self.declare_parameter("corridor_half_width", 0.35)
        self.declare_parameter("obstacle_threshold", 100)
        self.declare_parameter("steer_gain", 0.8)

        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("watchdog_timeout", 0.5)
        self.declare_parameter("wait_before_mark", 1.0)   # <-- 1 s, per spec
        self.declare_parameter("mark_period", 1.5)

        # recovery: back off and turn when the robot has been pinned this long
        self.declare_parameter("recovery_after", 2.5)
        self.declare_parameter("recovery_period", 2.0)
        self.declare_parameter("recovery_reverse", 0.07)
        self.declare_parameter("recovery_reverse_time", 0.7)
        self.declare_parameter("recovery_rotate", 0.9)

        gp = self.get_parameter
        self._costmap_topic = gp("costmap_topic").value
        self._cmd_in = gp("cmd_in_topic").value
        self._cmd_out = gp("cmd_out_topic").value
        self._blocked_topic = gp("blocked_topic").value
        self._base_frame = gp("base_frame").value
        self._map_frame = gp("map_frame").value

        self._v_max = float(gp("max_speed").value)
        self._d_safe = float(gp("safe_distance").value)
        self._d_min = float(gp("min_distance").value)
        self._k = float(gp("tanh_gain").value)
        self._scan_radius = float(gp("scan_radius").value)
        self._corridor = float(gp("corridor_half_width").value)
        self._obstacle_threshold = int(gp("obstacle_threshold").value)
        self._steer_gain = float(gp("steer_gain").value)

        self._rate = float(gp("control_rate").value)
        self._watchdog = float(gp("watchdog_timeout").value)
        self._wait_before_mark = float(gp("wait_before_mark").value)
        self._mark_period = float(gp("mark_period").value)
        self._recovery_after = float(gp("recovery_after").value)
        self._recovery_period = float(gp("recovery_period").value)
        self._recovery_reverse = float(gp("recovery_reverse").value)
        self._recovery_reverse_time = float(gp("recovery_reverse_time").value)
        self._recovery_rotate = float(gp("recovery_rotate").value)

        if self._d_min >= self._d_safe:
            self.get_logger().warn(
                f"min_distance ({self._d_min}) >= safe_distance ({self._d_safe}); "
                "the robot will only ever creep"
            )

        # ---- state ------------------------------------------------------
        self._view = CostmapView()
        self._costmap_stamp = -1e9
        self._cmd = Twist()
        self._cmd_stamp = -1e9
        self._block_since: float | None = None
        self._pinned_since = -1e9
        self._last_mark = -1e9
        self._last_state = ""
        self._recovery_until = -1e9
        self._recovery_dir = 1.0
        self._recovery_count = 0

        # ---- interfaces -------------------------------------------------
        self.create_subscription(
            OccupancyGrid, self._costmap_topic, self._on_costmap, 10
        )
        self.create_subscription(Twist, self._cmd_in, self._on_cmd, 10)

        self._pub_cmd = self.create_publisher(Twist, self._cmd_out, 10)
        self._pub_blocked = self.create_publisher(PointStamped, self._blocked_topic, 10)
        self._pub_distance = self.create_publisher(Float32, "avoidance/distance", 10)
        self._pub_scale = self.create_publisher(Float32, "avoidance/scale", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0 / self._rate, self._tick)

        self.get_logger().info(
            f"avoidance up: v_max={self._v_max:.2f} m/s, d_safe={self._d_safe:.2f} m, "
            f"d_min={self._d_min:.2f} m, k={self._k:.1f}, "
            f"mark after {self._wait_before_mark:.1f} s"
        )

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # inputs
    # ------------------------------------------------------------------
    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._view.update(msg)
        self._costmap_stamp = self._now()

    def _on_cmd(self, msg: Twist) -> None:
        # Keep the magnitude but remember the source command for scaling.
        self._cmd = msg
        self._cmd_stamp = self._now()

    # ------------------------------------------------------------------
    # control loop
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        now = self._now()
        out = Twist()

        has_cmd = (now - self._cmd_stamp) <= self._watchdog
        obstacle = self._nearest_obstacle(now)

        if obstacle is not None:
            distance, ox, oy = obstacle
            self._pub_distance.publish(Float32(data=float(distance)))
        else:
            distance = None
            self._pub_distance.publish(Float32(data=float("inf")))

        if not has_cmd:
            # Controller silent: never leave the last command latched on the wheels.
            self._publish(out, 0.0)
            self._block_since = None
            self._set_state("WATCHDOG")
            return

        # ---- recovery: pinned for too long, back off and turn -------------
        # A diff-drive robot that has driven its nose into a wall the map has not
        # learned yet cannot recover by stopping alone: stopping freezes the map,
        # which freezes the planner. Back off and rotate, then let the planner
        # (which has now been told about the obstacle) try again.
        if now < self._recovery_until:
            elapsed = self._recovery_period - (self._recovery_until - now)
            rec = Twist()
            if elapsed < self._recovery_reverse_time:
                rec.linear.x = -self._recovery_reverse
            else:
                rec.angular.z = self._recovery_rotate * self._recovery_dir
            self._publish(rec, 0.0)
            self._set_state("RECOVERY")
            return

        target = self._v_max
        if distance is not None:
            if distance <= self._d_min:
                target = 0.0
            else:
                target = self._v_max * math.tanh(self._k * (distance - self._d_safe))
                target = max(0.0, min(self._v_max, target))

        cmd = self._cmd
        if cmd.linear.x < 0.0:
            # Reverse is left untouched: the corridor only looks forward.
            out.linear.x = cmd.linear.x
            out.angular.z = cmd.angular.z
        else:
            out.linear.x = min(max(cmd.linear.x, 0.0), target)
            scale = 1.0
            if abs(cmd.linear.x) > 1e-3:
                scale = out.linear.x / cmd.linear.x
            out.angular.z = cmd.angular.z * scale

            # If we are pinned by an obstacle, steer toward the open side so the
            # robot does not just sit there (optional, small authority).
            if distance is not None and distance < self._d_safe and out.linear.x < 0.05:
                out.angular.z += -self._steer_gain * (self._d_safe - distance) * (
                    1.0 if oy > 0 else -1.0
                )

        self._publish(out, (out.linear.x / self._v_max) if self._v_max else 0.0)

        # ---- blocked-cell reporting -------------------------------------
        # "Blocked" means the controller wants to drive forward but the reflex
        # has taken the speed away. A pure in-place rotation (linear.x == 0)
        # is not an obstacle, and must not mark the map.
        blocking = (
            cmd.linear.x > 0.02
            and distance is not None
            and distance <= self._d_safe
            and out.linear.x <= 0.05
        )
        if blocking:
            if self._block_since is None:
                self._block_since = now
                self._pinned_since = now
            elapsed = now - self._block_since
            if elapsed >= self._wait_before_mark and (now - self._last_mark) >= self._mark_period:
                self._report_blocked(ox, oy, distance)
                self._last_mark = now
            if (now - self._pinned_since) >= self._recovery_after:
                self._recovery_dir *= -1
                self._recovery_count += 1
                self._recovery_until = now + self._recovery_period
                self._pinned_since = self._recovery_until
                self.get_logger().warn(
                    f"pinned {elapsed:.1f} s -> recovery #{self._recovery_count} "
                    f"({'left' if self._recovery_dir > 0 else 'right'})"
                )
            self._set_state("BLOCKED")
        else:
            self._block_since = None
            self._set_state("CRUISE" if target >= self._v_max * 0.99 else "SLOWING")

    def _publish(self, twist: Twist, scale: float) -> None:
        self._pub_cmd.publish(twist)
        self._pub_scale.publish(Float32(data=float(scale)))

    def _set_state(self, state: str) -> None:
        if state != self._last_state:
            self.get_logger().info(f"avoidance state: {state}")
            self._last_state = state

    # ------------------------------------------------------------------
    # sensing
    # ------------------------------------------------------------------
    def _nearest_obstacle(self, now: float):
        """Closest obstacle in the forward corridor.

        Returns ``(distance, world_x, world_y)`` or ``None`` when the corridor
        is clear. The costmap lives in its own frame (usually ``odom`` for the
        local costmap), so the robot is transformed into that frame rather than
        assuming anything about it.
        """
        view = self._view
        if view.grid is None or (now - self._costmap_stamp) > 2.0:
            return None

        frame = view.frame_id or self._map_frame
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, self._base_frame, rclpy.time.Time()
            )
        except Exception:
            return None

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        return view.nearest_in_corridor(
            robot_x=rx,
            robot_y=ry,
            cos_yaw=cos_y,
            sin_yaw=sin_y,
            radius=self._scan_radius,
            half_width=self._corridor,
            threshold=self._obstacle_threshold,
        )

    def _report_blocked(self, x: float, y: float, distance: float) -> None:
        point = PointStamped()
        point.header.frame_id = self._view.frame_id or self._map_frame
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = x
        point.point.y = y
        point.point.z = 0.0

        out = point
        if point.header.frame_id != self._map_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self._map_frame, point.header.frame_id, rclpy.time.Time()
                )
                out = do_transform_point(point, tf)
            except Exception as exc:
                self.get_logger().warn(f"cannot transform blocked cell to map: {exc}")
                return

        self._pub_blocked.publish(out)
        self.get_logger().warn(
            f"obstacle held for {self._wait_before_mark:.1f} s at "
            f"({out.point.x:.2f}, {out.point.y:.2f}) d={distance:.2f} m -> replan"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AvoidanceNode()
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
