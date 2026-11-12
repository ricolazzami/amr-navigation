#!/usr/bin/env python3
"""Slide the dynamic obstacle across the robot's corridor.

Gazebo (Harmonic) only exposes ``/world/<world>/set_pose`` as a **gz-transport**
service, and this launch file bridges topics but not services, so a ROS
service client can never connect to it. Driving the same service through the
``gz service`` CLI is the smallest honest fix: no extra bridge, no new
dependency, and it is exactly what ``gz model -p`` already uses for ground
truth.

The sweep phase is anchored to the **first goal**, not to the absolute
simulation clock. Phase-locking to the simulation clock made the encounter
depend on the wall time between launch and the goal, because the trigger is a
wall-clock wait: the same code produced a clean pass-through on one run and a
44 s pin on the next. Anchoring to the goal puts the box in the same place at
the same point in the route on every run, so the dynamic scenario - and its
travel time - is actually repeatable.
"""

from __future__ import annotations

import math
import shutil
import subprocess

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

GZ = shutil.which("gz") or "gz"


def sweep_value(dt: float, offset: float, period: float, lo: float, hi: float) -> float:
    """Position on the sweep axis ``dt`` seconds after the goal was received.

    Smooth there-and-back: 0 at ``dt + offset == 0``, then a cosine ramp up to
    ``hi`` at half a period and back down to ``lo`` at a full period.
    """
    phase = 2.0 * math.pi * ((dt + offset) % period) / period
    return lo + 0.5 * (1.0 - math.cos(phase)) * (hi - lo)


class ObstacleMover(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_mover")

        self.declare_parameter("world_name", "maze")
        self.declare_parameter("model_name", "mover")
        self.declare_parameter("axis", "y")
        self.declare_parameter("x", 0.6)
        self.declare_parameter("z", 0.25)
        self.declare_parameter("y_min", -3.0)
        self.declare_parameter("y_max", 1.6)
        self.declare_parameter("period", 14.0)
        self.declare_parameter("rate_hz", 5.0)
        # Where in the sweep the box sits when the goal is received, in
        # seconds of the sweep period. Tuned so the box reaches the robot's
        # lane (y ~= -4.33, crossed at 1.77 s and 7.23 s into the sweep) at the
        # moment the robot reaches x = 0.5, ~21-23 sim s after the goal.
        self.declare_parameter("phase_offset", 7.0)
        # The box is teleported, not simulated with a motor, so it would sweep
        # straight through a robot that has correctly stopped in front of it.
        # Hold position while the robot is this close: the obstacle yields, the
        # reflex still holds/marks/replans, and nothing physically overlaps.
        self.declare_parameter("guard_radius", 0.75)

        gp = self.get_parameter
        self._world = gp("world_name").value
        self._model = gp("model_name").value
        self._axis = gp("axis").value
        self._x = float(gp("x").value)
        self._z = float(gp("z").value)
        self._lo = float(gp("y_min").value)
        self._hi = float(gp("y_max").value)
        self._period = max(0.1, float(gp("period").value))
        self._offset = float(gp("phase_offset").value)
        self._guard = float(gp("guard_radius").value)
        rate = float(gp("rate_hz").value)

        self._warned = False
        self._held = False
        # Simulation time of the first goal; the sweep phase is measured from
        # here. None until a goal arrives, so the box waits parked at y_min.
        self._t0: float | None = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        # /goal_pose is published once per run by scripts/repeat.sh (or by the
        # operator). Same QoS as the A* planner's own goal subscription.
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"mover '{self._model}' sweeping {self._axis} "
            f"{self._lo:.1f}..{self._hi:.1f} every {self._period:.1f} s, "
            f"phase anchored to the first goal (+{self._offset:.1f} s), "
            f"yielding within {self._guard:.2f} m of the robot"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_goal(self, _msg: PoseStamped) -> None:
        if self._t0 is None:
            self._t0 = self._now()
            self.get_logger().info(
                f"goal received at sim {self._t0:.1f} s: "
                f"anchoring the sweep phase here"
            )

    def _robot_near(self, px: float, py: float) -> bool:
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except Exception:
            return False
        t = tf.transform.translation
        return math.hypot(t.x - px, t.y - py) < self._guard

    def _tick(self) -> None:
        if self._t0 is None:
            # Parked at the low end until a goal arrives, so the box is in the
            # same place at the start of every run.
            value = self._lo
        else:
            value = sweep_value(
                self._now() - self._t0, self._offset, self._period,
                self._lo, self._hi,
            )

        px = value if self._axis == "x" else self._x
        py = value if self._axis != "x" else 0.0

        if self._robot_near(px, py):
            if not self._held:
                self.get_logger().info("robot close: obstacle holds position")
                self._held = True
            return
        self._held = False

        req = (
            f'name: "{self._model}", '
            f"position: {{x: {px:.3f}, y: {py:.3f}, z: {self._z:.3f}}}, "
            "orientation: {w: 1.0}"
        )
        cmd = [
            GZ, "service", "-s", f"/world/{self._world}/set_pose",
            "--reqtype", "gz.msgs.Pose",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "500",
            "--req", req,
        ]
        try:
            out = subprocess.run(
                cmd, capture_output=True, text=True, timeout=1.0
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            if not self._warned:
                self.get_logger().warn(f"set_pose call failed: {exc}")
                self._warned = True
            return
        if out.returncode != 0 and not self._warned:
            self.get_logger().warn(
                f"set_pose returned {out.returncode}: {out.stderr.strip()[:120]}"
            )
            self._warned = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()