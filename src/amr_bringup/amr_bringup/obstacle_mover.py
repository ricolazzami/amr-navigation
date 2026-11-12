#!/usr/bin/env python3
"""Slide the dynamic obstacle across the robot's corridor.

Gazebo (Harmonic) only exposes ``/world/<world>/set_pose`` as a **gz-transport**
service, and this launch file bridges topics but not services, so a ROS
service client can never connect to it. Driving the same service through the
``gz service`` CLI is the smallest honest fix: no extra bridge, no new
dependency, and it is exactly what ``gz model -p`` already uses for ground
truth.

The sweep phase is anchored to the **first plan**, not to the absolute
simulation clock. Phase-locking to the simulation clock made the encounter
depend on the wall time between launch and the goal, because the trigger is a
wall-clock wait: the same code produced a clean pass-through on one run and a
44 s pin on the next. Anchoring to the plan puts the box in the same place at
the same point in the route on every run, so the dynamic scenario - and its
travel time - is actually repeatable.

The anchor deliberately hangs off ``/plan`` and not ``/goal_pose``. The goal is
published by a one-shot ``ros2 topic pub --once``, which exits as soon as it has
matched *a* subscriber; with a second subscriber on the same topic it matched
this node and not the A* planner on 2 of 3 runs, and the robot never moved. The
``/plan`` topic has one long-lived publisher (the A* node), so subscribing
cannot take a match away from anybody.

``phase_offset`` places the box in the lane at the moment the robot gets there.
Measured: the robot crosses x = 0.5 at 22.0-22.2 sim s after the first plan
(22.0-23.4 before, and arbitrary before that), and it comes inside the 0.75 m
guard radius at ~19.4 s. 0.7 s puts the box 0.02 m off the lane at that instant
(and within 0.14 m of it across a +/-0.3 s spread in the trigger), so the guard
freezes the box squarely in the corridor and the reflex has something to hold,
mark and replan around.
"""

from __future__ import annotations

import math
import shutil
import subprocess

import rclpy
import tf2_ros
from nav_msgs.msg import Path
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
        # Where in the sweep the box sits when the run starts, in seconds of the
        # sweep period. Tuned so the box reaches the robot's lane (y ~= -4.33,
        # crossed at 1.77 s and 7.23 s into the sweep) at the moment the robot
        # reaches x = 0.5, ~21-23 sim s after the goal.
        self.declare_parameter("phase_offset", 0.7)
        # The box is teleported, not simulated with a motor, so it would sweep
        # straight through a robot that has correctly stopped in front of it.
        # Hold position while the robot is this close: the obstacle yields, the
        # reflex still holds/marks/replans, and nothing physically overlaps.
        self.declare_parameter("guard_radius", 0.75)
        # ...but only while the box is in the robot's way. With the radius alone
        # the box froze wherever the *2-D* distance first dropped under it,
        # which was the bottom of the sweep ~0.6 m off the lane and outside the
        # reflex's 0.18 m corridor: the robot sailed past an obstacle it could
        # not see and the run tested nothing. Same number as amr_control's
        # corridor_half_width, so "in the way" means the same to both nodes.
        self.declare_parameter("guard_lateral", 0.18)
        # How long the box will yield before it gives up and sweeps on. Long
        # enough for the reflex to hold its 1 s, mark the cell and replan
        # (mark_period 1.5 s, recovery_after 2.5 s in amr_control), short enough
        # that a frozen box cannot wall the robot in forever.
        self.declare_parameter("hold_max_s", 6.0)

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
        self._guard_lat = float(gp("guard_lateral").value)
        self._hold_max = float(gp("hold_max_s").value)
        rate = float(gp("rate_hz").value)

        self._warned = False
        # Where the box stopped and when; None when it is sweeping. While it is
        # set, the box does not move (see _tick).
        self._held_xy: tuple[float, float] = (0.0, 0.0)
        self._held_at: float | None = None
        # Simulation time of the first plan; the sweep phase is measured from
        # here. None until then, so the box waits parked at y_min.
        self._t0: float | None = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        # The A* node publishes the plan a few ms after it receives a goal, so
        # the first plan is a stand-in for "the run has started". See the module
        # docstring for why this is not /goal_pose.
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"mover '{self._model}' sweeping {self._axis} "
            f"{self._lo:.1f}..{self._hi:.1f} every {self._period:.1f} s, "
            f"phase anchored to the first plan (+{self._offset:.1f} s), "
            f"yielding within {self._guard:.2f} m and {self._guard_lat:.2f} m "
            f"off the lane for up to {self._hold_max:.0f} s"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_plan(self, _msg: Path) -> None:
        if self._t0 is None:
            self._t0 = self._now()
            self.get_logger().info(
                f"first plan at sim {self._t0:.1f} s: "
                f"anchoring the sweep phase here"
            )

    def _robot_near(self, px: float, py: float) -> bool:
        """True when the robot is close *and* the box is in its way.

        The lateral gate is what the sweep axis is measured on: the box travels
        across the robot's route, so its distance along that axis from the robot
        is how far off the lane it is.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except Exception:
            return False
        t = tf.transform.translation
        lateral = (t.y - py) if self._axis == "y" else (t.x - px)
        if abs(lateral) > self._guard_lat:
            return False
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

        if self._held_at is not None:
            # Keep it where it stopped: testing the *scheduled* position instead
            # meant the box resumed as soon as the phase carried that scheduled
            # point out of the corridor, so the hold lasted under a second and
            # the reflex never reached its 1 s hold, never marked the cell and
            # never replanned. A real obstacle that has stopped does not
            # un-freeze on a timer.
            if (
                self._now() - self._held_at < self._hold_max
                and self._robot_near(*self._held_xy)
            ):
                return
            self._held_at = None
            self.get_logger().info("obstacle resumes its sweep")
        elif self._robot_near(px, py):
            self._held_xy = (px, py)
            self._held_at = self._now()
            self.get_logger().info("robot close: obstacle holds position")
            return

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