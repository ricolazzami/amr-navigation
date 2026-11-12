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

``phase_offset`` places the box in the lane at roughly the moment the robot gets
there. Measured: the robot crosses x = 0.5 at 22.0-22.2 sim s after the first
plan (22.0-23.4 before, and arbitrary before that), and it comes inside the
0.75 m guard radius at ~19.4 s. 0.7 s puts the box 0.02 m off the lane at that
instant (and within 0.14 m of it across a +/-0.3 s spread in the trigger).

Phase alone is not enough, though. Three probe runs put the robot at x = 0.5
between 71.5 and 74.9 sim s, and the box is only inside the 0.18 m corridor for
a slice of its 9 s period, so on some runs the sweep and the arrival simply miss
each other: the box crossed in front of the robot for 0.6 s (shorter than the
reflex's 1 s hold) and swept on, or reached the lane only *after* the robot had
driven past it - "resumes its sweep after 1.89 sim s: robot (+1.26, -4.19) box
(0.5, -4.22)", the box yielding 0.76 m *behind* the robot. Neither tests
avoidance.

So the sweep is not trusted to collide with the robot. ``guard_lead`` is a
one-shot look-ahead: the first time the robot comes within that distance of the
crossing line, the box is placed in the robot's lane a little ahead of it and
holds until the robot has passed. The encounter is then the same every run - a
box squarely in the corridor that the reflex has to hold for, mark and replan
around - while the sweep still provides the dynamic motion either side of it.
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


def lead_triggered(along: float | None, cross_line: float, lead: float) -> bool:
    """Is the robot inside the one-shot look-ahead band before the crossing?

    Only *before* the line: a robot already past the box must not have it staged
    behind it, which is what happened when the yield was left to the sweep.
    """
    return along is not None and 0.0 < (cross_line - along) <= lead


def stage_target(
    axis: str, x: float, lo: float, hi: float, robot_x: float, robot_y: float
) -> tuple[float, float]:
    """Where to stage the box so it stands in the robot's lane, ahead of it.

    The lane is the robot's own coordinate on the sweep axis - which *is* the
    point: the box is placed where the robot is going, rather than wherever the
    sweep happens to be when the robot arrives. Clamped to the configured sweep
    so it is never staged outside the world it moves in.
    """
    lane = robot_y if axis == "y" else robot_x
    lane = min(max(lane, lo), hi)
    return (x, lane) if axis == "y" else (lane, 0.0)


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
        # The one-shot look-ahead that makes the encounter repeatable: the first
        # time the robot comes this far from the box's crossing line, the box is
        # placed in the robot's lane ahead of it instead of relying on the sweep
        # to coincide with the robot's arrival.
        self.declare_parameter("guard_lead", 1.2)
        # The staged box holds until the robot is this far past the crossing
        # line, so the release is "the robot has gone by", not a timer. See the
        # module docstring for why the sweep alone is not enough.
        self.declare_parameter("release_margin", 0.5)
        # How long the box will yield before it gives up and sweeps on. Long
        # enough for the reflex to hold its 1 s, mark the cell and replan
        # (mark_period 1.5 s, recovery_after 2.5 s in amr_control), short enough
        # that a frozen box cannot wall the robot in forever.
        self.declare_parameter("hold_max_s", 6.0)
        # Same cap for the plain proximity hold, which exists only to stop the
        # teleported box overlapping the robot and is not meant to create a
        # second encounter: short, so a robot nudging past a swept-in box is
        # released quickly.
        self.declare_parameter("hold_max_near_s", 2.0)

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
        self._lead = float(gp("guard_lead").value)
        self._release_margin = float(gp("release_margin").value)
        self._hold_max = float(gp("hold_max_s").value)
        self._hold_max_near = float(gp("hold_max_near_s").value)
        rate = float(gp("rate_hz").value)

        # The robot crosses the sweep's line, so its coordinate that matters is
        # the one perpendicular to the sweep axis.
        self._cross_axis = "x" if self._axis == "y" else "y"
        self._cross_line = self._x if self._cross_axis == "x" else 0.0

        self._warned = False
        # Where the box stopped and when; None when it is sweeping. While it is
        # set, the box does not move (see _tick).
        self._held_xy: tuple[float, float] = (0.0, 0.0)
        self._held_at: float | None = None
        # "lead" is the one-shot staged encounter, "near" the plain proximity
        # hold; they release differently (by progress vs by distance).
        self._hold_kind = "near"
        # The staged encounter happens once. A second one would just be the
        # sweep crossing the corridor again, and the robot is already around it.
        self._froze_once = False
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
            f"phase anchored to the first plan (+{self._offset:.1f} s); "
            f"stages in the lane {self._lead:.1f} m before the crossing line "
            f"({self._cross_axis} = {self._cross_line:.2f}) and yields for up "
            f"to {self._hold_max:.0f} s, or holds within {self._guard:.2f} m "
            f"and {self._guard_lat:.2f} m off the lane for "
            f"{self._hold_max_near:.0f} s"
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

    def _robot_pos(self) -> tuple[float, float] | None:
        """The robot's map pose from TF, or None while the transform is missing."""
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            ).transform.translation
        except Exception:
            return None
        return t.x, t.y

    def _robot_offset(self, px: float, py: float) -> tuple[float, float] | None:
        """``(distance, off-lane offset)`` from the robot to the box, or None.

        The lateral term is measured on the sweep axis: the box travels across
        the robot's route, so its distance along that axis from the robot is how
        far off the lane it is.
        """
        pos = self._robot_pos()
        if pos is None:
            return None
        t_x, t_y = pos
        lateral = (t_y - py) if self._axis == "y" else (t_x - px)
        return math.hypot(t_x - px, t_y - py), lateral

    def _robot_near(self, px: float, py: float) -> bool:
        """True when the robot is close *and* the box is in its way."""
        off = self._robot_offset(px, py)
        if off is None:
            return False
        distance, lateral = off
        return abs(lateral) <= self._guard_lat and distance < self._guard

    def _scheduled_xy(self) -> tuple[float, float]:
        """Where the sweep wants the box right now (parked at ``lo`` pre-goal)."""
        if self._t0 is None:
            value = self._lo
        else:
            value = sweep_value(
                self._now() - self._t0, self._offset, self._period,
                self._lo, self._hi,
            )
        return (value, 0.0) if self._axis == "x" else (self._x, value)

    def _set_pose(self, px: float, py: float) -> None:
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

    def _release(self, held_for: float, cap: float, why: str) -> None:
        """Log why a hold ended.

        It matters: the reflex can only mark a cell if the box stays put long
        enough to pin it, so a hold that ends without the robot having stopped
        is the difference between a run that tests avoidance and one that does
        not.
        """
        pos = self._robot_pos()
        where = (
            "robot went out of TF" if pos is None
            else f"robot ({pos[0]:+.2f}, {pos[1]:+.2f}) "
                 f"box ({self._held_xy[0]:.2f}, {self._held_xy[1]:.2f})"
        )
        self.get_logger().info(
            f"obstacle resumes its sweep after {held_for:.2f} sim s "
            f"(of {cap:.1f} max, {why}): {where}"
        )

    def _tick(self) -> None:
        if self._held_at is not None:
            # Keep it where it stopped: testing the *scheduled* position instead
            # meant the box resumed as soon as the phase carried that scheduled
            # point out of the corridor, so the hold lasted under a second and
            # the reflex never reached its 1 s hold, never marked the cell and
            # never replanned. A real obstacle that has stopped does not
            # un-freeze on a timer.
            held_for = self._now() - self._held_at
            if self._hold_kind == "lead":
                # The staged box yields until the robot is past it: "out of the
                # way" is progress along the crossing axis, not a radius, because
                # the robot may steer sideways around it rather than away.
                pos = self._robot_pos()
                along = None if pos is None else (
                    pos[0] if self._cross_axis == "x" else pos[1]
                )
                done = (
                    along is not None
                    and along >= self._cross_line + self._release_margin
                )
                cap, why = self._hold_max, (
                    "robot past the crossing" if done else "hold cap reached"
                )
            else:
                # The proximity hold is only there so the teleported box cannot
                # overlap the robot; it needs no radius-or-lateral subtlety
                # beyond "the robot is still in it".
                off = self._robot_offset(*self._held_xy)
                done = off is not None and (
                    abs(off[1]) > self._guard_lat or off[0] > self._guard
                )
                cap, why = self._hold_max_near, (
                    "robot clear of it" if done else "hold cap reached"
                )
            if held_for < cap and not done:
                return
            self._held_at = None
            self._release(held_for, cap, why)

        if self._t0 is not None:
            pos = self._robot_pos()
            along = None if pos is None else (
                pos[0] if self._cross_axis == "x" else pos[1]
            )
            # One-shot look-ahead: stage the box in the robot's lane instead of
            # waiting for the sweep to coincide with the arrival (module
            # docstring). Once. After that only the proximity hold applies.
            if (
                not self._froze_once
                and lead_triggered(along, self._cross_line, self._lead)
                and pos is not None
            ):
                self._held_xy = stage_target(
                    self._axis, self._x, self._lo, self._hi, pos[0], pos[1]
                )
                self._held_at = self._now()
                self._hold_kind = "lead"
                self._froze_once = True
                self.get_logger().info(
                    f"robot {self._cross_line - along:.2f} m before the "
                    f"crossing: obstacle stages in the lane at "
                    f"({self._held_xy[0]:.2f}, {self._held_xy[1]:.2f}) and holds"
                )
                self._set_pose(*self._held_xy)
                return
            scheduled = self._scheduled_xy()
            if self._robot_near(*scheduled):
                self._held_xy = scheduled
                self._held_at = self._now()
                self._hold_kind = "near"
                self.get_logger().info("robot close: obstacle holds position")
                return

        self._set_pose(*self._scheduled_xy())


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