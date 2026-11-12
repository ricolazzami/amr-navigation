"""The dynamic scenario is only repeatable if the sweep phase is pinned.

These assert the property the dynamic test relies on: with the shipped
``obstacle_mover.yaml`` numbers, the box is in the robot's lane at the moment
the robot arrives at x = 0.5 (measured at 22.0-23.4 sim s after the plan).

Runs on the container (``rclpy``); skipped on a bare host.
"""

import math
import pathlib

import pytest

pytest.importorskip("rclpy")

from amr_bringup.obstacle_mover import sweep_value  # noqa: E402

LO, HI, PERIOD, OFFSET = -4.8, -3.4, 9.0, 7.0
LANE_Y = -4.33          # the y the A* route uses on the east leg
# Measured from the probe CSVs: the robot crosses x = 0.5 at 22.0-23.4 sim s
# after the plan (two unpinned runs: +25.6 s and +27.0 s of log, plan published
# ~3.6 sim s into the log). The pinned run is the outlier, not the range.
ARRIVAL_S = 22.5


def _lane_crossings() -> list[float]:
    """``dt`` values in the first three goal-relative cycles where y == LANE_Y."""
    out: list[float] = []
    for dt in [x * 0.01 for x in range(0, int(3 * PERIOD * 100))]:
        if abs(sweep_value(dt, OFFSET, PERIOD, LO, HI) - LANE_Y) < 0.02:
            if not out or dt - out[-1] > 0.5:
                out.append(round(dt, 2))
    return out


def test_phase_is_anchored_not_clock_locked():
    # Same dt always gives the same place, whatever the absolute sim clock is.
    assert sweep_value(3.0, OFFSET, PERIOD, LO, HI) == pytest.approx(
        sweep_value(3.0 + 4 * PERIOD, OFFSET, PERIOD, LO, HI)
    )


def test_box_is_parked_until_the_first_plan():
    # offset 0 -> dt 0 is the bottom of the sweep, so a plan anchors a known
    # phase rather than an arbitrary one.
    assert sweep_value(0.0, 0.0, PERIOD, LO, HI) == pytest.approx(LO)


def test_anchor_topic_is_not_goal_pose():
    """A second /goal_pose subscriber steals the match from the A* planner.

    /goal_pose comes from a one-shot `ros2 topic pub --once`, which exits once
    it has matched a subscriber. With this node also subscribed, 2 of 3
    dynamic runs delivered the goal here and not to the planner, and the robot
    never moved. Anchoring hangs off /plan instead.
    """
    src = pathlib.Path(
        __import__("amr_bringup.obstacle_mover", fromlist=["x"]).__file__
    ).read_text()
    body = src.split('"""', 2)[2]      # skip the module docstring, which does
    #                                    discuss /goal_pose and why not to use it
    assert "PoseStamped" not in body          # no /goal_pose subscription
    assert 'create_subscription(Path, "/plan"' in body


def test_box_meets_the_robot_at_the_lane():
    crossings = _lane_crossings()
    assert crossings, "the box never crosses the lane"
    nearest = min(abs(c - ARRIVAL_S) for c in crossings)
    # Within 1.5 s of arrival: the box is still inside the robot's 0.18 m
    # forward corridor (it spans it in ~1.3 s).
    assert nearest <= 1.5, (
        f"closest lane crossing to arrival is {nearest:.2f} s "
        f"(crossings {crossings}); retune phase_offset"
    )


def test_sweep_stays_inside_the_configured_range():
    vals = [sweep_value(d, OFFSET, PERIOD, LO, HI) for d in [x * 0.05 for x in range(400)]]
    assert min(vals) == pytest.approx(LO, abs=1e-6)
    assert max(vals) == pytest.approx(HI, abs=1e-6)


def test_offset_matches_the_shipped_yaml():
    import re

    cfg = pathlib.Path(__file__).parents[1] / "config" / "obstacle_mover.yaml"
    text = cfg.read_text()
    got = {
        k: float(re.search(rf"^\s*{k}:\s*([-\d.]+)", text, re.M).group(1))
        for k in ("y_min", "y_max", "period", "phase_offset")
    }
    assert got == {
        "y_min": LO, "y_max": HI, "period": PERIOD, "phase_offset": OFFSET
    }


def test_lane_crossing_times_are_the_documented_ones():
    # 1.77 s and 7.23 s into a sweep, from 0.5*(1-cos x) = (LANE_Y - LO)/(HI - LO).
    for expected in (1.77, 7.23):
        phase = 2 * math.pi * expected / PERIOD
        y = LO + 0.5 * (1 - math.cos(phase)) * (HI - LO)
        assert y == pytest.approx(LANE_Y, abs=0.005)
