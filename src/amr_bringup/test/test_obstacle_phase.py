"""The dynamic scenario is only repeatable if the sweep is pinned and the
obstacle actually ends up in the robot's way.

Runs on the container (``rclpy``); skipped on a bare host.
"""

import math
import pathlib
import re

import pytest

pytest.importorskip("rclpy")

from amr_bringup.obstacle_mover import sweep_value  # noqa: E402

LO, HI, PERIOD = -4.8, -3.4, 9.0
OFFSET = 0.7
GUARD_R = 0.75
GUARD_LAT = 0.18        # == amr_control's corridor_half_width
LANE_Y = -4.33          # the y the A* route uses on the east leg
ROBOT_Y = -4.20         # measured, on the east leg
# Measured from the probe CSVs: the robot comes inside the 0.75 m guard radius at
# ~19.4 s after the first plan and crosses x = 0.5 at 22.0-22.2 s (was
# 22.0-23.4 before the phase was anchored to the plan, and arbitrary before that).
FREEZE_S = 19.4
ARRIVAL_S = 22.1
YAML = pathlib.Path(__file__).parents[1] / "config" / "obstacle_mover.yaml"


def _box_y(dt: float) -> float:
    return sweep_value(dt, OFFSET, PERIOD, LO, HI)


def _crossings(level: float, spans: float = 3.0) -> list[float]:
    """``dt`` values in the first few cycles where the box reaches ``level``."""
    out: list[float] = []
    for step in range(int(spans * PERIOD * 100)):
        dt = step * 0.01
        if abs(_box_y(dt) - level) < 0.02:
            if not out or dt - out[-1] > 0.5:
                out.append(round(dt, 2))
    return out


def test_phase_is_anchored_not_clock_locked():
    # Same dt always gives the same place, whatever the absolute sim clock is -
    # the property that makes the encounter repeatable at any real-time factor.
    assert _box_y(3.0) == pytest.approx(_box_y(3.0 + 4 * PERIOD))


def test_box_is_parked_until_the_first_plan():
    # offset 0 -> dt 0 is the bottom of the sweep, so a plan anchors a known
    # phase rather than an arbitrary one, and parking at y_min is continuous
    # with the phase the anchor starts from.
    assert sweep_value(0.0, 0.0, PERIOD, LO, HI) == pytest.approx(LO)


def test_anchor_topic_is_not_goal_pose():
    """A second /goal_pose subscriber steals the match from the A* planner.

    /goal_pose comes from a one-shot `ros2 topic pub --once`, which exits once
    it has matched a subscriber. With this node also subscribed, 2 of 3 dynamic
    runs delivered the goal here and not to the planner, and the robot never
    moved. The anchor hangs off /plan instead.
    """
    src = pathlib.Path(
        __import__("amr_bringup.obstacle_mover", fromlist=["x"]).__file__
    ).read_text()
    body = src.split('"""', 2)[2]   # the module docstring does discuss
    #                                 /goal_pose, and why not to use it
    assert "PoseStamped" not in body
    assert 'create_subscription(Path, "/plan"' in body


def test_box_is_in_the_corridor_when_the_guard_fires():
    """The freeze has to land where the reflex can see it.

    With the old radius-only guard the box froze at the bottom of the sweep,
    ~0.6 m off the lane and outside the 0.18 m corridor: obs_d stayed inf and
    the run tested nothing. It must be inside the corridor at the freeze
    instant, and stay inside across the +/-0.3 s uncertainty in that instant.
    """
    for dt in (FREEZE_S - 0.3, FREEZE_S, FREEZE_S + 0.3):
        off_lane = abs(_box_y(dt) - ROBOT_Y)
        assert off_lane <= GUARD_LAT, (
            f"at t - t_plan = {dt:.1f} s the box is {off_lane:.2f} m off the "
            f"lane, outside the {GUARD_LAT} m corridor; retune phase_offset"
        )


def test_the_guard_lateral_is_the_reflex_corridor():
    """Both nodes must mean the same thing by "in the robot's way"."""
    avoidance = YAML.parent / "avoidance.yaml"
    got = float(
        re.search(r"corridor_half_width:\s*([\d.]+)", avoidance.read_text()).group(1)
    )
    assert got == pytest.approx(GUARD_LAT)


def test_box_reaches_the_lane_as_the_guard_starts_holding_it():
    # A lane crossing ~1 s either side of the freeze instant: the box is at (or
    # just short of) the lane exactly when the guard stops it, which is what
    # puts it in the corridor rather than at the bottom of the sweep.
    crossings = _crossings(LANE_Y)
    assert crossings, "the box never crosses the lane"
    assert [c for c in crossings if FREEZE_S - 1.0 <= c <= ARRIVAL_S], (
        f"no lane crossing near the freeze ({FREEZE_S} s); crossings {crossings}"
    )


def test_sweep_stays_inside_the_configured_range():
    vals = [_box_y(step * 0.05) for step in range(400)]
    assert min(vals) == pytest.approx(LO, abs=1e-6)
    assert max(vals) == pytest.approx(HI, abs=1e-6)


def test_lane_crossing_times_are_the_documented_ones():
    # 1.77 s and 7.23 s into a sweep, from 0.5*(1-cos x) = (LANE_Y - LO)/(HI - LO).
    for expected in (1.77, 7.23):
        phase = 2 * math.pi * expected / PERIOD
        y = LO + 0.5 * (1 - math.cos(phase)) * (HI - LO)
        assert y == pytest.approx(LANE_Y, abs=0.005)


def test_values_match_the_shipped_yaml():
    text = YAML.read_text()
    got = {}
    for key in ("y_min", "y_max", "period", "phase_offset",
                "guard_radius", "guard_lateral"):
        got[key] = float(re.search(rf"^\s*{key}:\s*([-\d.]+)", text, re.M).group(1))
    assert got == {
        "y_min": LO,
        "y_max": HI,
        "period": PERIOD,
        "phase_offset": OFFSET,
        "guard_radius": GUARD_R,
        "guard_lateral": GUARD_LAT,
    }
