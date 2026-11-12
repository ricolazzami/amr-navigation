"""The dynamic scenario is only repeatable if the sweep is pinned and the
obstacle actually ends up in the robot's way.

Runs on the container (``rclpy``); skipped on a bare host.
"""

import math
import pathlib
import re

import pytest

pytest.importorskip("rclpy")

from amr_bringup.obstacle_mover import (  # noqa: E402
    lead_triggered,
    stage_target,
    sweep_value,
)

LO, HI, PERIOD = -4.8, -3.4, 9.0
OFFSET = 0.7
GUARD_R = 0.75
GUARD_LAT = 0.18        # == amr_control's corridor_half_width
GUARD_LEAD = 1.2        # the staged encounter's look-ahead
RELEASE_MARGIN = 0.5
HOLD_MAX = 6.0          # s the box will yield before sweeping on
HOLD_MAX_NEAR = 2.0
LANE_Y = -4.33          # the y the A* route uses on the east leg
YAML = pathlib.Path(__file__).parents[1] / "config" / "obstacle_mover.yaml"


def _box_y(dt: float) -> float:
    return sweep_value(dt, OFFSET, PERIOD, LO, HI)


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


def test_the_guard_lateral_is_the_reflex_corridor():
    """Both nodes must mean the same thing by "in the robot's way"."""
    avoidance = YAML.parent / "avoidance.yaml"
    got = float(
        re.search(r"corridor_half_width:\s*([\d.]+)", avoidance.read_text()).group(1)
    )
    assert got == pytest.approx(GUARD_LAT)


def test_staging_puts_the_box_in_the_robot_lane_not_where_the_sweep_is():
    """The fix for the missed encounters: the box goes to the robot's lane.

    The sweep was timed against the robot's arrival and still missed it on some
    runs (the robot takes 71.5-74.9 sim s to reach x = 0.5 and the box is only in
    the 0.18 m corridor for a slice of its 9 s period). Staging uses the robot's
    own y, so the box is squarely in the lane whatever the phase - and the
    clamping keeps it inside the world it sweeps.
    """
    box = stage_target("y", 0.5, LO, HI, robot_x=-0.7, robot_y=-4.20)
    assert box == pytest.approx((0.5, -4.20))
    # latched, and clamped to the sweep range rather than following the robot off
    # the edge of the world
    assert stage_target("y", 0.5, LO, HI, robot_x=-0.7, robot_y=-9.0)[1] == LO
    assert stage_target("y", 0.5, LO, HI, robot_x=-0.7, robot_y=+9.0)[1] == HI


def test_staging_fires_before_the_crossing_and_only_once():
    """The look-ahead band: in front of the robot, never behind it."""
    line, lead = 0.5, 1.2
    assert lead_triggered(0.5 - 1.1, line, lead)      # approaching, in band
    assert not lead_triggered(0.5 - 1.9, line, lead)  # still too far
    assert not lead_triggered(0.5, line, lead)        # on the line
    assert not lead_triggered(0.5 + 0.1, line, lead)  # already past it
    assert not lead_triggered(None, line, lead)       # no transform yet


def test_the_encounter_is_staged_not_left_to_the_sweep():
    # The sweep is cosmetic now; the encounter hangs off guard_lead. Asserts the
    # wiring, the way test_anchor_topic_is_not_goal_pose does for the anchor.
    src = pathlib.Path(
        __import__("amr_bringup.obstacle_mover", fromlist=["x"]).__file__
    ).read_text()
    body = src.split('"""', 2)[2]
    assert "stage_target(" in body and "lead_triggered(" in body
    assert "self._froze_once" in body


def test_no_parameter_attribute_shadows_a_method():
    """A `self._release = <float>` shadowed `_release()` and killed the node.

    The mover died with `TypeError: 'float' object is not callable` the first
    time a hold ended, which parked the box in the robot's lane for the rest of
    the run. Nothing in a unit test runs `_tick`, so pin the naming instead: no
    attribute assigned in __init__ may share a name with a method.
    """
    src = pathlib.Path(
        __import__("amr_bringup.obstacle_mover", fromlist=["x"]).__file__
    ).read_text()
    methods = set(re.findall(r"^    def ([a-z_]+)\(", src, re.M))
    assigned = set(re.findall(r"^\s+self\.([a-z_]+)\s*=", src, re.M))
    assert not (methods & assigned), (
        f"these attributes shadow methods and would call as a TypeError: "
        f"{sorted(methods & assigned)}"
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
    for key in ("y_min", "y_max", "period", "phase_offset", "guard_radius",
                "guard_lateral", "guard_lead", "release_margin", "hold_max_s",
                "hold_max_near_s"):
        got[key] = float(re.search(rf"^\s*{key}:\s*([-\d.]+)", text, re.M).group(1))
    assert got == {
        "y_min": LO,
        "y_max": HI,
        "period": PERIOD,
        "phase_offset": OFFSET,
        "guard_radius": GUARD_R,
        "guard_lateral": GUARD_LAT,
        "guard_lead": GUARD_LEAD,
        "release_margin": RELEASE_MARGIN,
        "hold_max_s": HOLD_MAX,
        "hold_max_near_s": HOLD_MAX_NEAR,
    }
