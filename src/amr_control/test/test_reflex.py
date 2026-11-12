"""Unit tests for the reflex speed law and the hold gate (no ROS runtime).

The numbers are the ones in `src/amr_bringup/config/avoidance.yaml`.
"""

from __future__ import annotations

import math

import pytest

from amr_control.reflex import holds_robot, ramp_speed

V_MAX, D_SAFE, K, D_MIN, FRACTION = 0.26, 0.30, 3.0, 0.15, 0.15


def ramp(d: float) -> float:
    return ramp_speed(d, v_max=V_MAX, d_safe=D_SAFE, k=K, d_min=D_MIN)


def holds(d: float) -> bool:
    return holds_robot(d, v_max=V_MAX, d_safe=D_SAFE, k=K, d_min=D_MIN, fraction=FRACTION)


def test_ramp_is_the_tanh_law():
    d = D_SAFE + 0.1
    assert ramp(d) == pytest.approx(V_MAX * math.tanh(K * (d - D_SAFE)))
    assert ramp(1.0) > 0.95 * V_MAX
    assert ramp(100.0) == pytest.approx(V_MAX)


def test_ramp_is_zero_at_or_inside_the_margins():
    # Nothing is surrendered until d_min; between d_min and d_safe the law is
    # already asking for a negative speed, which clamps to a full stop.
    assert ramp(D_MIN) == 0.0
    assert ramp(D_MIN / 2) == 0.0
    assert ramp((D_MIN + D_SAFE) / 2) == 0.0


def test_ramp_never_exceeds_v_max():
    for d in (0.0, 0.29, 0.31, 0.5, 2.0, 50.0):
        assert 0.0 <= ramp(d) <= V_MAX


def test_holds_robot_covers_the_measured_standoffs():
    # Probe minima over three dynamic runs, against safe_distance 0.30. All
    # three have to mark, not one of three.
    for d in (0.294, 0.308, 0.313, 0.337):
        assert holds(d), d


def test_holds_robot_is_false_when_the_robot_is_still_being_driven():
    assert not holds(0.50)
    assert not holds(1.00)


def test_hold_threshold_is_the_ramp_at_the_fraction():
    # The gate must be about the speed law, not a hand-tuned distance.
    d_edge = D_SAFE + math.atanh(FRACTION) / K
    assert holds(d_edge - 1e-3)
    assert not holds(d_edge + 1e-3)
