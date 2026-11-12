"""The reflex speed law.

Kept out of ``avoidance_node`` because two things need the exact same ramp:
the command scaling in the control loop, and the hold gate that decides whether
the *obstacle* or the *controller* is what stopped the robot. Writing the law
twice is how the two drift apart.
"""

from __future__ import annotations

import math


def ramp_speed(d: float, *, v_max: float, d_safe: float, k: float, d_min: float) -> float:
    """Speed the reflex allows at obstacle distance ``d``.

    ``v_max * tanh(k * (d - d_safe))``: positive but small just outside the
    safety distance, so the robot eases to a stop instead of slamming the
    brakes. Clamped to ``[0, v_max]``, and zero at or inside ``d_min`` where the
    command is surrendered entirely.
    """
    if d <= d_min:
        return 0.0
    return max(0.0, min(v_max, v_max * math.tanh(k * (d - d_safe))))


def holds_robot(
    d: float, *, v_max: float, d_safe: float, k: float, d_min: float, fraction: float
) -> bool:
    """Is the obstacle, rather than the controller, what is holding the robot?

    True when the ramp caps the command below ``fraction`` of ``v_max``. Above
    that the robot is still being driven (or the controller is braking on its
    own) and a standstill is not the obstacle's doing, so it must not be marked.
    """
    return ramp_speed(d, v_max=v_max, d_safe=d_safe, k=k, d_min=d_min) < fraction * v_max
