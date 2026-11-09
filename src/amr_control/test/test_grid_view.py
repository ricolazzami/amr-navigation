"""Unit tests for the costmap corridor query (no ROS runtime needed)."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from amr_control.grid_view import CostmapView


def make_msg(values: list[int], width: int, height: int, res: float = 0.1):
    """Minimal stand-in for nav_msgs/OccupancyGrid."""
    info = SimpleNamespace(
        width=width,
        height=height,
        resolution=res,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
    )
    header = SimpleNamespace(
        frame_id="odom", stamp=SimpleNamespace(sec=0, nanosec=0)
    )
    return SimpleNamespace(info=info, header=header, data=values)


def grid_with(cells: dict[tuple[int, int], int], width=20, height=20) -> CostmapView:
    data = np.zeros(width * height, dtype=np.int8)
    for (gx, gy), value in cells.items():
        data[gy * width + gx] = value
    view = CostmapView()
    view.update(make_msg(data.tolist(), width, height))
    return view


def test_clear_corridor_returns_none():
    view = grid_with({})
    assert view.nearest_in_corridor(0.5, 0.5, 1.0, 0.0) is None


def test_ignores_obstacles_behind_and_to_the_side():
    # obstacle behind the robot, obstacle 0.45 m to the left
    view = grid_with({(2, 5): 100, (12, 9): 100})
    assert view.nearest_in_corridor(0.5, 0.5, 1.0, 0.0) is None


def test_picks_the_nearest_obstacle_ahead():
    # 0.55 m ahead at y=0.05, and 0.75 m ahead
    view = grid_with({(10, 5): 100, (12, 5): 100})
    result = view.nearest_in_corridor(0.5, 0.5, 1.0, 0.0)
    assert result is not None
    distance, wx, wy = result
    assert distance == pytest.approx(0.55, abs=0.02)
    assert (wx, wy) == pytest.approx((1.05, 0.55), abs=0.02)


def test_ignores_low_cost_cells_below_threshold():
    # inflation leftovers (cost 50) must not count as a physical obstacle
    view = grid_with({(10, 5): 50})
    assert view.nearest_in_corridor(0.5, 0.5, 1.0, 0.0) is None
    view = grid_with({(10, 5): 99})
    assert view.nearest_in_corridor(0.5, 0.5, 1.0, 0.0) is not None


def test_corridor_follows_the_robot_heading():
    # obstacle straight ahead only when facing +y (yaw = 90 deg)
    view = grid_with({(5, 12): 100})
    assert view.nearest_in_corridor(0.5, 0.5, 1.0, 0.0) is None
    result = view.nearest_in_corridor(0.5, 0.5, 0.0, 1.0)
    assert result is not None
    assert result[0] == pytest.approx(0.75, abs=0.02)


def test_respects_scan_radius():
    view = grid_with({(19, 5): 100})  # ~1.45 m ahead
    assert view.nearest_in_corridor(0.5, 0.5, 1.0, 0.0, radius=1.0) is None
    assert view.nearest_in_corridor(0.5, 0.5, 1.0, 0.0, radius=2.0) is not None


def test_empty_view_is_safe():
    view = CostmapView()
    assert view.nearest_in_corridor(0.0, 0.0, 1.0, 0.0) is None
