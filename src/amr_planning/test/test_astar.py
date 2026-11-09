"""Unit tests for the pure A* core.

Runs with plain pytest on the host:

    cd src/amr_planning && python -m pytest test -q
"""

from __future__ import annotations

import heapq
import time

import numpy as np
import pytest

from amr_planning.astar import astar, densify, manhattan, path_cost, simplify
from amr_planning.grid_utils import (
    GridSpec,
    blocked_mask,
    inflate,
    nearest_free,
    occupancy_to_numpy,
)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def dijkstra(mask: np.ndarray, start, goal) -> float:
    """Reference shortest-path cost, used to prove A* optimality."""
    height, width = mask.shape
    dist = {start: 0.0}
    heap = [(0.0, start)]
    seen = set()
    while heap:
        d, (x, y) = heapq.heappop(heap)
        if (x, y) in seen:
            continue
        if (x, y) == goal:
            return d
        seen.add((x, y))
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and not mask[ny, nx]:
                nd = d + 1.0
                if nd < dist.get((nx, ny), np.inf):
                    dist[(nx, ny)] = nd
                    heapq.heappush(heap, (nd, (nx, ny)))
    return np.inf


def assert_path_valid(mask: np.ndarray, path: list[tuple[int, int]]) -> None:
    height, width = mask.shape
    for x, y in path:
        assert 0 <= x < width and 0 <= y < height, f"cell {(x, y)} out of bounds"
        assert not mask[y, x], f"path enters blocked cell {(x, y)}"
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        assert manhattan((ax, ay), (bx, by)) == 1, "path is not 4-connected"


# --------------------------------------------------------------------------
# basic behaviour
# --------------------------------------------------------------------------


def test_manhattan_is_4_way_distance():
    assert manhattan((0, 0), (3, 4)) == 7
    assert manhattan((5, 5), (5, 5)) == 0


def test_straight_line_on_empty_grid():
    mask = np.zeros((20, 20), dtype=bool)
    path = astar(mask, (1, 1), (1, 10))
    assert path is not None
    assert_path_valid(mask, path)
    assert path[0] == (1, 1) and path[-1] == (1, 10)
    assert len(path) - 1 == 9  # pure vertical run, no detour
    assert path_cost(path, 0.05) == pytest.approx(0.45)


def test_detours_around_wall_with_gap():
    mask = np.zeros((15, 15), dtype=bool)
    mask[5:10, 7] = True      # vertical wall, rows 5..9 at column 7
    path = astar(mask, (2, 7), (12, 7))
    assert path is not None
    assert_path_valid(mask, path)
    # must leave the wall through one of its open ends (row 4 or row 10)
    assert any(y in (4, 10) for _, y in path)


def test_unreachable_goal_returns_none():
    mask = np.zeros((12, 12), dtype=bool)
    mask[:, 6] = True  # full-height wall
    assert astar(mask, (1, 1), (10, 10)) is None


def test_blocked_start_or_goal_returns_none():
    mask = np.zeros((10, 10), dtype=bool)
    mask[1, 1] = True
    assert astar(mask, (1, 1), (8, 8)) is None
    mask = np.zeros((10, 10), dtype=bool)
    mask[8, 8] = True
    assert astar(mask, (1, 1), (8, 8)) is None


def test_out_of_bounds_returns_none():
    mask = np.zeros((10, 10), dtype=bool)
    assert astar(mask, (0, 0), (10, 10)) is None


# --------------------------------------------------------------------------
# optimality: A* must match Dijkstra exactly with weight 1.0
# --------------------------------------------------------------------------


@pytest.mark.parametrize("seed", range(20))
def test_matches_dijkstra_on_random_maps(seed):
    rng = np.random.default_rng(seed)
    mask = rng.random((30, 30)) < 0.22
    # keep the corners free so the scenario is solvable
    mask[1, 1] = mask[28, 28] = False
    mask[0:3, 0:3] = False
    mask[27:30, 27:30] = False

    path = astar(mask, (1, 1), (28, 28))
    reference = dijkstra(mask, (1, 1), (28, 28))

    if reference == np.inf:
        assert path is None
        return

    assert path is not None, f"seed {seed}: A* found no path, Dijkstra did"
    assert_path_valid(mask, path)
    assert (len(path) - 1) == reference, f"seed {seed}: suboptimal path"


def test_weighted_astar_is_still_collision_free():
    rng = np.random.default_rng(7)
    mask = rng.random((40, 40)) < 0.2
    mask[1, 1] = mask[38, 38] = False
    path = astar(mask, (1, 1), (38, 38), weight=1.2)
    if path is not None:
        assert_path_valid(mask, path)


# --------------------------------------------------------------------------
# grid helpers
# --------------------------------------------------------------------------


def test_occupancy_values_map_to_blocked_mask():
    data = [0, 100, -1, 65, 64, 0]
    grid = occupancy_to_numpy(data, width=3, height=2)
    mask = blocked_mask(grid, threshold=65, unknown_is_blocked=True)
    assert mask.tolist() == [
        [False, True, True],   # 0 free, 100 occupied, -1 unknown -> blocked
        [True, False, False],  # 65 occupied, 64 free-ish, 0 free
    ]


def test_unknown_can_be_treated_as_free():
    grid = occupancy_to_numpy([-1, 0, 100], width=3, height=1)
    mask = blocked_mask(grid, unknown_is_blocked=False)
    assert mask.tolist() == [[False, False, True]]


def test_inflate_grows_obstacles_by_radius():
    mask = np.zeros((11, 11), dtype=bool)
    mask[5, 5] = True
    grown = inflate(mask, radius_cells=2)
    # square kernel: everything within 2 cells of the centre becomes blocked
    assert grown[3:8, 3:8].all()
    assert not grown[0, 0]
    assert grown.sum() == 25


def test_inflate_with_zero_radius_is_a_copy():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True
    out = inflate(mask, 0)
    assert out.tolist() == mask.tolist()
    out[0, 0] = True
    assert not mask[0, 0], "returned array must not alias the input"


def test_nearest_free_snaps_out_of_obstacle():
    mask = np.zeros((21, 21), dtype=bool)
    mask[9:12, 9:12] = True
    snapped = nearest_free(mask, (10, 10), max_radius=5)
    assert snapped is not None
    assert not mask[snapped[1], snapped[0]]
    assert manhattan(snapped, (10, 10)) == 2  # ring r=2 is the first free one


def test_nearest_free_returns_none_when_walled_in():
    mask = np.ones((5, 5), dtype=bool)
    assert nearest_free(mask, (2, 2), max_radius=3) is None


def test_grid_spec_round_trip():
    spec = GridSpec(width=200, height=200, resolution=0.05, origin_x=-5.0, origin_y=-5.0)
    cell = spec.world_to_grid(0.0, 0.0)
    assert cell == (100, 100)
    wx, wy = spec.grid_to_world(*cell)
    assert (wx, wy) == pytest.approx((0.025, 0.025))
    assert spec.in_bounds(cell)
    assert not spec.in_bounds((200, 0))


def test_simplify_keeps_endpoints_and_shape():
    path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
    assert simplify(path) == [(0, 0), (2, 0), (2, 2)]
    assert simplify([(0, 0), (1, 0)]) == [(0, 0), (1, 0)]


# --------------------------------------------------------------------------
# performance budget from the plan: 200x200 < 100 ms
# --------------------------------------------------------------------------


def test_performance_200x200_under_budget():
    rng = np.random.default_rng(42)
    mask = rng.random((200, 200)) < 0.15
    mask[5:15, 5:15] = False
    mask[185:195, 185:195] = False

    t0 = time.perf_counter()
    path = astar(mask, (10, 10), (190, 190))
    elapsed = time.perf_counter() - t0

    assert path is not None
    assert_path_valid(mask, path)
    print(f"\nA* 200x200: {elapsed * 1000:.1f} ms, {len(path)} cells")
    assert elapsed < 0.100, f"budget exceeded: {elapsed * 1000:.1f} ms"


# --------------------------------------------------------------------------
# densify: no gap may exceed `step`, and the result must stay on the path
# --------------------------------------------------------------------------

def test_densify_no_gap_exceeds_step():
    # A long staircase then a long straight run: the exact shape that made the
    # controller truncate to a pose inside the goal tolerance.
    path = [(0, 0), (5, 0), (5, 40), (60, 40)]
    dense = densify(simplify(path), 2)
    assert dense[0] == path[0] and dense[-1] == path[-1]
    for (x0, y0), (x1, y1) in zip(dense, dense[1:]):
        assert max(abs(x1 - x0), abs(y1 - y0)) <= 2


def test_densify_keeps_geometry_inside_free_cells():
    # Interpolating a simplified (axis-aligned) path must never leave it.
    mask = np.zeros((80, 80), dtype=bool)
    mask[:, 20] = True  # wall with a gap at y 0..9
    mask[0:10, 20] = False
    path = astar(mask, (5, 5), (70, 60))
    assert path is not None
    dense = densify(simplify(path), 3)
    for x, y in dense:
        assert not mask[y, x], f"densified path enters wall at {(x, y)}"


def test_densify_single_cell_and_step_one_are_noops():
    assert densify([(3, 4)], 2) == [(3, 4)]
    p = [(0, 0), (5, 5)]
    assert densify(p, 1) == p
