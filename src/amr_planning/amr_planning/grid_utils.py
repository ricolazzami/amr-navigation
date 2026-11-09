"""Grid maths shared by the planner and the unit tests.

No ROS imports here on purpose: this module must be testable with plain
``pytest`` on the host, without a ROS installation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

OCCUPIED_THRESHOLD = 65
"""Cells at or above this value count as an obstacle (Nav2 convention)."""


@dataclass(frozen=True)
class GridSpec:
    """Geometry of an occupancy grid, mirroring ``nav_msgs/MapMetaData``."""

    width: int
    height: int
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """World metres -> cell indices. Row is y, column is x."""
        return (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        """Cell indices -> world metres at the cell centre."""
        return (
            self.origin_x + (gx + 0.5) * self.resolution,
            self.origin_y + (gy + 0.5) * self.resolution,
        )

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        gx, gy = cell
        return 0 <= gx < self.width and 0 <= gy < self.height


def occupancy_to_numpy(data, width: int, height: int) -> np.ndarray:
    """Flattened ``OccupancyGrid.data`` -> ``(height, width)`` int8 array."""
    arr = np.asarray(data, dtype=np.int8)
    if arr.size != width * height:
        raise ValueError(
            f"grid data has {arr.size} cells, expected {width * height}"
        )
    return arr.reshape(height, width)


def blocked_mask(
    grid: np.ndarray,
    threshold: int = OCCUPIED_THRESHOLD,
    unknown_is_blocked: bool = True,
) -> np.ndarray:
    """Boolean mask of cells the planner may not enter.

    Unknown cells (``-1``) are blocked by default: planning through unseen
    space is how a robot drives into a wall it has not mapped yet.
    """
    mask = grid >= threshold
    if unknown_is_blocked:
        mask |= grid < 0
    return mask


def inflate(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    """Grow obstacles by ``radius_cells`` using a chebyshev (square) kernel.

    This is the planner-side equivalent of Nav2's inflation layer and is what
    keeps the chassis off the wall (slide "The Margin Constraint"). A square
    kernel is used rather than a diamond: it is never smaller than the true
    disc, so corners are cleared conservatively instead of clipped.
    """
    if radius_cells <= 0:
        return mask.copy()
    out = mask.copy()
    for _ in range(radius_cells):
        grown = out.copy()
        # 4-neighbourhood
        grown[1:, :] |= out[:-1, :]
        grown[:-1, :] |= out[1:, :]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        # diagonals, so the kernel is square (chebyshev) rather than diamond
        grown[1:, 1:] |= out[:-1, :-1]
        grown[1:, :-1] |= out[:-1, 1:]
        grown[:-1, 1:] |= out[1:, :-1]
        grown[:-1, :-1] |= out[1:, 1:]
        out = grown
    return out


def nearest_free(
    mask: np.ndarray,
    cell: tuple[int, int],
    max_radius: int = 20,
) -> tuple[int, int] | None:
    """Closest free cell to ``cell``, searching square rings outwards.

    Used when the robot's current cell is blocked (inside inflation, or an
    obstacle was marked on top of it) and when a goal lands on an obstacle.
    """
    height, width = mask.shape
    gx, gy = cell

    def free(x: int, y: int) -> bool:
        return 0 <= x < width and 0 <= y < height and not mask[y, x]

    if free(gx, gy):
        return cell

    for r in range(1, max_radius + 1):
        best = None
        best_d2 = None
        for dx in range(-r, r + 1):
            for dy in (-r, r):
                x, y = gx + dx, gy + dy
                if free(x, y):
                    d2 = dx * dx + dy * dy
                    if best_d2 is None or d2 < best_d2:
                        best, best_d2 = (x, y), d2
        for dy in range(-r + 1, r):
            for dx in (-r, r):
                x, y = gx + dx, gy + dy
                if free(x, y):
                    d2 = dx * dx + dy * dy
                    if best_d2 is None or d2 < best_d2:
                        best, best_d2 = (x, y), d2
        if best is not None:
            return best
    return None
