"""Read-only view over a Nav2 costmap, with the forward-corridor query.

Kept free of ROS imports so the geometry can be unit-tested on the host.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from amr_planning.grid_utils import occupancy_to_numpy


@dataclass
class CostmapView:
    """Latest costmap, decoded to numpy."""

    grid: np.ndarray | None = None
    frame_id: str = ""
    resolution: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    _stamp_ns: int = field(default=0, repr=False)

    # ------------------------------------------------------------------
    def update(self, msg) -> None:
        info = msg.info
        self.resolution = float(info.resolution)
        self.origin_x = float(info.origin.position.x)
        self.origin_y = float(info.origin.position.y)
        self.frame_id = msg.header.frame_id
        self.grid = occupancy_to_numpy(msg.data, info.width, info.height)
        self._stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        )

    # ------------------------------------------------------------------
    def nearest_in_corridor(
        self,
        robot_x: float,
        robot_y: float,
        cos_yaw: float,
        sin_yaw: float,
        radius: float = 1.5,
        half_width: float = 0.35,
        threshold: int = 99,
    ) -> tuple[float, float, float] | None:
        """Closest obstacle cell inside the forward corridor.

        The corridor is expressed in the robot frame: ``0 < x <= radius`` and
        ``|y| <= half_width``. Returns ``(distance, world_x, world_y)`` with the
        world coordinates expressed in the costmap frame, or ``None`` when the
        corridor is clear.
        """
        if self.grid is None or self.resolution <= 0.0:
            return None

        height, width = self.grid.shape
        res = self.resolution
        gx = int(np.floor((robot_x - self.origin_x) / res))
        gy = int(np.floor((robot_y - self.origin_y) / res))
        span = int(np.ceil(radius / res)) + 1

        x0, x1 = max(0, gx - span), min(width, gx + span + 1)
        y0, y1 = max(0, gy - span), min(height, gy + span + 1)
        if x0 >= x1 or y0 >= y1:
            return None

        window = self.grid[y0:y1, x0:x1]
        ys, xs = np.nonzero(window >= threshold)
        if xs.size == 0:
            return None

        wx = self.origin_x + (xs + x0 + 0.5) * res
        wy = self.origin_y + (ys + y0 + 0.5) * res

        dx = wx - robot_x
        dy = wy - robot_y
        fx = dx * cos_yaw + dy * sin_yaw
        fy = -dx * sin_yaw + dy * cos_yaw

        keep = (fx > 0.0) & (fx <= radius) & (np.abs(fy) <= half_width)
        if not np.any(keep):
            return None

        distances = np.hypot(fx[keep], fy[keep])
        i = int(np.argmin(distances))
        return float(distances[i]), float(wx[keep][i]), float(wy[keep][i])
