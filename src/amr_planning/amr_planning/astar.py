"""A* on a discrete 2-D occupancy array.

Deliberately 4-connected with a Manhattan heuristic: the brief (slide 16) asks
for exactly that, and Manhattan is only admissible on a 4-way grid. Allowing
diagonals here would quietly break optimality.
"""

from __future__ import annotations

import heapq

import numpy as np

NEIGHBOURS: tuple[tuple[int, int], ...] = ((1, 0), (-1, 0), (0, 1), (0, -1))
"""4-way connectivity, in (dx, dy) cell steps."""


def manhattan(a: tuple[int, int], b: tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(
    mask: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    *,
    weight: float = 1.0,
    max_expansions: int | None = None,
) -> list[tuple[int, int]] | None:
    """Shortest 4-way path from ``start`` to ``goal`` over free cells.

    Parameters
    ----------
    mask:
        ``(height, width)`` boolean array; ``True`` means blocked.
    weight:
        Heuristic multiplier. ``1.0`` is optimal A*. Above 1.0 trades
        optimality for speed (weighted A*), which is how we hit the
        400x400 budget.
    max_expansions:
        Safety valve so a pathological map cannot hang the node.

    Returns
    -------
    List of cells from start to goal inclusive, or ``None`` if unreachable.
    """
    height, width = mask.shape
    sx, sy = start
    gx, gy = goal

    if not (0 <= sx < width and 0 <= sy < height):
        return None
    if not (0 <= gx < width and 0 <= gy < height):
        return None
    if mask[sy, sx] or mask[gy, gx]:
        return None

    # Flat Python lists instead of numpy arrays for the hot loop: every
    # neighbour test touches a scalar, and numpy's scalar boxing dominates a
    # 200x200 search. This is ~1.6x faster with identical results.
    blocked = mask.ravel().tolist()
    size = width * height
    inf = float("inf")
    g_score = [inf] * size
    parent = [-1] * size
    closed = bytearray(size)

    start_idx = sy * width + sx
    goal_idx = gy * width + gx
    g_score[start_idx] = 0.0

    # (f, tie_breaker, flat_index); the counter keeps heap ordering deterministic
    open_heap: list[tuple[float, int, int]] = [
        (weight * manhattan(start, goal), 0, start_idx)
    ]
    counter = 0
    expansions = 0
    heappush = heapq.heappush
    heappop = heapq.heappop

    while open_heap:
        _, _, cur = heappop(open_heap)
        if closed[cur]:
            continue
        if cur == goal_idx:
            return _reconstruct(parent, start_idx, goal_idx, width)

        closed[cur] = 1
        expansions += 1
        if max_expansions is not None and expansions > max_expansions:
            return None

        base = g_score[cur] + 1.0
        cx = cur % width
        cy = cur // width

        # right
        if cx + 1 < width:
            ni = cur + 1
            if not blocked[ni] and not closed[ni] and base < g_score[ni]:
                g_score[ni] = base
                parent[ni] = cur
                counter += 1
                heappush(
                    open_heap,
                    (base + weight * (abs(cx + 1 - gx) + abs(cy - gy)), counter, ni),
                )
        # left
        if cx > 0:
            ni = cur - 1
            if not blocked[ni] and not closed[ni] and base < g_score[ni]:
                g_score[ni] = base
                parent[ni] = cur
                counter += 1
                heappush(
                    open_heap,
                    (base + weight * (abs(cx - 1 - gx) + abs(cy - gy)), counter, ni),
                )
        # down
        if cy + 1 < height:
            ni = cur + width
            if not blocked[ni] and not closed[ni] and base < g_score[ni]:
                g_score[ni] = base
                parent[ni] = cur
                counter += 1
                heappush(
                    open_heap,
                    (base + weight * (abs(cx - gx) + abs(cy + 1 - gy)), counter, ni),
                )
        # up
        if cy > 0:
            ni = cur - width
            if not blocked[ni] and not closed[ni] and base < g_score[ni]:
                g_score[ni] = base
                parent[ni] = cur
                counter += 1
                heappush(
                    open_heap,
                    (base + weight * (abs(cx - gx) + abs(cy - 1 - gy)), counter, ni),
                )

    return None


def _reconstruct(
    parent: list[int],
    start_idx: int,
    goal_idx: int,
    width: int,
) -> list[tuple[int, int]]:
    path: list[tuple[int, int]] = []
    cur = goal_idx
    while cur != start_idx:
        path.append((cur % width, cur // width))
        cur = parent[cur]
    path.append((start_idx % width, start_idx // width))
    path.reverse()
    return path


def path_cost(path: list[tuple[int, int]], resolution: float) -> float:
    """Length of a 4-connected path in metres."""
    if len(path) < 2:
        return 0.0
    return (len(path) - 1) * resolution


def densify(
    path: list[tuple[int, int]], step: int
) -> list[tuple[int, int]]:
    """Insert intermediate cells so no gap between waypoints exceeds ``step``.

    Call ``simplify`` first: between its corners the path is axis-aligned, so
    linear interpolation stays exactly on the original route (no corner is cut).

    Needed because Nav2's controller truncates the path to the local costmap
    window. A path with multi-metre jumps loses everything past the costmap
    edge, leaving a final pose that sits inside the goal tolerance, which makes
    the controller think it has arrived and pivot in place forever.
    """
    if len(path) < 2 or step <= 1:
        return list(path)
    out = [path[0]]
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        dx, dy = x1 - x0, y1 - y0
        span = max(abs(dx), abs(dy))
        pieces = max(1, -(-span // step))  # ceil
        for i in range(1, pieces + 1):
            out.append(
                (x0 + dx * i // pieces, y0 + dy * i // pieces)
            )
    return out


def simplify(path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Drop collinear intermediate cells (keeps the same cells, fewer points)."""
    if len(path) < 3:
        return list(path)
    out = [path[0]]
    for i in range(1, len(path) - 1):
        px, py = out[-1]
        cx, cy = path[i]
        nx, ny = path[i + 1]
        if (cx - px) * (ny - cy) == (cy - py) * (nx - cx):
            continue  # same direction, redundant waypoint
        out.append(path[i])
    out.append(path[-1])
    return out
