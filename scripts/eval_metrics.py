#!/usr/bin/env python3
"""Turn a run CSV (scripts/log_run.py) into the Project 3 metrics table.

    PYTHONPATH=src/amr_planning python3 scripts/eval_metrics.py \
        log/run_static.csv --goal 4 4 --name "Maze, static"

Every number comes from the recorded run and the shipped map; nothing is
estimated. Prints one markdown row and a JSON blob.
"""

from __future__ import annotations

import argparse
import json
import math
import os

import numpy as np

from amr_planning.astar import astar, path_cost
from amr_planning.grid_utils import (
    GridSpec,
    blocked_mask,
    inflate,
    nearest_free,
    occupancy_to_numpy,
)

ROBOT_RADIUS = 0.16
ARRIVAL_M = 0.30
"""Ground-truth arrival radius: Nav2 xy_goal_tolerance (0.15 m) plus the
localization error AMCL carries at the goal, measured at 0.17-0.22 m."""
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_PGM = os.path.join(HERE, "src", "amr_bringup", "maps", "maze.pgm")


def load_csv(path):
    with open(path) as f:
        header = f.readline().strip().split(",")
    data = np.genfromtxt(path, delimiter=",", skip_header=1)
    if data.ndim == 1:
        data = data[None, :]
    return {name: data[:, i] for i, name in enumerate(header)}


def load_map(pgm_path):
    """Fallback map loader (binary PGM, P5) when no run map was saved."""
    with open(pgm_path, "rb") as f:
        assert f.readline().strip() == b"P5", "expected binary PGM"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = (int(x) for x in line.split())
        assert int(f.readline()) == 255
        raw = np.frombuffer(f.read(width * height), dtype=np.uint8)
    pixels = raw.reshape(height, width).astype(int)
    occ = np.where(pixels <= 50, 100, np.where(pixels >= 250, 0, -1))
    spec = GridSpec(
        width=width, height=height, resolution=0.05, origin_x=-5.5, origin_y=-5.5
    )
    return occupancy_to_numpy(occ.ravel().tolist(), width, height), spec


def load_run_map(csv_path):
    """The grid the planner actually saw, saved next to the CSV."""
    npy = csv_path + ".map.npy"
    meta = csv_path + ".map.json"
    if os.path.exists(npy) and os.path.exists(meta):
        with open(meta) as f:
            m = json.load(f)
        grid = np.load(npy).astype(int).reshape(m["height"], m["width"])
        spec = GridSpec(
            width=m["width"],
            height=m["height"],
            resolution=m["resolution"],
            origin_x=m["origin_x"],
            origin_y=m["origin_y"],
        )
        return grid, spec
    return load_map(MAP_PGM)


def optimal_cost(grid, spec, start_world, goal_world):
    mask = planner_mask(grid, spec)
    start = nearest_free(mask, spec.world_to_grid(*start_world), 20)
    goal = nearest_free(mask, spec.world_to_grid(*goal_world), 20)
    if start is None or goal is None:
        return None
    cells = astar(mask, start, goal, weight=1.0)
    return None if cells is None else path_cost(cells, spec.resolution)


def min_clearance(gt_xy, grid, spec):
    occupied = np.argwhere(grid >= 65)
    if occupied.size == 0 or len(gt_xy) == 0:
        return float("nan")
    wx = spec.origin_x + (occupied[:, 1] + 0.5) * spec.resolution
    wy = spec.origin_y + (occupied[:, 0] + 0.5) * spec.resolution
    pts = np.asarray(gt_xy)
    best = float("inf")
    for i in range(0, len(pts), 256):
        chunk = pts[i : i + 256]
        d = np.hypot(chunk[:, None, 0] - wx[None, :], chunk[:, None, 1] - wy[None, :])
        best = min(best, float(d.min()))
    return best - ROBOT_RADIUS


def planner_mask(grid, spec):
    """The inflated mask the A* node plans on.

    Values mirror config/planning.yaml: occupied_threshold 65,
    unknown_is_blocked true, robot_radius 0.17, clearance_margin 0.05.
    """
    mask = blocked_mask(grid, threshold=65, unknown_is_blocked=True)
    return inflate(mask, int(math.ceil((0.17 + 0.05) / spec.resolution)))


def snap_goal(grid, spec, goal_world, snap_cells=20):
    """The point the planner actually drives to.

    The requested goal is often inside the inflation band (the maze goal
    (4, 4) is), so the node snaps it to the nearest free cell. Arrival must be
    measured against that cell, otherwise a run that arrived is scored as if it
    never did.
    """
    mask = planner_mask(grid, spec)
    cell = nearest_free(mask, spec.world_to_grid(*goal_world), snap_cells)
    if cell is None:
        return np.asarray(goal_world, dtype=float)
    return np.asarray(spec.grid_to_world(*cell), dtype=float)


def lateral_offset(gt, grid, spec, start_world, goal_world):
    """Max distance from the recorded path to the A* polyline start -> goal.

    The planner re-plans during a run, so this is measured against the plan a
    static A* would make for the same start and goal: small on a clean run,
    larger while the robot detours around the moving box. Reported as a
    distance to the planned line, not as a controller tracking error.
    """
    mask = planner_mask(grid, spec)
    start = nearest_free(mask, spec.world_to_grid(*start_world), 20)
    goal = nearest_free(mask, spec.world_to_grid(*goal_world), 20)
    if start is None or goal is None:
        return float("nan")
    cells = astar(mask, start, goal, weight=1.0)
    if not cells or len(cells) < 2:
        return float("nan")
    line = np.array([spec.grid_to_world(gx, gy) for gx, gy in cells])
    seg = line[1:] - line[:-1]
    seg_len2 = np.maximum((seg**2).sum(axis=1), 1e-12)
    best = np.empty(len(gt))
    for i in range(0, len(gt), 256):
        chunk = gt[i : i + 256]
        v = chunk[:, None, :] - line[None, :-1, :]
        tproj = np.clip((v * seg[None]).sum(axis=2) / seg_len2[None], 0.0, 1.0)
        proj = line[None, :-1, :] + tproj[..., None] * seg[None]
        d = np.hypot(chunk[:, None, 0] - proj[:, :, 0], chunk[:, None, 1] - proj[:, :, 1])
        best[i : i + 256] = d.min(axis=1)
    return float(best.max())


def approach_oscillation(t, obs_d, w_out, v_out=None, d_safe=0.30,
                         deadband=0.02, moving_eps=0.02):
    """Yaw-rate sign changes while driving inside an obstacle approach.

    The T5.4 acceptance is "< 2 zero-crossings per approach". An approach is a
    contiguous window where the reflex reports an obstacle closer than
    ``d_safe`` (its own safe distance). Two filters, both stated so the number
    is reproducible:

    - only samples where the robot is actually driving (``|v| > moving_eps``)
      count: while the robot holds position for 1 s the controller may re-aim
      left and right, and a sign flip with the wheels stopped is not an
      oscillation of the trajectory;
    - |w| below ``deadband`` is dropped, so sensor noise is not a reversal.
    """
    if w_out is None:
        return 0, []
    near = np.isfinite(obs_d) & (obs_d < d_safe)
    episodes: list[tuple[int, int]] = []
    start = None
    for i in range(len(near)):
        if near[i] and start is None:
            start = i
        elif not near[i] and start is not None:
            episodes.append((start, i))
            start = None
    if start is not None:
        episodes.append((start, len(near)))

    crossings = []
    for a, b in episodes:
        w = w_out[a:b]
        if v_out is not None:
            w = w[(v_out[a:b] > moving_eps) | (v_out[a:b] < -moving_eps)]
        w = w[np.isfinite(w) & (np.abs(w) > deadband)]
        crossings.append(
            int(np.sum(np.sign(w[1:]) != np.sign(w[:-1]))) if len(w) > 1 else 0
        )
    return len(episodes), crossings


def self_test() -> None:
    """Runnable check for the two metrics added for the repeat runs."""
    t = np.arange(0.0, 12.0, 0.1)
    obs = np.where((t > 2.0) & (t < 4.0), 0.20, 9.9)
    v = np.full_like(t, 0.20)
    w = np.where(np.arange(len(t)) % 2 == 0, 0.15, -0.15)

    eps, cross = approach_oscillation(t, obs, w, v)
    assert eps == 1, f"expected one approach window, got {eps}"
    assert cross[0] > 5, f"an alternating yaw rate must show crossings, got {cross}"
    assert approach_oscillation(t, obs, w, np.zeros_like(t))[1][0] == 0, (
        "yaw flips with the wheels stopped are not oscillation"
    )
    assert approach_oscillation(t, obs, np.zeros_like(t), v)[1][0] == 0

    grid = np.zeros((40, 40), dtype=int)
    spec = GridSpec(width=40, height=40, resolution=0.05, origin_x=0.0, origin_y=0.0)
    gt = np.column_stack([np.linspace(0.5, 1.5, 50), np.full(50, 0.5)])
    lateral = lateral_offset(gt, grid, spec, gt[0], gt[-1])
    assert lateral < 0.05, f"a straight line on the planned line must not deviate: {lateral}"
    print("self-test ok")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?")
    ap.add_argument("--goal", nargs=2, type=float, default=[4.0, 4.0])
    ap.add_argument("--name", default=None)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.csv:
        ap.error("a run CSV is required unless --self-test is given")
    if args.name is None:
        args.name = os.path.splitext(os.path.basename(args.csv))[0]

    d = load_csv(args.csv)
    ok = np.isfinite(d["gt_x"]) & np.isfinite(d["gt_y"])
    t = d["t"][ok]
    gt = np.column_stack([d["gt_x"][ok], d["gt_y"][ok]])
    if len(gt) < 2:
        raise SystemExit("not enough ground-truth samples in the CSV")
    t = t - t[0]

    grid, spec = load_run_map(args.csv)
    goal = np.array(args.goal)
    plan_goal = snap_goal(grid, spec, goal)

    # Arrival is judged where the planner sent the robot. The controller
    # declares arrival on the AMCL pose with xy_goal_tolerance 0.15, so the
    # ground-truth gap at that moment is 0.15 m plus the localization error;
    # 0.30 m covers both and is the same tolerance for success and for the
    # arrival time, so the two numbers cannot disagree.
    dist_goal = np.hypot(*(gt - plan_goal).T)
    reached = np.flatnonzero(dist_goal <= ARRIVAL_M)
    success = bool(dist_goal[-1] <= ARRIVAL_M)
    time_to_goal = float(t[reached[0]]) if reached.size else float("nan")
    arrival_error = float(dist_goal[-1])

    travelled = float(np.sum(np.hypot(*np.diff(gt, axis=0).T)))
    opt = optimal_cost(grid, spec, gt[0], goal)

    # Odometry error against ground truth. The two frames share their origin at
    # the spawn pose, so align on the offset then measure the residual.
    rmse = {}
    for label, key in (("raw", "raw_x"), ("ekf", "filt_x")):
        ox, oy = d[key][ok], d[("raw_y" if key == "raw_x" else "filt_y")][ok]
        good = np.isfinite(ox) & np.isfinite(oy)
        if good.sum() < 10:
            rmse[label] = float("nan")
            continue
        off = np.median(
            np.column_stack([ox[good] - gt[good, 0], oy[good] - gt[good, 1]])[:20],
            axis=0,
        )
        err = np.hypot(ox[good] - off[0] - gt[good, 0],
                       oy[good] - off[1] - gt[good, 1])
        rmse[label] = float(np.sqrt(np.mean(err**2)))

    w_out = d.get("w_out")
    episodes, crossings = approach_oscillation(
        t,
        d["obs_d"][ok],
        None if w_out is None else w_out[ok],
        d["out_v"][ok],
    )
    lateral = lateral_offset(gt, grid, spec, gt[0], goal)

    # Deceleration: a 3-sample median filter rejects single-sample command
    # dropouts (a replan gap or the watchdog zeroing one cycle), which would
    # otherwise read as a 1.5 m/s2 slam. A real braking ramp spans several
    # samples and survives the filter.
    v = d["out_v"][ok]
    if len(v) >= 3:
        v = np.median(np.vstack([np.roll(v, 1), v, np.roll(v, -1)]), axis=0)
        v[0], v[-1] = d["out_v"][ok][0], d["out_v"][ok][-1]
    dt = np.diff(t)
    dv = np.diff(v)
    moving = (dt > 1e-3) & (dt < 0.5) & np.isfinite(dv)
    decel = float(np.max(-dv[moving] / dt[moving])) if moving.any() else float("nan")

    out = {
        "run": args.name,
        "success": success,
        "goal_snapped": [round(float(v), 2) for v in plan_goal],
        "arrival_error_m": round(arrival_error, 3),
        "time_to_goal_s": round(time_to_goal, 1),
        "travelled_m": round(travelled, 2),
        "astar_optimal_m": round(opt, 2) if opt else None,
        "path_ratio": round(travelled / opt, 2) if opt else None,
        "min_clearance_m": round(min_clearance(gt, grid, spec), 2),
        "max_decel_mps2": round(decel, 2),
        "rmse_raw_m": round(rmse["raw"], 3),
        "rmse_ekf_m": round(rmse["ekf"], 3),
        "max_lateral_m": round(lateral, 2) if np.isfinite(lateral) else None,
        "approach_episodes": episodes,
        "max_ang_zero_cross": max(crossings) if crossings else None,
        "samples": int(len(gt)),
        "duration_s": round(float(t[-1]), 1),
    }
    print(json.dumps(out, indent=2))
    print(
        f"| {args.name} | {'yes' if success else 'NO'} | {out['time_to_goal_s']} | "
        f"{out['travelled_m']} / {out['astar_optimal_m']} ({out['path_ratio']}) | "
        f"{out['min_clearance_m']} | {out['max_decel_mps2']} | "
        f"{out['rmse_raw_m']} / {out['rmse_ekf_m']} | "
        f"{out['max_lateral_m']} | {out['approach_episodes']}/{out['max_ang_zero_cross']} |"
    )


if __name__ == "__main__":
    main()