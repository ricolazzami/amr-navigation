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
    mask = blocked_mask(grid, threshold=65, unknown_is_blocked=True)
    mask = inflate(mask, int(math.ceil((0.17 + 0.05) / spec.resolution)))
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--goal", nargs=2, type=float, default=[4.0, 4.0])
    ap.add_argument("--name", default=None)
    args = ap.parse_args()
    if args.name is None:
        args.name = os.path.splitext(os.path.basename(args.csv))[0]

    d = load_csv(args.csv)
    ok = np.isfinite(d["gt_x"]) & np.isfinite(d["gt_y"])
    t = d["t"][ok]
    gt = np.column_stack([d["gt_x"][ok], d["gt_y"][ok]])
    if len(gt) < 2:
        raise SystemExit("not enough ground-truth samples in the CSV")
    t = t - t[0]

    goal = np.array(args.goal)
    dist_goal = np.hypot(*(gt - goal).T)
    success = bool(dist_goal[-1] <= 0.30)
    reached = np.flatnonzero(dist_goal <= 0.20)
    time_to_goal = float(t[reached[0]]) if reached.size else float("nan")

    travelled = float(np.sum(np.hypot(*np.diff(gt, axis=0).T)))

    grid, spec = load_run_map(args.csv)
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

    v = d["out_v"][ok]
    dt = np.diff(t)
    dv = np.diff(v)
    moving = (dt > 1e-3) & (dt < 0.5) & np.isfinite(dv)
    decel = float(np.max(-dv[moving] / dt[moving])) if moving.any() else float("nan")

    out = {
        "run": args.name,
        "success": success,
        "time_to_goal_s": round(time_to_goal, 1),
        "travelled_m": round(travelled, 2),
        "astar_optimal_m": round(opt, 2) if opt else None,
        "path_ratio": round(travelled / opt, 2) if opt else None,
        "min_clearance_m": round(min_clearance(gt, grid, spec), 2),
        "max_decel_mps2": round(decel, 2),
        "rmse_raw_m": round(rmse["raw"], 3),
        "rmse_ekf_m": round(rmse["ekf"], 3),
        "samples": int(len(gt)),
        "duration_s": round(float(t[-1]), 1),
    }
    print(json.dumps(out, indent=2))
    print(
        f"| {args.name} | {'yes' if success else 'NO'} | {out['time_to_goal_s']} | "
        f"{out['travelled_m']} / {out['astar_optimal_m']} ({out['path_ratio']}) | "
        f"{out['min_clearance_m']} | {out['max_decel_mps2']} | "
        f"{out['rmse_raw_m']} / {out['rmse_ekf_m']} |"
    )


if __name__ == "__main__":
    main()