#!/usr/bin/env python3
"""Rasterise the Gazebo maze into a Nav2 occupancy map.

    python3 scripts/make_map.py

The worlds/maze.sdf boxes are axis-aligned, so the mapping is exact: every box
becomes an occupied rectangle. The result is written next to the bringup
package (maps/maze.pgm + maps/maze.yaml) in the **world frame**, so the launch
files can use one map: origin (-5.5, -5.5), 11 m x 11 m at 0.05 m, which puts
world (4, 4) at map (4, 4) too. That keeps the goal the README quotes literal.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
WORLD = os.path.join(ROOT, "src", "amr_bringup", "worlds", "maze.sdf")
OUT_DIR = os.path.join(ROOT, "src", "amr_bringup", "maps")

RES = 0.05
ORIGIN = (-5.5, -5.5)
SIZE = 11.0
SKIP = {"ground_plane", "mover"}  # no floor, no dynamic obstacle

FREE = 254
OCCUPIED = 0
UNKNOWN = 205

# Nav2 free/occupied thresholds the yaml mirrors.
OCC_THRESH = 0.65


def boxes(path):
    """Yield (x, y, sx, sy) for every axis-aligned box model in the world."""
    root = ET.parse(path).getroot()
    world = root.find("world")
    for model in world.findall("model"):
        if model.get("name") in SKIP:
            continue
        pose = model.find("pose")
        if pose is None:
            px = py = 0.0
        else:
            parts = [float(v) for v in pose.text.split()]
            px, py = parts[0], parts[1]
            if abs(parts[3]) > 1e-6 or abs(parts[4]) > 1e-6 or abs(parts[5]) > 1e-6:
                raise SystemExit(f"model {model.get('name')} is not axis aligned")
        for box in model.iter("box"):
            sx, sy, _ = [float(v) for v in box.find("size").text.split()]
            yield px, py, sx, sy


def main() -> None:
    width = int(round(SIZE / RES))
    height = width
    ox, oy = ORIGIN

    # pixel value per cell, row 0 = bottom (low y) while filling; flipped on write
    grid = [[UNKNOWN] * width for _ in range(height)]

    # free everywhere the robot is expected to drive (inside the perimeter)
    for gy in range(height):
        y = oy + (gy + 0.5) * RES
        for gx in range(width):
            x = ox + (gx + 0.5) * RES
            if abs(x) < 4.95 and abs(y) < 4.95:
                grid[gy][gx] = FREE

    for px, py, sx, sy in boxes(WORLD):
        x0, x1 = px - sx / 2.0, px + sx / 2.0
        y0, y1 = py - sy / 2.0, py + sy / 2.0
        gx0 = max(0, int((x0 - ox) / RES))
        gx1 = min(width - 1, int((x1 - ox) / RES))
        gy0 = max(0, int((y0 - oy) / RES))
        gy1 = min(height - 1, int((y1 - oy) / RES))
        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                grid[gy][gx] = OCCUPIED

    os.makedirs(OUT_DIR, exist_ok=True)
    pgm = os.path.join(OUT_DIR, "maze.pgm")
    with open(pgm, "wb") as f:
        f.write(b"P5\n")
        f.write(f"# make_map.py from {os.path.basename(WORLD)}\n".encode())
        f.write(f"{width} {height}\n255\n".encode())
        # nav2 map_server expects the image top row to be the highest y
        for gy in range(height - 1, -1, -1):
            f.write(bytes(grid[gy]))

    yaml = os.path.join(OUT_DIR, "maze.yaml")
    with open(yaml, "w") as f:
        f.write(
            "image: maze.pgm\n"
            f"resolution: {RES}\n"
            f"origin: [{ox}, {oy}, 0.0]\n"
            "negate: 0\n"
            f"occupied_thresh: {OCC_THRESH}\n"
            "free_thresh: 0.196\n"
        )

    free = sum(row.count(FREE) for row in grid)
    occ = sum(row.count(OCCUPIED) for row in grid)
    print(f"wrote {pgm} ({width}x{height}) free={free} occupied={occ}")
    print(f"wrote {yaml} origin=({ox},{oy}) res={RES}")


if __name__ == "__main__":
    main()