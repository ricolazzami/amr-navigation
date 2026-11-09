#!/usr/bin/env python3
"""One-shot diagnostic: dump the SLAM map around the robot as ASCII.

Run inside the container after the stack is up:

    ros2 run ...  # no; just:
    python3 scripts/dump_map.py

It subscribes to /map and the TF map->base_link, then prints a 60x40 window of
the occupancy grid centred on the robot, plus the A* masked view, so you can
see why the planner thinks the robot is walled in.
"""

import math
import sys

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

THRESHOLD = 65
ROBOT_RADIUS = 0.17
MARGIN = 0.05


def inflate(mask: np.ndarray, radius: int) -> np.ndarray:
    out = mask.copy()
    h, w = mask.shape
    ys, xs = np.nonzero(mask)
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            y = np.clip(ys + dy, 0, h - 1)
            x = np.clip(xs + dx, 0, w - 1)
            out[y, x] = True
    return out


class Dump(Node):
    def __init__(self):
        super().__init__("dump_map")
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.grid = None
        self.msg = None

    def on_map(self, msg):
        self.msg = msg
        info = msg.info
        arr = np.array(msg.data, dtype=np.int16).reshape(info.height, info.width)
        self.grid = arr

    def run(self):
        deadline = self.get_clock().now().nanoseconds + 60 * 10**9
        while rclpy.ok() and self.grid is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self.grid is None:
            print("no /map received")
            return
        try:
            for _ in range(100):
                try:
                    tf = self.buf.lookup_transform(
                        "map", "base_link", rclpy.time.Time()
                    )
                    break
                except Exception:
                    rclpy.spin_once(self, timeout_sec=0.1)
            else:
                print("no TF map->base_link")
                return
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except Exception as exc:
            print("no TF map->base_link:", exc)
            return

        info = self.msg.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        gx = int((rx - ox) / res)
        gy = int((ry - oy) / res)

        h, w = self.grid.shape
        mask = self.grid >= THRESHOLD
        mask |= self.grid < 0
        radius = int(math.ceil((ROBOT_RADIUS + MARGIN) / res))
        inf = inflate(mask, radius)

        print(f"map {w}x{h} res {res} origin ({ox:.2f},{oy:.2f})")
        print(f"robot map pose ({rx:.2f},{ry:.2f}) cell ({gx},{gy})")
        print(f"inflated by {radius} cells; R=robot #=occupied/unknown o=inflation .=free")

        win = 34
        x0, x1 = max(0, gx - win), min(w, gx + win + 1)
        y0, y1 = max(0, gy - win), min(h, gy + win + 1)
        for j in range(y1 - 1, y0 - 1, -1):
            row = ""
            for i in range(x0, x1):
                if i == gx and j == gy:
                    row += "R"
                elif self.grid[j, i] >= THRESHOLD:
                    row += "#"
                elif self.grid[j, i] < 0:
                    row += " "
                elif inf[j, i]:
                    row += "o"
                else:
                    row += "."
            print(row)


def main():
    rclpy.init()
    node = Dump()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()