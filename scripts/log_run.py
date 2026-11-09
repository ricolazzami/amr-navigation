#!/usr/bin/env python3
"""Log one navigation run to CSV so the metrics are measured, not estimated.

    python3 scripts/log_run.py /work/log/run_static.csv 180

Columns: t, gt_x, gt_y, gt_yaw, raw_x, raw_y, filt_x, filt_y,
         cmd_v, out_v, obs_d

gt_*   ground truth in the Gazebo world frame (/world/maze/dynamic_pose/info)
raw_*  raw wheel odometry after the frame relay (/wheel/odom_fixed, odom frame)
filt_* EKF output (/odometry/filtered, odom frame)
cmd_v  what the controller asked for (/cmd_vel_nav.linear.x)
out_v  what actually reached the wheels (/cmd_vel.linear.x)
obs_d  closest obstacle in the forward corridor (/avoidance/distance)

The first row is the header; pass the same file to scripts/eval_metrics.py.
"""

from __future__ import annotations

import math
import json
import sys

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage

COLS = [
    "t", "gt_x", "gt_y", "gt_yaw", "raw_x", "raw_y",
    "filt_x", "filt_y", "cmd_v", "out_v", "obs_d",
]


class RunLog(Node):
    def __init__(self, path: str, period: float):
        super().__init__("run_log")
        self._f = open(path, "w")
        self._f.write(",".join(COLS) + "\n")
        self._gt = None
        self._raw = None
        self._filt = None
        self._cmd_v = float("nan")
        self._out_v = float("nan")
        self._obs_d = float("nan")

        self._map_saved = False
        self._map_path = path + ".map"
        self.create_subscription(
            TFMessage, "/world/maze/dynamic_pose/info", self._on_gt, 10
        )
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(Odometry, "/wheel/odom_fixed", self._on_raw, 20)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_filt, 20)
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_cmd, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_out, 20)
        self.create_subscription(Float32, "/avoidance/distance", self._on_d, 10)
        self.create_timer(period, self._tick)
        self.get_logger().info(f"logging to {path}")

    def _yaw(self, q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def _on_gt(self, msg: TFMessage) -> None:
        # The ROS bridge of /world/maze/dynamic_pose/info drops the model name
        # (child_frame_id is empty), so pick the robot by height: the moving
        # box sits at z=0.25, the robot drives on the floor (z~=0). Once locked
        # on, follow the nearest candidate so a link transform cannot steal it.
        cands = [
            (tr.transform.translation.x, tr.transform.translation.y,
             self._yaw(tr.transform.rotation))
            for tr in msg.transforms
            if tr.transform.translation.z < 0.15
        ]
        if not cands:
            return
        if self._gt is None:
            self._gt = cands[0]
            return
        best = min(cands, key=lambda c: (c[0] - self._gt[0]) ** 2
                   + (c[1] - self._gt[1]) ** 2)
        # A jump above 1 m per message means we latched onto the wrong model.
        if abs(best[0] - self._gt[0]) < 1.0 and abs(best[1] - self._gt[1]) < 1.0:
            self._gt = best

    def _on_raw(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._raw = (p.x, p.y)

    def _on_filt(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._filt = (p.x, p.y)

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd_v = msg.linear.x

    def _on_out(self, msg: Twist) -> None:
        self._out_v = msg.linear.x

    def _on_d(self, msg: Float32) -> None:
        self._obs_d = msg.data

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Save the planner's own occupancy grid once, so the metrics use the
        exact array the planner saw (map_server re-maps PGM greys)."""
        if self._map_saved:
            return
        self._map_saved = True
        info = msg.info
        np.save(self._map_path, np.array(msg.data, dtype=np.int8))
        with open(self._map_path + ".json", "w") as f:
            json.dump(
                {
                    "width": info.width,
                    "height": info.height,
                    "resolution": info.resolution,
                    "origin_x": info.origin.position.x,
                    "origin_y": info.origin.position.y,
                },
                f,
            )

    def _tick(self) -> None:
        if self._gt is None:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        raw = self._raw or (float("nan"), float("nan"))
        filt = self._filt or (float("nan"), float("nan"))
        self._f.write(
            f"{t:.3f},{self._gt[0]:.4f},{self._gt[1]:.4f},{self._gt[2]:.4f},"
            f"{raw[0]:.4f},{raw[1]:.4f},{filt[0]:.4f},{filt[1]:.4f},"
            f"{self._cmd_v:.4f},{self._out_v:.4f},{self._obs_d:.4f}\n"
        )

    def close(self) -> None:
        self._f.close()


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else "run.csv"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 180.0
    rclpy.init()
    node = RunLog(path, period=0.1)
    end = node.get_clock().now().nanoseconds * 1e-9 + dur
    while rclpy.ok() and node.get_clock().now().nanoseconds * 1e-9 < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()