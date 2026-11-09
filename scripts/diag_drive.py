#!/usr/bin/env python3
"""Print a compact drive table: controller cmd, gated cmd, robot pose.

Run inside the container (needs the ROS environment):

    python3 scripts/diag_drive.py            # 30 s, every 0.5 s
    python3 scripts/diag_drive.py 60         # 60 s

Columns: t, nav(lin,ang) = /cmd_vel_nav, out(lin,ang) = /cmd_vel,
map(x,y,yaw) from TF map->base_link, d = /avoidance/distance, n = /plan poses.
"""

import math
import sys
import threading

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32


class Diag(Node):
    def __init__(self):
        super().__init__("diag_drive")
        self.nav = None
        self.out = None
        self.dist = float("nan")
        self.npts = 0
        self.first = None
        self.last = None
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_nav, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_out, 10)
        self.create_subscription(Float32, "/avoidance/distance", self.on_d, 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

    def on_nav(self, m):
        self.nav = m

    def on_out(self, m):
        self.out = m

    def on_d(self, m):
        self.dist = m.data

    def on_plan(self, m):
        self.npts = len(m.poses)
        if m.poses:
            self.first = m.poses[0].pose.position
            self.last = m.poses[-1].pose.position

    def pose(self):
        try:
            tf = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            )
            return t.x, t.y, yaw
        except Exception:
            return float("nan"), float("nan"), float("nan")


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = Diag()
    print("  t   nav(lin,ang)     out(lin,ang)     map(x,y,yaw)        d      n  first->last")
    n = int(dur / 0.5)
    for i in range(n):
        import time

        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        nv = node.nav
        ou = node.out
        px, py, pyaw = node.pose()
        nvs = f"{nv.linear.x:+.3f},{nv.angular.z:+.3f}" if nv else "  --,--"
        ous = f"{ou.linear.x:+.3f},{ou.angular.z:+.3f}" if ou else "  --,--"
        fl = ""
        if node.first and node.last:
            fl = (
                f"({node.first.x:+.2f},{node.first.y:+.2f})->"
                f"({node.last.x:+.2f},{node.last.y:+.2f})"
            )
        print(
            f"{i * 0.5:5.1f}  {nvs:>14}  {ous:>14}  "
            f"({px:+.2f},{py:+.2f},{pyaw:+.2f})  {node.dist:5.2f} {node.npts:3d}  {fl}"
        )
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
