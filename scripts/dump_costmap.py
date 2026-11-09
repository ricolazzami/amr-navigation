#!/usr/bin/env python3
"""Print the value scale of a Nav2 costmap topic (sanity check for thresholds).

    python3 scripts/dump_costmap.py /local_costmap/costmap 5

Prints the distinct non-zero values and their counts, plus max/lethal count.
Nav2 publishes costs in 0..100 (lethal 100, inscribed 99), never raw 0..255.
"""

import collections
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node


class Dump(Node):
    def __init__(self, topic):
        super().__init__("dump_costmap")
        self.done = False
        self.create_subscription(OccupancyGrid, topic, self.on_msg, 10)

    def on_msg(self, msg):
        counts = collections.Counter(msg.data)
        nonzero = {k: v for k, v in sorted(counts.items()) if k != 0}
        print(f"topic={msg.header.frame_id} {msg.info.width}x{msg.info.height} "
              f"res={msg.info.resolution} origin=({msg.info.origin.position.x:.2f},"
              f"{msg.info.origin.position.y:.2f})")
        print(f"max={max(counts)} distinct={len(counts)}")
        print(f"values>0: {nonzero}")
        self.done = True


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/local_costmap/costmap"
    rclpy.init()
    node = Dump(topic)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
