#!/usr/bin/env python3
"""Rewrite Gazebo's model-scoped odometry frames to plain TF frames.

gz's DiffDrive plugin scopes ``odom_frame``/``child_frame`` with the model name,
so it publishes ``amr/odom`` -> ``amr/base_footprint``. robot_localization
silently drops odometry whose frames are not the ones it was configured with,
which leaves the filter frozen at the origin. This relay republishes the same
message on a clean topic with ``odom`` -> ``base_footprint``.

    /wheel/odom (amr/odom -> amr/base_footprint)
        -> /wheel/odom_fixed (odom -> base_footprint)
"""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomFrameRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_frame_relay")

        self.declare_parameter("input_topic", "/wheel/odom")
        self.declare_parameter("output_topic", "/wheel/odom_fixed")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_footprint")

        gp = self.get_parameter
        in_topic = gp("input_topic").value
        out_topic = gp("output_topic").value
        self._frame = gp("frame_id").value
        self._child = gp("child_frame_id").value

        self._pub = self.create_publisher(Odometry, out_topic, 20)
        self.create_subscription(Odometry, in_topic, self._on_odom, 20)

        self.get_logger().info(
            f"odom relay: {in_topic} -> {out_topic} "
            f"({self._frame} -> {self._child})"
        )

    def _on_odom(self, msg: Odometry) -> None:
        msg.header.frame_id = self._frame
        msg.child_frame_id = self._child
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
