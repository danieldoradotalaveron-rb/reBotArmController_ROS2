#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rebotarm_msgs.msg import JointMotorCmd


class DemoJointPassthrough(Node):
    def __init__(self) -> None:
        super().__init__("demo_joint_passthrough")
        self._publisher = self.create_publisher(
            JointMotorCmd,
            "/rebotarm/joints/joint1/cmd",
            10,
        )

    def run(self) -> None:
        msg = JointMotorCmd()
        msg.mode = JointMotorCmd.MODE_MIT
        msg.use_pos = True
        msg.use_kp = True
        msg.use_kd = True
        msg.pos = 0.0
        msg.kp = 80.0
        msg.kd = 4.0
        msg.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(msg)
        self.get_logger().info("published joint1 MIT passthrough command")


def main() -> None:
    rclpy.init()
    node = DemoJointPassthrough()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
