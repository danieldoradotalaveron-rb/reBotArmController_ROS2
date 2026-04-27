#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node
from rebotarm_msgs.action import MoveToPose
from rebotarm_msgs.srv import SetMode
from sensor_msgs.msg import JointState


class DemoMoveToPose(Node):
    def __init__(self) -> None:
        super().__init__("demo_move_to_pose")
        self._latest_joint_state = None
        self.create_subscription(
            JointState,
            "/rebotarm/joint_states",
            self._joint_state_cb,
            10,
        )
        self._set_mode = self.create_client(SetMode, "/rebotarm/set_mode")
        self._move_to_pose = ActionClient(
            self,
            MoveToPose,
            "/rebotarm/move_to_pose",
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def run(self) -> bool:
        if not self._set_mode.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("set_mode service not available")
            return False
        req = SetMode.Request()
        req.mode = "pos_vel"
        future = self._set_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result() or not future.result().success:
            self.get_logger().error("failed to switch to pos_vel")
            return False

        if not self._move_to_pose.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_to_pose action not available")
            return False

        goal = MoveToPose.Goal()
        goal.target_pose = Pose()
        goal.target_pose.position.x = 0.30
        goal.target_pose.position.y = 0.0
        goal.target_pose.position.z = 0.30
        goal.target_pose.orientation.w = 1.0
        goal.duration = 2.0

        send_future = self._move_to_pose.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"success={result.success} message={result.message}")
        return bool(result.success)

    def _feedback_cb(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"progress={feedback.progress:.2f} elapsed={feedback.time_elapsed:.2f}s"
        )


def main() -> None:
    rclpy.init()
    node = DemoMoveToPose()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
