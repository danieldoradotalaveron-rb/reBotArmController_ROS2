from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rebotarm_msgs.action import MoveToPose
from trajectory_msgs.msg import JointTrajectoryPoint


class ArmActions:
    def __init__(self, node, hardware, namespace: str) -> None:
        self._node = node
        self._hardware = hardware
        self._namespace = namespace
        self._move_to_pose_server = ActionServer(
            node,
            MoveToPose,
            f"/{namespace}/move_to_pose",
            execute_callback=self.execute_move_to_pose,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_move_to_pose,
            callback_group=node.reentrant_group,
        )
        self._follow_joint_trajectory_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"/{namespace}/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_follow_joint_trajectory,
            callback_group=node.reentrant_group,
        )
        self._gripper_command_server = ActionServer(
            node,
            GripperCommand,
            f"/{namespace}/gripper/command",
            execute_callback=self.execute_gripper_command,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_gripper_command,
            callback_group=node.reentrant_group,
        )

    def goal_callback(self, _goal_request):
        return GoalResponse.ACCEPT

    def cancel_move_to_pose(self, _goal_handle):
        self._hardware.cancel_motion()
        return CancelResponse.ACCEPT

    def cancel_follow_joint_trajectory(self, _goal_handle):
        return CancelResponse.ACCEPT

    def cancel_gripper_command(self, _goal_handle):
        return CancelResponse.ACCEPT

    def execute_move_to_pose(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()

        try:
            self._hardware.set_state_machine("TRAJ_RUNNING")
            self._node.publish_arm_status()
            ok = self._hardware.move_to_pose_traj(goal.target_pose, float(goal.duration))
        except Exception as exc:
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            result.final_pose = self._hardware.current_pose()
            return result

        if not ok:
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
            goal_handle.abort()
            result.success = False
            result.message = "trajectory planning failed"
            result.final_pose = self._hardware.current_pose()
            return result

        start = time.monotonic()
        requested_duration = float(goal.duration)
        feedback = MoveToPose.Feedback()
        while self._hardware.motion_active():
            if goal_handle.is_cancel_requested:
                self._hardware.cancel_motion()
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()
                goal_handle.canceled()
                result.success = False
                result.message = "canceled"
                result.final_pose = self._hardware.current_pose()
                return result

            if self._hardware.state_machine != "TRAJ_RUNNING":
                goal_handle.abort()
                result.success = False
                result.message = "preempted"
                result.final_pose = self._hardware.current_pose()
                return result

            feedback.current_pose = self._hardware.current_pose()
            elapsed = float(time.monotonic() - start)
            if requested_duration > 0.0:
                feedback.progress = max(0.0, min(1.0, elapsed / requested_duration))
            else:
                feedback.progress = self._hardware.motion_progress()
            feedback.time_elapsed = elapsed
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)

        result.success = True
        result.message = "move_to_pose complete"
        result.final_pose = self._hardware.current_pose()
        self._hardware.set_state_machine("IDLE")
        self._node.publish_arm_status()
        goal_handle.succeed()
        return result

    def execute_follow_joint_trajectory(self, goal_handle):
        goal = goal_handle.request
        result = FollowJointTrajectory.Result()
        trajectory = goal.trajectory

        if not trajectory.joint_names or not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "trajectory must include joint_names and points"
            return result

        try:
            self._hardware.set_state_machine("TRAJ_RUNNING")
            self._node.publish_arm_status()
            self._hardware.ensure_pos_vel_control()
        except Exception as exc:
            self._hardware.set_state_machine("IDLE")
            self._node.publish_arm_status()
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

        start = time.monotonic()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(trajectory.joint_names)

        for point in trajectory.points:
            if len(point.positions) != len(trajectory.joint_names):
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "point.positions length must match joint_names"
                return result

            target_time = start + float(point.time_from_start.sec) + (
                float(point.time_from_start.nanosec) * 1e-9
            )
            while time.monotonic() < target_time:
                if goal_handle.is_cancel_requested:
                    self._hardware.set_state_machine("IDLE")
                    self._node.publish_arm_status()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = "canceled"
                    return result
                if self._hardware.state_machine != "TRAJ_RUNNING":
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = "preempted"
                    return result
                time.sleep(0.01)

            try:
                self._hardware.set_joint_target(
                    list(trajectory.joint_names),
                    [float(v) for v in point.positions],
                )
            except Exception as exc:
                self._hardware.set_state_machine("IDLE")
                self._node.publish_arm_status()
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = str(exc)
                return result

            feedback.desired = point
            feedback.actual = self._actual_point()
            feedback.error = self._error_point(point, feedback.actual)
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "follow_joint_trajectory complete"
        self._hardware.set_state_machine("IDLE")
        self._node.publish_arm_status()
        return result

    def _actual_point(self) -> JointTrajectoryPoint:
        pos, vel, _ = self._hardware.get_joint_state()
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in pos]
        point.velocities = [float(v) for v in vel]
        return point

    @staticmethod
    def _error_point(desired: JointTrajectoryPoint, actual: JointTrajectoryPoint) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = [
            float(a - d) for d, a in zip(desired.positions, actual.positions)
        ]
        if desired.velocities and actual.velocities:
            point.velocities = [
                float(a - d) for d, a in zip(desired.velocities, actual.velocities)
            ]
        return point

    def execute_gripper_command(self, goal_handle):
        goal = goal_handle.request.command
        result = GripperCommand.Result()
        feedback = GripperCommand.Feedback()

        try:
            self._hardware.set_gripper_target(goal.position, goal.max_effort)
        except Exception:
            goal_handle.abort()
            result.position = 0.0
            result.effort = 0.0
            result.stalled = False
            result.reached_goal = False
            return result

        start = time.monotonic()
        last_pos = self._hardware.gripper_position_m()
        stalled = False
        while time.monotonic() - start < 5.0:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.position = self._hardware.gripper_position_m()
                result.effort = self._hardware.get_gripper_state()[2]
                result.stalled = stalled
                result.reached_goal = False
                return result

            pos = self._hardware.gripper_position_m()
            effort = self._hardware.get_gripper_state()[2]
            reached = self._hardware.gripper_reached_target()
            stalled = abs(pos - last_pos) < 1e-4 and abs(effort) >= float(goal.max_effort)
            feedback.position = pos
            feedback.effort = effort
            feedback.stalled = stalled
            feedback.reached_goal = reached
            goal_handle.publish_feedback(feedback)
            if reached:
                break
            last_pos = pos
            time.sleep(0.05)

        result.position = self._hardware.gripper_position_m()
        result.effort = self._hardware.get_gripper_state()[2]
        result.stalled = stalled
        result.reached_goal = self._hardware.gripper_reached_target()
        goal_handle.succeed()
        return result
