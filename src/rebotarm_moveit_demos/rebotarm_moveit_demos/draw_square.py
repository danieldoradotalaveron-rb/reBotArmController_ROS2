from __future__ import annotations

import sys
from math import pi

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
import rclpy
from std_msgs.msg import Header
from tf_transformations import quaternion_from_euler

from rebotarm_moveit_demos.demo_common import MoveItDemoBase


class DrawSquare(MoveItDemoBase):
    """Move the TCP through four coplanar rectangle corners."""

    def __init__(self) -> None:
        super().__init__("draw_square")
        self.wrap_joint_names = {str(name) for name in self._param("wrap_joint_names")}
        self.max_wrap_joint_delta = float(self._param("max_wrap_joint_delta"))
        self.frame_id = str(self._param("frame_id"))
        self.tcp_link_name = str(self._param("tcp_link_name"))
        self.start_point = self._wrap_joints(
            [float(value) for value in self._param("start_point")]
        )
        self.rectangle_center = [float(value) for value in self._param("rectangle_center")]
        self.rectangle_width = float(self._param("rectangle_width"))
        self.rectangle_height = float(self._param("rectangle_height"))
        self.tcp_rpy = [float(value) for value in self._param("tcp_rpy")]
        self.tcp_yaw_offsets = [float(value) for value in self._param("tcp_yaw_offsets")]
        self.ik_timeout = float(self._param("ik_timeout"))
        self.result_timeout = float(self._param("result_timeout"))
        self.motion_duration = float(self._param("motion_duration"))
        self.avoid_collisions = bool(self._param("avoid_collisions"))

    def run(self) -> bool:
        if not self.wait_for_ik_service():
            return False
        if not self.wait_for_execute_server():
            return False

        current = self._wrap_joints(self._current_joint_values())
        if not self._move_joints("reset", current, self.start_point):
            return False
        current = self.start_point

        points = self._rectangle_points()
        points.append(points[0])

        for index, point in enumerate(points, start=1):
            label = f"corner {index}"
            target = self.corner_joint_target(point, current, label)
            if target is None or not self._move_joints(label, current, target):
                return False
            current = target

        self.node.get_logger().info("rectangle draw demo finished")
        return True

    def _rectangle_points(self) -> list[list[float]]:
        center = self.rectangle_center
        half_width = self.rectangle_width * 0.5
        half_height = self.rectangle_height * 0.5
        return [
            [center[0] - half_width, center[1] - half_height, center[2]],
            [center[0] + half_width, center[1] - half_height, center[2]],
            [center[0] + half_width, center[1] + half_height, center[2]],
            [center[0] - half_width, center[1] + half_height, center[2]],
        ]

    def _waypoint(self, tcp_position: list[float], yaw_offset: float = 0.0) -> Pose:
        roll, pitch, yaw = self.tcp_rpy
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw + yaw_offset)
        return Pose(
            position=Point(x=tcp_position[0], y=tcp_position[1], z=tcp_position[2]),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )

    def corner_joint_target(
        self,
        tcp_position: list[float],
        seed_values: list[float],
        label: str,
    ) -> list[float] | None:
        seed_values = self._wrap_joints(seed_values)
        self.node.get_logger().info(
            f"compute IK for {label}: "
            f"[{tcp_position[0]:.3f}, {tcp_position[1]:.3f}, {tcp_position[2]:.3f}]"
        )

        best = None
        best_yaw_offset = 0.0
        best_cost = float("inf")
        for yaw_offset in self.tcp_yaw_offsets:
            target = self._corner_joint_target(tcp_position, seed_values, label, yaw_offset)
            if target is None:
                continue
            if any(
                name in self.wrap_joint_names
                and abs(goal - start) > self.max_wrap_joint_delta
                for name, start, goal in zip(self.joint_names, seed_values, target)
            ):
                continue
            cost = sum(abs(goal - start) for start, goal in zip(seed_values, target))
            if cost < best_cost:
                best = target
                best_yaw_offset = yaw_offset
                best_cost = cost

        if best is None:
            self.node.get_logger().error(
                f"Failed to compute IK for {label} without wrapped-joint flip"
            )
            return None

        self.node.get_logger().info(
            f"{label} target yaw_offset={best_yaw_offset:.4f}: "
            f"{[round(value, 4) for value in best]}"
        )
        return best

    def _corner_joint_target(
        self,
        tcp_position: list[float],
        seed_values: list[float],
        label: str,
        yaw_offset: float,
    ) -> list[float] | None:
        target = self.compute_ik_joint_target(
            PoseStamped(
                header=Header(frame_id=self.frame_id),
                pose=self._waypoint(tcp_position, yaw_offset),
            ),
            seed_values,
            self.tcp_link_name,
            self.ik_timeout,
            self.avoid_collisions,
            f"IK for {label} yaw_offset={yaw_offset:.4f}",
            warn_only=True,
        )
        return None if target is None else self._wrap_joints(target, seed_values)

    def _wrap_joints(
        self,
        values: list[float],
        reference: list[float] | None = None,
    ) -> list[float]:
        result = []
        references = reference if reference is not None else [0.0] * len(values)
        for name, value, ref in zip(self.joint_names, values, references):
            if name not in self.wrap_joint_names:
                result.append(value)
                continue
            wrapped = ref + (value - ref + pi) % (2.0 * pi) - pi
            if wrapped < -pi or wrapped > pi:
                wrapped = (value + pi) % (2.0 * pi) - pi
            result.append(pi if wrapped == -pi and value > 0.0 else wrapped)
        return result

    def _move_joints(
        self,
        label: str,
        start_values: list[float],
        goal_values: list[float],
    ) -> bool:
        start_values = self._wrap_joints(start_values)
        goal_values = self._wrap_joints(goal_values)
        self.node.get_logger().info(f"move to {label}")
        return self.execute_trajectory(
            self.joint_trajectory(start_values, goal_values, self.motion_duration),
            self.result_timeout,
        )

    def _current_joint_values(self) -> list[float]:
        return self.current_joint_values(list(self.start_point), "start_point")


def main() -> None:
    rclpy.init()
    demo = DrawSquare()
    try:
        ok = demo.run()
    except Exception as exc:
        demo.node.get_logger().error(str(exc))
        ok = False
    finally:
        demo.node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
