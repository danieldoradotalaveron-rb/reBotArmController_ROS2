import math

import rclpy
from rclpy.node import Node
from rebotarm_msgs.msg import CartesianJogCmd, CartesianJogState
from sensor_msgs.msg import JointState

from .fake_joint_state import build_fake_joint_state, fake_joint_positions_to_publish
from .fk_kinematics import FkContext, compute_fk_pose, init_fk_context, initial_target_pose_from_fk
from .jog_core_logic import (
    IkConfig,
    WorkspaceLimits,
    build_cartesian_jog_state,
    compute_state_name,
    format_ik_failure_log,
    integrate_target_pose,
    solve_target_ik,
)


class CartesianJogCore(Node):
    def __init__(self):
        super().__init__("cartesian_jog_core")

        self.declare_parameter("cartesian_jog_cmd_topic", "/rebotarm/cartesian_jog_cmd")
        self.declare_parameter("cartesian_jog_state_topic", "/rebotarm/cartesian_jog_state")
        self.declare_parameter("output_mode", "dry_run")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("command_timeout_s", 0.3)
        self.declare_parameter("servo_hz", 50.0)

        self.declare_parameter("initial_x", 0.30)
        self.declare_parameter("initial_y", 0.00)
        self.declare_parameter("initial_z", 0.20)

        self.declare_parameter("workspace_x_min", 0.15)
        self.declare_parameter("workspace_x_max", 0.45)
        self.declare_parameter("workspace_y_min", -0.25)
        self.declare_parameter("workspace_y_max", 0.25)
        self.declare_parameter("workspace_z_min", 0.05)
        self.declare_parameter("workspace_z_max", 0.45)

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("ee_frame", "end_link")
        self.declare_parameter("initial_q", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.declare_parameter("ik_max_iterations", 100)
        self.declare_parameter("ik_tolerance", 0.001)
        self.declare_parameter("max_ik_error", 0.005)
        self.declare_parameter("max_joint_delta_rad", 0.25)
        self.declare_parameter("ik_failure_log_interval_s", 1.0)
        self.declare_parameter("publish_fake_joint_states", True)
        self.declare_parameter("fake_joint_states_topic", "/rebotarm/fake_joint_states")
        self.declare_parameter("fake_joint_state_hz", 50.0)

        cmd_topic = self.get_parameter("cartesian_jog_cmd_topic").value
        state_topic = self.get_parameter("cartesian_jog_state_topic").value

        self.output_mode = self.get_parameter("output_mode").value
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.servo_hz = float(self.get_parameter("servo_hz").value)

        self._workspace = WorkspaceLimits(
            x_min=float(self.get_parameter("workspace_x_min").value),
            x_max=float(self.get_parameter("workspace_x_max").value),
            y_min=float(self.get_parameter("workspace_y_min").value),
            y_max=float(self.get_parameter("workspace_y_max").value),
            z_min=float(self.get_parameter("workspace_z_min").value),
            z_max=float(self.get_parameter("workspace_z_max").value),
        )

        fallback_x = float(self.get_parameter("initial_x").value)
        fallback_y = float(self.get_parameter("initial_y").value)
        fallback_z = float(self.get_parameter("initial_z").value)

        urdf_path = str(self.get_parameter("urdf_path").value)
        ee_frame = str(self.get_parameter("ee_frame").value)
        initial_q = [float(v) for v in self.get_parameter("initial_q").value]

        self._fk: FkContext = init_fk_context(urdf_path, ee_frame, initial_q)
        if self._fk.ok:
            self.get_logger().info(
                f"FK model loaded (nq={self._fk.model.nq}, frame={self._fk.ee_frame})"
            )
            if self._fk.q_current is not None:
                q_str = ", ".join(f"{v:.4f}" for v in self._fk.q_current)
                self.get_logger().info(f"initial_q: [{q_str}]")
        else:
            self.get_logger().error(f"FK init failed: {self._fk.error}")

        target_init = initial_target_pose_from_fk(self._fk, fallback_x, fallback_y, fallback_z)
        self.target_x = target_init.x
        self.target_y = target_init.y
        self.target_z = target_init.z
        if target_init.from_fk:
            self.get_logger().info(
                "Initial target_pose from FK(q_current): "
                f"x={self.target_x:.3f}, y={self.target_y:.3f}, z={self.target_z:.3f}"
            )
        else:
            self.get_logger().warn(
                "Initial target_pose using YAML fallback "
                f"({target_init.fallback_reason}): "
                f"x={self.target_x:.3f}, y={self.target_y:.3f}, z={self.target_z:.3f}"
            )

        self.latest_cmd = None
        self.latest_cmd_time_ns = None
        self.last_tick_time_ns = self.get_clock().now().nanoseconds
        self.last_clamp_reason = ""
        self._fk_tick_error = ""
        self._last_q_target: list[float] | None = None
        self._ik_failure_log_interval_s = float(
            self.get_parameter("ik_failure_log_interval_s").value
        )
        self._last_ik_failure_log_ns = 0

        self._publish_fake_joint_states = bool(
            self.get_parameter("publish_fake_joint_states").value
        )
        fake_joint_states_topic = str(self.get_parameter("fake_joint_states_topic").value)
        self._fake_joint_state_hz = float(self.get_parameter("fake_joint_state_hz").value)
        self._last_valid_fake_q: list[float] | None = None
        if self._publish_fake_joint_states and self._fk.q_current is not None:
            self._last_valid_fake_q = [float(v) for v in self._fk.q_current]

        self._ik_config = IkConfig(
            max_iterations=int(self.get_parameter("ik_max_iterations").value),
            tolerance=float(self.get_parameter("ik_tolerance").value),
            max_ik_error=float(self.get_parameter("max_ik_error").value),
            max_joint_delta_rad=float(self.get_parameter("max_joint_delta_rad").value),
        )

        self.subscription = self.create_subscription(
            CartesianJogCmd,
            cmd_topic,
            self.on_cmd,
            10,
        )

        self.publisher = self.create_publisher(
            CartesianJogState,
            state_topic,
            10,
        )

        self._fake_joint_states_publisher = None
        self._fake_joint_states_timer = None
        if self._publish_fake_joint_states:
            self._fake_joint_states_publisher = self.create_publisher(
                JointState,
                fake_joint_states_topic,
                10,
            )
            self._fake_joint_states_timer = self.create_timer(
                1.0 / self._fake_joint_state_hz,
                self._publish_fake_joint_state,
            )

        self.timer = self.create_timer(1.0 / self.servo_hz, self.tick)

        self.get_logger().info("cartesian_jog_core started")
        self.get_logger().info(f"Listening to: {cmd_topic}")
        self.get_logger().info(f"Publishing to: {state_topic}")
        self.get_logger().info(f"Output mode: {self.output_mode}")
        self.get_logger().info(f"Dry run: {self.dry_run}")
        self.get_logger().info(
            "Initial target pose: "
            f"x={self.target_x:.3f}, y={self.target_y:.3f}, z={self.target_z:.3f}"
        )
        if self._publish_fake_joint_states:
            self.get_logger().info(
                f"Publishing fake joint states to: {fake_joint_states_topic} "
                f"at {self._fake_joint_state_hz:.1f} Hz"
            )

    def on_cmd(self, msg: CartesianJogCmd):
        self.latest_cmd = msg
        self.latest_cmd_time_ns = self.get_clock().now().nanoseconds

    def get_command_age(self) -> float:
        if self.latest_cmd_time_ns is None:
            return math.inf

        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self.latest_cmd_time_ns) / 1e9

    def _fk_error_reason(self) -> str:
        if not self._fk.ok:
            return self._fk.error
        return self._fk_tick_error

    def _current_pose_and_q(self):
        fk_error = self._fk_error_reason()
        if fk_error:
            return None, None, fk_error

        pose, tick_error = compute_fk_pose(self._fk)
        if tick_error:
            self._fk_tick_error = tick_error
            return None, None, tick_error
        self._fk_tick_error = ""

        q_list = None
        if self._fk.q_current is not None:
            q_list = [float(v) for v in self._fk.q_current]
        return pose, q_list, ""

    def _maybe_log_ik_failure(self, diag, now_ns: int) -> None:
        if diag is None:
            return
        interval_ns = int(self._ik_failure_log_interval_s * 1e9)
        if now_ns - self._last_ik_failure_log_ns < interval_ns:
            return
        self._last_ik_failure_log_ns = now_ns
        self.get_logger().warn(format_ik_failure_log(diag))

    def _publish_fake_joint_state(self) -> None:
        if self._fake_joint_states_publisher is None or self._last_valid_fake_q is None:
            return
        msg = build_fake_joint_state(self._last_valid_fake_q)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._fake_joint_states_publisher.publish(msg)

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        dt = (now_ns - self.last_tick_time_ns) / 1e9
        self.last_tick_time_ns = now_ns

        command_age = self.get_command_age()
        state_name = compute_state_name(
            self.latest_cmd,
            command_age,
            self.command_timeout_s,
        )

        self.target_x, self.target_y, self.target_z, self.last_clamp_reason = integrate_target_pose(
            self.target_x,
            self.target_y,
            self.target_z,
            self.latest_cmd,
            dt,
            state_name,
            self._workspace,
        )

        current_pose, q_current, fk_error = self._current_pose_and_q()

        q_target, ik_success, ik_reason, self._last_q_target, ik_failure_diag = solve_target_ik(
            fk_ctx=self._fk,
            state_name=state_name,
            target_x=self.target_x,
            target_y=self.target_y,
            target_z=self.target_z,
            current_pose=current_pose,
            ik_config=self._ik_config,
            last_q_target=self._last_q_target,
            clamp_reason=self.last_clamp_reason,
        )
        self._maybe_log_ik_failure(ik_failure_diag, now_ns)

        if self._publish_fake_joint_states:
            self._last_valid_fake_q, _ = fake_joint_positions_to_publish(
                enabled=True,
                last_valid_fake_q=self._last_valid_fake_q,
                q_current=q_current,
                ik_success=ik_success,
                q_target=q_target,
            )

        msg = build_cartesian_jog_state(
            state_name=state_name,
            target_x=self.target_x,
            target_y=self.target_y,
            target_z=self.target_z,
            latest_cmd=self.latest_cmd,
            clamp_reason=self.last_clamp_reason,
            dry_run=self.dry_run,
            output_mode=self.output_mode,
            command_age=command_age,
            current_pose=current_pose,
            q_current=q_current,
            q_target=q_target,
            ik_success=ik_success,
            fk_error=fk_error,
            ik_reason=ik_reason,
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianJogCore()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
