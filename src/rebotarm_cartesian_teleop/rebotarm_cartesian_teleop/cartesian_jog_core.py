import math

import rclpy
from rclpy.node import Node
from rebotarm_msgs.msg import CartesianJogCmd, CartesianJogState


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

        cmd_topic = self.get_parameter("cartesian_jog_cmd_topic").value
        state_topic = self.get_parameter("cartesian_jog_state_topic").value

        self.output_mode = self.get_parameter("output_mode").value
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.servo_hz = float(self.get_parameter("servo_hz").value)

        self.workspace_x_min = float(self.get_parameter("workspace_x_min").value)
        self.workspace_x_max = float(self.get_parameter("workspace_x_max").value)
        self.workspace_y_min = float(self.get_parameter("workspace_y_min").value)
        self.workspace_y_max = float(self.get_parameter("workspace_y_max").value)
        self.workspace_z_min = float(self.get_parameter("workspace_z_min").value)
        self.workspace_z_max = float(self.get_parameter("workspace_z_max").value)

        self.target_x = float(self.get_parameter("initial_x").value)
        self.target_y = float(self.get_parameter("initial_y").value)
        self.target_z = float(self.get_parameter("initial_z").value)

        self.latest_cmd = None
        self.latest_cmd_time = None
        self.last_tick_time = self.get_clock().now()
        self.last_clamp_reason = ""

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

        self.timer = self.create_timer(1.0 / self.servo_hz, self.tick)

        self.get_logger().info("cartesian_jog_core started")
        self.get_logger().info(f"Listening to: {cmd_topic}")
        self.get_logger().info(f"Publishing to: {state_topic}")
        self.get_logger().info(f"Output mode: {self.output_mode}")
        self.get_logger().info(f"Dry run: {self.dry_run}")
        self.get_logger().info(
            f"Initial target pose: x={self.target_x:.3f}, y={self.target_y:.3f}, z={self.target_z:.3f}"
        )

    def on_cmd(self, msg: CartesianJogCmd):
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def get_command_age(self) -> float:
        if self.latest_cmd_time is None:
            return math.inf

        now = self.get_clock().now()
        age_ns = (now - self.latest_cmd_time).nanoseconds
        return age_ns / 1e9

    def compute_state_name(self, command_age: float) -> str:
        if self.latest_cmd is None:
            return "IDLE"

        if command_age > self.command_timeout_s:
            return "TIMEOUT"

        if self.latest_cmd.soft_stop:
            return "SOFT_STOP"

        if not self.latest_cmd.deadman:
            return "DEADMAN_UP"

        return "ACTIVE"

    def clamp(self, value: float, min_value: float, max_value: float):
        if value < min_value:
            return min_value, True
        if value > max_value:
            return max_value, True
        return value, False

    def integrate_target_pose(self, dt: float, state_name: str):
        self.last_clamp_reason = ""

        if state_name != "ACTIVE":
            return

        vx = float(self.latest_cmd.linear.x)
        vy = float(self.latest_cmd.linear.y)
        vz = float(self.latest_cmd.linear.z)

        self.target_x += vx * dt
        self.target_y += vy * dt
        self.target_z += vz * dt

        clamp_reasons = []

        self.target_x, clamped_x = self.clamp(
            self.target_x,
            self.workspace_x_min,
            self.workspace_x_max,
        )
        self.target_y, clamped_y = self.clamp(
            self.target_y,
            self.workspace_y_min,
            self.workspace_y_max,
        )
        self.target_z, clamped_z = self.clamp(
            self.target_z,
            self.workspace_z_min,
            self.workspace_z_max,
        )

        if clamped_x:
            clamp_reasons.append("WORKSPACE_X")
        if clamped_y:
            clamp_reasons.append("WORKSPACE_Y")
        if clamped_z:
            clamp_reasons.append("WORKSPACE_Z")

        self.last_clamp_reason = ",".join(clamp_reasons)

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick_time).nanoseconds / 1e9
        self.last_tick_time = now

        command_age = self.get_command_age()
        state_name = self.compute_state_name(command_age)

        self.integrate_target_pose(dt, state_name)
        self.publish_state(command_age, state_name)

    def publish_state(self, command_age: float, state_name: str):
        msg = CartesianJogState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.state = state_name

        msg.current_pose.position.x = self.target_x
        msg.current_pose.position.y = self.target_y
        msg.current_pose.position.z = self.target_z
        msg.current_pose.orientation.w = 1.0

        msg.target_pose.position.x = self.target_x
        msg.target_pose.position.y = self.target_y
        msg.target_pose.position.z = self.target_z
        msg.target_pose.orientation.w = 1.0

        if self.latest_cmd is not None:
            msg.commanded_twist.linear = self.latest_cmd.linear
            msg.commanded_twist.angular = self.latest_cmd.angular

        msg.q_current = []
        msg.q_target = []

        msg.ik_success = False
        msg.rejection_reason = ""
        msg.clamp_reason = self.last_clamp_reason
        msg.dry_run = self.dry_run
        msg.output_mode = self.output_mode
        msg.command_age_s = command_age if command_age != math.inf else -1.0

        if state_name == "TIMEOUT":
            msg.rejection_reason = "COMMAND_TIMEOUT"
        elif state_name == "DEADMAN_UP":
            msg.rejection_reason = "DEADMAN_UP"
        elif state_name == "SOFT_STOP":
            msg.rejection_reason = "SOFT_STOP"

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
