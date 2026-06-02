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

        cmd_topic = self.get_parameter("cartesian_jog_cmd_topic").value
        state_topic = self.get_parameter("cartesian_jog_state_topic").value

        self.output_mode = self.get_parameter("output_mode").value
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)

        self.latest_cmd = None
        self.latest_cmd_time = None

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

        self.timer = self.create_timer(1.0 / 50.0, self.publish_state)

        self.get_logger().info("cartesian_jog_core started")
        self.get_logger().info(f"Listening to: {cmd_topic}")
        self.get_logger().info(f"Publishing to: {state_topic}")
        self.get_logger().info(f"Output mode: {self.output_mode}")
        self.get_logger().info(f"Dry run: {self.dry_run}")

    def on_cmd(self, msg: CartesianJogCmd):
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()

    def get_command_age(self) -> float:
        if self.latest_cmd_time is None:
            return float("inf")

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

    def publish_state(self):
        command_age = self.get_command_age()
        state_name = self.compute_state_name(command_age)

        msg = CartesianJogState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.state = state_name

        if self.latest_cmd is not None:
            msg.commanded_twist.linear = self.latest_cmd.linear
            msg.commanded_twist.angular = self.latest_cmd.angular

        msg.q_current = []
        msg.q_target = []

        msg.ik_success = False
        msg.rejection_reason = ""
        msg.clamp_reason = ""
        msg.dry_run = self.dry_run
        msg.output_mode = self.output_mode
        msg.command_age_s = command_age if command_age != float("inf") else -1.0

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
