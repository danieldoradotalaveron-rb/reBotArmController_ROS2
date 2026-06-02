import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rebotarm_msgs.msg import CartesianJogCmd


class JoyCartesianMapper(Node):
    def __init__(self):
        super().__init__("joy_cartesian_mapper")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cartesian_jog_cmd_topic", "/rebotarm/cartesian_jog_cmd")
        self.declare_parameter("deadzone", 0.15)

        self.declare_parameter("deadman_button", 4)
        self.declare_parameter("soft_stop_button", 2)
        self.declare_parameter("speed_boost_button", 5)

        self.declare_parameter("speed_scale_default", 1.0)
        self.declare_parameter("speed_scale_boost", 1.5)

        joy_topic = self.get_parameter("joy_topic").value
        cmd_topic = self.get_parameter("cartesian_jog_cmd_topic").value

        self.deadzone = float(self.get_parameter("deadzone").value)
        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.soft_stop_button = int(self.get_parameter("soft_stop_button").value)
        self.speed_boost_button = int(self.get_parameter("speed_boost_button").value)
        self.speed_scale_default = float(self.get_parameter("speed_scale_default").value)
        self.speed_scale_boost = float(self.get_parameter("speed_scale_boost").value)

        self.latest_joy = None

        self.subscription = self.create_subscription(
            Joy,
            joy_topic,
            self.on_joy,
            10,
        )

        self.publisher = self.create_publisher(
            CartesianJogCmd,
            cmd_topic,
            10,
        )

        self.timer = self.create_timer(1.0 / 30.0, self.publish_cmd)

        self.get_logger().info("joy_cartesian_mapper started")
        self.get_logger().info(f"Listening to: {joy_topic}")
        self.get_logger().info(f"Publishing to: {cmd_topic}")
        self.get_logger().info(f"Deadman button: {self.deadman_button}")
        self.get_logger().info(f"Soft stop button: {self.soft_stop_button}")
        self.get_logger().info(f"Speed boost button: {self.speed_boost_button}")

    def on_joy(self, msg: Joy):
        self.latest_joy = msg
        self.get_logger().info(
            f"Joy received: axes={len(msg.axes)} buttons={len(msg.buttons)}",
            throttle_duration_sec=2.0,
        )

    def button_pressed(self, button_index: int) -> bool:
        if self.latest_joy is None:
            return False

        if button_index < 0 or button_index >= len(self.latest_joy.buttons):
            return False

        return self.latest_joy.buttons[button_index] == 1

    def publish_cmd(self):
        msg = CartesianJogCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        deadman = self.button_pressed(self.deadman_button)
        soft_stop = self.button_pressed(self.soft_stop_button)
        speed_boost = self.button_pressed(self.speed_boost_button)

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        msg.deadman = deadman
        msg.soft_stop = soft_stop
        msg.speed_scale = self.speed_scale_boost if speed_boost else self.speed_scale_default
        msg.enable_orientation = False

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyCartesianMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
