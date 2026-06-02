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

        joy_topic = self.get_parameter("joy_topic").value
        cmd_topic = self.get_parameter("cartesian_jog_cmd_topic").value
        self.deadzone = float(self.get_parameter("deadzone").value)

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
        self.get_logger().info(f"Deadzone: {self.deadzone}")

    def on_joy(self, msg: Joy):
        self.latest_joy = msg
        self.get_logger().info(
            f"Joy received: axes={len(msg.axes)} buttons={len(msg.buttons)}",
            throttle_duration_sec=2.0,
        )

    def publish_cmd(self):
        msg = CartesianJogCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        msg.deadman = False
        msg.soft_stop = False
        msg.speed_scale = 0.0
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
