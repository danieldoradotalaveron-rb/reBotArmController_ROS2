import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyCartesianMapper(Node):
    def __init__(self):
        super().__init__("joy_cartesian_mapper")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("deadzone", 0.15)

        joy_topic = self.get_parameter("joy_topic").value
        self.deadzone = float(self.get_parameter("deadzone").value)

        self.subscription = self.create_subscription(
            Joy,
            joy_topic,
            self.on_joy,
            10,
        )

        self.get_logger().info("joy_cartesian_mapper started")
        self.get_logger().info(f"Listening to: {joy_topic}")
        self.get_logger().info(f"Deadzone: {self.deadzone}")

    def on_joy(self, msg: Joy):
        self.get_logger().info(
            f"Joy received: axes={len(msg.axes)} buttons={len(msg.buttons)}",
            throttle_duration_sec=2.0,
        )


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
