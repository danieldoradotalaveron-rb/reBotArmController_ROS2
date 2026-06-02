import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rebotarm_msgs.msg import CartesianJogCmd


class JoyCartesianMapper(Node):
    def __init__(self):
        super().__init__("joy_cartesian_mapper")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("cartesian_jog_cmd_topic", "/rebotarm/cartesian_jog_cmd")

        self.declare_parameter("axis_x", 1)
        self.declare_parameter("axis_y", 0)
        self.declare_parameter("axis_z", 5)

        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)
        self.declare_parameter("invert_z", False)

        self.declare_parameter("deadzone", 0.15)
        self.declare_parameter("max_linear_velocity_m_s", 0.03)

        self.declare_parameter("deadman_button", 4)
        self.declare_parameter("soft_stop_button", 2)
        self.declare_parameter("speed_boost_button", 5)

        self.declare_parameter("speed_scale_default", 1.0)
        self.declare_parameter("speed_scale_boost", 1.5)

        joy_topic = self.get_parameter("joy_topic").value
        cmd_topic = self.get_parameter("cartesian_jog_cmd_topic").value

        self.axis_x = int(self.get_parameter("axis_x").value)
        self.axis_y = int(self.get_parameter("axis_y").value)
        self.axis_z = int(self.get_parameter("axis_z").value)

        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)
        self.invert_z = bool(self.get_parameter("invert_z").value)

        self.deadzone = float(self.get_parameter("deadzone").value)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity_m_s").value)

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
        self.get_logger().info(f"Axes x/y/z: {self.axis_x}/{self.axis_y}/{self.axis_z}")
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

    def axis_value(self, axis_index: int, invert: bool) -> float:
        if self.latest_joy is None:
            return 0.0

        if axis_index < 0 or axis_index >= len(self.latest_joy.axes):
            return 0.0

        value = float(self.latest_joy.axes[axis_index])

        if abs(value) < self.deadzone:
            return 0.0

        if invert:
            value = -value

        return value

    def publish_cmd(self):
        msg = CartesianJogCmd()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        deadman = self.button_pressed(self.deadman_button)
        soft_stop = self.button_pressed(self.soft_stop_button)
        speed_boost = self.button_pressed(self.speed_boost_button)

        speed_scale = self.speed_scale_boost if speed_boost else self.speed_scale_default

        x = self.axis_value(self.axis_x, self.invert_x)
        y = self.axis_value(self.axis_y, self.invert_y)
        z = self.axis_value(self.axis_z, self.invert_z)

        if not deadman or soft_stop:
            x = 0.0
            y = 0.0
            z = 0.0

        msg.linear.x = x * self.max_linear_velocity * speed_scale
        msg.linear.y = y * self.max_linear_velocity * speed_scale
        msg.linear.z = z * self.max_linear_velocity * speed_scale

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        msg.deadman = deadman
        msg.soft_stop = soft_stop
        msg.speed_scale = speed_scale
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
