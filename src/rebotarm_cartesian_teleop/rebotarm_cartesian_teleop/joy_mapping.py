"""Pure Joy -> CartesianJogCmd mapping (no ROS node dependencies)."""

from __future__ import annotations

from dataclasses import dataclass

from rebotarm_msgs.msg import CartesianJogCmd
from sensor_msgs.msg import Joy


@dataclass(frozen=True)
class JoyMapperConfig:
    axis_x: int = 1
    axis_y: int = 0
    axis_z: int = 5
    invert_x: bool = False
    invert_y: bool = False
    invert_z: bool = False
    deadzone: float = 0.15
    joy_timeout_s: float = 0.3
    max_linear_velocity: float = 0.03
    deadman_button: int = 4
    soft_stop_button: int = 2
    speed_boost_button: int = 5
    speed_scale_default: float = 1.0
    speed_scale_boost: float = 1.5


def button_pressed(joy: Joy | None, button_index: int) -> bool:
    if joy is None:
        return False
    if button_index < 0 or button_index >= len(joy.buttons):
        return False
    return joy.buttons[button_index] == 1


def axis_value(
    joy: Joy | None,
    axis_index: int,
    invert: bool,
    deadzone: float,
) -> float:
    if joy is None:
        return 0.0
    if axis_index < 0 or axis_index >= len(joy.axes):
        return 0.0

    value = float(joy.axes[axis_index])
    if abs(value) < deadzone:
        return 0.0
    if invert:
        value = -value
    return value


def is_joy_fresh(
    latest_joy_time_ns: int | None,
    now_ns: int,
    joy_timeout_s: float,
) -> bool:
    if latest_joy_time_ns is None:
        return False
    age_s = (now_ns - latest_joy_time_ns) / 1e9
    return age_s <= joy_timeout_s


def map_joy_to_cmd(
    joy: Joy | None,
    cfg: JoyMapperConfig,
    *,
    latest_joy_time_ns: int | None,
    now_ns: int,
) -> CartesianJogCmd | None:
    """Map Joy to CartesianJogCmd, or None if Joy is stale (do not publish)."""
    if not is_joy_fresh(latest_joy_time_ns, now_ns, cfg.joy_timeout_s):
        return None

    msg = CartesianJogCmd()
    msg.header.frame_id = "base_link"

    deadman = button_pressed(joy, cfg.deadman_button)
    soft_stop = button_pressed(joy, cfg.soft_stop_button)
    speed_boost = button_pressed(joy, cfg.speed_boost_button)

    speed_scale = cfg.speed_scale_boost if speed_boost else cfg.speed_scale_default

    x = axis_value(joy, cfg.axis_x, cfg.invert_x, cfg.deadzone)
    y = axis_value(joy, cfg.axis_y, cfg.invert_y, cfg.deadzone)
    z = axis_value(joy, cfg.axis_z, cfg.invert_z, cfg.deadzone)

    if not deadman or soft_stop:
        x = 0.0
        y = 0.0
        z = 0.0

    scale = cfg.max_linear_velocity * speed_scale
    msg.linear.x = x * scale
    msg.linear.y = y * scale
    msg.linear.z = z * scale

    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    msg.deadman = deadman
    msg.soft_stop = soft_stop
    msg.speed_scale = speed_scale
    msg.enable_orientation = False

    return msg
