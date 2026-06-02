"""Pure Cartesian jog core logic (state machine, integration, clamps)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from rebotarm_msgs.msg import CartesianJogCmd, CartesianJogState


@dataclass(frozen=True)
class WorkspaceLimits:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


def compute_state_name(
    latest_cmd: CartesianJogCmd | None,
    command_age: float,
    command_timeout_s: float,
) -> str:
    if latest_cmd is None:
        return "IDLE"

    if command_age > command_timeout_s:
        return "TIMEOUT"

    if latest_cmd.soft_stop:
        return "SOFT_STOP"

    if not latest_cmd.deadman:
        return "DEADMAN_UP"

    return "ACTIVE"


def rejection_reason_for_state(state_name: str) -> str:
    if state_name == "TIMEOUT":
        return "COMMAND_TIMEOUT"
    if state_name == "DEADMAN_UP":
        return "DEADMAN_UP"
    if state_name == "SOFT_STOP":
        return "SOFT_STOP"
    return ""


def clamp(value: float, min_value: float, max_value: float) -> tuple[float, bool]:
    if value < min_value:
        return min_value, True
    if value > max_value:
        return max_value, True
    return value, False


def integrate_target_pose(
    target_x: float,
    target_y: float,
    target_z: float,
    latest_cmd: CartesianJogCmd | None,
    dt: float,
    state_name: str,
    workspace: WorkspaceLimits,
) -> tuple[float, float, float, str]:
    if state_name != "ACTIVE" or latest_cmd is None:
        return target_x, target_y, target_z, ""

    vx = float(latest_cmd.linear.x)
    vy = float(latest_cmd.linear.y)
    vz = float(latest_cmd.linear.z)

    target_x += vx * dt
    target_y += vy * dt
    target_z += vz * dt

    clamp_reasons: list[str] = []

    target_x, clamped_x = clamp(target_x, workspace.x_min, workspace.x_max)
    target_y, clamped_y = clamp(target_y, workspace.y_min, workspace.y_max)
    target_z, clamped_z = clamp(target_z, workspace.z_min, workspace.z_max)

    if clamped_x:
        clamp_reasons.append("WORKSPACE_X")
    if clamped_y:
        clamp_reasons.append("WORKSPACE_Y")
    if clamped_z:
        clamp_reasons.append("WORKSPACE_Z")

    return target_x, target_y, target_z, ",".join(clamp_reasons)


def build_cartesian_jog_state(
    *,
    state_name: str,
    target_x: float,
    target_y: float,
    target_z: float,
    latest_cmd: CartesianJogCmd | None,
    clamp_reason: str,
    dry_run: bool,
    output_mode: str,
    command_age: float,
) -> CartesianJogState:
    msg = CartesianJogState()
    msg.header.frame_id = "base_link"
    msg.state = state_name

    msg.current_pose.position.x = target_x
    msg.current_pose.position.y = target_y
    msg.current_pose.position.z = target_z
    msg.current_pose.orientation.w = 1.0

    msg.target_pose.position.x = target_x
    msg.target_pose.position.y = target_y
    msg.target_pose.position.z = target_z
    msg.target_pose.orientation.w = 1.0

    if latest_cmd is not None:
        msg.commanded_twist.linear = latest_cmd.linear
        msg.commanded_twist.angular = latest_cmd.angular

    msg.q_current = []
    msg.q_target = []
    msg.ik_success = False
    msg.rejection_reason = rejection_reason_for_state(state_name)
    msg.clamp_reason = clamp_reason
    msg.dry_run = dry_run
    msg.output_mode = output_mode
    msg.command_age_s = command_age if command_age != math.inf else -1.0

    return msg
