"""Pure Cartesian jog core logic (state machine, integration, clamps)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import Pose
from rebotarm_msgs.msg import CartesianJogCmd, CartesianJogState

from .fk_kinematics import FkContext
from .fk_pose import pose_to_rotation_matrix
from .ik_kinematics import compute_ik_for_pose, joint_delta_within_limit


@dataclass(frozen=True)
class IkConfig:
    max_iterations: int
    tolerance: float
    max_ik_error: float
    max_joint_delta_rad: float


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


def resolve_rejection_reason(state_name: str, fk_error: str, ik_reason: str) -> str:
    state_reason = rejection_reason_for_state(state_name)
    if state_reason:
        return state_reason
    if fk_error:
        return fk_error
    return ik_reason


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


def solve_target_ik(
    *,
    fk_ctx: FkContext,
    state_name: str,
    target_x: float,
    target_y: float,
    target_z: float,
    current_pose: Pose | None,
    ik_config: IkConfig,
    last_q_target: list[float] | None,
) -> tuple[list[float], bool, str, list[float] | None]:
    """Compute q_target from integrated target position and FK orientation."""
    if state_name != "ACTIVE" or not fk_ctx.ok or current_pose is None:
        return [], False, "", last_q_target

    if (
        fk_ctx.model is None
        or fk_ctx.data is None
        or fk_ctx.end_frame_id is None
        or fk_ctx.q_current is None
    ):
        return [], False, "", last_q_target

    target_pos = np.array([target_x, target_y, target_z], dtype=np.float64)
    target_rot = pose_to_rotation_matrix(current_pose)

    if last_q_target is not None and len(last_q_target) == fk_ctx.model.nq:
        q_seed = np.asarray(last_q_target, dtype=np.float64)
    else:
        q_seed = fk_ctx.q_current.copy()

    ik_result = compute_ik_for_pose(
        fk_ctx.model,
        fk_ctx.data,
        fk_ctx.end_frame_id,
        target_pos,
        target_rot,
        q_seed,
        ik_config.max_iterations,
        ik_config.tolerance,
        ik_config.max_ik_error,
    )

    if not ik_result.success:
        return [], False, ik_result.reason, last_q_target

    if len(ik_result.q_target) != fk_ctx.model.nq:
        return [], False, "INVALID_IK_RESULT", last_q_target

    if not joint_delta_within_limit(
        ik_result.q_target,
        q_seed,
        ik_config.max_joint_delta_rad,
    ):
        return [], False, "JOINT_DELTA_TOO_LARGE", last_q_target

    new_last = list(ik_result.q_target)
    return new_last, True, "", new_last


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
    current_pose: Pose | None = None,
    q_current: list[float] | None = None,
    q_target: list[float] | None = None,
    ik_success: bool = False,
    fk_error: str = "",
    ik_reason: str = "",
) -> CartesianJogState:
    msg = CartesianJogState()
    msg.header.frame_id = "base_link"
    msg.state = state_name

    if current_pose is not None:
        msg.current_pose = current_pose
    else:
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

    msg.q_current = [float(v) for v in q_current] if q_current is not None else []
    msg.q_target = [float(v) for v in q_target] if q_target is not None else []
    msg.ik_success = bool(ik_success)
    msg.rejection_reason = resolve_rejection_reason(state_name, fk_error, ik_reason)
    msg.clamp_reason = clamp_reason
    msg.dry_run = dry_run
    msg.output_mode = output_mode
    msg.command_age_s = command_age if command_age != math.inf else -1.0

    return msg
