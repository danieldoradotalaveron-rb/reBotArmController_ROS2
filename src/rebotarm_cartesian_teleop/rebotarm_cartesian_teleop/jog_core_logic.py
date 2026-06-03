"""Pure Cartesian jog core logic (state machine, integration, clamps)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from geometry_msgs.msg import Pose
from rebotarm_msgs.msg import CartesianJogCmd, CartesianJogState

from .fk_kinematics import FkContext, compute_fk_pose_for_q
from .fk_pose import fk_arrays_to_pose
from .ik_kinematics import compute_ik_for_pose, joint_delta_within_limit


@dataclass(frozen=True)
class IkConfig:
    max_iterations: int
    tolerance: float
    max_ik_error: float
    max_joint_delta_rad: float


@dataclass(frozen=True)
class IkFailureDiagnostics:
    rejection_reason: str
    candidate_target: tuple[float, float, float]
    seed_q: tuple[float, ...]
    ik_error: float | None
    ik_iterations: int | None
    max_ik_error: float
    max_joint_delta_rad: float
    clamp_reason: str
    state: str
    target_rotation_from_fk: bool
    committed_target: tuple[float, float, float] | None = None


def format_ik_failure_log(diag: IkFailureDiagnostics) -> str:
    cx, cy, cz = diag.committed_target if diag.committed_target is not None else (0.0, 0.0, 0.0)
    tx, ty, tz = diag.candidate_target
    committed_str = (
        f"({cx:.4f}, {cy:.4f}, {cz:.4f})" if diag.committed_target is not None else "n/a"
    )
    seed_str = ", ".join(f"{v:.4f}" for v in diag.seed_q)
    ik_error_str = f"{diag.ik_error:.6f}" if diag.ik_error is not None else "n/a"
    ik_iter_str = str(diag.ik_iterations) if diag.ik_iterations is not None else "n/a"
    clamp_str = diag.clamp_reason if diag.clamp_reason else "(none)"
    rot_src = "FK(q_sim)" if diag.target_rotation_from_fk else "other"
    return (
        f"IK failure: reason={diag.rejection_reason} state={diag.state} "
        f"committed={committed_str} candidate=({tx:.4f}, {ty:.4f}, {tz:.4f}) "
        f"seed_q=[{seed_str}] ik_error={ik_error_str} ik_iterations={ik_iter_str} "
        f"max_ik_error={diag.max_ik_error:.6f} max_joint_delta_rad={diag.max_joint_delta_rad:.4f} "
        f"clamp_reason={clamp_str} target_rotation={rot_src}"
    )


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


def compute_candidate_target(
    sim_x: float,
    sim_y: float,
    sim_z: float,
    latest_cmd: CartesianJogCmd,
    dt: float,
    workspace: WorkspaceLimits,
) -> tuple[float, float, float, str]:
    """Integrate joystick delta from FK(q_sim) position into a candidate target."""
    vx = float(latest_cmd.linear.x)
    vy = float(latest_cmd.linear.y)
    vz = float(latest_cmd.linear.z)

    candidate_x = sim_x + vx * dt
    candidate_y = sim_y + vy * dt
    candidate_z = sim_z + vz * dt

    clamp_reasons: list[str] = []

    candidate_x, clamped_x = clamp(candidate_x, workspace.x_min, workspace.x_max)
    candidate_y, clamped_y = clamp(candidate_y, workspace.y_min, workspace.y_max)
    candidate_z, clamped_z = clamp(candidate_z, workspace.z_min, workspace.z_max)

    if clamped_x:
        clamp_reasons.append("WORKSPACE_X")
    if clamped_y:
        clamp_reasons.append("WORKSPACE_Y")
    if clamped_z:
        clamp_reasons.append("WORKSPACE_Z")

    return candidate_x, candidate_y, candidate_z, ",".join(clamp_reasons)


def commit_target_on_ik_success(
    committed_x: float,
    committed_y: float,
    committed_z: float,
    candidate_x: float,
    candidate_y: float,
    candidate_z: float,
    ik_success: bool,
) -> tuple[float, float, float]:
    """Legacy helper: commit candidate on success (superseded by resync_committed_from_q_sim)."""
    if ik_success:
        return candidate_x, candidate_y, candidate_z
    return committed_x, committed_y, committed_z


def update_q_sim_on_ik_success(
    q_sim: np.ndarray,
    candidate_q: list[float],
    ik_success: bool,
) -> np.ndarray:
    if ik_success and candidate_q:
        return np.asarray(candidate_q, dtype=np.float64).reshape(q_sim.shape)
    return np.asarray(q_sim, dtype=np.float64)


def compute_candidate_drift_m(
    fk_ctx: FkContext,
    candidate_x: float,
    candidate_y: float,
    candidate_z: float,
    candidate_q: list[float],
) -> float:
    """Position drift between ideal candidate and FK(candidate_q) (diagnostics only)."""
    if not candidate_q:
        return 0.0
    pose, _, err = compute_fk_pose_for_q(fk_ctx, np.asarray(candidate_q, dtype=np.float64))
    if err or pose is None:
        return 0.0
    dx = candidate_x - float(pose.position.x)
    dy = candidate_y - float(pose.position.y)
    dz = candidate_z - float(pose.position.z)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def resync_committed_from_q_sim(
    fk_ctx: FkContext,
    q_sim: np.ndarray,
) -> tuple[float, float, float, np.ndarray | None, Pose | None, str]:
    """Set committed pose from FK(q_sim). Always used after accepted IK."""
    pose, rotation, err = compute_fk_pose_for_q(fk_ctx, q_sim)
    if err or pose is None or rotation is None:
        return 0.0, 0.0, 0.0, None, None, err or "FK_NOT_READY"
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        rotation,
        pose,
        "",
    )


def build_committed_target_pose(
    x: float,
    y: float,
    z: float,
    rotation: np.ndarray | None,
) -> Pose:
    if rotation is not None:
        pos = np.array([x, y, z], dtype=np.float64)
        return fk_arrays_to_pose(pos, rotation)
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation.w = 1.0
    return pose


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

    return compute_candidate_target(target_x, target_y, target_z, latest_cmd, dt, workspace)


def _ik_failure_diagnostics(
    *,
    reason: str,
    candidate_x: float,
    candidate_y: float,
    candidate_z: float,
    q_seed: np.ndarray,
    ik_config: IkConfig,
    state_name: str,
    clamp_reason: str,
    committed_x: float | None = None,
    committed_y: float | None = None,
    committed_z: float | None = None,
    ik_error: float | None = None,
    ik_iterations: int | None = None,
) -> IkFailureDiagnostics:
    committed = None
    if committed_x is not None and committed_y is not None and committed_z is not None:
        committed = (committed_x, committed_y, committed_z)
    return IkFailureDiagnostics(
        rejection_reason=reason,
        candidate_target=(candidate_x, candidate_y, candidate_z),
        seed_q=tuple(float(v) for v in q_seed),
        ik_error=ik_error,
        ik_iterations=ik_iterations,
        max_ik_error=ik_config.max_ik_error,
        max_joint_delta_rad=ik_config.max_joint_delta_rad,
        clamp_reason=clamp_reason,
        state=state_name,
        target_rotation_from_fk=True,
        committed_target=committed,
    )


def solve_target_ik(
    *,
    fk_ctx: FkContext,
    state_name: str,
    target_x: float,
    target_y: float,
    target_z: float,
    target_rotation: np.ndarray,
    q_seed: np.ndarray,
    ik_config: IkConfig,
    clamp_reason: str = "",
    committed_x: float | None = None,
    committed_y: float | None = None,
    committed_z: float | None = None,
) -> tuple[list[float], bool, str, IkFailureDiagnostics | None]:
    """Compute q_target from candidate position and FK(q_sim) orientation."""
    if state_name != "ACTIVE" or not fk_ctx.ok:
        return [], False, "", None

    if (
        fk_ctx.model is None
        or fk_ctx.data is None
        or fk_ctx.end_frame_id is None
    ):
        return [], False, "", None

    target_pos = np.array([target_x, target_y, target_z], dtype=np.float64)
    target_rot = np.asarray(target_rotation, dtype=np.float64).reshape(3, 3)
    q_seed_arr = np.asarray(q_seed, dtype=np.float64).reshape(fk_ctx.model.nq)

    ik_result = compute_ik_for_pose(
        fk_ctx.model,
        fk_ctx.data,
        fk_ctx.end_frame_id,
        target_pos,
        target_rot,
        q_seed_arr,
        ik_config.max_iterations,
        ik_config.tolerance,
        ik_config.max_ik_error,
    )

    if not ik_result.success:
        diag = _ik_failure_diagnostics(
            reason=ik_result.reason,
            candidate_x=target_x,
            candidate_y=target_y,
            candidate_z=target_z,
            q_seed=q_seed_arr,
            ik_config=ik_config,
            state_name=state_name,
            clamp_reason=clamp_reason,
            committed_x=committed_x,
            committed_y=committed_y,
            committed_z=committed_z,
            ik_error=ik_result.error,
            ik_iterations=ik_result.iterations,
        )
        return [], False, ik_result.reason, diag

    if len(ik_result.q_target) != fk_ctx.model.nq:
        diag = _ik_failure_diagnostics(
            reason="INVALID_IK_RESULT",
            candidate_x=target_x,
            candidate_y=target_y,
            candidate_z=target_z,
            q_seed=q_seed_arr,
            ik_config=ik_config,
            state_name=state_name,
            clamp_reason=clamp_reason,
            committed_x=committed_x,
            committed_y=committed_y,
            committed_z=committed_z,
            ik_error=ik_result.error,
            ik_iterations=ik_result.iterations,
        )
        return [], False, "INVALID_IK_RESULT", diag

    if not joint_delta_within_limit(
        ik_result.q_target,
        q_seed_arr,
        ik_config.max_joint_delta_rad,
    ):
        diag = _ik_failure_diagnostics(
            reason="JOINT_DELTA_TOO_LARGE",
            candidate_x=target_x,
            candidate_y=target_y,
            candidate_z=target_z,
            q_seed=q_seed_arr,
            ik_config=ik_config,
            state_name=state_name,
            clamp_reason=clamp_reason,
            committed_x=committed_x,
            committed_y=committed_y,
            committed_z=committed_z,
            ik_error=ik_result.error,
            ik_iterations=ik_result.iterations,
        )
        return [], False, "JOINT_DELTA_TOO_LARGE", diag

    return list(ik_result.q_target), True, "", None


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
    target_pose: Pose | None = None,
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

    if target_pose is not None:
        msg.target_pose = target_pose
    else:
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
