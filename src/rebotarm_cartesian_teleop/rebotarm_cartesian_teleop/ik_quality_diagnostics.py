"""Pure IK solution quality diagnostics (logging only, no acceptance policy)."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

DEFAULT_JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
)


@dataclass(frozen=True)
class IkQualityLogConfig:
    """Thresholds for warning logs only; never gate IK acceptance."""

    joint_limit_warn_margin_rad: float = 0.35
    joint5_warn_abs_rad: float = 1.0
    joint4_warn_abs_rad: float = 1.0
    q_delta_warn_rad: float = 0.15
    candidate_drift_warn_m: float = 0.003
    reached_step_warn_min_m: float = 0.0001


@dataclass(frozen=True)
class JointQualityDiagnostic:
    name: str
    q_before: float
    q_target: float
    q_delta: float
    abs_q_delta: float
    lower_limit: float
    upper_limit: float
    margin_to_lower: float
    margin_to_upper: float
    nearest_margin: float
    nearest_side: str


@dataclass(frozen=True)
class IkQualityDiagnostics:
    joints: tuple[JointQualityDiagnostic, ...]
    max_abs_q_delta: float
    max_abs_q_delta_joint: str
    nearest_limit_joint: str
    nearest_limit_margin: float
    nearest_limit_side: str
    any_joint_near_limit: bool
    candidate_drift_m: float
    ik_error: float
    fk_position_before: tuple[float, float, float]
    fk_position_target: tuple[float, float, float]
    reached_step_m: float
    candidate_step_m: float
    q_step_norm: float
    posture_distance_from_initial_q: float
    joint4: JointQualityDiagnostic
    joint5: JointQualityDiagnostic
    log_reasons: tuple[str, ...]


def joint_names_from_model(model) -> list[str]:
    """Return revolute joint names in Pinocchio q order."""
    return [str(model.names[i + 1]) for i in range(model.nq)]


def joint_limits_from_model(model) -> tuple[list[float], list[float]]:
    lower = [float(model.lowerPositionLimit[i]) for i in range(model.nq)]
    upper = [float(model.upperPositionLimit[i]) for i in range(model.nq)]
    return lower, upper


def _nearest_side(margin_to_lower: float, margin_to_upper: float) -> str:
    return "lower" if margin_to_lower <= margin_to_upper else "upper"


def _joint_diagnostic(
    name: str,
    q_before: float,
    q_target: float,
    lower_limit: float,
    upper_limit: float,
) -> JointQualityDiagnostic:
    q_delta = q_target - q_before
    margin_to_lower = q_target - lower_limit
    margin_to_upper = upper_limit - q_target
    nearest_margin = min(margin_to_lower, margin_to_upper)
    return JointQualityDiagnostic(
        name=name,
        q_before=q_before,
        q_target=q_target,
        q_delta=q_delta,
        abs_q_delta=abs(q_delta),
        lower_limit=lower_limit,
        upper_limit=upper_limit,
        margin_to_lower=margin_to_lower,
        margin_to_upper=margin_to_upper,
        nearest_margin=nearest_margin,
        nearest_side=_nearest_side(margin_to_lower, margin_to_upper),
    )


def compute_joint_quality_diagnostics(
    joint_names: Sequence[str],
    q_before: Sequence[float],
    q_target: Sequence[float],
    lower_limits: Sequence[float],
    upper_limits: Sequence[float],
    initial_q: Sequence[float],
    *,
    fk_position_before: Sequence[float],
    fk_position_target: Sequence[float],
    candidate_drift_m: float = 0.0,
    ik_error: float = 0.0,
    candidate_step_m: float = 0.0,
    joint_limit_near_rad: float = 0.35,
) -> IkQualityDiagnostics:
    """Build per-joint and global IK quality diagnostics without mutating inputs."""
    qb = np.asarray(q_before, dtype=np.float64).reshape(-1)
    qt = np.asarray(q_target, dtype=np.float64).reshape(-1)
    q0 = np.asarray(initial_q, dtype=np.float64).reshape(-1)
    lo = np.asarray(lower_limits, dtype=np.float64).reshape(-1)
    hi = np.asarray(upper_limits, dtype=np.float64).reshape(-1)

    if not (len(joint_names) == len(qb) == len(qt) == len(lo) == len(hi) == len(q0)):
        raise ValueError("joint_names, q arrays, limits, and initial_q length mismatch")

    joints = tuple(
        _joint_diagnostic(
            str(joint_names[i]),
            float(qb[i]),
            float(qt[i]),
            float(lo[i]),
            float(hi[i]),
        )
        for i in range(len(joint_names))
    )

    max_idx = max(range(len(joints)), key=lambda i: joints[i].abs_q_delta)
    nearest_idx = min(range(len(joints)), key=lambda i: joints[i].nearest_margin)

    pos_before = np.asarray(fk_position_before, dtype=np.float64).reshape(3)
    pos_target = np.asarray(fk_position_target, dtype=np.float64).reshape(3)
    reached_step_m = float(np.linalg.norm(pos_target - pos_before))

    joint_by_name = {j.name: j for j in joints}
    j4_name = "joint4" if "joint4" in joint_by_name else joint_names[3]
    j5_name = "joint5" if "joint5" in joint_by_name else joint_names[4]

    any_near = any(j.nearest_margin < joint_limit_near_rad for j in joints)

    return IkQualityDiagnostics(
        joints=joints,
        max_abs_q_delta=joints[max_idx].abs_q_delta,
        max_abs_q_delta_joint=joints[max_idx].name,
        nearest_limit_joint=joints[nearest_idx].name,
        nearest_limit_margin=joints[nearest_idx].nearest_margin,
        nearest_limit_side=joints[nearest_idx].nearest_side,
        any_joint_near_limit=any_near,
        candidate_drift_m=float(candidate_drift_m),
        ik_error=float(ik_error),
        fk_position_before=(float(pos_before[0]), float(pos_before[1]), float(pos_before[2])),
        fk_position_target=(float(pos_target[0]), float(pos_target[1]), float(pos_target[2])),
        reached_step_m=reached_step_m,
        candidate_step_m=float(candidate_step_m),
        q_step_norm=float(np.linalg.norm(qt - qb)),
        posture_distance_from_initial_q=float(np.linalg.norm(qt - q0)),
        joint4=joint_by_name[j4_name],
        joint5=joint_by_name[j5_name],
        log_reasons=(),
    )


def _collect_log_reasons(diag: IkQualityDiagnostics, config: IkQualityLogConfig) -> tuple[str, ...]:
    reasons: list[str] = []
    if diag.nearest_limit_margin < config.joint_limit_warn_margin_rad:
        reasons.append(
            f"nearest_limit_margin={diag.nearest_limit_margin:.4f}<{config.joint_limit_warn_margin_rad}"
        )
    if abs(diag.joint5.q_target) > config.joint5_warn_abs_rad:
        reasons.append(
            f"|joint5|={abs(diag.joint5.q_target):.4f}>{config.joint5_warn_abs_rad}"
        )
    if abs(diag.joint4.q_target) > config.joint4_warn_abs_rad:
        reasons.append(
            f"|joint4|={abs(diag.joint4.q_target):.4f}>{config.joint4_warn_abs_rad}"
        )
    if diag.max_abs_q_delta > config.q_delta_warn_rad:
        reasons.append(
            f"max_abs_q_delta={diag.max_abs_q_delta:.4f}>{config.q_delta_warn_rad}"
        )
    if diag.candidate_drift_m > config.candidate_drift_warn_m:
        reasons.append(
            f"candidate_drift_m={diag.candidate_drift_m:.6f}>{config.candidate_drift_warn_m}"
        )
    trivial_candidate = diag.candidate_step_m > 1e-4
    if trivial_candidate and diag.reached_step_m < config.reached_step_warn_min_m:
        reasons.append(
            f"reached_step_m={diag.reached_step_m:.6f}<{config.reached_step_warn_min_m}"
            f" while candidate_step_m={diag.candidate_step_m:.6f}"
        )
    return tuple(reasons)


def should_log_ik_quality_diagnostics(
    diag: IkQualityDiagnostics,
    config: IkQualityLogConfig,
    *,
    ik_failure: bool = False,
) -> bool:
    if ik_failure:
        return True
    return bool(_collect_log_reasons(diag, config))


def with_log_reasons(
    diag: IkQualityDiagnostics,
    config: IkQualityLogConfig,
    *,
    ik_failure: bool = False,
) -> IkQualityDiagnostics:
    reasons = list(_collect_log_reasons(diag, config))
    if ik_failure:
        reasons.insert(0, "ik_failure")
    return IkQualityDiagnostics(
        joints=diag.joints,
        max_abs_q_delta=diag.max_abs_q_delta,
        max_abs_q_delta_joint=diag.max_abs_q_delta_joint,
        nearest_limit_joint=diag.nearest_limit_joint,
        nearest_limit_margin=diag.nearest_limit_margin,
        nearest_limit_side=diag.nearest_limit_side,
        any_joint_near_limit=diag.any_joint_near_limit,
        candidate_drift_m=diag.candidate_drift_m,
        ik_error=diag.ik_error,
        fk_position_before=diag.fk_position_before,
        fk_position_target=diag.fk_position_target,
        reached_step_m=diag.reached_step_m,
        candidate_step_m=diag.candidate_step_m,
        q_step_norm=diag.q_step_norm,
        posture_distance_from_initial_q=diag.posture_distance_from_initial_q,
        joint4=diag.joint4,
        joint5=diag.joint5,
        log_reasons=tuple(reasons),
    )


def format_ik_quality_diagnostics(diag: IkQualityDiagnostics) -> str:
    lines = ["IK quality diagnostics:"]
    if diag.log_reasons:
        lines.append(f"  triggers: {', '.join(diag.log_reasons)}")
    lines.extend(
        [
            f"  global: max_abs_q_delta={diag.max_abs_q_delta:.4f} ({diag.max_abs_q_delta_joint})",
            (
                "  global: nearest_limit="
                f"{diag.nearest_limit_joint} margin={diag.nearest_limit_margin:.4f} "
                f"side={diag.nearest_limit_side} any_near={diag.any_joint_near_limit}"
            ),
            (
                "  motion: candidate_step_m="
                f"{diag.candidate_step_m:.6f} reached_step_m={diag.reached_step_m:.6f} "
                f"candidate_drift_m={diag.candidate_drift_m:.6f} ik_error={diag.ik_error:.6f}"
            ),
            (
                "  fk: before="
                f"({diag.fk_position_before[0]:.4f}, {diag.fk_position_before[1]:.4f}, "
                f"{diag.fk_position_before[2]:.4f}) target="
                f"({diag.fk_position_target[0]:.4f}, {diag.fk_position_target[1]:.4f}, "
                f"{diag.fk_position_target[2]:.4f})"
            ),
            (
                f"  posture: q_step_norm={diag.q_step_norm:.4f} "
                f"dist_from_initial_q={diag.posture_distance_from_initial_q:.4f}"
            ),
            (
                "  highlight joint4 (elbow): "
                f"q {diag.joint4.q_before:.4f}->{diag.joint4.q_target:.4f} "
                f"dq={diag.joint4.q_delta:+.4f} nearest_margin={diag.joint4.nearest_margin:.4f} "
                f"side={diag.joint4.nearest_side}"
            ),
            (
                "  highlight joint5 (wrist): "
                f"q {diag.joint5.q_before:.4f}->{diag.joint5.q_target:.4f} "
                f"dq={diag.joint5.q_delta:+.4f} nearest_margin={diag.joint5.nearest_margin:.4f} "
                f"side={diag.joint5.nearest_side}"
            ),
        ]
    )
    for joint in diag.joints:
        lines.append(
            f"  {joint.name}: q {joint.q_before:.4f}->{joint.q_target:.4f} "
            f"dq={joint.q_delta:+.4f} limits=[{joint.lower_limit:.2f},{joint.upper_limit:.2f}] "
            f"margins=(lo+{joint.margin_to_lower:.4f}, hi-{joint.margin_to_upper:.4f}) "
            f"nearest={joint.nearest_margin:.4f} ({joint.nearest_side})"
        )
    return "\n".join(lines)


def pos3_from_pose(pose) -> tuple[float, float, float]:
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    )


def candidate_step_m(
    fk_position_before: Sequence[float],
    candidate_position: Sequence[float],
) -> float:
    before = np.asarray(fk_position_before, dtype=np.float64).reshape(3)
    candidate = np.asarray(candidate_position, dtype=np.float64).reshape(3)
    return float(np.linalg.norm(candidate - before))
