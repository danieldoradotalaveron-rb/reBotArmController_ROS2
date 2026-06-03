"""Pure IK helper using reBotArm_control_py (no hardware, no retry)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .sdk_path import ensure_rebot_sdk_in_syspath


@dataclass
class IkSolveResult:
    success: bool
    q_target: list[float]
    error: float
    iterations: int
    reason: str


def _failure(reason: str, error: float = 0.0, iterations: int = 0) -> IkSolveResult:
    return IkSolveResult(
        success=False,
        q_target=[],
        error=error,
        iterations=iterations,
        reason=reason,
    )


def compute_ik_for_pose(
    model: Any,
    data: Any,
    end_frame_id: int,
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    q_seed: np.ndarray,
    max_iterations: int,
    tolerance: float,
    max_ik_error: float,
) -> IkSolveResult:
    """Solve IK for a Cartesian target pose (deterministic, no retry).

    SDK ``result.success`` reflects convergence against ``ik_tolerance`` (strict).
    For dry-run Cartesian teleop we accept any solution whose final ``error`` is
    within ``max_ik_error``, even when the SDK marks ``success=False``. Hardware
    output remains disabled upstream.
    """
    ensure_rebot_sdk_in_syspath()
    from reBotArm_control_py.kinematics.inverse_kinematics import IKParams, pos_rot_to_se3, solve_ik

    try:
        pos = np.asarray(target_position, dtype=np.float64).reshape(3)
        rot = np.asarray(target_rotation, dtype=np.float64).reshape(3, 3)
        q_init = np.asarray(q_seed, dtype=np.float64).reshape(model.nq)
        target = pos_rot_to_se3(pos, rot)
        params = IKParams(max_iter=int(max_iterations), tolerance=float(tolerance))
        result = solve_ik(model, data, end_frame_id, target, q_init, params)
    except Exception:
        return _failure("IK_EXCEPTION")

    error = float(result.error)
    iterations = int(result.iterations)

    if len(result.q) != model.nq:
        return _failure("INVALID_IK_RESULT", error=error, iterations=iterations)

    if error > float(max_ik_error):
        return _failure("IK_ERROR_TOO_HIGH", error=error, iterations=iterations)

    return IkSolveResult(
        success=True,
        q_target=[float(v) for v in result.q],
        error=error,
        iterations=iterations,
        reason="",
    )


def joint_delta_within_limit(
    q_target: list[float],
    q_seed: np.ndarray,
    max_joint_delta_rad: float,
) -> bool:
    q_t = np.asarray(q_target, dtype=np.float64)
    q_s = np.asarray(q_seed, dtype=np.float64).reshape(q_t.shape)
    return float(np.max(np.abs(q_t - q_s))) <= float(max_joint_delta_rad)
