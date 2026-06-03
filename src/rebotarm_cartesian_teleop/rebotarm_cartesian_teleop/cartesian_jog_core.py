import math

import numpy as np
import rclpy
from rclpy.node import Node
from rebotarm_msgs.msg import CartesianJogCmd, CartesianJogState
from sensor_msgs.msg import JointState

from .fake_joint_state import build_fake_joint_state
from .fk_kinematics import (
    FkContext,
    compute_fk_pose_for_q,
    init_fk_context,
)
from .ik_kinematics import compute_ik_for_pose
from .ik_quality_diagnostics import (
    IkQualityLogConfig,
    candidate_step_m,
    compute_joint_quality_diagnostics,
    format_ik_quality_diagnostics,
    joint_limits_from_model,
    joint_names_from_model,
    pos3_from_pose,
    should_log_ik_quality_diagnostics,
    with_log_reasons,
)
from .jog_core_logic import (
    IkConfig,
    IkNoEffectConfig,
    JointLimitRejectConfig,
    WorkspaceLimits,
    build_cartesian_jog_state,
    build_committed_target_pose,
    compute_candidate_drift_m,
    compute_candidate_target,
    compute_state_name,
    format_ik_failure_log,
    format_joint_near_limit_log,
    reject_ik_if_near_joint_limit,
    reject_ik_if_no_effect,
    resync_committed_from_q_sim,
    solve_target_ik,
    update_q_sim_on_ik_success,
)


class CartesianJogCore(Node):
    def __init__(self):
        super().__init__("cartesian_jog_core")

        self.declare_parameter("cartesian_jog_cmd_topic", "/rebotarm/cartesian_jog_cmd")
        self.declare_parameter("cartesian_jog_state_topic", "/rebotarm/cartesian_jog_state")
        self.declare_parameter("output_mode", "dry_run")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("command_timeout_s", 0.3)
        self.declare_parameter("servo_hz", 50.0)

        self.declare_parameter("initial_x", 0.30)
        self.declare_parameter("initial_y", 0.00)
        self.declare_parameter("initial_z", 0.20)

        self.declare_parameter("workspace_x_min", 0.15)
        self.declare_parameter("workspace_x_max", 0.45)
        self.declare_parameter("workspace_y_min", -0.25)
        self.declare_parameter("workspace_y_max", 0.25)
        self.declare_parameter("workspace_z_min", 0.05)
        self.declare_parameter("workspace_z_max", 0.45)

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("ee_frame", "end_link")
        self.declare_parameter("initial_q", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.declare_parameter("ik_max_iterations", 100)
        self.declare_parameter("ik_tolerance", 0.001)
        self.declare_parameter("max_ik_error", 0.005)
        self.declare_parameter("max_joint_delta_rad", 0.25)
        self.declare_parameter("ik_failure_log_interval_s", 1.0)
        self.declare_parameter("candidate_drift_log_threshold_m", 0.001)
        self.declare_parameter("joint_limit_warn_margin_rad", 0.35)
        self.declare_parameter("joint_limit_reject_margin_rad", 0.05)
        self.declare_parameter("joint5_warn_abs_rad", 1.0)
        self.declare_parameter("joint4_warn_abs_rad", 1.0)
        self.declare_parameter("q_delta_warn_rad", 0.15)
        self.declare_parameter("candidate_drift_warn_m", 0.003)
        self.declare_parameter("reached_step_warn_min_m", 0.0001)
        self.declare_parameter("ik_quality_log_interval_s", 1.0)
        self.declare_parameter("ik_no_effect_candidate_step_min_m", 0.0005)
        self.declare_parameter("ik_no_effect_reached_step_min_m", 0.0001)
        self.declare_parameter("ik_no_effect_q_step_min_norm", 1.0e-6)
        self.declare_parameter("publish_fake_joint_states", True)
        self.declare_parameter("fake_joint_states_topic", "/rebotarm/fake_joint_states")
        self.declare_parameter("fake_joint_state_hz", 50.0)

        cmd_topic = self.get_parameter("cartesian_jog_cmd_topic").value
        state_topic = self.get_parameter("cartesian_jog_state_topic").value

        self.output_mode = self.get_parameter("output_mode").value
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.servo_hz = float(self.get_parameter("servo_hz").value)

        self._workspace = WorkspaceLimits(
            x_min=float(self.get_parameter("workspace_x_min").value),
            x_max=float(self.get_parameter("workspace_x_max").value),
            y_min=float(self.get_parameter("workspace_y_min").value),
            y_max=float(self.get_parameter("workspace_y_max").value),
            z_min=float(self.get_parameter("workspace_z_min").value),
            z_max=float(self.get_parameter("workspace_z_max").value),
        )

        urdf_path = str(self.get_parameter("urdf_path").value)
        ee_frame = str(self.get_parameter("ee_frame").value)
        initial_q = [float(v) for v in self.get_parameter("initial_q").value]
        self._initial_q = np.asarray(initial_q, dtype=np.float64)

        self._fk: FkContext = init_fk_context(urdf_path, ee_frame, initial_q)
        if self._fk.ok:
            self.get_logger().info(
                f"FK model loaded (nq={self._fk.model.nq}, frame={self._fk.ee_frame})"
            )
            if self._fk.q_current is not None:
                q_str = ", ".join(f"{v:.4f}" for v in self._fk.q_current)
                self.get_logger().info(f"initial_q: [{q_str}]")
        else:
            self.get_logger().error(f"FK init failed: {self._fk.error}")

        if self._fk.ok and self._fk.q_current is not None:
            self._q_sim = np.asarray(self._fk.q_current, dtype=np.float64).copy()
        else:
            self._q_sim = np.zeros(len(initial_q), dtype=np.float64)

        (
            self.committed_target_x,
            self.committed_target_y,
            self.committed_target_z,
            self._committed_rotation,
            _,
            fk_init_err,
        ) = resync_committed_from_q_sim(self._fk, self._q_sim)
        if fk_init_err:
            fallback_x = float(self.get_parameter("initial_x").value)
            fallback_y = float(self.get_parameter("initial_y").value)
            fallback_z = float(self.get_parameter("initial_z").value)
            self.committed_target_x = fallback_x
            self.committed_target_y = fallback_y
            self.committed_target_z = fallback_z
            self.get_logger().warn(
                f"FK(q_sim) init failed ({fk_init_err}); using YAML fallback target"
            )
        else:
            self.get_logger().info(
                "Initial committed target from FK(q_sim): "
                f"x={self.committed_target_x:.3f}, y={self.committed_target_y:.3f}, "
                f"z={self.committed_target_z:.3f}"
            )

        self.latest_cmd = None
        self.latest_cmd_time_ns = None
        self.last_tick_time_ns = self.get_clock().now().nanoseconds
        self.last_clamp_reason = ""
        self._fk_tick_error = ""
        self._ik_failure_log_interval_s = float(
            self.get_parameter("ik_failure_log_interval_s").value
        )
        self._candidate_drift_log_threshold_m = float(
            self.get_parameter("candidate_drift_log_threshold_m").value
        )
        self._ik_quality_log_config = IkQualityLogConfig(
            joint_limit_warn_margin_rad=float(
                self.get_parameter("joint_limit_warn_margin_rad").value
            ),
            joint5_warn_abs_rad=float(self.get_parameter("joint5_warn_abs_rad").value),
            joint4_warn_abs_rad=float(self.get_parameter("joint4_warn_abs_rad").value),
            q_delta_warn_rad=float(self.get_parameter("q_delta_warn_rad").value),
            candidate_drift_warn_m=float(self.get_parameter("candidate_drift_warn_m").value),
            reached_step_warn_min_m=float(self.get_parameter("reached_step_warn_min_m").value),
        )
        self._ik_quality_log_interval_s = float(
            self.get_parameter("ik_quality_log_interval_s").value
        )
        self._ik_no_effect_config = IkNoEffectConfig(
            candidate_step_min_m=float(
                self.get_parameter("ik_no_effect_candidate_step_min_m").value
            ),
            reached_step_min_m=float(
                self.get_parameter("ik_no_effect_reached_step_min_m").value
            ),
            q_step_min_norm=float(self.get_parameter("ik_no_effect_q_step_min_norm").value),
        )
        self._joint_limit_reject_config = JointLimitRejectConfig(
            reject_margin_rad=float(self.get_parameter("joint_limit_reject_margin_rad").value),
        )
        self._last_ik_failure_log_ns = 0
        self._last_ik_quality_log_ns = 0

        self._joint_names: list[str] = []
        self._joint_lower_limits: list[float] = []
        self._joint_upper_limits: list[float] = []
        if self._fk.ok and self._fk.model is not None:
            self._joint_names = joint_names_from_model(self._fk.model)
            lo, hi = joint_limits_from_model(self._fk.model)
            self._joint_lower_limits = lo
            self._joint_upper_limits = hi

        self._publish_fake_joint_states = bool(
            self.get_parameter("publish_fake_joint_states").value
        )
        fake_joint_states_topic = str(self.get_parameter("fake_joint_states_topic").value)
        self._fake_joint_state_hz = float(self.get_parameter("fake_joint_state_hz").value)
        self._last_valid_fake_q: list[float] = [float(v) for v in self._q_sim]

        self._ik_config = IkConfig(
            max_iterations=int(self.get_parameter("ik_max_iterations").value),
            tolerance=float(self.get_parameter("ik_tolerance").value),
            max_ik_error=float(self.get_parameter("max_ik_error").value),
            max_joint_delta_rad=float(self.get_parameter("max_joint_delta_rad").value),
        )

        self.subscription = self.create_subscription(
            CartesianJogCmd,
            cmd_topic,
            self.on_cmd,
            10,
        )

        self.publisher = self.create_publisher(
            CartesianJogState,
            state_topic,
            10,
        )

        self._fake_joint_states_publisher = None
        self._fake_joint_states_timer = None
        if self._publish_fake_joint_states:
            self._fake_joint_states_publisher = self.create_publisher(
                JointState,
                fake_joint_states_topic,
                10,
            )
            self._fake_joint_states_timer = self.create_timer(
                1.0 / self._fake_joint_state_hz,
                self._publish_fake_joint_state,
            )

        self.timer = self.create_timer(1.0 / self.servo_hz, self.tick)

        self.get_logger().info("cartesian_jog_core started")
        self.get_logger().info(f"Listening to: {cmd_topic}")
        self.get_logger().info(f"Publishing to: {state_topic}")
        self.get_logger().info(f"Output mode: {self.output_mode}")
        self.get_logger().info(f"Dry run: {self.dry_run}")
        if self._publish_fake_joint_states:
            self.get_logger().info(
                f"Publishing fake joint states to: {fake_joint_states_topic} "
                f"at {self._fake_joint_state_hz:.1f} Hz"
            )

    def on_cmd(self, msg: CartesianJogCmd):
        self.latest_cmd = msg
        self.latest_cmd_time_ns = self.get_clock().now().nanoseconds

    def get_command_age(self) -> float:
        if self.latest_cmd_time_ns is None:
            return math.inf

        now_ns = self.get_clock().now().nanoseconds
        return (now_ns - self.latest_cmd_time_ns) / 1e9

    def _fk_error_reason(self) -> str:
        if not self._fk.ok:
            return self._fk.error
        return self._fk_tick_error

    def _pose_from_q_sim(self):
        fk_error = self._fk_error_reason()
        if fk_error:
            return None, None, fk_error

        pose, rotation, tick_error = compute_fk_pose_for_q(self._fk, self._q_sim)
        if tick_error:
            self._fk_tick_error = tick_error
            return None, None, tick_error
        self._fk_tick_error = ""
        return pose, rotation, ""

    def _maybe_log_ik_failure(self, diag, now_ns: int) -> None:
        if diag is None:
            return
        interval_ns = int(self._ik_failure_log_interval_s * 1e9)
        if now_ns - self._last_ik_failure_log_ns < interval_ns:
            return
        self._last_ik_failure_log_ns = now_ns
        self.get_logger().warn(format_ik_failure_log(diag))

    def _maybe_log_joint_near_limit(self, info, now_ns: int) -> None:
        if info is None:
            return
        interval_ns = int(self._ik_failure_log_interval_s * 1e9)
        if now_ns - self._last_ik_failure_log_ns < interval_ns:
            return
        self._last_ik_failure_log_ns = now_ns
        self.get_logger().warn(format_joint_near_limit_log(info))

    def _maybe_log_ik_quality(
        self,
        *,
        q_before: np.ndarray,
        q_target: np.ndarray,
        fk_position_before: tuple[float, float, float],
        fk_position_target: tuple[float, float, float],
        candidate_x: float,
        candidate_y: float,
        candidate_z: float,
        candidate_drift_m: float,
        now_ns: int,
        ik_failure: bool = False,
        resolve_ik_error=None,
    ) -> None:
        if not self._joint_names:
            return

        cand_step = candidate_step_m(
            fk_position_before,
            (candidate_x, candidate_y, candidate_z),
        )
        diag = compute_joint_quality_diagnostics(
            self._joint_names,
            q_before,
            q_target,
            self._joint_lower_limits,
            self._joint_upper_limits,
            self._initial_q,
            fk_position_before=fk_position_before,
            fk_position_target=fk_position_target,
            candidate_drift_m=candidate_drift_m,
            ik_error=0.0,
            candidate_step_m=cand_step,
            joint_limit_near_rad=self._ik_quality_log_config.joint_limit_warn_margin_rad,
        )
        if not should_log_ik_quality_diagnostics(
            diag, self._ik_quality_log_config, ik_failure=ik_failure
        ):
            return

        interval_ns = int(self._ik_quality_log_interval_s * 1e9)
        if now_ns - self._last_ik_quality_log_ns < interval_ns:
            return
        self._last_ik_quality_log_ns = now_ns

        ik_error = 0.0
        if resolve_ik_error is not None:
            ik_error = float(resolve_ik_error())
        diag = compute_joint_quality_diagnostics(
            self._joint_names,
            q_before,
            q_target,
            self._joint_lower_limits,
            self._joint_upper_limits,
            self._initial_q,
            fk_position_before=fk_position_before,
            fk_position_target=fk_position_target,
            candidate_drift_m=candidate_drift_m,
            ik_error=ik_error,
            candidate_step_m=cand_step,
            joint_limit_near_rad=self._ik_quality_log_config.joint_limit_warn_margin_rad,
        )
        diag = with_log_reasons(
            diag, self._ik_quality_log_config, ik_failure=ik_failure
        )
        self.get_logger().warn(format_ik_quality_diagnostics(diag))

    def _resolve_ik_error_for_log(
        self,
        *,
        ik_failure: bool,
        ik_failure_diag,
        candidate_x: float,
        candidate_y: float,
        candidate_z: float,
        sim_rotation: np.ndarray | None,
        q_seed: np.ndarray,
    ) -> float:
        if ik_failure and ik_failure_diag is not None and ik_failure_diag.ik_error is not None:
            return float(ik_failure_diag.ik_error)
        if (
            not self._fk.ok
            or self._fk.model is None
            or self._fk.data is None
            or self._fk.end_frame_id is None
            or sim_rotation is None
        ):
            return 0.0
        ik_result = compute_ik_for_pose(
            self._fk.model,
            self._fk.data,
            self._fk.end_frame_id,
            np.array([candidate_x, candidate_y, candidate_z], dtype=np.float64),
            sim_rotation,
            q_seed,
            self._ik_config.max_iterations,
            self._ik_config.tolerance,
            self._ik_config.max_ik_error,
        )
        return float(ik_result.error)

    def _publish_fake_joint_state(self) -> None:
        if self._fake_joint_states_publisher is None:
            return
        msg = build_fake_joint_state(self._last_valid_fake_q)
        msg.header.stamp = self.get_clock().now().to_msg()
        self._fake_joint_states_publisher.publish(msg)

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        dt = (now_ns - self.last_tick_time_ns) / 1e9
        self.last_tick_time_ns = now_ns

        command_age = self.get_command_age()
        state_name = compute_state_name(
            self.latest_cmd,
            command_age,
            self.command_timeout_s,
        )

        self.last_clamp_reason = ""

        current_pose, sim_rotation, fk_error = self._pose_from_q_sim()
        q_current_list = [float(v) for v in self._q_sim]

        sim_x = self.committed_target_x
        sim_y = self.committed_target_y
        sim_z = self.committed_target_z
        if current_pose is not None:
            sim_x = float(current_pose.position.x)
            sim_y = float(current_pose.position.y)
            sim_z = float(current_pose.position.z)

        candidate_x = sim_x
        candidate_y = sim_y
        candidate_z = sim_z
        if state_name == "ACTIVE" and self.latest_cmd is not None:
            candidate_x, candidate_y, candidate_z, self.last_clamp_reason = (
                compute_candidate_target(
                    sim_x,
                    sim_y,
                    sim_z,
                    self.latest_cmd,
                    dt,
                    self._workspace,
                )
            )

        q_target: list[float] = []
        ik_success = False
        ik_reason = ""
        ik_failure_diag = None
        q_before_ik = np.asarray(self._q_sim, dtype=np.float64).copy()
        fk_position_before = (
            pos3_from_pose(current_pose) if current_pose is not None else (sim_x, sim_y, sim_z)
        )

        if (
            state_name == "ACTIVE"
            and current_pose is not None
            and sim_rotation is not None
        ):
            q_target, ik_success, ik_reason, ik_failure_diag = solve_target_ik(
                fk_ctx=self._fk,
                state_name=state_name,
                target_x=candidate_x,
                target_y=candidate_y,
                target_z=candidate_z,
                target_rotation=sim_rotation,
                q_seed=self._q_sim,
                ik_config=self._ik_config,
                clamp_reason=self.last_clamp_reason,
                committed_x=self.committed_target_x,
                committed_y=self.committed_target_y,
                committed_z=self.committed_target_z,
            )
            self._maybe_log_ik_failure(ik_failure_diag, now_ns)

            if not ik_success:
                self._maybe_log_ik_quality(
                    q_before=q_before_ik,
                    q_target=q_before_ik,
                    fk_position_before=fk_position_before,
                    fk_position_target=fk_position_before,
                    candidate_x=candidate_x,
                    candidate_y=candidate_y,
                    candidate_z=candidate_z,
                    candidate_drift_m=0.0,
                    now_ns=now_ns,
                    ik_failure=True,
                    resolve_ik_error=lambda: self._resolve_ik_error_for_log(
                        ik_failure=True,
                        ik_failure_diag=ik_failure_diag,
                        candidate_x=candidate_x,
                        candidate_y=candidate_y,
                        candidate_z=candidate_z,
                        sim_rotation=sim_rotation,
                        q_seed=q_before_ik,
                    ),
                )

        if ik_success:
            q_target, ik_success, ik_reason, joint_limit_info = reject_ik_if_near_joint_limit(
                q_target,
                self._joint_names,
                self._joint_lower_limits,
                self._joint_upper_limits,
                self._joint_limit_reject_config,
            )
            if not ik_success:
                self._maybe_log_joint_near_limit(joint_limit_info, now_ns)
                self._maybe_log_ik_quality(
                    q_before=q_before_ik,
                    q_target=np.asarray(q_before_ik, dtype=np.float64),
                    fk_position_before=fk_position_before,
                    fk_position_target=fk_position_before,
                    candidate_x=candidate_x,
                    candidate_y=candidate_y,
                    candidate_z=candidate_z,
                    candidate_drift_m=0.0,
                    now_ns=now_ns,
                    ik_failure=True,
                    resolve_ik_error=lambda: self._resolve_ik_error_for_log(
                        ik_failure=True,
                        ik_failure_diag=None,
                        candidate_x=candidate_x,
                        candidate_y=candidate_y,
                        candidate_z=candidate_z,
                        sim_rotation=sim_rotation,
                        q_seed=q_before_ik,
                    ),
                )

        if ik_success:
            q_target, ik_success, ik_reason, _ = reject_ik_if_no_effect(
                self._fk,
                q_before_ik,
                q_target,
                candidate_x,
                candidate_y,
                candidate_z,
                self._ik_no_effect_config,
                fk_position_before=fk_position_before,
            )
            if not ik_success:
                self._maybe_log_ik_quality(
                    q_before=q_before_ik,
                    q_target=q_before_ik,
                    fk_position_before=fk_position_before,
                    fk_position_target=fk_position_before,
                    candidate_x=candidate_x,
                    candidate_y=candidate_y,
                    candidate_z=candidate_z,
                    candidate_drift_m=0.0,
                    now_ns=now_ns,
                    ik_failure=True,
                    resolve_ik_error=lambda: self._resolve_ik_error_for_log(
                        ik_failure=True,
                        ik_failure_diag=None,
                        candidate_x=candidate_x,
                        candidate_y=candidate_y,
                        candidate_z=candidate_z,
                        sim_rotation=sim_rotation,
                        q_seed=q_before_ik,
                    ),
                )

        if ik_success:
            drift_m = compute_candidate_drift_m(
                self._fk,
                candidate_x,
                candidate_y,
                candidate_z,
                q_target,
            )
            q_target_arr = np.asarray(q_target, dtype=np.float64)
            fk_target_pose, _, fk_target_err = compute_fk_pose_for_q(self._fk, q_target_arr)
            fk_position_target = (
                pos3_from_pose(fk_target_pose)
                if fk_target_pose is not None and not fk_target_err
                else fk_position_before
            )
            self._maybe_log_ik_quality(
                q_before=q_before_ik,
                q_target=q_target_arr,
                fk_position_before=fk_position_before,
                fk_position_target=fk_position_target,
                candidate_x=candidate_x,
                candidate_y=candidate_y,
                candidate_z=candidate_z,
                candidate_drift_m=drift_m,
                now_ns=now_ns,
                ik_failure=False,
                resolve_ik_error=lambda: self._resolve_ik_error_for_log(
                    ik_failure=False,
                    ik_failure_diag=None,
                    candidate_x=candidate_x,
                    candidate_y=candidate_y,
                    candidate_z=candidate_z,
                    sim_rotation=sim_rotation,
                    q_seed=q_before_ik,
                ),
            )
            self._q_sim = update_q_sim_on_ik_success(self._q_sim, q_target, True)
            (
                self.committed_target_x,
                self.committed_target_y,
                self.committed_target_z,
                self._committed_rotation,
                current_pose,
                fk_resync_err,
            ) = resync_committed_from_q_sim(self._fk, self._q_sim)
            if fk_resync_err:
                self._fk_tick_error = fk_resync_err
            else:
                self._fk_tick_error = ""
            if drift_m > self._candidate_drift_log_threshold_m:
                self.get_logger().debug(
                    f"IK candidate drift (log only): {drift_m:.6f} m "
                    f"(threshold {self._candidate_drift_log_threshold_m:.6f} m)"
                )
            q_current_list = [float(v) for v in self._q_sim]
            self._last_valid_fake_q = q_current_list
            q_target = q_current_list

        committed_target_pose = build_committed_target_pose(
            self.committed_target_x,
            self.committed_target_y,
            self.committed_target_z,
            self._committed_rotation,
        )

        msg = build_cartesian_jog_state(
            state_name=state_name,
            target_x=self.committed_target_x,
            target_y=self.committed_target_y,
            target_z=self.committed_target_z,
            latest_cmd=self.latest_cmd,
            clamp_reason=self.last_clamp_reason,
            dry_run=self.dry_run,
            output_mode=self.output_mode,
            command_age=command_age,
            current_pose=current_pose,
            target_pose=committed_target_pose,
            q_current=q_current_list,
            q_target=q_target,
            ik_success=ik_success,
            fk_error=fk_error,
            ik_reason=ik_reason,
        )
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianJogCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
