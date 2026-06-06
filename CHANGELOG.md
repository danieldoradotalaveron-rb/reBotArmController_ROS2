# Changelog

Granular history of every change this fork adds on top of upstream
[`Seeed-Projects/reBotArmController_ROS2`](https://github.com/Seeed-Projects/reBotArmController_ROS2).
Each entry links to the exact commit so reviewers can read the diff in
isolation. For the rationale behind each feature see
[`FORK_CHANGES.md`](FORK_CHANGES.md); for the dev workflow see
[`CONTRIBUTOR_README.md`](CONTRIBUTOR_README.md).

## Fork v0.1.0

Continues fork v0.0.2. Adds simulation-first Cartesian gamepad teleop,
RViz validation stack, joint1 base jog + safety gates, and D405 eye-in-hand TF.
HEAD: [`fc82136`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/fc8213620a7eb36cb9f8c37022f6918dc1c241a9).

| # | Commit | Summary | Touches |
|---|--------|---------|---------|
| 1 | [`92a0b7f`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/92a0b7f536540424a7fa44d2c8c5db9e41ffc07f) | **feat(msgs): Cartesian jog messages.** Add `CartesianJogCmd` and `CartesianJogState` to `rebotarm_msgs`. | `rebotarm_msgs/msg/CartesianJog*.msg`, `CMakeLists.txt` |
| 2 | [`bec2f95`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/bec2f954be08819dd0207dcbe165b643d229c8af) | **feat(teleop): gamepad mapper package and YAML config.** New `rebotarm_cartesian_teleop` package; `joy_cartesian_mapper` maps sticks, deadman, soft-stop, and speed boost to `CartesianJogCmd`; `cartesian_teleop.yaml` parameters. | `rebotarm_cartesian_teleop/`, `config/cartesian_teleop.yaml`, `justfile` |
| 3 | [`72567cb`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/72567cb784c91af3f8da567a5ba6659fa595c69c) | **feat(teleop): FK/IK core and fake joint states for RViz.** `cartesian_jog_core` runs FK/IK servo; publishes `/rebotarm/fake_joint_states`; `q_sim` as simulation source of truth; RSP remapping for sim RViz. | `cartesian_jog_core.py`, `fk_kinematics.py`, `ik_kinematics.py`, `fake_joint_state.py`, launches |
| 4 | [`625844f`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/625844f17ce80868031eed6d495d47397f159064) | **feat(teleop): IK quality diagnostics and phantom-success rejection.** Throttled IK quality logs; acceptance / IK_NO_EFFECT / near-limit policies; improved IK failure visibility. | `ik_quality_diagnostics.py`, `jog_core_logic.py`, tests |
| 5 | [`1ef5fd9`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/1ef5fd9ce32ad56685353200bf5f124720d6456e) | **feat(teleop): RViz validation stack and local-window teleop.** Validation markers/targets, `cartesian_teleop_validation_rviz.launch.py`, gripper second window, local-window frame semantics. | `teleop_viz_markers.py`, `teleop_validation_targets.py`, `rviz/*.rviz`, launches |
| 6 | [`d073e7c`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/d073e7cf16d23c4b29ebd568792287f67a865ba7) | **feat(teleop): joint1 base jog and anchor/global gates.** Hat-axis base jog (`base_jog_active`); joint1 anchor hard gate and global operational cap; local-window + base-jog integration tests. | `joy_mapping.py`, `cartesian_jog_core.py`, `jog_core_logic.py`, tests |
| 7 | [`fc82136`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/fc8213620a7eb36cb9f8c37022f6918dc1c241a9) | **feat(bringup): D405 eye-in-hand TF and shared robot_description helper.** Rigid D405 xacro under `end_link`; core URDF split for FK/IK; `robot_description_launch.py`; `d405_tf_diagnostics.launch.py`; mount launch args. | `d405_eye_in_hand.xacro`, `reBot-DevArm_fixend*.xacro/urdf`, `robot_description_launch.py`, tests |
| 8 | _this commit_ | **docs: teleop usage in CONTRIBUTOR, FORK_CHANGES, CHANGELOG.** Document build/run, architecture, launch matrix, and v0.1.0 commit map. | `CONTRIBUTOR_README.md`, `FORK_CHANGES.md`, `CHANGELOG.md` |

Also between v0.0.2 and v0.1.0: `876cbc3` adds `svc-gravity-start` /
`svc-gravity-stop` just recipes; several monitor submodule pointer bumps (no
driver changes).

---

## Fork v0.0.2

Forked from upstream commit
[`d3a415e`](https://github.com/Seeed-Projects/reBotArmController_ROS2/commit/d3a415ec0d52f7117f44dbe0cca27f29eddef8fc).

| # | Commit | Summary | Touches |
|---|--------|---------|---------|
| 1 | [`20c6fa8`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/20c6fa82cdf096a913a5d22b8be8cab9c5123d80) | **feat(driver): safe park on shutdown and via `/rebotarm/park` service.** Capture rest pose on connect; pos_vel slow return on shutdown / park; expose `/rebotarm/park` Trigger service. | `hardware_manager.py`, `ros_services.py`, `examples/gravity_compensation.py`, `rebotarm_controller.py` |
| 2 | [`5379e1f`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/5379e1fb8c65cf1185147fafb524ea468fae01f3) | **feat(driver): smooth MIT ramp-out when leaving gravity compensation.** 12 MIT steps × 20 ms fade `tau_g → 0` and ramp `kp 7→12` before the pos_vel handoff; 120 ms settle. Eliminates the clack on `stop_gravity_compensation`. | `hardware_manager.py` |
| 3 | [`63dc83e`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/63dc83e2d575cb30cdd937529796e2533550460f) | **feat(monitor): link `rebotarm_monitor_ros2` as git submodule (v0.2.2).** Passive `/diagnostics` overlay pinned to a reproducible SHA; no driver coupling. | `.gitmodules`, `rebotarm_monitor_ros2` |
| 4 | [`285e5af`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/285e5af7585bf7d7aff809afb9146a219acc44e1) | **chore: dev tooling (justfile, pyproject, gitignore) and config polish.** One-terminal-per-recipe workflow; ROS sourced inside each recipe; English comments in `arm.yaml`/`gripper.yaml`. | `justfile`, `pyproject.toml`, `.gitignore`, `arm.yaml`, `gripper.yaml` |
| 5 | [`27694f2`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/27694f249b8e604f449e09978adc3b716db4594b) | **docs: split fork documentation (README header, CONTRIBUTOR, FORK_CHANGES).** Landing header in `README.md`; build/run guide in `CONTRIBUTOR_README.md`; rationale catalogue in `FORK_CHANGES.md`. | `README.md`, `CONTRIBUTOR_README.md`, `FORK_CHANGES.md` |
| 6 | [`8378571`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/8378571314437c1d75c333db06cc96f32fda9c3b) | **chore(monitor): bump submodule to `5086a85` (v0.2.2 + README sync).** Adds the aggregator-group-selection notes to the monitor's READMEs without changing package version. | `rebotarm_monitor_ros2`, `CHANGELOG.md` |
| 7 | [`9cf7d7d`](https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2/commit/9cf7d7df3cad389417d483ba1b686f0d1136b221) | **docs: add CHANGELOG.md mapping each fork commit to its permalink.** Per-commit fork history with links for review. | `CHANGELOG.md` |
