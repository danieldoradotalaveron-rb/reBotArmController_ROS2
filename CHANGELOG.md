# Changelog

Granular history of every change this fork adds on top of upstream
[`Seeed-Projects/reBotArmController_ROS2`](https://github.com/Seeed-Projects/reBotArmController_ROS2).
Each entry links to the exact commit so reviewers can read the diff in
isolation. For the rationale behind each feature see
[`FORK_CHANGES.md`](FORK_CHANGES.md); for the dev workflow see
[`CONTRIBUTOR_README.md`](CONTRIBUTOR_README.md).

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
| 6 | _this commit_ | **chore(monitor): bump submodule to `5086a85` (v0.2.2 + README sync).** Adds the aggregator-group-selection notes to the monitor's READMEs without changing package version. | `rebotarm_monitor_ros2`, `CHANGELOG.md` |
