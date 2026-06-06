# Contributor README

How to build and run this fork. For what the fork adds and why, see
[`FORK_CHANGES.md`](FORK_CHANGES.md).

## Setup

```bash
git clone --recurse-submodules \
  https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2.git
cd reBotArmController_ROS2
sudo apt install just
just build-all
```

For Cartesian teleop (simulation / RViz only), also install the joy driver:

```bash
sudo apt install ros-jazzy-joy
```

If your ROS distro is not Jazzy, edit `ros_setup` at the top of `justfile`.

## Recipes

| Prefix | Use |
|--------|-----|
| `build-*` | compile (no robot side effects) |
| `run-*` | launch a node/GUI (one per terminal) |
| `svc-*` | call a `/rebotarm/…` service |

```bash
just                                       # list recipes
just run-driver /dev/ttyRebotB601          # terminal 1
just run-monitor /dev/ttyRebotB601         # terminal 2
just run-rqt                               # terminal 3
just svc-park                              # slow return to rest
```

Default device: `/dev/ttyACM0` (Seeed `arm.yaml`). Ctrl+C in the driver
terminal triggers the safe-park shutdown.

### Cartesian teleop (simulation / RViz)

Dry-run stack: gamepad → IK → fake joint states → RViz. **Does not command
the real arm** (`dry_run: true`, `output_mode: dry_run` in
`cartesian_teleop.yaml`).

**Main validation workflow** (four terminals):

```bash
just run-joy                    # terminal 1 — /joy
just run-joy-mapper             # terminal 2 — joy_cartesian_mapper
just run-cartesian-core         # terminal 3 — cartesian_jog_core
just run-teleop-validation-rviz # terminal 4 — RSP + markers + RViz
```

Optional second RViz window (gripper-following view; needs terminal 4 TF):

```bash
just run-teleop-gripper-rviz
```

In the validation window, select saved view **TeleopBaseValidation**. Jog one
axis at a time; guide the TCP sphere into the blue validation targets.

**D405 TF check** (standalone; do not run together with
`run-teleop-validation-rviz` — both start `robot_state_publisher`):

```bash
ros2 launch rebotarm_bringup d405_tf_diagnostics.launch.py
ros2 run tf2_ros tf2_echo end_link d405_color_optical_frame
```

**Topics (stable defaults):**

| Topic | Publisher | Role |
|-------|-----------|------|
| `/joy` | `joy_node` | Raw gamepad |
| `/rebotarm/cartesian_jog_cmd` | `joy_cartesian_mapper` | Jog command |
| `/rebotarm/cartesian_jog_state` | `cartesian_jog_core` | Pose, IK state, `q_target` |
| `/rebotarm/fake_joint_states` | `cartesian_jog_core` | Sim joint states for RSP |

Config: `src/rebotarm_cartesian_teleop/config/cartesian_teleop.yaml`.
Rationale and launch matrix: [`FORK_CHANGES.md`](FORK_CHANGES.md) §5–§6.

Full list: `build-driver`, `build-monitor`, `build-teleop`, `build-all`,
`clean`, `run-driver [dev]`, `run-monitor [dev]`, `run-rqt`, `run-gravity`,
`run-joy`, `run-joy-mapper`, `run-cartesian-core`,
`run-teleop-validation-rviz`, `run-teleop-gripper-rviz`,
`run-teleop-sim-rviz`, `svc-park`, `svc-enable`, `svc-disable`,
`svc-gravity-start`, `svc-gravity-stop`, `test-monitor`.

Rebuild teleop only after edits:

```bash
just build-teleop
```

Run teleop tests:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon test --packages-select rebotarm_cartesian_teleop rebotarm_msgs
colcon test-result --verbose
```
