# Fork changes

What this fork adds on top of upstream
`Seeed-Projects/reBotArmController_ROS2`, and why. For build/run commands see
[`CONTRIBUTOR_README.md`](CONTRIBUTOR_README.md).

| # | Area | Why it exists |
|---|------|---------------|
| 1 | Safe park | Upstream goes to q=0 electrical pose and then cuts torque; the arm drops slightly. The fork captures the gravity rest pose on connect and returns to it slowly (pos_vel) before disconnecting. |
| 2 | Smooth gravity-comp stop | Leaving gravity compensation (MIT + tau_g) by snapping to pos_vel causes a clack. A 12-step MIT ramp fades the feedforward torque to zero and stiffens kp before the mode switch. |
| 3 | Passive monitor | Upstream has no monitoring/observability. An overlay (`rebotarm_monitor`) publishes `/diagnostics` from topics + host metrics, with no driver coupling. |
| 4 | Dev tooling | `justfile` + `pyproject.toml` so build/run/services are one command and the source order is correct. |
| 5 | Cartesian teleop (sim) | Upstream has no gamepad Cartesian jog path for simulation and RViz validation before sending hardware commands. |
| 6 | D405 eye-in-hand TF | Rigid camera frames under `end_link` for RViz/TF inspection without a RealSense driver or hand-eye calibration. |

---

## 1. Safe park

**Problem.** Upstream `disable()` on Ctrl+C drops torque immediately after
going to q=0, which is slightly higher than the real gravity rest pose — the
arm drops from there.

**How.** Always pos_vel:

- **On connect** (`HardwareManager.connect`): polls joints until two reads
  agree, stores them as `_rest_pose`, and seeds the pos_vel loop with that
  target. Otherwise `ArmEndPos` defaults to q=0 and the arm drifts there.
- **On park / shutdown** (`_park_at_rest_pose` → `_return_to_rest_pose`):
  drives every joint back to `_rest_pose` capped at `_PARK_VLIM = 0.3 rad/s`
  via `_vlim_override`. Blocks until tolerance (0.02 rad) or 30 s timeout.
  Degenerate-capture fallback: if `_rest_pose ≈ 0` but the arm is far from
  it, holds current joints instead of driving the wrong way.

**Triggers.** Ctrl+C, `ros2 service call /rebotarm/park std_srvs/srv/Trigger`,
`just svc-park`, or scripts (e.g. `examples/gravity_compensation.py` calls
park in `finally` before disable).

Files: `src/rebotarmcontroller/.../hardware_manager.py`, `ros_services.py`.

---

## 2. Smooth gravity-comp stop (MIT ramp-out)

**Problem.** Gravity compensation runs in MIT mode with feedforward torque
`tau_g`. Switching directly to pos_vel (or disabling) makes `tau_g` vanish in
one bus tick, audible clack and a jolt to the gearboxes.

**How.** `HardwareManager._ramp_out_gravity_compensation` runs **12 MIT steps
× 20 ms**, linearly fading `tau_g → 0` while ramping `kp` from `_GC_KP=7.0` to
`_GC_STOP_HOLD_KP=12.0`. Then the loop stops, the mode switches to pos_vel
with the held target, and `_settle_pos_vel_hold` keeps velocity at zero for
`_GC_STOP_SETTLE_S=0.12 s` so pos_vel converges before any motion.

Composes with §1: when park is called while gravity comp is active,
`_park_at_rest_pose` runs `stop_gravity_compensation(hold_target=gc_q)` (which
includes this ramp) **before** the pos_vel slow return. Outside of park, the
same ramp runs on `/rebotarm/gravity_compensation/stop`.

Files: `hardware_manager.py` → `_ramp_out_gravity_compensation`,
`_settle_pos_vel_hold`, `stop_gravity_compensation`.

---

## 3. Passive monitor (`rebotarm_monitor`)

**Problem.** Upstream has no `/diagnostics` output. Field issues (joint stale,
high effort, driver crash, USB unplugged) had to be debugged with `ros2 topic
echo` and `top`.

**How.** Separate overlay workspace (git submodule
`rebotarm_monitor_ros2/`, version **v0.2.2**) that subscribes to driver topics
and polls host metrics, publishing `diagnostic_msgs/DiagnosticArray` on
`/diagnostics` + an aggregator for `rqt_robot_monitor`. Reads only — never
commands the arm.

Trackers (rqt groups): Hardware, Joints, Gripper, Link, System, and Bus when
`enable_can_monitor:=true`. The aggregator config is selected at launch time
to avoid a STALE Bus group on USB/serial setups.

Repo: <https://github.com/danieldoradotalaveron-rb/rebotarm_monitor_ros2>
([README](rebotarm_monitor_ros2/README.md)).

---

## 4. Dev tooling

**Problem.** Two workspaces (driver + monitor submodule), strict source
order, easy to launch the wrong one or mix `ros2 launch` syntax with shell.

**How.** `justfile` with prefixed recipes (`build-*` / `run-*` /
`svc-*`), absolute paths baked in, ROS sourced inside each recipe. One
terminal per long-running process; Ctrl+C cleans up. See
[`CONTRIBUTOR_README.md`](CONTRIBUTOR_README.md).

---

## 5. Cartesian gamepad teleop (simulation-first)

**Problem.** Upstream exposes joint/pose services but no integrated gamepad
Cartesian jog loop for dry-run simulation, IK tuning, and RViz validation.

**How.** New packages `rebotarm_msgs` and `rebotarm_cartesian_teleop`:

```text
joy_node  →  joy_cartesian_mapper  →  /rebotarm/cartesian_jog_cmd
                                              ↓
                                    cartesian_jog_core
                                              ↓
                         /rebotarm/cartesian_jog_state
                         /rebotarm/fake_joint_states
                                              ↓
                              robot_state_publisher  →  /tf
                                              ↓
                         teleop_viz_markers, teleop_validation_targets, RViz
```

**Nodes**

| Node | Executable | Role |
|------|------------|------|
| `joy_node` | `joy joy_node` | Publishes `/joy` |
| `joy_cartesian_mapper` | `rebotarm_cartesian_teleop joy_cartesian_mapper` | Sticks → `CartesianJogCmd`; deadman / soft-stop / speed boost |
| `cartesian_jog_core` | `rebotarm_cartesian_teleop cartesian_jog_core` | FK/IK servo, safety gates, fake `JointState` |
| `teleop_viz_markers` | `rebotarm_cartesian_teleop teleop_viz_markers` | TCP axes + trail in RViz |
| `teleop_validation_targets` | `rebotarm_cartesian_teleop teleop_validation_targets` | Reach-test spheres |

**Messages** (`rebotarm_msgs`):

- `CartesianJogCmd` — linear/angular velocity, `deadman`, `base_jog_active`,
  `joint1_jog_velocity_rad_s`, `command_frame_kind` (`local_window_frame` or
  `base_link`).
- `CartesianJogState` — poses, `q_current` / `q_target` (joint1…joint6),
  IK success, rejection/clamp reasons, `dry_run`, `output_mode`.

**Teleop behaviour (stable defaults in `cartesian_teleop.yaml`)**

- **Dry run only** — `dry_run: true`, `output_mode: dry_run`; no hardware
  commands.
- **Local-window jog** — linear sticks move the target in a box anchored at
  session start (`local_window_*_m`); mapper sets
  `command_frame_kind: local_window_frame`.
- **Base (joint1) jog** — gamepad axis 6 (hat) → `base_jog_active`; core
  integrates joint1 and skips Cartesian IK for that tick.
- **Joint1 gates** — anchor hard gate and global operational cap
  (`enable_joint1_anchor_hard_gate`, `enable_joint1_global_operational_cap`).
- **IK** — `position_only`, acceptance / IK_NO_EFFECT / near-limit policies;
  FK/IK URDF is arm-only (`reBot-DevArm_fixend_core.urdf`).

**Launches / recipes**

| Recipe / launch | Starts |
|-----------------|--------|
| `just run-joy` + `run-joy-mapper` + `run-cartesian-core` | Teleop core (no RViz) |
| `just run-teleop-validation-rviz` | RSP + markers + targets + RViz (`TeleopBaseValidation`) |
| `just run-teleop-gripper-rviz` | Second RViz only (`GripperFollowD405`, Target `end_link`) |
| `cartesian_teleop_sim_rviz.launch.py` | RSP + generic `rebotarm.rviz` |

Run **one** `robot_state_publisher` per session. Do not combine
`run-teleop-validation-rviz` with `d405_tf_diagnostics` or
`fake_robot_state_publisher`.

**Out of scope (not in this fork):** hardware teleop bridge, RealSense driver,
Gazebo physics, Isaac Sim, hand-eye calibration.

Files: `src/rebotarm_cartesian_teleop/`, `src/rebotarm_msgs/msg/CartesianJog*.msg`,
`src/rebotarm_cartesian_teleop/config/cartesian_teleop.yaml`,
`src/rebotarm_cartesian_teleop/launch/*.launch.py`.

---

## 6. D405 eye-in-hand TF

**Problem.** Eye-in-hand RViz and TF inspection need camera frames on the URDF
without running the RealSense node or calibrating hand-eye.

**How.** Xacro macro `d405_eye_in_hand` adds rigid links/joints under
`end_link`:

```text
end_link → d405_mount_link → d405_camera_body_link
         → d405_color_optical_frame / d405_depth_optical_frame
```

- Top-level URDF: `reBot-DevArm_fixend.xacro` (RSP / RViz).
- Arm-only URDF: `reBot-DevArm_fixend_core.urdf` (FK/IK — no D405 links).
- Shared launch helper: `rebotarm_bringup/robot_description_launch.py`
  (`d405_launch_arguments()`, `robot_description_parameter()`).
- Defaults: `d405_mount_xyz:="-0.09 0.0 0.07"`, `d405_mount_rpy:="0 0.5 0"`.
- TF-only launch: `d405_tf_diagnostics.launch.py` (RSP + zero joint states).

**Triggers.**

```bash
ros2 launch rebotarm_bringup d405_tf_diagnostics.launch.py
ros2 run tf2_ros tf2_echo end_link d405_color_optical_frame
```

Files: `src/rebotarm_bringup/description/urdf/includes/d405_eye_in_hand.xacro`,
`robot_description_launch.py`, `launch/d405_tf_diagnostics.launch.py`,
`config/d405_mount.yaml` (reference values; runtime args come from the launch
helper).

---

## Workspace layout

```text
reBotArmController_ROS2/
├── README.md                   # Seeed README + links to fork docs
├── CONTRIBUTOR_README.md       # build & run
├── FORK_CHANGES.md             # this file
├── CHANGELOG.md                # per-commit fork history
├── justfile, pyproject.toml    # dev tooling
├── src/
│   ├── rebotarm_msgs/          # CartesianJogCmd / CartesianJogState
│   ├── rebotarm_cartesian_teleop/
│   ├── rebotarm_bringup/       # URDF, launches, D405 xacro
│   └── rebotarmcontroller/
└── rebotarm_monitor_ros2/      # git submodule (passive monitor)
```

Update the monitor pointer with
`git submodule update --remote rebotarm_monitor_ros2 && just build-monitor`.
