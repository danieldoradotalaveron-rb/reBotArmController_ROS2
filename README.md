# reBotArm ROS2 SDK

ROS 2 **Jazzy** workspace for the **reBot Arm B601-DM**: motor driver, bringup,
URDF, and optional overlays (monitor, gamepad teleop). This repo is a **fork**
of [`Seeed-Projects/reBotArmController_ROS2`](https://github.com/Seeed-Projects/reBotArmController_ROS2)
with safety, observability, and simulation tooling on top.

Upstream Seeed driver docs (English): [Seeed README](https://github.com/Seeed-Projects/reBotArmController_ROS2/blob/main/README.md).

---

## At a glance

| Piece | Role |
|-------|------|
| **`src/`** | Driver packages: `rebotarm_msgs`, `rebotarmcontroller`, `rebotarm_bringup` |
| **`third_party/reBotArm_control_py`** | Python SDK (FK/IK, hardware API) — [submodule](https://github.com/dorado-ai-devops/reBotArm_control_py), branch `main` |
| **`rebotarm_monitor_ros2/`** | Passive `/diagnostics` overlay — no commands to the arm |
| **`rebotarm_cartesian_gamepad_teleop_ros2/`** | Gamepad Cartesian jog for **RViz / dry-run** (WIP, not hardware yet) |
| **`integration/`** | Fork-only pytest (needs driver + SDK + overlays built) |
| **`justfile`** | Build, run, and test recipes — one terminal per long-running process |

**Typical stack:** driver on serial → optional monitor in rqt → teleop in RViz
for sim validation. Safe park on Ctrl+C or `just svc-park`.

---

## Quick start

```bash
git clone --recurse-submodules \
  https://github.com/danieldoradotalaveron-rb/reBotArmController_ROS2.git
cd reBotArmController_ROS2
sudo apt install just
just build-all
just run-driver /dev/ttyACM0    # terminal 1
```

Details, teleop workflow, and tests → [`CONTRIBUTOR_README.md`](CONTRIBUTOR_README.md).

---

## What this fork adds (summary)

| Area | One line |
|------|----------|
| Safe park | Slow return to captured rest pose before torque off |
| Gravity-comp ramp | Smooth MIT → pos_vel when leaving compensation |
| Monitor | `/diagnostics` from topics + host metrics |
| Cartesian teleop | Gamepad → IK → fake joints → RViz (`dry_run` default) |
| D405 TF | Eye-in-hand camera frames in URDF (no RealSense node) |
| Dev tooling | `just` recipes, overlay workspaces, CI split unit / integration |

Full rationale and file pointers → [`FORK_CHANGES.md`](FORK_CHANGES.md).  
Commit history → [`CHANGELOG.md`](CHANGELOG.md).

---

## Where to read next

| Doc | Use when you want to… |
|-----|------------------------|
| [`CONTRIBUTOR_README.md`](CONTRIBUTOR_README.md) | Build, run, test, submodule layout |
| [`FORK_CHANGES.md`](FORK_CHANGES.md) | Understand *why* each fork feature exists |
| [`CHANGELOG.md`](CHANGELOG.md) | Trace changes commit-by-commit |
| [Seeed upstream README](https://github.com/Seeed-Projects/reBotArmController_ROS2/blob/main/README.md) | Original driver usage (English) |
