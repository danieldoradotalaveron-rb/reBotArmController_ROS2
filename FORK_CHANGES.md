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

## Workspace layout

```text
reBotArmController_ROS2/
├── README.md                   # Seeed README + links to fork docs
├── CONTRIBUTOR_README.md       # build & run
├── FORK_CHANGES.md             # this file
├── justfile, pyproject.toml    # dev tooling
├── src/                        # driver packages
└── rebotarm_monitor_ros2/      # git submodule (passive monitor)
```

Update the monitor pointer with
`git submodule update --remote rebotarm_monitor_ros2 && just build-monitor`.
