# rebotarm_cartesian_teleop — fork integration tests

Pytest suite that requires the **driver fork workspace**: `rebotarm_msgs`,
`rebotarm_bringup`, vendored `reBotArm_control_py`, and a built teleop overlay.

Unit tests that run with only this repo + standard ROS messages live in the
teleop submodule under `src/rebotarm_cartesian_teleop/test/unit/`.

## Run locally

From the driver fork root (after `just build-all` or equivalent):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source rebotarm_cartesian_gamepad_teleop_ros2/install/setup.bash
export REBOTARM_DRIVER_WS="$PWD"
python3 -m pytest integration/rebotarm_cartesian_teleop/test -q
```

Or: `just test-teleop-integration`
