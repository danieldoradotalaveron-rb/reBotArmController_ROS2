# rebotarm_monitor — fork integration tests

Pytest suite that requires the **driver fork workspace**: built `rebotarm_msgs`,
installed `rebotarm_monitor` overlay, and tracker modules that import generated
message types.

Unit tests that run with only this repo + standard ROS messages live in the
monitor submodule under `src/rebotarm_monitor/test/unit/`.

## Run locally

From the driver fork root (after `just build-all` or equivalent):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source rebotarm_monitor_ros2/install/setup.bash
python3 -m pytest integration/rebotarm_monitor/test -q
```

Or: `just test-monitor-integration`
