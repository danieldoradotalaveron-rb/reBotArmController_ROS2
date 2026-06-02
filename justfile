# reBot Arm fork — one terminal per recipe.
# Each recipe sources ROS + workspaces and runs one command. No orchestration.
#
# Naming:
#   build-*   compile a workspace (no side effects on the robot)
#   run-*     launch a ROS node / GUI
#   svc-*     call a /rebotarm service
#
# Examples:
#   just build-all                            # compile both workspaces
#   just run-driver /dev/ttyRebotB601         # terminal 1
#   just run-monitor /dev/ttyRebotB601        # terminal 2 (payload_profile:=light)
#   just run-monitor-medium /dev/ttyRebotB601 # terminal 2 (1.0 kg thresholds)
#   just run-rqt                              # terminal 3
#   just svc-park
#   just run-gravity                          # terminal 4 (Ctrl+C → park + disable)
#   just svc-gravity-start / svc-gravity-stop
#
# If your ROS distro is not Jazzy, edit `ros_setup` below.

root        := justfile_directory()
monitor_ws  := root + "/rebotarm_monitor_ros2"
ros_setup   := "/opt/ros/jazzy/setup.bash"
src_driver  := "source " + ros_setup + " && source " + root + "/install/setup.bash"
src_monitor := src_driver + " && source " + monitor_ws + "/install/setup.bash"

default_device := "/dev/ttyACM0"

default:
    @just --list

# --- Build -------------------------------------------------------------------

# Builds all packages under src/ (msgs, controller, bringup, cartesian_teleop, …).
build-driver:
    bash -c 'export PATH=/usr/bin:$PATH && source {{ros_setup}} && cd {{root}} && colcon build --symlink-install --base-paths src'

build-monitor:
    bash -c 'export PATH=/usr/bin:$PATH && source {{ros_setup}} && cd {{monitor_ws}} && colcon build --symlink-install --base-paths src'

# Rebuild only teleop after edits (msgs already built by build-driver).
build-teleop:
    bash -c 'export PATH=/usr/bin:$PATH && source {{ros_setup}} && cd {{root}} && colcon build --symlink-install --packages-select rebotarm_cartesian_teleop'

build-all: build-driver build-monitor

clean:
    rm -rf {{root}}/build {{root}}/install {{root}}/log
    rm -rf {{monitor_ws}}/build {{monitor_ws}}/install {{monitor_ws}}/log

# --- Run (one terminal each) -------------------------------------------------

run-driver device=default_device:
    bash -c '{{src_driver}} && ros2 launch rebotarm_bringup driver_only.launch.py channel:={{device}}'

# payload_profile: light (0.5 kg, default), medium (1.0 kg), rated (1.5 kg)
run-monitor device=default_device profile="light":
    bash -c '{{src_monitor}} && ros2 launch rebotarm_monitor monitor.launch.py serial_device:={{device}} enable_can_monitor:=false payload_profile:={{profile}}'

run-monitor-light device=default_device:
    just run-monitor {{device}} light

run-monitor-medium device=default_device:
    just run-monitor {{device}} medium

run-monitor-rated device=default_device:
    just run-monitor {{device}} rated

run-rqt:
    bash -c '{{src_monitor}} && ros2 run rqt_robot_monitor rqt_robot_monitor'

run-gravity:
    bash -c '{{src_driver}} && ros2 run rebotarmcontroller GravityCompensation'

teleop_params := '$(ros2 pkg prefix rebotarm_cartesian_teleop)/share/rebotarm_cartesian_teleop/config/cartesian_teleop.yaml'

run-joy:
    bash -c '{{src_driver}} && ros2 run joy joy_node'

run-joy-mapper:
    bash -c '{{src_driver}} && ros2 run rebotarm_cartesian_teleop joy_cartesian_mapper --ros-args --params-file {{teleop_params}}'

run-cartesian-core:
    bash -c '{{src_driver}} && ros2 run rebotarm_cartesian_teleop cartesian_jog_core --ros-args --params-file {{teleop_params}}'

# --- Services ----------------------------------------------------------------

svc-park:
    bash -c '{{src_driver}} && ros2 service call /rebotarm/park std_srvs/srv/Trigger {}'

svc-enable:
    bash -c '{{src_driver}} && ros2 service call /rebotarm/enable std_srvs/srv/Trigger {}'

svc-disable:
    bash -c '{{src_driver}} && ros2 service call /rebotarm/disable std_srvs/srv/Trigger {}'

svc-gravity-start:
    bash -c '{{src_driver}} && ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger {}'

svc-gravity-stop:
    bash -c '{{src_driver}} && ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger {}'

# --- Tests -------------------------------------------------------------------

test-monitor:
    bash -c '{{src_driver}} && cd {{monitor_ws}} && colcon test --packages-select rebotarm_monitor && colcon test-result --verbose'
