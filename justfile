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
#   just run-monitor /dev/ttyRebotB601        # terminal 2
#   just run-rqt                              # terminal 3
#   just svc-park
#   just run-gravity                          # terminal 4 (Ctrl+C → park + disable)
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

build-driver:
    bash -c 'export PATH=/usr/bin:$PATH && source {{ros_setup}} && cd {{root}} && colcon build --symlink-install --base-paths src'

build-monitor:
    bash -c 'export PATH=/usr/bin:$PATH && source {{ros_setup}} && cd {{monitor_ws}} && colcon build --symlink-install --base-paths src'

build-all: build-driver build-monitor

clean:
    rm -rf {{root}}/build {{root}}/install {{root}}/log
    rm -rf {{monitor_ws}}/build {{monitor_ws}}/install {{monitor_ws}}/log

# --- Run (one terminal each) -------------------------------------------------

run-driver device=default_device:
    bash -c '{{src_driver}} && ros2 launch rebotarm_bringup driver_only.launch.py channel:={{device}}'

run-monitor device=default_device:
    bash -c '{{src_monitor}} && ros2 launch rebotarm_monitor monitor.launch.py serial_device:={{device}} enable_can_monitor:=false'

run-rqt:
    bash -c '{{src_monitor}} && ros2 run rqt_robot_monitor rqt_robot_monitor'

run-gravity:
    bash -c '{{src_driver}} && ros2 run rebotarmcontroller GravityCompensation'

# --- Services ----------------------------------------------------------------

svc-park:
    bash -c '{{src_driver}} && ros2 service call /rebotarm/park std_srvs/srv/Trigger {}'

svc-enable:
    bash -c '{{src_driver}} && ros2 service call /rebotarm/enable std_srvs/srv/Trigger {}'

svc-disable:
    bash -c '{{src_driver}} && ros2 service call /rebotarm/disable std_srvs/srv/Trigger {}'

# --- Tests -------------------------------------------------------------------

test-monitor:
    bash -c '{{src_driver}} && cd {{monitor_ws}} && colcon test --packages-select rebotarm_monitor && colcon test-result --verbose'
