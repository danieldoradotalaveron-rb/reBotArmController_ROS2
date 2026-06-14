set shell := ["bash", "-cu"]

ros_distro := "jazzy"
python := "/usr/bin/python3"
core_packages := "rebotarm_msgs rebotarmcontroller rebotarm_bringup"
all_packages := "rebotarm_msgs rebotarmcontroller rebotarm_bringup rebotarm_moveit_config rebotarm_moveit_demos"

default:
    @just --list

clean:
    rm -rf build install log

link-sdk:
    #!/usr/bin/env bash
    set -eo pipefail
    root="$(pwd)"
    sdk_src="${root}/third_party/reBotArm_control_py"
    sdk_dst="${root}/install/rebotarmcontroller/lib/third_party/reBotArm_control_py"
    if [[ ! -d "${sdk_src}/reBotArm_control_py" ]]; then
      echo "Missing SDK at ${sdk_src}" >&2
      exit 1
    fi
    if [[ ! -d "${root}/install/rebotarmcontroller/lib" ]]; then
      echo "Build rebotarmcontroller first (just build)" >&2
      exit 1
    fi
    mkdir -p "${root}/install/rebotarmcontroller/lib/third_party"
    ln -sfn "${sdk_src}" "${sdk_dst}"

# Full clean build of driver stack (msgs + controller + bringup).
build: clean
    #!/usr/bin/env bash
    set -eo pipefail
    unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
    export PATH="/usr/bin:$PATH"
    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
    source /opt/ros/{{ros_distro}}/setup.bash

    colcon build --packages-select rebotarm_msgs \
      --cmake-args -DPython3_EXECUTABLE={{python}}
    source install/setup.bash

    colcon build --packages-select rebotarmcontroller \
      --cmake-args -DPython3_EXECUTABLE={{python}}
    source install/setup.bash

    colcon build --packages-select rebotarm_bringup
    just link-sdk
    echo
    echo "Build OK. Run: source install/setup.bash"

# Full clean build of entire workspace (includes MoveIt packages).
build-all: clean
    #!/usr/bin/env bash
    set -eo pipefail
    unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
    export PATH="/usr/bin:$PATH"
    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
    source /opt/ros/{{ros_distro}}/setup.bash

    colcon build --packages-select {{core_packages}} \
      --cmake-args -DPython3_EXECUTABLE={{python}}
    source install/setup.bash

    colcon build --packages-select rebotarm_moveit_config rebotarm_moveit_demos \
      --cmake-args -DPython3_EXECUTABLE={{python}}
    just link-sdk
    echo
    echo "Build OK. Run: source install/setup.bash"

# Incremental bringup-only build with a clean ROS environment.
build-bringup:
    #!/usr/bin/env bash
    set -eo pipefail
    unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
    export PATH="/usr/bin:$PATH"
    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
    source /opt/ros/{{ros_distro}}/setup.bash
    if [[ -f install/setup.bash ]]; then
      source install/setup.bash
    fi
    colcon build --packages-select rebotarm_bringup
    just link-sdk
    echo
    echo "Build OK. Run: source install/setup.bash"
