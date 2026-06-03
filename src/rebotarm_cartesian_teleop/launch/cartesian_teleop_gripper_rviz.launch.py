"""Second RViz window: gripper-following view (eye-in-hand style).

Requires the teleop stack + cartesian_teleop_validation_rviz.launch.py (or equivalent)
already publishing TF, robot_description, and teleop markers.

  just run-teleop-validation-rviz   # window 1: base_link validation view
  just run-teleop-gripper-rviz      # window 2: ThirdPersonFollower on end_link
"""

from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    teleop_share = FindPackageShare("rebotarm_cartesian_teleop")
    rviz_config = PathJoinSubstitution(
        [teleop_share, "rviz", "cartesian_teleop_gripper_view.rviz"]
    )

    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_gripper_view",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
