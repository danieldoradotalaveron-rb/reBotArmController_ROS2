"""RViz validation stack for Cartesian teleop (base_link-aligned view + TCP markers).

Launch together with (separate terminals):
  just run-joy
  just run-joy-mapper
  just run-cartesian-core

Validation workflow:
  1. In RViz Views panel, select saved view ``TeleopBaseValidation`` (or ``TeleopTopDownZ``).
  2. Do not orbit the camera while testing axis motion.
  3. Jog one axis at a time (+X, -X, +Y, -Y, +Z, -Z in base_link).
  4. Judge TCP motion via green sphere + blue trail vs base_link grid/axes.
  5. Guide the green TCP sphere into grey validation targets; they turn red on contact.
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    teleop_share = FindPackageShare("rebotarm_cartesian_teleop")

    fake_joint_states_topic = LaunchConfiguration("fake_joint_states_topic")
    cartesian_jog_state_topic = LaunchConfiguration("cartesian_jog_state_topic")

    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "reBot-DevArm_fixend.urdf"]
    )
    rviz_config = PathJoinSubstitution(
        [teleop_share, "rviz", "cartesian_teleop_validation.rviz"]
    )
    teleop_params = PathJoinSubstitution(
        [teleop_share, "config", "cartesian_teleop.yaml"]
    )
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fake_joint_states_topic",
                default_value="/rebotarm/fake_joint_states",
            ),
            DeclareLaunchArgument(
                "cartesian_jog_state_topic",
                default_value="/rebotarm/cartesian_jog_state",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", fake_joint_states_topic)],
            ),
            Node(
                package="rebotarm_cartesian_teleop",
                executable="teleop_viz_markers",
                name="teleop_viz_markers",
                output="screen",
                parameters=[
                    {"cartesian_jog_state_topic": cartesian_jog_state_topic},
                ],
            ),
            Node(
                package="rebotarm_cartesian_teleop",
                executable="teleop_validation_targets",
                name="teleop_validation_targets",
                output="screen",
                parameters=[teleop_params],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
