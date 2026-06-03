"""Simulation-only robot_state_publisher fed by /rebotarm/fake_joint_states."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    bringup_share = FindPackageShare("rebotarm_bringup")
    fake_joint_states_topic = LaunchConfiguration("fake_joint_states_topic")

    urdf_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "reBot-DevArm_fixend.urdf"]
    )
    robot_description = ParameterValue(Command(["cat ", urdf_file]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fake_joint_states_topic",
                default_value="/rebotarm/fake_joint_states",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", fake_joint_states_topic)],
            ),
        ]
    )
