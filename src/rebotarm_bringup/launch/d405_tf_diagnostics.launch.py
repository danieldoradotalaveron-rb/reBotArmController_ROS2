"""D405 TF diagnostics: robot_state_publisher (with D405) + static joint states.

Publishes the full TF tree including the rigid D405 eye-in-hand frames so they
can be inspected with tf2_echo / view_frames without running teleop.

  ros2 launch rebotarm_bringup d405_tf_diagnostics.launch.py
  # then, in another terminal:
  ros2 run tf2_ros tf2_echo end_link d405_color_optical_frame
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rebotarm_bringup.robot_description_launch import (
    d405_launch_arguments,
    robot_description_parameter,
)

from launch import LaunchDescription


def generate_launch_description():
    fake_joint_states_topic = LaunchConfiguration("fake_joint_states_topic")
    robot_description = robot_description_parameter()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fake_joint_states_topic",
                default_value="/rebotarm/fake_joint_states",
            ),
            *d405_launch_arguments(),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[("/joint_states", fake_joint_states_topic)],
            ),
            # Drive the 6 arm joints to zero so /tf is populated without teleop.
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                remappings=[("/joint_states", fake_joint_states_topic)],
            ),
        ]
    )
