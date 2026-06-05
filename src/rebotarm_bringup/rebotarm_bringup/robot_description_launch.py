"""Shared robot_description helpers for bringup/teleop launch files.

Centralizes the xacro expansion of ``reBot-DevArm_fixend.xacro`` so every
launch file loads the same robot description with consistent D405 arguments.

This is launch-level glue only; it contains no teleop/IK logic.
"""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def d405_launch_arguments() -> list[DeclareLaunchArgument]:
    """Declare the standard D405 launch arguments (defaults match the xacro)."""
    return [
        DeclareLaunchArgument("enable_d405", default_value="true"),
        DeclareLaunchArgument("d405_mount_xyz", default_value="-0.09 0.0 0.07"),
        DeclareLaunchArgument("d405_mount_rpy", default_value="0 0.5 0"),
        DeclareLaunchArgument("d405_camera_body_xyz", default_value="0 0 0"),
        DeclareLaunchArgument("d405_camera_body_rpy", default_value="0 0 0"),
    ]


def robot_description_parameter() -> ParameterValue:
    """Return robot_description from the top-level xacro with D405 launch args."""
    bringup_share = FindPackageShare("rebotarm_bringup")
    xacro_file = PathJoinSubstitution(
        [bringup_share, "description", "urdf", "reBot-DevArm_fixend.xacro"]
    )
    return ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " enable_d405:=",
                LaunchConfiguration("enable_d405"),
                " d405_mount_xyz:='",
                LaunchConfiguration("d405_mount_xyz"),
                "'",
                " d405_mount_rpy:='",
                LaunchConfiguration("d405_mount_rpy"),
                "'",
                " d405_camera_body_xyz:='",
                LaunchConfiguration("d405_camera_body_xyz"),
                "'",
                " d405_camera_body_rpy:='",
                LaunchConfiguration("d405_camera_body_rpy"),
                "'",
            ]
        ),
        value_type=str,
    )
