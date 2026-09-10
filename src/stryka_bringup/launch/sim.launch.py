"""Bring up the STRYKA mission manager against SITL.

MAVROS is launched separately for now (see the run instructions). This file
brings up only ``mission_manager`` with ``params.yaml`` loaded.

    ros2 launch stryka_bringup sim.launch.py
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("stryka_bringup"), "config", "params.yaml"
    )

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the STRYKA params.yaml consumed by every node.",
    )

    mission_manager = Node(
        package="stryka_mission",
        executable="mission_manager",
        name="mission_manager",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config_file")}],
    )

    return LaunchDescription([config_arg, mission_manager])
