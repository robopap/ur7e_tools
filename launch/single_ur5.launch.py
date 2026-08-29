#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")

    own_share = get_package_share_directory("ur7e_tools")

    ur_launch = os.path.join(
        own_share,
        "launch",
        "ur_control_namespaced.launch.py",
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_launch),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,

            "tf_prefix": "",

            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": "true",

            "initial_joint_controller":
                "joint_trajectory_controller",

            "launch_rviz": launch_rviz,

            "reverse_port": "50001",
            "script_sender_port": "50002",
            "trajectory_port": "50003",
            "script_command_port": "50004",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5",
            choices=["ur5", "ur5e"],
        ),

        DeclareLaunchArgument(
            "robot_ip",
            default_value="127.0.0.1",
        ),

        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
        ),

        robot,
    ])