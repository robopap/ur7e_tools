#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")

    own_share = get_package_share_directory("ur7e_tools")

    # ------------------------------------------------------------------
    # Vention workcell
    # ------------------------------------------------------------------
    workcell_urdf = os.path.join(
        own_share,
        "urdf",
        "single_ur3_workcell.urdf",
    )

    with open(workcell_urdf, "r") as f:
        workcell_description = f.read()

    workcell_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="single_workcell",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": workcell_description}
        ],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
    )

    # ------------------------------------------------------------------
    # Custom RViz: UR3 + Vention workcell
    # ------------------------------------------------------------------
    rviz_config = os.path.join(
        own_share,
        "config",
        "single_ur3.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(launch_rviz),
    )

    # ------------------------------------------------------------------
    # UR3 / UR3e
    # ------------------------------------------------------------------
    ur_launch = os.path.join(
        own_share,
        "launch",
        "ur_control_single_ur3.launch.py",
    )

    # ------------------------------------------------------------------
    # Robotiq 2F-140 RViz visualizer — simulation only
    # ------------------------------------------------------------------
    robotiq_visualizer = Node(
        package="ur7e_tools",
        executable="robotiq_gripper_visualizer",
        name="robotiq_gripper_visualizer",
        output="screen",
        condition=IfCondition(use_fake_hardware),
        parameters=[
            {"gripper_joint_name": "gripper_finger_joint"}
        ],
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_launch),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,

            "tf_prefix": "",
            "base_x": "-0.38",
            "base_y": "0.30",
            "base_z": "1.04",

            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": "true",

            "initial_joint_controller":
                "joint_trajectory_controller",

            # RViz is launched here with our custom config.
            "launch_rviz": "false",

            "reverse_port": "50001",
            "script_sender_port": "50002",
            "trajectory_port": "50003",
            "script_command_port": "50004",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3",
            choices=["ur3", "ur3e"],
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

        workcell_state_publisher,
        robotiq_visualizer,
        rviz_node,
        robot,
        
    ])
