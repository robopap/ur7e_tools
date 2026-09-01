#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node
from launch.conditions import IfCondition

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    robot1_ip = LaunchConfiguration("robot1_ip")
    robot2_ip = LaunchConfiguration("robot2_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot1_x = LaunchConfiguration("robot1_x")
    robot1_y = LaunchConfiguration("robot1_y")
    robot1_z = LaunchConfiguration("robot1_z")
    robot1_roll = LaunchConfiguration("robot1_roll")
    robot1_pitch = LaunchConfiguration("robot1_pitch")
    robot1_yaw = LaunchConfiguration("robot1_yaw")

    robot2_x = LaunchConfiguration("robot2_x")
    robot2_y = LaunchConfiguration("robot2_y")
    robot2_z = LaunchConfiguration("robot2_z")
    robot2_roll = LaunchConfiguration("robot2_roll")
    robot2_pitch = LaunchConfiguration("robot2_pitch")
    robot2_yaw = LaunchConfiguration("robot2_yaw")

    own_share = get_package_share_directory("ur7e_tools")

    gripper_visualizer_robot1 = Node(
        package="ur7e_tools",
        executable="gripper_visualizer",
        namespace="robot1",
        name="gripper_visualizer",
        output="screen",
        parameters=[
            {
                "gripper_joint_name":
                    "robot1_gripper_gripper_joint"
            }
        ],
    )

    gripper_visualizer_robot2 = Node(
        package="ur7e_tools",
        executable="gripper_visualizer",
        namespace="robot2",
        name="gripper_visualizer",
        output="screen",
        parameters=[
            {
                "gripper_joint_name":
                    "robot2_gripper_gripper_joint"
            }
        ],
    )

    rviz_config = os.path.join(    own_share,    "config",    "dual_ur7e.rviz",)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(        LaunchConfiguration("launch_rviz")    ),
    )

    workcell_urdf = os.path.join(own_share, "urdf", "workcell.urdf")
    with open(workcell_urdf, "r") as f:
         workcell_description = f.read()

    workcell_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="workcell",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": workcell_description}],
        remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
    )

    robot1_controllers = os.path.join(own_share, "config", "robot1_ur_controllers.yaml")
    robot2_controllers = os.path.join(own_share, "config", "robot2_ur_controllers.yaml")

    robot1_kinematics = os.path.join(own_share, "config", "calibration", "robot1_ur7e_calibration.yaml")
    robot2_kinematics = os.path.join(own_share, "config", "calibration", "robot2_ur7e_calibration.yaml")

    ur_launch = os.path.join(
        own_share,
        "launch",
        "ur_control_namespaced.launch.py",
    )

    robot1 = GroupAction(
        actions=[
            PushRosNamespace("robot1"),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_launch),
                launch_arguments={
                    "ur_type": "ur7e",
                    "robot_ip": robot1_ip,

                    "tf_prefix": "robot1_",
                    "use_2fg7": "true",
                    "base_x": robot1_x,
                    "base_y": robot1_y,
                    "base_z": robot1_z,
                    "base_roll": robot1_roll,
                    "base_pitch": robot1_pitch,
                    "base_yaw": robot1_yaw,

                    "controllers_file": robot1_controllers,
                    "kinematics_params_file": robot1_kinematics,

                    "use_fake_hardware": use_fake_hardware,
                    "fake_sensor_commands": "true",

                    "initial_joint_controller":
                        "joint_trajectory_controller",

                    "launch_rviz": "false",

                    "reverse_port": "50001",
                    "script_sender_port": "50002",
                    "trajectory_port": "50003",
                    "script_command_port": "50004",
                }.items(),
            ),
        ]
    )

    robot2 = GroupAction(
        actions=[
            PushRosNamespace("robot2"),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_launch),
                launch_arguments={
                    "ur_type": "ur7e",
                    "robot_ip": robot2_ip,

                    "tf_prefix": "robot2_",
                    "use_2fg7": "true",
                    "base_x": robot2_x,
                    "base_y": robot2_y,
                    "base_z": robot2_z,
                    "base_roll": robot2_roll,
                    "base_pitch": robot2_pitch,
                    "base_yaw": robot2_yaw,

                    "controllers_file": robot2_controllers,
                    "kinematics_params_file": robot2_kinematics,

                    "use_fake_hardware": use_fake_hardware,
                    "fake_sensor_commands": "true",

                    "initial_joint_controller":
                        "joint_trajectory_controller",

                    "launch_rviz": "false",

                    "reverse_port": "50011",
                    "script_sender_port": "50012",
                    "trajectory_port": "50013",
                    "script_command_port": "50014",
                }.items(),
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot1_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("robot2_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("robot1_x", default_value="0.27"),
        DeclareLaunchArgument("robot1_y", default_value="0.275"),
        DeclareLaunchArgument("robot1_z", default_value="1.0"),
        DeclareLaunchArgument("robot1_roll", default_value="1.5708"),
        DeclareLaunchArgument("robot1_pitch", default_value="0.0"),
        DeclareLaunchArgument("robot1_yaw", default_value="1.5708"),
        # DeclareLaunchArgument("robot1_roll", default_value="0.0"),
        # DeclareLaunchArgument("robot1_pitch", default_value="1.5708"),
        # DeclareLaunchArgument("robot1_yaw", default_value="0.0"),

        DeclareLaunchArgument("robot2_x", default_value="-0.22"),
        DeclareLaunchArgument("robot2_y", default_value="0.275"),
        DeclareLaunchArgument("robot2_z", default_value="1.0"),
        DeclareLaunchArgument("robot2_roll", default_value="1.5708"),
        DeclareLaunchArgument("robot2_pitch", default_value="0.0"),
        DeclareLaunchArgument("robot2_yaw", default_value="-1.5708"),

        workcell_state_publisher,
        robot1,
        robot2,
        rviz_node,
        gripper_visualizer_robot1,
        gripper_visualizer_robot2,
    ])