#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gripper_type = LaunchConfiguration("gripper_type")

    own_share = get_package_share_directory("ur7e_tools")

    # Same Vention workcell used by the Single UR3 setup.
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
        parameters=[{"robot_description": workcell_description}],
        remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
    )

    rviz_config = os.path.join(
        own_share,
        "config",
        "single_ur7e.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(launch_rviz),
    )

    onrobot_selected = IfCondition(
        PythonExpression(["'", gripper_type, "' == 'onrobot'"])
    )
    robotiq_selected = IfCondition(
        PythonExpression(["'", gripper_type, "' == 'robotiq'"])
    )

    onrobot_control_launch = os.path.join(
        own_share,
        "launch",
        "ur_control_namespaced.launch.py",
    )
    robotiq_control_launch = os.path.join(
        own_share,
        "launch",
        "ur_control_namespaced_robotiq.launch.py",
    )

    common_args = {
        "ur_type": "ur7e",
        "robot_ip": robot_ip,
        "tf_prefix": "",
        "base_x": "-0.38",
        "base_y": "0.30",
        "base_z": "1.04",
        "use_fake_hardware": use_fake_hardware,
        "fake_sensor_commands": "true",
        "initial_joint_controller": "joint_trajectory_controller",
        "launch_rviz": "false",
        "reverse_port": "50001",
        "script_sender_port": "50002",
        "trajectory_port": "50003",
        "script_command_port": "50004",
    }

    onrobot_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(onrobot_control_launch),
        condition=onrobot_selected,
        launch_arguments={
            **common_args,
            "use_2fg7": "true",
            "use_wrist_camera": "false",
        }.items(),
    )

    robotiq_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robotiq_control_launch),
        condition=robotiq_selected,
        launch_arguments={
            **common_args,
            "use_2fg7": "false",
            "use_wrist_camera": "false",
        }.items(),
    )

    # Run the selected visualizer in both simulation and real mode. In real
    # mode it merges live UR joint_states with the selected gripper state so
    # RViz retains the complete robot model.
    onrobot_visualizer = Node(
        package="ur7e_tools",
        executable="gripper_visualizer",
        name="single_ur7e_onrobot_visualizer",
        output="screen",
        parameters=[{"gripper_joint_name": "gripper_gripper_joint"}],
        condition=onrobot_selected,
    )

    robotiq_visualizer = Node(
        package="ur7e_tools",
        executable="robotiq_gripper_visualizer",
        name="single_ur7e_robotiq_visualizer",
        output="screen",
        parameters=[{"gripper_joint_name": "gripper_finger_joint"}],
        condition=robotiq_selected,
    )

    return LaunchDescription([
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
        DeclareLaunchArgument(
            "gripper_type",
            default_value="onrobot",
            choices=["onrobot", "robotiq"],
        ),
        workcell_state_publisher,
        rviz_node,
        onrobot_visualizer,
        robotiq_visualizer,
        onrobot_robot,
        robotiq_robot,
    ])
