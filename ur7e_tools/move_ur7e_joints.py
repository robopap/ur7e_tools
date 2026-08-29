#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


BASE_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class UR7eJointMover(Node):

    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.prefix = f"{robot_name}_"

        self.joint_names = [
            self.prefix + name
            for name in BASE_JOINT_NAMES
        ]

        super().__init__(
            "ur7e_joint_mover",
            namespace=robot_name,
        )

        self.current_positions = None

        self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            10,
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
        )

    def joint_state_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))

        if all(name in positions for name in self.joint_names):
            self.current_positions = {
                name: positions[name]
                for name in self.joint_names
            }

    def wait_for_joint_states(self):
        self.get_logger().info(
            f"Waiting for /{self.robot_name}/joint_states..."
        )

        while rclpy.ok() and self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_positions is None:
            raise RuntimeError("Could not receive joint states.")

        self.get_logger().info("Joint states received.")

    def move_relative(self, base_joint_name, delta, duration):

        if base_joint_name not in BASE_JOINT_NAMES:
            raise ValueError(
                f"Unknown joint '{base_joint_name}'"
            )

        full_joint_name = self.prefix + base_joint_name

        current = [
            self.current_positions[name]
            for name in self.joint_names
        ]

        target = current.copy()

        index = self.joint_names.index(full_joint_name)
        target[index] += delta

        print(
            f"\nCurrent joint positions for {self.robot_name}:"
        )

        for name, value in zip(self.joint_names, current):
            print(f"{name:32s} = {value: .6f}")

        print(
            f"\nMoving {full_joint_name} "
            f"by {delta:+.3f} rad "
            f"over {duration:.1f} seconds..."
        )

        point = JointTrajectoryPoint()
        point.positions = target

        whole_seconds = int(duration)
        nanoseconds = int(
            (duration - whole_seconds) * 1e9
        )

        point.time_from_start.sec = whole_seconds
        point.time_from_start.nanosec = nanoseconds

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.points = [point]

        self.get_logger().info(
            "Waiting for trajectory action server..."
        )

        if not self.action_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "Trajectory action server is not available."
            )

        self.get_logger().info(
            "Sending trajectory goal..."
        )

        goal_future = self.action_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            goal_future,
        )

        goal_handle = goal_future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                "Trajectory goal was rejected."
            )

        self.get_logger().info("Goal accepted.")

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        result = result_future.result()

        if result is None:
            raise RuntimeError(
                "No trajectory result received."
            )

        error_code = result.result.error_code

        if error_code == 0:
            self.get_logger().info(
                "Trajectory completed successfully."
            )
        else:
            self.get_logger().error(
                f"Trajectory failed with "
                f"error code {error_code}: "
                f"{result.result.error_string}"
            )


def parse_arguments():

    parser = argparse.ArgumentParser(
        description=(
            "Move one joint of robot1 or robot2 "
            "relative to its current position."
        )
    )

    parser.add_argument(
        "--robot",
        required=True,
        choices=["robot1", "robot2"],
        help="Robot namespace.",
    )

    parser.add_argument(
        "--joint",
        required=True,
        choices=BASE_JOINT_NAMES,
        help="Joint to move.",
    )

    parser.add_argument(
        "--delta",
        required=True,
        type=float,
        help="Relative movement in radians.",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Trajectory duration in seconds.",
    )

    cli_args = remove_ros_args(args=sys.argv)[1:]

    return parser.parse_args(cli_args)


def main(args=None):

    cli_args = parse_arguments()

    rclpy.init(args=args)

    node = UR7eJointMover(cli_args.robot)

    try:
        node.wait_for_joint_states()

        node.move_relative(
            cli_args.joint,
            cli_args.delta,
            cli_args.duration,
        )

    except Exception as exc:
        node.get_logger().error(str(exc))

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()