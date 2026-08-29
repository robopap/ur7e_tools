#!/usr/bin/env python3

import argparse
import os
import sys
import time
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


BASE_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


TARGETS = {
    "ur5": {
        "namespace": "",
        "prefix": "",
    },
    "robot1": {
        "namespace": "/robot1",
        "prefix": "robot1_",
    },
    "robot2": {
        "namespace": "/robot2",
        "prefix": "robot2_",
    },
}



CONFIG_FILES = {
    "ur5": os.path.expanduser(
        "~/ros2_ws/src/ur7e_tools/config/home_ur5.yaml"
    ),
    "robot1": os.path.expanduser(
        "~/ros2_ws/src/ur7e_tools/config/home_robot1.yaml"
    ),
    "robot2": os.path.expanduser(
        "~/ros2_ws/src/ur7e_tools/config/home_robot2.yaml"
    ),
}


def config_path(target):
    return CONFIG_FILES[target]


def load_config(target):

    path = config_path(target)

    if not os.path.exists(path):
        return {}

    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def save_config(target, data):

    path = config_path(target)

    with open(path, "w") as f:
        yaml.safe_dump(
            data,
            f,
            sort_keys=False,
        )




class HomePoseNode(Node):

    def __init__(self, target):
        super().__init__(
            f"home_pose_{target}"
        )

        self.target = target

        target_config = TARGETS[target]

        self.namespace = target_config["namespace"]
        self.prefix = target_config["prefix"]

        self.joint_names = [
            self.prefix + joint
            for joint in BASE_JOINTS
        ]

        if self.namespace:
            self.joint_state_topic = (
                f"{self.namespace}/joint_states"
            )

            self.action_topic = (
                f"{self.namespace}/"
                "joint_trajectory_controller/"
                "follow_joint_trajectory"
            )
        else:
            self.joint_state_topic = "/joint_states"

            self.action_topic = (
                "/joint_trajectory_controller/"
                "follow_joint_trajectory"
            )

        self.latest_joint_state = None

        self.subscription = self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_topic,
        )

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def get_current_positions(self, timeout=5.0):

        start = time.time()

        while (
            rclpy.ok()
            and self.latest_joint_state is None
            and time.time() - start < timeout
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        if self.latest_joint_state is None:
            raise RuntimeError(
                f"No joint state received from "
                f"{self.joint_state_topic}"
            )

        values = dict(
            zip(
                self.latest_joint_state.name,
                self.latest_joint_state.position,
            )
        )

        missing = [
            name
            for name in self.joint_names
            if name not in values
        ]

        if missing:
            raise RuntimeError(
                "Missing joints in joint_states: "
                + ", ".join(missing)
            )

        return [
            float(values[name])
            for name in self.joint_names
        ]

    def save_current_as_home(self):

        positions = self.get_current_positions()

        data = {
            "joint_names": BASE_JOINTS,
            "positions": positions,
        }

        save_config(
            self.target,
            data,
        )

        print()
        print(
            f"Saved HOME for {self.target}:"
        )

        for joint, value in zip(
            BASE_JOINTS,
            positions,
        ):
            print(
                f"  {joint:22s} = {value: .6f}"
            )

        print()
        print(f"Saved in: {config_path(self.target)}")

    def move_to_home(self, duration):

        data = load_config(self.target)
        positions = data.get("positions")
        if positions is None:
            raise RuntimeError(
                f"No HOME saved for '{self.target}'. "
                f"Run first with --save."
            )

        if len(positions) != 6:
            raise RuntimeError(
                f"Invalid HOME configuration "
                f"for {self.target}"
            )

        print(
            f"Waiting for action server:\n"
            f"  {self.action_topic}"
        )

        if not self.action_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "Trajectory action server "
                "is not available."
            )

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = (
            self.joint_names
        )

        point = JointTrajectoryPoint()

        point.positions = [
            float(value)
            for value in positions
        ]

        seconds = int(duration)
        nanoseconds = int(
            (duration - seconds) * 1e9
        )

        point.time_from_start = Duration(
            sec=seconds,
            nanosec=nanoseconds,
        )

        goal.trajectory.points = [point]

        print(
            f"Moving {self.target} to HOME "
            f"over {duration:.1f} s..."
        )

        future = (
            self.action_client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        goal_handle = future.result()

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            raise RuntimeError(
                "Trajectory goal was rejected."
            )

        print("Goal accepted.")

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        result = result_future.result()

        if result is None:
            raise RuntimeError(
                "No trajectory result received."
            )

        error_code = (
            result.result.error_code
        )

        if error_code != 0:
            raise RuntimeError(
                f"Trajectory failed "
                f"with error code {error_code}"
            )

        print(
            f"{self.target} reached HOME."
        )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--target",
        required=True,
        choices=[
            "ur5",
            "robot1",
            "robot2",
        ],
    )

    mode = parser.add_mutually_exclusive_group(
        required=True
    )

    mode.add_argument(
        "--save",
        action="store_true",
        help="Save current joint pose as HOME",
    )

    mode.add_argument(
        "--move",
        action="store_true",
        help="Move to saved HOME",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
    )

    args = parser.parse_args()

    if args.duration <= 0:
        print(
            "Duration must be > 0.",
            file=sys.stderr,
        )
        sys.exit(1)

    rclpy.init()

    node = HomePoseNode(
        args.target
    )

    try:

        if args.save:
            node.save_current_as_home()

        elif args.move:
            node.move_to_home(
                args.duration
            )

    except Exception as exc:

        print(
            f"ERROR: {exc}",
            file=sys.stderr,
        )

        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()