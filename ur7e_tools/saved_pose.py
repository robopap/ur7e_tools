#!/usr/bin/env python3

import argparse
import re
import sys
import time
from pathlib import Path

import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from ur7e_tools.workcell_safety_runtime import (
    LiveWorkcellSafety,
    print_safety_report,
)


BASE_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


TARGETS = {
    "robot1": {
        "namespace": "/robot1",
        "prefix": "robot1_",
    },
    "robot2": {
        "namespace": "/robot2",
        "prefix": "robot2_",
    },
}


CONFIG_DIR = (
    Path(__file__).resolve().parents[1]
    / "config"
)

START_HOLD_SEC = 1.0
DEFAULT_DURATION_SEC = 10.0
DEFAULT_TOLERANCE_RAD = 0.02


def seconds_to_duration(seconds):
    whole = int(seconds)
    nanoseconds = int(
        round((seconds - whole) * 1e9)
    )

    if nanoseconds >= 1_000_000_000:
        whole += 1
        nanoseconds -= 1_000_000_000

    return Duration(
        sec=whole,
        nanosec=nanoseconds,
    )


def normalize_pose_name(name):
    name = name.strip().lower()

    if not re.fullmatch(
        r"[a-z0-9][a-z0-9_-]*",
        name,
    ):
        raise ValueError(
            "Pose name may contain only "
            "letters, numbers, '_' and '-'."
        )

    return name


def pose_path(target, pose_name):
    pose_name = normalize_pose_name(
        pose_name
    )

    # Keep compatibility with the existing
    # validated HOME configuration files.
    if pose_name == "home":
        return (
            CONFIG_DIR
            / f"home_{target}.yaml"
        )

    return (
        CONFIG_DIR
        / f"pose_{target}_{pose_name}.yaml"
    )


def discover_poses(target):
    poses = {}

    home_path = (
        CONFIG_DIR
        / f"home_{target}.yaml"
    )

    if home_path.is_file():
        poses["home"] = home_path

    prefix = f"pose_{target}_"

    for path in sorted(
        CONFIG_DIR.glob(
            f"{prefix}*.yaml"
        )
    ):
        stem = path.stem

        pose_name = stem[
            len(prefix):
        ]

        if pose_name:
            poses[pose_name] = path

    return poses


def load_pose(target, pose_name):
    path = pose_path(
        target,
        pose_name,
    )

    if not path.is_file():
        available = ", ".join(
            sorted(discover_poses(target))
        )

        raise RuntimeError(
            f"Saved pose '{pose_name}' "
            f"does not exist for {target}.\n"
            f"Available: {available or '(none)'}"
        )

    with path.open(
        "r",
        encoding="utf-8",
    ) as f:
        data = yaml.safe_load(f) or {}

    joint_names = data.get(
        "joint_names"
    )
    positions = data.get(
        "positions"
    )

    if joint_names != BASE_JOINTS:
        raise RuntimeError(
            f"Invalid joint_names in:\n{path}"
        )

    if (
        not isinstance(positions, list)
        or len(positions) != 6
    ):
        raise RuntimeError(
            f"Invalid positions in:\n{path}"
        )

    return (
        [float(x) for x in positions],
        path,
    )


class SavedPoseNode(Node):

    def __init__(self, target):
        super().__init__(
            f"saved_pose_{target}"
        )

        if target not in TARGETS:
            raise ValueError(
                f"Unsupported target: {target}"
            )

        self.target = target

        cfg = TARGETS[target]

        self.namespace = cfg["namespace"]
        self.prefix = cfg["prefix"]

        self.joint_names = [
            self.prefix + joint
            for joint in BASE_JOINTS
        ]

        self.joint_state_topic = (
            f"{self.namespace}/joint_states"
        )

        self.action_topic = (
            f"{self.namespace}/"
            "joint_trajectory_controller/"
            "follow_joint_trajectory"
        )

        self.latest_joint_state = None

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._joint_state_callback,
            10,
        )

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_topic,
        )

    def _joint_state_callback(
        self,
        msg,
    ):
        self.latest_joint_state = msg

    def get_current_positions(
        self,
        timeout=5.0,
    ):
        start = time.monotonic()

        while (
            rclpy.ok()
            and self.latest_joint_state is None
            and time.monotonic() - start
            < timeout
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        if self.latest_joint_state is None:
            raise RuntimeError(
                "No joint state received from "
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

    def save_current_pose(
        self,
        pose_name,
    ):
        pose_name = normalize_pose_name(
            pose_name
        )

        # HOME remains owned by home_pose.py.
        if pose_name == "home":
            raise RuntimeError(
                "HOME is managed by home_pose.py. "
                "Use another saved-pose name."
            )

        positions = (
            self.get_current_positions()
        )

        path = pose_path(
            self.target,
            pose_name,
        )

        data = {
            "joint_names": BASE_JOINTS,
            "positions": positions,
        }

        with path.open(
            "w",
            encoding="utf-8",
        ) as f:
            yaml.safe_dump(
                data,
                f,
                sort_keys=False,
            )

        print()
        print(
            f"Saved pose '{pose_name}' "
            f"for {self.target}:"
        )

        for joint, value in zip(
            BASE_JOINTS,
            positions,
        ):
            print(
                f"  {joint:22s} "
                f"= {value: .6f}"
            )

        print()
        print(f"Saved in: {path}")

    def print_comparison(
        self,
        current,
        target,
        title,
    ):
        errors = [
            target_q - current_q
            for current_q, target_q
            in zip(current, target)
        ]

        print()
        print("=" * 72)
        print(title)
        print("=" * 72)

        for (
            joint,
            current_q,
            target_q,
            error,
        ) in zip(
            BASE_JOINTS,
            current,
            target,
            errors,
        ):
            print(
                f"{joint:22s} "
                f"current={current_q: .6f}  "
                f"target={target_q: .6f}  "
                f"error={error: .6f}"
            )

        max_error = max(
            abs(error)
            for error in errors
        )

        print()
        print(
            "Maximum absolute joint error: "
            f"{max_error:.6f} rad"
        )

        return max_error

    def build_move_goal(
        self,
        current,
        target,
        duration,
    ):
        goal = (
            FollowJointTrajectory.Goal()
        )

        goal.trajectory.joint_names = (
            list(self.joint_names)
        )

        # Hold measured current position briefly.
        p0 = JointTrajectoryPoint()

        p0.positions = [
            float(x)
            for x in current
        ]

        p0.time_from_start = (
            seconds_to_duration(
                START_HOLD_SEC
            )
        )

        goal.trajectory.points.append(
            p0
        )

        # Conservative move to saved pose.
        p1 = JointTrajectoryPoint()

        p1.positions = [
            float(x)
            for x in target
        ]

        p1.time_from_start = (
            seconds_to_duration(
                START_HOLD_SEC
                + duration
            )
        )

        goal.trajectory.points.append(
            p1
        )

        return goal

    def run_path_safety_check(
        self,
        current,
        target,
    ):
        """
        Mandatory full-workcell safety scan.

        Checks the complete interpolated path of the
        moving robot against:
          - the live stationary other robot
          - the table

        The live expanded robot_description is used,
        including arm, 2FG7 and camera geometry.
        """

        checker = LiveWorkcellSafety(
            self
        )

        report = checker.check_path(
            moving_robot=self.target,
            moving_start_q=current,
            moving_target_q=target,
        )

        print_safety_report(
            report
        )

        return report

    def move_to_pose(
        self,
        pose_name,
        duration=DEFAULT_DURATION_SEC,
        tolerance=DEFAULT_TOLERANCE_RAD,
        skip_confirmation=False,
    ):
        if duration <= 0.0:
            raise ValueError(
                "duration must be > 0"
            )

        if tolerance <= 0.0:
            raise ValueError(
                "tolerance must be > 0"
            )

        target, path = load_pose(
            self.target,
            pose_name,
        )

        current = (
            self.get_current_positions()
        )

        start_error = (
            self.print_comparison(
                current,
                target,
                (
                    f"{self.target.upper()} "
                    f"TARGET CHECK: {pose_name}"
                ),
            )
        )

        if start_error <= tolerance:
            print()
            print(
                f"{self.target} is already "
                f"at '{pose_name}'."
            )
            return True

        print()
        print("Saved pose file:")
        print(path)

        # ------------------------------------------------
        # SAFETY GATE 1
        # Full path must be safe before confirmation.
        # ------------------------------------------------

        print()
        print(
            "Running mandatory full-path "
            "workcell safety check..."
        )

        report = self.run_path_safety_check(
            current,
            target,
        )

        if not report.safe:
            print()
            print(
                "MOVE BLOCKED: trajectory "
                "was NOT sent."
            )
            return False

        # ------------------------------------------------
        # User/UI confirmation happens only after the
        # candidate path has passed the safety scan.
        # --yes skips ONLY this confirmation, never safety.
        # ------------------------------------------------

        if not skip_confirmation:
            expected = (
                f"MOVE {self.target.upper()} "
                f"TO {pose_name.upper()}"
            )

            print()
            print(
                "Requested motion:"
            )
            print(
                f"  {self.target} "
                f"→ {pose_name}"
            )

            answer = input(
                f"\nType {expected} "
                "to continue: "
            )

            if answer.strip() != expected:
                print(
                    "Motion cancelled."
                )
                return False

        # ------------------------------------------------
        # SAFETY GATE 2
        # Confirmation may take time. Obtain a fresh
        # measured moving-robot state and re-run the full
        # workcell scan immediately before sending motion.
        # ------------------------------------------------

        self.latest_joint_state = None

        current = (
            self.get_current_positions()
        )

        print()
        print(
            "Re-checking full path immediately "
            "before trajectory submission..."
        )

        report = self.run_path_safety_check(
            current,
            target,
        )

        if not report.safe:
            print()
            print(
                "MOVE BLOCKED: workcell state "
                "changed or path is unsafe. "
                "Trajectory was NOT sent."
            )
            return False

        print()
        print(
            "Waiting for action server:"
        )
        print(self.action_topic)

        if not (
            self.action_client.wait_for_server(
                timeout_sec=5.0
            )
        ):
            raise RuntimeError(
                "Trajectory action server "
                "is not available."
            )

        goal = self.build_move_goal(
            current,
            target,
            duration,
        )

        print()
        print(
            f"Moving {self.target} "
            f"to '{pose_name}' "
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

        wrapped = result_future.result()

        if wrapped is None:
            raise RuntimeError(
                "No trajectory result received."
            )

        error_code = (
            wrapped.result.error_code
        )

        if error_code != 0:
            raise RuntimeError(
                "Trajectory failed with "
                f"error code {error_code}: "
                f"{wrapped.result.error_string}"
            )

        # Get fresh measured state after motion.
        self.latest_joint_state = None

        reached = (
            self.get_current_positions()
        )

        reached_error = (
            self.print_comparison(
                reached,
                target,
                (
                    f"{self.target.upper()} "
                    f"REACHED CHECK: {pose_name}"
                ),
            )
        )

        if reached_error > tolerance:
            raise RuntimeError(
                f"{self.target} trajectory "
                "completed but target tolerance "
                "was not satisfied."
            )

        print()
        print(
            f"{self.target}: AT TARGET "
            f"'{pose_name}'"
        )

        return True


def print_saved_poses(target):
    poses = discover_poses(target)

    print(
        f"Saved poses for {target}:"
    )

    if not poses:
        print("  (none)")
        return

    for name, path in poses.items():
        print(
            f"  {name:28s} "
            f"{path.name}"
        )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Save, list, or move a UR7e "
            "to a named joint-space pose."
        )
    )

    parser.add_argument(
        "--target",
        required=True,
        choices=[
            "robot1",
            "robot2",
        ],
    )

    mode = (
        parser.add_mutually_exclusive_group(
            required=True
        )
    )

    mode.add_argument(
        "--list",
        action="store_true",
        help="List saved poses.",
    )

    mode.add_argument(
        "--save",
        action="store_true",
        help="Save current joints as a named pose.",
    )

    mode.add_argument(
        "--move",
        action="store_true",
        help="Move to a named saved pose.",
    )

    mode.add_argument(
        "--check",
        action="store_true",
        help="Check current joints against a saved pose without motion.",
    )

    mode.add_argument(
        "--path-check",
        action="store_true",
        help=(
            "Run the full workcell path safety "
            "check without sending any motion."
        ),
    )

    parser.add_argument(
        "--pose",
        default=None,
        help="Saved pose name.",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_SEC,
    )

    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE_RAD,
    )

    parser.add_argument(
        "--yes",
        action="store_true",
        help=(
            "Skip CLI typed confirmation. "
            "Intended for a UI that already "
            "confirmed the motion."
        ),
    )

    cli_args = remove_ros_args(
        args=sys.argv
    )[1:]

    return parser.parse_args(
        cli_args
    )


def main(args=None):
    cli = parse_arguments()

    if cli.list:
        print_saved_poses(
            cli.target
        )
        return

    if not cli.pose:
        raise RuntimeError(
            "--pose is required for "
            "--save, --check, --path-check and --move."
        )

    pose_name = normalize_pose_name(
        cli.pose
    )

    rclpy.init(args=args)

    node = SavedPoseNode(
        cli.target
    )

    try:
        if cli.save:
            node.save_current_pose(
                pose_name
            )

        elif cli.check:
            target, path = load_pose(
                cli.target,
                pose_name,
            )

            current = node.get_current_positions()

            error = node.print_comparison(
                current,
                target,
                (
                    f"{cli.target.upper()} "
                    f"POSE CHECK: {pose_name}"
                ),
            )

            print()
            print("Saved pose file:")
            print(path)

            print()

            if error <= cli.tolerance:
                print(
                    f"{cli.target}: AT TARGET "
                    f"'{pose_name}'"
                )
            else:
                print(
                    f"{cli.target}: NOT AT TARGET "
                    f"'{pose_name}'"
                )

        elif cli.path_check:
            target, path = load_pose(
                cli.target,
                pose_name,
            )

            current = (
                node.get_current_positions()
            )

            node.print_comparison(
                current,
                target,
                (
                    f"{cli.target.upper()} "
                    f"PATH CHECK: {pose_name}"
                ),
            )

            print()
            print("Saved pose file:")
            print(path)

            report = (
                node.run_path_safety_check(
                    current,
                    target,
                )
            )

            if not report.safe:
                print()
                print(
                    "PATH CHECK RESULT: "
                    "MOVE BLOCKED"
                )
                sys.exit(2)

            print()
            print(
                "PATH CHECK RESULT: "
                "PATH ACCEPTED"
            )

        elif cli.move:
            success = node.move_to_pose(
                pose_name,
                duration=cli.duration,
                tolerance=cli.tolerance,
                skip_confirmation=cli.yes,
            )

            if not success:
                sys.exit(2)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
