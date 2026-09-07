#!/usr/bin/env python3

import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy

from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState

from ur7e_tools.workcell_safety import (
    ARM_JOINTS,
    check_move_path,
    load_robot_scene,
    load_table_scene,
)


REPO_ROOT = Path(__file__).resolve().parents[1]

ROBOT_ROBOT_MARGIN_M = 0.020
TABLE_MARGIN_M = 0.010
MAX_SAMPLE_STEP_RAD = 0.020

# Per-process geometry cache. A saved-pose MOVE performs more than one safety
# scan; the robot descriptions and collision meshes do not change between
# those scans, so rebuilding Pinocchio/Coal models is unnecessary.
_SCENE_CACHE = None


class LiveWorkcellSafety:

    def __init__(self, node):
        self.node = node

        self.arm_states = {
            "robot1": None,
            "robot2": None,
        }

        self.visual_states = {
            "robot1": None,
            "robot2": None,
        }

        self.subscriptions = []

        for robot in ["robot1", "robot2"]:

            self.subscriptions.append(
                node.create_subscription(
                    JointState,
                    f"/{robot}/joint_states",
                    lambda msg, r=robot:
                        self._arm_callback(r, msg),
                    10,
                )
            )

            self.subscriptions.append(
                node.create_subscription(
                    JointState,
                    f"/{robot}/visual_joint_states",
                    lambda msg, r=robot:
                        self._visual_callback(r, msg),
                    10,
                )
            )

    def _arm_callback(self, robot, msg):
        self.arm_states[robot] = msg

    def _visual_callback(self, robot, msg):
        self.visual_states[robot] = msg

    def _wait_for_message(
        self,
        storage,
        robot,
        timeout=5.0,
    ):
        start = time.monotonic()

        while (
            rclpy.ok()
            and storage[robot] is None
            and time.monotonic() - start < timeout
        ):
            rclpy.spin_once(
                self.node,
                timeout_sec=0.1,
            )

        if storage[robot] is None:
            raise RuntimeError(
                f"No live state received for {robot}"
            )

        return storage[robot]

    def get_arm_q(self, robot):
        msg = self._wait_for_message(
            self.arm_states,
            robot,
        )

        values = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        names = [
            f"{robot}_{joint}"
            for joint in ARM_JOINTS
        ]

        missing = [
            name
            for name in names
            if name not in values
        ]

        if missing:
            raise RuntimeError(
                f"{robot}: missing arm joints: "
                + ", ".join(missing)
            )

        return np.asarray(
            [
                float(values[name])
                for name in names
            ],
            dtype=float,
        )

    def get_gripper_position(self, robot):
        msg = self._wait_for_message(
            self.visual_states,
            robot,
        )

        name = (
            f"{robot}_gripper_gripper_joint"
        )

        values = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        if name not in values:
            raise RuntimeError(
                f"{robot}: gripper state "
                "not available in visual_joint_states"
            )

        return float(values[name])

    def get_robot_description(self, robot):
        remote = (
            f"/{robot}/robot_state_publisher"
        )

        service = (
            f"{remote}/get_parameters"
        )

        client = self.node.create_client(
            GetParameters,
            service,
        )

        if not client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                f"Parameter service unavailable: "
                f"{service}"
            )

        request = GetParameters.Request()
        request.names = [
            "robot_description"
        ]

        future = client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=10.0,
        )

        response = future.result()

        if (
            response is None
            or not response.values
        ):
            raise RuntimeError(
                f"No robot_description "
                f"from {robot}"
            )

        xml = (
            response.values[0]
            .string_value
        )

        if not xml.strip():
            raise RuntimeError(
                f"Empty robot_description "
                f"from {robot}"
            )

        return xml

    def _refresh_live_states(self):
        """Require new ROS samples for every safety scan."""
        for robot in ("robot1", "robot2"):
            self.arm_states[robot] = None
            self.visual_states[robot] = None

    def _get_cached_scenes(self):
        """Load live robot collision scenes once per saved-pose process."""
        global _SCENE_CACHE

        if _SCENE_CACHE is not None:
            return _SCENE_CACHE

        robot1_xml = self.get_robot_description(
            "robot1"
        )
        robot2_xml = self.get_robot_description(
            "robot2"
        )

        with tempfile.TemporaryDirectory(
            prefix="ur7e_safety_"
        ) as tmpdir:
            tmpdir = Path(tmpdir)

            r1_path = tmpdir / "robot1_live.urdf"
            r2_path = tmpdir / "robot2_live.urdf"

            r1_path.write_text(
                robot1_xml,
                encoding="utf-8",
            )
            r2_path.write_text(
                robot2_xml,
                encoding="utf-8",
            )

            robot1_scene = load_robot_scene(
                "robot1",
                r1_path,
            )
            robot2_scene = load_robot_scene(
                "robot2",
                r2_path,
            )

        (
            _table_model,
            _table_data,
            table_geom_model,
            table_geom_data,
        ) = load_table_scene(
            REPO_ROOT / "urdf/workcell.urdf"
        )

        _SCENE_CACHE = (
            {
                "robot1": robot1_scene,
                "robot2": robot2_scene,
            },
            table_geom_model,
            table_geom_data,
        )

        return _SCENE_CACHE

    def check_path(
        self,
        moving_robot,
        moving_start_q,
        moving_target_q,
    ):
        if moving_robot not in {
            "robot1",
            "robot2",
        }:
            raise ValueError(
                f"Unsupported robot: "
                f"{moving_robot}"
            )

        other_robot = (
            "robot2"
            if moving_robot == "robot1"
            else "robot1"
        )

        # The second pre-send scan must use fresh live robot/gripper state.
        self._refresh_live_states()

        other_q = self.get_arm_q(
            other_robot
        )

        moving_gripper = (
            self.get_gripper_position(
                moving_robot
            )
        )

        other_gripper = (
            self.get_gripper_position(
                other_robot
            )
        )

        (
            scenes,
            table_geom_model,
            table_geom_data,
        ) = self._get_cached_scenes()

        report = check_move_path(
            moving_scene=scenes[
                moving_robot
            ],
            other_scene=scenes[
                other_robot
            ],
            table_geom_model=(
                table_geom_model
            ),
            table_geom_data=(
                table_geom_data
            ),
            moving_start_arm_q=(
                moving_start_q
            ),
            moving_target_arm_q=(
                moving_target_q
            ),
            other_arm_q=other_q,
            moving_gripper_position=(
                moving_gripper
            ),
            other_gripper_position=(
                other_gripper
            ),
            robot_robot_margin_m=(
                ROBOT_ROBOT_MARGIN_M
            ),
            table_margin_m=(
                TABLE_MARGIN_M
            ),
            max_sample_step_rad=(
                MAX_SAMPLE_STEP_RAD
            ),
        )

        return report



def print_safety_report(report):

    print()
    print("=" * 72)
    print("FULL-PATH WORKCELL SAFETY CHECK")
    print("=" * 72)

    print(
        f"Samples checked: {report.samples}"
    )

    if np.isfinite(
        report.min_robot_robot_distance_m
    ):
        print(
            "Minimum robot-robot clearance: "
            f"{1000.0 * report.min_robot_robot_distance_m:.2f} mm"
        )

    if np.isfinite(
        report.min_table_distance_m
    ):
        print(
            "Minimum robot-table clearance: "
            f"{1000.0 * report.min_table_distance_m:.2f} mm"
        )

    print(
        "Required robot-robot margin: "
        f"{1000.0 * ROBOT_ROBOT_MARGIN_M:.1f} mm"
    )

    print(
        "Required table margin: "
        f"{1000.0 * TABLE_MARGIN_M:.1f} mm"
    )

    if (
        not np.isfinite(report.min_robot_robot_distance_m)
        and not np.isfinite(report.min_table_distance_m)
    ):
        print(
            "Clearance mode: FAST THRESHOLD "
            "(exact minimum distances not computed)"
        )

    print()

    if report.safe:
        print("SAFETY RESULT: PATH ACCEPTED")

    else:
        print("SAFETY RESULT: MOVE BLOCKED")
        print(
            "Reason:",
            report.collision_kind,
        )

        if report.sample_index is not None:
            print(
                "Sample:",
                report.sample_index,
            )

        if report.object_a:
            print(
                "Object A:",
                report.object_a,
            )

        if report.object_b:
            print(
                "Object B:",
                report.object_b,
            )
