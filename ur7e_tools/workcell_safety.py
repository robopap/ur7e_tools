#!/usr/bin/env python3

from dataclasses import dataclass
from pathlib import Path
import math

import coal
import numpy as np
import pinocchio as pin
import yaml


PACKAGE_DIRS = [
    "/home/mines/ros2_ws/src",
    "/opt/ros/humble/share",
]

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

GRIPPER_CLOSED = 0.0115
GRIPPER_OPEN = 0.0305

DEFAULT_MAX_SAMPLE_STEP_RAD = 0.05


@dataclass
class RobotScene:
    name: str
    model: object
    data: object
    geom_model: object
    geom_data: object


@dataclass
class SafetyReport:
    safe: bool
    samples: int
    min_robot_robot_distance_m: float
    min_table_distance_m: float
    collision_kind: str | None = None
    sample_index: int | None = None
    object_a: str | None = None
    object_b: str | None = None


def load_pose_yaml(path):
    path = Path(path)

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data.get("joint_names") != ARM_JOINTS:
        raise RuntimeError(
            f"Unexpected joint_names in {path}"
        )

    q = np.asarray(
        data.get("positions"),
        dtype=float,
    )

    if q.shape != (6,):
        raise RuntimeError(
            f"Expected six positions in {path}"
        )

    return q


def load_robot_scene(robot_name, urdf_path):
    urdf_path = Path(urdf_path)

    model = pin.buildModelFromUrdf(
        str(urdf_path)
    )

    geom_model = pin.buildGeomFromUrdf(
        model,
        str(urdf_path),
        pin.GeometryType.COLLISION,
        PACKAGE_DIRS,
    )

    return RobotScene(
        name=robot_name,
        model=model,
        data=model.createData(),
        geom_model=geom_model,
        geom_data=geom_model.createData(),
    )


def load_table_scene(workcell_urdf):
    workcell_urdf = Path(workcell_urdf)

    model = pin.buildModelFromUrdf(
        str(workcell_urdf)
    )

    # Table is currently declared as VISUAL,
    # but we use the exact same mesh as an
    # obstacle for safety calculations.
    geom_model = pin.buildGeomFromUrdf(
        model,
        str(workcell_urdf),
        pin.GeometryType.VISUAL,
        PACKAGE_DIRS,
    )

    data = model.createData()
    geom_data = geom_model.createData()

    q = np.zeros(model.nq)

    pin.updateGeometryPlacements(
        model,
        data,
        geom_model,
        geom_data,
        q,
    )

    return (
        model,
        data,
        geom_model,
        geom_data,
    )


def build_robot_q(
    scene,
    arm_q,
    gripper_position,
):
    arm_q = np.asarray(
        arm_q,
        dtype=float,
    )

    if arm_q.shape != (6,):
        raise ValueError(
            "arm_q must have shape (6,)"
        )

    q = np.zeros(scene.model.nq)

    values = {
        f"{scene.name}_{joint}": float(value)
        for joint, value in zip(
            ARM_JOINTS,
            arm_q,
        )
    }

    # 2FG7 mimic:
    # right_finger_joint =
    # 1.0 * gripper_joint
    values[
        f"{scene.name}_gripper_gripper_joint"
    ] = float(gripper_position)

    values[
        f"{scene.name}_gripper_right_finger_joint"
    ] = float(gripper_position)

    for joint_name, value in values.items():
        jid = scene.model.getJointId(
            joint_name
        )

        if jid == 0:
            raise RuntimeError(
                f"Joint not found: {joint_name}"
            )

        joint = scene.model.joints[jid]

        if joint.nq != 1:
            raise RuntimeError(
                f"{joint_name}: expected nq=1"
            )

        q[joint.idx_q] = value

    return q


def update_robot_scene(scene, q):
    pin.updateGeometryPlacements(
        scene.model,
        scene.data,
        scene.geom_model,
        scene.geom_data,
        q,
    )


def coal_transform(M):
    return coal.Transform3s(
        np.asarray(
            M.rotation,
            dtype=float,
        ),
        np.asarray(
            M.translation,
            dtype=float,
        ),
    )


def objects_collide(
    geometry_a,
    placement_a,
    geometry_b,
    placement_b,
):
    request = coal.CollisionRequest()
    result = coal.CollisionResult()

    count = coal.collide(
        geometry_a,
        coal_transform(placement_a),
        geometry_b,
        coal_transform(placement_b),
        request,
        result,
    )

    try:
        flag = result.isCollision()
    except AttributeError:
        flag = False

    return bool(count > 0 or flag)


def objects_distance(
    geometry_a,
    placement_a,
    geometry_b,
    placement_b,
):
    request = coal.DistanceRequest()
    result = coal.DistanceResult()

    distance = coal.distance(
        geometry_a,
        coal_transform(placement_a),
        geometry_b,
        coal_transform(placement_b),
        request,
        result,
    )

    return float(distance)


def generate_joint_path(
    q_start,
    q_target,
    max_step_rad=DEFAULT_MAX_SAMPLE_STEP_RAD,
):
    q_start = np.asarray(
        q_start,
        dtype=float,
    )

    q_target = np.asarray(
        q_target,
        dtype=float,
    )

    max_delta = float(
        np.max(
            np.abs(
                q_target - q_start
            )
        )
    )

    n_segments = max(
        1,
        int(
            math.ceil(
                max_delta / max_step_rad
            )
        ),
    )

    alpha = np.linspace(
        0.0,
        1.0,
        n_segments + 1,
    )

    return (
        q_start[None, :]
        + alpha[:, None]
        * (
            q_target
            - q_start
        )[None, :]
    )


def check_move_path(
    moving_scene,
    other_scene,
    table_geom_model,
    table_geom_data,
    moving_start_arm_q,
    moving_target_arm_q,
    other_arm_q,
    moving_gripper_position=GRIPPER_OPEN,
    other_gripper_position=GRIPPER_OPEN,
    robot_robot_margin_m=0.0,
    table_margin_m=0.0,
    max_sample_step_rad=DEFAULT_MAX_SAMPLE_STEP_RAD,
):
    """
    Fast coarse guard for setup moves.

    This intentionally checks only actual mesh collisions along a sampled
    joint-space path.  It does NOT compute exact pairwise clearances.

    Checked:
      - moving robot vs stationary other robot
      - moving robot vs table
      - whole interpolated path

    robot_robot_margin_m/table_margin_m are accepted for API compatibility,
    but no expensive distance-margin calculation is performed here.
    """
    path = generate_joint_path(
        moving_start_arm_q,
        moving_target_arm_q,
        max_sample_step_rad,
    )

    other_q = build_robot_q(
        other_scene,
        other_arm_q,
        other_gripper_position,
    )
    update_robot_scene(
        other_scene,
        other_q,
    )

    for sample_index, arm_q in enumerate(path):
        moving_q = build_robot_q(
            moving_scene,
            arm_q,
            moving_gripper_position,
        )
        update_robot_scene(
            moving_scene,
            moving_q,
        )

        # Moving robot vs stationary robot.
        for i, obj_a in enumerate(
            moving_scene.geom_model.geometryObjects
        ):
            placement_a = moving_scene.geom_data.oMg[i]

            for j, obj_b in enumerate(
                other_scene.geom_model.geometryObjects
            ):
                placement_b = other_scene.geom_data.oMg[j]

                if objects_collide(
                    obj_a.geometry,
                    placement_a,
                    obj_b.geometry,
                    placement_b,
                ):
                    return SafetyReport(
                        safe=False,
                        samples=len(path),
                        min_robot_robot_distance_m=float("inf"),
                        min_table_distance_m=float("inf"),
                        collision_kind="robot_robot",
                        sample_index=sample_index,
                        object_a=obj_a.name,
                        object_b=obj_b.name,
                    )

        # Moving robot vs table.
        for i, obj_a in enumerate(
            moving_scene.geom_model.geometryObjects
        ):
            # Intentional robot mounting contact is ignored.
            if obj_a.name.endswith("base_link_inertia_0"):
                continue

            placement_a = moving_scene.geom_data.oMg[i]

            for j, table_obj in enumerate(
                table_geom_model.geometryObjects
            ):
                table_placement = table_geom_data.oMg[j]

                if objects_collide(
                    obj_a.geometry,
                    placement_a,
                    table_obj.geometry,
                    table_placement,
                ):
                    return SafetyReport(
                        safe=False,
                        samples=len(path),
                        min_robot_robot_distance_m=float("inf"),
                        min_table_distance_m=float("inf"),
                        collision_kind="robot_table",
                        sample_index=sample_index,
                        object_a=obj_a.name,
                        object_b=table_obj.name,
                    )

    return SafetyReport(
        safe=True,
        samples=len(path),
        min_robot_robot_distance_m=float("inf"),
        min_table_distance_m=float("inf"),
    )

def main():
    repo = Path(
        "/home/mines/ros2_ws/src/ur7e_tools"
    )

    robot1 = load_robot_scene(
        "robot1",
        "/tmp/robot1_live.urdf",
    )

    robot2 = load_robot_scene(
        "robot2",
        "/tmp/robot2_live.urdf",
    )

    (
        _table_model,
        _table_data,
        table_geom_model,
        table_geom_data,
    ) = load_table_scene(
        repo / "urdf/workcell.urdf"
    )

    # Reproduce the exact setup that caused
    # the unsafe Robot2 move:
    #
    # Robot1 fixed at Compound Right start.
    # Robot2 HOME -> Compound Right anchor.

    robot1_q = load_pose_yaml(
        repo
        / "config/pose_robot1_compound_right_start.yaml"
    )

    robot2_start = load_pose_yaml(
        repo
        / "config/home_robot2.yaml"
    )

    robot2_target = load_pose_yaml(
        repo
        / "config/pose_robot2_compound_right_anchor.yaml"
    )

    print()
    print("=" * 72)
    print("WORKCELL SAFETY CHECK")
    print("=" * 72)
    print(
        "Moving robot : robot2"
    )
    print(
        "Start        : home"
    )
    print(
        "Target       : compound_right_anchor"
    )
    print(
        "Other robot  : robot1 / compound_right_start"
    )
    print(
        "Grippers     : conservative OPEN geometry"
    )

    report = check_move_path(
        moving_scene=robot2,
        other_scene=robot1,
        table_geom_model=table_geom_model,
        table_geom_data=table_geom_data,
        moving_start_arm_q=robot2_start,
        moving_target_arm_q=robot2_target,
        other_arm_q=robot1_q,
        moving_gripper_position=GRIPPER_OPEN,
        other_gripper_position=GRIPPER_OPEN,
    )

    print()
    print("Samples:", report.samples)

    if np.isfinite(
        report.min_robot_robot_distance_m
    ):
        print(
            "Min robot-robot distance:",
            f"{1000.0 * report.min_robot_robot_distance_m:.2f} mm",
        )

    if np.isfinite(
        report.min_table_distance_m
    ):
        print(
            "Min robot-table distance:",
            f"{1000.0 * report.min_table_distance_m:.2f} mm",
        )

    print()

    if report.safe:
        print("RESULT: SAFE")
    else:
        print("RESULT: MOVE BLOCKED")
        print(
            "Reason:",
            report.collision_kind,
        )

        if report.sample_index is not None:
            print(
                "Collision sample:",
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


if __name__ == "__main__":
    main()
