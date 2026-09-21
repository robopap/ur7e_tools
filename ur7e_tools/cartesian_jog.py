#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import tempfile
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
import yaml

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

from ur7e_tools.cartesian_jog_core import (
    translated_target_pose,
    validate_axis_delta,
    validate_pose_name,
)
from ur7e_tools.workcell_safety import ARM_JOINTS
from ur7e_tools.workcell_safety_runtime import LiveWorkcellSafety, print_safety_report

ROBOTS = ("robot1", "robot2")
IK_MAX_ITERS = 400
IK_DAMPING = 1e-6
IK_STEP = 0.25
MAX_FINAL_POS_ERR_M = 5e-4
MAX_FINAL_ROT_ERR_RAD = math.radians(0.10)
MAX_JOINT_DELTA_RAD = 0.45
POST_MOVE_JOINT_TOL_RAD = 0.03


def duration_msg(seconds):
    seconds = float(seconds)
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--robot", choices=ROBOTS, required=True)
    p.add_argument("--dx", type=float, default=0.0)
    p.add_argument("--dy", type=float, default=0.0)
    p.add_argument("--dz", type=float, default=0.0)
    p.add_argument("--duration", type=float, default=1.0)
    p.add_argument("--save-as", default=None)
    p.add_argument("--overwrite", action="store_true")
    p.add_argument("--yes", action="store_true")
    return p.parse_args()


def resolve_arm_indices(model, robot):
    q_idx, v_idx = [], []
    names = set(model.names)
    for base in ARM_JOINTS:
        name = next((n for n in (f"{robot}_{base}", base) if n in names), None)
        if name is None:
            raise RuntimeError(f"{robot}: missing arm joint {base}")
        joint = model.joints[model.getJointId(name)]
        if joint.nq != 1 or joint.nv != 1:
            raise RuntimeError(f"{name}: expected 1-DoF joint")
        q_idx.append(joint.idx_q)
        v_idx.append(joint.idx_v)
    return np.asarray(q_idx), np.asarray(v_idx)


def resolve_tool_frame(model, robot):
    for name in (f"{robot}_tool0", "tool0"):
        fid = model.getFrameId(name)
        if fid < len(model.frames) and model.frames[fid].name == name:
            return fid, name
    raise RuntimeError(f"{robot}: tool0 frame not found")


def model_q(model, q_indices, arm_q):
    q = pin.neutral(model)
    q[q_indices] = np.asarray(arm_q, dtype=float)
    return q


def fk(model, data, q, frame_id):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return data.oMf[frame_id].copy()


def solve_ik(model, data, frame_id, q_seed, v_indices, target):
    q = q_seed.copy()
    for _ in range(IK_MAX_ITERS):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        current = data.oMf[frame_id]
        iMd = current.actInv(target)
        err = pin.log6(iMd).vector
        if float(np.linalg.norm(err)) < 1e-8:
            return q, True
        J = pin.computeFrameJacobian(
            model, data, q, frame_id, pin.ReferenceFrame.LOCAL
        )
        J = -pin.Jlog6(iMd.inverse()) @ J
        Jarm = J[:, v_indices]
        dq = -Jarm.T @ np.linalg.solve(
            Jarm @ Jarm.T + IK_DAMPING * np.eye(6), err
        )
        v = np.zeros(model.nv)
        v[v_indices] = IK_STEP * dq
        q = pin.integrate(model, q, v)
        if not np.all(np.isfinite(q)):
            break
    return q, False


def load_live_model(safety, robot):
    xml = safety.get_robot_description(robot)
    with tempfile.TemporaryDirectory(prefix="cartesian_jog_") as td:
        path = Path(td) / f"{robot}.urdf"
        path.write_text(xml, encoding="utf-8")
        return pin.buildModelFromUrdf(str(path))


def command_joint_names(safety, robot):
    safety.get_arm_q(robot)
    msg = safety.arm_states.get(robot)
    available = set(msg.name)
    result = []
    for base in ARM_JOINTS:
        name = next((n for n in (f"{robot}_{base}", base) if n in available), None)
        if name is None:
            raise RuntimeError(f"{robot}: missing command joint {base}")
        result.append(name)
    return result


def save_pose(repo, safety, robot, name, overwrite):
    name = validate_pose_name(name)
    q = safety.get_arm_q(robot)
    path = repo / "config" / f"pose_{robot}_{name}.yaml"
    if path.exists() and not overwrite:
        raise RuntimeError(f"Pose already exists: {path}")
    path.write_text(
        yaml.safe_dump(
            {
                "joint_names": list(ARM_JOINTS),
                "positions": [float(v) for v in q],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    print(f"POSE_SAVED={path}")


def send_move(node, safety, robot, q0, q1, duration_s):
    if duration_s < 0.25 or not math.isfinite(duration_s):
        raise RuntimeError("duration must be >= 0.25 s")

    action = f"/{robot}/joint_trajectory_controller/follow_joint_trajectory"
    client = ActionClient(node, FollowJointTrajectory, action)
    if not client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError(f"Action unavailable: {action}")

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = command_joint_names(safety, robot)

    p0 = JointTrajectoryPoint()
    p0.positions = [float(v) for v in q0]
    p0.time_from_start = duration_msg(0.05)
    goal.trajectory.points.append(p0)

    p1 = JointTrajectoryPoint()
    p1.positions = [float(v) for v in q1]
    p1.time_from_start = duration_msg(0.05 + duration_s)
    goal.trajectory.points.append(p1)

    sf = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sf, timeout_sec=5.0)
    gh = sf.result()
    if gh is None or not gh.accepted:
        raise RuntimeError(f"{robot}: jog goal rejected")

    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=duration_s + 8.0)
    wrapped = rf.result()
    if wrapped is None:
        gh.cancel_goal_async()
        raise RuntimeError(f"{robot}: jog timeout")

    if (
        int(wrapped.status) != int(GoalStatus.STATUS_SUCCEEDED)
        or int(wrapped.result.error_code) != 0
    ):
        raise RuntimeError(
            f"{robot}: jog failed "
            f"(status={wrapped.status}, code={wrapped.result.error_code})"
        )


def execute_jog(node, safety, robot, delta_base, duration_s, assume_yes):
    q_start = safety.get_arm_q(robot)
    model = load_live_model(safety, robot)
    data = model.createData()
    q_idx, v_idx = resolve_arm_indices(model, robot)
    frame_id, frame_name = resolve_tool_frame(model, robot)

    q_seed = model_q(model, q_idx, q_start)
    current = fk(model, data, q_seed, frame_id)

    R_target, p_target = translated_target_pose(
        np.asarray(current.rotation),
        np.asarray(current.translation),
        delta_base,
    )
    target = pin.SE3(R_target, p_target)

    q_sol, ok = solve_ik(model, data, frame_id, q_seed, v_idx, target)
    if not ok:
        raise RuntimeError(f"{robot}: IK failed")

    q_target = np.asarray(q_sol)[q_idx]
    lo = np.asarray(model.lowerPositionLimit)[q_idx]
    hi = np.asarray(model.upperPositionLimit)[q_idx]
    if np.any(q_target < lo) or np.any(q_target > hi):
        raise RuntimeError(f"{robot}: IK target violates joint limits")

    max_dq = float(np.max(np.abs(q_target - q_start)))
    if max_dq > MAX_JOINT_DELTA_RAD:
        raise RuntimeError(
            f"{robot}: jog needs {max_dq:.3f} rad joint change; blocked"
        )

    actual = fk(model, data, q_sol, frame_id)
    pos_err = float(np.linalg.norm(np.asarray(actual.translation) - p_target))
    rot_err = float(np.linalg.norm(
        pin.log3(R_target.T @ np.asarray(actual.rotation))
    ))
    if pos_err > MAX_FINAL_POS_ERR_M or rot_err > MAX_FINAL_ROT_ERR_RAD:
        raise RuntimeError(
            f"{robot}: IK residual too large "
            f"({1000*pos_err:.3f} mm, {math.degrees(rot_err):.4f} deg)"
        )

    print(f"ROBOT={robot}")
    print("FRAME=robot_base")
    print(f"TCP_FRAME={frame_name}")
    print("DELTA_MM=", [round(1000*float(v), 3) for v in delta_base])
    print("TCP_ORIENTATION=preserved")

    report = safety.check_path(
        moving_robot=robot,
        moving_start_q=q_start,
        moving_target_q=q_target,
    )
    print_safety_report(report)
    if not report.safe:
        print("MOVE BLOCKED")
        raise RuntimeError("MOVE BLOCKED by workcell safety")

    if not assume_yes:
        expected = f"JOG {robot.upper()}"
        if input(f"Type {expected}: ").strip() != expected:
            raise RuntimeError("Jog cancelled")

    send_move(node, safety, robot, q_start, q_target, duration_s)

    safety.arm_states[robot] = None
    q_final = safety.get_arm_q(robot)
    err = float(np.max(np.abs(q_final - q_target)))
    if err > POST_MOVE_JOINT_TOL_RAD:
        raise RuntimeError(
            f"{robot}: post-move joint error {err:.4f} rad"
        )
    print("CARTESIAN JOG COMPLETE")


def main():
    args = parse_args()
    repo = Path(__file__).resolve().parents[1]
    delta = np.asarray([args.dx, args.dy, args.dz], dtype=float)

    if args.save_as is not None:
        if np.any(np.abs(delta) > 1e-12):
            raise RuntimeError("--save-as cannot be combined with dx/dy/dz")
    else:
        delta = validate_axis_delta(args.dx, args.dy, args.dz)

    rclpy.init()
    node = Node(f"{args.robot}_cartesian_jog")
    safety = LiveWorkcellSafety(node)
    try:
        if args.save_as is not None:
            save_pose(
                repo, safety, args.robot, args.save_as, bool(args.overwrite)
            )
        else:
            execute_jog(
                node, safety, args.robot, delta,
                float(args.duration), bool(args.yes)
            )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
