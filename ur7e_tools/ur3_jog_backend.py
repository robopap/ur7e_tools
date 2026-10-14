"""Single-UR3 bounded Cartesian joystick process.

Protocol on stdin: ARM velocity|position, JOG AXIS +/- SPEED, STOP, DISARM.
No motion is possible until explicitly armed and a controller switch succeeds.
"""
import argparse
import select
import sys
import time

import numpy as np
import pinocchio as pin
import rclpy
from controller_manager_msgs.srv import SwitchController, ListControllers
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ur7e_tools.ur3_jog_math import axis_twist, joint_step, slew_joint_velocity, enforce_joint_limits, controller_states_match

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
TIMEOUT = 0.16
PERIOD = 0.02
JOINT_STATE_MAX_AGE = 0.75
JOINT_STATE_WAIT = 4.0
JOINT_LIMIT_MARGIN = 0.03  # rad; bounded URDF joints only


class JogNode(Node):
    def __init__(self):
        super().__init__('ur3_cartesian_joystick')
        self.q = None
        self.joint_stamp = 0.
        self.model = None
        self.data = None
        self.frame = None
        self.qidx = None
        self.qnq = None
        self.vidx = None
        self.backend = None
        self.limits_lower = np.full(6, -np.inf)
        self.limits_upper = np.full(6, np.inf)
        self.deadline = 0.
        self.active_twist = np.zeros(6)
        self.q_command = None
        self.qdot_command = np.zeros(6)
        self.previous_time = time.monotonic()
        self.publisher_velocity = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.publisher_position = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.joints_cb, 10)
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.list_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.params_client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')

    def joints_cb(self, msg):
        try:
            q = np.array([msg.position[msg.name.index(n)] for n in JOINTS], dtype=float)
            if np.isfinite(q).all():
                self.q = q
                self.joint_stamp = time.monotonic()
        except (ValueError, IndexError):
            pass

    def call(self, client, request, timeout=3.):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        future = client.call_async(request)
        limit = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < limit:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Service timeout: {client.srv_name}')
        return future.result()

    def initialize_model(self):
        req = GetParameters.Request()
        req.names = ['robot_description']
        response = self.call(self.params_client, req)
        if not response.values or not response.values[0].string_value:
            raise RuntimeError('No robot_description on robot_state_publisher')
        self.model = pin.buildModelFromXML(response.values[0].string_value)
        self.data = self.model.createData()
        self.qidx, self.qnq, self.vidx = [], [], []
        lower, upper = [], []
        for name in JOINTS:
            idx = self.model.getJointId(name)
            if idx == 0 or idx >= self.model.njoints:
                raise RuntimeError(f'Missing model joint: {name}')
            joint = self.model.joints[idx]
            # A URDF continuous revolute joint has nq=2 (cos/sin), nv=1.
            # Bounded revolute joints typically have nq=1, nv=1.
            if joint.nv != 1 or joint.nq not in (1, 2):
                raise RuntimeError(f'Unsupported model joint: {name} (nq={joint.nq}, nv={joint.nv})')
            self.qidx.append(joint.idx_q)
            self.qnq.append(joint.nq)
            self.vidx.append(joint.idx_v)
            # Pinocchio represents continuous joints with nq=2; they do not
            # have finite position limits. Bounded revolute joints have nq=1.
            lo = float(self.model.lowerPositionLimit[joint.idx_q]) if joint.nq == 1 else -np.inf
            hi = float(self.model.upperPositionLimit[joint.idx_q]) if joint.nq == 1 else np.inf
            if joint.nq == 1 and not (np.isfinite(lo) and np.isfinite(hi) and lo < hi):
                raise RuntimeError(f'Invalid URDF position limits for {name}: {lo}, {hi}')
            lower.append(lo)
            upper.append(hi)
        self.limits_lower = np.array(lower)
        self.limits_upper = np.array(upper)
        for name in ('tool0', 'flange'):
            idx = self.model.getFrameId(name)
            if idx < len(self.model.frames) and self.model.frames[idx].name == name:
                self.frame = idx
                break
        if self.frame is None:
            raise RuntimeError('UR3 tool0/flange frame missing')

    def switch(self, start, stop):
        req = SwitchController.Request()
        # ROS2 Humble uses start/stop; newer controller_manager uses activate/deactivate.
        if hasattr(req, 'activate_controllers'):
            req.activate_controllers = start
            req.deactivate_controllers = stop
            req.activate_asap = True
        else:
            req.start_controllers = start
            req.stop_controllers = stop
            req.start_asap = True
        req.strictness = SwitchController.Request.STRICT
        req.timeout.sec = 3
        response = self.call(self.switch_client, req, 5.)
        if not response.ok:
            raise RuntimeError(f'Controller switch rejected ({start} / {stop})')

    def controller_states(self):
        response = self.call(self.list_client, ListControllers.Request(), 4.)
        return {c.name: c.state for c in response.controller}

    def verify_controllers(self, expected_active, expected_inactive):
        states = self.controller_states()
        if not controller_states_match(states, expected_active, expected_inactive):
            raise RuntimeError(f'Controller states not confirmed: {states}')

    def restore_controller(self):
        """Reconcile known forward controller states after a partial/failed switch."""
        states = self.controller_states()
        forward = [f'forward_{m}_controller' for m in ('velocity', 'position')]
        active_forward = [n for n in forward if states.get(n) == 'active']
        if states.get('joint_trajectory_controller') != 'active' or active_forward:
            self.switch(['joint_trajectory_controller'], active_forward)
        self.verify_controllers(['joint_trajectory_controller'], forward)
        self.backend = None
        print('DISARMED trajectory controller restored', flush=True)

    def send(self, values):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in values]
        (self.publisher_velocity if self.backend == 'velocity' else self.publisher_position).publish(msg)

    def stop_motion(self):
        self.deadline = 0.
        self.active_twist[:] = 0.
        self.qdot_command[:] = 0.
        if self.backend == 'velocity':
            self.send(np.zeros(6))
        elif self.backend == 'position' and self.q is not None:
            self.q_command = self.q.copy()
            self.send(self.q_command)

    def disarm(self):
        # An ARM switch may have partially succeeded even if ARM raised an
        # exception. Always reconcile actual controller states on DISARM.
        self.stop_motion()
        for _ in range(6):
            self.stop_motion()
            rclpy.spin_once(self, timeout_sec=PERIOD)
        self.restore_controller()

    def arm(self, mode):
        if mode not in ('velocity', 'position') or self.backend:
            raise ValueError('Invalid or already armed backend')
        # The UI sends ARM as soon as this process prints READY. A new ROS2
        # subscriber may not have received its first JointState yet, even when
        # /joint_states is publishing normally. Wait *before* switching away
        # from the trajectory controller; never arm with an unknown position.
        end = time.monotonic() + JOINT_STATE_WAIT
        while rclpy.ok() and time.monotonic() < end:
            if self.q is not None and time.monotonic() - self.joint_stamp <= JOINT_STATE_MAX_AGE:
                break
            rclpy.spin_once(self, timeout_sec=PERIOD)
        else:
            raise RuntimeError('Fresh /joint_states required (none received within 4 seconds); trajectory controller unchanged')
        if self.model is None:
            self.initialize_model()
        if time.monotonic() - self.joint_stamp > JOINT_STATE_MAX_AGE:
            raise RuntimeError('Joint states became stale during model initialization; controller unchanged')
        self.verify_controllers(['joint_trajectory_controller'],
                                ['forward_velocity_controller', 'forward_position_controller'])
        try:
            self.switch([f'forward_{mode}_controller'], ['joint_trajectory_controller'])
            self.verify_controllers([f'forward_{mode}_controller'],
                                    ['joint_trajectory_controller',
                                     f'forward_{"position" if mode == "velocity" else "velocity"}_controller'])
        except Exception:
            # Do not report ARMED after a partial switch. Let the UI initiate
            # DISARM and retain the error if recovery is not possible.
            try:
                self.restore_controller()
            except Exception as recovery_exc:
                print(f'ERROR controller recovery: {recovery_exc}', flush=True)
            raise
        self.backend = mode
        self.q_command = self.q.copy()
        self.stop_motion()
        print(f'ARMED {mode}', flush=True)

    def step(self):
        now = time.monotonic()
        dt = min(max(now - self.previous_time, 0.), 0.04)
        self.previous_time = now
        if not self.backend:
            return
        if self.q is None or now - self.joint_stamp > JOINT_STATE_MAX_AGE:
            print('STOP stale joint states', flush=True)
            self.stop_motion()
            return
        if now > self.deadline:
            self.stop_motion()
            return
        q_model = pin.neutral(self.model)
        for angle, q_index, nq in zip(self.q, self.qidx, self.qnq):
            if nq == 1:
                q_model[q_index] = angle
            else:
                # Pinocchio JointModelRUBZ expects [cos(theta), sin(theta)].
                q_model[q_index] = np.cos(angle)
                q_model[q_index + 1] = np.sin(angle)
        pin.computeJointJacobians(self.model, self.data, q_model)
        pin.updateFramePlacements(self.model, self.data)
        j = pin.getFrameJacobian(self.model, self.data, self.frame, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, self.vidx]
        qdot_target = joint_step(j, self.active_twist)
        self.qdot_command = slew_joint_velocity(self.qdot_command, qdot_target, dt)
        qdot = enforce_joint_limits(self.q, self.qdot_command,
                                    self.limits_lower, self.limits_upper,
                                    JOINT_LIMIT_MARGIN)
        self.qdot_command = qdot
        if self.backend == 'velocity':
            self.send(qdot)
        else:
            # Bound accumulated position setpoints relative to measured state.
            self.q_command = self.q_command + qdot * dt
            self.q_command = np.clip(self.q_command, self.q - 0.025, self.q + 0.025)
            self.q_command = np.clip(self.q_command,
                                     self.limits_lower + JOINT_LIMIT_MARGIN,
                                     self.limits_upper - JOINT_LIMIT_MARGIN)
            self.send(self.q_command)

    def command(self, line):
        parts = line.strip().split()
        if not parts:
            return
        if parts[0] == 'ARM' and len(parts) == 2:
            self.arm(parts[1])
        elif parts[0] == 'JOG' and len(parts) == 4 and self.backend:
            axis, sign, speed = parts[1], int(parts[2]), float(parts[3])
            self.active_twist = axis_twist(axis, sign, speed / 1000., speed * 0.003)
            self.deadline = time.monotonic() + TIMEOUT
        elif parts[0] == 'STOP':
            self.stop_motion()
        elif parts[0] == 'DISARM':
            self.disarm()
        else:
            raise ValueError('Unknown command or not armed')


def main():
    rclpy.init()
    node = JogNode()
    print('READY — motion disabled until ARM', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=PERIOD)
            readable, _, _ = select.select([sys.stdin], [], [], 0)
            if readable:
                line = sys.stdin.readline()
                if not line:
                    break
                try:
                    node.command(line)
                except Exception as exc:
                    node.stop_motion()
                    print(f'ERROR {exc}', flush=True)
            try:
                node.step()
            except Exception as exc:
                node.stop_motion()
                print(f'ERROR jog stopped: {exc}', flush=True)
    finally:
        try:
            node.disarm()
        except Exception as exc:
            print(f'ERROR controller restoration: {exc}', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
