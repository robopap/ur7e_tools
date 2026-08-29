#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


OPEN_POSITION = 0.0305
CLOSED_POSITION = 0.0115


class GripperVisualizer(Node):

    def __init__(self):
        super().__init__("gripper_visualizer")

        self.declare_parameter(
            "gripper_joint_name",
            "gripper_gripper_joint",
        )

        self.gripper_joint_name = (
            self.get_parameter("gripper_joint_name")
            .get_parameter_value()
            .string_value
        )

        self.gripper_position = OPEN_POSITION
        self.latest_robot_state = None

        # IMPORTANT:
        # We READ the controller joint_states.
        # We NEVER write back to joint_states.
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            10,
        )

        # Combined state used only for visualization.
        self.publisher = self.create_publisher(
            JointState,
            "visual_joint_states",
            10,
        )

        self.create_service(
            Trigger,
            "gripper_visual/open",
            self.open_callback,
        )

        self.create_service(
            Trigger,
            "gripper_visual/close",
            self.close_callback,
        )

        self.get_logger().info(
            f"2FG7 simulation visualizer ready: "
            f"{self.gripper_joint_name}"
        )

    def joint_state_callback(self, msg):
        self.latest_robot_state = msg
        self.publish_combined_state()

    def publish_combined_state(self):

        if self.latest_robot_state is None:
            return

        original = self.latest_robot_state

        msg = JointState()

        # Fresh timestamp for robot_state_publisher
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = original.header.frame_id

        original_names = list(original.name)

        msg.name = list(original.name)
        msg.position = list(original.position)

        # Preserve velocity/effort only if structurally valid.
        if len(original.velocity) == len(original_names):
            msg.velocity = list(original.velocity)

        if len(original.effort) == len(original_names):
            msg.effort = list(original.effort)

        # Normally the gripper joint is NOT in the UR controller state.
        # Handle both cases safely anyway.
        if self.gripper_joint_name in msg.name:

            index = msg.name.index(self.gripper_joint_name)

            if index < len(msg.position):
                msg.position[index] = self.gripper_position

            if len(msg.velocity) == len(msg.name):
                msg.velocity[index] = 0.0

            if len(msg.effort) == len(msg.name):
                msg.effort[index] = 0.0

        else:

            msg.name.append(self.gripper_joint_name)
            msg.position.append(self.gripper_position)

            if len(msg.velocity) == len(original_names):
                msg.velocity.append(0.0)

            if len(msg.effort) == len(original_names):
                msg.effort.append(0.0)

        self.publisher.publish(msg)

    def open_callback(self, request, response):

        self.gripper_position = OPEN_POSITION

        # Update RViz immediately.
        self.publish_combined_state()

        response.success = True
        response.message = "2FG7 opened"

        return response

    def close_callback(self, request, response):

        self.gripper_position = CLOSED_POSITION

        # Update RViz immediately.
        self.publish_combined_state()

        response.success = True
        response.message = "2FG7 closed"

        return response


def main(args=None):

    rclpy.init(args=args)

    node = GripperVisualizer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()