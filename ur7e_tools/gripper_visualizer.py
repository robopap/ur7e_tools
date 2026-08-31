#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


CLOSED_POSITION = 0.0115
OPEN_POSITION = 0.0305


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

        self.normalized_position = 1.0
        self.latest_robot_state = None

        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            10,
        )

        self.publisher = self.create_publisher(
            JointState,
            "visual_joint_states",
            10,
        )

        self.position_subscription = self.create_subscription(
            Float64,
            "gripper_visual/position",
            self.position_callback,
            10,
        )

        # Keep OPEN/CLOSE services for compatibility with older UI versions.
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
            f"2FG7 visualizer ready: {self.gripper_joint_name}"
        )

    def normalized_to_joint_position(self):
        return (
            CLOSED_POSITION
            + self.normalized_position
            * (OPEN_POSITION - CLOSED_POSITION)
        )

    def joint_state_callback(self, msg):
        self.latest_robot_state = msg
        self.publish_combined_state()

    def position_callback(self, msg):
        self.normalized_position = max(
            0.0,
            min(1.0, float(msg.data)),
        )
        self.publish_combined_state()

    def publish_combined_state(self):

        if self.latest_robot_state is None:
            return

        original = self.latest_robot_state
        original_names = list(original.name)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = original.header.frame_id

        msg.name = list(original.name)
        msg.position = list(original.position)

        if len(original.velocity) == len(original_names):
            msg.velocity = list(original.velocity)

        if len(original.effort) == len(original_names):
            msg.effort = list(original.effort)

        gripper_position = self.normalized_to_joint_position()

        if self.gripper_joint_name in msg.name:
            index = msg.name.index(self.gripper_joint_name)

            if index < len(msg.position):
                msg.position[index] = gripper_position

            if len(msg.velocity) == len(msg.name):
                msg.velocity[index] = 0.0

            if len(msg.effort) == len(msg.name):
                msg.effort[index] = 0.0

        else:
            msg.name.append(self.gripper_joint_name)
            msg.position.append(gripper_position)

            if len(msg.velocity) == len(original_names):
                msg.velocity.append(0.0)

            if len(msg.effort) == len(original_names):
                msg.effort.append(0.0)

        self.publisher.publish(msg)

    def open_callback(self, request, response):
        self.normalized_position = 1.0
        self.publish_combined_state()

        response.success = True
        response.message = "2FG7 visualization opened"
        return response

    def close_callback(self, request, response):
        self.normalized_position = 0.0
        self.publish_combined_state()

        response.success = True
        response.message = "2FG7 visualization closed"
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
