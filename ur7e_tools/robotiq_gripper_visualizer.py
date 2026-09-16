#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


# Robotiq 2F-140 main joint:
#   0.0 rad   ~= fully open
#   0.695 rad ~= fully closed
#
# UI convention kept identical to the existing 2FG7 controls:
#   normalized 0.0 = CLOSED
#   normalized 1.0 = OPEN
OPEN_POSITION = 0.0
CLOSED_POSITION = 0.695


class RobotiqGripperVisualizer(Node):

    def __init__(self):
        super().__init__("robotiq_gripper_visualizer")

        self.declare_parameter(
            "gripper_joint_name",
            "gripper_finger_joint",
        )

        self.gripper_joint_name = (
            self.get_parameter("gripper_joint_name")
            .get_parameter_value()
            .string_value
        )

        # Start visually open.
        self.normalized_position = 1.0
        self.latest_robot_state = None

        # Raw UR joint states from joint_state_broadcaster.
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_state_callback,
            10,
        )

        # Combined UR + Robotiq joint-state stream consumed by
        # robot_state_publisher in Single UR3 / Single UR7e.
        self.publisher = self.create_publisher(
            JointState,
            "visual_joint_states",
            10,
        )

        self.position_subscription = self.create_subscription(
            Float64,
            "robotiq_visual/position",
            self.position_callback,
            10,
        )

        self.create_service(
            Trigger,
            "robotiq_visual/open",
            self.open_callback,
        )

        self.create_service(
            Trigger,
            "robotiq_visual/close",
            self.close_callback,
        )

        self.get_logger().info(
            f"Robotiq 2F-140 visualizer ready: {self.gripper_joint_name}"
        )

    def normalized_to_joint_position(self):
        # normalized 0 -> closed, normalized 1 -> open
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

        # Important: publish with a current timestamp.
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
        response.message = "Robotiq 2F-140 visualization opened"
        return response

    def close_callback(self, request, response):
        self.normalized_position = 0.0
        self.publish_combined_state()

        response.success = True
        response.message = "Robotiq 2F-140 visualization closed"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotiqGripperVisualizer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # ROS 2 Humble can raise this from the executor when SIGINT arrives
        # while a subscription message is being taken. It is a shutdown race,
        # not a runtime failure of the visualizer.
        if "Unable to convert call argument to Python object" not in str(exc):
            raise
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except (KeyboardInterrupt, RuntimeError):
                pass


if __name__ == "__main__":
    main()
