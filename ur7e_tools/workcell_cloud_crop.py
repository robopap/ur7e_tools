#!/usr/bin/env python3

import argparse

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener


X_MIN, X_MAX = -0.8, 0.8
Y_MIN, Y_MAX = -0.6, 0.8
Z_MIN, Z_MAX = 0.6, 2.0


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


class WorkcellCloudCrop(Node):
    def __init__(self, robot):
        super().__init__(f"workcell_cloud_crop_{robot}")

        if robot not in ("robot1", "robot2"):
            raise ValueError(f"Unsupported robot namespace: {robot}")

        self.robot = robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            f"/{robot}/camera/workcell_points",
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            f"/{robot}/camera/depth/color/points",
            self.cloud_callback,
            input_qos,
        )

        self.get_logger().info(
            f"{robot} world-crop active: "
            f"X [{X_MIN}, {X_MAX}] "
            f"Y [{Y_MIN}, {Y_MAX}] "
            f"Z [{Z_MIN}, {Z_MAX}]"
        )

    def cloud_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                "world",
                msg.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.01),
            )
        except Exception:
            return

        data = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z", "rgb"),
            skip_nans=True,
        )
        if data.size == 0:
            return

        xyz = data[:, :3].astype(np.float64, copy=False)
        translation_msg = transform.transform.translation
        rotation_msg = transform.transform.rotation
        rotation = quaternion_to_matrix(
            rotation_msg.x,
            rotation_msg.y,
            rotation_msg.z,
            rotation_msg.w,
        )
        translation = np.array(
            [translation_msg.x, translation_msg.y, translation_msg.z],
            dtype=np.float64,
        )
        xyz_world = xyz @ rotation.T + translation

        mask = (
            (xyz_world[:, 0] >= X_MIN)
            & (xyz_world[:, 0] <= X_MAX)
            & (xyz_world[:, 1] >= Y_MIN)
            & (xyz_world[:, 1] <= Y_MAX)
            & (xyz_world[:, 2] >= Z_MIN)
            & (xyz_world[:, 2] <= Z_MAX)
        )

        if not np.any(mask):
            return

        xyz_keep = xyz_world[mask]
        rgb_keep = data[mask, 3]

        dtype = point_cloud2.dtype_from_fields(
            msg.fields,
            msg.point_step,
        )
        points = np.zeros(len(xyz_keep), dtype=dtype)
        points["x"] = xyz_keep[:, 0].astype(np.float32)
        points["y"] = xyz_keep[:, 1].astype(np.float32)
        points["z"] = xyz_keep[:, 2].astype(np.float32)
        points["rgb"] = rgb_keep.astype(np.float32)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = "world"

        output = point_cloud2.create_cloud(
            header,
            msg.fields,
            points,
        )
        self.publisher.publish(output)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Crop a wrist RealSense point cloud in the fixed world frame."
    )
    parser.add_argument(
        "--robot",
        required=True,
        choices=("robot1", "robot2"),
        help="Robot/camera namespace to process.",
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = WorkcellCloudCrop(args.robot)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
