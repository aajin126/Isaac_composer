# SPDX-License-Identifier: Apache-2.0
"""Fuse two PointCloud2 LiDAR feeds into one cloud in a shared target frame.

Mirrors the Unitree A2 firmware behavior where the front/rear Hesai JT128
point clouds (``points1``/``points2``) are combined into a single fusion
cloud (``points``) expressed in the front LiDAR frame. The front/rear mount
offset is fixed, so the TF lookup always resolves to the same transform.
"""

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class LidarFusionNode(Node):
    """Synchronize, transform and concatenate two LiDAR point clouds."""

    def __init__(self) -> None:
        super().__init__("lidar_fusion_node")
        self.declare_parameter("input_topics", ["points1", "points2"])
        self.declare_parameter("output_topic", "points")
        self.declare_parameter("target_frame", "front_lidar_link")
        self.declare_parameter("sync_slop", 0.1)
        self.declare_parameter("queue_size", 5)

        input_topics = [str(topic) for topic in self.get_parameter("input_topics").value]
        if len(input_topics) != 2:
            raise ValueError("lidar_fusion_node requires exactly two input_topics")
        self._target_frame = str(self.get_parameter("target_frame").value)
        queue_size = int(self.get_parameter("queue_size").value)
        slop = float(self.get_parameter("sync_slop").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        subscribers = [
            Subscriber(self, PointCloud2, topic, qos_profile=qos_profile_sensor_data)
            for topic in input_topics
        ]
        self._sync = ApproximateTimeSynchronizer(subscribers, queue_size, slop)
        self._sync.registerCallback(self._on_clouds)
        self.get_logger().info(
            f"lidar_fusion_node merging {input_topics} into {output_topic} "
            f"(frame={self._target_frame})"
        )

    def _transform_to_target(self, cloud: PointCloud2) -> PointCloud2 | None:
        if cloud.header.frame_id == self._target_frame:
            return cloud
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame, cloud.header.frame_id, rclpy.time.Time()
            )
        except Exception as exc:  # tf2 raises several distinct exception types
            self.get_logger().warn(
                f"lidar_fusion_node: no transform {cloud.header.frame_id}->"
                f"{self._target_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        return do_transform_cloud(cloud, transform)

    def _on_clouds(self, cloud_a: PointCloud2, cloud_b: PointCloud2) -> None:
        transformed_a = self._transform_to_target(cloud_a)
        transformed_b = self._transform_to_target(cloud_b)
        if transformed_a is None or transformed_b is None:
            return

        names_a = [field.name for field in transformed_a.fields]
        names_b = [field.name for field in transformed_b.fields]
        if names_a != names_b:
            self.get_logger().error(
                "lidar_fusion_node: mismatched point fields between inputs, cannot merge",
                throttle_duration_sec=5.0,
            )
            return

        points_a = point_cloud2.read_points(transformed_a, skip_nans=False)
        points_b = point_cloud2.read_points(transformed_b, skip_nans=False)
        merged_points = np.concatenate([points_a, points_b])
        merged = point_cloud2.create_cloud(transformed_a.header, transformed_a.fields, merged_points)
        merged.header.frame_id = self._target_frame
        merged.header.stamp = cloud_a.header.stamp
        self._publisher.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
