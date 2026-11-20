# SPDX-License-Identifier: Apache-2.0
"""Merge complementary LaserScan feeds in one common frame."""

from __future__ import annotations

import copy
import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer


def _stamp_seconds(scan: LaserScan) -> float:
    return float(scan.header.stamp.sec) + float(scan.header.stamp.nanosec) * 1.0e-9


def _angle_mask(
    angles: np.ndarray,
    minimum: float,
    maximum: float,
) -> np.ndarray:
    """Build an inclusive angular mask, supporting intervals across +/-pi."""

    if minimum <= maximum:
        return (angles >= minimum) & (angles <= maximum)
    return (angles >= minimum) | (angles <= maximum)


def merge_ranges(
    first: LaserScan,
    second: LaserScan,
    *,
    first_angle_min: float = -math.pi,
    first_angle_max: float = math.pi,
    second_angle_min: float = -math.pi,
    second_angle_max: float = math.pi,
) -> list[float]:
    """Return the nearest valid range in every common angular bin."""

    if len(first.ranges) != len(second.ranges):
        raise ValueError("LaserScan inputs have different range counts")
    for field in ("angle_min", "angle_max", "angle_increment"):
        if not np.isclose(getattr(first, field), getattr(second, field), atol=1.0e-6):
            raise ValueError(f"LaserScan inputs have different {field}")
    if first.header.frame_id != second.header.frame_id:
        raise ValueError("LaserScan inputs must already use the same target frame")

    a = np.asarray(first.ranges, dtype=np.float32)
    b = np.asarray(second.ranges, dtype=np.float32)
    angles = first.angle_min + np.arange(a.size) * first.angle_increment
    a_valid = np.isfinite(a) & _angle_mask(angles, first_angle_min, first_angle_max)
    b_valid = np.isfinite(b) & _angle_mask(angles, second_angle_min, second_angle_max)
    merged = np.full(a.shape, np.inf, dtype=np.float32)
    merged[a_valid & ~b_valid] = a[a_valid & ~b_valid]
    merged[b_valid & ~a_valid] = b[b_valid & ~a_valid]
    both = a_valid & b_valid
    merged[both] = np.minimum(a[both], b[both])
    return merged.tolist()


class LaserScanFusionNode(Node):
    """Combine front/rear scans without copying their point clouds."""

    def __init__(self) -> None:
        super().__init__("laser_scan_fusion_node")
        self.declare_parameter("input_topics", ["scan_front", "scan_rear"])
        self.declare_parameter("output_topic", "scan")
        self.declare_parameter("max_stamp_delta", 0.25)
        self.declare_parameter("reliable_input", False)
        self.declare_parameter("synchronize_inputs", False)
        self.declare_parameter("sync_tolerance_sec", 0.03)
        self.declare_parameter("first_angle_min", -math.pi)
        self.declare_parameter("first_angle_max", math.pi)
        self.declare_parameter("second_angle_min", -math.pi)
        self.declare_parameter("second_angle_max", math.pi)

        topics = [str(topic) for topic in self.get_parameter("input_topics").value]
        if len(topics) != 2:
            raise ValueError("laser_scan_fusion_node requires exactly two input_topics")
        self._max_stamp_delta = float(self.get_parameter("max_stamp_delta").value)
        self._angle_limits = {
            name: float(self.get_parameter(name).value)
            for name in (
                "first_angle_min",
                "first_angle_max",
                "second_angle_min",
                "second_angle_max",
            )
        }
        self._latest: list[LaserScan | None] = [None, None]
        input_qos = (
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            if self.get_parameter("reliable_input").value
            else qos_profile_sensor_data
        )
        self._publisher = self.create_publisher(
            LaserScan,
            str(self.get_parameter("output_topic").value),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        if self.get_parameter("synchronize_inputs").value:
            self._scan_subscribers = [
                Subscriber(self, LaserScan, topic, qos_profile=input_qos) for topic in topics
            ]
            self._synchronizer = ApproximateTimeSynchronizer(
                self._scan_subscribers, 20, float(self.get_parameter("sync_tolerance_sec").value)
            )
            self._synchronizer.registerCallback(self._publish_pair)
        else:
            self._scan_subscribers = [
                self.create_subscription(
                    LaserScan,
                    topic,
                    lambda message, index=index: self._on_scan(index, message),
                    input_qos,
                )
                for index, topic in enumerate(topics)
            ]
        self.get_logger().info(
            f"laser_scan_fusion_node merging {topics} into "
            f"{self.get_parameter('output_topic').value}"
        )

    def _on_scan(self, index: int, scan: LaserScan) -> None:
        self._latest[index] = scan
        first, second = self._latest
        if first is None or second is None:
            return
        self._publish_pair(first, second)

    def _publish_pair(self, first: LaserScan, second: LaserScan) -> None:
        if abs(_stamp_seconds(first) - _stamp_seconds(second)) > self._max_stamp_delta:
            return
        try:
            ranges = merge_ranges(first, second, **self._angle_limits)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=5.0)
            return

        newest = first if _stamp_seconds(first) >= _stamp_seconds(second) else second
        merged = copy.deepcopy(newest)
        merged.ranges = ranges
        merged.intensities = []
        self._publisher.publish(merged)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaserScanFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
