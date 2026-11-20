from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

from sensor_msgs.msg import LaserScan

from isaac_composer.sensors.laser_scan_fusion_node import LaserScanFusionNode, merge_ranges


def _scan(ranges):
    scan = LaserScan()
    scan.header.frame_id = "base_link"
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = math.pi / 2.0
    scan.ranges = ranges
    return scan


class LaserScanFusionTests(unittest.TestCase):
    def test_complementary_occluded_scans_are_combined(self):
        front = _scan([1.0, math.inf, 3.0, math.inf])
        rear = _scan([math.inf, 2.0, 4.0, 5.0])
        self.assertEqual(merge_ranges(front, rear), [1.0, 2.0, 3.0, 5.0])

    def test_nearest_obstacle_wins_when_both_sensors_see_it(self):
        front = _scan([2.0])
        rear = _scan([1.5])
        self.assertEqual(merge_ranges(front, rear), [1.5])

    def test_front_and_rear_are_limited_to_their_unoccluded_half_planes(self):
        front = _scan([0.2, 1.0, 1.0, 1.0])
        rear = _scan([3.0, 1.0, 0.2, 4.0])
        merged = merge_ranges(
            front,
            rear,
            first_angle_min=-math.pi / 2.0,
            first_angle_max=math.pi / 2.0,
            second_angle_min=math.pi / 2.0,
            second_angle_max=-math.pi / 2.0,
        )
        self.assertEqual(merged, [3.0, 1.0, 1.0, 1.0])


def test_synchronized_fusion_waits_for_matching_half_without_replacing_ros_subscriptions(monkeypatch):
    from rclpy.node import Node
    from rclpy.qos import ReliabilityPolicy

    subscriptions, outputs, settings = [], [], {}

    def init(node, name):
        node._subscriptions = subscriptions

    def declare(node, name, default):
        settings[name] = {"reliable_input": True, "synchronize_inputs": True}.get(name, default)

    def subscribe(node, msg_type, topic, callback, qos_profile):
        sub = SimpleNamespace(callback=callback, qos_profile=qos_profile)
        subscriptions.append(sub)
        return sub

    monkeypatch.setattr(Node, "__init__", init)
    monkeypatch.setattr(Node, "declare_parameter", declare)
    monkeypatch.setattr(Node, "get_parameter", lambda node, name: SimpleNamespace(value=settings[name]))
    monkeypatch.setattr(Node, "create_subscription", subscribe)
    monkeypatch.setattr(Node, "create_publisher", lambda *a, **k: SimpleNamespace(publish=outputs.append))
    monkeypatch.setattr(Node, "get_logger", lambda node: SimpleNamespace(info=lambda text: None))
    node = LaserScanFusionNode()
    # rclpy owns this list; storing message_filters wrappers here breaks spin().
    assert node._subscriptions is subscriptions and len(subscriptions) == 2
    assert all(sub.qos_profile.reliability == ReliabilityPolicy.RELIABLE for sub in subscriptions)

    def send(index, stamp_ns):
        scan = _scan([1.0, 2.0])
        scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        subscriptions[index].callback(scan)

    send(0, 1_000_000_000)
    assert not outputs
    send(1, 1_000_000_000)
    assert len(outputs) == 1
    send(0, 1_100_000_000)
    # A new front scan must not reuse the previous rear scan.
    assert len(outputs) == 1
    send(1, 1_100_000_000)
    assert len(outputs) == 2
    assert outputs[-1].header.stamp.nanosec == 100_000_000
    send(0, 1_200_000_000)
    send(1, 1_300_000_000)
    assert len(outputs) == 2


if __name__ == "__main__":
    unittest.main()
