from __future__ import annotations

import math
import unittest

from sensor_msgs.msg import LaserScan

from isaac_composer.sensors.laser_scan_fusion_node import merge_ranges


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


if __name__ == "__main__":
    unittest.main()
