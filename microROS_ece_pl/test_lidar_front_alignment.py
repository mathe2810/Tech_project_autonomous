#!/usr/bin/env python3
"""Test LiDAR front alignment.

Usage:
1) Place a flat obstacle (carton/mur) at ~25-40 cm in front of the robot body.
2) Keep robot static.
3) Run this script and wait 5-10 seconds.
4) Send me the printed "recommended_offset_deg" value.

The script estimates at which LiDAR angle the front obstacle appears.
If front is not at 0 deg, it prints the angle offset to apply.
"""

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarFrontAlignmentTest(Node):
    def __init__(self) -> None:
        super().__init__('lidar_front_alignment_test')

        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('min_range_m', 0.08)
        self.declare_parameter('max_range_m', 1.50)
        self.declare_parameter('report_hz', 2.0)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        report_hz = float(self.get_parameter('report_hz').value)

        self.last_scan: Optional[LaserScan] = None
        self.sample_count = 0

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(1.0 / max(0.5, report_hz), self.report)

        self.get_logger().info('=== LiDAR Front Alignment Test ===')
        self.get_logger().info(f'Subscribed to: {self.scan_topic}')
        self.get_logger().info('Place obstacle at 25-40 cm straight in FRONT of robot, then wait...')

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.sample_count += 1

    def report(self) -> None:
        if self.last_scan is None:
            self.get_logger().info('Waiting for scan...')
            return

        angle_deg, dist_m = self.find_front_obstacle_angle(self.last_scan)
        if angle_deg is None:
            self.get_logger().info('No valid close obstacle detected in selected range.')
            return

        recommended_offset_deg = -angle_deg

        self.get_logger().info(
            f'[RESULT] front_obstacle_angle_deg={angle_deg:+.1f} dist={dist_m:.2f} m | '
            f'recommended_offset_deg={recommended_offset_deg:+.1f}'
        )
        self.get_logger().info(
            'Interpretation: if this obstacle is physically in front of robot body, '
            'apply recommended_offset_deg in navigation node.'
        )

    def find_front_obstacle_angle(self, scan: LaserScan) -> Tuple[Optional[float], Optional[float]]:
        pairs: List[Tuple[float, float]] = []
        for i, r in enumerate(scan.ranges):
            if r is None or math.isnan(r) or math.isinf(r):
                continue
            dist = float(r)
            if dist < self.min_range_m or dist > self.max_range_m:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            pairs.append((angle, dist))

        if not pairs:
            return None, None

        pairs.sort(key=lambda x: x[1])
        nearest_dist = pairs[0][1]
        cluster_threshold = nearest_dist + 0.08

        cluster = [(a, d) for a, d in pairs if d <= cluster_threshold]
        if not cluster:
            return None, None

        sum_w = 0.0
        sum_x = 0.0
        sum_y = 0.0
        for angle, dist in cluster:
            w = 1.0 / max(dist, 1e-3)
            sum_w += w
            sum_x += w * math.cos(angle)
            sum_y += w * math.sin(angle)

        mean_angle = math.atan2(sum_y / sum_w, sum_x / sum_w)
        mean_dist = sum(d for _, d in cluster) / len(cluster)

        return math.degrees(mean_angle), mean_dist


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarFrontAlignmentTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
