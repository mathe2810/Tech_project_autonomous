#!/usr/bin/env python3
"""Reactive LiDAR trajectory node for tight circuits.

Algorithm: Follow-The-Gap (FTG)
- subscribes to /scan_raw (LaserScan)
- publishes /cmd_vel (Twist)
- handles consecutive opposite turns better than single-wall following
"""

import math
import signal
import time
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class LidarGapFollower(Node):
    def __init__(self) -> None:
        super().__init__('lidar_gap_follower')

        self.declare_parameter('scan_topic', '/scan_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('max_range_clip', 4.0)
        self.declare_parameter('front_fov_deg', 220.0)
        # LiDAR front offset calibration: -90 degrees is the true front
        self.declare_parameter('lidar_front_offset_deg', -90.0)
        self.declare_parameter('stop_distance_m', 0.20)
        self.declare_parameter('slow_distance_m', 0.55)
        self.declare_parameter('bubble_radius_m', 0.25)
        self.declare_parameter('min_valid_range_m', 0.05)
        self.declare_parameter('max_linear_mps', 0.5)
        self.declare_parameter('min_linear_mps', 0.2)
        self.declare_parameter('max_angular_rps', 2)
        self.declare_parameter('steering_smoothing', 0.45)
        self.declare_parameter('steering_penalty', 0.35)
        self.declare_parameter('control_hz', 15.0)
        self.declare_parameter('linear_accel_limit_mps2', 0.35)
        self.declare_parameter('angular_accel_limit_rps2', 2.8)
        self.declare_parameter('stuck_trigger_s', 0.70)
        self.declare_parameter('recovery_reverse_mps', -0.09)
        self.declare_parameter('recovery_reverse_s', 0.35)
        self.declare_parameter('recovery_turn_rps', 1.20)
        self.declare_parameter('recovery_turn_s', 0.55)
        self.declare_parameter('debug_enabled', True)
        self.declare_parameter('debug_interval_s', 0.50)
        self.declare_parameter('near_obstacle_dist_m', 0.34)
        self.declare_parameter('near_obstacle_min_linear_mps', 0.15)
        self.declare_parameter('near_obstacle_max_angular_rps', 1.1)
        self.declare_parameter('stop_delay_s', 0.04)
        self.declare_parameter('stop_repeats_startup', 4)
        self.declare_parameter('stop_repeats_sigint', 30)
        self.declare_parameter('stop_repeats_final', 20)
        self.declare_parameter('stop_settle_s', 0.30)

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.max_range_clip = float(self.get_parameter('max_range_clip').value)
        self.front_fov_deg = float(self.get_parameter('front_fov_deg').value)
        self.lidar_front_offset_deg = float(self.get_parameter('lidar_front_offset_deg').value)
        self.stop_distance_m = float(self.get_parameter('stop_distance_m').value)
        self.slow_distance_m = float(self.get_parameter('slow_distance_m').value)
        self.bubble_radius_m = float(self.get_parameter('bubble_radius_m').value)
        self.min_valid_range_m = float(self.get_parameter('min_valid_range_m').value)
        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.min_linear_mps = float(self.get_parameter('min_linear_mps').value)
        self.max_angular_rps = float(self.get_parameter('max_angular_rps').value)
        self.steering_smoothing = float(self.get_parameter('steering_smoothing').value)
        self.linear_accel_limit_mps2 = float(self.get_parameter('linear_accel_limit_mps2').value)
        self.angular_accel_limit_rps2 = float(self.get_parameter('angular_accel_limit_rps2').value)
        self.stuck_trigger_s = float(self.get_parameter('stuck_trigger_s').value)
        self.recovery_reverse_mps = float(self.get_parameter('recovery_reverse_mps').value)
        self.recovery_reverse_s = float(self.get_parameter('recovery_reverse_s').value)
        self.recovery_turn_rps = float(self.get_parameter('recovery_turn_rps').value)
        self.recovery_turn_s = float(self.get_parameter('recovery_turn_s').value)
        self.debug_enabled = bool(self.get_parameter('debug_enabled').value)
        self.debug_interval_s = float(self.get_parameter('debug_interval_s').value)
        self.near_obstacle_dist_m = float(self.get_parameter('near_obstacle_dist_m').value)
        self.near_obstacle_min_linear_mps = float(self.get_parameter('near_obstacle_min_linear_mps').value)
        self.near_obstacle_max_angular_rps = float(self.get_parameter('near_obstacle_max_angular_rps').value)
        self.stop_delay_s = float(self.get_parameter('stop_delay_s').value)
        self.stop_repeats_startup = int(self.get_parameter('stop_repeats_startup').value)
        self.stop_repeats_sigint = int(self.get_parameter('stop_repeats_sigint').value)
        self.stop_repeats_final = int(self.get_parameter('stop_repeats_final').value)
        self.stop_settle_s = float(self.get_parameter('stop_settle_s').value)

        control_hz = float(self.get_parameter('control_hz').value)
        self.control_dt = 1.0 / max(control_hz, 1.0)
        self.steering_penalty = float(self.get_parameter('steering_penalty').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(self.control_dt, self.control_step)

        self.last_scan: Optional[LaserScan] = None
        self.filtered_steer = 0.0
        self.scan_count = 0
        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0
        self.blocked_count = 0
        self.recovery_phase: Optional[str] = None
        self.recovery_steps_left = 0
        self.recovery_turn_sign = 1.0
        self.last_debug_time = 0.0

        self.get_logger().info(
            f'LiDAR Gap Follower ready: {self.scan_topic} -> {self.cmd_topic} at {control_hz:.1f} Hz'
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.scan_count += 1

    def control_step(self) -> None:
        twist = Twist()

        if self.last_scan is None:
            self.cmd_pub.publish(twist)
            return

        if self.recovery_phase is not None:
            self.step_recovery()
            return

        steer_angle, front_min = self.compute_steering(self.last_scan)

        if front_min <= self.stop_distance_m:
            self.blocked_count += 1
        else:
            self.blocked_count = 0

        blocked_time = self.blocked_count * self.control_dt
        if blocked_time >= self.stuck_trigger_s:
            self.start_recovery(self.last_scan)
            self.step_recovery()
            return

        if front_min <= self.stop_distance_m:
            twist.linear.x = 0.0
            target_angular = self.clamp(2.2 * steer_angle, -self.max_angular_rps, self.max_angular_rps)
            twist.angular.z = self.slew_limit(target_angular, self.last_angular_cmd, self.angular_accel_limit_rps2)
            self.last_linear_cmd = twist.linear.x
            self.last_angular_cmd = twist.angular.z
            self.cmd_pub.publish(twist)
            self.debug_log(
                f"mode=avoid front={front_min:.2f} steer={steer_angle:.2f} cmd=({twist.linear.x:.2f},{twist.angular.z:.2f})"
            )
            return

        linear = self.speed_from_front_clearance(front_min)
        turn_ratio = min(abs(steer_angle) / math.radians(65.0), 1.0)
        linear *= (1.0 - 0.6 * turn_ratio)
        linear = self.clamp(linear, self.min_linear_mps, self.max_linear_mps)

        angular_cmd = self.clamp(2.0 * steer_angle, -self.max_angular_rps, self.max_angular_rps)

        # Anti-stall shaping: near obstacles, keep enough forward thrust and avoid over-rotation
        if front_min <= self.near_obstacle_dist_m:
            linear = max(linear, self.near_obstacle_min_linear_mps)
            angular_cmd = self.clamp(
                angular_cmd,
                -self.near_obstacle_max_angular_rps,
                self.near_obstacle_max_angular_rps,
            )

        twist.linear.x = self.slew_limit(linear, self.last_linear_cmd, self.linear_accel_limit_mps2)
        twist.angular.z = self.slew_limit(angular_cmd, self.last_angular_cmd, self.angular_accel_limit_rps2)
        self.last_linear_cmd = twist.linear.x
        self.last_angular_cmd = twist.angular.z
        self.cmd_pub.publish(twist)
        self.debug_log(
            f"mode=ftg front={front_min:.2f} steer={steer_angle:.2f} cmd=({twist.linear.x:.2f},{twist.angular.z:.2f})"
        )

    def start_recovery(self, scan: LaserScan) -> None:
        self.blocked_count = 0
        self.recovery_phase = 'reverse'
        self.recovery_steps_left = max(1, int(self.recovery_reverse_s / self.control_dt))

        left_clear, right_clear = self.compute_side_clearance(scan)
        self.recovery_turn_sign = 1.0 if left_clear >= right_clear else -1.0

        turn_side = 'left' if self.recovery_turn_sign > 0.0 else 'right'
        self.get_logger().info(f'[RECOVERY] Triggered -> reverse then turn {turn_side}')
        self.debug_log(
            f"recovery_start left_clear={left_clear:.2f} right_clear={right_clear:.2f} turn={turn_side}",
            force=True,
        )

    def step_recovery(self) -> None:
        if self.recovery_phase is None:
            return

        twist = Twist()
        if self.recovery_phase == 'reverse':
            twist.linear.x = self.recovery_reverse_mps
            twist.angular.z = 0.0
        elif self.recovery_phase == 'turn':
            twist.linear.x = 0.0
            twist.angular.z = self.recovery_turn_sign * self.recovery_turn_rps

        twist.linear.x = self.slew_limit(twist.linear.x, self.last_linear_cmd, self.linear_accel_limit_mps2)
        twist.angular.z = self.slew_limit(twist.angular.z, self.last_angular_cmd, self.angular_accel_limit_rps2)

        self.last_linear_cmd = twist.linear.x
        self.last_angular_cmd = twist.angular.z
        self.cmd_pub.publish(twist)
        self.debug_log(
            f"mode=recovery phase={self.recovery_phase} steps_left={self.recovery_steps_left} cmd=({twist.linear.x:.2f},{twist.angular.z:.2f})"
        )

        self.recovery_steps_left -= 1
        if self.recovery_steps_left > 0:
            return

        if self.recovery_phase == 'reverse':
            self.recovery_phase = 'turn'
            self.recovery_steps_left = max(1, int(self.recovery_turn_s / self.control_dt))
            self.debug_log('recovery_phase_switch reverse->turn', force=True)
            return

        self.recovery_phase = None
        self.recovery_steps_left = 0
        self.debug_log('recovery_done resume=ftg', force=True)

    def compute_side_clearance(self, scan: LaserScan) -> Tuple[float, float]:
        ranges = self.sanitize_ranges(scan)
        if not ranges:
            return 0.0, 0.0

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        n = len(ranges)

        left_start = max(0, int((math.radians(35.0) - angle_min) / angle_inc))
        left_end = min(n - 1, int((math.radians(120.0) - angle_min) / angle_inc))
        right_start = max(0, int((math.radians(-120.0) - angle_min) / angle_inc))
        right_end = min(n - 1, int((math.radians(-35.0) - angle_min) / angle_inc))

        left_sector = ranges[left_start:left_end + 1] if left_end >= left_start else []
        right_sector = ranges[right_start:right_end + 1] if right_end >= right_start else []

        left_clear = max(left_sector) if left_sector else 0.0
        right_clear = max(right_sector) if right_sector else 0.0
        return left_clear, right_clear

    def compute_steering(self, scan: LaserScan) -> Tuple[float, float]:
        ranges = self.sanitize_ranges(scan)
        if not ranges:
            return 0.0, 0.0

        total = len(ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment

        # Apply LiDAR front offset calibration
        front_offset_rad = math.radians(self.lidar_front_offset_deg)
        half_fov = math.radians(self.front_fov_deg * 0.5)
        i_start = max(0, int((front_offset_rad - half_fov - angle_min) / angle_inc))
        i_end = min(total - 1, int((front_offset_rad + half_fov - angle_min) / angle_inc))
        if i_end <= i_start:
            return 0.0, min(ranges)

        sector = ranges[i_start:i_end + 1]
        front_min = min(sector)

        closest_local = min(range(len(sector)), key=sector.__getitem__)
        closest_dist = sector[closest_local]

        bubble_half_count = self.bubble_half_width_indices(
            radius=self.bubble_radius_m,
            distance=max(closest_dist, self.min_valid_range_m),
            angle_inc=angle_inc,
        )

        blocked = sector[:]
        b_start = max(0, closest_local - bubble_half_count)
        b_end = min(len(blocked) - 1, closest_local + bubble_half_count)
        for i in range(b_start, b_end + 1):
            blocked[i] = 0.0

        best_gap = self.find_largest_gap(blocked, min_free=self.stop_distance_m)
        if best_gap is None:
            return 0.0, front_min

        g0, g1 = best_gap
        target_local = self.pick_best_point_in_gap(
            raw=sector,
            gap_start=g0,
            gap_end=g1,
            angle_start=angle_min + (i_start * angle_inc),
            angle_inc=angle_inc,
        )

        target_idx = i_start + target_local
        target_angle = angle_min + (target_idx * angle_inc)

        self.filtered_steer = (
            (1.0 - self.steering_smoothing) * target_angle
            + self.steering_smoothing * self.filtered_steer
        )
        return self.filtered_steer, front_min

    def sanitize_ranges(self, scan: LaserScan) -> List[float]:
        out: List[float] = []
        low = max(scan.range_min, self.min_valid_range_m)
        high = min(scan.range_max, self.max_range_clip)

        for value in scan.ranges:
            if value is None or math.isinf(value) or math.isnan(value):
                out.append(high)
            else:
                out.append(self.clamp(float(value), low, high))

        return out

    def bubble_half_width_indices(self, radius: float, distance: float, angle_inc: float) -> int:
        half_angle = math.atan2(radius, max(distance, 1e-3))
        return max(1, int(half_angle / max(angle_inc, 1e-6)))

    def find_largest_gap(self, blocked: List[float], min_free: float) -> Optional[Tuple[int, int]]:
        best = None
        best_len = 0
        start = None

        for i, d in enumerate(blocked):
            if d > min_free:
                if start is None:
                    start = i
            else:
                if start is not None:
                    length = i - start
                    if length > best_len:
                        best = (start, i - 1)
                        best_len = length
                    start = None

        if start is not None:
            length = len(blocked) - start
            if length > best_len:
                best = (start, len(blocked) - 1)

        return best

    def pick_best_point_in_gap(
        self,
        raw: List[float],
        gap_start: int,
        gap_end: int,
        angle_start: float,
        angle_inc: float,
    ) -> int:
        center = 0.5 * (gap_start + gap_end)
        best_i = int(center)
        best_score = -1e9

        for i in range(gap_start, gap_end + 1):
            angle = angle_start + (i * angle_inc)
            score = raw[i] - self.steering_penalty * abs(angle)
            if score > best_score:
                best_score = score
                best_i = i

        return best_i

    def speed_from_front_clearance(self, front_min: float) -> float:
        if front_min <= self.slow_distance_m:
            t = (front_min - self.stop_distance_m) / max(self.slow_distance_m - self.stop_distance_m, 1e-3)
            t = self.clamp(t, 0.0, 1.0)
            return self.min_linear_mps + t * (self.max_linear_mps - self.min_linear_mps)
        return self.max_linear_mps

    def slew_limit(self, target: float, previous: float, accel_limit: float) -> float:
        max_step = max(accel_limit, 1e-3) * self.control_dt
        delta = self.clamp(target - previous, -max_step, max_step)
        return previous + delta

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(value, high))

    def publish_stop(self, repeats: int = 5, delay_s: float = 0.03) -> int:
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        print("⊙ STOP", flush=True)
        sent = 0
        for _ in range(max(1, repeats)):
            try:
                self.cmd_pub.publish(stop)
                sent += 1
            except Exception:
                break
            time.sleep(max(0.0, delay_s))
        self.last_linear_cmd = 0.0
        self.last_angular_cmd = 0.0
        if rclpy.ok():
            self.debug_log(f'stop_published repeats={repeats}', force=True)
        return sent

    def debug_log(self, message: str, force: bool = False) -> None:
        if not self.debug_enabled or not rclpy.ok():
            return

        now = time.time()
        if not force and (now - self.last_debug_time) < max(0.05, self.debug_interval_s):
            return

        self.last_debug_time = now
        self.get_logger().info(f'[DEBUG] {message}')


def main(args=None) -> None:
    stop_requested = {'value': False}
    stop_done = {'value': False}

    def _run_stop_sequence(node: LidarGapFollower, reason: str) -> None:
        if stop_done['value']:
            return
        stop_done['value'] = True

        print(f"\n[STOP] Triggered by: {reason}", flush=True)
        print(
            f"[STOP] Sending burst 1: repeats={node.stop_repeats_sigint}, delay={node.stop_delay_s:.2f}s",
            flush=True,
        )
        sent_1 = 0
        try:
            sent_1 = node.publish_stop(repeats=node.stop_repeats_sigint, delay_s=node.stop_delay_s)
        except Exception as exc:
            print(f"[STOP] Burst 1 exception: {exc}", flush=True)

        print(
            f"[STOP] Sending burst 2: repeats={node.stop_repeats_final}, delay={node.stop_delay_s:.2f}s",
            flush=True,
        )
        sent_2 = 0
        try:
            sent_2 = node.publish_stop(repeats=node.stop_repeats_final, delay_s=node.stop_delay_s)
        except Exception as exc:
            print(f"[STOP] Burst 2 exception: {exc}", flush=True)

        settle_s = max(0.0, node.stop_settle_s)
        if settle_s > 0.0:
            print(f"[STOP] Settle wait: {settle_s:.2f}s", flush=True)
            time.sleep(settle_s)

        print(f"[STOP] Done (sent={sent_1 + sent_2})", flush=True)

    # Disable rclpy default SIGINT handling so we can publish STOP before context shutdown
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = LidarGapFollower()

    def _sigint_handler(signum, frame):
        del signum, frame
        stop_requested['value'] = True
        _run_stop_sequence(node, 'SIGINT signal handler')

    previous_sigint_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, _sigint_handler)

    # Clear any stale command at startup
    try:
        node.publish_stop(repeats=node.stop_repeats_startup, delay_s=node.stop_delay_s)
    except Exception:
        pass

    try:
        while rclpy.ok() and not stop_requested['value']:
            rclpy.spin_once(node, timeout_sec=0.1)

        if stop_requested['value']:
            _run_stop_sequence(node, 'SIGINT handler')
    except (KeyboardInterrupt, ExternalShutdownException):
        _run_stop_sequence(node, 'KeyboardInterrupt/ExternalShutdownException')
    finally:
        _run_stop_sequence(node, 'finalize')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            signal.signal(signal.SIGINT, previous_sigint_handler)
        except Exception:
            pass


if __name__ == '__main__':
    main()
