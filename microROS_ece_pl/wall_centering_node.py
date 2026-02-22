#!/usr/bin/env python3
"""
ROS2 node: wall_centering_node

Algorithme de centrage entre deux murs à partir d'un LIDAR.
- Souscrit à /scan_raw (LaserScan)
- Publie sur /cmd_vel (Twist)
- Fréquence: 15 Hz

Le robot reste au milieu entre deux murs et adapte sa vitesse pour le mapping.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np  # Assure-toi que numpy est installé (pip install numpy)
import csv
import os

class WallCenteringNode(Node):
    def __init__(self):
        super().__init__('wall_centering_node')
        self.lidar_offset_deg = -89.1  # Offset calibré selon test_lidar_front_alignment
        self.get_logger().info(f"WallCenteringNode __init__ called, lidar_offset_deg={self.lidar_offset_deg}")
        self.stop_requested = False
        # CSV log file setup
        import csv
        import os
        self.csv_log_path = os.path.join(os.path.dirname(__file__), 'logs', 'wall_centering_log.csv')
        self.csv_log_header_written = False
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan_raw', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0/15.0, self.control_step)
        self.last_scan = None
        self.Kp = 0.4  # Correction plus douce
        self.base_speed = 0.25
        self.min_linear_x = 0.3  # Vitesse minimale
        self.safety_dist = 0.5  # Seuil augmenté pour arrêt plus tôt
        self.state = 'forward'  # 'forward' ou 'rotate'
        self.rotate_counter = 0
        self.max_rotate_steps = 5  # Nombre de cycles de rotation
        self.forward_counter = 0
        self.max_forward_steps = 10  # Nombre de cycles d'avance
        self.get_logger().info('Wall Centering Node ready.')

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
            import time
            time.sleep(max(0.0, delay_s))
        return sent

    # ...existing code...

    def scan_callback(self, msg):
        self.last_scan = msg

    def control_step(self):
        # Empêche toute publication après STOP
        if self.stop_requested:
            return
        if self.last_scan is None:
            return

        scan = self.last_scan
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        n = len(ranges)

        def valid_mean(start_deg, end_deg):
            start_rad = np.deg2rad(start_deg)
            end_rad = np.deg2rad(end_deg)
            i_start = int((start_rad - angle_min) / angle_inc)
            i_end = int((end_rad - angle_min) / angle_inc)
            i_start = np.clip(i_start, 0, n-1)
            i_end = np.clip(i_end, 0, n-1)
            vals = ranges[i_start:i_end+1]
            vals = vals[np.isfinite(vals)]
            return np.mean(vals) if len(vals) > 0 else np.nan

        # Secteurs pour LIDAR 360°
        left_start = -10
        left_end = +10
        front_start = 80
        front_end = 100
        right_start = 170
        right_end = 190

        d_left = valid_mean(left_start, left_end)
        d_right = valid_mean(right_start, right_end)
        d_front = valid_mean(front_start, front_end)
        error = d_left - d_right

        twist = Twist()

        # Séquence alternée
        if np.isfinite(d_front) and d_front < self.safety_dist:
            # Rotation stabilisée : toujours à gauche
            self.state = 'rotate'
            twist.linear.x = 0.0
            twist.angular.z = 1.3  # Toujours à gauche, rotation saturée
            # On ne sort de rotate que si d_front > safety_dist
            if np.isfinite(d_front) and d_front >= self.safety_dist:
                self.state = 'forward'
                self.forward_counter = 0
        elif self.state == 'forward':
            if self.forward_counter < self.max_forward_steps:
                twist.linear.x = 0.15  # Avance doucement
                twist.angular.z = 0.0
                self.forward_counter += 1
            else:
                self.state = 'rotate'
                self.rotate_counter = 0
        else:
            # Contrôle normal
            angular_z = -self.Kp * error
            # Ajoute un offset pour tourner plus fort dès qu'il y a une erreur
            if abs(error) > 0.01:
                angular_z += np.sign(angular_z) * 0.3
            linear_x = self.base_speed * (1 - abs(angular_z)/1.3)
            linear_x = max(self.min_linear_x, linear_x)
            twist.linear.x = float(linear_x)
            twist.angular.z = float(angular_z)

        # Clip angular_z only at publication
        twist.angular.z = float(np.clip(twist.angular.z, -1.3, 1.3))
        self.cmd_pub.publish(twist)

        # Mur gauche
        d_left = valid_mean(left_start, left_end)
        # Mur droit
        d_right = valid_mean(right_start, right_end)
        # Mur frontal
        d_front = valid_mean(front_start, front_end)
        # Debug secteur frontal
        front_sector = ranges[int((np.deg2rad(-100) - angle_min) / angle_inc):int((np.deg2rad(-80) - angle_min) / angle_inc)+1]
        front_valid = front_sector[np.isfinite(front_sector)]
        self.get_logger().info(f"FRONT SECTOR: {front_valid}")

        # Calcul de l'erreur de centrage
        error = d_left - d_right

        # Contrôleur proportionnel
        angular_z = -self.Kp * error
        # Saturation
        angular_z = np.clip(angular_z, -1.3, 1.3)

        # Adaptation de la vitesse linéaire
        linear_x = self.base_speed * (1 - abs(angular_z))
        # Imposer une vitesse minimale
        linear_x = max(self.min_linear_x, linear_x)

        # Sécurité frontale
        if np.isfinite(d_front) and d_front < self.safety_dist:
            linear_x = 0.0
            angular_z = 1.3  # Toujours à gauche, suppression oscillation
            self.get_logger().info("Obstacle devant, rotation stabilisée à gauche (max 1.3 rad/s)")

        # Publication
        # Publication déjà faite plus haut, ne pas dupliquer

        # Debug log
        self.get_logger().info(
            f"d_left={d_left:.2f} d_right={d_right:.2f} d_front={d_front:.2f} error={error:.2f} cmd=({linear_x:.2f},{angular_z:.2f})")
        # CSV logging
        import time
        log_row = [time.time(), d_left, d_right, d_front, error, linear_x, angular_z, self.state]
        try:
            write_header = False
            if not self.csv_log_header_written:
                write_header = not os.path.exists(self.csv_log_path)
            with open(self.csv_log_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                if write_header:
                    writer.writerow(['timestamp', 'd_left', 'd_right', 'd_front', 'error', 'cmd_v', 'cmd_w', 'state'])
                    self.csv_log_header_written = True
                writer.writerow(log_row)
        except Exception as e:
            self.get_logger().warn(f"CSV log error: {e}")


def main():
    import signal
    from rclpy.signals import SignalHandlerOptions

    stop_requested = {'value': False}
    stop_done = {'value': False}

    def _run_stop_sequence(node: WallCenteringNode, reason: str) -> None:
        if stop_done['value']:
            return
        stop_done['value'] = True
        node.stop_requested = True  # Bloque toute publication
        print(f"\n[STOP] Triggered by: {reason}", flush=True)
        print(f"[STOP] Sending burst 1: repeats=30, delay=0.04s", flush=True)
        try:
            node.publish_stop(repeats=30, delay_s=0.04)
        except Exception as exc:
            print(f"[STOP] Burst 1 exception: {exc}", flush=True)
        print(f"[STOP] Sending burst 2: repeats=20, delay=0.04s", flush=True)
        try:
            node.publish_stop(repeats=20, delay_s=0.04)
        except Exception as exc:
            print(f"[STOP] Burst 2 exception: {exc}", flush=True)
        print(f"[STOP] Settle wait: 0.30s", flush=True)
        import time
        time.sleep(0.30)
        print(f"[STOP] Done", flush=True)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WallCenteringNode()

    def _sigint_handler(signum, frame):
        del signum, frame
        node.stop_requested = True
        _run_stop_sequence(node, 'SIGINT signal handler')

    previous_sigint_handler = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, _sigint_handler)

    # Suppression de l'appel STOP au démarrage

    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.stop_requested:
            _run_stop_sequence(node, 'SIGINT handler')
    except KeyboardInterrupt:
        _run_stop_sequence(node, 'KeyboardInterrupt')
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
