#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
import socket, pickle, struct, math, time
import threading

class SimuBridge(Node):
    def __init__(self):
        super().__init__('simu_bridge')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos)
        self.odom_gt_pub = self.create_publisher(Odometry, '/odom_ground_truth', qos)
        self._buf = b''
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect(('127.0.0.1', 5005))
            self.sock.setblocking(False)
            self.get_logger().info('✅ Bridge connecté -> /scan')
        except Exception as e:
            self.get_logger().error(f"Erreur connexion : {e}")
        
        # Throttle scans to reduce SLAM Message Filter overload
        self.scan_count = 0
        self.scan_throttle = 2  # Publish 1 scan every 2 scans (50% reduction)
        
        # Utilise un Timer ROS au lieu de spin_once
        self.create_timer(0.01, self.update)

    def update(self):
        try:
            chunk = self.sock.recv(65536)
            if not chunk: return
            self._buf += chunk
            while len(self._buf) >= 4:
                size = struct.unpack('>I', self._buf[:4])[0]
                if len(self._buf) < 4 + size: break
                data = pickle.loads(self._buf[4:4+size])
                self._buf = self._buf[4+size:]
                
                now = self.get_clock().now().to_msg()
                
                # Publier le scan LaserScan
                scan = LaserScan()
                scan.header.stamp = now
                scan.header.frame_id = 'laser_link'
                
                num_rays = len(data['ranges'])
                scan.angle_min = -math.pi
                scan.angle_increment = (2.0 * math.pi) / num_rays
                scan.angle_max = scan.angle_min + (num_rays - 1) * scan.angle_increment
                scan.time_increment = 0.0
                scan.range_min = 0.15
                scan.range_max = 1.0  # Réduit à 1.0m pour minimiser le bruit blanc SLAM
                
                # Garder les rayons dans l'ordre - le simulateur les envoie déjà bien
                scan.ranges = [float(r) for r in data['ranges']]
                scan.intensities = []
                
                # Apply throttle: publish 1 out of N scans
                self.scan_count += 1
                if self.scan_count % self.scan_throttle == 0:
                    self.scan_pub.publish(scan)
                
                # Publier l'odométrie de vérité terrain (ground truth)
                if 'pose' in data:
                    pose_data = data['pose']
                    odom_msg = Odometry()
                    odom_msg.header.stamp = now
                    odom_msg.header.frame_id = 'odom'
                    odom_msg.child_frame_id = 'base_link'
                    
                    odom_msg.pose.pose.position.x = float(pose_data['x'])
                    odom_msg.pose.pose.position.y = float(pose_data['y'])
                    odom_msg.pose.pose.position.z = 0.0
                    
                    q = pose_data.get('quaternion', [0, 0, 0, 1])
                    odom_msg.pose.pose.orientation.x = float(q[0])
                    odom_msg.pose.pose.orientation.y = float(q[1])
                    odom_msg.pose.pose.orientation.z = float(q[2])
                    odom_msg.pose.pose.orientation.w = float(q[3])
                    
                    self.odom_gt_pub.publish(odom_msg)
                    
                    # Debug logging (once per 30 frames)
                    if int(time.time() * 30) % 30 == 0:
                        self.get_logger().info(
                            f"🎯 Ground Truth: x={pose_data['x']:6.3f}m y={pose_data['y']:6.3f}m "
                            f"θ={pose_data['theta']:6.3f}rad q=[{q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f}]"
                        )
                    
        except BlockingIOError: pass
        except Exception as e:
            self.get_logger().error(f'❌ Error: {e}')

def main():
    try:
        rclpy.init()
    except:
        pass
    
    node = SimuBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            rclpy.shutdown()
        except:
            # Context already shutdown
            pass

if __name__ == '__main__':
    main()