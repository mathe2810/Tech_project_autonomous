#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import socket, pickle, math

class SimuBridge(Node):
    def __init__(self):
        super().__init__('simu_bridge')
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_br = TransformBroadcaster(self)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # Variables pour garder en mémoire la dernière position connue
        self.last_pose = [0.0, 0.0, 0.0]
        self.last_ranges = [0.0] * 180 
        
        try:
            self.sock.connect(('127.0.0.1', 5005))
            self.sock.setblocking(False)
            self.get_logger().info('✅ Bridge Connecté à 50Hz')
        except Exception as e:
            self.get_logger().error(f'Erreur : {e}')
        
        self.create_timer(0.02, self.loop) # 50Hz constant
        self.map_res = 0.0133

    def loop(self):
        data = None
        # On essaie de lire le dernier paquet disponible
        try:
            while True:
                part = self.sock.recv(65536) # Buffer plus gros pour pickle
                if not part: break
                data = part
        except (BlockingIOError, socket.error):
            pass

        if data:
            try:
                msg = pickle.loads(data)
                self.last_pose = msg['pose']
                self.last_ranges = msg['ranges']
            except:
                pass

        # ON PUBLIE TOUJOURS, même si data est vide (on utilise la dernière pose)
        # C'est ce qui empêche le "Failed to compute odom pose"
        stamp = self.get_clock().now().to_msg()
        px, py, pth = self.last_pose
        x, y, th = px * self.map_res, -py * self.map_res, -pth

        # TF Odom -> Base
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        # Correction Quaternion (Normalisation)
        t.transform.rotation.z = math.sin(th / 2.0)
        t.transform.rotation.w = math.cos(th / 2.0)
        self.tf_br.sendTransform(t)

        # Scan
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_link'
        scan.angle_min, scan.angle_max = 0.0, 2.0*math.pi
        scan.angle_increment = (2.0*math.pi) / len(self.last_ranges)
        scan.range_min, scan.range_max = 0.1, 5.0
        scan.ranges = [float(r) for r in self.last_ranges]
        self.scan_pub.publish(scan)

def main():
    rclpy.init()
    node = SimuBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()