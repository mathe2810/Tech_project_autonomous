#!/usr/bin/env python3
"""
Publie la vraie position du robot simulé sur /odom (Odometry)
Récupère robot_pos et robot_theta via socket, publie à 30Hz
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import socket
import pickle

HOST = '127.0.0.1'
PORT = 5006

class SimuOdomPublisher(Node):
    def __init__(self):
        super().__init__('simu_odom_publisher')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((HOST, PORT))
        self.sock.listen(1)
        print(f"SimuOdomPublisher listening on {HOST}:{PORT}")
        self.conn, _ = self.sock.accept()
        print("SimuOdomPublisher: client connected")
        self.timer = self.create_timer(1.0/30.0, self.publish_odom)
        self.last_pose = (0.0, 0.0, 0.0)

    def publish_odom(self):
        MAP_RESOLUTION = 0.02  # 2 cm/pixel, à adapter si besoin
        WIDTH = 800
        HEIGHT = 600
        # Origine map.yaml (par défaut [0.0, 0.0, 0.0])
        # Pour avoir l'origine au centre, décaler :
        x_origin = WIDTH / 2
        y_origin = HEIGHT / 2
        try:
            data = self.conn.recv(1024)
            robot_pos, robot_theta = pickle.loads(data)
            self.last_pose = (robot_pos[0], robot_pos[1], robot_theta)
        except Exception:
            robot_pos, robot_theta = self.last_pose[:2], self.last_pose[2]

        # Conversion pixels -> mètres avec origine au centre
        x_m = (robot_pos[0] - x_origin) * MAP_RESOLUTION
        y_m = (robot_pos[1] - y_origin) * MAP_RESOLUTION

        now = self.get_clock().now()
        # Publication /odom et TF désactivée pour éviter conflit avec rf2o_laser_odometry
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SimuOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
