#!/usr/bin/env python3
"""
Vérifie la qualité des scans LIDAR en les superposant sur l'image du circuit
Pour diagnostiquer si le problème vient du LIDAR ou de SLAM
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import pygame
import math
import numpy as np

WIDTH, HEIGHT = 800, 600
MAP_RES = 0.0133

class LidarQualityTest(Node):
    def __init__(self):
        super().__init__('lidar_quality_test')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom_ground_truth', self.odom_callback, qos)
        
        self.latest_scan = None
        self.latest_pose = None
        
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("LIDAR Quality Test - ESC to quit")
        
        # Charger l'image circuit
        try:
            self.circuit_surf = pygame.image.load("image_corridor_grossi.png").convert()
            self.circuit_surf = pygame.transform.scale(self.circuit_surf, (WIDTH, HEIGHT))
        except:
            self.circuit_surf = pygame.Surface((WIDTH, HEIGHT))
            self.circuit_surf.fill((255, 255, 255))
        
        self.get_logger().info('🔍 Test qualité LIDAR - Appuyez ESC pour quitter')
        
    def odom_callback(self, msg):
        # Convertir quaternion en angle
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2.0 * math.atan2(qz, qw)
        
        self.latest_pose = {
            'x': msg.pose.pose.position.x / MAP_RES,  # Retour en pixels
            'y': msg.pose.pose.position.y / MAP_RES,
            'theta': theta
        }
    
    def scan_callback(self, msg):
        self.latest_scan = {
            'ranges': list(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_increment': msg.angle_increment
        }
    
    def draw(self):
        # Fond = image circuit
        self.screen.blit(self.circuit_surf, (0, 0))
        
        if self.latest_scan and self.latest_pose:
            # Robot (bleu)
            rx = int(self.latest_pose['x'])
            ry = int(self.latest_pose['y'])
            pygame.draw.circle(self.screen, (0, 0, 255), (rx, ry), 10)
            
            # Direction
            front_x = int(rx + 15 * math.cos(self.latest_pose['theta']))
            front_y = int(ry + 15 * math.sin(self.latest_pose['theta']))
            pygame.draw.line(self.screen, (255, 0, 0), (rx, ry), (front_x, front_y), 3)
            
            # Points LIDAR (rouge vif pour voir les erreurs)
            ranges = self.latest_scan['ranges']
            angle_min = self.latest_scan['angle_min']
            angle_inc = self.latest_scan['angle_increment']
            
            for i, r in enumerate(ranges):
                if r < 0.15 or r > 3.0 or not math.isfinite(r):
                    continue
                
                # Angle du rayon dans le repère monde
                ray_angle = angle_min + i * angle_inc + self.latest_pose['theta']
                
                # Position du point détecté
                px = int(rx + (r / MAP_RES) * math.cos(ray_angle))
                py = int(ry + (r / MAP_RES) * math.sin(ray_angle))
                
                # Dessiner point (rouge = détection LIDAR)
                if 0 <= px < WIDTH and 0 <= py < HEIGHT:
                    pygame.draw.circle(self.screen, (255, 0, 0), (px, py), 2)
            
            # Stats
            font = pygame.font.SysFont("monospace", 14)
            valid_ranges = [r for r in ranges if 0.15 < r < 3.0 and math.isfinite(r)]
            text = font.render(f"Scans valides: {len(valid_ranges)}/{len(ranges)}", True, (255, 255, 0))
            self.screen.blit(text, (10, 10))
        
        pygame.display.flip()

def main():
    rclpy.init()
    node = LidarQualityTest()
    clock = pygame.time.Clock()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    return
            
            node.draw()
            clock.tick(30)
    finally:
        pygame.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
