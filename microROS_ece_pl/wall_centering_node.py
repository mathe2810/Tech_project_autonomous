#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import pygame
import math
import time

class CorridorRailNode(Node):
    def __init__(self):
        super().__init__('corridor_rail_node')
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        
        # --- RÉGLAGES "RAIL VIRTUEL" — MODE CARRELAGE ---
        self.cruise_speed = 0.4       # Vitesse nominale stable
        self.alpha_rot = 0.80         # Amortissement des oscillations
        self.deadzone = 0.10          # Inchangé
        self.gain_rot = 1.0           # Correction angulaire : force légèrement plus de rotation
        self.min_rotation = 0.38      # Seuil moteur : ne crée PAS de virage énorme
        
        self.prev_w = 0.0
        self.last_scan = None
        
        # --- PAUSE PÉRIODIQUE POUR SLAM ---
        self.last_pause_time = time.time()
        self.pause_interval_s = 5.0   # Pause toutes les 5 secondes
        self.pause_duration_s = 1.5   # Arrêt de 1.5 secondes
        self.is_paused = False
        self.pause_start_time = None
        
        pygame.init()
        self.screen = pygame.display.set_mode((600, 750))
        self.font = pygame.font.SysFont("monospace", 18, bold=True)

    def callback(self, msg):
        self.last_scan = msg
        self.run_logic()

    def run_logic(self):
        if not self.last_scan: return

        # --- GESTION PAUSE POUR SLAM ---
        now = time.time()
        
        # Vérifier si une pause est en cours
        if self.is_paused:
            if now - self.pause_start_time >= self.pause_duration_s:
                # Fin de pause, reprendre
                self.is_paused = False
                self.last_pause_time = now
                self.get_logger().info("✅ Pause SLAM terminée, reprise mouvement")
            else:
                # Toujours en pause, publier STOP
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.pub.publish(stop_cmd)
                return
        
        # Vérifier si une pause est nécessaire
        if now - self.last_pause_time >= self.pause_interval_s:
            self.is_paused = True
            self.pause_start_time = now
            self.get_logger().info(f"⏸️  PAUSE SLAM ({self.pause_duration_s}s) - Map update")
            # Publier STOP immédiatement
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.pub.publish(stop_cmd)
            return

        ranges = np.array(self.last_scan.ranges)
        # On limite la portée à 2.0m : inutile de voir plus loin dans un couloir
        ranges = np.where(np.isfinite(ranges) & (ranges > 0.15), ranges, 2.0)
        
        angle_inc = self.last_scan.angle_increment
        idx_front = int((math.pi/2 - self.last_scan.angle_min) / angle_inc)
        idx_front = 0
        
        # On regarde large sur les côtés (60°) pour bien capter les murs du couloir
        side_angle = int(math.radians(60) / angle_inc)
        window = int(math.radians(20) / angle_inc)

        # Mesure de la distance aux murs latéraux
        dist_l = np.mean(ranges[idx_front + side_angle - window : idx_front + side_angle + window])
        dist_r = np.mean(ranges[idx_front - side_angle - window : idx_front - side_angle + window])
        # Distance devant pour freiner si le virage est trop serré
        dist_f = np.mean(ranges[idx_front - 10 : idx_front + 10])

        # --- LOGIQUE DU RAIL ---
        # Si L=0.5 et R=0.5, error=0 (Parfaitement centré)
        # Si L=0.3 et R=0.7, error=-0.4 (Trop à gauche, on doit tourner à droite)
        error = dist_l - dist_r
        
        if abs(error) < self.deadzone:
            target_w = 0.0
        else:
            # On calcule une rotation proportionnelle très douce
            target_w = error * self.gain_rot
            # Application du seuil moteur
            if abs(target_w) < self.min_rotation:
                target_w = np.sign(target_w) * self.min_rotation

        # --- FILTRE ANTI-ZIGZAG ---
        # On interdit les changements brusques (Slew Rate Limit)
        smoothed_w = (self.alpha_rot * self.prev_w) + ((1 - self.alpha_rot) * target_w)
        
        # On limite l'accélération de la rotation
        max_delta = 0.1
        diff = smoothed_w - self.prev_w
        if abs(diff) > max_delta:
            smoothed_w = self.prev_w + np.sign(diff) * max_delta
            
        self.prev_w = smoothed_w

        # Commande
        cmd = Twist()
        # On ralentit un peu si le mur d'en face se rapproche (virage serré)
        cmd.linear.x = float(self.cruise_speed if dist_f > 0.8 else self.cruise_speed * 0.7)
        cmd.angular.z = float(np.clip(smoothed_w, -1.0, 1.0))
        
        self.pub.publish(cmd)
        self.draw_ui(ranges, idx_front, dist_l, dist_r, cmd)

    def draw_ui(self, ranges, idx_f, dl, dr, cmd):
        self.screen.fill((15, 15, 20))
        cx, cy = 300, 400
        # Affichage du couloir
        for i in range(0, len(ranges), 2):
            r = ranges[i]
            if r >= 2.0: continue
            a = (i - idx_f) * self.last_scan.angle_increment - math.pi/2
            px, py = int(cx + r*150*math.cos(a)), int(cy + r*150*math.sin(a))
            pygame.draw.circle(self.screen, (0, 255, 150), (px, py), 1)

        # Robot
        pygame.draw.rect(self.screen, (255, 255, 255), (cx-10, cy-15, 20, 30), 2)
        
        # HUD
        pygame.draw.rect(self.screen, (40, 40, 50), (0, 0, 600, 110))
        txt_v = self.font.render(f"VITESSE : {cmd.linear.x:.2f} m/s", True, (255, 255, 255))
        txt_w = self.font.render(f"DIRECTION : {cmd.angular.z:.3f} rad/s", True, (255, 150, 0))
        txt_pos = self.font.render(f"POSITION L:{dl:.2f}m | R:{dr:.2f}m", True, (100, 255, 100))
        
        self.screen.blit(txt_v, (20, 15))
        self.screen.blit(txt_w, (20, 45))
        self.screen.blit(txt_pos, (20, 75))
        
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT: rclpy.shutdown()

def main():
    rclpy.init()
    node = CorridorRailNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop long pour purger le WiFi
        stop = Twist()
        for _ in range(100):
            node.pub.publish(stop)
            time.sleep(0.02)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()