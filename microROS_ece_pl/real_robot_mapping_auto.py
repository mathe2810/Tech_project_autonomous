#!/usr/bin/env python3
"""
REAL ROBOT MAPPING AUTO - Orchestrateur principal pour robot réel

Gère les 3 phases du mapping autonome:
1. AUTONOME: Suivi mur + enregistrement trajec (wall_centering_node)
2. DÉTECTION: Boucle complétée (loop_closure_detector)
3. RETOUR: Suivi chemin A* vers le départ (simple PID control)

Lance en parallèle:
- wall_centering_node (autonomie)
- loop_closure_detector (détection boucle)
- trajectory_saver (sauvegarde trajec)
- return_path_generator (A* planning)
"""

import os
import math
import time
import json
import numpy as np
from pathlib import Path
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import LaserScan


class RobotState(Enum):
    """États du robot durant le mapping autonome"""
    AUTONOME = 1        # Mode autonome (suivi mur)
    RETURN_PLANNED = 2  # Retour planifié, attente signal
    RETURNING = 3       # Suivi du chemin de retour
    COMPLETED = 4       # Mission terminée


class RealRobotMappingAuto(Node):
    def __init__(self):
        super().__init__('real_robot_mapping_auto')
        
        # État global
        self.state = RobotState.AUTONOME
        self.should_stop = False
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Paramètres retour
        self.return_path_m = []  # [(x, y), ...]
        self.return_path_idx = 0
        
        # Contrôle PID pour suivi chemin
        self.last_heading_error = 0.0
        self.return_kp = 1.8
        self.return_kd = 0.25
        self.return_lookahead_m = 0.3
        self.return_goal_tolerance_m = 0.15
        self.return_max_speed = 0.5
        
        # Compteurs
        self.return_attempts = 0
        self.max_return_attempts = 3
        
        # TF2 lookups
        self.robot_frame = "base_link"
        self.map_frame = "map"
        
        # Subscribers
        self.create_subscription(PathMsg, '/return_path', self.path_callback, 10)
        self.create_subscription(Bool, '/loop_closure/return_detected', 
                                 self.return_detected_callback, 10)
        self.create_subscription(Float32, '/loop_closure/distance_to_start',
                                 self.distance_callback, 10)
        
        # Publisher cmd_vel pour le retour
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel_return', 10)
        self.pub_state = self.create_publisher(Bool, '/mapping_completed', 10)
        
        # Timer pour contrôle principal
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("🤖 Real Robot Mapping Auto démarré")
        self.get_logger().info("   Phase 1: AUTONOME (suivi mur)")
        self.get_logger().info("   Phase 2: RETURNING (A* path following)")
        self.get_logger().info("   Phase 3: COMPLETED")
    
    def path_callback(self, msg):
        """Reçoit le chemin A* planifié depuis return_path_generator"""
        self.return_path_m = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self.return_path_idx = 0
        
        self.get_logger().info(f"✅ Chemin reçu: {len(self.return_path_m)} points")
    
    def return_detected_callback(self, msg):
        """Appelé quand boucle est détectée - prépare retour planifié"""
        if msg.data and self.state == RobotState.AUTONOME:
            self.state = RobotState.RETURN_PLANNED
            self.get_logger().info("🔄 État → RETURN_PLANNED (attente chemin A*)")
    
    def distance_callback(self, msg):
        """Reçoit distance au départ (info)"""
        if self.state == RobotState.RETURNING:
            pass  # Pourrait être utilisé pour ajustement fin
    
    def get_robot_pose_in_map(self):
        """
        Récupère pose robot dans frame map
        Retourne: (x_m, y_m, theta_rad) ou None
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time()
            )
            
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            
            q = tf.transform.rotation
            theta = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )
            
            return x, y, theta
        except Exception:
            return None
    
    def compute_desired_heading(self, robot_x_m, robot_y_m, target_x_m, target_y_m):
        """Calcule cap désirée vers cible"""
        dx = target_x_m - robot_x_m
        dy = target_y_m - robot_y_m
        desired_heading = math.atan2(dy, dx)
        return desired_heading
    
    def normalize_angle(self, angle):
        """Normalise angle en [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def pid_heading_control(self, heading_error):
        """Contrôle PID pour correction cap"""
        # Normaliser erreur
        heading_error = self.normalize_angle(heading_error)
        
        # PID
        p_term = self.return_kp * heading_error
        d_term = self.return_kd * (heading_error - self.last_heading_error)
        angular_cmd = p_term + d_term
        
        # Limiter
        angular_cmd = max(-0.3, min(0.3, angular_cmd))
        
        self.last_heading_error = heading_error
        return angular_cmd
    
    def distance_m(self, x1, y1, x2, y2):
        """Distance euclidienne"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def control_loop(self):
        """Boucle de contrôle principal"""
        
        if self.state == RobotState.AUTONOME:
            # wall_centering_node gère seul - rien à faire ici
            return
        
        elif self.state == RobotState.RETURN_PLANNED:
            # Attente que return_path_generator ait calculé le chemin
            if len(self.return_path_m) > 0:
                self.state = RobotState.RETURNING
                self.return_path_idx = 0
                self.last_heading_error = 0.0
                self.get_logger().info("🔄 État → RETURNING (suivi chemin A*)")
            return
        
        elif self.state == RobotState.RETURNING:
            pose = self.get_robot_pose_in_map()
            if pose is None:
                self.get_logger().warn("⚠️ Pose TF indisponible")
                return
            
            robot_x_m, robot_y_m, robot_theta = pose
            
            # Trouver le prochain waypoint en lookahead
            next_wp_idx = self.return_path_idx
            
            for i in range(self.return_path_idx, len(self.return_path_m)):
                wp_x, wp_y = self.return_path_m[i]
                dist = self.distance_m(robot_x_m, robot_y_m, wp_x, wp_y)
                
                if dist >= self.return_lookahead_m:
                    next_wp_idx = i
                    break
                elif i == len(self.return_path_m) - 1:
                    next_wp_idx = i
            
            target_x, target_y = self.return_path_m[next_wp_idx]
            
            # Distance au but final
            goal_x, goal_y = self.return_path_m[-1]
            dist_to_goal = self.distance_m(robot_x_m, robot_y_m, goal_x, goal_y)
            
            # Vérifier si but atteint
            if dist_to_goal < self.return_goal_tolerance_m:
                self.get_logger().info(f"✅ BUT ATTEINT! Distance: {dist_to_goal:.3f}m")
                self.complete_mission()
                return
            
            # Calculer cap désiré et l'erreur
            desired_heading = self.compute_desired_heading(
                robot_x_m, robot_y_m, target_x, target_y
            )
            heading_error = self.normalize_angle(desired_heading - robot_theta)
            
            # Contrôle PID
            angular_cmd = self.pid_heading_control(heading_error)
            
            # Vitesse linéaire (constant pour maintenant)
            linear_cmd = self.return_max_speed
            
            # Publier commande
            cmd = Twist()
            cmd.linear.x = linear_cmd
            cmd.angular.z = angular_cmd
            self.pub_cmd_vel.publish(cmd)
            
            # Debug tous les 10 pas (0.5s à 20Hz)
            if self.return_path_idx % 10 == 0:
                self.get_logger().debug(
                    f"🧭 WP {next_wp_idx}/{len(self.return_path_m)} "
                    f"Dist goal: {dist_to_goal:.2f}m "
                    f"Err: {heading_error:.2f}rad"
                )
            
            self.return_path_idx += 1
        
        elif self.state == RobotState.COMPLETED:
            # Stop total
            cmd = Twist()
            self.pub_cmd_vel.publish(cmd)
    
    def complete_mission(self):
        """Marque fin de mission"""
        self.state = RobotState.COMPLETED
        self.get_logger().info("\n✅ MISSION COMPLÉTÉE!")
        
        # Stop moteurs
        cmd = Twist()
        self.pub_cmd_vel.publish(cmd)
        
        # Publier signal
        msg = Bool()
        msg.data = True
        self.pub_state.publish(msg)
        
        # Sauvegarder logs
        self.save_mission_log()
    
    def save_mission_log(self):
        """Sauvegarde les logs de la mission"""
        try:
            log_dir = Path.home() / "mapping_logs"
            log_dir.mkdir(exist_ok=True)
            
            log_file = log_dir / f"mission_{int(time.time())}.json"
            
            log_data = {
                "timestamp": time.time(),
                "state": self.state.name,
                "return_path_waypoints": len(self.return_path_m),
                "return_attempts": self.return_attempts,
            }
            
            with open(log_file, 'w') as f:
                json.dump(log_data, f, indent=2)
            
            self.get_logger().info(f"💾 Log mission: {log_file}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erreur log: {e}")


def main(args=None):
    """
    Lance le nœud orchestrateur + les 3 nœuds subsidiaires
    """
    import subprocess
    import time
    
    # Récupérer répertoire courant
    pkg_path = Path(__file__).parent
    
    processes = []
    
    try:
        # Lancer wall_centering_node (autonomie)
        print("🚀 Lancement wall_centering_node...")
        p1 = subprocess.Popen(
            ['python3', str(pkg_path / 'wall_centering_node.py')],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        processes.append(p1)
        time.sleep(1)
        
        # Lancer loop_closure_detector
        print("🚀 Lancement loop_closure_detector...")
        p2 = subprocess.Popen(
            ['python3', str(pkg_path / 'loop_closure_detector.py')],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        processes.append(p2)
        time.sleep(0.5)
        
        # Lancer trajectory_saver
        print("🚀 Lancement trajectory_saver...")
        p3 = subprocess.Popen(
            ['python3', str(pkg_path / 'trajectory_saver.py')],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        processes.append(p3)
        time.sleep(0.5)
        
        # Lancer return_path_generator
        print("🚀 Lancement return_path_generator...")
        p4 = subprocess.Popen(
            ['python3', str(pkg_path / 'return_path_generator.py')],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        processes.append(p4)
        time.sleep(0.5)
        
        print("✅ Tous les nœuds lancés, démarrage orchestrateur...\n")
        
        # Lancer l'orchestrateur principal
        rclpy.init(args=args)
        node = RealRobotMappingAuto()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n⏹️  Arrêt détecté, fermeture...")
    finally:
        # Terminer tous les processus enfants
        for p in processes:
            try:
                p.terminate()
                p.wait(timeout=2)
            except Exception:
                p.kill()
        
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        
        print("✅ Arrêt complet")


if __name__ == '__main__':
    main()
