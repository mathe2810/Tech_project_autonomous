#!/usr/bin/env python3
"""
LOOP CLOSURE DETECTOR - Détecte quand le robot revient au point de départ

Écoute la pose du robot via TF2 et compare la distance avec le point de départ.
Émet un signal quand la boucle est complétée.

Paramètres clés:
- RETURN_DETECTION_DISTANCE: Seuil de distance pour détecter retour au départ
- MIN_DISTANCE_TRAVELED: Distance minimale avant de permettre la détection
- LEAVE_START_DISTANCE: Distance minimale avant de quitter la zone de départ
- LAP_SAMPLE_DISTANCE: Intervalle d'échantillonnage de la trajectoire
"""

import os
import sys
import math
import time
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Empty

# --- PARAMÈTRES LOOP CLOSURE ---
RETURN_DETECTION_DISTANCE = 0.20      # Distance tolérance (m) pour détecter retour
MIN_DISTANCE_TRAVELED = 3.0           # Distance min avant de permettre détection (m)
LEAVE_START_DISTANCE = 0.60           # Distance pour quitter zone départ (m)
EXTRA_DISTANCE_AFTER_RETURN = 2.0     # Distance à parcourir APRÈS retour (m)
LAP_SAMPLE_DISTANCE = 0.08            # Intervalle échantillonnage trajectoire (m)
SLAM_MAP_FRAME = "map"
ROBOT_FRAME = "base_link"


class LoopClosureDetector(Node):
    def __init__(self):
        super().__init__('loop_closure_detector')
        
        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # État loop closure
        self.start_pose_x = None
        self.start_pose_y = None
        self.max_distance_reached = 0.0
        self.has_left_start_zone = False
        self.loop_return_detected = False
        self.extra_distance_after_return = 0.0
        self.last_robot_pos = None
        self.last_lap_sample_pos = None
        
        # Trajectoire enregistrée
        self.lap_points_m = []
        self.lap_recording_started = False
        self.lap_recording_done = False
        
        # Publishers
        self.pub_return_detected = self.create_publisher(Bool, '/loop_closure/return_detected', 10)
        self.pub_distance_to_start = self.create_publisher(Float32, '/loop_closure/distance_to_start', 10)
        self.pub_trajectory = self.create_publisher(PoseStamped, '/loop_closure/trajectory_point', 10)
        
        # Timer pour vérifier la pose régulièrement
        self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("🔍 Loop Closure Detector démarré")
    
    def get_robot_pose_in_map(self):
        """
        Récupère la pose du robot dans le frame map via TF2
        Retourne: (x_m, y_m, theta_rad) ou None si TF non disponible
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                SLAM_MAP_FRAME,
                ROBOT_FRAME,
                Time()
            )
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Extraire angle theta du quaternion
            q = transform.transform.rotation
            theta = math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z)
            )
            
            return x, y, theta
        except Exception as e:
            return None
    
    def distance_m(self, x1_m, y1_m, x2_m, y2_m):
        """Calcule distance euclidienne en mètres"""
        dx = x2_m - x1_m
        dy = y2_m - y1_m
        return math.sqrt(dx*dx + dy*dy)
    
    def timer_callback(self):
        """Vérification périodique (100ms) de la pose et détection de boucle"""
        
        # Récupérer pose robot
        pose = self.get_robot_pose_in_map()
        if pose is None:
            return
        
        robot_x_m, robot_y_m, robot_theta = pose
        
        # Initialiser point de départ lors du premier appel
        if self.start_pose_x is None:
            self.start_pose_x = robot_x_m
            self.start_pose_y = robot_y_m
            self.last_robot_pos = (robot_x_m, robot_y_m)
            self.get_logger().info(f"📍 Point de départ configuré: ({robot_x_m:.2f}, {robot_y_m:.2f})")
            return
        
        # Distances
        dist_to_start = self.distance_m(
            robot_x_m, robot_y_m,
            self.start_pose_x, self.start_pose_y
        )
        
        # Publier distance au départ
        dist_msg = Float32()
        dist_msg.data = dist_to_start
        self.pub_distance_to_start.publish(dist_msg)
        
        # Calcul distance parcourue depuis dernière pose
        dx = robot_x_m - self.last_robot_pos[0]
        dy = robot_y_m - self.last_robot_pos[1]
        step_m = math.sqrt(dx*dx + dy*dy)
        self.last_robot_pos = (robot_x_m, robot_y_m)
        
        # Mettre à jour distance max atteinte
        if dist_to_start > self.max_distance_reached:
            self.max_distance_reached = dist_to_start
        
        # === LOGIQUE ENREGISTREMENT & DÉTECTION ===
        
        # Phase 1: Zone de départ - attend que robot s'éloigne
        if not self.has_left_start_zone and dist_to_start > LEAVE_START_DISTANCE:
            self.has_left_start_zone = True
            self.get_logger().info("✅ Robot a quitté zone de départ, commence enregistrement")
        
        # Phase 2: Début enregistrement trajectoire
        if self.has_left_start_zone and not self.lap_recording_started:
            self.lap_recording_started = True
            self.lap_points_m.clear()
            self.last_lap_sample_pos = None
            self.get_logger().info("📝 Début enregistrement trajectoire du tour")
        
        # Phase 3: Échantillonnage trajectoire
        if self.lap_recording_started and not self.lap_recording_done:
            # Sample tous les LAP_SAMPLE_DISTANCE mètres
            if (self.last_lap_sample_pos is None or 
                self.distance_m(
                    robot_x_m, robot_y_m,
                    self.last_lap_sample_pos[0], self.last_lap_sample_pos[1]
                ) >= LAP_SAMPLE_DISTANCE):
                
                self.lap_points_m.append((robot_x_m, robot_y_m))
                self.last_lap_sample_pos = (robot_x_m, robot_y_m)
                
                # Publier le point
                msg = PoseStamped()
                msg.header.frame_id = SLAM_MAP_FRAME
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.position.x = robot_x_m
                msg.pose.position.y = robot_y_m
                msg.pose.position.z = 0.0
                self.pub_trajectory.publish(msg)
        
        # Phase 4: Détection du retour au départ
        if (not self.loop_return_detected and 
            self.has_left_start_zone and 
            self.max_distance_reached > MIN_DISTANCE_TRAVELED and 
            dist_to_start < RETURN_DETECTION_DISTANCE):
            
            self.loop_return_detected = True
            self.extra_distance_after_return = 0.0
            self.lap_recording_done = True
            
            self.get_logger().info(f"\n🎯 RETOUR AU DÉPART DÉTECTÉ!")
            self.get_logger().info(f"   Distance: {dist_to_start:.2f}m")
            self.get_logger().info(f"   Max atteint: {self.max_distance_reached:.2f}m")
            self.get_logger().info(f"   Trajectoire capturée: {len(self.lap_points_m)} points")
            self.get_logger().info(f"➡️  Continuant {EXTRA_DISTANCE_AFTER_RETURN:.2f}m pour compléter la carte...")
            
            # Publier signal de détection
            ret_msg = Bool()
            ret_msg.data = True
            self.pub_return_detected.publish(ret_msg)
        
        # Phase 5: Enregistrement distance après retour
        elif self.loop_return_detected:
            self.extra_distance_after_return += step_m
            
            if self.extra_distance_after_return >= EXTRA_DISTANCE_AFTER_RETURN:
                self.get_logger().info(f"✅ DISTANCE SUPPLÉMENTAIRE ATTEINTE: {self.extra_distance_after_return:.2f}m")
                self.get_logger().info("🧭 Fin mapping automatique")
                self.save_trajectory()
    
    def save_trajectory(self):
        """Sauvegarde la trajectoire enregistrée dans un fichier"""
        if not self.lap_points_m:
            self.get_logger().warn("⚠️ Aucun point de trajectoire à sauvegarder")
            return
        
        try:
            # Créer dossier trajectories s'il n'existe pas
            traj_dir = Path(os.path.dirname(__file__)) / "trajectories"
            traj_dir.mkdir(exist_ok=True)
            
            # Sauvegarder en JSON
            timestamp = int(time.time())
            traj_file = traj_dir / f"lap_trajectory_{timestamp}.json"
            
            data = {
                "timestamp": timestamp,
                "num_points": len(self.lap_points_m),
                "start_pose": [self.start_pose_x, self.start_pose_y],
                "max_distance": self.max_distance_reached,
                "points": self.lap_points_m  # [(x, y), ...]
            }
            
            with open(traj_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.get_logger().info(f"💾 Trajectoire sauvegardée: {traj_file}")
            
            # Aussi sauvegarder en numpy pour accès rapide
            import numpy as np
            npy_file = traj_dir / f"lap_trajectory_{timestamp}.npy"
            np.save(npy_file, np.array(self.lap_points_m))
            self.get_logger().info(f"💾 Format NPY: {npy_file}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erreur sauvegarde trajectoire: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LoopClosureDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
