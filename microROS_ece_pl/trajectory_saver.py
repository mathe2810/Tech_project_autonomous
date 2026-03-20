#!/usr/bin/env python3
"""
TRAJECTORY SAVER - Enregistre et gère la trajectoire réelle du robot

Reçoit les points de trajectoire du loop_closure_detector et les sauvegarde
avec options d'optimisation et de conversion.
"""

import json
import numpy as np
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class TrajectorySaver(Node):
    def __init__(self):
        super().__init__('trajectory_saver')
        
        self.trajectory_points = []
        self.is_recording = False
        self.mapping_completed = False
        
        # Subscribers
        self.create_subscription(
            PoseStamped, 
            '/loop_closure/trajectory_point',
            self.trajectory_point_callback,
            10
        )
        self.create_subscription(
            Bool,
            '/loop_closure/return_detected',
            self.return_detected_callback,
            10
        )
        
        # Créer dossier trajectories
        self.traj_dir = Path.home() / "trajectories" / "auto_mapping"
        self.traj_dir.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info("💾 Trajectory Saver démarré")
    
    def trajectory_point_callback(self, msg):
        """Reçoit et enregistre un point de trajectoire"""
        if not self.mapping_completed:
            point = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]
            self.trajectory_points.append(point)
            
            if len(self.trajectory_points) % 10 == 0:  # Log tous les 10 points
                self.get_logger().debug(f"📍 Point {len(self.trajectory_points)} enregistré")
    
    def return_detected_callback(self, msg):
        """Appelé quand le retour au départ est détecté"""
        if msg.data and not self.mapping_completed:
            self.is_recording = True
            self.get_logger().info(f"✅ Return détecté - {len(self.trajectory_points)} points enregistrés")
    
    def save_all_formats(self, prefix=""):
        """Sauvegarde la trajectoire dans tous les formats utiles"""
        if not self.trajectory_points:
            self.get_logger().warn("⚠️ Aucun point à sauvegarder")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"trajectory_{timestamp}_{prefix}" if prefix else f"trajectory_{timestamp}"
        
        try:
            # 1. Format JSON (lisible, portable)
            json_file = self.traj_dir / f"{filename}.json"
            json_data = {
                "timestamp": timestamp,
                "num_points": len(self.trajectory_points),
                "points": [
                    {"x": p[0], "y": p[1], "z": p[2]} 
                    for p in self.trajectory_points
                ]
            }
            with open(json_file, 'w') as f:
                json.dump(json_data, f, indent=2)
            self.get_logger().info(f"✅ Sauvegarde JSON: {json_file}")
            
            # 2. Format NumPy (accès rapide, calculs)
            npy_file = self.traj_dir / f"{filename}.npy"
            np.save(npy_file, np.array(self.trajectory_points))
            self.get_logger().info(f"✅ Sauvegarde NumPy: {npy_file}")
            
            # 3. Format CSV (compatibilité outils externes)
            csv_file = self.traj_dir / f"{filename}.csv"
            np.savetxt(
                csv_file,
                self.trajectory_points,
                delimiter=',',
                header='x_m,y_m,z_m',
                comments=''
            )
            self.get_logger().info(f"✅ Sauvegarde CSV: {csv_file}")
            
            # 4. Statistiques
            arr = np.array(self.trajectory_points)
            stats = {
                "num_points": len(self.trajectory_points),
                "x_range": [float(arr[:, 0].min()), float(arr[:, 0].max())],
                "y_range": [float(arr[:, 1].min()), float(arr[:, 1].max())],
                "total_length_m": float(self._calculate_path_length(arr)),
            }
            
            stats_file = self.traj_dir / f"{filename}_stats.json"
            with open(stats_file, 'w') as f:
                json.dump(stats, f, indent=2)
            self.get_logger().info(f"📊 Statistiques: {stats_file}")
            
            self.mapping_completed = True
            
        except Exception as e:
            self.get_logger().error(f"❌ Erreur sauvegarde: {e}")
    
    @staticmethod
    def _calculate_path_length(points):
        """Calcule la longueur totale du chemin"""
        if len(points) < 2:
            return 0.0
        
        diffs = np.diff(points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        return float(np.sum(distances))
    
    def optimize_trajectory(self, max_distance_between_samples=0.15):
        """
        Optimise le chemin en supprimant points redondants/bruyants
        """
        if len(self.trajectory_points) < 3:
            return self.trajectory_points
        
        arr = np.array(self.trajectory_points)
        optimized = [arr[0].tolist()]
        
        for i in range(1, len(arr)):
            dist_to_last = np.linalg.norm(arr[i] - np.array(optimized[-1]))
            if dist_to_last >= max_distance_between_samples:
                optimized.append(arr[i].tolist())
        
        self.get_logger().info(
            f"🔧 Trajectoire optimisée: {len(self.trajectory_points)} → {len(optimized)} points"
        )
        return optimized


def main(args=None):
    rclpy.init(args=args)
    node = TrajectorySaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Sauvegarder avant de quitter
        node.save_all_formats("final")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
