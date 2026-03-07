#!/usr/bin/env python3
"""
Enregistre la carte SLAM en PNG pour debug/visualisation
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image
import os
import time

class SlamMapSaver(Node):
    def __init__(self):
        super().__init__('slam_map_saver')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.map_callback, 
            qos
        )
        
        self.last_save_time = 0
        self.save_interval = 2.0  # Sauvegarder toutes les 2 secondes
        self.output_path = os.path.join(os.getcwd(), 'slam_map_debug.png')
        self.frame_count = 0
        
        self.get_logger().info(f'🗺️  SLAM Map Saver démarré -> {self.output_path}')
        
    def map_callback(self, msg):
        """Converti la carte OccupancyGrid en PNG"""
        self.frame_count += 1
        current_time = time.time()
        
        # Debug: log reception (once per 5 callbacks)
        if self.frame_count % 5 == 0:
            self.get_logger().info(f'📍 Reçu message /map #{self.frame_count}: {msg.info.width}x{msg.info.height}px')
        
        # Sauvegarder seulement si interval écoulé
        if current_time - self.last_save_time < self.save_interval:
            return
            
        try:
            self.last_save_time = current_time
            
            # Récupérer les données
            width = msg.info.width
            height = msg.info.height
            data = np.array(msg.data, dtype=np.uint8).reshape((height, width))
            
            # Convertir occupancy grid (-1=inconnue, 0=libre, 100=occupé)
            # en image RGB (0-255)
            img_array = np.zeros((height, width, 3), dtype=np.uint8)
            
            # Blanc = zone libre (0)
            img_array[data == 0] = [255, 255, 255]
            
            # Noir = occupé (100)
            img_array[data == 100] = [0, 0, 0]
            
            # Gris = inconnu (-1)
            img_array[data == -1] = [128, 128, 128]
            
            # Pour les valeurs intermédiaires (détection de parois partielles)
            mask_intermediate = (data > 0) & (data < 100)
            intermediate_val = (100 - data[mask_intermediate]) * 2.55
            img_array[mask_intermediate] = intermediate_val[:, np.newaxis]
            
            # Inverser Y (ROS: origine en bas-gauche, image: origine en haut-gauche)
            img_array = np.flipud(img_array)
            
            # Sauvegarder
            img = Image.fromarray(img_array, 'RGB')
            img.save(self.output_path)
            
            # Log
            self.get_logger().info(
                f'✅ [#{self.frame_count}] Carte sauvegardée: {width}x{height}px '
                f'(résolution: {msg.info.resolution:.3f}m/px) '
                f'-> {self.output_path}'
            )
            
        except Exception as e:
            self.get_logger().error(f'❌ Erreur sauvegarde PNG: {e}')

def main():
    try:
        rclpy.init()
    except:
        pass
    
    node = SlamMapSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
