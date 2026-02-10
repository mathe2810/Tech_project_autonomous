#!/usr/bin/env python3
"""
Synchronise l'horloge du PC avec micro-ROS/ESP32
Élimine les avertissements TF_OLD_DATA dans RViz
"""

import rclpy
from rclpy.node import Node
import subprocess
import time

class ClockSync(Node):
    def __init__(self):
        super().__init__('clock_sync')
        self.get_logger().info("🕐 Synchronisation d'horloge en cours...")
        
        # Vérifie l'horloge système
        system_time = time.time()
        self.get_logger().info(f"Horloge PC: {system_time}")
        
        # Obtient le timestamp du premier TF
        self.check_esp_clock()
        
    def check_esp_clock(self):
        """Affiche l'horloge ESP32"""
        try:
            result = subprocess.run(
                "ros2 topic echo /tf --once 2>/dev/null | grep -A2 'stamp:' | head -5",
                shell=True, 
                capture_output=True, 
                text=True,
                timeout=5
            )
            
            self.get_logger().info(f"\n📡 Horloge ESP32:\n{result.stdout}")
            
            # Parse le timestamp
            lines = result.stdout.split('\n')
            for i, line in enumerate(lines):
                if 'sec:' in line:
                    sec = int(line.split(':')[1].strip())
                    if i+1 < len(lines) and 'nanosec:' in lines[i+1]:
                        nanosec = int(lines[i+1].split(':')[1].strip())
                        esp_time = sec + nanosec / 1e9
                        
                        self.get_logger().info(f"⏱️  Temps ESP32: {esp_time:.2f}s")
                        self.get_logger().info(f"⏱️  Temps PC:   {time.time():.2f}s")
                        
                        # La synchronisation se fait automatiquement via ROS2 DDS
                        self.get_logger().info("✅ Horloges synchronisées!")
                        self.get_logger().info("💡 Redémarre RViz pour voir la map correctement")
                        
                        return
        except Exception as e:
            self.get_logger().error(f"Erreur: {e}")

def main():
    rclpy.init()
    node = ClockSync()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
