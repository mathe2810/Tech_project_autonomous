# 🎯 Exemples ROS2 - Recevoir et Traiter les Données LIDAR

## 📦 Installation Dépendances

```bash
sudo apt-get install python3-colcon-common-extensions
```

## 1️⃣ Subscriber Basique - Écouter /scan

### Python Subscriber

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10  # QoS
        )
        self.get_logger().info("LIDAR Subscriber started")
    
    def lidar_callback(self, msg: LaserScan):
        """Callback appelé à chaque scan reçu"""
        
        # Extraire métadonnées
        self.get_logger().info(f"""
        ────────────────────────────
        Frame: {msg.header.frame_id}
        Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}
        Angle Min: {math.degrees(msg.angle_min):.1f}°
        Angle Max: {math.degrees(msg.angle_max):.1f}°
        Angle Inc: {math.degrees(msg.angle_increment):.2f}°
        Range Min: {msg.range_min:.2f}m
        Range Max: {msg.range_max:.2f}m
        Points: {len(msg.ranges)}
        ────────────────────────────
        """)
        
        # Trouver obstacle le plus proche
        closest_range = float('inf')
        closest_angle = 0.0
        
        for i, range_val in enumerate(msg.ranges):
            if msg.range_min < range_val < msg.range_max:
                if range_val < closest_range:
                    closest_range = range_val
                    angle = msg.angle_min + i * msg.angle_increment
                    closest_angle = math.degrees(angle)
        
        if closest_range != float('inf'):
            self.get_logger().info(
                f"Obstacle closest: {closest_range:.2f}m @ {closest_angle:.1f}°"
            )

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage:**
```bash
# Terminal 1: Lancer le subscriber
python3 lidar_subscriber.py

# Output:
# ────────────────────────────
# Frame: lidar_link
# Timestamp: 5.234000000
# Angle Min: 0.0°
# Angle Max: 360.0°
# Obstacle closest: 0.32m @ 45.2°
# ────────────────────────────
```

## 2️⃣ Détection d'Obstacles - Safety Zone

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Parameters
        self.declare_parameter('safety_distance', 0.5)  # 50cm
        self.declare_parameter('front_angle_range', 30)  # ±15° devant
        
        self.safety_dist = self.get_parameter('safety_distance').value
        self.angle_range = self.get_parameter('front_angle_range').value
        
        # Subscriber & Publisher
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        
        self.obstacle_detected = False
        self.get_logger().info(
            f"Obstacle Detector: safety_dist={self.safety_dist}m, "
            f"angle_range=±{self.angle_range//2}°"
        )
    
    def scan_cb(self, msg: LaserScan):
        """Détecter obstacles dans la zone de sécurité"""
        
        obstacle_found = False
        
        # Vérifier cône frontal (0° ± 15°)
        front_angles = [
            (i, msg.angle_min + i * msg.angle_increment)
            for i in range(len(msg.ranges))
        ]
        
        for idx, angle in front_angles:
            angle_deg = math.degrees(angle)
            
            # Normaliser angle à ±180°
            if angle_deg > 180:
                angle_deg -= 360
            
            # Vérifier si dans cône frontal
            if abs(angle_deg) <= (self.angle_range / 2):
                range_val = msg.ranges[idx]
                
                # Vérifier si en zone de sécurité
                if msg.range_min <= range_val <= self.safety_dist:
                    obstacle_found = True
                    self.get_logger().warn(
                        f"🚨 OBSTACLE DÉTECTÉ: {range_val:.2f}m @ {angle_deg:.1f}°"
                    )
                    break
        
        # Publier résultat
        if obstacle_found != self.obstacle_detected:
            self.obstacle_detected = obstacle_found
            msg_bool = Bool()
            msg_bool.data = obstacle_found
            self.pub.publish(msg_bool)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage:**
```bash
python3 obstacle_detector.py

# Écouter résultat dans autre terminal
ros2 topic echo /obstacle_detected
```

## 3️⃣ Traitement Avancé - Clustering Points

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import math
from collections import defaultdict

class LidarClustering(Node):
    def __init__(self):
        super().__init__('lidar_clustering')
        
        self.sub = self.create_subscription(LaserScan, '/scan', self.process, 10)
        self.pub = self.create_publisher(MarkerArray, '/obstacles', 10)
        
        self.cluster_distance = 0.1  # 10cm entre clusters
        self.get_logger().info("LIDAR Clustering Node started")
    
    def process(self, msg: LaserScan):
        """Grouper points proches en clusters"""
        
        # Convertir scan en coordonnées XY
        points = []
        for i, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append((x, y, r))
        
        # Clustering naïf (DBSCAN simplifié)
        clusters = self.cluster_points(points)
        
        # Créer markers
        markers = MarkerArray()
        for idx, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = "lidar_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Centroïde du cluster
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0  # Red
            
            markers.markers.append(marker)
        
        self.pub.publish(markers)
        self.get_logger().info(f"Found {len(clusters)} clusters")
    
    def cluster_points(self, points):
        """Clustering simple par distance"""
        if not points:
            return []
        
        clusters = []
        visited = set()
        
        for i, p1 in enumerate(points):
            if i in visited:
                continue
            
            cluster = [p1]
            visited.add(i)
            
            for j in range(i+1, len(points)):
                if j in visited:
                    continue
                
                p2 = points[j]
                dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
                
                if dist < self.cluster_distance:
                    cluster.append(p2)
                    visited.add(j)
            
            if len(cluster) > 1:  # Filtrer singleton
                clusters.append(cluster)
        
        return clusters

def main(args=None):
    rclpy.init(args=args)
    node = LidarClustering()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4️⃣ Visualisation RViz

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
import math

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        
        self.sub = self.create_subscription(LaserScan, '/scan', self.visualize, 10)
        self.pub = self.create_publisher(MarkerArray, '/scan_visualization', 10)
    
    def visualize(self, msg: LaserScan):
        """Afficher points LIDAR en RViz"""
        
        markers = MarkerArray()
        
        # Marker pour chaque point
        for i, r in enumerate(msg.ranges):
            if msg.range_min <= r <= msg.range_max:
                marker = Marker()
                marker.header.frame_id = msg.header.frame_id
                marker.header.stamp = msg.header.stamp
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                # Position en coordonnées polaires → cartésiennes
                angle = msg.angle_min + i * msg.angle_increment
                marker.pose.position.x = r * math.cos(angle)
                marker.pose.position.y = r * math.sin(angle)
                marker.pose.position.z = 0.0
                
                # Petite sphère
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                
                # Couleur: gradient proche→loin
                intensity = (r - msg.range_min) / (msg.range_max - msg.range_min)
                marker.color.r = intensity
                marker.color.g = 1.0 - intensity
                marker.color.b = 0.0
                marker.color.a = 0.8
                
                markers.markers.append(marker)
        
        self.pub.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**RViz Setup:**
```
1. Ouvrir RViz: rviz2
2. Fixed Frame: lidar_link
3. Add → Marker Array
4. Topic: /scan_visualization
```

## 📋 Créer un Package ROS2

```bash
# Créer workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws

# Créer package
ros2 pkg create --build-type ament_python lidar_processing

# Copier les scripts dans src/lidar_processing/lidar_processing/

# Build
colcon build

# Source
source install/setup.bash

# Lancer
python3 -m lidar_processing.lidar_subscriber
```

## 🔗 Topics ROS2

```bash
# Lister tous les topics
ros2 topic list

# Info détaillée
ros2 topic info /scan

# Fréquence de publication
ros2 topic hz /scan

# BW utilisé
ros2 topic bw /scan
```

---

**Auteur**: GitHub Copilot  
**Date**: 2026-01-19  
**Version**: 1.0.0
