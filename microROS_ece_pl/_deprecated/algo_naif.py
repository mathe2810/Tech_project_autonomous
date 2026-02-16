#!/usr/bin/env python3
"""
Algorithme NAIF pour améliorer le mapping avec rotations sans odométrie.

Le principe: on détecte les rotations via le LIDAR lui-même.
- On sauvegarde une "signature" du scan (angles des pics de murs)
- On tourne jusqu'à retrouver cette signature rotée
"""

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np
import time


def extract_peaks(ranges, angle_increment, angle_min, threshold=0.5):
    """
    Extrait les positions angulaires des murs (pics de faible distance).
    Retourne une liste d'angles en radians où on détecte des murs.
    """
    peaks = []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and r < threshold:
            angle = angle_min + i * angle_increment
            peaks.append(angle)
    return peaks


def normalize_angle(angle):
    """Ramène un angle dans [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def angle_diff(angle1, angle2):
    """Différence d'angle normalisée"""
    diff = normalize_angle(angle1 - angle2)
    return diff


def estimate_rotation_from_scans(scan_old, scan_new, max_rotation=math.pi/2):
    """
    ALGO NAIF: Compare deux scans pour estimer la rotation.
    
    Stratégie:
    1) Extraire les pics de murs dans chaque scan
    2) Chercher le décalage angulaire qui aligne les pics
    3) Cet angle = la rotation effectuée
    """
    peaks_old = extract_peaks(
        scan_old.ranges, 
        scan_old.angle_increment, 
        scan_old.angle_min
    )
    peaks_new = extract_peaks(
        scan_new.ranges,
        scan_new.angle_increment,
        scan_new.angle_min
    )
    
    if len(peaks_old) < 3 or len(peaks_new) < 3:
        return None  # Pas assez de features
    
    # Convertir en numpy pour croiser les angles
    peaks_old_arr = np.array(peaks_old)
    peaks_new_arr = np.array(peaks_new)
    
    # Essayer différents décalages et voir lequel minimise l'erreur
    best_rotation = 0.0
    best_error = float('inf')
    
    # Tester des rotations de -45° à +45° par pas de 2°
    for rot_deg in np.linspace(-45, 45, 45):
        rot = math.radians(rot_deg)
        
        # Appliquer la rotation hypothétique aux anciens pics
        rotated_old = peaks_old_arr + rot
        
        # Calculer la distance minimale pour chaque pic ancien vers les nouveaux
        errors = []
        for angle_old in rotated_old:
            # Trouver le pic nouveau le plus proche
            min_dist = min([abs(angle_diff(angle_old, angle_new)) 
                           for angle_new in peaks_new_arr])
            errors.append(min_dist)
        
        avg_error = np.mean(errors)
        
        if avg_error < best_error:
            best_error = avg_error
            best_rotation = rot
    
    # Retourner la rotation seulement si confiance suffisante
    if best_error < math.radians(10):  # Erreur < 10°
        return best_rotation
    
    return None


class NaiveRotationEstimator:
    """
    Classe pour tracker les rotations du robot via LIDAR scanning.
    """
    
    def __init__(self):
        self.last_scan = None
        self.estimated_heading = 0.0  # En radians
        self.confidence = 0.0
        
    def update(self, scan):
        """
        Traite un nouveau scan et estime la rotation.
        Retourne l'angle de rotation estimé (radians).
        """
        if self.last_scan is None:
            self.last_scan = scan
            return 0.0
        
        rotation = estimate_rotation_from_scans(self.last_scan, scan)
        
        if rotation is not None:
            self.estimated_heading = normalize_angle(self.estimated_heading + rotation)
            self.confidence = 0.9
            self.last_scan = scan
            return rotation
        else:
            # Perte de suivi, confiance baisse
            self.confidence *= 0.95
            return 0.0


def rotate_until_target(node, cmd_vel_pub, estimator, target_rotation_deg, tolerance_deg=5):
    """
    Algorithme NAIF pour effectuer une rotation.
    
    Au lieu de tourner "pendant X secondes", on tourne jusqu'à ce que
    le LIDAR détecte qu'on a tourné l'angle cible.
    """
    target_rad = math.radians(target_rotation_deg)
    tolerance_rad = math.radians(tolerance_deg)
    
    start_heading = estimator.estimated_heading
    target_heading = normalize_angle(start_heading + target_rad)
    
    print(f"🔄 Rotation naïve: cible {target_rotation_deg}°")
    
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.3 if target_rotation_deg > 0 else -0.3
    
    start_time = time.time()
    max_time = abs(target_rotation_deg) / 20.0 + 5.0  # Timeout
    
    while rclpy.ok() and (time.time() - start_time) < max_time:
        rclpy.spin_once(node, timeout_sec=0.05)
        
        # Vérifier si on a atteint la cible
        current_diff = normalize_angle(estimator.estimated_heading - target_heading)
        
        if abs(current_diff) < tolerance_rad:
            print(f"✓ Rotation atteinte! Heading: {math.degrees(estimator.estimated_heading):.1f}°")
            break
        
        cmd_vel_pub.publish(twist)
        time.sleep(0.05)
    
    # Arrêter
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)


def main():
    rclpy.init()
    node = rclpy.create_node('naif_rotation_mapper')
    
    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    estimator = NaiveRotationEstimator()
    
    def scan_cb(scan):
        rot = estimator.update(scan)
        if rot is not None:
            heading_deg = math.degrees(estimator.estimated_heading)
            print(f"  Rotation détectée: {math.degrees(rot):+.1f}° | "
                  f"Heading: {heading_deg:.1f}° | Confiance: {estimator.confidence:.1%}")
    
    node.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data)
    
    print("""
    ╔════════════════════════════════════════╗
    │   NAÏVE LIDAR-BASED ROTATION MAPPER   │
    ╠════════════════════════════════════════╣
    │  Les rotations sont trackées via LIDAR │
    │  au lieu de l'odométrie.              │
    ╚════════════════════════════════════════╝
    """)
    
    try:
        # Laisser le LIDAR se calibrer
        print("Calibration LIDAR...")
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.05)
        
        # TEST: Effectuer une rotation de 90°
        print("\n→ Rotation test 1: +90°")
        rotate_until_target(node, cmd_vel_pub, estimator, 90.0)
        time.sleep(1)
        
        print("\n← Rotation test 2: -180°")
        rotate_until_target(node, cmd_vel_pub, estimator, -180.0)
        time.sleep(1)
        
        print("\n→ Rotation test 3: +90° (retour)")
        rotate_until_target(node, cmd_vel_pub, estimator, 90.0)
        
        print("\n✓ Tests terminés")
        
    except KeyboardInterrupt:
        print("\nArrêt...")
        twist = Twist()
        cmd_vel_pub.publish(twist)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
