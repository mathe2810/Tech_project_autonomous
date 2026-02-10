#!/usr/bin/env python3
"""
Simple SLAM/Mapping node using LIDAR scans
Builds a local map by aligning consecutive scans with ICP (Iterative Closest Point)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
import time


class SimpleSLAM(Node):
    def __init__(self):
        super().__init__('simple_slam')
        
        # Parameters
        self.declare_parameter('grid_size', 5)  # 5x5 meters (bigger circuit)
        self.declare_parameter('resolution', 0.05)  # 5cm per cell (100x100 cells = 5x5m)
        self.declare_parameter('max_range', 12.0)  # LIDAR max range
        
        self.grid_size = self.get_parameter('grid_size').value
        self.resolution = self.get_parameter('resolution').value
        self.max_range = self.get_parameter('max_range').value
        
        # Internal state
        self.scan_history = []
        self.max_scans = 30  # Keep last 30 scans only (faster matching)
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.poses = [self.robot_pose.copy()]
        
        # Occupancy grid
        self.grid_width = int(self.grid_size / self.resolution)
        self.occupancy_grid = np.zeros((self.grid_width, self.grid_width), dtype=np.int8)
        
        # Publishers
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/slam/pose', 10)
        self.pub_scans = self.create_publisher(PointCloud2, '/slam/local_scans', 10)
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer pour publier la map à 10Hz
        self.create_timer(0.1, self.timer_callback)
        self.last_timestamp = None
        
        # Subscriber
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info(f'Simple SLAM node started (grid: {self.grid_size}m, res: {self.resolution}m)')
        
    def scan_callback(self, msg: LaserScan):
        """Process incoming LIDAR scan"""
        self.last_timestamp = msg.header.stamp  # Mémorise le timestamp
        
        # Convert polar coordinates to cartesian
        points = self.scan_to_points(msg)
        
        if len(self.scan_history) > 0:
            # Estimate motion using scan matching (ICP-like)
            prev_points = self.scan_history[-1]['points']
            dx, dy, dtheta = self.estimate_motion(prev_points, points)
            
            # Ignore tiny motions (noise threshold)
            if abs(dx) < 0.01 and abs(dy) < 0.01 and abs(dtheta) < 0.02:
                dx, dy, dtheta = 0, 0, 0
            
            # Update robot pose
            self.robot_pose[2] += dtheta
            cos_theta = np.cos(self.robot_pose[2])
            sin_theta = np.sin(self.robot_pose[2])
            self.robot_pose[0] += dx * cos_theta - dy * sin_theta
            self.robot_pose[1] += dx * sin_theta + dy * cos_theta
            
            self.poses.append(self.robot_pose.copy())
        
        # Store scan
        self.scan_history.append({
            'points': points,
            'time': msg.header.stamp,
            'pose': self.robot_pose.copy()
        })
        
        # Limit history size
        if len(self.scan_history) > self.max_scans:
            self.scan_history.pop(0)
        
        # Update occupancy grid with all scans
        self.update_occupancy_grid()
        
        # Timestamp will be published by timer at 10Hz
        
    def scan_to_points(self, scan: LaserScan):
        """Convert LaserScan message to 2D points (x, y)"""
        points = []
        for i, range_val in enumerate(scan.ranges):
            if np.isnan(range_val) or range_val < 0.1 or range_val > self.max_range:
                continue
            
            angle = scan.angle_min + i * scan.angle_increment
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            points.append([x, y])
        
        return np.array(points) if points else np.empty((0, 2))
    
    def estimate_motion(self, prev_points, curr_points, max_iterations=10):
        """
        Simple scan matching to estimate motion
        Uses nearest neighbor + SVD for rigid transformation
        """
        if len(prev_points) == 0 or len(curr_points) == 0:
            return 0.0, 0.0, 0.0
        
        dx, dy, dtheta = 0.0, 0.0, 0.0
        
        try:
            for iteration in range(max_iterations):
                # Find nearest neighbors
                from scipy.spatial.distance import cdist
                distances = cdist(prev_points, curr_points)
                nearest = np.argmin(distances, axis=1)
                
                matched_prev = prev_points
                matched_curr = curr_points[nearest]
                
                # Compute centroids
                centroid_prev = np.mean(matched_prev, axis=0)
                centroid_curr = np.mean(matched_curr, axis=0)
                
                # Center points
                centered_prev = matched_prev - centroid_prev
                centered_curr = matched_curr - centroid_curr
                
                # SVD for rotation + translation
                H = centered_prev.T @ centered_curr
                U, _, Vt = np.linalg.svd(H)
                R_matrix = Vt.T @ U.T
                
                # Ensure proper rotation
                if np.linalg.det(R_matrix) < 0:
                    Vt[-1, :] *= -1
                    R_matrix = Vt.T @ U.T
                
                # Extract angle
                theta = np.arctan2(R_matrix[1, 0], R_matrix[0, 0])
                
                # Translation
                t = centroid_curr - R_matrix @ centroid_prev
                
                dx, dy, dtheta = t[0], t[1], theta
                
                # Early exit if converged
                if abs(dtheta) < 0.001 and np.linalg.norm(t) < 0.01:
                    break
        except:
            pass  # Fallback to zero motion on error
        
        return dx, dy, dtheta
    
    def update_occupancy_grid(self):
        """Mark occupied cells in grid from all scans"""
        self.occupancy_grid.fill(0)  # Reset
        
        center_idx = self.grid_width // 2
        
        for scan_data in self.scan_history:
            points = scan_data['points']
            pose = scan_data['pose']
            
            # Transform points to world frame
            cos_theta = np.cos(pose[2])
            sin_theta = np.sin(pose[2])
            
            for point in points:
                # Rotate
                x = cos_theta * point[0] - sin_theta * point[1] + pose[0]
                y = sin_theta * point[0] + cos_theta * point[1] + pose[1]
                
                # Convert to grid coordinates
                grid_x = int((x / self.resolution) + center_idx)
                grid_y = int((y / self.resolution) + center_idx)
                
                if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_width:
                    self.occupancy_grid[grid_y, grid_x] = 100  # Occupied
    
    def publish_map(self, timestamp):
        """Publish occupancy grid"""
        msg = OccupancyGrid()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_width
        msg.info.origin.position.x = -(self.grid_width // 2) * self.resolution
        msg.info.origin.position.y = -(self.grid_width // 2) * self.resolution
        
        msg.data = self.occupancy_grid.flatten().tolist()
        self.pub_map.publish(msg)
    
    def publish_pose(self, timestamp):
        """Publish current pose"""
        msg = PoseStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self.robot_pose[0])
        msg.pose.position.y = float(self.robot_pose[1])
        
        # Quaternion from yaw angle
        quat = R.from_euler('z', self.robot_pose[2]).as_quat()
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.pub_pose.publish(msg)
    
    def publish_transforms(self, timestamp):
        """Publish map -> odom transform"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        t.transform.translation.x = float(self.robot_pose[0])
        t.transform.translation.y = float(self.robot_pose[1])
        t.transform.translation.z = 0.0
        
        quat = R.from_euler('z', self.robot_pose[2]).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        """Publie la map à 10Hz"""
        if self.last_timestamp is not None:
            self.publish_map(self.last_timestamp)
            self.publish_pose(self.last_timestamp)
            self.publish_transforms(self.last_timestamp)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSLAM()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
