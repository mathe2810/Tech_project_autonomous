#!/usr/bin/env python3
"""
Simple SLAM/Mapping node using LIDAR scans
Builds a local map by aligning consecutive scans with ICP (Iterative Closest Point)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid, Odometry
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
        self.declare_parameter('enable_icp_correction', True)  # Correct odometry with ICP
        
        self.grid_size = self.get_parameter('grid_size').value
        self.resolution = self.get_parameter('resolution').value
        self.max_range = self.get_parameter('max_range').value
        self.enable_icp_correction = self.get_parameter('enable_icp_correction').value
        
        # Internal state
        self.scan_history = []
        self.max_scans = 30  # Keep last 30 scans only (faster matching)
        self.odom_pose = np.array([0.0, 0.0, 0.0])  # Raw motor odometry (x, y, theta)
        self.corrected_pose = np.array([0.0, 0.0, 0.0])  # ICP-corrected pose
        self.pose_correction = np.array([0.0, 0.0, 0.0])  # Correction applied
        
        # Occupancy grid
        self.grid_width = int(self.grid_size / self.resolution)
        self.occupancy_grid = np.zeros((self.grid_width, self.grid_width), dtype=np.int8)
        
        # Publishers
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/slam/pose', 10)
        self.pub_odom_corrected = self.create_publisher(Odometry, '/odom/corrected', 10)
        self.pub_scans = self.create_publisher(PointCloud2, '/slam/local_scans', 10)
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer pour publier la map à 10Hz
        self.create_timer(0.1, self.timer_callback)
        self.last_timestamp = None
        
        # Odometry subscriber - use raw motor odometry for pose
        self.last_odom_pose = None
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Subscriber
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info(f'Simple SLAM node started (grid: {self.grid_size}m, res: {self.resolution}m)')
        
    def odom_callback(self, msg: Odometry):
        """
        Receive raw motor odometry.
        This is the raw pose from motor_odom_node (with accumulated drift).
        SLAM will correct it using ICP scan matching.
        """
        # Extract raw pose from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        yaw = rot.as_euler('xyz')[2]
        
        # Store raw odometry pose
        self.odom_pose = np.array([x, y, yaw])
        
        # Use corrected pose for map (if ICP correction is enabled)
        # Otherwise fall back to raw odometry
        if self.enable_icp_correction:
            # The corrected pose is calculated from ICP on scans
            # It starts as raw odom and gets adjusted by ICP
            pass
        else:
            self.corrected_pose = self.odom_pose.copy()
        
    def scan_callback(self, msg: LaserScan):
        """Process incoming LIDAR scan"""
        self.last_timestamp = msg.header.stamp
        
        # Convert polar coordinates to cartesian
        points = self.scan_to_points(msg)
        
        if len(self.scan_history) > 0 and self.enable_icp_correction:
            # Estimate motion using scan matching (ICP)
            # Compare current scan to previous scan to correct odometry drift
            prev_points = self.scan_history[-1]['points']
            dx_icp, dy_icp, dtheta_icp = self.estimate_motion_icp(prev_points, points)
            
            # This ICP correction represents the motion since last scan
            # Apply it to correct the accumulated odometry error
            # Previous pose + ICP motion = corrected current pose
            prev_corrected = self.scan_history[-1]['corrected_pose']
            
            # Apply motion correction
            cos_th = np.cos(prev_corrected[2])
            sin_th = np.sin(prev_corrected[2])
            
            self.corrected_pose[0] = prev_corrected[0] + dx_icp * cos_th - dy_icp * sin_th
            self.corrected_pose[1] = prev_corrected[1] + dx_icp * sin_th + dy_icp * cos_th
            self.corrected_pose[2] = prev_corrected[2] + dtheta_icp
            
            # Calculate how much odometry drifted from our ICP estimate
            self.pose_correction = self.corrected_pose - self.odom_pose
        else:
            # No previous scan or ICP disabled: trust raw odometry
            self.corrected_pose = self.odom_pose.copy()
            self.pose_correction = np.array([0.0, 0.0, 0.0])
        
        # Store scan at CORRECTED position (not raw odom)
        self.scan_history.append({
            'points': points,
            'time': msg.header.stamp,
            'odom_pose': self.odom_pose.copy(),
            'corrected_pose': self.corrected_pose.copy()
        })
        
        # Limit history size
        if len(self.scan_history) > self.max_scans:
            self.scan_history.pop(0)
        
        # Update occupancy grid with all scans at their CORRECTED positions
        self.update_occupancy_grid()
        
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
    
    def estimate_motion_icp(self, prev_points, curr_points, max_iterations=5):
        """
        Iterative Closest Point (ICP) for scan matching.
        Estimates the rigid transformation between two point clouds.
        Returns: (dx, dy, dtheta)
        """
        if len(prev_points) == 0 or len(curr_points) == 0:
            return 0.0, 0.0, 0.0
        
        dx, dy, dtheta = 0.0, 0.0, 0.0
        
        try:
            for iteration in range(max_iterations):
                # Find nearest neighbors from prev to curr
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
                
                # SVD for optimal rotation
                H = centered_prev.T @ centered_curr
                U, _, Vt = np.linalg.svd(H)
                R_matrix = Vt.T @ U.T
                
                # Ensure proper rotation (det = 1, not -1)
                if np.linalg.det(R_matrix) < 0:
                    Vt[-1, :] *= -1
                    R_matrix = Vt.T @ U.T
                
                # Extract rotation angle
                theta = np.arctan2(R_matrix[1, 0], R_matrix[0, 0])
                
                # Translation
                t = centroid_curr - R_matrix @ centroid_prev
                
                dx, dy, dtheta = t[0], t[1], theta
                
                # Early exit if converged
                if abs(dtheta) < 0.0001 and np.linalg.norm(t) < 0.001:
                    break
        except Exception as e:
            self.get_logger().warn(f'ICP error: {e}')
            return 0.0, 0.0, 0.0
        
        return dx, dy, dtheta
    
    def update_occupancy_grid(self):
        """Mark occupied cells in grid from all scans using CORRECTED poses"""
        self.occupancy_grid.fill(0)  # Reset
        
        center_idx = self.grid_width // 2
        
        for scan_data in self.scan_history:
            points = scan_data['points']
            # Use CORRECTED pose (ICP-adjusted), not raw odometry
            pose = scan_data['corrected_pose']
            
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
        """
        Publish CORRECTED pose (ICP-adjusted odometry).
        """
        msg = PoseStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'  # In map frame (corrected)
        msg.pose.position.x = float(self.corrected_pose[0])
        msg.pose.position.y = float(self.corrected_pose[1])
        
        # Quaternion from yaw angle
        quat = R.from_euler('z', self.corrected_pose[2]).as_quat()
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.pub_pose.publish(msg)
    
    def publish_transforms(self, timestamp):
        """
        Publish map -> base_link transform.
        
        Frame hierarchy:
        map (static, maintained by SLAM with ICP correction)
          └─→ base_link (robot position from corrected odometry)
        
        The SLAM applies ICP to correct odometry drift and maintains
        the map as a static reference frame.
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        # Direct transform from map to robot (using corrected pose)
        t.transform.translation.x = float(self.corrected_pose[0])
        t.transform.translation.y = float(self.corrected_pose[1])
        t.transform.translation.z = 0.0
        
        quat = R.from_euler('z', self.corrected_pose[2]).as_quat()
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
