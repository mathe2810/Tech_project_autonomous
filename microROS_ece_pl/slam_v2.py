#!/usr/bin/env python3
"""
Simple SLAM v2 - Filtered Scan Accumulation
Builds a static map by accumulating LIDAR scans at odometry positions.
Filters out small movements to reduce noise and prevent map drift.

Key idea: Only add scans when robot moves significantly (>10cm or >5°)
This creates a truly static map that updates as robot explores.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np
import math


class SLAMv2(Node):
    def __init__(self):
        super().__init__('slam_v2')
        
        # Parameters
        self.declare_parameter('grid_size', 10)  # 10x10 meters
        self.declare_parameter('resolution', 0.05)  # 5cm per cell
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('movement_threshold_dist', 0.1)  # 10cm
        self.declare_parameter('movement_threshold_angle', 0.087)  # ~5 degrees
        
        self.grid_size = self.get_parameter('grid_size').value
        self.resolution = self.get_parameter('resolution').value
        self.max_range = self.get_parameter('max_range').value
        self.min_dist = self.get_parameter('movement_threshold_dist').value
        self.min_angle = self.get_parameter('movement_threshold_angle').value
        
        # Grid
        self.grid_width = int(self.grid_size / self.resolution)
        self.occupancy_grid = np.zeros((self.grid_width, self.grid_width), dtype=np.int8)
        self.map_origin_x = -(self.grid_width // 2) * self.resolution
        self.map_origin_y = -(self.grid_width // 2) * self.resolution
        
        # Robot pose (from odometry)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Last pose where map was updated
        self.last_map_update_x = 0.0
        self.last_map_update_y = 0.0
        self.last_map_update_theta = 0.0
        
        # Publishers
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/slam/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Timer to publish map at 5Hz
        self.create_timer(0.2, self.timer_callback)
        self.last_timestamp = None
        
        self.get_logger().info(
            f'SLAM v2 started\n'
            f'  Grid: {self.grid_size}m x {self.grid_size}m\n'
            f'  Resolution: {self.resolution}m/cell\n'
            f'  Movement threshold: {self.min_dist}m, {math.degrees(self.min_angle)}°'
        )
    
    def odom_callback(self, msg: Odometry):
        """Receive robot odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        self.robot_theta = rot.as_euler('xyz')[2]
    
    def scan_callback(self, msg: LaserScan):
        """Process LIDAR scan and update map if robot moved enough"""
        self.last_timestamp = msg.header.stamp
        
        # Check if robot moved significantly
        dx = self.robot_x - self.last_map_update_x
        dy = self.robot_y - self.last_map_update_y
        dtheta = abs(self._normalize_angle(self.robot_theta - self.last_map_update_theta))
        
        distance = math.sqrt(dx**2 + dy**2)
        
        # Only update map if movement exceeds threshold
        if distance > self.min_dist or dtheta > self.min_angle:
            self.get_logger().debug(
                f'Map update: dist={distance:.3f}m, angle={math.degrees(dtheta):.1f}°'
            )
            
            # Convert scan to points
            points = self.scan_to_points(msg)
            
            # Add points to occupancy grid
            self.add_scan_to_grid(points)
            
            # Remember this pose
            self.last_map_update_x = self.robot_x
            self.last_map_update_y = self.robot_y
            self.last_map_update_theta = self.robot_theta
    
    def scan_to_points(self, scan: LaserScan):
        """Convert LaserScan to 2D points (x, y) in robot frame"""
        points = []
        for i, range_val in enumerate(scan.ranges):
            if np.isnan(range_val) or range_val < 0.1 or range_val > self.max_range:
                continue
            
            angle = scan.angle_min + i * scan.angle_increment
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            points.append([x, y])
        
        return np.array(points) if points else np.empty((0, 2))
    
    def add_scan_to_grid(self, points):
        """Add scan points to occupancy grid at current robot pose"""
        if len(points) == 0:
            return
        
        # Rotate and translate points to map frame
        cos_theta = np.cos(self.robot_theta)
        sin_theta = np.sin(self.robot_theta)
        
        center_idx_x = (self.grid_width // 2)
        center_idx_y = (self.grid_width // 2)
        
        for point in points:
            # Rotate to world frame
            world_x = cos_theta * point[0] - sin_theta * point[1] + self.robot_x
            world_y = sin_theta * point[0] + cos_theta * point[1] + self.robot_y
            
            # Convert to grid coordinates
            grid_x = int((world_x - self.map_origin_x) / self.resolution)
            grid_y = int((world_y - self.map_origin_y) / self.resolution)
            
            # Mark as occupied (hit)
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_width:
                # Increase confidence if already occupied
                if self.occupancy_grid[grid_y, grid_x] < 100:
                    self.occupancy_grid[grid_y, grid_x] = min(100, self.occupancy_grid[grid_y, grid_x] + 20)
                else:
                    self.occupancy_grid[grid_y, grid_x] = 100
            
            # Optional: raycast from robot to point to mark free space
            # (Can be added later for better accuracy)
    
    def publish_map(self, timestamp):
        """Publish occupancy grid"""
        msg = OccupancyGrid()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_width
        msg.info.height = self.grid_width
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        
        # Convert to 0-100 scale for ROS
        map_data = (self.occupancy_grid / 100.0 * 100).astype(np.int8)
        msg.data = map_data.flatten().tolist()
        
        self.pub_map.publish(msg)
    
    def publish_pose(self, timestamp):
        """Publish robot pose"""
        msg = PoseStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(self.robot_x)
        msg.pose.position.y = float(self.robot_y)
        
        quat = R.from_euler('z', self.robot_theta).as_quat()
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        
        self.pub_pose.publish(msg)
    
    def publish_tf(self, timestamp):
        """Publish map -> base_link transform"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = float(self.robot_x)
        t.transform.translation.y = float(self.robot_y)
        t.transform.translation.z = 0.0
        
        quat = R.from_euler('z', self.robot_theta).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def timer_callback(self):
        """Publish map, pose, and transforms"""
        if self.last_timestamp is not None:
            self.publish_map(self.last_timestamp)
            self.publish_pose(self.last_timestamp)
            self.publish_tf(self.last_timestamp)
    
    def save_map(self, filename='/tmp/slam_map.pgm'):
        """Save occupancy grid as PGM file"""
        try:
            # Convert to 0-255 scale
            map_image = (self.occupancy_grid / 100.0 * 255).astype(np.uint8)
            map_image = 255 - map_image  # Invert: 0=white (free), 255=black (occupied)
            
            with open(filename, 'wb') as f:
                # PGM header
                f.write(b'P5\n')
                f.write(f'{self.grid_width} {self.grid_width}\n'.encode())
                f.write(b'255\n')
                f.write(map_image.tobytes())
            
            # Save metadata
            metadata_file = filename.replace('.pgm', '.yaml')
            with open(metadata_file, 'w') as f:
                f.write(f'# SLAM Map Metadata\n')
                f.write(f'resolution: {self.resolution}\n')
                f.write(f'width: {self.grid_width}\n')
                f.write(f'height: {self.grid_width}\n')
                f.write(f'origin_x: {self.map_origin_x}\n')
                f.write(f'origin_y: {self.map_origin_y}\n')
                f.write(f'robot_x: {self.robot_x}\n')
                f.write(f'robot_y: {self.robot_y}\n')
                f.write(f'robot_theta: {self.robot_theta}\n')
            
            self.get_logger().info(f'✅ Map saved to {filename}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save map: {e}')
            return False
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = SLAMv2()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Saving map...')
        node.save_map()
    finally:
        node.get_logger().info('SLAM v2 shutdown')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
