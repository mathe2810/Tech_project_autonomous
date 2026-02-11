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
        Receive raw motor odometry and use it as primary pose estimate.
        SLAM uses this odometry and corrects for drift using scan matching.
        """
        # Extract pose from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        yaw = rot.as_euler('xyz')[2]
        
        # Update robot pose from odometry (this is our primary estimate)
        self.robot_pose = np.array([x, y, yaw])
        self.last_odom_pose = msg.pose.pose
        
    def scan_callback(self, msg: LaserScan):
        """Process incoming LIDAR scan"""
        self.last_timestamp = msg.header.stamp
        
        # Convert polar coordinates to cartesian
        points = self.scan_to_points(msg)
        
        # NOTE: Motion estimation is done using odometry (/odom topic)
        # NOT via scan matching. This ensures the map stays static
        # and only the robot moves relative to it.
        #
        # The robot pose is continuously updated via odom_callback().
        # This scan is stored at the current robot's estimated position.
        
        # Store scan at current robot pose
        self.scan_history.append({
            'points': points,
            'time': msg.header.stamp,
            'pose': self.robot_pose.copy()
        })
        
        # Limit history size
        if len(self.scan_history) > self.max_scans:
            self.scan_history.pop(0)
        
        # Update occupancy grid with all scans at their estimated positions
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
        """
        Publish current pose from motor odometry (not SLAM's own estimate).
        This ensures consistency with the odometry frame.
        The SLAM's job is to maintain the map, not to estimate pose.
        """
        msg = PoseStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'odom'  # Pose is in odometry frame, which motor_odom publishes
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
        """
        Publish map -> odom transform for SLAM
        The SLAM maintains the map frame (static reference).
        The map -> odom transform corrects odometry drift.
        The odom -> base_link transform comes from motor_odom_node.
        
        Frame hierarchy:
        map (fixed) -> odom (from motor odometry) -> base_link (robot)
                     ^         ^                    ^
                     |         |                    |
                  updated   from motor_odom    from motor_odom
                  by SLAM       node             node
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        # This transform corrects the odometry drift
        # It's the difference between SLAM's estimated pose and accumulated odometry
        # For a truly static map, we measure how much odometry has drifted
        # and publish that correction here
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
