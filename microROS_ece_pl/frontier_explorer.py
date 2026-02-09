#!/usr/bin/env python3
"""
Frontier-Based Exploration Node for Nav2
- Listens to /map (SLAM Toolbox)
- Detects frontiers (boundaries between explored and unexplored)
- Sends frontier centroids as goals to Nav2
- Autonomous mapping with intelligent exploration
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy import ndimage
import math

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('frontier_threshold', 0.5)
        self.declare_parameter('min_frontier_size', 5)
        
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.frontier_threshold = self.get_parameter('frontier_threshold').get_parameter_value().double_value
        self.min_frontier_size = self.get_parameter('min_frontier_size').get_parameter_value().integer_value
        
        # Subscribe to map
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 1)
        
        # Action client for Nav2 goals
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State
        self.map_data = None
        self.exploring = False
        self.current_frontier = None
        self.goal_sent = False
        
        self.get_logger().info("Frontier Explorer started - waiting for map...")
        
        # Timer for exploration loop
        self.timer = self.create_timer(1.0, self.exploration_loop)
    
    def map_callback(self, msg: OccupancyGrid):
        """Store the latest map"""
        self.map_data = msg
    
    def find_frontiers(self):
        """
        Detect frontier cells in the occupancy grid.
        Frontiers are boundaries between explored (0-100) and unknown (-1) cells.
        
        Returns list of frontier clusters (each cluster is list of (x,y) coordinates)
        """
        if self.map_data is None:
            return []
        
        # Convert map to numpy array
        width = self.map_data.info.width
        height = self.map_data.info.height
        data = np.array(self.map_data.data).reshape(height, width)
        
        # Create binary map: unknown cells = 1, known cells = 0
        unknown = (data == -1).astype(np.uint8)
        
        # Create binary map: free cells = 1, others = 0
        free = (data == 0).astype(np.uint8)
        
        # Find frontier: free cells adjacent to unknown cells
        frontier = np.zeros_like(unknown)
        
        for i in range(1, height - 1):
            for j in range(1, width - 1):
                if free[i, j]:  # Cell is free
                    # Check 8-neighbors
                    neighbors = [
                        unknown[i-1, j-1], unknown[i-1, j], unknown[i-1, j+1],
                        unknown[i, j-1], unknown[i, j+1],
                        unknown[i+1, j-1], unknown[i+1, j], unknown[i+1, j+1]
                    ]
                    if any(neighbors):  # Has unknown neighbor
                        frontier[i, j] = 1
        
        # Label connected components
        labeled, num_features = ndimage.label(frontier)
        
        # Extract frontier clusters
        frontiers = []
        for cluster_id in range(1, num_features + 1):
            cluster_mask = (labeled == cluster_id)
            cluster_cells = np.argwhere(cluster_mask)
            
            # Filter by size
            if len(cluster_cells) >= self.min_frontier_size:
                frontiers.append(cluster_cells)
        
        return frontiers
    
    def centroid_to_pose(self, centroid_pixel):
        """
        Convert pixel centroid to world pose (map frame)
        """
        if self.map_data is None:
            return None
        
        py, px = centroid_pixel  # argwhere returns (row, col) = (y, x)
        
        # Convert to world coordinates
        map_x = self.map_data.info.origin.position.x + px * self.map_data.info.resolution
        map_y = self.map_data.info.origin.position.y + py * self.map_data.info.resolution
        
        pose = PoseStamped()
        pose.header.frame_id = self.map_data.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = map_x
        pose.pose.position.y = map_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # No rotation
        
        return pose
    
    def select_best_frontier(self, frontiers):
        """
        Select the best frontier to explore next.
        Currently: closest frontier to robot (at origin if no odometry).
        Could be improved with information gain metrics.
        """
        if not frontiers:
            return None
        
        robot_x = 0.0
        robot_y = 0.0
        
        best_frontier = None
        best_distance = float('inf')
        
        for frontier in frontiers:
            # Compute centroid
            centroid = np.mean(frontier, axis=0)
            pose = self.centroid_to_pose(centroid)
            
            if pose is None:
                continue
            
            # Distance to robot
            dist = math.sqrt(pose.pose.position.x**2 + pose.pose.position.y**2)
            
            if dist < best_distance:
                best_distance = dist
                best_frontier = pose
        
        return best_frontier
    
    def send_goal(self, pose):
        """Send goal to Nav2 (navigate_to_pose action)"""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2 not ready yet")
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
        self.goal_sent = True
        self.get_logger().info(f"Goal sent to Nav2: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        
        return True
    
    def goal_response_callback(self, future):
        """Callback when Nav2 accepts the goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.goal_sent = False
            return
        
        self.get_logger().info("Goal accepted by Nav2")
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Callback when goal is reached"""
        result = future.result().result
        self.get_logger().info("Goal reached!")
        self.goal_sent = False
    
    def exploration_loop(self):
        """Main exploration loop"""
        if self.map_data is None:
            self.get_logger().info("Waiting for map...")
            return
        
        # If we just sent a goal, wait for it to complete
        if self.goal_sent:
            return
        
        # Detect frontiers
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().info("No more frontiers found - exploration complete!")
            self.exploring = False
            return
        
        self.get_logger().info(f"Found {len(frontiers)} frontier clusters")
        
        # Select best frontier
        best_frontier = self.select_best_frontier(frontiers)
        
        if best_frontier is None:
            self.get_logger().warn("Could not compute frontier pose")
            return
        
        # Send goal to Nav2
        if not self.exploring:
            self.exploring = True
            self.get_logger().info("[FRONTIER] Starting autonomous exploration")
        
        self.send_goal(best_frontier)

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Frontier explorer stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
