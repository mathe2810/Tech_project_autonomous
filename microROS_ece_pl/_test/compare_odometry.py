#!/usr/bin/env python3
"""Compare RF2O estimated odometry with ground truth from simulator"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
import math

class OdomComparator(Node):
    def __init__(self):
        super().__init__('odom_comparator')
        
        # QoS compatible avec BEST_EFFORT
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to both odometries
        self.sub_rf2o = self.create_subscription(Odometry, '/odom_rf2o', self.callback_rf2o, qos)
        self.sub_gt = self.create_subscription(Odometry, '/odom_ground_truth', self.callback_gt, qos)
        
        self.rf2o_odom = None
        self.gt_odom = None
        self.count = 0
        self.initial_gt_x = None
        self.initial_gt_y = None
        
        self.get_logger().info('🔍 Odometry Comparator Started - comparing RF2O vs Ground Truth')
        self.timer = self.create_timer(2.0, self.compare)
        
    def callback_rf2o(self, msg):
        self.rf2o_odom = msg
        
    def callback_gt(self, msg):
        self.gt_odom = msg
        
    def quaternion_to_yaw(self, quat):
        """Convert quaternion to yaw angle in radians"""
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        
        # atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        return yaw
        
    def compare(self):
        if self.rf2o_odom is None or self.gt_odom is None:
            if self.count % 5 == 0:
                status = []
                if self.rf2o_odom is None:
                    status.append("RF2O: ❌")
                else:
                    status.append("RF2O: ✅")
                if self.gt_odom is None:
                    status.append("GT: ❌")
                else:
                    status.append("GT: ✅")
                self.get_logger().warning(f"Waiting for data... {' | '.join(status)}")
            self.count += 1
            return
        
        # Initialize reference positions on first measurement
        if self.initial_gt_x is None:
            self.initial_gt_x = self.gt_odom.pose.pose.position.x
            self.initial_gt_y = self.gt_odom.pose.pose.position.y
            
        # Extract positions
        x_rf2o = self.rf2o_odom.pose.pose.position.x
        y_rf2o = self.rf2o_odom.pose.pose.position.y
        
        x_gt = self.gt_odom.pose.pose.position.x - self.initial_gt_x
        y_gt = self.gt_odom.pose.pose.position.y - self.initial_gt_y
        
        # Calculate position error
        err_x = x_rf2o - x_gt
        err_y = y_rf2o - y_gt
        err_dist = math.sqrt(err_x**2 + err_y**2)
        
        # Extract yaw angles
        yaw_rf2o = self.quaternion_to_yaw(self.rf2o_odom.pose.pose.orientation)
        yaw_gt = self.quaternion_to_yaw(self.gt_odom.pose.pose.orientation)
        
        # Calculate angle error (wrap to [-pi, pi])
        err_yaw = yaw_rf2o - yaw_gt
        while err_yaw > math.pi:
            err_yaw -= 2 * math.pi
        while err_yaw < -math.pi:
            err_yaw += 2 * math.pi
        err_yaw_deg = math.degrees(err_yaw)
        
        # Format output
        self.get_logger().info(
            f"\n{'='*70}\n"
            f"📊 ODOMETRY COMPARISON (relative to start)\n"
            f"{'='*70}\n"
            f"Ground Truth Position:  x={x_gt:7.3f}m  y={y_gt:7.3f}m  θ={math.degrees(yaw_gt):7.1f}°\n"
            f"RF2O Estimated:         x={x_rf2o:7.3f}m  y={y_rf2o:7.3f}m  θ={math.degrees(yaw_rf2o):7.1f}°\n"
            f"{'─'*70}\n"
            f"📍 Position Error:      Δx={err_x:7.3f}m  Δy={err_y:7.3f}m  dist={err_dist:6.3f}m\n"
            f"🔄 Angle Error:         Δθ={err_yaw_deg:6.1f}°\n"
            f"{'='*70}"
        )

def main():
    rclpy.init()
    node = OdomComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
