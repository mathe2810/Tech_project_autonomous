#!/usr/bin/env python3
"""
Compare 3 odometries:
1. Ground Truth (from simulator)
2. RF2O (raw laser odometry)
3. SLAM (with closed-loop correction via TF)
"""
import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException

class OdomComparator(Node):
    def __init__(self):
        super().__init__('odom_comparator_slam')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # TF2 for SLAM position (map->base_link)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to Ground Truth and RF2O
        self.sub_gt = self.create_subscription(Odometry, '/odom_ground_truth', self.cb_gt, qos)
        self.sub_rf2o = self.create_subscription(Odometry, '/odom_rf2o', self.cb_rf2o, qos)
        
        self.gt = None
        self.rf2o = None
        self.initial_gt_x = None
        self.initial_gt_y = None
        self.initial_slam_x = None
        self.initial_slam_y = None
        self.count = 0
        
        self.get_logger().info('🔍 Comparing: Ground Truth vs RF2O vs SLAM (via TF)')
        self.create_timer(2.0, self.compare)
        
    def cb_gt(self, msg): self.gt = msg
    def cb_rf2o(self, msg): self.rf2o = msg
        
    def quat_to_yaw(self, q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    
    def compare(self):
        if not self.gt or not self.rf2o:
            if self.count % 5 == 0:
                self.get_logger().warning('⏳ Waiting for Ground Truth and RF2O...')
            self.count += 1
            return
        
        # Initialize reference on first measurement
        if self.initial_gt_x is None:
            self.initial_gt_x = self.gt.pose.pose.position.x
            self.initial_gt_y = self.gt.pose.pose.position.y
        
        # Extract and normalize positions
        x_gt = self.gt.pose.pose.position.x - self.initial_gt_x
        y_gt = self.gt.pose.pose.position.y - self.initial_gt_y
        yaw_gt = self.quat_to_yaw(self.gt.pose.pose.orientation)
        
        x_rf2o = self.rf2o.pose.pose.position.x
        y_rf2o = self.rf2o.pose.pose.position.y
        yaw_rf2o = self.quat_to_yaw(self.rf2o.pose.pose.orientation)
        
        # Calculate RF2O errors
        err_x_rf2o = x_rf2o - x_gt
        err_y_rf2o = y_rf2o - y_gt
        err_dist_rf2o = math.sqrt(err_x_rf2o**2 + err_y_rf2o**2)
        err_yaw_rf2o = yaw_rf2o - yaw_gt
        while err_yaw_rf2o > math.pi: err_yaw_rf2o -= 2*math.pi
        while err_yaw_rf2o < -math.pi: err_yaw_rf2o += 2*math.pi
        
        # Get SLAM position from TF (map->base_link)
        slam_available = False
        x_slam = y_slam = yaw_slam = 0
        slam_err_x = slam_err_y = slam_err_dist = slam_err_yaw = 0
        
        try:
            # Lookup transform from map to base_link
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            x_slam = trans.transform.translation.x
            y_slam = trans.transform.translation.y
            q = trans.transform.rotation
            yaw_slam = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            
            # Initialize SLAM reference on first successful lookup
            if self.initial_slam_x is None:
                self.initial_slam_x = x_slam
                self.initial_slam_y = y_slam
            
            # Normalize SLAM position
            x_slam -= self.initial_slam_x
            y_slam -= self.initial_slam_y
            
            slam_err_x = x_slam - x_gt
            slam_err_y = y_slam - y_gt
            slam_err_dist = math.sqrt(slam_err_x**2 + slam_err_y**2)
            slam_err_yaw = yaw_slam - yaw_gt
            while slam_err_yaw > math.pi: slam_err_yaw -= 2*math.pi
            while slam_err_yaw < -math.pi: slam_err_yaw += 2*math.pi
            
            slam_available = True
            
        except TransformException as ex:
            if self.count % 5 == 0:
                self.get_logger().warning(f'⏳ Waiting for SLAM TF (map->base_link)...')
        
        # Display comparison
        msg = (
            f"\n{'='*75}\n"
            f"📊 ODOMETRY COMPARISON (relative to start)\n"
            f"{'='*75}\n"
            f"\n🎯 GROUND TRUTH:\n"
            f"   Position:  x={x_gt:7.3f}m  y={y_gt:7.3f}m\n"
            f"   Heading:   θ={math.degrees(yaw_gt):7.1f}°\n"
            f"\n🔴 RF2O (Raw Laser Odometry):\n"
            f"   Position:  x={x_rf2o:7.3f}m  y={y_rf2o:7.3f}m\n"
            f"   Heading:   θ={math.degrees(yaw_rf2o):7.1f}°\n"
            f"   Error:     Δx={err_x_rf2o:+7.3f}m  Δy={err_y_rf2o:+7.3f}m  dist={err_dist_rf2o:6.3f}m  Δθ={math.degrees(err_yaw_rf2o):+6.1f}°\n"
        )
        
        if slam_available:
            msg += (
                f"\n🟢 SLAM (Closed-Loop Corrected via TF):\n"
                f"   Position:  x={x_slam:7.3f}m  y={y_slam:7.3f}m\n"
                f"   Heading:   θ={math.degrees(yaw_slam):7.1f}°\n"
                f"   Error:     Δx={slam_err_x:+7.3f}m  Δy={slam_err_y:+7.3f}m  dist={slam_err_dist:6.3f}m  Δθ={math.degrees(slam_err_yaw):+6.1f}°\n"
                f"\n📈 IMPROVEMENT with SLAM:\n"
                f"   Position:  {err_dist_rf2o:6.3f}m → {slam_err_dist:6.3f}m  ({(1-slam_err_dist/max(err_dist_rf2o, 0.001))*100:+6.1f}%)\n"
                f"   Heading:   {abs(math.degrees(err_yaw_rf2o)):6.1f}° → {abs(math.degrees(slam_err_yaw)):6.1f}°\n"
            )
        else:
            msg += f"\n🟡 SLAM: Not available yet (waiting for map->base_link TF)...\n"
        
        msg += f"{'='*75}\n"
        self.get_logger().info(msg)

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
