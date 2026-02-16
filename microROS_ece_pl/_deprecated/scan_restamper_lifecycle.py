#!/usr/bin/env python3
"""
Scan Restamper Lifecycle Node
Fixes timestamp mismatches from LIDAR by restamping with system clock.

Lifecycle aware - conforms to Nav2 lifecycle architecture.
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import State
from sensor_msgs.msg import LaserScan


class ScanRestamperLifecycleNode(LifecycleNode):
    """Scan timestamp restamper - Lifecycle Node"""
    
    def __init__(self):
        super().__init__('scan_restamper')
        
        # Publishers and subscribers
        self.scan_sub = None
        self.scan_pub = None
        self.scan_count = 0
        
        self.get_logger().info("Scan Restamper Node created (UNCONFIGURED)")
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure - create publishers and subscribers"""
        self.get_logger().info("Configuring Scan Restamper Node...")
        
        try:
            # Create subscriber
            self.scan_sub = self.create_subscription(
                LaserScan, '/scan_raw', self.scan_callback, 10
            )
            
            # Create publisher
            self.scan_pub = self.create_publisher(
                LaserScan, '/scan', 10
            )
            
            self.scan_count = 0
            
            self.get_logger().info("✓ Scan Restamper configured (/scan_raw → /scan)")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Configuration error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate - start publishing"""
        self.get_logger().info("Activating Scan Restamper Node...")
        try:
            self.get_logger().info("✓ Scan Restamper active")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Activation error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate - stop publishing"""
        self.get_logger().info("Deactivating Scan Restamper Node...")
        try:
            self.get_logger().info("✓ Scan Restamper deactivated")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Deactivation error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup - destroy resources"""
        self.get_logger().info("Cleaning up Scan Restamper Node...")
        try:
            if self.scan_sub is not None:
                self.destroy_subscription(self.scan_sub)
                self.scan_sub = None
            
            if self.scan_pub is not None:
                self.destroy_publisher(self.scan_pub)
                self.scan_pub = None
            
            self.get_logger().info("✓ Scan Restamper cleanup complete")
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Cleanup error: {e}")
            return TransitionCallbackReturn.FAILURE
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown"""
        self.get_logger().info("Shutting down Scan Restamper Node...")
        return TransitionCallbackReturn.SUCCESS
    
    def scan_callback(self, msg: LaserScan):
        """Receive and restamp scan"""
        if self.get_current_state().id != 3:  # Not ACTIVE
            return
        
        # Copy scan and restamp
        msg.header.stamp = self.get_clock().now().to_msg()
        
        self.scan_pub.publish(msg)
        self.scan_count += 1
        
        if self.scan_count % 100 == 0:
            self.get_logger().debug(f"Restamped {self.scan_count} scans")


def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamperLifecycleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
