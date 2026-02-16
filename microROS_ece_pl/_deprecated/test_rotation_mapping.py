#!/usr/bin/env python3
"""
Test rotation mapping with 100% LIDAR mode.
Performs slow, controlled rotations to verify SLAM can track yaw correctly.
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
import time
import math

def main():
    rclpy.init()
    node = rclpy.create_node('test_rotation')
    
    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    # TF listener to monitor yaw from SLAM
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    twist = Twist()
    
    print("""
╔════════════════════════════════════════════╗
║  ROTATION MAPPING TEST (100% LIDAR MODE)  ║
╚════════════════════════════════════════════╝
    
Test sequence:
1. Wait 2s (let SLAM settle)
2. Rotate SLOW (w=0.1 rad/s = 5.7°/s) for 5s → ~30° rotation
3. Wait 3s (let SLAM match scans)
4. Rotate SLOW in opposite direction for 5s → back to ~0°
5. Check: map should NOT have rotated

Watch RViz:
- Grid reference frame should stay FIXED
- Robot should rotate in place
- Map should NOT rotate with robot
    """)
    
    time.sleep(2)
    print("Step 1: Initial wait (2s) - SLAM settling...")
    node.get_logger().info("SLAM should match initial scans")
    
    # Step 2: Slow rotation (5.7°/s)
    print("\nStep 2: Rotating CCW slowly (w=0.1 rad/s for 5s)...")
    twist.angular.z = 0.1  # 5.7°/s
    start_time = time.time()
    while time.time() - start_time < 5.0:
        cmd_vel_pub.publish(twist)
        time.sleep(0.01)
    
    # Stop
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    
    print("Rotation complete. Waiting 3s for SLAM to match new scans...")
    time.sleep(3)
    
    # Step 3: Verify orientation
    try:
        trans = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        quat = trans.transform.rotation
        
        # Convert quaternion to yaw
        yaw = math.atan2(2*(quat.w*quat.z + quat.x*quat.y), 
                         1 - 2*(quat.y*quat.y + quat.z*quat.z))
        
        print(f"\n✓ Yaw after rotation: {math.degrees(yaw):.1f}°")
        print(f"  Expected: ~30° (if SLAM working)")
        print(f"  If yaw=0° or wildly different: SLAM rotation tracking FAILED")
    except Exception as e:
        print(f"⚠ Could not read transform: {e}")
    
    # Step 4: Rotate back
    print("\nStep 3: Rotating CW slowly back to start (w=-0.1 rad/s for 5s)...")
    twist.angular.z = -0.1
    start_time = time.time()
    while time.time() - start_time < 5.0:
        cmd_vel_pub.publish(twist)
        time.sleep(0.01)
    
    # Stop
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    
    print("Rotation complete. Waiting 3s for SLAM...")
    time.sleep(3)
    
    # Final check
    try:
        trans = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        quat = trans.transform.rotation
        
        yaw = math.atan2(2*(quat.w*quat.z + quat.x*quat.y), 
                         1 - 2*(quat.y*quat.y + quat.z*quat.z))
        
        print(f"\n✓ Final yaw: {math.degrees(yaw):.1f}°")
        print(f"  Expected: ~0° (back to start)")
        
        if abs(math.degrees(yaw)) < 15:
            print("\n✅ SUCCESS: Rotation mapping working!")
        else:
            print("\n❌ FAILED: Map rotated with robot (yaw divergence)")
            
    except Exception as e:
        print(f"⚠ Could not read final transform: {e}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
