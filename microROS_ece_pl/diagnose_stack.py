#!/usr/bin/env python3
"""
Advanced diagnostics: Compare raw vs filtered data to validate Kalman fusion
"""

import subprocess
import json
import time
import re

def get_topic_data(topic_name):
    """Get one message from a topic and parse it"""
    try:
        result = subprocess.run(
            f"timeout 5 ros2 topic echo {topic_name} --once",
            shell=True, capture_output=True, text=True
        )
        return result.stdout
    except:
        return None

def extract_position(data_str):
    """Extract x, y from position field"""
    try:
        x_match = re.search(r'x: ([-\d.]+)', data_str)
        y_match = re.search(r'y: ([-\d.]+)', data_str)
        if x_match and y_match:
            return float(x_match.group(1)), float(y_match.group(1))
    except:
        pass
    return None, None

def extract_accel(data_str):
    """Extract accelerometer data"""
    try:
        ax = re.search(r'linear_acceleration:\s*x: ([-\d.]+)', data_str)
        ay = re.search(r'y: ([-\d.]+)', data_str)
        az = re.search(r'z: ([-\d.]+)', data_str)
        if ax and ay and az:
            return float(ax.group(1)), float(ay.group(1)), float(az.group(1))
    except:
        pass
    return None, None, None

def main():
    print("\n" + "="*70)
    print("        ADVANCED DIAGNOSTICS - DATA FLOW VALIDATION")
    print("="*70 + "\n")
    
    print("📊 Comparing /odom (raw) vs /odom_filtered (fused)...")
    print("   (Collecting data, please wait...)\n")
    
    # Get raw odometry
    odom_raw = get_topic_data("/odom")
    time.sleep(0.5)
    
    # Get filtered odometry
    odom_filtered = get_topic_data("/odom_filtered")
    time.sleep(0.5)
    
    # Get IMU data
    imu_raw = get_topic_data("/imu/data")
    imu_filtered = get_topic_data("/imu/data_filtered")
    
    # Get SLAM pose
    slam_pose = get_topic_data("/slam/pose")
    
    print("┌─ ODOMETRY COMPARISON ────────────────────────────────────┐")
    print("│")
    
    if odom_raw:
        x_raw, y_raw = extract_position(odom_raw)
        if x_raw is not None:
            print(f"  Raw (/odom):             X={x_raw:+.4f}  Y={y_raw:+.4f}")
        else:
            print(f"  Raw (/odom):             ❌ Could not parse")
    else:
        print(f"  Raw (/odom):             ❌ No data")
    
    if odom_filtered:
        x_filt, y_filt = extract_position(odom_filtered)
        if x_filt is not None:
            print(f"  Filtered (/odom_filtered): X={x_filt:+.4f}  Y={y_filt:+.4f}")
            
            # Compare
            if x_raw is not None and y_raw is not None:
                diff_x = abs(x_filt - x_raw)
                diff_y = abs(y_filt - y_raw)
                print(f"  Difference:            ΔX={diff_x:.4f}  ΔY={diff_y:.4f}")
                
                if diff_x < 0.01 and diff_y < 0.01:
                    print("  Status:                ✅ Fusion working (minimal drift)")
                elif diff_x < 0.1 and diff_y < 0.1:
                    print("  Status:                ✅ Fusion working (moderate difference)")
                else:
                    print("  Status:                ⚠️  Large difference - check SLAM")
        else:
            print(f"  Filtered (/odom_filtered): ❌ Could not parse")
    else:
        print(f"  Filtered (/odom_filtered): ❌ No data")
    
    print("│")
    print("└──────────────────────────────────────────────────────────┘")
    
    print("\n┌─ IMU FILTERING VALIDATION ───────────────────────────────┐")
    print("│")
    
    if imu_raw and imu_filtered:
        ax_raw, ay_raw, az_raw = extract_accel(imu_raw)
        ax_filt, ay_filt, az_filt = extract_accel(imu_filtered)
        
        if ax_raw is not None and ax_filt is not None:
            print(f"  Raw accel X:           {ax_raw:+.4f} m/s²")
            print(f"  Filtered accel X:      {ax_filt:+.4f} m/s²")
            
            # Check if filtering is reducing noise
            raw_variance = (ax_raw ** 2 + ay_raw ** 2 + az_raw ** 2) ** 0.5
            filt_variance = (ax_filt ** 2 + ay_filt ** 2 + az_filt ** 2) ** 0.5
            
            if filt_variance < raw_variance:
                reduction = (1 - filt_variance/raw_variance) * 100
                print(f"  Filtering reduction:   {reduction:.1f}% ✅")
            else:
                print(f"  Filtering reduction:   Not detected ⚠️")
        else:
            print(f"  Could not parse IMU data")
    else:
        print(f"  Missing IMU data streams")
    
    print("│")
    print("└──────────────────────────────────────────────────────────┘")
    
    print("\n┌─ SLAM STATUS ───────────────────────────────────────────┐")
    print("│")
    
    if slam_pose:
        x_slam, y_slam = extract_position(slam_pose)
        if x_slam is not None:
            print(f"  SLAM Pose:             X={x_slam:+.4f}  Y={y_slam:+.4f}")
            
            if x_filt is not None and y_filt is not None:
                slam_diff_x = abs(x_slam - x_filt)
                slam_diff_y = abs(y_slam - y_filt)
                print(f"  vs Fused Odom:         ΔX={slam_diff_x:.4f}  ΔY={slam_diff_y:.4f}")
                
                if slam_diff_x < 0.5 and slam_diff_y < 0.5:
                    print("  Consistency:           ✅ SLAM and odom agree")
                else:
                    print("  Consistency:           ⚠️  Check SLAM quality")
            
            print("  Status:                ✅ SLAM producing poses")
        else:
            print(f"  SLAM Pose:             ❌ Could not parse")
    else:
        print(f"  SLAM Pose:             ❌ No data (SLAM might be initializing)")
    
    print("│")
    print("└──────────────────────────────────────────────────────────┘")
    
    print("\n" + "="*70)
    print("                      DIAGNOSTICS COMPLETE")
    print("="*70)
    print("\n💡 INTERPRETATION:")
    print("   ✅ If Fusion difference < 0.01: Odometry is very stable (no drift)")
    print("   ✅ If IMU filtering > 20%: Noise reduction is working")
    print("   ✅ If SLAM and odom agree: Both systems tracking correctly")
    print("\n📝 NEXT STEP: Run your robot and watch /odom_filtered for stability\n")

if __name__ == '__main__':
    main()
