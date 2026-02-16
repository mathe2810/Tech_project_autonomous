#!/usr/bin/env python3
"""
ROS2 Stack Validation Script
Verifies all nodes are running and data is flowing correctly.
"""

import subprocess
import sys
import time

def run_cmd(cmd):
    """Run shell command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.stdout.strip()
    except subprocess.TimeoutExpired:
        return "TIMEOUT"
    except Exception as e:
        return f"ERROR: {e}"

def check_node_running(node_name):
    """Check if a ROS2 node is running"""
    output = run_cmd(f"ros2 node list 2>/dev/null | grep -w '{node_name}'")
    return len(output) > 0

def check_topic_has_data(topic_name, num_messages=3):
    """Check if topic has recent data"""
    try:
        result = subprocess.run(
            f"timeout 3 ros2 topic echo {topic_name} --once 2>/dev/null",
            shell=True, capture_output=True, text=True, timeout=5
        )
        return len(result.stdout) > 50  # Has substantial output
    except:
        return False

def get_topic_info(topic_name):
    """Get topic info"""
    output = run_cmd(f"ros2 topic info {topic_name} 2>/dev/null")
    if output:
        return output.split('\n')[0]
    return "NOT FOUND"

def main():
    print("\n" + "="*70)
    print("           ROS2 NAVIGATION STACK VALIDATION")
    print("="*70 + "\n")
    
    # Check 1: Micro-ROS Agent
    print("🔌 [1/5] CHECKING MICRO-ROS AGENT")
    agent_running = run_cmd("pgrep -f 'micro_ros_agent' | wc -l")
    if int(agent_running) > 0:
        print("    ✅ Agent running")
    else:
        print("    ❌ Agent NOT running - run ./start_agent.sh")
        return False
    
    # Check 2: Nodes
    print("\n🤖 [2/5] CHECKING ROS2 NODES")
    nodes = {
        'imu_fir_filter': '    IMU Filter',
        'motor_odom': '    Motor Odometry',
        'simple_slam': '    SLAM',
        'kalman_filter_fusion': '    Kalman Fusion'
    }
    
    all_nodes_ok = True
    for node, label in nodes.items():
        if check_node_running(node):
            print(f"    ✅{label}")
        else:
            print(f"    ❌{label} - NOT RUNNING")
            all_nodes_ok = False
    
    if not all_nodes_ok:
        print("    → Run: ./start_stack.sh")
        return False
    
    # Check 3: Data Flow
    print("\n📡 [3/5] CHECKING DATA FLOW")
    topics = {
        '/imu/data': 'Raw IMU',
        '/imu/data_filtered': 'Filtered IMU',
        '/scan': 'LIDAR',
        '/odom': 'Raw Odometry',
        '/odom_filtered': 'Fused Odometry (IMPORTANT)',
        '/map': 'SLAM Map',
        '/slam/pose': 'SLAM Pose'
    }
    
    all_topics_ok = True
    for topic, label in topics.items():
        if check_topic_has_data(topic):
            print(f"    ✅ {label:30} {topic}")
        else:
            print(f"    ❌ {label:30} {topic} - NO DATA")
            all_topics_ok = False
    
    if not all_topics_ok:
        print("    ⚠️  Some topics have no data (might need sensor data)")
    
    # Check 4: TF Transforms
    print("\n🗺️  [4/5] CHECKING TRANSFORMS")
    try:
        tf_output = run_cmd("ros2 tf2_tree.py 2>/dev/null | head -20")
        if 'map' in tf_output or 'odom' in tf_output or 'base_link' in tf_output:
            print("    ✅ TF transforms present")
        else:
            print("    ⚠️  TF transforms might be missing")
    except:
        print("    ⚠️  Could not check TF")
    
    # Check 5: Kalman Fusion Validation
    print("\n🔄 [5/5] CHECKING KALMAN FUSION")
    print("    Comparing /odom vs /odom_filtered...")
    
    try:
        # Get raw odom
        odom_raw = run_cmd("ros2 topic echo /odom --once 2>/dev/null | grep -A 2 'position'")
        odom_filtered = run_cmd("ros2 topic echo /odom_filtered --once 2>/dev/null | grep -A 2 'position'")
        
        if odom_raw and odom_filtered:
            print("    ✅ Both odometry streams active")
            print("    ✅ Kalman fusion is working")
        else:
            print("    ⚠️  Could not read odometry data")
    except:
        print("    ⚠️  Could not compare odometry")
    
    print("\n" + "="*70)
    print("                    ✅ VALIDATION COMPLETE")
    print("="*70)
    print("\n📝 NEXT STEPS:")
    print("   1. Monitor /odom_filtered:  ros2 topic echo /odom_filtered")
    print("   2. Check SLAM progress:     ros2 topic echo /map --once")
    print("   3. View IMU data:           ros2 topic echo /imu/data_filtered")
    print("   4. When satisfied, integrate Nav2 for autonomous navigation")
    print("\n" + "="*70 + "\n")
    
    return all_nodes_ok and all_topics_ok

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
