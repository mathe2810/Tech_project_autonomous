#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

MAP_RES = 0.0133
START_X_PX = 270.0
START_Y_PX = 250.0
START_YAW_RAD = 0.0

START_X_M = START_X_PX * MAP_RES
START_Y_M = START_Y_PX * MAP_RES

class Nav2GoalStart(Node):
    def __init__(self):
        super().__init__('nav2_goal_start')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self):
        self.get_logger().info('⏳ Waiting for Nav2 action server /navigate_to_pose...')
        if not self.client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('❌ Nav2 action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(START_X_M)
        pose.pose.position.y = float(START_Y_M)
        pose.pose.position.z = 0.0

        half = START_YAW_RAD / 2.0
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)

        goal_msg.pose = pose

        self.get_logger().info(
            f'🎯 Sending goal to track start: x={START_X_M:.3f}m, y={START_Y_M:.3f}m, yaw={START_YAW_RAD:.3f}rad'
        )

        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by Nav2')
            return False

        self.get_logger().info('✅ Goal accepted by Nav2')
        return True


def main():
    rclpy.init()
    node = Nav2GoalStart()
    ok = False
    try:
        ok = node.send_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
