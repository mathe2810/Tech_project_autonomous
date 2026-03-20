#!/usr/bin/env python3

import math
import threading

import pygame
import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def quat_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomPygameVisualizer(Node):
    def __init__(self) -> None:
        super().__init__('odom_pygame_visualizer')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('heading_offset', 0.0)
        self.declare_parameter('scan_yaw_offset', 0.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.heading_offset = float(self.get_parameter('heading_offset').value)
        self.scan_yaw_offset = float(self.get_parameter('scan_yaw_offset').value)

        self._lock = threading.Lock()
        self._ranges = []
        self._angle_min = 0.0
        self._angle_increment = 0.0

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._map_data = None
        self._map_width = 0
        self._map_height = 0
        self._map_resolution = 0.0
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
        self._map_stamp_ns = 0

        self._robot_map_x = None
        self._robot_map_y = None
        self._robot_map_yaw = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, scan_qos)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, odom_qos)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, map_qos)

        self.get_logger().info(
            f'Pygame viz started: scan={self.scan_topic}, odom={self.odom_topic}, map={self.map_topic}, '
            f'heading_offset={self.heading_offset:.3f}, scan_yaw_offset={self.scan_yaw_offset:.3f}'
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        with self._lock:
            self._ranges = list(msg.ranges)
            self._angle_min = msg.angle_min
            self._angle_increment = msg.angle_increment

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y
            self._yaw = quat_to_yaw(msg.pose.pose.orientation)

            odom_frame = msg.header.frame_id if msg.header.frame_id else 'odom'
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.map_frame,
                    odom_frame,
                    rclpy.time.Time(),
                )
                tf_yaw = quat_to_yaw(transform.transform.rotation)
                cos_yaw = math.cos(tf_yaw)
                sin_yaw = math.sin(tf_yaw)

                self._robot_map_x = (
                    transform.transform.translation.x
                    + cos_yaw * self._x
                    - sin_yaw * self._y
                )
                self._robot_map_y = (
                    transform.transform.translation.y
                    + sin_yaw * self._x
                    + cos_yaw * self._y
                )
                self._robot_map_yaw = tf_yaw + self._yaw
            except TransformException:
                self._robot_map_x = None
                self._robot_map_y = None
                self._robot_map_yaw = None

    def _map_cb(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._map_width = int(msg.info.width)
            self._map_height = int(msg.info.height)
            self._map_resolution = float(msg.info.resolution)
            self._map_origin_x = float(msg.info.origin.position.x)
            self._map_origin_y = float(msg.info.origin.position.y)
            self._map_data = list(msg.data)
            self._map_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def snapshot(self):
        with self._lock:
            return {
                'ranges': list(self._ranges),
                'angle_min': self._angle_min,
                'angle_increment': self._angle_increment,
                'x': self._x,
                'y': self._y,
                'yaw': self._yaw,
                'map_data': self._map_data,
                'map_width': self._map_width,
                'map_height': self._map_height,
                'map_resolution': self._map_resolution,
                'map_origin_x': self._map_origin_x,
                'map_origin_y': self._map_origin_y,
                'map_stamp_ns': self._map_stamp_ns,
                'robot_map_x': self._robot_map_x,
                'robot_map_y': self._robot_map_y,
                'robot_map_yaw': self._robot_map_yaw,
            }


def draw(node: OdomPygameVisualizer) -> None:
    pygame.init()
    width, height = 1100, 800
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption('Robot Corridor Viz - Odom + Laser')
    clock = pygame.time.Clock()
    font = pygame.font.SysFont('Arial', 18)

    center_x = width // 2
    center_y = height // 2
    pixels_per_meter = 170.0
    main_view_right_margin = 380
    panel_margin = 16
    panel_width = 340
    panel_height = 340
    panel_x = width - panel_width - panel_margin
    panel_y = panel_margin

    odom_history = []
    max_history = 300

    running = True
    while running and rclpy.ok():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        data = node.snapshot()
        screen.fill((20, 22, 28))

        center_x = (width - main_view_right_margin) // 2

        robot_px = int(center_x + data['x'] * pixels_per_meter)
        robot_py = int(center_y - data['y'] * pixels_per_meter)
        pygame.draw.circle(screen, (255, 220, 80), (robot_px, robot_py), 6)

        odom_history.append((robot_px, robot_py))
        if len(odom_history) > max_history:
            odom_history.pop(0)
        for i, (hp, hy) in enumerate(odom_history):
            alpha = min(255, int(255 * i / max_history))
            pygame.draw.circle(screen, (100, 150, 255), (hp, hy), 2)

        heading_len = 28
        yaw_display = data['yaw'] + node.heading_offset
        hx = int(robot_px + heading_len * math.cos(yaw_display))
        hy = int(robot_py - heading_len * math.sin(yaw_display))
        pygame.draw.line(screen, (255, 120, 80), (robot_px, robot_py), (hx, hy), 3)

        angle = data['angle_min']
        for r in data['ranges']:
            if math.isfinite(r) and 0.02 < r <= node.max_range:
                scan_world_angle = data['yaw'] + angle + node.scan_yaw_offset
                wx = data['x'] + r * math.cos(scan_world_angle)
                wy = data['y'] + r * math.sin(scan_world_angle)
                sx = int(center_x + wx * pixels_per_meter)
                sy = int(center_y - wy * pixels_per_meter)
                if 0 <= sx < width and 0 <= sy < height:
                    screen.set_at((sx, sy), (120, 255, 120))
            angle += data['angle_increment']

        pygame.draw.rect(screen, (40, 44, 54), (panel_x - 2, panel_y - 2, panel_width + 4, panel_height + 4), 0, border_radius=6)
        pygame.draw.rect(screen, (18, 20, 24), (panel_x, panel_y, panel_width, panel_height), 0, border_radius=6)

        map_data = data['map_data']
        map_w = data['map_width']
        map_h = data['map_height']
        if map_data and map_w > 0 and map_h > 0:
            raw_surface = pygame.Surface((map_w, map_h))
            for j in range(map_h):
                src_row = map_h - 1 - j
                row_offset = src_row * map_w
                for i in range(map_w):
                    occ = map_data[row_offset + i]
                    if occ < 0:
                        color = (125, 125, 125)
                    elif occ >= 65:
                        color = (30, 30, 30)
                    else:
                        color = (235, 235, 235)
                    raw_surface.set_at((i, j), color)

            scale = min(panel_width / map_w, panel_height / map_h)
            scaled_w = max(1, int(map_w * scale))
            scaled_h = max(1, int(map_h * scale))
            map_surface = pygame.transform.scale(raw_surface, (scaled_w, scaled_h))

            mx = panel_x + (panel_width - map_surface.get_width()) // 2
            my = panel_y + (panel_height - map_surface.get_height()) // 2
            screen.blit(map_surface, (mx, my))

            map_resolution = data['map_resolution']
            if map_resolution > 0.0:
                map_origin_x = data['map_origin_x']
                map_origin_y = data['map_origin_y']
                if data['robot_map_x'] is None or data['robot_map_y'] is None:
                    robot_map_x = None
                    robot_map_y = None
                else:
                    robot_map_x = (data['robot_map_x'] - map_origin_x) / map_resolution
                    robot_map_y = (data['robot_map_y'] - map_origin_y) / map_resolution

                if robot_map_x is not None and 0.0 <= robot_map_x < map_w and 0.0 <= robot_map_y < map_h:
                    image_x = robot_map_x
                    image_y = (map_h - 1) - robot_map_y

                    scale_x = map_surface.get_width() / float(map_w)
                    scale_y = map_surface.get_height() / float(map_h)
                    robot_px_map = int(mx + image_x * scale_x)
                    robot_py_map = int(my + image_y * scale_y)

                    pygame.draw.circle(screen, (255, 220, 80), (robot_px_map, robot_py_map), 4)

                    heading_len_map = 12
                    yaw_display = data['robot_map_yaw'] if data['robot_map_yaw'] is not None else data['yaw']
                    heading_x = int(robot_px_map + heading_len_map * math.cos(yaw_display))
                    heading_y = int(robot_py_map - heading_len_map * math.sin(yaw_display))
                    pygame.draw.line(
                        screen,
                        (255, 120, 80),
                        (robot_px_map, robot_py_map),
                        (heading_x, heading_y),
                        2,
                    )
        else:
            no_map = font.render('Waiting /map ...', True, (180, 180, 180))
            screen.blit(no_map, (panel_x + 90, panel_y + panel_height // 2 - 10))

        text1 = font.render(
            f'odom x={data["x"]:.3f}  y={data["y"]:.3f}  yaw={data["yaw"]:.3f} rad',
            True,
            (230, 230, 230),
        )
        text2 = font.render(
            f'scan points={len(data["ranges"])}',
            True,
            (190, 190, 190),
        )
        text3 = font.render('slam map', True, (220, 220, 220))

        screen.blit(text1, (14, 12))
        screen.blit(text2, (14, 36))
        screen.blit(text3, (panel_x + 8, panel_y - 22))

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()


def main() -> None:
    rclpy.init()
    node = OdomPygameVisualizer()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        draw(node)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
