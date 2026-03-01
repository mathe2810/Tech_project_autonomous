#!/usr/bin/env python3
import pygame, math, socket, pickle, struct, os

# --- CONFIG ---
WIDTH, HEIGHT = 800, 600
MAP_RES       = 0.0133
NUM_RAYS      = 180  # 180 rayons suffisent largement et boostent les FPS
CIRCUIT_PATH  = "image_corridor_grossi.png"

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock  = pygame.time.Clock()

if os.path.exists(CIRCUIT_PATH):
    circuit_surf = pygame.image.load(CIRCUIT_PATH).convert()
    circuit_surf = pygame.transform.scale(circuit_surf, (WIDTH, HEIGHT))
else:
    circuit_surf = pygame.Surface((WIDTH, HEIGHT))
    circuit_surf.fill((255,255,255))

server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_sock.bind(('127.0.0.1', 5005))
server_sock.listen(1)
server_sock.setblocking(False)

robot_pos, robot_th = [270.0, 250.0], 0.0
lidar_conn = None

def get_lidar_ranges(pos, theta):
    ranges = []
    for i in range(NUM_RAYS):
        angle = (2.0 * math.pi * i / NUM_RAYS) + theta
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        dist = 6.0
        # Pas de 2 pixels pour une détection fine des murs
        for r in range(5, int(6.0/MAP_RES), 2):
            tx, ty = int(pos[0] + r*cos_a), int(pos[1] + r*sin_a)
            if not (0 <= tx < WIDTH and 0 <= ty < HEIGHT) or circuit_surf.get_at((tx, ty))[0] < 80:
                dist = r * MAP_RES
                break
        ranges.append(float(dist))
    return ranges

def normalize_angle(angle):
    """Normalise un angle en [-pi, pi]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def angle_to_quaternion(angle):
    """Convertit un angle 2D en quaternion ROS"""
    half_angle = angle / 2.0
    return {
        'z': math.sin(half_angle),
        'w': math.cos(half_angle)
    }

run = True
while run:
    if not lidar_conn:
        try: lidar_conn, _ = server_sock.accept()
        except: pass

    for e in pygame.event.get():
        if e.type == pygame.QUIT: run = False

    k = pygame.key.get_pressed()
    if k[pygame.K_LEFT]:  robot_th -= 0.08
    if k[pygame.K_RIGHT]: robot_th += 0.08
    if k[pygame.K_UP]:
        robot_pos[0] += 3.0 * math.cos(robot_th)
        robot_pos[1] += 3.0 * math.sin(robot_th)

    screen.blit(circuit_surf, (0,0))
    pygame.draw.circle(screen, (0,0,255), (int(robot_pos[0]), int(robot_pos[1])), 10)
    # Trait rouge indiquant le devant du robot
    front_x = int(robot_pos[0] + 15 * math.cos(robot_th))
    front_y = int(robot_pos[1] + 15 * math.sin(robot_th))
    pygame.draw.line(screen, (255,0,0), (int(robot_pos[0]), int(robot_pos[1])), (front_x, front_y), 3)
    pygame.display.flip()

    if lidar_conn:
        try:
            q = angle_to_quaternion(robot_th)
            # Convertir les positions de pixels en mètres
            data = {
                'ranges': get_lidar_ranges(robot_pos, robot_th),
                'pose': {
                    'x': robot_pos[0] * MAP_RES,
                    'y': robot_pos[1] * MAP_RES,
                    'theta': normalize_angle(robot_th),
                    'quaternion': [0, 0, q['z'], q['w']]
                }
            }
            p = pickle.dumps(data)
            lidar_conn.sendall(struct.pack('>I', len(p)) + p)
        except: lidar_conn = None
    clock.tick(30)
pygame.quit()