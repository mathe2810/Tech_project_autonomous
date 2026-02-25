import pygame
import numpy as np
import math
from PIL import Image

# --- Paramètres du simulateur ---
WIDTH, HEIGHT = 800, 600  # Taille de la fenêtre (px)
MAP_RESOLUTION = 0.02     # 2 cm/pixel pour la carte d'occupation
ROBOT_RADIUS = 0  # sera calculé dynamiquement
LIDAR_RANGE = 200         # px (max range lidar)
LIDAR_ANGLE_STEP = 2      # degrés

# --- Initialisation Pygame ---
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Simu 2D Robot + Lidar")
clock = pygame.time.Clock()

# Initialisation police après pygame.init()
font = pygame.font.SysFont(None, 28)


# --- Charger une image de circuit ---
def draw_circuit(surface):
    # Charge l'image image.png et l'affiche comme fond
    img = pygame.image.load("image.png")
    img = pygame.transform.scale(img, (WIDTH, HEIGHT))
    surface.blit(img, (0, 0))


# --- Robot ---
# Trouver la largeur du corridor (noir) automatiquement
def find_corridor_width(img):
    arr = pygame.surfarray.array2d(img)
    # Cherche une ligne avec du noir
    for y in range(HEIGHT//2, HEIGHT):
        blacks = np.where(arr[y] == 0)[0]
        if len(blacks) > 1:
            return blacks[-1] - blacks[0]
    return 40  # valeur par défaut



# --- Position manuelle du robot ---
# Choisis ici la position de départ (x, y)
robot_pos = [110, 465]
robot_theta = 0  # radians

def corridor_width_at_pos(img, pos):
    arr = pygame.surfarray.array2d(img)
    y = int(pos[1])
    blacks = np.where(arr[y] == 0)[0]
    if len(blacks) > 1:
        return blacks[-1] - blacks[0]
    return 40  # valeur par défaut

def update_robot_size_and_pos(pos, surf):
    global robot_pos, ROBOT_RADIUS
    robot_pos = list(pos)
    robot_corridor_width = corridor_width_at_pos(surf, robot_pos)
    ROBOT_RADIUS = max(6, int(robot_corridor_width * 0.2 / 2))  # 40% de la largeur, minimum 6px

# Correction : dimensionnement initial sur une surface temporaire
surface_tmp = pygame.Surface((WIDTH, HEIGHT))
surface_tmp.fill((255,255,255))
draw_circuit(surface_tmp)
update_robot_size_and_pos(robot_pos, surface_tmp)

# --- Lidar simulation ---
def simulate_lidar(surface, pos, theta):
    scan = []
    for angle in range(0, 360, LIDAR_ANGLE_STEP):
        a = math.radians(angle) + theta
        for r in range(0, LIDAR_RANGE, 2):
            x = int(pos[0] + r * math.cos(a))
            y = int(pos[1] + r * math.sin(a))
            if x < 0 or x >= WIDTH or y < 0 or y >= HEIGHT:
                break
            color = surface.get_at((x, y))
            if color == (255,255,255,255):  # mur (blanc)
                scan.append((x, y))
                break
    return scan

# --- Boucle principale ---
run = True
while run:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            update_robot_size_and_pos(event.pos)
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        robot_theta -= 0.05
    if keys[pygame.K_RIGHT]:
        robot_theta += 0.05
    if keys[pygame.K_UP]:
        robot_pos[0] += int(2 * math.cos(robot_theta))
        robot_pos[1] += int(2 * math.sin(robot_theta))
    if keys[pygame.K_DOWN]:
        robot_pos[0] -= int(2 * math.cos(robot_theta))
        robot_pos[1] -= int(2 * math.sin(robot_theta))

    # Dessin
    screen.fill((255,255,255))
    draw_circuit(screen)
    # Correction : resize initial après dessin du circuit
    if pygame.time.get_ticks() < 100:
        update_robot_size_and_pos(robot_pos)
    # Le robot ne doit évoluer que sur le noir (libre)
    robot_color = screen.get_at((int(robot_pos[0]), int(robot_pos[1])))
    if robot_color == (255,255,255,255):
        pygame.draw.circle(screen, (255,0,0), (int(robot_pos[0]), int(robot_pos[1])), ROBOT_RADIUS)  # rouge si sur mur
    else:
        pygame.draw.circle(screen, (0,0,255), (int(robot_pos[0]), int(robot_pos[1])), ROBOT_RADIUS)  # bleu si sur piste
    # Lidar
    scan = simulate_lidar(screen, robot_pos, robot_theta)
    for pt in scan:
        pygame.draw.circle(screen, (255,0,0), pt, 2)

    # Affichage de la position du robot
    pos_text = font.render(f"Robot: x={int(robot_pos[0])} y={int(robot_pos[1])}", True, (0,0,0))
    screen.blit(pos_text, (10, 10))
    pygame.display.flip()
    clock.tick(30)

pygame.quit()

# --- Export carte d'occupation (noir = mur, blanc = libre) ---

def export_occupancy_map():
    # Recharge l'image pour être sûr d'avoir la map d'origine
    img = Image.open("image.png").convert("L")
    img = img.resize((WIDTH, HEIGHT), Image.NEAREST)
    arr = np.array(img)
    # Seuil pour binaire (noir = libre, blanc = mur) -> inversé pour ROS (noir = mur, blanc = libre)
    occ_map = np.where(arr < 128, 254, 0).astype(np.uint8)

    # Ajout du quadrillage visuel (même résolution que slam_toolbox)
    grid_px = int(0.05 / MAP_RESOLUTION)  # 0.05m/case
    for x in range(0, occ_map.shape[1], grid_px):
        occ_map[:, x] = 128  # gris pour la grille
    for y in range(0, occ_map.shape[0], grid_px):
        occ_map[y, :] = 128

    Image.fromarray(occ_map).save("map.pgm")
    with open("map.yaml", "w") as f:
        f.write(f"image: map.pgm\nresolution: {MAP_RESOLUTION}\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.5\nfree_thresh: 0.2\n")

# Pour exporter la map à partir de image.png :
export_occupancy_map()
