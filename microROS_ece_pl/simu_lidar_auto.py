#!/usr/bin/env python3
import pygame, math, socket, pickle, struct, os
import numpy as np

# --- CONFIG ---
WIDTH, HEIGHT = 800, 600
MAP_RES       = 0.0133
NUM_RAYS      = 180
CIRCUIT_PATH  = "image_corridor_grossi.png"

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock  = pygame.time.Clock()
font = pygame.font.SysFont("monospace", 16, bold=True)

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
start_pos = [270.0, 250.0]  # 🎯 POINT DE DÉPART - pour comparaison visuelle
lidar_conn = None

# --- PARAMÈTRES AUTONOMIE (EXACTEMENT wall_centering_node) ---
CRUISE_SPEED = 0.7        # 🐢 Vitesse réduite pour meilleure précision RF2O
ALPHA_ROT = 0.95          # Amortissement très fort (comme wall_centering)
DEADZONE = 0.15           # Zone morte AUGMENTÉE (mètres) - pour rester au centre sans osciller
GAIN_ROT = 0.4            # Gain très doux pour "glisser" dans les virages
MIN_ROTATION = 0.38       # Seuil minimal pour faire bouger les moteurs
MAX_LINEAR_SPEED = 0.7    # Cap global vitesse linéaire (px/frame)
MAX_ANGULAR_SPEED = 0.03  # Cap global rotation (rad/frame)

# DÉLAI DE DÉMARRAGE - Attend que SLAM soit prêt avant d'activer autonome
STARTUP_DELAY_SECONDS = 15.0  # ⏱️ 15 secondes pour laisser SLAM s'initialiser complètement

# ARRÊT AUTOMATIQUE AU RETOUR - Pour mapper un circuit complet
ENABLE_AUTO_STOP = True        # 🔄 True = Arrêt auto en 2 phases (retour puis +2m), False = boucle continue
AUTO_STOP_DISTANCE = 0.05      # Distance au départ pour arrêt automatique (mètres)
MIN_DISTANCE_TRAVELED = 3.0   # Distance minimale avant de permettre l'arrêt (évite arrêt immédiat)
EXTRA_DISTANCE_AFTER_RETURN = 2.0  # Distance à parcourir APRÈS retour au départ avant arrêt (mètres)

auto_mode = False         # ⏳ Démarre en MANUEL, s'activera automatiquement après délai
startup_timer = 0.0       # Compteur pour activer auto après délai
max_distance_reached = 0.0  # Distance max atteinte depuis le départ
prev_w = 0.0              # Rotation précédente (filtre)
loop_return_detected = False
extra_distance_after_return = 0.0
last_robot_pos_for_stop = robot_pos.copy()

def get_lidar_ranges(pos, theta):
    ranges = []
    max_range = 2.0  # Portée LIDAR (m)
    for i in range(NUM_RAYS):
        angle = (2.0 * math.pi * i / NUM_RAYS) + theta
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        dist = max_range
        # Step=1 au lieu de 2 pour ne pas sauter de pixels dans les coins
        for r in range(5, int(max_range/MAP_RES), 1):
            tx, ty = int(pos[0] + r*cos_a), int(pos[1] + r*sin_a)
            if not (0 <= tx < WIDTH and 0 <= ty < HEIGHT) or circuit_surf.get_at((tx, ty))[0] < 80:
                dist = r * MAP_RES
                break
        ranges.append(float(dist))
    return ranges

def autonomous_control(ranges):
    """
    Logique de centrage dans le couloir (EXACTEMENT comme wall_centering_node)
    Retourne: (vitesse_lineaire, vitesse_angulaire)
    """
    global prev_w
    
    ranges = np.array(ranges)
    # IMPORTANT: Pas de limite haute! On limite seulement à 2.0m pour les valeurs invalides
    # Comme dans wall_centering_node: ranges = np.where(np.isfinite(ranges) & (ranges > 0.15), ranges, 2.0)
    ranges = np.where((ranges > 0.15) & np.isfinite(ranges), ranges, 2.0)
    
    # Dans notre simulateur: rayon 0 = avant (angle robot_th)
    idx_front = 0
    
    # On regarde sur les côtés (60°) mais avec une FENÊTRE ÉTROITE
    # pour ne pas voir les murs au loin qui gênent lors des virages
    side_angle = NUM_RAYS // 6      # 60° = 180/6 = 30 rayons (direction latérale)
    window = int(NUM_RAYS * 10 / 180)  # RÉDUIT à 10° (au lieu de 20°) pour ne voir que le mur IMMÉDIAT
    
    # Indices comme avant (ce qui marchait mieux)
    idx_left = (idx_front - side_angle) % NUM_RAYS
    idx_right = (idx_front + side_angle) % NUM_RAYS
    
    # Mesure de la distance aux murs latéraux (moyenne sur ±10° seulement)
    # Cela évite de voir le mur du virage au loin pendant les zigzag
    left_start = (idx_left - window) % NUM_RAYS
    left_end = (idx_left + window) % NUM_RAYS
    right_start = (idx_right - window) % NUM_RAYS
    right_end = (idx_right + window) % NUM_RAYS
    
    # Gestion des indices circulaires
    if left_start < left_end:
        dist_l = np.mean(ranges[left_start:left_end])
    else:
        dist_l = np.mean(np.concatenate([ranges[left_start:], ranges[:left_end]]))
    
    if right_start < right_end:
        dist_r = np.mean(ranges[right_start:right_end])
    else:
        dist_r = np.mean(np.concatenate([ranges[right_start:], ranges[:right_end]]))
    
    # Distance devant pour freiner si le virage est trop serré (±10 rayons)
    dist_f = np.mean(ranges[max(0, idx_front-10):min(NUM_RAYS, idx_front+10)])
    
    # --- LOGIQUE SIMPLE (COMME WALL_CENTERING_NODE) ---
    # Pas de modes compliqués, juste centrage progressif
    error = dist_l - dist_r
    
    if abs(error) < DEADZONE:
        target_w = 0.0
    else:
        # Correction proportionnelle très douce
        target_w = -error * GAIN_ROT
        if abs(target_w) < MIN_ROTATION:
            target_w = np.sign(target_w) * MIN_ROTATION
    
    emergency_mode = False
    warning_mode = False
    
    # --- FILTRE ANTI-ZIGZAG (SIMPLE COMME WALL_CENTERING) ---
    smoothed_w = (ALPHA_ROT * prev_w) + ((1 - ALPHA_ROT) * target_w)
    
    # Limite accélération rotation
    max_delta = 0.04  # Comme wall_centering_node
    diff = smoothed_w - prev_w
    if abs(diff) > max_delta:
        smoothed_w = prev_w + np.sign(diff) * max_delta
    
    prev_w = smoothed_w
    
    # Vitesse linéaire (SIMPLE: ralentit juste si mur devant)
    if dist_f > 0.8:
        speed = CRUISE_SPEED
    else:
        speed = CRUISE_SPEED * 0.7   # Ralenti si mur devant
    
    # Limite rotation avec cap global
    rotation = np.clip(smoothed_w, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
    speed = min(speed, MAX_LINEAR_SPEED)
    
    return speed, rotation, dist_l, dist_r, dist_f, emergency_mode, warning_mode

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def angle_to_quaternion(angle):
    half_angle = angle / 2.0
    return {'z': math.sin(half_angle), 'w': math.cos(half_angle)}

def draw_hud(auto_mode, speed, rotation, dist_l, dist_r, dist_f, emergency_mode=False, warning_mode=False, dist_to_start=0, startup_timer=0, startup_delay=5.0):
    """Affiche les infos de débogage"""
    y_offset = 10
    
    # Affiche compte à rebours si pas encore en auto
    if not auto_mode and startup_timer < startup_delay:
        remaining = startup_delay - startup_timer
        texts = [
            f"⏳ ATTENTE SLAM: {remaining:.1f}s restantes...",
            f"MODE: MANUEL (auto dans {remaining:.1f}s)",
            f"🎯 Distance au départ: {dist_to_start:.2f}m"
        ]
    else:
        texts = [
            f"MODE: {'AUTONOME' if auto_mode else 'MANUEL'} (A pour changer)",
            f"Vitesse: {speed:.2f} px/f ({speed*0.0133*30:.2f} m/s)",
            f"Rotation: {rotation:.3f} rad/s",
            f"[ROUGE=Devant] F:{dist_f:.2f}m",
            f"[VERT=Gauche] L:{dist_l:.2f}m",
            f"[BLEU=Droite] R:{dist_r:.2f}m",
            f"Centrage: {(dist_l-dist_r):.2f}m (L-R)",
            f"🎯 Distance au départ: {dist_to_start:.2f}m"
        ]
    
    # Ajout des alertes si nécessaire
    if emergency_mode:
        texts.insert(1, "🚨 URGENCE CRITIQUE: <0.4m! 🚨")
    elif warning_mode:
        texts.insert(1, "⚠ ALERTE: Mur proche (<0.55m)")
    
    for i, txt in enumerate(texts):
        if i == 0:
            color = (0, 255, 0) if auto_mode else (255, 255, 255)
        elif emergency_mode and i == 1:
            color = (255, 0, 0)  # ROUGE VIF pour urgence critique
        elif warning_mode and i == 1:
            color = (255, 165, 0)  # ORANGE pour alerte
        elif i == 3 or ((emergency_mode or warning_mode) and i == 4):
            color = (255, 100, 100)  # Rouge pour F
        elif i == 4 or ((emergency_mode or warning_mode) and i == 5):
            color = (100, 255, 100)  # Vert pour L
        elif i == 5 or ((emergency_mode or warning_mode) and i == 6):
            color = (100, 100, 255)  # Bleu pour R
        elif i == 6 or ((emergency_mode or warning_mode) and i == 7):
            error = dist_l - dist_r
            color = (0, 255, 0) if abs(error) < 0.08 else (255, 150, 0)
        else:
            color = (255, 255, 255)
        
        rendered = font.render(txt, True, color, (0, 0, 0))
        screen.blit(rendered, (10, y_offset + i * 20))

# --- BOUCLE PRINCIPALE ---
run = True
speed, rotation, dist_l, dist_r, dist_f = 0, 0, 0, 0, 0
emergency_mode = False
warning_mode = False

while run:
    # ⏱️ AUTO-ACTIVATION APRÈS DÉLAI (pour laisser SLAM s'initialiser)
    if not auto_mode and startup_timer < STARTUP_DELAY_SECONDS:
        startup_timer += 1.0/30.0  # Incrémente (30 FPS)
        if startup_timer >= STARTUP_DELAY_SECONDS:
            auto_mode = True
            print(f"🟢 AUTO MODE ACTIVÉ après {STARTUP_DELAY_SECONDS}s - SLAM prêt!")
    
    if not lidar_conn:
        try: 
            lidar_conn, _ = server_sock.accept()
        except: 
            pass

    for e in pygame.event.get():
        if e.type == pygame.QUIT: 
            run = False
        if e.type == pygame.KEYDOWN:
            if e.key == pygame.K_a:  # Toggle autonome
                auto_mode = not auto_mode
                prev_w = 0.0  # Reset filtre

    # --- CONTRÔLE ---
    ranges = get_lidar_ranges(robot_pos, robot_th)
    
    if auto_mode:
        # Mode autonome
        speed, rotation, dist_l, dist_r, dist_f, emergency_mode, warning_mode = autonomous_control(ranges)
        robot_pos[0] += speed * math.cos(robot_th)
        robot_pos[1] += speed * math.sin(robot_th)
        robot_th += rotation
        robot_th = normalize_angle(robot_th)
    else:
        # Mode manuel (clavier)
        k = pygame.key.get_pressed()
        if k[pygame.K_LEFT]:  robot_th -= MAX_ANGULAR_SPEED
        if k[pygame.K_RIGHT]: robot_th += MAX_ANGULAR_SPEED
        if k[pygame.K_UP]:
            robot_pos[0] += MAX_LINEAR_SPEED * math.cos(robot_th)
            robot_pos[1] += MAX_LINEAR_SPEED * math.sin(robot_th)
        speed = MAX_LINEAR_SPEED if k[pygame.K_UP] else 0.0
        rotation = 0.0
        emergency_mode = False
        warning_mode = False
        dist_l, dist_r, dist_f = 0.0, 0.0, 0.0

    # --- AFFICHAGE ---
    screen.blit(circuit_surf, (0,0))
    
    # Affichage zones de détection en mode autonome
    if auto_mode and len(ranges) > 0:
        cx, cy = int(robot_pos[0]), int(robot_pos[1])
        
        # Zone DEVANT (rouge)
        for i in range(-10, 10):
            idx = i % NUM_RAYS
            r = ranges[idx]
            if r < 2.0:
                angle = (2.0 * math.pi * idx / NUM_RAYS) + robot_th
                px = int(robot_pos[0] + r*75*math.cos(angle))
                py = int(robot_pos[1] + r*75*math.sin(angle))
                pygame.draw.circle(screen, (255, 0, 0), (px, py), 3)
        
        # Zone GAUCHE (vert)
        offset_60deg = NUM_RAYS // 6
        idx_left = -offset_60deg % NUM_RAYS
        for i in range(-10, 10):
            idx = (idx_left + i) % NUM_RAYS
            r = ranges[idx]
            if r < 2.0:
                angle = (2.0 * math.pi * idx / NUM_RAYS) + robot_th
                px = int(robot_pos[0] + r*75*math.cos(angle))
                py = int(robot_pos[1] + r*75*math.sin(angle))
                pygame.draw.circle(screen, (0, 255, 0), (px, py), 3)
        
        # Zone DROITE (bleu)
        idx_right = offset_60deg % NUM_RAYS
        for i in range(-10, 10):
            idx = (idx_right + i) % NUM_RAYS
            r = ranges[idx]
            if r < 2.0:
                angle = (2.0 * math.pi * idx / NUM_RAYS) + robot_th
                px = int(robot_pos[0] + r*75*math.cos(angle))
                py = int(robot_pos[1] + r*75*math.sin(angle))
                pygame.draw.circle(screen, (0, 0, 255), (px, py), 3)
    
    # 🎯 POINT DE DÉPART (en JAUNE)
    pygame.draw.circle(screen, (255, 255, 0), (int(start_pos[0]), int(start_pos[1])), 12, 3)  # Cercle jaune vide
    pygame.draw.circle(screen, (255, 255, 0), (int(start_pos[0]), int(start_pos[1])), 5)      # Point jaune plein
    
    # Calcul distance au départ
    dist_to_start = math.sqrt((robot_pos[0]-start_pos[0])**2 + (robot_pos[1]-start_pos[1])**2)
    dist_m = dist_to_start * MAP_RES
    
    # 🔄 ARRÊT AUTOMATIQUE EN 2 PHASES
    # 1) Détecte le retour près du départ après un tour complet
    # 2) Continue encore EXTRA_DISTANCE_AFTER_RETURN puis stop
    dx_step = robot_pos[0] - last_robot_pos_for_stop[0]
    dy_step = robot_pos[1] - last_robot_pos_for_stop[1]
    step_m = math.sqrt(dx_step*dx_step + dy_step*dy_step) * MAP_RES
    last_robot_pos_for_stop = robot_pos.copy()

    if auto_mode and dist_m > max_distance_reached:
        max_distance_reached = dist_m

    if ENABLE_AUTO_STOP and auto_mode:
        if (not loop_return_detected) and max_distance_reached > MIN_DISTANCE_TRAVELED and dist_m < AUTO_STOP_DISTANCE:
            loop_return_detected = True
            extra_distance_after_return = 0.0
            print(f"\n🎯 RETOUR AU DÉPART DÉTECTÉ! Distance: {dist_m:.2f}m (max atteint: {max_distance_reached:.2f}m)")
            print(f"➡️  Continue encore {EXTRA_DISTANCE_AFTER_RETURN:.2f}m pour compléter la carte...")
        elif loop_return_detected:
            extra_distance_after_return += step_m
            if extra_distance_after_return >= EXTRA_DISTANCE_AFTER_RETURN:
                print(
                    f"✅ DISTANCE SUPPLÉMENTAIRE ATTEINTE: {extra_distance_after_return:.2f}m "
                    f"(objectif {EXTRA_DISTANCE_AFTER_RETURN:.2f}m)"
                )
                print("🛑 Arrêt automatique - sauvegarde finale")
                run = False
    
    # Robot (bleu)
    pygame.draw.circle(screen, (0,0,255), (int(robot_pos[0]), int(robot_pos[1])), 10)
    
    # Trait rouge devant
    front_x = int(robot_pos[0] + 15 * math.cos(robot_th))
    front_y = int(robot_pos[1] + 15 * math.sin(robot_th))
    pygame.draw.line(screen, (255,0,0), (int(robot_pos[0]), int(robot_pos[1])), (front_x, front_y), 3)
    
    # HUD
    draw_hud(auto_mode, speed, rotation, dist_l, dist_r, dist_f, emergency_mode, warning_mode, dist_m, startup_timer, STARTUP_DELAY_SECONDS)
    
    pygame.display.flip()

    # --- ENVOI SOCKET ---
    if lidar_conn:
        try:
            q = angle_to_quaternion(robot_th)
            data = {
                'ranges': ranges,
                'pose': {
                    'x': robot_pos[0] * MAP_RES,
                    'y': robot_pos[1] * MAP_RES,
                    'theta': normalize_angle(robot_th),
                    'quaternion': [0, 0, q['z'], q['w']]
                }
            }
            p = pickle.dumps(data)
            lidar_conn.sendall(struct.pack('>I', len(p)) + p)
        except: 
            lidar_conn = None

    clock.tick(30)

pygame.quit()
