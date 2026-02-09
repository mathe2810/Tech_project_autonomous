# Plan d'Implémentation SLAM - Based on ECE SLAM Lecture

## 📋 Objectifs du Projet
1. **Localization** : Position absolue du robot (x, y, θ) dans sa carte
2. **Mapping** : Créer une grille de probabilité occupancy grid 
3. **Simultaneous** : Sans hypothèses initiales, localiser en construisant la carte
4. **Planning** : Naviguer dans la carte incomplète

---

## 🏗️ Architecture Globale (d'après Nav2)

```
ESP32 (Sensors) 
    ↓ MicroROS
ROS2 Topics (/scan, /imu/data)
    ↓
┌─────────────────────────────────┐
│  SLAM Toolbox (simple_slam.py)  │ ← Localization + Mapping
│  - ICP scan matching            │
│  - Occupancy grid update        │
│  - Transform /map → /base_link  │
└─────────────────────────────────┘
    ↓
┌─────────────────────────────────┐
│  Nav2 Framework                 │ ← Planning + Control
│  - Planner Server (path)        │
│  - Controller Server (velocity) │
│  - BT Navigator                 │
└─────────────────────────────────┘
    ↓
Motor Commands (PWM) → ESP32
```

---

## 📍 Phase 1 : Localization (Sensor Fusion)

### 1.1 Capteurs Disponibles
- **LIDAR LD06** : 30 Hz, 360 points/scan, distance + intensité
- **IMU MPU6050** : Accéléromètre (100Hz raw), Gyro (100Hz raw), Magnéto
- **Aucun encodeur** : ❌ Pas d'odométrie directe

### 1.2 Stratégie de Fusion (Kalman Filter)

**Modèle d'état** (2D simplifié):
```
State = [x, y, θ, vx, vy, ωz]

Propagation (IMU @ 100Hz):
  x(k+1) = x(k) + vx(k)·Δt + 0.5·ax·Δt²
  y(k+1) = y(k) + vy(k)·Δt + 0.5·ay·Δt²
  θ(k+1) = θ(k) + ωz(k)·Δt
  
  vx(k+1) = vx(k) + ax·Δt
  vy(k+1) = vy(k) + ay·Δt
  ωz(k+1) = ωz(k) + αz·Δt
```

**Mesures disponibles**:
- **LIDAR** (30 Hz) : Relative position via ICP matching
  - Entrée : Deux scans consécutifs
  - Sortie : Δx, Δy, Δθ (transformation rigide)
  
- **IMU** (100 Hz) : Accélération + vitesse angulaire
  - Filtre EMA déjà implémenté (alpha=0.35/0.4)

**Processus de fusion** :
```python
# Pseudo-code Kalman Filter
def kalman_update(state, P, imu_accel, imu_gyro, lidar_delta_pose):
    # Prédiction (IMU)
    state_pred = propagate_imu(state, imu_accel, imu_gyro, dt=0.01)
    P_pred = F @ P @ F.T + Q_imu  # Q_imu = covariance IMU
    
    # Correction LIDAR (tous les 3-4 IMU samples)
    if lidar_measurement_available:
        innovation = lidar_delta_pose - H @ state_pred
        S = H @ P_pred @ H.T + R_lidar  # R_lidar = covariance LIDAR
        K = P_pred @ H.T @ inv(S)
        state = state_pred + K @ innovation
        P = (I - K @ H) @ P_pred
    else:
        state = state_pred
        P = P_pred
    
    return state, P, covariance
```

### 1.3 Implémentation dans simple_slam.py

**Classe IMULocalization** :
```python
class KalmanFilter2D:
    def __init__(self):
        self.state = np.array([0, 0, 0, 0, 0, 0])  # x,y,θ,vx,vy,ωz
        self.P = np.eye(6) * 0.1  # Covariance initiale
        self.Q = np.eye(6) * [0.001, 0.001, 0.0001, 0.01, 0.01, 0.01]  # Bruit IMU
        self.R = np.eye(3) * [0.01, 0.01, 0.005]  # Bruit LIDAR (ICP)
    
    def predict(self, ax, ay, gz, dt):
        # Mise à jour vitesses
        vx_new = self.state[3] + ax * dt
        vy_new = self.state[4] + ay * dt
        
        # Mise à jour positions
        x_new = self.state[0] + self.state[3] * dt + 0.5 * ax * dt**2
        y_new = self.state[1] + self.state[4] * dt + 0.5 * ay * dt**2
        θ_new = self.state[2] + self.state[5] * dt
        
        self.state = np.array([x_new, y_new, θ_new, vx_new, vy_new, gz])
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update_lidar(self, delta_x, delta_y, delta_theta):
        # Correction par LIDAR
        innovation = np.array([delta_x - self.state[0],
                               delta_y - self.state[1],
                               delta_theta - self.state[2]])
        
        K = self.P[:3, :3] @ inv(self.P[:3, :3] + self.R)
        self.state[:3] += (K @ innovation)
        self.P -= K @ self.P[:3, :3]
```

### 1.4 Résultats Attendus
- ✅ Position (x, y, θ) mise à jour en temps réel
- ✅ Uncertainty (covariance) estimée
- ✅ Publication sur `/odometry/filtered`

---

## 🗺️ Phase 2 : Mapping (Occupancy Grid)

### 2.1 Représentation de la Carte

**Grille de probabilité** :
```
Grid = 2D array, 50m × 50m, résolution 10cm = 500×500 cells

Cell[x, y] ∈ [0, 1]
  0.0 = Libre
  0.5 = Inconnu (prior)
  1.0 = Occupé
```

**Stockage mémoire** :
```
500 × 500 cells × float32 = 1 MB ✓ Acceptable
```

### 2.2 Modèle de Capteur LIDAR

**Rayon casting** (pour chaque point LIDAR):
```python
# Bresenham line from robot position to hit point
ray = bresenham_line(robot_pos, hit_point)

for cell in ray[:-1]:
    # Cellules traversées = libre
    p_free = 0.25  # Prior: 25% occupé
    update_cell(cell, p_free)

for cell in [hit_point]:
    # Dernière cellule = occupée
    p_occupied = 0.75  # Prior: 75% occupé
    update_cell(cell, p_occupied)
```

### 2.3 Mise à Jour Bayésienne

**Formule de Bayes** (comme dans le PDF):
```
P(m_i | z_{1:t}) = P(z_t | m_i) · P(m_i | z_{1:t-1}) / P(z_t)

Implémentation en log-odds (numériquement stable) :
```

```python
def update_occupancy_grid(grid, robot_pose, lidar_scan):
    for angle, distance in lidar_scan.points:
        if distance > MAX_RANGE:
            continue
        
        # Angle absolu
        world_angle = robot_pose.theta + angle
        hit_x = robot_pose.x + distance * cos(world_angle)
        hit_y = robot_pose.y + distance * sin(world_angle)
        
        # Rayon du robot au hit
        ray = bresenham_line(robot_pose.xy, (hit_x, hit_y))
        
        for cell in ray[:-1]:
            # Libre
            grid[cell] = occupancy_update(grid[cell], p_hit=0.2)
        
        # Occupé
        grid[ray[-1]] = occupancy_update(grid[ray[-1]], p_hit=0.8)

def occupancy_update(p_prior, p_hit):
    # Bayes rule en log-odds
    log_odds = log(p_prior / (1 - p_prior)) + log(p_hit / (1 - p_hit))
    p_post = 1 - 1 / (1 + exp(log_odds))
    return p_post
```

### 2.4 Publication ROS2

```python
# Topic: /map (OccupancyGrid)
map_msg = OccupancyGrid()
map_msg.header.frame_id = "map"
map_msg.info.resolution = 0.1  # 10cm
map_msg.info.width = 500
map_msg.info.height = 500
map_msg.info.origin.position.x = -25  # Centré
map_msg.info.origin.position.y = -25

# Convertir [0,1] → [0,100] pour OccupancyGrid
map_msg.data = (grid * 100).flatten().astype(int8)
```

### 2.5 Résultats Attendus
- ✅ Carte visible dans RViz (terrain libre/occupé)
- ✅ Mise à jour en temps réel (~10-20 FPS)
- ✅ Mémoire < 10 MB

---

## 🚀 Phase 3 : Autonomous Navigation (Mapping Circuit)

### 3.1 Stratégie de Couverture

Comme mentionné dans le PDF, plusieurs approches :

**Option 1 : Random Walk** (actuellement implémenté)
```
✓ Garantit couverture complète
✗ Inefficace, très long
```

**Option 2 : Spiral Coverage** (recommandé)
```cpp
// Pseudo-code
void spiral_mapping() {
    float radius = 0.5;  // 50cm initial
    
    while (radius < MAX_RADIUS) {
        // Tourner en spirale
        for (int i = 0; i < 8; i++) {
            angle = i * 45°;
            move_forward(radius);
            turn(45°);
        }
        radius += 1.0;  // Augmenter rayon
    }
}
```

**Option 3 : Wall Following**
```
Suivre le mur de gauche → couverture des frontières
```

### 3.2 Adaptation du main.cpp Existant

**État courant** :
```cpp
enum MappingState { IDLE, MOVING_FORWARD, TURNING, COMPLETING_CIRCUIT };

// Circuit simple : carré 4×4m, ~18 secondes
// forward 3s, turn 0.5s × 4 = circuit complet
```

**Amélioration vers spiral** :
```cpp
void motors_drive_spiral() {
    static float radius = 0.5;
    static int spiral_count = 0;
    
    if (spiral_count < 8) {
        motors_drive(200, 0);  // Forward
        spiral_count++;
    } else {
        motors_drive(0, 100);  // Turn
        radius += 0.2;
        spiral_count = 0;
    }
}
```

### 3.3 Détection de Boucle (Loop Closure)

**Problème** : Après compléter un circuit, le robot est à (0,0) mais SLAM pense être ailleurs → **drift**.

**Solution : Scan Matching Amélioré (ICP)**
```python
def detect_loop_closure(current_scan, reference_scans):
    """
    Comparer le scan actuel avec les anciens scans
    Si correspondance > seuil : boucle fermée !
    """
    for ref_scan in reference_scans[-20:]:  # Derniers 20 scans
        icp_error, transform = icp_align(current_scan, ref_scan)
        
        if icp_error < THRESHOLD:  # par ex. < 0.1m
            # Loop closure détecté !
            graph.add_constraint(current_pose, ref_pose, transform)
            optimize_graph()  # Corriger tout le chemin
            return True
    
    return False
```

---

## 🎯 Phase 4 : Navigation (Nav2 Integration)

### 4.1 Architecture

```
Simple SLAM (vous avez déjà ✓)
    ↓ /map, /odometry
Nav2 Framework
    ├─ Planner (Dijkstra, A*, RRT)
    ├─ Controller (DWA, PurePersuit)
    └─ BT Navigator (behavior tree)
    ↓ /cmd_vel
Motor Controller (dans main.cpp)
```

### 4.2 Installation Nav2

```bash
sudo apt install ros-humble-nav2*
sudo apt install ros-humble-slam-toolbox
```

### 4.3 Configuration (Nav2)

**Fichier `nav2_config.yaml`** :
```yaml
planner_server:
  plugins: ["GridBased"]
  GridBased:
    plugin: "nav2_theta_star_planner/ThetaStarPlanner"  # ou Dijkstra
    
controller_server:
  plugins: ["FollowPath"]
  FollowPath:
    plugin: "nav2_pure_pursuit_controller/PurePursuitController"
    desired_linear_vel: 0.5
    max_angular_vel: 2.0
```

### 4.4 Nœud ROS2 pour Moteurs

```python
class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.motor_pub = self.create_publisher(Int16MultiArray, '/motors', 10)
    
    def cmd_vel_callback(self, msg):
        """
        Twist msg (vx, ωz) → PWM commands
        Convertir vitesses en PWM pour moteurs
        """
        # v = 0.5 m/s → throttle = 200 PWM
        throttle = int(msg.linear.x / 0.5 * 200)
        
        # ω = 1.0 rad/s → steering = 100 PWM (radius = ~0.5m)
        steering = int(msg.angular.z / 1.0 * 100)
        
        self.motor_pub.publish(
            Int16MultiArray(data=[throttle, steering])
        )
```

---

## 📊 Résumé des Étapes d'Implémentation

| Phase | Composant | Priorité | Effort | État |
|-------|-----------|----------|--------|------|
| 1 | Kalman Filter (IMU + LIDAR) | 🔴 Critical | 40h | ⏳ À faire |
| 2 | Occupancy Grid Mapping | 🟠 High | 30h | ⏳ Partial |
| 3 | Loop Closure Detection | 🟡 Medium | 20h | ❌ Absent |
| 4 | Spiral Coverage | 🟡 Medium | 10h | ❌ Absent |
| 5 | Nav2 Integration | 🟢 Low | 25h | ❌ Absent |

---

## 🔧 Fichiers à Créer/Modifier

### À Créer :
1. **`localization_node.py`** - Kalman Filter pour fusion capteurs
2. **`mapping_node.py`** - Mise à jour occupancy grid (amélioration simple_slam.py)
3. **`loop_closure_node.py`** - Détection de boucles
4. **`nav2_config.yaml`** - Configuration Nav2

### À Modifier :
1. **`main.cpp`** - Ajouter spiral au lieu de circuit simple
2. **`simple_slam.py`** - Ajouter Kalman Filter
3. **`start_stack.sh`** - Ajouter nouveaux nœuds

---

## 🎓 Références du PDF

- **Kalman Filtering** : Slide 14-15 (fusion de capteurs)
- **Bayesian Mapping** : Slide 22-24 (occupancy grid)
- **Coverage Strategies** : Slide 28-29 (spiral, wall-following)
- **Planning While Mapping** : Slide 32-40 (trajectory computation)
- **Nav2 Framework** : Slide 47-50 (architecture globale)

---

## 📝 Checklist de Validation

- [ ] Kalman Filter compile et publie /odometry/filtered
- [ ] Occupancy grid visible dans RViz
- [ ] Spiral mapping parcourt toute la zone
- [ ] Loop closure détecte le retour au point de départ
- [ ] Erreur de fermeture de boucle < 0.5m
- [ ] Nav2 génère des trajectoires valides
- [ ] Robot suit les commandes /cmd_vel de Nav2

---

## 🚀 Prochaines Étapes Immédiates

1. **Cette semaine** : Implémenter Kalman Filter dans simple_slam.py
2. **Semaine 2** : Tester mapping avec spiral coverage
3. **Semaine 3** : Intégrer loop closure detection
4. **Semaine 4** : Installer et configurer Nav2
5. **Semaine 5** : Tests d'intégration complets

Bonne implémentation ! 🎯
