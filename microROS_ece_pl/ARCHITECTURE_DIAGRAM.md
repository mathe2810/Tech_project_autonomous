# Architecture Multithreading - Schéma de Flux

## 🏗️ Architecture Globale

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32 (Dual Core)                    │
├──────────────────────────────┬──────────────────────────────┤
│         CŒUR 0               │         CŒUR 1               │
│      (micro_ros)             │      (LIDAR)                 │
├──────────────────────────────┼──────────────────────────────┤
│                              │                              │
│  WiFi ──► micro_ros          │   UART ──► Parser LD06       │
│  |        |                  │   |        |                 │
│  └─► RCL Engine              │   └─► CRC8 Validation        │
│      |                       │        |                     │
│      ├─► Publisher /data     │        └─► Mutex Semaphore   │
│      └─► Publisher /scan◄────┼────────────────┐             │
│                              │                |             │
│  Thread-Safe Read (Mutex)    │   Thread-Safe Write (Mutex)  │
│                              │                              │
└──────────────────────────────┴──────────────────────────────┘
         ▲
         │ WiFi
         │
   ┌─────┴─────────┐
   │ ROS2 Agent    │
   │ (Linux/Mac)   │
   └───────────────┘
```

## 📊 Flux de Données - Étapes Séquentielles

### Sequence 1: Startup (0-2s)

```
[SETUP]
  │
  ├─► Serial.begin(115200)
  │
  ├─► lidar_init()  [CŒUR 1]
  │   ├─► LIDAR_SERIAL.begin(230400)
  │   ├─► lidar_mutex = xSemaphoreCreateMutex()
  │   └─► xTaskCreatePinnedToCore(lidarTask, ..., core=1)
  │
  ├─► WiFi.begin(SSID, PASSWORD)
  │
  └─► set_microros_wifi_transports(...)
```

### Sequence 2: Runtime Normal

```
┌─────────────────────────────────────────────┐
│ CŒUR 1: Continuous LIDAR Reading            │
└─────────────────────────────────────────────┘
  │
  ├─► lidarTask() loop
  │   │
  │   ├─► Wait: LIDAR_SERIAL.available()
  │   │   │
  │   │   └─► Read byte-by-byte (non-blocking)
  │   │
  │   ├─► State Machine Parse
  │   │   │
  │   │   ├─ FIND_HEADER (0x54)
  │   │   │
  │   │   ├─ READ_FIXED (6 bytes)
  │   │   │  └─ Version, Speed, StartAngle
  │   │   │
  │   │   ├─ READ_POINTS (3 × N bytes)
  │   │   │  └─ Distance (2B), Confidence (1B) × 12
  │   │   │
  │   │   └─ READ_TAIL (5 bytes)
  │   │      └─ EndAngle, Timestamp, CRC8
  │   │
  │   ├─► CRC8 Validation ✓
  │   │   │
  │   │   └─ Table lookup: O(1)
  │   │
  │   ├─ xSemaphoreTake(lidar_mutex, 5ms) ◄─ LOCK
  │   │   │
  │   │   ├─ lidar_latest_distance = dist
  │   │   ├─ lidar_latest_angle = angle
  │   │   ├─ lidar_latest_confidence = conf
  │   │   └─ lidar_data_ready = true
  │   │
  │   └─ xSemaphoreGive(lidar_mutex) ◄─ UNLOCK
  │
  └─► vTaskDelay(1ms)
      │
      └─ CPU dormant 99% du temps (efficient)


┌─────────────────────────────────────────────┐
│ CŒUR 0: Continuous ROS Publishing (1 Hz)    │
└─────────────────────────────────────────────┘
  │
  ├─► loop()
  │   │
  │   ├─► switch(state)
  │   │   │
  │   │   ├─► WAITING_AGENT
  │   │   │   │
  │   │   │   └─ rmw_uros_ping_agent() ──► Try connect
  │   │   │
  │   │   └─► AGENT_CONNECTED
  │   │       │
  │   │       ├─ Publish /data (counter)
  │   │       │   │
  │   │       │   └─ msg_int.data = counter++
  │   │       │
  │   │       ├─ Publish /scan (LIDAR) [IF ENABLED]
  │   │       │   │
  │   │       │   ├─ xSemaphoreTake(lidar_mutex, 10ms) ◄─ LOCK
  │   │       │   │   │
  │   │       │   │   ├─ IF lidar_data_ready
  │   │       │   │   │   │
  │   │       │   │   │   ├─ msg_lidar.header.frame_id = "lidar_link"
  │   │       │   │   │   ├─ msg_lidar.angle_min = 0.0
  │   │       │   │   │   ├─ msg_lidar.angle_max = 2π
  │   │       │   │   │   ├─ msg_lidar.range_min = 0.06m
  │   │       │   │   │   ├─ msg_lidar.range_max = 12m
  │   │       │   │   │   │
  │   │       │   │   │   ├─ dist_m = distance_mm / 1000.0
  │   │       │   │   │   │
  │   │       │   │   │   └─ rcl_publish(&pub_lidar, &msg_lidar, NULL)
  │   │       │   │   │
  │   │       │   │   └─ lidar_data_ready = false
  │   │       │   │
  │   │       │   └─ xSemaphoreGive(lidar_mutex) ◄─ UNLOCK
  │   │       │
  │   │       └─ delay(1000ms) ──► Next publish cycle
  │   │
  │   └─► rclc_executor_spin_some()  ◄─ Process ROS callbacks
  │
  └─► Repeat ∞
```

## 🔐 Synchronisation Mutex - Détail

### Timeline d'Accès Concurrent

```
┌─── CŒUR 1 (Écriture)                 ┌─── CŒUR 0 (Lecture)
│                                       │
│ [T0] lidarTask() spin                 │ [T0] loop() in AGENT_CONNECTED
│      │
│      ├─ Parse frame LD06              │
│      │  (état machine)                │
│      │  ✓ CRC validated               │
│      │                                │
│ [T1] xSemaphoreTake() ◄─ LOCK         │
│      │                                │ [T1] xSemaphoreTake()
│      │                                │      → WAIT (blocked)
│      │                                │
│ [T2] Distance = 320mm                 │
│      Angle = 45.2°                    │ [T2] WAITING FOR LOCK...
│      Confidence = 180                 │
│      ready = true                     │
│      │                                │
│ [T3] xSemaphoreGive() ◄─ UNLOCK       │
│      │                                │ [T3] xSemaphoreTake() ✓
│      │                                │      Distance_copy = 320mm
│      │                                │      ready_copy = true
│      │                                │
│ [T4] vTaskDelay(1ms)                  │ [T4] xSemaphoreGive()
│      CPU sleep                        │
│                                       │      ready = false
│                                       │      rcl_publish()
│                                       │
│                                       │ [T5] delay(1000ms) before next
└───────────────────────────────────────┴───────────────────────

Timing:
- Cœur 1: Acquiert mutex ~100µs
- Cœur 0: Acquiert mutex ~50µs
- Pas de deadlock (timeout: 5-10ms)
- Pas de spinlock (µcontroller-friendly)
```

## 📈 Fréquences de Données

```
LIDAR (Cœur 1)
  │
  └─► ~375 Hz (12 points × 31 frames/sec)
      │
      └─ Stocker 1 point / cycle
         (ou decimation: chaque 2e point)
         

ROS Publisher (Cœur 0)
  │
  └─► 1 Hz (delay 1000ms)
      │
      └─ Publier 1 point LiDAR / seconde
         + 1 compteur / seconde
```

## 🛡️ Sécurité Thread-Safe

### Race Condition Prevention

```
❌ SANS Mutex (DATA RACE):
   Cœur 1: distance = 320
   Cœur 0: printf(distance)  ──► UNDEFINED !
                                 Peut être: 0, 320, garbage, etc.

✅ AVEC Mutex (SAFE):
   Cœur 1: xSemaphoreTake()
           distance = 320
           xSemaphoreGive()
   
   Cœur 0: xSemaphoreTake()
           copy = distance  ──► Toujours 320 !
           xSemaphoreGive()
```

### Deadlock Prevention

```
Timeout Courts (5-10ms):
  - Cœur 1: Libère mutex en <100µs
  - Cœur 0: Libère mutex en <50µs
  - Si timeout → Continueer sans mutex plutôt que bloquer ∞

Priorité Cœur 1 = 2 (élevée):
  - Parsing temps-réel = prioritaire
  - ROS (Cœur 0) = priorité 1 (normal)
  - Garantit LIDAR ne perd pas frames
```

## 📊 Bande Passante WiFi

```
Publication Rates:

/data (Int32)
  ├─ Payload: ~4 bytes
  ├─ Frequency: 1 Hz
  └─ BW: ~32 bps (negligible)

/scan (LaserScan)
  ├─ Payload: ~2 KB (360 floats)
  ├─ Frequency: 1 Hz
  └─ BW: ~16 Kbps

Total: ~16 Kbps << 54 Mbps WiFi capacity ✓
```

## ✨ Avantages de cette Architecture

```
✓ Parallélisme: 2 tâches indépendantes
✓ Temps-réel: LIDAR prioritaire
✓ Pas de racine: Utilisation Mutex
✓ Performant: State machine (pas allocation)
✓ Robuste: CRC validation
✓ Scalable: Facile ajouter capteurs
✓ ROS2 compliant: Formats standard
```

---

**Diagramme créé**: 2026-01-19  
**Version**: 1.0
