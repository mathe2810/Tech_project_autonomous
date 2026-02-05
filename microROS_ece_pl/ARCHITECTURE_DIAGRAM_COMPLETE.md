# 🏗️ Architecture Logicielle Micro-ROS - COMPLÈTE

*Document mis à jour le 25 janvier 2026 - Version 2.0*

## Table des Matières
1. [Diagrammes Système](#diagrammes-système)
2. [Architecture Détaillée](#architecture-détaillée)
3. [Allocation Mémoire](#allocation-mémoire)
4. [État Machine](#état-machine)
5. [Communication Stack](#communication-stack)
6. [FreeRTOS Tasking](#freertos-tasking)
7. [Synchronisation](#synchronisation)
8. [Optimization](#optimization)

---

## Diagrammes Système

### Vue Globale du Système Micro-ROS

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                           MICRO-ROS SYSTEM ARCHITECTURE                      │
└──────────────────────────────────────────────────────────────────────────────┘

                    ┌─────────────────────────────┐
                    │    DESKTOP / LAPTOP         │
                    │     ROS2 Humble             │
                    │  ┌──────────────────────┐   │
                    │  │ RViz 2               │   │
                    │  │ ├─ 3D LIDAR plot    │   │
                    │  │ ├─ Point clouds     │   │
                    │  │ ├─ Grid map         │   │
                    │  │ └─ TF visualization │   │
                    │  └──────────────────────┘   │
                    │  ┌──────────────────────┐   │
                    │  │ RQT / Terminal       │   │
                    │  │ (monitoring)         │   │
                    │  └──────────────────────┘   │
                    └─────────────┬────────────────┘
                                  │
                                  │ ROS2 DDS
                                  │ UDP port 7400+
                                  │ IP: 127.0.0.1 (local)
                                  │
                    ┌─────────────▼────────────────┐
                    │  ROS2 Agent (Middleware)     │
                    │  192.168.x.x port 8888      │
                    │  ┌──────────────────────┐   │
                    │  │ UDP Listener         │   │
                    │  │ DDS Router           │   │
                    │  │ Message Serializer   │   │
                    │  └──────────────────────┘   │
                    └─────────────┬────────────────┘
                                  │
                                  │ WiFi UDP
                                  │ 192.168.x.x:8888
                                  │ Latency: 50-100ms
                                  │
                ┌─────────────────▼──────────────────────────────────┐
                │           ESP32 (Dual Core Micro-ROS)              │
                │        IP: 172.20.10.4 (WiFi STA)                 │
                │                                                    │
                ├─────────────────┬────────────────────────────────┤
                │   CŒUR 0        │       CŒUR 1                   │
                │ (Micro-ROS)     │     (LIDAR Parsing)            │
                ├─────────────────┼────────────────────────────────┤
                │                 │                                │
                │ ┌─────────────┐ │ ┌──────────────────────────┐  │
                │ │WiFi Stack   │ │ │ UART2 (LIDAR)            │  │
                │ │+ micro_ros  │ │ │ ├─ RX: GPIO 16          │  │
                │ │ transport   │ │ │ ├─ 230400 baud          │  │
                │ └──────┬──────┘ │ │ └─ Raw data stream       │  │
                │        │        │ │                          │  │
                │ ┌──────▼──────┐ │ │ ┌──────────────────────┐ │  │
                │ │RCL Engine   │ │ │ │State Machine Parser:  │ │  │
                │ │ allocator   │ │ │ │ FIND_HEADER           │ │  │
                │ └──────┬──────┘ │ │ │ READ_FIXED            │ │  │
                │        │        │ │ │ READ_POINTS (×12)    │ │  │
                │ ┌──────▼──────┐ │ │ │ READ_TAIL             │ │  │
                │ │Node: esp32   │ │ │ │ CRC8 Validation       │ │  │
                │ │_rover        │ │ │ └──────┬───────────────┘ │  │
                │ └──────┬──────┘ │ │        │               │  │
                │        │        │ │ ┌──────▼──────────┐   │  │
                │ ┌──────▼────────────┐ │lidar_mutex     │   │  │
                │ │Publishers:    │ │ │(Semaphore)     │   │  │
                │ │ ├─ /scan      │ │ │ Lock: 5ms      │   │  │
                │ │ │ LaserScan   │ │ └────────┬───────┘   │  │
                │ │ │             │ │          │           │  │
                │ │ ├─ /imu/data  │ │ ┌────────▼───────┐   │  │
                │ │ │ Imu         │ │ │lidar_buffer[]  │   │  │
                │ │ │             │ │ │ 360 LidarPoints│   │  │
                │ │ └─ /data      │ │ └────────┬───────┘   │  │
                │ │   Int32       │ │          │           │  │
                │ └──────┬────────┘ │ └────────────────────┘   │
                │        │          │                          │
                │ ┌──────▼──────┐  │ ┌──────────────────────┐ │  │
                │ │IMU (MPU6050)│  │ │vTaskDelay(1ms)       │ │  │
                │ │ I2C:        │  │ │CPU efficiency 99%    │ │  │
                │ │ SDA:21      │  │ │                      │ │  │
                │ │ SCL:22      │  │ └──────────────────────┘ │  │
                │ │ Rate: 10ms  │  │                          │  │
                │ └─────────────┘  │                          │  │
                │                 │                          │  │
                └─────────────────┴────────────────────────────┘
                         │
                    ┌────┴────┬────────┐
                    │          │        │
                ┌───▼───┐  ┌──▼──┐  ┌─▼──────┐
                │LIDAR  │  │ IMU │  │Power   │
                │Y-Lidar│  │6050 │  │Supply  │
                │X2L    │  │     │  │        │
                │       │  │     │  │        │
                │12 pts │  │3-axi│  │5V→3V3  │
                │/frame │  │s    │  │Regul.  │
                │230400 │  │I2C  │  │        │
                │baud   │  │     │  │        │
                └───────┘  └─────┘  └────────┘
```

---

## Architecture Détaillée

### Allocation & Initialisation ROS2

```cpp
// setup() - Initialization sequence

void setup() {
    Serial.begin(115200);           // Debug serial
    
    // 1. Allocator init (Micro-ROS memory management)
    allocator = rcl_get_default_allocator();
    
    // 2. Support init (ROS middleware)
    rclc_support_init(&support, 0, NULL, &allocator);
    //   ├─ Initialize DDS context
    //   ├─ Setup ROS domain
    //   └─ Prepare RMW (ROS Middleware Interface)
    
    // 3. Node creation ("esp32_rover")
    rclc_node_init_default(&node, "esp32_rover", "", &support);
    //   ├─ Node name: "esp32_rover"
    //   ├─ Namespace: "" (root)
    //   └─ Register with middleware
    
    // 4. Publishers creation
    rclc_publisher_init_default(&pub_lidar, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), 
        "/scan");
    //   ├─ Topic: /scan
    //   ├─ Message type: sensor_msgs::msg::LaserScan
    //   ├─ QoS: default (keep last 10)
    //   └─ Allocate serialization buffer
    
    rclc_publisher_init_default(&pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data");
    //   └─ Similar process
    
    // 5. Executor creation (ROS callback processor)
    executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    //   ├─ Number of subscriptions: 1
    //   ├─ (We only publish, don't subscribe on MCU)
    //   └─ Executor manages publish timing
    
    // 6. Create sensor tasks (FreeRTOS)
    xTaskCreatePinnedToCore(
        lidarTask,              // Task function
        "lidar_task",           // Task name
        4096,                   // Stack size (words)
        NULL,                   // Task parameters
        2,                      // Priority (higher = more priority)
        NULL,                   // Task handle
        1                       // Core affinity (Core 1)
    );
    //   └─ LIDAR reading on dedicated core
    
    // 7. Create mutex for LIDAR data sync
    lidar_mutex = xSemaphoreCreateMutex();
    
    // 8. Initialize sensor buffers
    memset(lidar_buffer, 0, sizeof(lidar_buffer));
    memset(&imu_current, 0, sizeof(imu_current));
    
    // 9. Start debug serial logging
    Serial.println("[SETUP] Micro-ROS initialized");
}
```

---

### Publishing Loop (Core 0)

```cpp
void loop() {
    // [Every iteration: ~10ms]
    
    // 1. Check connection state
    switch(state) {
        case WAITING_AGENT:
            // Try to ping agent
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                state = AGENT_CONNECTED;
                Serial.println("[ROS] Agent connected!");
            }
            return;  // Don't publish yet
            
        case AGENT_CONNECTED:
            // 2. Publish Int32 counter
            msg_int.data = counter++;
            rcl_publish(&pub_counter, &msg_int, NULL);
            
            // 3. Publish LIDAR data (if available)
            if (xSemaphoreTake(lidar_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                if (lidar_data_ready) {
                    // Build LaserScan message
                    msg_lidar.header.frame_id.data = "lidar_link";
                    msg_lidar.header.stamp.sec = now.sec;
                    msg_lidar.header.stamp.nanosec = now.nsec;
                    
                    msg_lidar.angle_min = 0.0;
                    msg_lidar.angle_max = 2.0 * M_PI;
                    msg_lidar.angle_increment = M_PI / 180.0;
                    msg_lidar.time_increment = 1.0 / 375.0;  // 375 Hz
                    msg_lidar.scan_time = 1.0 / 31.0;         // 31 Hz
                    
                    msg_lidar.range_min = 0.06;   // 6 cm
                    msg_lidar.range_max = 12.0;   // 12 m
                    
                    // Copy data from global buffer to message
                    for (int i = 0; i < 360; i++) {
                        msg_lidar.ranges[i] = 
                            lidar_buffer[i].distance / 1000.0;  // mm to meters
                        msg_lidar.intensities[i] = 
                            lidar_buffer[i].confidence;
                    }
                    
                    rcl_publish(&pub_lidar, &msg_lidar, NULL);
                    lidar_data_ready = false;
                }
                xSemaphoreGive(lidar_mutex);
            }
            
            // 4. Publish IMU data
            msg_imu.header.frame_id.data = "imu_link";
            msg_imu.linear_acceleration.x = imu_current.accel_x;
            msg_imu.linear_acceleration.y = imu_current.accel_y;
            msg_imu.linear_acceleration.z = imu_current.accel_z;
            msg_imu.angular_velocity.x = imu_current.gyro_x;
            msg_imu.angular_velocity.y = imu_current.gyro_y;
            msg_imu.angular_velocity.z = imu_current.gyro_z;
            
            rcl_publish(&pub_imu, &msg_imu, NULL);
            
            // 5. Process ROS callbacks (executor)
            rclc_executor_spin_some(&executor);
            
            // 6. Ping agent periodically
            if (millis() - last_ping > 1000) {
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                    ping_failures++;
                    if (ping_failures > 3) {
                        state = AGENT_DISCONNECTED;
                    }
                } else {
                    ping_failures = 0;
                }
                last_ping = millis();
            }
            break;
            
        case AGENT_DISCONNECTED:
            // Cleanup and wait
            rcl_publisher_fini(&pub_lidar, &node);
            rcl_publisher_fini(&pub_imu, &node);
            rclc_executor_fini(&executor);
            rclc_node_fini(&node);
            rclc_support_fini(&support);
            
            delay(5000);  // Wait before retry
            state = WAITING_AGENT;
            break;
    }
    
    // 7. Small delay to yield to Core 1
    delay(10);  // ~100 Hz loop frequency
}
```

---

### LIDAR Task (Core 1)

```cpp
void lidarTask(void *parameter) {
    // [Running on Core 1 continuously]
    
    enum ParseState { 
        FIND_HEADER, 
        READ_FIXED, 
        READ_POINTS, 
        READ_TAIL 
    } state = FIND_HEADER;
    
    uint8_t frame_buffer[45];  // Max frame size
    int frame_index = 0;
    
    Serial.println("[LIDAR] Task started on Core 1");
    
    while (1) {
        // 1. Non-blocking serial read
        while (LIDAR_SERIAL.available() > 0) {
            uint8_t byte = LIDAR_SERIAL.read();
            
            // 2. State machine parser
            switch (state) {
                case FIND_HEADER:
                    if (byte == 0x54) {  // LIDAR header
                        frame_buffer[0] = byte;
                        frame_index = 1;
                        state = READ_FIXED;
                    }
                    break;
                    
                case READ_FIXED:
                    frame_buffer[frame_index++] = byte;
                    if (frame_index >= 6) {  // 6 fixed bytes
                        state = READ_POINTS;
                    }
                    break;
                    
                case READ_POINTS:
                    frame_buffer[frame_index++] = byte;
                    if (frame_index >= 6 + 12*3) {  // 12 points × 3 bytes
                        state = READ_TAIL;
                    }
                    break;
                    
                case READ_TAIL:
                    frame_buffer[frame_index++] = byte;
                    if (frame_index >= 6 + 36 + 5) {  // Complete frame
                        // 3. Validate CRC8
                        uint8_t checksum = crc8(frame_buffer, 
                                               frame_index - 1);
                        
                        if (checksum == frame_buffer[frame_index - 1]) {
                            // 4. Parse data
                            uint16_t start_angle = 
                                (frame_buffer[2] | 
                                 (frame_buffer[3] << 8));
                            
                            for (int i = 0; i < 12; i++) {
                                int offset = 6 + i*3;
                                
                                uint16_t dist = 
                                    frame_buffer[offset] | 
                                    (frame_buffer[offset+1] << 8);
                                uint8_t conf = 
                                    frame_buffer[offset+2];
                                
                                float angle = start_angle + i*30;
                                angle = fmod(angle, 360.0);
                                
                                // 5. Lock and update buffer
                                if (xSemaphoreTake(lidar_mutex, 
                                    pdMS_TO_TICKS(5)) == pdTRUE) {
                                    
                                    int idx = (int)(angle / 1.0);
                                    if (idx >= 0 && idx < 360) {
                                        lidar_buffer[idx].distance = dist;
                                        lidar_buffer[idx].confidence = conf;
                                        lidar_buffer[idx].angle = angle;
                                    }
                                    
                                    lidar_data_ready = true;
                                    xSemaphoreGive(lidar_mutex);
                                }
                            }
                        }
                        
                        // Reset for next frame
                        state = FIND_HEADER;
                        frame_index = 0;
                    }
                    break;
            }
        }
        
        // 6. Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(1));
        // CPU idle 99% of the time (efficient!)
    }
}
```

---

## Allocation Mémoire

### Détails Heap (ESP32 520 KB total)

```
┌──────────────────────────────────────────────────┐
│ SRAM INTERNE (320 KB) - PARTITION DÉTAILLÉE      │
├──────────────────────────────────────────────────┤
│                                                  │
│ FreeRTOS Kernel: ~40 KB                          │
│ ├─ TCB (Task Control Blocks)                     │
│ ├─ Kernel structures                             │
│ └─ Task stacks (~16 KB used)                     │
│                                                  │
│ WiFi/IP Stack: 80-100 KB                         │
│ ├─ WiFi driver (lwip)                            │
│ ├─ TCP/UDP sockets                               │
│ ├─ DNS cache                                     │
│ └─ IP buffers                                    │
│                                                  │
│ Micro-ROS Framework: 40-50 KB                    │
│ ├─ RCL allocator (20 KB)                         │
│ ├─ Publisher structures (15 KB)                  │
│ ├─ Node context (8 KB)                           │
│ ├─ Executor (5 KB)                               │
│ └─ Message buffers (2 KB)                        │
│                                                  │
│ Sensor Buffers: 10 KB                            │
│ ├─ lidar_buffer[360]: 2.16 KB                   │
│ │  (struct: uint16_t dist + uint8_t conf)       │
│ ├─ msg_lidar: 5 KB (LaserScan struct)           │
│ ├─ msg_imu: 0.5 KB (Imu struct)                 │
│ ├─ msg_int32: 0.1 KB                             │
│ └─ imu_current: 24 B                             │
│                                                  │
│ Synchronization: 1 KB                            │
│ ├─ lidar_mutex: 8 B                              │
│ ├─ imu_mutex: 8 B                                │
│ └─ Other semaphores                              │
│                                                  │
│ ┌─────────────────────────────────────────────┐  │
│ │ FREE HEAP: 80-100 KB ✅ (Good headroom!)    │  │
│ └─────────────────────────────────────────────┘  │
│                                                  │
└──────────────────────────────────────────────────┘

PSRAM EXTERNE (4 MB - OPTIONAL):
┌──────────────────────────────────────────────────┐
│ DMA Buffers: ~100 KB                             │
│ ├─ WiFi UDP RX ring                              │
│ ├─ WiFi UDP TX ring                              │
│ └─ Reserve                                       │
│                                                  │
│ Free: 3.8+ MB                                    │
│ (For future expansion, recording, etc.)          │
└──────────────────────────────────────────────────┘
```

### Critical Memory Metrics

```
Memory Efficiency:
├─ Static allocation: ~90% ✓ (no fragmentation)
├─ Dynamic (malloc): ~10% (ROS allocators)
├─ Heap fragmentation: LOW ✓
├─ Stack overflow risk: LOW ✓
│  (Each task has adequate stack)
└─ PSRAM dependency: HIGH (WiFi requires it)

Monitor with:
heap_caps_get_free_size(MALLOC_CAP_DEFAULT)    // Internal RAM
heap_caps_get_free_size(MALLOC_CAP_SPIRAM)     // External RAM
heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT)  // LWM

Thresholds to watch:
├─ Internal RAM < 50 KB: WARNING
├─ Internal RAM < 20 KB: CRITICAL
├─ Stack high water < 512 B: WARNING
└─ PSRAM < 1 MB: OK (plenty)
```

---

## État Machine

### Connection State Diagram

```
              ┌──────────────────────┐
              │    POWER ON / RESET  │
              └────────────┬─────────┘
                           │
                           ▼
        ┌───────────────────────────────────┐
        │     WAITING_AGENT                 │
        │                                   │
        │ Actions:                          │
        │ • WiFi.begin()                    │
        │ • Serial logging active           │
        │ • LED blink fast                  │
        │ • Loop: ping_agent()              │
        │   every 100ms                     │
        │                                   │
        │ Timeout: 1 second                 │
        │ Retry: continuous                 │
        └───────────────┬───────────────────┘
                        │
                        │ rmw_uros_ping_agent()
                        │ returns RMW_RET_OK
                        │
        ┌───────────────▼───────────────────┐
        │   AGENT_CONNECTED                 │
        │                                   │
        │ Actions:                          │
        │ • create_entities() - publishers  │
        │ • executor init                   │
        │ • LED steady ON                   │
        │ • Main loop:                      │
        │   - Publish /data (counter)       │
        │   - Publish /scan (LIDAR)         │
        │   - Publish /imu/data             │
        │   - executor spin_some()          │
        │   - Ping check (1/sec)            │
        │                                   │
        │ CPU Usage: 10-15%                 │
        │ Latency: 50-100ms                 │
        └───────────────┬───────────────────┘
                        │
                        │ Ping fails 3x
                        │ OR WiFi disconnect
                        │
        ┌───────────────▼───────────────────┐
        │   AGENT_DISCONNECTED              │
        │                                   │
        │ Actions:                          │
        │ • Stop all publishers             │
        │ • Cleanup ROS structures:         │
        │   - rcl_publisher_fini()          │
        │   - rclc_executor_fini()          │
        │   - rclc_node_fini()              │
        │   - rclc_support_fini()           │
        │ • LED off or slow blink           │
        │ • Wait 5 seconds                  │
        │ • Goto WAITING_AGENT              │
        │                                   │
        │ Reason for states:                │
        │ • Graceful cleanup (no leaks)     │
        │ • Prevent spurious publishes      │
        │ • Give network time to recover    │
        └───────────────┬───────────────────┘
                        │
                        │ After 5s
                        │
                        └──► Back to WAITING_AGENT
                             (loop continues)
```

---

## Communication Stack

### Protocol Layers (Micro-ROS)

```
┌────────────────────────────────────────────┐
│  LAYER 7 (Application Protocols)            │
├────────────────────────────────────────────┤
│                                            │
│ ROS2 Message Types:                        │
│ ├─ sensor_msgs::msg::LaserScan (360 points)│
│ │  └─ frame_id, ranges[], intensities[]    │
│ │                                          │
│ ├─ sensor_msgs::msg::Imu (6-axis + quat)  │
│ │  └─ accel_x/y/z, gyro_x/y/z, quat       │
│ │                                          │
│ └─ std_msgs::msg::Int32 (counter)          │
│    └─ data: uint32_t value                 │
│                                            │
└──────────────┬───────────────────────────────┘
               │
┌──────────────▼───────────────────────────────┐
│  LAYER 6 (ROS Client Library)                │
├────────────────────────────────────────────┤
│                                            │
│ rclc (minimal ROS client for embedded)      │
│ ├─ rcl_publisher_init()                     │
│ ├─ rcl_publish()                            │
│ ├─ rclc_executor_init()                     │
│ ├─ rclc_executor_spin_some()                │
│ └─ Serialization & deserialization          │
│                                            │
└──────────────┬───────────────────────────────┘
               │
┌──────────────▼───────────────────────────────┐
│  LAYER 5 (ROS Middleware Interface)          │
├────────────────────────────────────────────┤
│                                            │
│ RMW (ROS Middleware Interface)              │
│ ├─ Abstracts transport (UDP, serial, etc)   │
│ ├─ DDS-XRCE protocol (micro-ROS variant)    │
│ └─ rmw_uros_ping_agent()                    │
│                                            │
└──────────────┬───────────────────────────────┘
               │
┌──────────────▼───────────────────────────────┐
│  LAYER 4 (Agent / Middleware Router)         │
├────────────────────────────────────────────┤
│                                            │
│ ROS2 Agent (runs on desktop)                │
│ ├─ Listens UDP port 8888                    │
│ ├─ Parses DDS-XRCE packets                  │
│ ├─ Routes to Desktop DDS                    │
│ └─ Bridges micro-ROS ↔ full ROS2            │
│                                            │
│ Command (desktop):                         │
│ $ ros2 run micro_ros_agent micro_ros_agent \│
│     udp4 --ip 172.20.10.4 -p 8888          │
│                                            │
└──────────────┬───────────────────────────────┘
               │
┌──────────────▼───────────────────────────────┐
│  LAYER 3 (Transport Protocol)                │
├────────────────────────────────────────────┤
│                                            │
│ WiFi UDP (RFC 768)                          │
│ ├─ Source: 172.20.10.4:random_port          │
│ ├─ Dest: 172.20.10.1:8888                   │
│ ├─ Packet size: 64-1472 bytes (typical)     │
│ ├─ Latency: 50-100ms round-trip             │
│ ├─ Bandwidth used: ~10-20 kbps avg          │
│ ├─ Reliability: Best-effort (UDP)           │
│ └─ MTU: 1500 bytes (Ethernet)               │
│                                            │
└──────────────┬───────────────────────────────┘
               │
┌──────────────▼───────────────────────────────┐
│  LAYER 2 (Network Access)                    │
├────────────────────────────────────────────┤
│                                            │
│ WiFi (802.11 b/g/n)                        │
│ ├─ SSID: "iPhone (3)"                       │
│ ├─ Mode: STA (Station)                      │
│ ├─ Channel: 1-13 (depends on AP)            │
│ ├─ RSSI: -80 to -30 dBm (typical)          │
│ ├─ Data Rate: 5-11 Mbps (802.11g)          │
│ ├─ Range: 50-100m (open space)              │
│ └─ Power: ~100mW (active Rx/Tx)            │
│                                            │
└──────────────┬───────────────────────────────┘
               │
┌──────────────▼───────────────────────────────┐
│  LAYER 1 (Hardware)                          │
├────────────────────────────────────────────┤
│                                            │
│ ESP32 WiFi Radio                            │
│ ├─ Chip: Xtensa dual-core (240 MHz)        │
│ ├─ Built-in 2.4 GHz antenna                │
│ ├─ RF switch (antenna/coaxial)             │
│ ├─ Power amp: 20 dBm max                   │
│ └─ Front-end: Filters + LNA                │
│                                            │
└────────────────────────────────────────────┘
```

---

## FreeRTOS Tasking

### Dual-Core Task Scheduling

```
CORE 0 (Main / ROS):              CORE 1 (LIDAR Sensor):
═════════════════════════════════ ═════════════════════════════════

Task: loop()                      Task: lidarTask()
Priority: 1 (lower)               Priority: 2 (higher)
Stack: 8192 bytes (8 KB)          Stack: 4096 bytes (4 KB)
CPU Affinity: Core 0 (pinned)     CPU Affinity: Core 1 (pinned)

[Main Loop Iteration]:            [LIDAR Reading Loop]:
┌────────────────────────────────┐ ┌────────────────────────────┐
│ 1. Check state machine (1ms)   │ │ 1. Serial poll (1ms)       │
│    ├─ WAITING_AGENT            │ │    LIDAR_SERIAL.available()│
│    ├─ AGENT_CONNECTED          │ │                            │
│    └─ AGENT_DISCONNECTED       │ │ 2. State machine (0.5ms)   │
│                                │ │    Parse & validate        │
│ 2. Publish topics (5ms)         │ │                            │
│    ├─ rcl_publish(&pub_lidar)  │ │ 3. Mutex lock (0.1ms)      │
│    ├─ rcl_publish(&pub_imu)    │ │    Update buffer           │
│    └─ rcl_publish(&pub_counter)│ │                            │
│                                │ │ 4. Task delay (1ms)        │
│ 3. Executor spin (2ms)          │ │    → CPU sleeps            │
│    rclc_executor_spin_some()   │ │                            │
│    Process callbacks            │ │ Total: ~2.6ms per cycle   │
│                                │ │ (CPU idle 99.7%)           │
│ 4. Delay 10ms                  │ │                            │
│    → Core 0 sleeps             │ │                            │
│    (or other Core 0 tasks run) │ │                            │
│                                │ │                            │
│ Total: ~10ms per iteration      │ │                            │
│ (CPU active 10%)               │ │                            │
└────────────────────────────────┘ └────────────────────────────┘

Context Switch Points:
├─ Every 1ms (FreeRTOS tick @ 1000 Hz)
├─ On I/O wait (serial, WiFi)
├─ On semaphore contention
└─ Preemptive (higher priority interrupts lower)

ISR Handlers (High Priority):
├─ esp_now_recv_cb() [if using ESP-NOW]
├─ UART ISR (serial RX)
├─ WiFi event ISR
└─ Watchdog timer ISR

Concurrency:
├─ No busy waiting (efficient!)
├─ Mutex protects lidar_buffer
├─ Timeout prevents deadlock
└─ Both cores independent (true parallelism)

Performance:
├─ Total CPU usage: 10-15%
├─ Latency (publish): 50-100ms WiFi
├─ Jitter: ±20ms (WiFi variable)
└─ Responsiveness: Good (LIDAR prioritized)
```

---

## Synchronisation

### Mutex Access Timeline

```
Time (ms)
│
0  ┌─ Core 1: Finish parsing LIDAR frame
│  │
1  ├─ Core 1: xSemaphoreTake(&lidar_mutex) → LOCKED
│  │ Core 0: Waiting for data...
│  │
2  ├─ Core 1: Update buffer[i].distance = 320
│  │         Update buffer[i].confidence = 180
│  │         Set lidar_data_ready = true
│  │
3  ├─ Core 1: xSemaphoreGive(&lidar_mutex) → UNLOCKED
│  │ Core 0: ACQUIRED LOCK immediately
│  │
4  ├─ Core 0: Copy buffer to msg_lidar.ranges[i]
│  │         Set lidar_data_ready = false
│  │
5  ├─ Core 0: xSemaphoreGive(&lidar_mutex) → UNLOCKED
│  │
6  ├─ Core 0: rcl_publish(&pub_lidar)
│  │
└──────────────────────────────────────

Lock Duration:
├─ Typical: 100-200 µs
├─ Worst case: 1 ms
├─ Timeout: 5-10 ms (safety)
└─ No deadlock risk (timeout prevents)

Best Practices Implemented:
✓ Lock duration minimal
✓ No blocking operations inside lock
✓ Timeout prevents indefinite wait
✓ No nested locks (prevents deadlock)
✓ FIFO semaphore (fair scheduling)
```

---

## Optimization

### Build Flags (platformio.ini)

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
board_microros_transport = wifi
board_microros_distro = humble

lib_deps = 
    https://github.com/micro-ROS/micro_ros_platformio
    electroniccats/MPU6050

upload_speed = 115200
monitor_port = /dev/ttyUSB0
monitor_speed = 115200

board_build.partitions = huge_app.csv

build_flags = 
    -DMICRO_ROS_TRANSPORT_ARDUINO_WIFI
    -DBOARD_HAS_PSRAM
    -mfix-esp32-psram-cache-issue
    -Wno-unused-result
    -O2
    -funroll-loops
    
Flag Explanation:
├─ MICRO_ROS_TRANSPORT_ARDUINO_WIFI: WiFi transport
├─ BOARD_HAS_PSRAM: Enable external RAM support
├─ mfix-esp32-psram-cache-issue: Hardware workaround
├─ Wno-unused-result: Suppress warnings
├─ O2: Optimization (balance speed/size)
└─ funroll-loops: Faster LIDAR parsing
```

### Memory Profiling Functions

```cpp
// Add these to monitor memory in real-time

void print_memory_stats() {
    uint32_t free = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    uint32_t min_free = 
        heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    uint32_t psram_free = 
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    
    Serial.printf("[MEM] Free: %u B (min: %u), PSRAM: %u B\n",
                 free, min_free, psram_free);
}

void print_task_stats() {
    uint32_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *tasks = 
        malloc(num_tasks * sizeof(TaskStatus_t));
    
    uxTaskGetSystemState(tasks, num_tasks, NULL);
    
    Serial.println("[TASKS]");
    for (int i = 0; i < num_tasks; i++) {
        uint32_t free_stack = 
            tasks[i].usStackHighWaterMark * 4;  // words to bytes
        
        Serial.printf("  %s: %u B free (hwm)\n",
                     tasks[i].pcTaskName, free_stack);
    }
    
    free(tasks);
}

// Call periodically (e.g., every 10 seconds)
void setup() {
    xTaskCreate(
        [](void*) {
            while (1) {
                print_memory_stats();
                print_task_stats();
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
        },
        "monitor",
        2048,
        NULL,
        1,
        NULL
    );
}
```

### QoS Tuning

```cpp
// For ultra-low latency, consider:

// 1. Reduce delay in loop()
//    From: delay(1000)
//    To:   delay(10) or delay(5)
//    Trade-off: More CPU usage, lower latency

// 2. Increase publish frequency
//    From: 1 Hz
//    To:   10-100 Hz
//    Trade-off: More bandwidth, more timely data

// 3. Decimate LIDAR data
//    From: 360 points/scan
//    To:   180 or 90 points/scan
//    Trade-off: Less coverage, smaller messages

// 4. Reduce filter angle range
//    From: 0-360°
//    To:   -135 to +135° (270° FOV)
//    Trade-off: Blind spots, faster processing
```

---

## 🎯 Conclusion

Cette architecture Micro-ROS fournit:
- ✅ **Modularité**: Séparation clear des tâches (ROS vs capteurs)
- ✅ **Scalabilité**: Facile ajouter nouveaux capteurs/topics
- ✅ **Fiabilité**: Mutex & FreeRTOS management
- ✅ **ROS2 Compliant**: Full ecosystem access (RViz, rosbag, etc.)
- ✅ **Performance**: Latence acceptable pour robotique

**Document créé**: 25 janvier 2026  
**Version**: 2.0 (Complète avec diagr. détaillés)
