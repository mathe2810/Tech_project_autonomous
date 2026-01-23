#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <MPU6050.h>

// Configuration
#define WIFI_SSID "iPhone (3)"
#define WIFI_PASSWORD "Dr69qf76&*"
#define AGENT_IP IPAddress(172, 20, 10, 4)
#define AGENT_PORT 8888

// LIDAR Configuration
#define LIDAR_ENABLED 1
#define LIDAR_RX 16
#define LIDAR_PWM 25
#define LIDAR_BAUD 460800  // Maximum pour ESP32 (2x plus rapide = plus de frames/sec)

// ROS Objects
rcl_publisher_t pub_int;
rcl_publisher_t pub_lidar;
rcl_publisher_t pub_imu;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
std_msgs__msg__Int32 msg_int;
sensor_msgs__msg__LaserScan msg_lidar;
sensor_msgs__msg__Imu msg_imu;

// State Machine
enum states {
    WAITING_AGENT,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

// Thread-safe structures
static SemaphoreHandle_t lidar_mutex = NULL;
static SemaphoreHandle_t lidar_ready_semaphore = NULL;
static SemaphoreHandle_t imu_mutex = NULL;
static SemaphoreHandle_t imu_ready_semaphore = NULL;
static TaskHandle_t publish_task_handle = NULL;
static MPU6050 mpu;

// Structure pour stocker un frame LIDAR complet (360 points)
typedef struct {
    float angles[360];           // Angle en degrés (0-360)
    float distances[360];        // Distance en mètres
    uint8_t confidences[360];    // Confiance 0-255
    int valid_points;            // Nombre de points valides
    uint32_t timestamp_ms;       // Timestamp du frame
} LidarFrame;

// Double buffer pour éviter les race conditions
static LidarFrame lidar_frames[2];
static volatile int active_frame_idx = 0;  // Index du frame en cours de remplissage
static volatile bool frame_ready = false;   // Un nouveau frame est prêt à publier

// Structure pour IMU data
typedef struct {
    float accel_x, accel_y, accel_z;    // m/s²
    float gyro_x, gyro_y, gyro_z;       // rad/s
    float temperature;                   // °C
    uint32_t timestamp_ms;
} ImuData;

static ImuData imu_data[2];
static volatile int active_imu_idx = 0;
static volatile bool imu_ready = false;

int counter = 0;

// Statistiques LIDAR
struct LidarStats {
  uint32_t frames_total = 0;
  uint32_t frames_valid = 0;
  uint32_t frames_crc_error = 0;
  uint32_t points_total = 0;
  uint32_t points_valid = 0;  // Points avec distance valide (0.06-12m, conf > 50)
  float avg_distance = 0.0f;
  uint32_t last_report = 0;
} lidar_stats;

// Create ROS Entities
bool create_entities() {
    allocator = rcl_get_default_allocator();
    
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "esp32_node", "", &support) != RCL_RET_OK) return false;
    
    // Publisher pour Int32 (compteur)
    if (rclc_publisher_init_default(&pub_int, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/data") != RCL_RET_OK) return false;
    
    #if LIDAR_ENABLED
    // Publisher pour LaserScan (LIDAR)
    if (rclc_publisher_init_default(&pub_lidar, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan") != RCL_RET_OK) return false;
    #endif
    
    // Publisher pour IMU
    if (rclc_publisher_init_default(&pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data") != RCL_RET_OK) return false;
    
    executor = rclc_executor_get_zero_initialized_executor();
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    
    Serial.println("[OK] ROS Entities created!");
    return true;
}

// Destroy ROS Entities
void destroy_entities() {
    rcl_publisher_fini(&pub_int, &node);
    #if LIDAR_ENABLED
    rcl_publisher_fini(&pub_lidar, &node);
    #endif
    rcl_publisher_fini(&pub_imu, &node);
    rcl_node_fini(&node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    Serial.println("[CLEANUP] Entities destroyed");
}

// ==================== TÂCHE LIDAR (Cœur 1) ====================
#if LIDAR_ENABLED
static HardwareSerial LIDAR_SERIAL(2);
static const uint8_t LIDAR_HEADER = 0x54;
static const int LIDAR_POINTS_PER_FRAME = 12;

// CRC8 lookup table pour LD06
static const uint8_t CRC8[256] = {
  0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C,
  0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5,
  0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43,
  0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA,
  0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62,
  0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB,
  0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D,
  0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4,
  0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20,
  0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89,
  0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F,
  0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96,
  0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E,
  0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7,
  0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01,
  0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8
};

static inline uint8_t crc8_calc(const uint8_t* p, uint8_t n) {
  uint8_t c = 0;
  while(n--) c = CRC8[(c ^ *p++) & 0xFF];
  return c;
}

void lidarTask(void *parameter) {
  enum ParseState { FIND_HEADER, READ_FIXED, READ_POINTS, READ_TAIL };
  ParseState state = FIND_HEADER;
  
  uint8_t version_length = 0;
  int points_expected = 12;
  const int MAX_FRAME_SIZE = 1 + 1 + 2 + 2 + 12 * 3 + 2 + 2 + 1;
  uint8_t frame_buffer[MAX_FRAME_SIZE];
  int frame_index = 0;
  
  // Buffer de points bruts pour assembler un rotation complète (360°)
  float angle_accumulator[360];
  float distance_accumulator[360];
  uint8_t confidence_accumulator[360];
  int angle_to_point[360];  // Mapping angle → index dans le frame
  
  // Initialiser les accumulateurs
  memset(angle_to_point, -1, sizeof(angle_to_point));
  for(int i = 0; i < 360; i++) {
    angle_accumulator[i] = 0.0f;
    distance_accumulator[i] = 0.0f;
    confidence_accumulator[i] = 0;
  }
  
  uint32_t frames_parsed = 0;
  uint32_t timestamp_frame = millis();
  
  Serial.println("[LIDAR] Task started on core 1 - 360° assembly mode");
  
  while(1) {
    while(LIDAR_SERIAL.available() > 0) {
      uint8_t byte_read = LIDAR_SERIAL.read();
      
      switch(state) {
        case FIND_HEADER:
          if(byte_read == LIDAR_HEADER) {
            frame_buffer[0] = byte_read;
            frame_index = 1;
            state = READ_FIXED;
          }
          break;
          
        case READ_FIXED:
          frame_buffer[frame_index++] = byte_read;
          if(frame_index == 2) {
            version_length = frame_buffer[1];
            points_expected = version_length & 0x1F;
            if(points_expected <= 0 || points_expected > 12) 
              points_expected = 12;
          }
          if(frame_index == 6) state = READ_POINTS;
          break;
          
        case READ_POINTS:
          frame_buffer[frame_index++] = byte_read;
          if(frame_index == (1 + 1 + 2 + 2 + points_expected * 3))
            state = READ_TAIL;
          break;
          
        case READ_TAIL:
          frame_buffer[frame_index++] = byte_read;
          if(frame_index == (1 + 1 + 2 + 2 + points_expected * 3 + 2 + 2 + 1)) {
            uint8_t crc_received = frame_buffer[frame_index - 1];
            
            if(crc8_calc(frame_buffer, frame_index - 1) == crc_received) {
              frames_parsed++;
              lidar_stats.frames_total++;
              lidar_stats.frames_valid++;
              
              // Parse frame
              int pi = 0;
              pi++; // header
              frame_buffer[pi++]; // ver_len
              uint16_t speed = frame_buffer[pi++] | (frame_buffer[pi++] << 8);
              uint16_t startA = frame_buffer[pi++] | (frame_buffer[pi++] << 8);
              
              float start_angle_deg = startA * 0.01f;
              
              // Assembler les points dans l'accumulateur 360°
              for(int i = 0; i < points_expected; i++) {
                uint16_t dist = frame_buffer[pi++] | (frame_buffer[pi++] << 8);
                uint8_t conf = frame_buffer[pi++];
                float dist_m = dist / 1000.0f;
                float angle_deg = fmod(start_angle_deg + (float)i * (10.0f / points_expected), 360.0f);
                
                int angle_idx = (int)round(angle_deg) % 360;
                
                angle_accumulator[angle_idx] = angle_deg;
                distance_accumulator[angle_idx] = dist_m;
                confidence_accumulator[angle_idx] = conf;
                
                if(dist_m >= 0.06f && dist_m <= 12.0f && conf >= 50) {
                  lidar_stats.points_valid++;
                }
                lidar_stats.points_total++;
              }
              
              // ⭐ TOUS LES 3-5 FRAMES: assembler et publier le frame 360°
              if(frames_parsed % 3 == 0) {  // ~33 Hz si capteur à 100 Hz
                // Double buffer: écrire dans l'autre buffer
                int write_idx = 1 - active_frame_idx;
                
                if(xSemaphoreTake(lidar_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                  // Copier l'accumulateur
                  for(int i = 0; i < 360; i++) {
                    lidar_frames[write_idx].angles[i] = angle_accumulator[i];
                    lidar_frames[write_idx].distances[i] = distance_accumulator[i];
                    lidar_frames[write_idx].confidences[i] = confidence_accumulator[i];
                  }
                  lidar_frames[write_idx].timestamp_ms = millis();
                  lidar_frames[write_idx].valid_points = lidar_stats.points_valid;
                  
                  // Swap le buffer actif
                  active_frame_idx = write_idx;
                  frame_ready = true;
                  
                  xSemaphoreGive(lidar_mutex);
                }
                
                // Notifier la tâche de publication
                if(lidar_ready_semaphore) {
                  xSemaphoreGive(lidar_ready_semaphore);
                }
              }
              
              // Log tous les 100 frames (~3-4 secondes)
              if(frames_parsed % 100 == 0) {
                Serial.print("[LIDAR] ");
                Serial.print(frames_parsed);
                Serial.print(" frames | ");
                Serial.print(lidar_stats.points_valid);
                Serial.print(" valid pts | Freq: ");
                Serial.print((frames_parsed / 3) / 4.0f);  // Estimation
                Serial.println(" Hz (pub)");
              }
            } else {
              lidar_stats.frames_crc_error++;
            }
            
            state = FIND_HEADER;
            frame_index = 0;
          }
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(0));  // Laisser respirer
  }
}

// ==================== TÂCHE PUBLICATION (Cœur 0) ====================
void lidarPublishTask(void *parameter) {
  Serial.println("[PUBLISH] Task started on core 0");
  uint32_t publish_count = 0;
  uint32_t last_log = millis();
  
  while(1) {
    // Attendre qu'un frame soit prêt (avec timeout)
    if(xSemaphoreTake(lidar_ready_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      
      if(frame_ready && xSemaphoreTake(lidar_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        frame_ready = false;
        
        // Utiliser le frame prêt
        int read_idx = active_frame_idx;
        LidarFrame* frame = &lidar_frames[read_idx];
        
        // Créer et publier le message ROS2
        msg_lidar.header.stamp.sec = (uint32_t)(frame->timestamp_ms / 1000);
        msg_lidar.header.stamp.nanosec = (frame->timestamp_ms % 1000) * 1000000;
        msg_lidar.header.frame_id.data = (char*)"lidar_link";
        msg_lidar.header.frame_id.size = strlen("lidar_link");
        
        msg_lidar.angle_min = 0.0f;
        msg_lidar.angle_max = 2.0f * 3.14159265f;
        msg_lidar.angle_increment = (2.0f * 3.14159265f) / 360.0f;
        msg_lidar.time_increment = 0.0f;
        msg_lidar.scan_time = 1.0f / 30.0f;  // 30 Hz = ~33ms par scan
        msg_lidar.range_min = 0.06f;
        msg_lidar.range_max = 12.0f;
        
        // Allouer ranges pour 360 points
        msg_lidar.ranges.data = (float*)malloc(sizeof(float) * 360);
        msg_lidar.ranges.size = 360;
        
        // Remplir avec les données du frame
        for(int i = 0; i < 360; i++) {
          msg_lidar.ranges.data[i] = frame->distances[i];
        }
        
        msg_lidar.intensities.data = NULL;
        msg_lidar.intensities.size = 0;
        
        // Publier
        rcl_ret_t ret = rcl_publish(&pub_lidar, &msg_lidar, NULL);
        if(ret == RCL_RET_OK) {
          publish_count++;
        } else {
          Serial.print("[PUBLISH ERROR] ");
          Serial.println(ret);
        }
        
        free(msg_lidar.ranges.data);
        xSemaphoreGive(lidar_mutex);
        
        // Log tous les 30 publications
        uint32_t now = millis();
        if(publish_count % 30 == 0 && now - last_log > 1000) {
          Serial.print("[LIDAR PUB] ");
          Serial.print(publish_count);
          Serial.print(" pub | Freq: ");
          Serial.print((publish_count - (publish_count/30)*30) / ((now - last_log)/1000.0f));
          Serial.println(" Hz");
          last_log = now;
        }
      }
    }
  }
}

// ==================== TÂCHE IMU (Cœur 1 - Alternance avec LIDAR) ====================
void imuTask(void *parameter) {
  Serial.println("[IMU] Task started - MPU6050 reading @ 20 Hz");
  
  uint32_t imu_count = 0;
  uint32_t last_log = millis();
  
  while(1) {
    // Lire MPU6050 @ 20 Hz (50ms)
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp_raw;
    
    // Lecture directe des données brutes
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    temp_raw = mpu.getTemperature();
    
    // Convertir en SI units
    // Accélération: ±2g, 16384 LSB/g
    float accel_x = (ax / 16384.0f) * 9.81f;
    float accel_y = (ay / 16384.0f) * 9.81f;
    float accel_z = (az / 16384.0f) * 9.81f;
    
    // Gyroscope: ±250°/s, 131 LSB/°/s
    float gyro_x = (gx / 131.0f) * (3.14159265f / 180.0f);
    float gyro_y = (gy / 131.0f) * (3.14159265f / 180.0f);
    float gyro_z = (gz / 131.0f) * (3.14159265f / 180.0f);
    
    // Température: 35°C @ 0 LSB, +1°C per 340 LSB
    float temperature = 35.0f + ((float)temp_raw / 340.0f);
    
    // Double buffer - écrire dans l'autre buffer
    int write_idx = 1 - active_imu_idx;
    
    if(xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      imu_data[write_idx].accel_x = accel_x;
      imu_data[write_idx].accel_y = accel_y;
      imu_data[write_idx].accel_z = accel_z;
      imu_data[write_idx].gyro_x = gyro_x;
      imu_data[write_idx].gyro_y = gyro_y;
      imu_data[write_idx].gyro_z = gyro_z;
      imu_data[write_idx].temperature = temperature;
      imu_data[write_idx].timestamp_ms = millis();
      
      active_imu_idx = write_idx;
      imu_ready = true;
      
      xSemaphoreGive(imu_mutex);
    }
    
    // Notifier la tâche de publication
    if(imu_ready_semaphore) {
      xSemaphoreGive(imu_ready_semaphore);
    }
    
    imu_count++;
    
    // Log tous les 100 lectures
    uint32_t now = millis();
    if(imu_count % 100 == 0 && now - last_log > 5000) {
      Serial.print("[IMU] ");
      Serial.print(imu_count / 5);  // 100 lectures = ~5 secondes @ 20Hz
      Serial.print(" samples | T: ");
      Serial.print(imu_data[active_imu_idx].temperature, 1);
      Serial.println("°C");
      last_log = now;
    }
    
    vTaskDelay(pdMS_TO_TICKS(50));  // 20 Hz (50ms)
  }
}

// ==================== TÂCHE PUBLICATION IMU ====================
void imuPublishTask(void *parameter) {
  Serial.println("[IMU PUBLISH] Task started");
  uint32_t pub_count = 0;
  uint32_t last_log = millis();
  
  while(1) {
    if(xSemaphoreTake(imu_ready_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      
      if(imu_ready && xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        imu_ready = false;
        
        int read_idx = active_imu_idx;
        ImuData* data = &imu_data[read_idx];
        
        // Remplir le message IMU
        msg_imu.header.stamp.sec = (uint32_t)(data->timestamp_ms / 1000);
        msg_imu.header.stamp.nanosec = (data->timestamp_ms % 1000) * 1000000;
        msg_imu.header.frame_id.data = (char*)"imu_link";
        msg_imu.header.frame_id.size = strlen("imu_link");
        
        // Orientation (pas disponible sans calibration)
        msg_imu.orientation.x = 0.0;
        msg_imu.orientation.y = 0.0;
        msg_imu.orientation.z = 0.0;
        msg_imu.orientation.w = 1.0;
        msg_imu.orientation_covariance[0] = -1.0;  // Not available
        
        // Accélération (m/s²)
        msg_imu.linear_acceleration.x = data->accel_x;
        msg_imu.linear_acceleration.y = data->accel_y;
        msg_imu.linear_acceleration.z = data->accel_z;
        for(int i = 0; i < 9; i++) {
          msg_imu.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        }
        
        // Vitesse angulaire (rad/s)
        msg_imu.angular_velocity.x = data->gyro_x;
        msg_imu.angular_velocity.y = data->gyro_y;
        msg_imu.angular_velocity.z = data->gyro_z;
        for(int i = 0; i < 9; i++) {
          msg_imu.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        }
        
        // Publier
        rcl_ret_t ret = rcl_publish(&pub_imu, &msg_imu, NULL);
        if(ret == RCL_RET_OK) {
          pub_count++;
        } else {
          Serial.print("[IMU PUB ERROR] ");
          Serial.println(ret);
        }
        
        xSemaphoreGive(imu_mutex);
        
        // Log tous les 20 publications
        uint32_t now = millis();
        if(pub_count % 20 == 0 && now - last_log > 1000) {
          Serial.print("[IMU PUB] ");
          Serial.print(pub_count);
          Serial.println(" pub");
          last_log = now;
        }
      }
    }
  }
}

void lidar_init() {
  LIDAR_SERIAL.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, -1);
  lidar_mutex = xSemaphoreCreateMutex();
  lidar_ready_semaphore = xSemaphoreCreateBinary();
  
  if (!lidar_mutex || !lidar_ready_semaphore) {
    Serial.println("[LIDAR] ERROR: Failed to create synchronization primitives");
    return;
  }
  
  // Tâche LIDAR: Core 1, haute priorité (lecture brute)
  xTaskCreatePinnedToCore(
    lidarTask,
    "LidarReadTask",
    4096,
    NULL,
    3,      // Très haute priorité
    NULL,
    1       // Core 1
  );
  
  // Tâche Publication: Core 0, priorité moyenne (traitement + pub ROS2)
  xTaskCreatePinnedToCore(
    lidarPublishTask,
    "LidarPublishTask",
    4096,
    NULL,
    2,      // Priorité moyenne
    &publish_task_handle,
    0       // Core 0 (où tourne le ROS2)
  );
  
  Serial.println("[LIDAR] ⚡ OPTIMIZED: Core 1 reads, Core 0 publishes");
  Serial.println("[LIDAR] 📊 Target: 30+ Hz publication, 360 points per scan");
}
#endif

void mpu_init() {
  // Initialize I2C
  Wire.begin(21, 22, 400000);  // SDA=21, SCL=22, 400kHz
  delay(100);
  
  // Initialize MPU6050
  Serial.print("[MPU6050] Initializing...");
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println(" FAILED!");
    return;
  }
  Serial.println(" OK!");
  
  // Configure MPU6050
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250°/s
  mpu.setSleepEnabled(false);
  
  // Create synchronization primitives
  imu_mutex = xSemaphoreCreateMutex();
  imu_ready_semaphore = xSemaphoreCreateBinary();
  
  if (!imu_mutex || !imu_ready_semaphore) {
    Serial.println("[MPU6050] ERROR: Failed to create sync primitives");
    return;
  }
  
  // Create IMU tasks
  xTaskCreatePinnedToCore(
    imuTask,
    "ImuReadTask",
    3072,
    NULL,
    2,
    NULL,
    1     // Core 1 (alternates with LIDAR)
  );
  
  xTaskCreatePinnedToCore(
    imuPublishTask,
    "ImuPublishTask",
    3072,
    NULL,
    1,
    NULL,
    0     // Core 0
  );
  
  Serial.println("[MPU6050] ✅ Initialized - I2C (SDA=21, SCL=22) @ 20Hz");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== SETUP START ===");
    
    #if LIDAR_ENABLED
    lidar_init();
    delay(500);
    #endif
    
    mpu_init();
    delay(500);
    
    char ssid[] = "iPhone (3)";
    char password[] = "Dr69qf76&*";
    
    Serial.print("[WiFi] Connecting to ");
    Serial.print(ssid);
    Serial.print("...");
    set_microros_wifi_transports(ssid, password, AGENT_IP, AGENT_PORT);
    
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        Serial.print(".");
        timeout++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" OK!");
        Serial.print("[WiFi] IP locale: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println(" FAILED!");
    }
    
    state = WAITING_AGENT;
    Serial.println("[STATE] Waiting for agent...");
}

void loop() {
    switch (state) {
        case WAITING_AGENT:
            // Try to ping agent
            if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
                Serial.println("[AGENT] Agent found! Connecting...");
                if (create_entities()) {
                    state = AGENT_CONNECTED;
                    Serial.println("[STATE] CONNECTED!");
                } else {
                    Serial.println("[ERROR] Failed to create entities");
                    destroy_entities();
                    state = WAITING_AGENT;
                }
            } else {
                Serial.println("[AGENT] Agent not found, retrying...");
            }
            delay(2000);
            break;

        case AGENT_CONNECTED: {
            // Check WiFi health
            if (WiFi.status() != WL_CONNECTED) {
                Serial.println("[ERROR] WiFi disconnected!");
                state = AGENT_DISCONNECTED;
                break;
            }
            
            // ⭐ Les tâches LIDAR publient automatiquement
            // On n'a qu'à pump l'executor ROS2
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
            
            // Publier counter tous les 30 publications LIDAR (~1 Hz @ 30Hz LIDAR)
            static int pub_count = 0;
            if(pub_count++ % 30 == 0) {
                msg_int.data = counter++;
                rcl_ret_t ret = rcl_publish(&pub_int, &msg_int, NULL);
                if (ret != RCL_RET_OK) {
                    Serial.print("[COUNTER ERROR] ");
                    Serial.println(ret);
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(1));
            break;
        }

        case AGENT_DISCONNECTED:
            destroy_entities();
            state = WAITING_AGENT;
            Serial.println("[STATE] Back to WAITING_AGENT");
            break;
    }
}
