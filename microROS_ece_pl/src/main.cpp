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

// ==================== CONFIG ====================
#define WIFI_SSID "iPhone (3)"
#define WIFI_PASSWORD "Dr69qf76&*"
#define AGENT_IP IPAddress(172, 20, 10, 4)
#define AGENT_PORT 8888

// LIDAR
#define LIDAR_ENABLED 1
#define LIDAR_RX 16
#define LIDAR_BAUD 230400
#define LIDAR_HEADER 0x54
#define PTS_PER_FRAME 12

// ==================== ROS OBJECTS ====================
rcl_publisher_t pub_int, pub_lidar, pub_imu;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
std_msgs__msg__Int32 msg_int;
sensor_msgs__msg__LaserScan msg_lidar;
sensor_msgs__msg__Imu msg_imu;

enum states { WAITING_AGENT, AGENT_CONNECTED, AGENT_DISCONNECTED } state;
static volatile bool agent_connected = false;

// ==================== SYNCHRONIZATION ====================
static SemaphoreHandle_t lidar_mutex = NULL;
static SemaphoreHandle_t lidar_ready_semaphore = NULL;
static SemaphoreHandle_t imu_mutex = NULL;
static SemaphoreHandle_t imu_ready_semaphore = NULL;
static MPU6050 mpu;

// ==================== LIDAR: CRC8 TABLE ====================
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

static inline uint8_t crc8(const uint8_t* p, uint8_t n) {
  uint8_t c = 0;
  while(n--) c = CRC8[(c ^ *p++) & 0xFF];
  return c;
}

// ==================== LIDAR BUFFERS ====================
#define LIDAR_MAX_POINTS 360
typedef struct { float angle; float distance; uint8_t confidence; } LidarPoint;

static volatile LidarPoint lidar_buffer[LIDAR_MAX_POINTS];
static volatile uint32_t lidar_last_update = 0;
static HardwareSerial LIDAR_SERIAL(2);

struct LidarStats { 
  uint32_t frames_total = 0, frames_valid = 0, frames_crc_error = 0;
  uint32_t points_total = 0, points_valid = 0;
} lidar_stats;

// ==================== IMU DATA ====================
typedef struct {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float temperature;
  uint32_t timestamp_ms;
} ImuData;

static ImuData imu_data[2];
static volatile int active_imu_idx = 0;
static volatile bool imu_ready = false;
static uint32_t last_imu_read = 0;

// ==================== ROS SETUP ====================
bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32_rover", "", &support) != RCL_RET_OK) return false;
  
  if (rclc_publisher_init_default(&pub_int, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/data") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&pub_lidar, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data") != RCL_RET_OK) return false;
  
  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  
  Serial.println("[ROS] Entities created ✅");
  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&pub_int, &node);
  rcl_publisher_fini(&pub_lidar, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

// ==================== LIDAR TASK (Core 1) ====================
void lidarTask(void *param) {
  enum ParseState { FIND_HEADER, READ_FIXED, READ_POINTS, READ_TAIL };
  ParseState state = FIND_HEADER;
  
  uint8_t version_length = 0;
  int points_expected = PTS_PER_FRAME;
  const int MAX_FRAME_SIZE = 1+1+2+2 + PTS_PER_FRAME*3 + 2+2+1;
  uint8_t frame_buffer[MAX_FRAME_SIZE];
  int frame_index = 0;
  
  Serial.println("[LIDAR] 🚀 Task started on Core 1 @ 230400 baud");
  
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
            if(points_expected <= 0 || points_expected > PTS_PER_FRAME) 
              points_expected = PTS_PER_FRAME;
          }
          if(frame_index == 6) state = READ_POINTS;
          break;
          
        case READ_POINTS:
          frame_buffer[frame_index++] = byte_read;
          if(frame_index == (1+1+2+2 + points_expected*3))
            state = READ_TAIL;
          break;
          
        case READ_TAIL:
          frame_buffer[frame_index++] = byte_read;
          if(frame_index == (1+1+2+2 + points_expected*3 + 2+2+1)) {
            uint8_t crc_received = frame_buffer[frame_index-1];
            
            if(crc8(frame_buffer, frame_index-1) == crc_received) {
              int pi = 0;
              pi++; // header
              uint8_t ver_len = frame_buffer[pi++];
              uint16_t speed = frame_buffer[pi++] | (frame_buffer[pi++] << 8);
              uint16_t startA = frame_buffer[pi++] | (frame_buffer[pi++] << 8);
              
              // Skip points temp
              uint16_t *point_data = (uint16_t*)(frame_buffer + pi);
              pi += points_expected * 3;
              
              uint16_t endA = frame_buffer[pi++] | (frame_buffer[pi++] << 8);
              uint16_t ts = frame_buffer[pi++] | (frame_buffer[pi++] << 8);
              
              // Angle calculation
              float start_angle_deg = startA * 0.01f;
              float end_angle_deg = endA * 0.01f;
              float delta_angle = end_angle_deg - start_angle_deg;
              if(delta_angle < -180) delta_angle += 360;
              else if(delta_angle > 180) delta_angle -= 360;
              float step_angle = (points_expected > 1) ? (delta_angle / (points_expected - 1)) : 0;
              
              // Parse points and fill circular buffer
              if(xSemaphoreTake(lidar_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                lidar_stats.frames_valid++;
                
                pi = 6; // Back to points start
                for(int i = 0; i < points_expected; i++) {
                  uint16_t dist = frame_buffer[pi++] | (frame_buffer[pi++] << 8);
                  uint8_t conf = frame_buffer[pi++];
                  float angle = start_angle_deg + step_angle * i;
                  
                  // Normalize angle to 0-360
                  while(angle < 0) angle += 360;
                  while(angle >= 360) angle -= 360;
                  
                  int angle_idx = (int)round(angle) % 360;
                  lidar_buffer[angle_idx].angle = angle;
                  lidar_buffer[angle_idx].distance = dist / 1000.0f;
                  lidar_buffer[angle_idx].confidence = conf;
                  
                  if(dist >= 60 && dist <= 12000 && conf >= 40) {
                    lidar_stats.points_valid++;
                  }
                  lidar_stats.points_total++;
                }
                
                lidar_last_update = millis();
                xSemaphoreGive(lidar_mutex);
                
                // Signal publish task
                if(lidar_stats.frames_valid % 3 == 0) {
                  xSemaphoreGive(lidar_ready_semaphore);
                }
              }
              
              lidar_stats.frames_total++;
              if(lidar_stats.frames_total % 30 == 0) {
                Serial.printf("[LIDAR] Frame %u | Points: %u | CRC OK\n", 
                  lidar_stats.frames_total, lidar_stats.points_valid);
              }
            } else {
              lidar_stats.frames_crc_error++;
              if(lidar_stats.frames_total < 5) {
                Serial.println("[LIDAR] ⚠️ CRC error");
              }
            }
            
            state = FIND_HEADER;
            frame_index = 0;
          }
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ==================== IMU TASK (Core 1) ====================
void imuTask(void *param) {
  Serial.println("[IMU] Task started on Core 1");
  Wire.begin(33, 32, 400000);  // SDA=33, SCL=32 (GPIO pins)
  delay(100);
  
  if(!mpu.testConnection()) {
    Serial.println("[IMU] ❌ Not found");
    vTaskDelete(NULL);
  }
  
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
  Serial.println("[IMU] ✅ Ready @ 20Hz");
  
  while(1) {
    if(millis() - last_imu_read >= 50) {  // 20 Hz
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      int16_t temp = mpu.getTemperature();
      
      int write_idx = 1 - active_imu_idx;
      if(xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        imu_data[write_idx].accel_x = ax / 16384.0f * 9.81f;
        imu_data[write_idx].accel_y = ay / 16384.0f * 9.81f;
        imu_data[write_idx].accel_z = az / 16384.0f * 9.81f;
        imu_data[write_idx].gyro_x = gx / 131.0f * 0.01745f;
        imu_data[write_idx].gyro_y = gy / 131.0f * 0.01745f;
        imu_data[write_idx].gyro_z = gz / 131.0f * 0.01745f;
        imu_data[write_idx].temperature = temp / 340.0f + 36.53f;
        imu_data[write_idx].timestamp_ms = millis();
        
        active_imu_idx = write_idx;
        imu_ready = true;
        xSemaphoreGive(imu_mutex);
        xSemaphoreGive(imu_ready_semaphore);
      }
      
      last_imu_read = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ==================== PUBLISH LIDAR TASK (Core 0) ====================
void lidarPublishTask(void *param) {
  Serial.println("[PUBLISH-LIDAR] Task started on Core 0");
  uint32_t pub_count = 0;
  
  while(1) {
    if(!agent_connected) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    if(xSemaphoreTake(lidar_ready_semaphore, pdMS_TO_TICKS(500)) == pdTRUE) {
      if(xSemaphoreTake(lidar_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        msg_lidar.header.stamp.sec = (uint32_t)(lidar_last_update / 1000);
        msg_lidar.header.stamp.nanosec = (lidar_last_update % 1000) * 1000000;
        msg_lidar.header.frame_id.data = (char*)"lidar_link";
        msg_lidar.header.frame_id.size = strlen("lidar_link");
        
        msg_lidar.angle_min = 0.0f;
        msg_lidar.angle_max = 6.283185f;
        msg_lidar.angle_increment = 0.017453f;
        msg_lidar.time_increment = 0.0f;
        msg_lidar.scan_time = 0.033f;
        msg_lidar.range_min = 0.06f;
        msg_lidar.range_max = 12.0f;
        
        msg_lidar.ranges.data = (float*)malloc(sizeof(float) * 360);
        msg_lidar.ranges.size = 360;
        
        for(int i = 0; i < 360; i++) {
          msg_lidar.ranges.data[i] = lidar_buffer[i].distance;
        }
        
        msg_lidar.intensities.data = NULL;
        msg_lidar.intensities.size = 0;
        
        if(rcl_publish(&pub_lidar, &msg_lidar, NULL) == RCL_RET_OK) {
          pub_count++;
        }
        
        free(msg_lidar.ranges.data);
        xSemaphoreGive(lidar_mutex);
        
        if(pub_count % 30 == 0) {
          Serial.printf("[PUBLISH-LIDAR] %u pubs sent\n", pub_count);
        }
      }
    }
  }
}

// ==================== PUBLISH IMU TASK (Core 0) ====================
void imuPublishTask(void *param) {
  Serial.println("[PUBLISH-IMU] Task started on Core 0");
  uint32_t pub_count = 0;
  
  while(1) {
    if(!agent_connected) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    if(xSemaphoreTake(imu_ready_semaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
      if(xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        int read_idx = active_imu_idx;
        
        msg_imu.header.stamp.sec = imu_data[read_idx].timestamp_ms / 1000;
        msg_imu.header.stamp.nanosec = (imu_data[read_idx].timestamp_ms % 1000) * 1000000;
        msg_imu.header.frame_id.data = (char*)"imu_link";
        msg_imu.header.frame_id.size = strlen("imu_link");
        
        msg_imu.linear_acceleration.x = imu_data[read_idx].accel_x;
        msg_imu.linear_acceleration.y = imu_data[read_idx].accel_y;
        msg_imu.linear_acceleration.z = imu_data[read_idx].accel_z;
        
        msg_imu.angular_velocity.x = imu_data[read_idx].gyro_x;
        msg_imu.angular_velocity.y = imu_data[read_idx].gyro_y;
        msg_imu.angular_velocity.z = imu_data[read_idx].gyro_z;
        
        if(rcl_publish(&pub_imu, &msg_imu, NULL) == RCL_RET_OK) {
          pub_count++;
        }
        
        xSemaphoreGive(imu_mutex);
        
        if(pub_count % 20 == 0) {
          Serial.printf("[PUBLISH-IMU] %u pubs sent\n", pub_count);
        }
      }
    }
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== SETUP START ===");
  
  #if LIDAR_ENABLED
  LIDAR_SERIAL.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, -1);
  lidar_mutex = xSemaphoreCreateMutex();
  lidar_ready_semaphore = xSemaphoreCreateBinary();
  
  xTaskCreatePinnedToCore(lidarTask, "LidarTask", 5120, NULL, 3, NULL, 1);
  Serial.println("[LIDAR] ✅ Initialized");
  #endif
  
  imu_mutex = xSemaphoreCreateMutex();
  imu_ready_semaphore = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(imuTask, "ImuTask", 4096, NULL, 2, NULL, 1);
  Serial.println("[IMU] ✅ Initialized");
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int timeout = 0;
  while(WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.print("\n[WiFi] ✅ Connected: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] ❌ Failed");
  }
  
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  state = WAITING_AGENT;
  Serial.println("[STATE] Waiting for agent...");
}

// ==================== LOOP ====================
void loop() {
  switch(state) {
    case WAITING_AGENT:
      if(rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        Serial.println("[AGENT] ✅ Found!");
        if(create_entities()) {
          state = AGENT_CONNECTED;
          agent_connected = true;
          
          xTaskCreatePinnedToCore(lidarPublishTask, "LidarPubTask", 4096, NULL, 2, NULL, 0);
          xTaskCreatePinnedToCore(imuPublishTask, "ImuPubTask", 4096, NULL, 1, NULL, 0);
          Serial.println("[STATE] CONNECTED!");
        } else {
          Serial.println("[ERROR] Failed to create entities");
          destroy_entities();
          state = WAITING_AGENT;
        }
      } else {
        Serial.println("[AGENT] ❌ Not found");
      }
      delay(2000);
      break;
      
    case AGENT_CONNECTED:
      if(WiFi.status() != WL_CONNECTED) {
        Serial.println("[ERROR] WiFi disconnected!");
        agent_connected = false;
        state = AGENT_DISCONNECTED;
        break;
      }
      
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      vTaskDelay(pdMS_TO_TICKS(1));
      break;
      
    case AGENT_DISCONNECTED:
      agent_connected = false;
      destroy_entities();
      state = WAITING_AGENT;
      Serial.println("[STATE] Back to WAITING");
      break;
  }
}
