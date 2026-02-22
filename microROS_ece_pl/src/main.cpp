#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Wire.h>
#include <MPU6050.h>
#include <cmath>
#include <atomic>
#include "../motor_control.h"

#define WIFI_SSID "iPhone (3)"
#define WIFI_PASSWORD "Dr69qf76&*"
#define AGENT_HOSTNAME "microros-agent.local"
#define AGENT_FALLBACK_IP_PRIMARY IPAddress(172, 20, 10, 3)
#define AGENT_FALLBACK_IP_SECONDARY IPAddress(172, 20, 10, 4)
#define AGENT_PORT 8888

#define LIDAR_RX 16
#define LIDAR_BAUD 230400
#define LIDAR_HEADER 0x54
#define PTS_PER_FRAME 12

rcl_publisher_t pub_lidar, pub_imu;
rcl_subscription_t sub_cmd_vel;  // Independent subscriber, NOT in executor
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
sensor_msgs__msg__LaserScan msg_lidar;
sensor_msgs__msg__Imu msg_imu;
geometry_msgs__msg__Twist msg_cmd_vel;

enum states { WAITING_AGENT, AGENT_CONNECTED, AGENT_DISCONNECTED } state;
static volatile bool agent_connected = false;
static TaskHandle_t publish_task_handle = NULL;  // Track publish task to avoid duplicates

// Lock-free circular buffers
#define LIDAR_BUFFER_SIZE 4
#define IMU_BUFFER_SIZE 16

typedef struct {
  float ranges[360];
  uint32_t timestamp;
  uint16_t scan_id;
} LidarScan;

typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  uint32_t ts;
} ImuData;

// Double-buffer (ping-pong) for LIDAR - minimal latency
static LidarScan lidar_buffers[2];
static std::atomic<uint8_t> lidar_write_buf(0);
static std::atomic<uint8_t> lidar_read_buf(1);

// FIFO circular buffer for IMU
static ImuData imu_buffer[IMU_BUFFER_SIZE];
static std::atomic<uint8_t> imu_write_idx(0);
static std::atomic<uint8_t> imu_read_idx(0);

static MPU6050 mpu;

// CRC8
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

// LIDAR buffer - track by angle [0-360]
#define LIDAR_MAX_POINTS 360
static volatile uint32_t lidar_last_update = 0;
static HardwareSerial LIDAR_SERIAL(2);

// Pre-allocated buffers (NO malloc in loop!)
static float lidar_ranges[360];

// IMU avec Kalman simplifié
static uint32_t last_imu_read = 0;

// Kalman simple
typedef struct {
  float x;
  float P;
  float R;
} KalmanFilter1D;

static KalmanFilter1D kf_ax, kf_ay, kf_az;

static bool resolve_agent_ip(IPAddress &agent_ip) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  for (int attempt = 0; attempt < 5; attempt++) {
    if (WiFi.hostByName(AGENT_HOSTNAME, agent_ip) == 1) {
      return true;
    }
    delay(200);
  }

  return false;
}

static bool probe_agent_with_transport(
  char *wifi_ssid,
  char *wifi_password,
  const IPAddress &candidate_ip,
  uint16_t port
) {
  set_microros_wifi_transports(wifi_ssid, wifi_password, candidate_ip, port);

  for (int attempt = 0; attempt < 3; attempt++) {
    if (rmw_uros_ping_agent(150, 1) == RMW_RET_OK) {
      return true;
    }
    delay(120);
  }

  return false;
}

void kalman_init(KalmanFilter1D *kf) {
  kf->x = 0.0f;
  kf->P = 1.0f;
  kf->R = 5.0f;  // Increased to trust sensor more (was 0.1f - too aggressive smoothing)
}

float kalman_update(KalmanFilter1D *kf, float z) {
  float S = kf->P + kf->R;
  float K = kf->P / S;
  kf->x = kf->x + K * (z - kf->x);
  kf->P = (1.0f - K) * kf->P;
  return kf->x;
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32_rover", "", &support) != RCL_RET_OK) return false;
  
  if (rclc_publisher_init_default(&pub_lidar, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan_raw") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&pub_imu, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data") != RCL_RET_OK) return false;
  
  // Create independent cmd_vel subscriber (NOT added to executor to avoid interference)
  if (rclc_subscription_init_default(&sub_cmd_vel, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel") != RCL_RET_OK) return false;
  
  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  
  msg_lidar.header.frame_id.data = (char*)"laser_link";
  msg_lidar.header.frame_id.size = strlen("laser_link");
  msg_lidar.ranges.data = lidar_ranges;
  msg_lidar.ranges.size = 360;
  
  msg_imu.header.frame_id.data = (char*)"imu";
  msg_imu.header.frame_id.size = strlen("imu");
  
  Serial.println("[ROS] OK");
  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&pub_lidar, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_subscription_fini(&sub_cmd_vel, &node);  // Clean up independent subscriber
  rcl_node_fini(&node);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);
}

void lidarTask(void *param) {
  enum ParseState { FIND_HEADER, READ_FIXED, READ_POINTS, READ_TAIL };
  ParseState state = FIND_HEADER;
  
  uint8_t version_length = 0;
  int points_expected = PTS_PER_FRAME;
  const int MAX_FRAME_SIZE = 1+1+2+2 + PTS_PER_FRAME*3 + 2+2+1;
  uint8_t frame_buffer[MAX_FRAME_SIZE];
  int frame_index = 0;
  
  static uint16_t scan_id = 0;
  
  Serial.println("[LIDAR] Start");
  delay(500);
  Serial.printf("[LIDAR] Serial available: %d\n", LIDAR_SERIAL.available());
  
  uint32_t last_debug = 0;
  uint32_t byte_count = 0;
  uint32_t header_found = 0;
  uint32_t crc_ok = 0;
  uint32_t crc_fail = 0;
  
  while(1) {
    int available = LIDAR_SERIAL.available();
    if(available > 0) {
      byte_count += available;
    }
    
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
              int pi = 6;  // Skip header, ver, speed, startA
              uint16_t startA = frame_buffer[4] | (frame_buffer[5] << 8);
              float start_angle_deg = startA * 0.01f;
              
              // Lock-free double-buffer write
              uint8_t write_buf = lidar_write_buf.load();
              LidarScan *scan = &lidar_buffers[write_buf];
              
              for(int i = 0; i < points_expected; i++) {
                uint16_t dist = frame_buffer[pi++] | (frame_buffer[pi++] << 8);
                uint8_t conf = frame_buffer[pi++];
                float angle = start_angle_deg + (float)i * (10.0f / points_expected);
                
                while(angle >= 360.0f) angle -= 360.0f;
                int idx = (int)round(angle) % 360;
                
                scan->ranges[idx] = dist / 1000.0f;
              }
              
              scan->timestamp = millis();
              scan->scan_id = scan_id++;
              lidar_last_update = millis();
              
              // Swap buffers: reader gets the freshest data
              lidar_read_buf.store(write_buf);
              lidar_write_buf.store(1 - write_buf);
            }
            
            state = FIND_HEADER;
            frame_index = 0;
          }
          break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));  // Allow other tasks to run (WiFi, etc)
  }
}

void imuTask(void *param) {
  Serial.println("[IMU] Start");
  Wire.begin(4, 5, 400000);
  delay(100);
  
  if(!mpu.testConnection()) {
    Serial.println("[IMU] Not found");
    vTaskDelete(NULL);
  }
  
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  Serial.println("[IMU] OK");
  
  kalman_init(&kf_ax);
  kalman_init(&kf_ay);
  kalman_init(&kf_az);
  
  uint32_t debug_count = 0;
  
  while(1) {
    if(millis() - last_imu_read >= 10) {
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      // Debug: affiche les valeurs brutes toutes les 100 lectures
      if(debug_count++ % 100 == 0) {
        Serial.printf("[IMU RAW] ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n", ax, ay, az, gx, gy, gz);
      }
      
      // Convert raw counts to m/s² and rad/s (no filtering - raw data only)
      // 1g = 16384 LSB = 9.81 m/s²
      // 1°/s = 131 LSB
      float accel_x = ax / 16384.0f * 9.81f;
      float accel_y = ay / 16384.0f * 9.81f;
      float accel_z = az / 16384.0f * 9.81f;
      
      float gyro_x = gx / 131.0f * 0.01745f;  // deg/s to rad/s
      float gyro_y = gy / 131.0f * 0.01745f;
      float gyro_z = gz / 131.0f * 0.01745f;
      
      // Remap axes based on ACTUAL physical sensor orientation:
      // Sensor X (vertical/gravity) → ROS Z (remove gravity component: -9.81)
      // Sensor Y (forward/backward) → ROS X
      // Sensor Z (left/right) → ROS Y
      float accel_x_ros = accel_y;              // Sensor Y → ROS X (forward/back)
      float accel_y_ros = accel_z;              // Sensor Z → ROS Y (left/right)
      float accel_z_ros = accel_x - 9.81f;     // Sensor X → ROS Z (gravity removed)
      
      // Same remap for gyro axes
      float gyro_x_ros = gyro_y;                // Sensor Y → ROS X
      float gyro_y_ros = gyro_z;                // Sensor Z → ROS Y
      float gyro_z_ros = gyro_x;                // Sensor X → ROS Z
      
      // Lock-free write to circular buffer
      uint8_t current_write = imu_write_idx.load();
      uint8_t next_write = (current_write + 1) % IMU_BUFFER_SIZE;
      
      // Check if buffer is full, skip write if necessary
      if(next_write != imu_read_idx.load()) {
        ImuData *imu = &imu_buffer[current_write];
        
        // Send raw converted data (no Kalman filtering on ESP32)
        imu->ax = accel_x_ros;
        imu->ay = accel_y_ros;
        imu->az = accel_z_ros;
        imu->gx = gyro_x_ros;
        imu->gy = gyro_y_ros;
        imu->gz = gyro_z_ros;
        imu->ts = millis();
        
        imu_write_idx.store(next_write);
      }
      
      last_imu_read = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(0));  // Yield to other tasks
  }
}

void publishTask(void *param) {
  Serial.println("[PUBLISH] Start");
  uint32_t pub_count = 0;
  uint32_t last_imu_pub = 0;
  uint16_t last_scan_id = 0xFFFF;
  
  while(1) {
    if(!agent_connected) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    uint32_t now = millis();
    
    // Publish LIDAR 360 - double-buffer read (always get freshest)
    uint8_t read_buf = lidar_read_buf.load();
    LidarScan *scan = &lidar_buffers[read_buf];
    
    if(scan->scan_id != last_scan_id) {
      msg_lidar.angle_min = 0.0f;
      msg_lidar.angle_max = 2*M_PI - 0.017453f;
      msg_lidar.angle_increment = 0.017453f;
      msg_lidar.range_min = 0.06f;
      msg_lidar.range_max = 12.0f;
      msg_lidar.time_increment = 0.0f;
      msg_lidar.scan_time = 0.1f;
      
      // Direct pointer - no copy, minimum latency
      msg_lidar.ranges.data = scan->ranges;
      msg_lidar.ranges.size = 360;
      
      msg_lidar.header.stamp.sec = scan->timestamp / 1000;
      msg_lidar.header.stamp.nanosec = (scan->timestamp % 1000) * 1000000;
      
      rcl_publish(&pub_lidar, &msg_lidar, NULL);
      pub_count++;
      
      last_scan_id = scan->scan_id;
    }
    
    // Publish IMU 20Hz - lock-free read from circular buffer
    if(now - last_imu_pub >= 50) {
      uint8_t imu_idx = imu_read_idx.load();
      if(imu_idx != imu_write_idx.load()) {
        ImuData *imu = &imu_buffer[imu_idx];
        
        msg_imu.header.stamp.sec = imu->ts / 1000;
        msg_imu.header.stamp.nanosec = (imu->ts % 1000) * 1000000;
        msg_imu.linear_acceleration.x = imu->ax;
        msg_imu.linear_acceleration.y = imu->ay;
        msg_imu.linear_acceleration.z = imu->az;
        msg_imu.angular_velocity.x = imu->gx;
        msg_imu.angular_velocity.y = imu->gy;
        msg_imu.angular_velocity.z = imu->gz;
        
        rcl_publish(&pub_imu, &msg_imu, NULL);
        imu_read_idx.store((imu_idx + 1) % IMU_BUFFER_SIZE);
      }
      last_imu_pub = now;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  static char wifi_ssid[] = WIFI_SSID;
  static char wifi_password[] = WIFI_PASSWORD;

  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== SETUP ===");
  
  LIDAR_SERIAL.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, -1);
  
  xTaskCreatePinnedToCore(lidarTask, "LIDAR", 5120, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(imuTask, "IMU", 4096, NULL, 3, NULL, 1);
  
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int timeout = 0;
  while(WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if(WiFi.status() == WL_CONNECTED) {
    Serial.print("\n[WiFi] OK ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WiFi] FAIL");
  }
  
  // Initialize motors
  motor_init();

  IPAddress agent_ip;
  bool endpoint_ready = false;

  if (resolve_agent_ip(agent_ip)) {
    Serial.print("[AGENT] Resolved ");
    Serial.print(AGENT_HOSTNAME);
    Serial.print(" -> ");
    Serial.println(agent_ip);
    endpoint_ready = probe_agent_with_transport(wifi_ssid, wifi_password, agent_ip, AGENT_PORT);
    if (!endpoint_ready) {
      Serial.println("[AGENT] Hostname resolved but not reachable, trying fallback IPs...");
    }
  } else {
    Serial.println("[AGENT] DNS failed, trying fallback IPs...");
  }

  if (!endpoint_ready) {
    const IPAddress fallback_candidates[] = {
      AGENT_FALLBACK_IP_PRIMARY,
      AGENT_FALLBACK_IP_SECONDARY,
    };

    for (const IPAddress &candidate : fallback_candidates) {
      Serial.print("[AGENT] Trying fallback ");
      Serial.println(candidate);
      if (probe_agent_with_transport(wifi_ssid, wifi_password, candidate, AGENT_PORT)) {
        agent_ip = candidate;
        endpoint_ready = true;
        Serial.print("[AGENT] Fallback reachable: ");
        Serial.println(agent_ip);
        break;
      }
    }
  }

  if (!endpoint_ready) {
    agent_ip = AGENT_FALLBACK_IP_PRIMARY;
    set_microros_wifi_transports(wifi_ssid, wifi_password, agent_ip, AGENT_PORT);
    Serial.print("[AGENT] No endpoint reachable now, keeping primary fallback: ");
    Serial.println(agent_ip);
  }

  state = WAITING_AGENT;
}

void loop() {
  static uint32_t last_agent_check = 0;
  uint32_t now = millis();
  
  switch(state) {
    case WAITING_AGENT:
      if(WiFi.status() == WL_CONNECTED) {
        // Only ping agent every 1s to avoid flooding
        if(now - last_agent_check >= 1000) {
          if(rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            Serial.println("[AGENT] OK");
            if(create_entities()) {
              state = AGENT_CONNECTED;
              agent_connected = true;
              
              // Delete old task if it exists (safety check)
              if(publish_task_handle != NULL) {
                vTaskDelete(publish_task_handle);
                publish_task_handle = NULL;
                delay(100);
              }
              
              // Create new publish task
              xTaskCreatePinnedToCore(publishTask, "PUBLISH", 4096, NULL, 2, &publish_task_handle, 0);
              Serial.println("[STATE] CONNECTED");
            } else {
              destroy_entities();
              state = WAITING_AGENT;
              Serial.println("[AGENT] create_entities failed, retrying...");
            }
          } else {
            Serial.print(".");
          }
          last_agent_check = now;
        }
      } else {
        Serial.println("[WiFi] Connecting...");
        last_agent_check = now;
      }
      delay(500);
      break;
      
    case AGENT_CONNECTED:
      // Only check WiFi - don't ping agent while connected, it interferes with data flow
      if(WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Lost connection");
        agent_connected = false;
        state = AGENT_DISCONNECTED;
        break;
      }
      
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      
      // Check for cmd_vel messages independently (non-blocking, every 10ms)
      static uint32_t last_cmd_check = 0;
      if(millis() - last_cmd_check >= 10) {
        rmw_message_info_t info;
        if(rcl_take(&sub_cmd_vel, &msg_cmd_vel, &info, nullptr) == RCL_RET_OK) {
          float linear_x = msg_cmd_vel.linear.x;
          float angular_z = msg_cmd_vel.angular.z;
          Serial.printf("[CMD_VEL] rx: %.2f, rz: %.2f\n", linear_x, angular_z);
          
          // Apply motor control
          motor_control(linear_x, angular_z);
        }
        last_cmd_check = millis();
      }
      
      vTaskDelay(pdMS_TO_TICKS(1));
      break;
      
    case AGENT_DISCONNECTED:
      agent_connected = false;
      
      // Delete publish task before destroying entities
      if(publish_task_handle != NULL) {
        vTaskDelete(publish_task_handle);
        publish_task_handle = NULL;
        delay(100);
      }
      
      destroy_entities();
      state = WAITING_AGENT;
      Serial.println("[STATE] Reconnecting...");
      last_agent_check = 0;  // Reset timer to check immediately
      delay(1000);
      break;
  }
}
