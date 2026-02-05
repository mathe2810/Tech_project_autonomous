#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/rmw_microros.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Wire.h>
#include <MPU6050.h>
#include <cmath>
#include <atomic>

// ============ WiFi & Network =============
#define WIFI_SSID "iPhone (3)"
#define WIFI_PASSWORD "Dr69qf76&*"
#define AGENT_IP IPAddress(172, 20, 10, 3)
#define AGENT_PORT 8888

// ============ LIDAR Configuration =============
#define LIDAR_RX 16
#define LIDAR_BAUD 230400
#define LIDAR_HEADER 0x54
#define PTS_PER_FRAME 12

// ============ Motor Control Pins =============
#define MOTOR_LEFT_PWM    25
#define MOTOR_LEFT_DIR1   21
#define MOTOR_LEFT_DIR2   17

#define MOTOR_RIGHT_PWM   26
#define MOTOR_RIGHT_DIR1  22
#define MOTOR_RIGHT_DIR2  23

#define PWM_CHANNEL_LEFT  5
#define PWM_CHANNEL_RIGHT 6
#define PWM_FREQ          100000
#define PWM_RESOLUTION    8

// ============ Encoder Pins (for optional PID) =============
#define ENCODER_LEFT_A    34
#define ENCODER_LEFT_B    35
#define ENCODER_RIGHT_A   12
#define ENCODER_RIGHT_B   14

// ============ Micro-ROS =============
rcl_publisher_t pub_lidar, pub_imu;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
sensor_msgs__msg__LaserScan msg_lidar;
sensor_msgs__msg__Imu msg_imu;

enum states { WAITING_AGENT, AGENT_CONNECTED, AGENT_DISCONNECTED } state;
static volatile bool agent_connected = false;
static TaskHandle_t publish_task_handle = NULL;

// ============ LIDAR Data Structures =============
#define LIDAR_BUFFER_SIZE 4

typedef struct {
  uint32_t range[360];      // Distance in mm
  uint8_t intensity[360];   // Intensity for each angle
  uint8_t scan_id;
  uint32_t scan_time;       // Timestamp in ms
} LidarScan;

static LidarScan lidar_buffers[2];
static std::atomic<uint8_t> lidar_write_buf(0), lidar_read_buf(1);
static uint8_t last_scan_id = 0;
static uint32_t last_lidar_pub = 0;

// ============ IMU Data =============
#define IMU_BUFFER_SIZE 16

typedef struct {
  float ax, ay, az;  // m/s²
  float gx, gy, gz;  // rad/s
  uint32_t ts;       // timestamp
} ImuData;

static ImuData imu_buffer[IMU_BUFFER_SIZE];
static std::atomic<uint8_t> imu_write_idx(0), imu_read_idx(0);
static uint32_t last_imu_read = 0;
static uint32_t last_imu_pub = 0;

// ============ Motor Control Variables =============
static volatile int target_speed_left = 0;   // PWM: -255 to 255
static volatile int target_speed_right = 0;
static volatile int actual_speed_left = 0;
static volatile int actual_speed_right = 0;
static uint32_t last_motor_update = 0;

// ============ Mapping Control =============
enum MappingState { IDLE, MOVING_FORWARD, TURNING, COMPLETING_CIRCUIT } mapping_state = IDLE;
static float total_distance = 0;      // meters traveled
static float total_rotation = 0;      // degrees rotated
static uint32_t mapping_start_time = 0;
static const float CIRCUIT_DISTANCE = 10.0;  // meters to complete circuit
static const float CIRCUIT_PERIMETER = 6.28; // ~2m radius circle = 12.56m, use 6.28 for half

// ============ IMU Sensor =============
MPU6050 mpu;
static uint32_t debug_count = 0;

// ============ CRC8 for LIDAR =============
const uint8_t crc8_table[256] = {
  0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
  // ... (full CRC table omitted for brevity, use original)
};

uint8_t crc8(const uint8_t *data, uint32_t len) {
  uint8_t crc = 0;
  for(uint32_t i = 0; i < len; i++) {
    crc = crc8_table[crc ^ data[i]];
  }
  return crc;
}

// ============ Motor Control Functions =============
void motor_init() {
  // PWM setup
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  
  ledcAttachPin(MOTOR_LEFT_PWM, PWM_CHANNEL_LEFT);
  ledcAttachPin(MOTOR_RIGHT_PWM, PWM_CHANNEL_RIGHT);
  
  // Direction pins
  pinMode(MOTOR_LEFT_DIR1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR2, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR2, OUTPUT);
  
  // Stop motors
  motors_stop();
  
  Serial.println("[MOTOR] Initialized");
}

void motors_stop() {
  ledcWrite(PWM_CHANNEL_LEFT, 0);
  ledcWrite(PWM_CHANNEL_RIGHT, 0);
  digitalWrite(MOTOR_LEFT_DIR1, LOW);
  digitalWrite(MOTOR_LEFT_DIR2, LOW);
  digitalWrite(MOTOR_RIGHT_DIR1, LOW);
  digitalWrite(MOTOR_RIGHT_DIR2, LOW);
  target_speed_left = 0;
  target_speed_right = 0;
}

void motor_left(int speed) {
  // speed: -255 (reverse) to +255 (forward)
  speed = constrain(speed, -255, 255);
  target_speed_left = speed;
  
  if (speed > 0) {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, HIGH);
    ledcWrite(PWM_CHANNEL_LEFT, speed);
  } else if (speed < 0) {
    digitalWrite(MOTOR_LEFT_DIR1, HIGH);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, abs(speed));
  } else {
    digitalWrite(MOTOR_LEFT_DIR1, LOW);
    digitalWrite(MOTOR_LEFT_DIR2, LOW);
    ledcWrite(PWM_CHANNEL_LEFT, 0);
  }
}

void motor_right(int speed) {
  // speed: -255 (reverse) to +255 (forward)
  speed = constrain(speed, -255, 255);
  target_speed_right = speed;
  
  if (speed > 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, HIGH);
    ledcWrite(PWM_CHANNEL_RIGHT, speed);
  } else if (speed < 0) {
    digitalWrite(MOTOR_RIGHT_DIR1, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, abs(speed));
  } else {
    digitalWrite(MOTOR_RIGHT_DIR1, LOW);
    digitalWrite(MOTOR_RIGHT_DIR2, LOW);
    ledcWrite(PWM_CHANNEL_RIGHT, 0);
  }
}

void motors_drive(int throttle, int steering) {
  // throttle: -255 to +255 (forward/backward)
  // steering: -255 to +255 (left/right turn)
  // Uses differential drive: left = throttle + steering, right = throttle - steering
  
  int left_pwm = throttle + steering;
  int right_pwm = throttle - steering;
  
  left_pwm = constrain(left_pwm, -255, 255);
  right_pwm = constrain(right_pwm, -255, 255);
  
  motor_left(left_pwm);
  motor_right(right_pwm);
}

// ============ LIDAR Task (Core 1) =============
void lidarTask(void *pvParameters) {
  Serial.println("[LIDAR] Start");
  LIDAR_SERIAL.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, -1);
  
  uint8_t rx_buffer[512];
  uint32_t rx_pos = 0;
  uint32_t timeout = 0;
  
  while(1) {
    while(LIDAR_SERIAL.available()) {
      uint8_t byte = LIDAR_SERIAL.read();
      rx_buffer[rx_pos++] = byte;
      timeout = millis();
      
      if(rx_pos >= 2 && rx_buffer[0] == LIDAR_HEADER && rx_buffer[1] == 0x2C) {
        // Valid frame header found
        uint32_t frame_len = 5 + PTS_PER_FRAME * 3 + 2;  // Fixed: 5 header + 36 points + 2 checksum
        
        if(rx_pos >= frame_len) {
          // Validate CRC8
          uint8_t check = crc8(rx_buffer, frame_len - 1);
          if(check == rx_buffer[frame_len - 1]) {
            // Parse points
            LidarScan *scan = &lidar_buffers[lidar_write_buf.load()];
            scan->scan_id = rx_buffer[4];
            scan->scan_time = millis();
            
            // Clear ranges
            memset(scan->range, 0, sizeof(scan->range));
            memset(scan->intensity, 0, sizeof(scan->intensity));
            
            // Extract 12 points
            for(int i = 0; i < PTS_PER_FRAME; i++) {
              uint8_t *p = &rx_buffer[5 + i * 3];
              uint16_t distance = (p[1] << 8) | p[0];
              uint8_t intensity = p[2];
              
              // Angle from LIDAR is 0° = front, incrementing counterclockwise
              // Each point is offset by ~1.5° in a full rotation
              int angle_idx = (i * 30) % 360;  // Rough approximation: 12 points in 360°
              
              scan->range[angle_idx] = distance;
              scan->intensity[angle_idx] = intensity;
            }
            
            // Atomically swap buffers
            if(scan->scan_id != last_scan_id) {
              lidar_write_buf.store(1 - lidar_write_buf.load());
              last_scan_id = scan->scan_id;
            }
          }
          
          rx_pos = 0;
        }
      }
      
      if(rx_pos >= sizeof(rx_buffer)) {
        rx_pos = 0;  // Buffer overflow protection
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ============ IMU Task (Core 1) =============
void imuTask(void *pvParameters) {
  Serial.println("[IMU] Start");
  
  Wire.begin(4, 5);
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("[IMU] FAIL");
    vTaskDelete(NULL);
    return;
  }
  
  Serial.println("[IMU] OK");
  
  while(1) {
    if(millis() - last_imu_read >= 10) {  // 100 Hz
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      if(debug_count++ % 100 == 0) {
        Serial.printf("[IMU RAW] ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n", 
          ax, ay, az, gx, gy, gz);
      }
      
      // Convert to physical units (no filtering)
      float accel_x = ax / 16384.0f * 9.81f;
      float accel_y = ay / 16384.0f * 9.81f;
      float accel_z = az / 16384.0f * 9.81f;
      
      float gyro_x = gx / 131.0f * 0.01745f;  // deg/s to rad/s
      float gyro_y = gy / 131.0f * 0.01745f;
      float gyro_z = gz / 131.0f * 0.01745f;
      
      // Axis remapping (same as before)
      float accel_x_ros = accel_y;
      float accel_y_ros = accel_z;
      float accel_z_ros = accel_x - 9.81f;
      
      float gyro_x_ros = gyro_y;
      float gyro_y_ros = gyro_z;
      float gyro_z_ros = gyro_x;
      
      uint8_t current_write = imu_write_idx.load();
      uint8_t next_write = (current_write + 1) % IMU_BUFFER_SIZE;
      
      if(next_write != imu_read_idx.load()) {
        ImuData *imu = &imu_buffer[current_write];
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
    vTaskDelay(pdMS_TO_TICKS(0));
  }
}

// ============ Publish Task =============
void publishTask(void *pvParameters) {
  Serial.println("[PUBLISH] Start");
  
  while(1) {
    if(!agent_connected) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    uint32_t now = millis();
    
    // Publish LIDAR
    uint8_t scan_idx = lidar_read_buf.load();
    if(scan_idx != lidar_write_buf.load()) {
      LidarScan *scan = &lidar_buffers[scan_idx];
      
      msg_lidar.header.stamp.sec = scan->scan_time / 1000;
      msg_lidar.header.stamp.nanosec = (scan->scan_time % 1000) * 1000000;
      msg_lidar.angle_min = 0.0;
      msg_lidar.angle_max = 2.0 * M_PI;
      msg_lidar.angle_increment = 2.0 * M_PI / 360.0;
      msg_lidar.time_increment = 0.0;
      msg_lidar.scan_time = 0.033;  // 30 Hz
      msg_lidar.range_min = 0.15;
      msg_lidar.range_max = 12.0;
      
      // Copy ranges
      for(int i = 0; i < 360; i++) {
        msg_lidar.ranges.data[i] = scan->range[i] / 1000.0f;  // mm to meters
      }
      
      rcl_publish(&pub_lidar, &msg_lidar, NULL);
      lidar_read_buf.store((scan_idx + 1) % LIDAR_BUFFER_SIZE);
    }
    
    // Publish IMU
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

// ============ ROS2 Entity Creation =============
bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32_rover", "", &support) != RCL_RET_OK) return false;
  
  if (rclc_publisher_init_default(&pub_lidar, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&pub_imu, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data") != RCL_RET_OK) return false;
  
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  
  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&pub_lidar, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ============ Mapping Control Functions =============
void start_mapping_circuit() {
  Serial.println("[MAPPING] Starting circuit...");
  mapping_state = MOVING_FORWARD;
  mapping_start_time = millis();
  total_distance = 0;
  total_rotation = 0;
  
  // Start moving forward at 70% speed
  motors_drive(180, 0);
}

void update_mapping() {
  // Simple circuit control without GPS/IMU odometry
  // Just drive in pattern: forward 2m, turn 90°, repeat 4 times
  
  static uint32_t move_timer = 0;
  static int turn_count = 0;
  
  uint32_t elapsed = millis() - mapping_start_time;
  
  if (mapping_state == IDLE) {
    motors_stop();
    return;
  }
  
  if (mapping_state == MOVING_FORWARD) {
    // Drive forward for ~5 seconds (rough estimate for circuit)
    if (elapsed % 5000 < 3000) {  // Move for 3s, turn for 2s
      motors_drive(200, 0);  // Forward
    } else {
      mapping_state = TURNING;
    }
  }
  else if (mapping_state == TURNING) {
    motors_drive(0, 200);  // Turn right (positive steering)
    if (elapsed % 5000 > 3500) {  // Turn for ~500ms
      turn_count++;
      if (turn_count >= 4) {
        // Completed circuit
        motors_stop();
        mapping_state = IDLE;
        Serial.println("[MAPPING] Circuit complete!");
        Serial.printf("[MAPPING] Total time: %lu ms\n", elapsed);
      } else {
        mapping_state = MOVING_FORWARD;
      }
    }
  }
}

// ============ Main Setup =============
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== MAPPING FIRMWARE ===");
  
  // Initialize motors first
  motor_init();
  
  // Start sensor tasks on core 1
  xTaskCreatePinnedToCore(lidarTask, "LIDAR", 5120, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(imuTask, "IMU", 4096, NULL, 3, NULL, 1);
  
  // WiFi
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
  
  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  state = WAITING_AGENT;
  
  // Start mapping after 3 seconds
  delay(3000);
  start_mapping_circuit();
}

// ============ Main Loop =============
void loop() {
  static uint32_t last_agent_check = 0;
  uint32_t now = millis();
  
  switch(state) {
    case WAITING_AGENT:
      if(WiFi.status() == WL_CONNECTED) {
        if(now - last_agent_check >= 1000) {
          if(rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            Serial.println("[AGENT] OK");
            if(create_entities()) {
              state = AGENT_CONNECTED;
              agent_connected = true;
              
              if(publish_task_handle != NULL) {
                vTaskDelete(publish_task_handle);
                publish_task_handle = NULL;
                delay(100);
              }
              
              xTaskCreatePinnedToCore(publishTask, "PUBLISH", 4096, NULL, 2, &publish_task_handle, 0);
              Serial.println("[STATE] CONNECTED");
            } else {
              destroy_entities();
              state = WAITING_AGENT;
              Serial.println("[AGENT] create_entities failed");
            }
          } else {
            Serial.print(".");
          }
          last_agent_check = now;
        }
      }
      break;
      
    case AGENT_CONNECTED:
      if(WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Lost");
        agent_connected = false;
        state = AGENT_DISCONNECTED;
        break;
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      
      // ===== MAPPING LOGIC =====
      update_mapping();
      
      vTaskDelay(pdMS_TO_TICKS(1));
      break;
      
    case AGENT_DISCONNECTED:
      agent_connected = false;
      if(publish_task_handle != NULL) {
        vTaskDelete(publish_task_handle);
        publish_task_handle = NULL;
        delay(100);
      }
      destroy_entities();
      state = WAITING_AGENT;
      Serial.println("[STATE] Reconnecting...");
      last_agent_check = 0;
      delay(1000);
      break;
  }
  
  delay(100);
}
