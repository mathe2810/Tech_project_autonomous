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
#include <Wire.h>
#include <MPU6050.h>
#include <cmath>
#include <atomic>

// ============ WiFi & Network =============
#define WIFI_SSID "iPhone (3)"
#define WIFI_PASSWORD "Dr69qf76&*"
#define AGENT_IP IPAddress(172, 20, 10, 4)
#define AGENT_PORT 8888

// ============ LIDAR Configuration =============
#define LIDAR_RX 16
#define LIDAR_TX -1  // RX-only, no TX
#define LIDAR_BAUD 230400
#define LIDAR_HEADER 0x54
#define PTS_PER_FRAME 12
static HardwareSerial LIDAR_SERIAL(1);  // Use Serial1 for LIDAR

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

// ============ Encoder Pins (NOT USED - no encoders on this robot) =============
// Encoders not available: use IMU gyro + LIDAR scan matching for odometry

// ============ Micro-ROS =============
rcl_publisher_t pub_lidar, pub_imu;
rcl_subscription_t sub_cmd_vel;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
sensor_msgs__msg__LaserScan msg_lidar;
sensor_msgs__msg__Imu msg_imu;
geometry_msgs__msg__Twist msg_cmd_vel;

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

// Pre-allocated buffers for ROS2 messages (MUST be global, before create_entities)
static float lidar_ranges[360];
static float lidar_intensities[360];
static double imu_linear_acceleration_cov[9] = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
static double imu_angular_velocity_cov[9] = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
static char frame_id_lidar[64] = "base_link";
static char frame_id_imu[64] = "imu";

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

// ============ Motor Control Variables =============
static volatile int target_speed_left = 0;   // PWM: -255 to 255
static volatile int target_speed_right = 0;
static volatile int actual_speed_left = 0;
static volatile int actual_speed_right = 0;
static uint32_t last_motor_update = 0;

// ============ ROS2 Control Mode =============
static volatile bool cmd_vel_mode = false;      // ROS2 /cmd_vel control (coverage_node)
static volatile float cmd_vel_linear = 0.0f;    // m/s
static volatile float cmd_vel_angular = 0.0f;   // rad/s
static volatile uint32_t last_cmd_vel_ms = 0;
static const uint32_t CMD_VEL_TIMEOUT_MS = 500;

// ============ IMU Sensor =============
MPU6050 mpu;
static uint32_t debug_count = 0;

// ============ CRC8 for LIDAR =============
// CRC8 table for LD06 LIDAR (polynomial 0x07)
const uint8_t crc8_table[256] = {
  0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
  0xF3, 0xBE, 0x69, 0x24, 0x8A, 0xC7, 0x10, 0x5D, 0x01, 0x4C, 0x9B, 0xD6, 0x78, 0x35, 0xE2, 0xAF,
  0xE5, 0xA8, 0x7F, 0x32, 0x9C, 0xD1, 0x06, 0x4B, 0x17, 0x5A, 0x8D, 0xC0, 0x6E, 0x23, 0xF4, 0xB9,
  0x16, 0x5B, 0x8C, 0xC1, 0x6F, 0x22, 0xF5, 0xB8, 0xE4, 0xA9, 0x7E, 0x33, 0x9D, 0xD0, 0x07, 0x4A,
  0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
  0x39, 0x74, 0xA3, 0xEE, 0x40, 0x0D, 0xDA, 0x97, 0xCB, 0x86, 0x51, 0x1C, 0xB2, 0xFF, 0x28, 0x65,
  0x2F, 0x62, 0xB5, 0xF8, 0x56, 0x1B, 0xCC, 0x81, 0xDD, 0x90, 0x47, 0x0A, 0xA4, 0xE9, 0x3E, 0x73,
  0xDC, 0x91, 0x46, 0x0B, 0xA5, 0xE8, 0x3F, 0x72, 0x2E, 0x63, 0xB4, 0xF9, 0x57, 0x1A, 0xCD, 0x80,
  0x95, 0xD8, 0x0F, 0x42, 0xEC, 0xA1, 0x76, 0x3B, 0x67, 0x2A, 0xFD, 0xB0, 0x1E, 0x53, 0x84, 0xC9,
  0x66, 0x2B, 0xFC, 0xB1, 0x1F, 0x52, 0x85, 0xC8, 0x94, 0xD9, 0x0E, 0x43, 0xED, 0xA0, 0x77, 0x3A,
  0x70, 0x3D, 0xEA, 0xA7, 0x09, 0x44, 0x93, 0xDE, 0x82, 0xCF, 0x18, 0x55, 0xFB, 0xB6, 0x61, 0x2C,
  0x83, 0xCE, 0x19, 0x54, 0xFA, 0xB7, 0x60, 0x2D, 0x71, 0x3C, 0xEB, 0xA6, 0x08, 0x45, 0x92, 0xDF,
  0x5F, 0x12, 0xC5, 0x88, 0x26, 0x6B, 0xBC, 0xF1, 0xAD, 0xE0, 0x37, 0x7A, 0xD4, 0x99, 0x4E, 0x03,
  0xAC, 0xE1, 0x36, 0x7B, 0xD5, 0x98, 0x4F, 0x02, 0x5E, 0x13, 0xC4, 0x89, 0x27, 0x6A, 0xBD, 0xF0,
  0xBA, 0xF7, 0x20, 0x6D, 0xC3, 0x8E, 0x59, 0x14, 0x48, 0x05, 0xD2, 0x9F, 0x31, 0x7C, 0xAB, 0xE6,
  0x49, 0x04, 0xD3, 0x9E, 0x30, 0x7D, 0xAA, 0xE7, 0xBB, 0xF6, 0x21, 0x6C, 0xC2, 0x8F, 0x58, 0x15
};

uint8_t crc8(const uint8_t *data, uint32_t len) {
  uint8_t crc = 0;
  for(uint32_t i = 0; i < len; i++) {
    crc = crc8_table[crc ^ data[i]];
  }
  return crc;
}

// ============ ROS2 Support =============
// ESP32 subscribes /cmd_vel for Nav2 control (fallback to autonomous circuit if no commands)

// ============ ROS2 Callback =============
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_vel_linear = msg->linear.x;
  cmd_vel_angular = msg->angular.z;
  cmd_vel_mode = true;
  last_cmd_vel_ms = millis();
}

// ============ Forward Declarations =============
void motors_stop();
void motor_left(int speed);
void motor_right(int speed);
void motors_drive(int throttle, int steering);

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

void motors_from_twist(float linear_vel, float angular_vel) {
  // Convertir Twist (m/s, rad/s) → PWM (-255 to 255)
  // linear_vel: -1.0 to +1.0 m/s (robot speed)
  // angular_vel: -3.0 to +3.0 rad/s (turning speed)
  
  // Limiter les vitesses
  linear_vel = constrain(linear_vel, -1.0f, 1.0f);
  angular_vel = constrain(angular_vel, -3.0f, 3.0f);
  
  // Convertir en throttle/steering PWM
  // Échelle : 1.0 m/s = 200 PWM (78% vitesse)
  int throttle = (int)(linear_vel * 200.0f);
  
  // Échelle : 1.0 rad/s = 100 PWM (39% rotation)
  int steering = (int)(angular_vel / 3.0f * 100.0f);
  
  motors_drive(throttle, steering);
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
  
  uint32_t last_lidar_pub = 0;
  uint32_t last_imu_pub = 0;
  
  while(1) {
    if(!agent_connected) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    
    uint32_t now = millis();
    
    // Publish LIDAR at max 10 Hz to avoid overwhelming network
    if(now - last_lidar_pub >= 100) {
      uint8_t scan_idx = lidar_read_buf.load();
      if(scan_idx != lidar_write_buf.load()) {
        LidarScan *scan = &lidar_buffers[scan_idx];
        
        // IMPORTANT: Only update message fields, don't reallocate
        msg_lidar.header.stamp.sec = scan->scan_time / 1000;
        msg_lidar.header.stamp.nanosec = (scan->scan_time % 1000) * 1000000;
        msg_lidar.angle_min = 0.0;
        msg_lidar.angle_max = 2.0 * M_PI;
        msg_lidar.angle_increment = 2.0 * M_PI / 360.0;
        msg_lidar.time_increment = 0.0;
        msg_lidar.scan_time = 0.1;  // 10 Hz publish rate
        msg_lidar.range_min = 0.15;
        msg_lidar.range_max = 12.0;
        
        // Copy ranges ONLY - don't touch intensities to avoid pbuf issues
        for(int i = 0; i < 360; i++) {
          msg_lidar.ranges.data[i] = scan->range[i] / 1000.0f;  // mm to meters
        }
        
        rcl_publish(&pub_lidar, &msg_lidar, NULL);
        lidar_read_buf.store((scan_idx + 1) % LIDAR_BUFFER_SIZE);
        last_lidar_pub = now;
      }
    }
    
    // Publish IMU at 20 Hz
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

  if (rclc_subscription_init_default(&sub_cmd_vel, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel") != RCL_RET_OK) return false;
  
  // Initialize LaserScan message with pre-allocated global buffers
  msg_lidar.header.frame_id.data = frame_id_lidar;
  msg_lidar.header.frame_id.size = strlen(frame_id_lidar);
  msg_lidar.ranges.data = lidar_ranges;
  msg_lidar.ranges.capacity = 360;
  msg_lidar.ranges.size = 360;
  
  // NOTE: Intensities disabled to avoid pbuf corruption in lwIP
  // msg_lidar.intensities.data = lidar_intensities;
  // msg_lidar.intensities.capacity = 360;
  // msg_lidar.intensities.size = 360;
  
  // Initialize Imu message with pre-allocated global buffers
  msg_imu.header.frame_id.data = frame_id_imu;
  msg_imu.header.frame_id.size = strlen(frame_id_imu);
  
  // Copy covariance values directly (they are arrays, not sequences)
  for(int i = 0; i < 9; i++) {
    msg_imu.linear_acceleration_covariance[i] = imu_linear_acceleration_cov[i];
    msg_imu.angular_velocity_covariance[i] = imu_angular_velocity_cov[i];
  }
  
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;

  if (rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel,
      &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;
  
  Serial.println("[ROS2] Entities created");
  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&pub_lidar, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_subscription_fini(&sub_cmd_vel, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
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
  
  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  state = WAITING_AGENT;
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
      
      // ===== MOTOR CONTROL =====
      // Listen to /cmd_vel from Nav2
      if(cmd_vel_mode) {
        if(millis() - last_cmd_vel_ms > CMD_VEL_TIMEOUT_MS) {
          cmd_vel_mode = false;
          motors_stop();
          Serial.println("[CMD_VEL] Timeout - motors stopped");
        } else {
          motors_from_twist(cmd_vel_linear, cmd_vel_angular);
        }
      } else {
        motors_stop();  // No commands = stop
      }
      
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
