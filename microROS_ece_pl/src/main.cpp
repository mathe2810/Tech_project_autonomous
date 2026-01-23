#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <Wire.h>
#include <MPU6050.h>

// ==================== CONFIGURATION HARDWARE ====================
#define LIDAR_RX 16    // Serial2 RX
#define LIDAR_TX 17    // Serial2 TX
#define MPU6050_ADDR 0x68  // Adresse I2C par défaut
#define I2C_SDA 21
#define I2C_SCL 22

// ==================== BUFFERS LIDAR ====================
#define LIDAR_POINTS 720
static float lidar_ranges[LIDAR_POINTS];
static float lidar_intensities[LIDAR_POINTS];
static int lidar_index = 0;

// ==================== MPU6050 ====================
MPU6050 mpu;
int16_t ax, ay, az;  // Accelération brute
int16_t gx, gy, gz;  // Gyroscope brut
int16_t temp;         // Température

// ==================== microROS ====================
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

// Publishers
rcl_publisher_t laser_scan_pub;
rcl_publisher_t imu_pub;

// Timers
rcl_timer_t lidar_timer;
rcl_timer_t imu_timer;

// Messages
sensor_msgs__msg__LaserScan laser_scan_msg;
sensor_msgs__msg__Imu imu_msg;

// ==================== LIDAR PROCESSING ====================
void read_lidar_data() {
    if (Serial2.available() > 0) {
        uint8_t byte = Serial2.read();
        
        // Format simplifié: [HEADER(0xFF)][INDEX(1byte)][DISTANCE(2bytes)][INTENSITY(1byte)][CHECKSUM]
        static uint8_t buffer[6];
        static int buf_idx = 0;
        
        if (byte == 0xFF) {  // En-tête
            buf_idx = 0;
            buffer[buf_idx++] = byte;
        } else if (buf_idx > 0 && buf_idx < 6) {
            buffer[buf_idx++] = byte;
            
            if (buf_idx == 6) {  // Paquet complet
                int idx = buffer[1] % LIDAR_POINTS;
                float distance = ((buffer[2] << 8) | buffer[3]) / 1000.0;  // mm to m
                float intensity = buffer[4] / 255.0;  // Normaliser 0-1
                
                lidar_ranges[idx] = distance;
                lidar_intensities[idx] = intensity;
                lidar_index = (idx + 1) % LIDAR_POINTS;
            }
        }
    }
}

// ==================== MPU6050 PROCESSING ====================
void read_mpu_data() {
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    temp = mpu.getTemperature();
}

// ==================== TIMERS CALLBACKS ====================
void lidar_timer_callback(rcl_timer_t *, int64_t) {
    // Publier le scan complet
    laser_scan_msg.header.stamp.sec = (uint32_t)(millis() / 1000);
    laser_scan_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1e6);
    laser_scan_msg.header.frame_id.data = "lidar";
    laser_scan_msg.header.frame_id.size = 5;
    
    laser_scan_msg.angle_min = 0.0;
    laser_scan_msg.angle_max = 2.0 * 3.14159265359;  // 2π
    laser_scan_msg.angle_increment = 0.00872664626;  // 2π / 720
    laser_scan_msg.time_increment = 0.0;
    laser_scan_msg.scan_time = 0.1;  // 10 Hz
    laser_scan_msg.range_min = 0.1;
    laser_scan_msg.range_max = 10.0;
    
    // Copier les ranges et intensities
    for (int i = 0; i < LIDAR_POINTS; i++) {
        laser_scan_msg.ranges.data[i] = lidar_ranges[i];
        laser_scan_msg.intensities.data[i] = lidar_intensities[i];
    }
    
    laser_scan_msg.ranges.size = LIDAR_POINTS;
    laser_scan_msg.intensities.size = LIDAR_POINTS;
    
    rcl_publish(&laser_scan_pub, &laser_scan_msg, NULL);
    
    Serial.println("[LIDAR] Scan published");
}

void imu_timer_callback(rcl_timer_t *, int64_t) {
    read_mpu_data();
    
    // Convertir les données brutes en SI units
    // MPU6050: ±2g (16384 LSB/g), ±250°/s (131 LSB/°/s)
    float accel_scale = 9.81 / 16384.0;  // m/s²
    float gyro_scale = 3.14159265359 / (180.0 * 131.0);  // rad/s
    
    imu_msg.header.stamp.sec = (uint32_t)(millis() / 1000);
    imu_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1e6);
    imu_msg.header.frame_id.data = "imu_link";
    imu_msg.header.frame_id.size = 8;
    
    // Accélération (m/s²)
    imu_msg.linear_acceleration.x = ax * accel_scale;
    imu_msg.linear_acceleration.y = ay * accel_scale;
    imu_msg.linear_acceleration.z = az * accel_scale;
    
    // Vitesse angulaire (rad/s)
    imu_msg.angular_velocity.x = gx * gyro_scale;
    imu_msg.angular_velocity.y = gy * gyro_scale;
    imu_msg.angular_velocity.z = gz * gyro_scale;
    
    // Covariance (optionnel)
    for (int i = 0; i < 9; i++) {
        imu_msg.linear_acceleration_covariance[i] = 0.01;
        imu_msg.angular_velocity_covariance[i] = 0.01;
    }
    
    rcl_publish(&imu_pub, &imu_msg, NULL);
    
    float temp_celsius = (temp + 12412.0) / 340.0;
    Serial.printf("[MPU6050] Accel(m/s²): [%.2f, %.2f, %.2f] | Gyro(rad/s): [%.4f, %.4f, %.4f] | Temp: %.1f°C\n",
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z,
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z,
        temp_celsius);
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n[SETUP] Initialisation ESP32 + LIDAR + MPU6050 + microROS");
    
    // ===== I2C & MPU6050 =====
    Serial.println("[I2C] Initialisation...");
    Wire.begin(I2C_SDA, I2C_SCL, 400000);  // 400kHz
    delay(100);
    
    if (!mpu.begin(MPU6050_ADDR)) {
        Serial.println("[MPU6050] ERROR: Impossible de détecter le capteur!");
        while (true) {
            delay(1000);
        }
    }
    Serial.println("[MPU6050] Détecté et initialisé!");
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);   // ±2g
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250°/s
    
    // ===== LIDAR Serial =====
    Serial2.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
    Serial.println("[LIDAR] Serial2 initialisé");
    
    // ===== microROS =====
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();
    
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_sensor_fusion", "", &support);
    
    // Publisher LIDAR
    rclc_publisher_init_default(
        &laser_scan_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "/scan");
    
    // Publisher IMU
    rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data");
    
    // Timers
    rclc_timer_init_default(&lidar_timer, &support, RCL_MS_TO_NS(100), lidar_timer_callback);  // 10 Hz
    rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(50), imu_timer_callback);      // 20 Hz
    
    // Executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &lidar_timer);
    rclc_executor_add_timer(&executor, &imu_timer);
    
    // Init buffers
    laser_scan_msg.ranges.data = lidar_ranges;
    laser_scan_msg.ranges.capacity = LIDAR_POINTS;
    laser_scan_msg.intensities.data = lidar_intensities;
    laser_scan_msg.intensities.capacity = LIDAR_POINTS;
    
    Serial.println("\n[SETUP] ✓ Tous les composants initialisés!");
    Serial.println("[SETUP] Prêt à publier les données...\n");
}

// ==================== MAIN LOOP ====================
void loop() {
    read_lidar_data();
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    delay(5);
}