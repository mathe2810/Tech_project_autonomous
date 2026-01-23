#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#define TRIG_PIN 5
#define ECHO_PIN 18
rcl_publisher_t distance_pub;
rcl_subscription_t cmd_sub;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
std_msgs__msg__Float32 distance_msg;
std_msgs__msg__String cmd_msg;
char cmd_buffer[20];
float read_distance_cm() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30 ms
    if (duration == 0) return -1.0;
    return duration * 0.034 / 2.0;
}
void timer_callback(rcl_timer_t *, int64_t) {
    float d = read_distance_cm();
    if (d > 0) {
        distance_msg.data = d;
        rcl_publish(&distance_pub, &distance_msg, NULL);
    }
}
void cmd_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    Serial.print("Commande reçue: ");
    Serial.println(msg->data.data);
}
void setup() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_ultrasonic", "", &support);
    rclc_publisher_init_default(
        &distance_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/ultrasonic/distance");
    rclc_subscription_init_default(
        &cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/ultrasonic/cmd");
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(200),
        timer_callback);
    cmd_msg.data.data = cmd_buffer;
    cmd_msg.data.capacity = sizeof(cmd_buffer);
    cmd_msg.data.size = 0;
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(
        &executor, &cmd_sub, &cmd_msg, &cmd_callback, ON_NEW_DATA);
}
void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}