#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <iomanip>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node //Create the node
{
public:
  MinimalPublisher()
  : Node("lidar_publisher"), count_(0)
  {
    // Subscriber pour LaserScan (LIDAR de l'ESP32)
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MinimalPublisher::lidar_callback, this, std::placeholders::_1));
    
    // Subscriber pour Int32 (compteur)
    counter_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/data", 10, std::bind(&MinimalPublisher::counter_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "📊 Listeners démarrés - Topics:");
    RCLCPP_INFO(this->get_logger(), "   - /scan (LaserScan depuis ESP32 LIDAR)");
    RCLCPP_INFO(this->get_logger(), "   - /data (Counter)");
  }
private:
  void counter_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Counter received: %d", msg->data);
  }
  
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Compter les points avec détection
    int points_detected = 0;
    float min_range = 999.0f;
    float max_range = 0.0f;
    float avg_range = 0.0f;
    
    for(size_t i = 0; i < msg->ranges.size(); i++) {
      float range = msg->ranges[i];
      if(range > msg->range_min && range < msg->range_max) {
        points_detected++;
        avg_range += range;
        if(range < min_range) min_range = range;
        if(range > max_range) max_range = range;
      }
    }
    
    if(points_detected > 0) {
      avg_range /= points_detected;
    }
    
    // Afficher les statistiques
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "LIDAR Data: " << points_detected << "/" << msg->ranges.size() << " points valid"
        << " | Min: " << min_range << "m"
        << " | Max: " << max_range << "m"  
        << " | Avg: " << avg_range << "m"
        << " | Angle range: " << (msg->angle_max - msg->angle_min) * 180.0f / M_PI << "°";
    
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    
    // Log les 10 premiers points avec détection pour debug
    if(points_detected > 0) {
      std::ostringstream points_str;
      points_str << std::fixed << std::setprecision(3);
      int logged = 0;
      for(size_t i = 0; i < msg->ranges.size() && logged < 10; i++) {
        if(msg->ranges[i] > msg->range_min && msg->ranges[i] < msg->range_max) {
          float angle_deg = (msg->angle_min + i * msg->angle_increment) * 180.0f / M_PI;
          points_str << "\n   [" << i << "] angle=" << angle_deg << "° distance=" << msg->ranges[i] << "m";
          logged++;
        }
      }
      RCLCPP_DEBUG(this->get_logger(), "First detected points:%s", points_str.str().c_str());
    }
  }
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr counter_subscription_;
  size_t count_;
};
int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<MinimalPublisher>());
rclcpp::shutdown();
return 0;
}