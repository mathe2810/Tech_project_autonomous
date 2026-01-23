#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("lidar_subscriber"), scan_count_(0), counter_count_(0)
  {
    // Subscriber pour LaserScan
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MinimalSubscriber::scan_callback, this, _1));
    
    // Subscriber pour Counter
    counter_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "/data", 10, std::bind(&MinimalSubscriber::counter_callback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "LIDAR Subscriber démarré - Topics: /scan, /data");
  }

private:
  void counter_callback(const std_msgs::msg::Int32::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "📊 Counter reçu: %d", msg->data);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_count_++;
    
    // Afficher info générale du scan
    RCLCPP_INFO(this->get_logger(), 
      "\n════════════════════════════════════════════\n"
      "🔍 SCAN LIDAR #%zu reçu\n"
      "Frame ID: %s\n"
      "Nombre de points: %zu\n"
      "Angle Min/Max: %.2f / %.2f rad (%.1f° / %.1f°)\n"
      "Angle Increment: %.4f rad (%.2f°)\n"
      "Range Min/Max: %.2f / %.2f m\n"
      "════════════════════════════════════════════",
      scan_count_,
      msg->header.frame_id.c_str(),
      msg->ranges.size(),
      msg->angle_min, msg->angle_max,
      msg->angle_min * 180.0f / M_PI, msg->angle_max * 180.0f / M_PI,
      msg->angle_increment,
      msg->angle_increment * 180.0f / M_PI,
      msg->range_min, msg->range_max);
    
    // Trouver l'obstacle le plus proche
    float closest_range = msg->range_max + 1.0f;
    int closest_idx = -1;
    float front_min_distance = msg->range_max + 1.0f;
    int front_min_idx = -1;
    
    for(size_t i = 0; i < msg->ranges.size(); ++i)
    {
      float r = msg->ranges[i];
      
      // Trouver minimum global
      if(r < closest_range && r >= msg->range_min && r <= msg->range_max)
      {
        closest_range = r;
        closest_idx = i;
      }
      
      // Trouver minimum en avant (±15° autour de 0°)
      float angle_deg = i * msg->angle_increment * 180.0f / M_PI;
      if(angle_deg > 345.0f || angle_deg < 15.0f)  // Cône frontal ±15°
      {
        if(r < front_min_distance && r >= msg->range_min && r <= msg->range_max)
        {
          front_min_distance = r;
          front_min_idx = i;
        }
      }
    }
    
    // Afficher résultats
    if(closest_idx >= 0)
    {
      float angle = closest_idx * msg->angle_increment;
      float angle_deg = angle * 180.0f / M_PI;
      RCLCPP_WARN(this->get_logger(),
        "⚠️  Obstacle le plus proche: %.2f m @ indice %d (angle: %.1f°)",
        closest_range, closest_idx, angle_deg);
    }
    
    if(front_min_idx >= 0 && front_min_distance < 1.0f)
    {
      float angle = front_min_idx * msg->angle_increment;
      float angle_deg = angle * 180.0f / M_PI;
      RCLCPP_ERROR(this->get_logger(),
        "🚨 DANGER! Obstacle devant: %.2f m @ %.1f° (trop proche!)",
        front_min_distance, angle_deg);
    }
    
    // Afficher statistiques sur les distances
    float avg_distance = 0.0f;
    int valid_points = 0;
    
    for(size_t i = 0; i < msg->ranges.size(); ++i)
    {
      if(msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
      {
        avg_distance += msg->ranges[i];
        valid_points++;
      }
    }
    
    if(valid_points > 0)
    {
      avg_distance /= valid_points;
      RCLCPP_INFO(this->get_logger(),
        "📈 Statistiques - Points valides: %d / %zu, Distance moyenne: %.2f m",
        valid_points, msg->ranges.size(), avg_distance);
    }
    
    // Afficher les 5 premiers points (pour debug)
    if(msg->ranges.size() > 0)
    {
      RCLCPP_DEBUG(this->get_logger(), "Premiers points LIDAR:");
      for(size_t i = 0; i < std::min(size_t(5), msg->ranges.size()); ++i)
      {
        RCLCPP_DEBUG(this->get_logger(),
          "  Point %zu: distance=%.3f m, intensité=%.1f",
          i, msg->ranges[i], 
          i < msg->intensities.size() ? msg->intensities[i] : 0.0f);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr counter_subscription_;
  mutable size_t scan_count_;
  mutable size_t counter_count_;
};
int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<MinimalSubscriber>());
rclcpp::shutdown();
return 0;
}