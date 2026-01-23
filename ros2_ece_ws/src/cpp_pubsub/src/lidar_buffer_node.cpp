#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <deque>
#include <cmath>

using LaserScan = sensor_msgs::msg::LaserScan;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

class LidarBufferNode : public rclcpp::Node {
public:
    LidarBufferNode() : Node("lidar_buffer") {
        // Paramètres
        this->declare_parameter("buffer_size", 50);
        buffer_size_ = this->get_parameter("buffer_size").as_int();
        
        // Subscribers et Publishers
        sub_ = this->create_subscription<LaserScan>(
            "/scan", 10,
            std::bind(&LidarBufferNode::scan_callback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<PointCloud2>("/scan_cloud", 10);
        
        RCLCPP_INFO(this->get_logger(), "🔄 Buffer LIDAR démarré (buffer_size=%d)", buffer_size_);
    }

private:
    void scan_callback(const LaserScan::SharedPtr msg) {
        // Convertir le scan LaserScan en PointCloud2
        auto cloud = std::make_shared<PointCloud2>();
        cloud->header.frame_id = "lidar_link";
        cloud->header.stamp = msg->header.stamp;
        cloud->height = 1;
        cloud->is_dense = true;
        
        // Ajouter les champs (x, y, z, intensity)
        sensor_msgs::PointCloud2Modifier modifier(*cloud);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        // Compter les points valides
        int valid_count = 0;
        for (const auto& range : msg->ranges) {
            if (!std::isnan(range) && !std::isinf(range) && 
                range >= msg->range_min && range <= msg->range_max) {
                valid_count++;
            }
        }
        
        modifier.resize(valid_count);
        
        // Remplir le cloud avec les points valides
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(*cloud, "intensity");
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (std::isnan(range) || std::isinf(range) || 
                range < msg->range_min || range > msg->range_max) {
                continue;
            }
            
            float angle = msg->angle_min + i * msg->angle_increment;
            *iter_x = range * std::cos(angle);
            *iter_y = range * std::sin(angle);
            *iter_z = 0.0f;
            *iter_intensity = msg->intensities.empty() ? 100.0f : msg->intensities[i];
            
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
        }
        
        // Ajouter au buffer
        buffer_.push_back(cloud);
        if (buffer_.size() > static_cast<size_t>(buffer_size_)) {
            buffer_.pop_front();
        }
        
        // Publier le cloud accumulé
        publish_accumulated_cloud();
    }
    
    void publish_accumulated_cloud() {
        if (buffer_.empty()) return;
        
        // Créer un cloud avec tous les points du buffer
        auto combined_cloud = std::make_shared<PointCloud2>();
        combined_cloud->header.frame_id = "lidar_link";
        combined_cloud->header.stamp = this->now();
        combined_cloud->height = 1;
        combined_cloud->is_dense = true;
        
        // Compter le total de points
        size_t total_points = 0;
        for (const auto& cloud : buffer_) {
            total_points += cloud->width;
        }
        
        // Configurer les champs
        sensor_msgs::PointCloud2Modifier modifier(*combined_cloud);
        modifier.setPointCloud2Fields(4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(total_points);
        
        // Copier tous les points
        sensor_msgs::PointCloud2Iterator<float> iter_x(*combined_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*combined_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*combined_cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(*combined_cloud, "intensity");
        
        for (const auto& cloud : buffer_) {
            sensor_msgs::PointCloud2ConstIterator<float> src_x(*cloud, "x");
            sensor_msgs::PointCloud2ConstIterator<float> src_y(*cloud, "y");
            sensor_msgs::PointCloud2ConstIterator<float> src_z(*cloud, "z");
            sensor_msgs::PointCloud2ConstIterator<float> src_intensity(*cloud, "intensity");
            
            for (size_t i = 0; i < cloud->width; ++i) {
                *iter_x = *src_x;
                *iter_y = *src_y;
                *iter_z = *src_z;
                *iter_intensity = *src_intensity;
                
                ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
                ++src_x; ++src_y; ++src_z; ++src_intensity;
            }
        }
        
        pub_->publish(*combined_cloud);
    }
    
    rclcpp::Subscription<LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<PointCloud2>::SharedPtr pub_;
    std::deque<PointCloud2::SharedPtr> buffer_;
    int buffer_size_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarBufferNode>());
    rclcpp::shutdown();
    return 0;
}
