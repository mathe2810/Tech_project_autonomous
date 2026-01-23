#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <memory>

class TfBroadcaster : public rclcpp::Node {
public:
    TfBroadcaster() : Node("lidar_tf_broadcaster") {
        // Créer un static broadcaster pour la frame lidar_link
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Publier la transformation: odom -> lidar_link
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "lidar_link";
        
        // Position du LIDAR (0, 0, 0.2 mètres de hauteur)
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.2;
        
        // Pas de rotation (orientation identité)
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        tf_broadcaster_->sendTransform(transform);
        
        RCLCPP_INFO(this->get_logger(), "✅ TF broadcaster démarré: odom -> lidar_link");
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
