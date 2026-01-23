#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <iomanip>

class SensorFusionListener : public rclcpp::Node
{
public:
    SensorFusionListener() : Node("sensor_fusion_listener")
    {
        // Subscriber IMU
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&SensorFusionListener::imu_callback, this, std::placeholders::_1));

        // Subscriber LIDAR
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SensorFusionListener::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Sensor Fusion Listener démarré!");
        RCLCPP_INFO(this->get_logger(), "Topics: /imu/data, /scan");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
        static int count = 0;
        if (count++ % 5 == 0)  // Afficher toutes les 5 fois (4Hz au lieu de 20Hz)
        {
            RCLCPP_INFO(this->get_logger(),
                "\n=== IMU DATA ===\n"
                "Accel (m/s²): X=%.3f, Y=%.3f, Z=%.3f\n"
                "Gyro (rad/s): X=%.4f, Y=%.4f, Z=%.4f\n"
                "Frame: %s",
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z,
                msg->header.frame_id.c_str());
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
        // Calculer stats du scan
        float min_range = 10.0, max_range = 0.0, avg_range = 0.0;
        int valid_points = 0;

        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            float r = msg->ranges[i];
            if (r > msg->range_min && r < msg->range_max)
            {
                min_range = std::min(min_range, r);
                max_range = std::max(max_range, r);
                avg_range += r;
                valid_points++;
            }
        }

        if (valid_points > 0)
        {
            avg_range /= valid_points;
            RCLCPP_INFO(this->get_logger(),
                "\n=== LIDAR SCAN ===\n"
                "Points: %zu | Valides: %d\n"
                "Range: Min=%.2f m, Max=%.2f m, Avg=%.2f m\n"
                "Angle: [%.2f, %.2f] rad, Increment=%.4f rad",
                msg->ranges.size(), valid_points,
                min_range, max_range, avg_range,
                msg->angle_min, msg->angle_max, msg->angle_increment);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusionListener>());
    rclcpp::shutdown();
    return 0;
}
