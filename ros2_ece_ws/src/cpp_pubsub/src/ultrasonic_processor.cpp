#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class PotentiometerProcessor : public rclcpp::Node
{
public:
  PotentiometerProcessor() : Node("potentiometer_processor")
  {
    sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/potentiometer/value",
      10,
      std::bind(&PotentiometerProcessor::potentiometer_callback, this,
                std::placeholders::_1));

    pub_ = this->create_publisher<std_msgs::msg::String>(
      "/ultrasonic/cmd",
      10);

    RCLCPP_INFO(this->get_logger(), "Potentiometer processor started");
  }

private:
  void potentiometer_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    std_msgs::msg::String cmd;
    if (msg->data < 1.65) {
      cmd.data = "gauche";
    } else {
      cmd.data = "droite";
    }
    pub_->publish(cmd);
    RCLCPP_INFO(
      this->get_logger(),
      "Potentiometer: %.2f V -> %s",
      msg->data,
      cmd.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PotentiometerProcessor>());
  rclcpp::shutdown();
  return 0;
}
