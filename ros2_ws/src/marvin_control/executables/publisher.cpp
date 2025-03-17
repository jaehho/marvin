#include <memory>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("pose_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("marvin_poses", 10);

    auto timer_callback =
      [this]() -> void {
        // Define the two alternating joint positions
        std::vector<double> joint_positions1 = {0.0, -100, -9, -46, 0.0, -100, -9, -46};
        std::vector<double> joint_positions2 = {0.0, 90, -6, -81, 0.0, 90, -6, -81};

        // Alternate between the two positions every second
        std::vector<double> current_positions = (count_ % 2 == 0) ? joint_positions1 : joint_positions2;

        // Create and publish the message
        auto message = std_msgs::msg::Float64MultiArray();
        message.data = current_positions;
        RCLCPP_INFO(this->get_logger(), "Publishing joint positions");
        this->publisher_->publish(message);

        count_++;
      };
    timer_ = this->create_wall_timer(7s, timer_callback);

    
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
