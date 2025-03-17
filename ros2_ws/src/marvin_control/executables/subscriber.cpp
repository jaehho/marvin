#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("pose_subscriber")
  {
    // Subscribe to the topic
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "marvin_poses", 10,
      [this](std_msgs::msg::Float64MultiArray::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard joint positions:");
        for (const auto &position : msg->data) {
          RCLCPP_INFO(this->get_logger(), "%.2f", position);
        }

        //Stop any ongoing motion
        this->stop_execution();

        // Plan and execute with received joint positions
        this->execute_plan(msg->data);
      });
  }

  // Initialize the MoveGroupInterface
  void initialize_move_group()
  {
    // Use shared_from_this() from rclcpp::Node
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      this->shared_from_this(), "marvin");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

  //Function to stop ongoing motion
  void stop_execution()
  {
    if (move_group_interface_) {
      RCLCPP_WARN(this->get_logger(), "Stopping any ongoing motion...");
      move_group_interface_->stop();
    }
  }

  // Function to plan and execute
  void execute_plan(const std::vector<double> &joint_positions)
  {
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized!");
      return;
    }

    // Set the joint positions as the target
    move_group_interface_->setJointValueTarget(joint_positions);

    

    // Plan the motion
    //at face value, move_group_interface_->move(); should be equivalent
    auto const [success, plan] = [&]() {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface_->plan(msg));
      return std::make_pair(ok, msg);
    }();

    if (success) {
      // Execute the motion
      RCLCPP_INFO(this->get_logger(), "Motion plan succeeded. Executing...");
      move_group_interface_->asyncExecute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed!");
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create the subscriber node as a shared_ptr
  auto subscriber_node = std::make_shared<MinimalSubscriber>();

  // Initialize the MoveGroupInterface
  subscriber_node->initialize_move_group();

  // Spin the node
  rclcpp::spin(subscriber_node);

  rclcpp::shutdown();
  return 0;
}
