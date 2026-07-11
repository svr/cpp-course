#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/servo_command.hpp"

constexpr auto yaw_servo_topic = "/servo/cmd";

const char* servo_direction_to_string(int8_t direction) {
  switch (direction) {
    case antidrone_turret::msg::ServoCommand::LEFT:
      return "LEFT";
    case antidrone_turret::msg::ServoCommand::CENTER:
      return "CENTER";
    case antidrone_turret::msg::ServoCommand::RIGHT:
      return "RIGHT";
    default:
      return "UNKNOWN";
  }
}

class YawServoDriverNode final : public rclcpp::Node {
public:
  YawServoDriverNode(): Node("yaw_servo_driver_node") {
    servo_command_subscription_ = create_subscription<antidrone_turret::msg::ServoCommand>(
        yaw_servo_topic,
        10,
        [this](const antidrone_turret::msg::ServoCommand& command) {
          on_yaw_servo_command(command);
        }
    );
 }

private:
  rclcpp::Subscription<antidrone_turret::msg::ServoCommand>::SharedPtr servo_command_subscription_;

  void on_yaw_servo_command(const antidrone_turret::msg::ServoCommand& command){
    RCLCPP_INFO(
      get_logger(),
      "yaw_servo_driver_node received: direction=%s target_x=%.1f error_x=%.1f",
      servo_direction_to_string(command.direction),
      command.target_x,
      command.error_x
    );
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawServoDriverNode>());
  rclcpp::shutdown();
  return 0;
}
