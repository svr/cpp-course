#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/msg/gimbal_command.hpp"

constexpr auto gimbal_topic = "/gimbal/cmd";

const char* gimbal_direction_to_string(int8_t direction) {
  switch (direction) {
    case antidrone_turret::msg::GimbalCommand::DOWN:
      return "DOWN";
    case antidrone_turret::msg::GimbalCommand::CENTER:
      return "CENTER";
    case antidrone_turret::msg::GimbalCommand::UP:
      return "UP";
    default:
      return "UNKNOWN";
  }
}

class GimbalDriverNode final : public rclcpp::Node {
public:
  GimbalDriverNode(): Node("gimbal_driver_node") {
    gimbal_command_subscription_ = create_subscription<antidrone_turret::msg::GimbalCommand>(
        gimbal_topic,
        10,
        [this](const antidrone_turret::msg::GimbalCommand& command) {
          on_gimbal_command(command);
        }
    );
 }

private:
  rclcpp::Subscription<antidrone_turret::msg::GimbalCommand>::SharedPtr gimbal_command_subscription_;

  void on_gimbal_command(const antidrone_turret::msg::GimbalCommand& command){
    RCLCPP_INFO(
      get_logger(),
      "gimbal_driver_node received: direction=%s target_y=%.1f error_y=%.1f",
      gimbal_direction_to_string(command.direction),
      command.target_y,
      command.error_y
    );
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalDriverNode>());
  rclcpp::shutdown();
  return 0;
}
