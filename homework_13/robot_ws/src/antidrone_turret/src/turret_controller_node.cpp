#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "antidrone_turret/turret_controller.hpp"

#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"

constexpr auto yaw_servo_topic = "/servo/cmd";
constexpr auto gimbal_topic = "/gimbal/cmd";
constexpr auto target_topic = "/perception/target";
constexpr auto actuator_trigger_topic = "/actuator/trigger";
constexpr auto actuator_status_topic = "/actuator/status";
constexpr auto turret_status_topic = "/turret/status";

class TurretControllerNode : public rclcpp::Node {
public:
  TurretControllerNode(): Node("turret_controller_node") {
    confidence_threshold_ = declare_parameter<float>("confidence_threshold", 0.8);
    max_distance_m_ = declare_parameter<float>("max_distance_m", 30.0);
    turret_controller_ = antidrone_turret::TurretController(confidence_threshold_, max_distance_m_);

    trigger_client_ = create_client<antidrone_turret::srv::TriggerActuator>(actuator_trigger_topic);

    target_subscription_ = create_subscription<antidrone_turret::msg::Target>(
      target_topic,
      10,
      [this](const antidrone_turret::msg::Target& target) { on_target(target); });

    actuator_status_subscription_ = create_subscription<antidrone_turret::msg::ActuatorStatus>(
      actuator_status_topic,
      10,
      [this](const antidrone_turret::msg::ActuatorStatus& status) { on_actuator_status(status); });

    gimbal_publisher_ = create_publisher<antidrone_turret::msg::GimbalCommand>(gimbal_topic, 10);
    yaw_servo_publisher_ = create_publisher<antidrone_turret::msg::ServoCommand>(yaw_servo_topic, 10);
    turret_status_publisher_ = create_publisher<antidrone_turret::msg::TurretStatus>(turret_status_topic, 10);
  }

private:
  float confidence_threshold_{0.8F};
  float max_distance_m_{30.0F};
  antidrone_turret::TurretController turret_controller_{confidence_threshold_, max_distance_m_};

  rclcpp::Publisher<antidrone_turret::msg::GimbalCommand>::SharedPtr gimbal_publisher_;
  rclcpp::Publisher<antidrone_turret::msg::ServoCommand>::SharedPtr yaw_servo_publisher_;
  rclcpp::Publisher<antidrone_turret::msg::TurretStatus>::SharedPtr turret_status_publisher_;

  rclcpp::Subscription<antidrone_turret::msg::Target>::SharedPtr target_subscription_;
  rclcpp::Subscription<antidrone_turret::msg::ActuatorStatus>::SharedPtr actuator_status_subscription_;

  rclcpp::Client<antidrone_turret::srv::TriggerActuator>::SharedPtr trigger_client_;

  void on_actuator_status(const antidrone_turret::msg::ActuatorStatus& status) {
    RCLCPP_DEBUG(
      get_logger(),
      "Received actuator status: state=%d, trigger_count=%u",
      status.state,
      status.trigger_count
    );
    turret_controller_.set_actuator_state(status.state);
  }

  void on_target(const antidrone_turret::msg::Target& target) {
    RCLCPP_DEBUG(
      get_logger(),
      "Received target: x=%.2f, y=%.2f",
      target.x,
      target.y
    );

    antidrone_turret::State state = turret_controller_.compute_state(target);
    if(state.action_state == antidrone_turret::msg::TurretStatus::ACTION_TRACK) {
      auto servo_command = turret_controller_.compute_servo_command(target.x);
      auto gimbal_command = turret_controller_.compute_gimbal_command(target.y);
      yaw_servo_publisher_->publish(servo_command);
      gimbal_publisher_->publish(gimbal_command);
    }

    if(state.trigger_state == antidrone_turret::msg::TurretStatus::TRIGGER_REQUESTED) {
      RCLCPP_DEBUG(get_logger(), "Triggering actuator");
      auto request = std::make_shared<antidrone_turret::srv::TriggerActuator::Request>();

      request->confidence = target.confidence;
      request->distance_m = target.distance_m;

      trigger_client_->async_send_request(
          request,
          [this](rclcpp::Client<antidrone_turret::srv::TriggerActuator>::SharedFuture future) {
          const auto response = future.get();
          RCLCPP_INFO(
              get_logger(),
              "accepted=%s trigger_count=%u",
              response->accepted ? "true" : "false",
              response->trigger_count);
          });
    }

    antidrone_turret::msg::TurretStatus turret_status_msg;
    turret_status_msg.target_state = static_cast<uint8_t>(state.target_state);
    turret_status_msg.action = static_cast<uint8_t>(state.action_state);
    turret_status_msg.trigger_state = static_cast<uint8_t>(state.trigger_state);
    turret_status_msg.confidence = target.confidence;
    turret_status_msg.distance_m = target.distance_m;
    turret_status_publisher_->publish(turret_status_msg);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}