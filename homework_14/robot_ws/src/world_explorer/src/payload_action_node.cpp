#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {
using underground_world::msg::EnemyDown;
using underground_world::srv::PayloadTrigger;

constexpr auto PAYLOAD_TRIGGER_SERVICE = "/payload/trigger";
constexpr auto ENEMY_DOWN_TOPIC = "/payload/enemy_down";


class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode(): Node("payload_action_node"){
    const auto qos = rclcpp::QoS{10};

    service_ = create_service<PayloadTrigger>(
      PAYLOAD_TRIGGER_SERVICE,
      [this](
        const std::shared_ptr<PayloadTrigger::Request> request,
        std::shared_ptr<PayloadTrigger::Response> response) {
        on_trigger(request, response);
      });

    enemy_down_pub_ = create_publisher<EnemyDown>(ENEMY_DOWN_TOPIC, qos);


    RCLCPP_INFO(get_logger(), "Payload Action node is running; service=%s", PAYLOAD_TRIGGER_SERVICE);
  }

private:
  void on_trigger(const std::shared_ptr<PayloadTrigger::Request> request,
                  std::shared_ptr<PayloadTrigger::Response> response) {
    RCLCPP_INFO(get_logger(),
                "Trigger service called for contact_id=%d at (%d,%d)",
                request->contact_id,
                request->x,
                request->y);
    response->accepted = true;
    response->reason = "trigger accepted";

    EnemyDown msg;
    msg.x = request->x;
    msg.y = request->y;
    msg.contact_id = request->contact_id;

    enemy_down_pub_->publish(msg);
  }

  rclcpp::Service<PayloadTrigger>::SharedPtr service_;
  rclcpp::Publisher<EnemyDown>::SharedPtr enemy_down_pub_;
};
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}
