#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/robot_metrics.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {
using underground_world::msg::LocalScan;
using underground_world::msg::MoveCommand;
using underground_world::msg::RobotMetrics;
using underground_world::msg::StudentStatus;
using underground_world::srv::PayloadTrigger;

constexpr auto SCAN_TOPIC = "/robot/local_scan";
constexpr auto MOVE_TOPIC = "/robot/cmd_move";
constexpr auto METRICS_TOPIC = "/robot/metrics";
constexpr auto STUDENT_STATUS_TOPIC = "/student/status";
constexpr auto PAYLOAD_TRIGGER_SERVICE = "/payload/trigger";

struct GridPoint {
  int x = 0;
  int y = 0;
};

std::int64_t point_key(const int x, const int y) {
  return (static_cast<std::int64_t>(x) << 32) ^ static_cast<std::uint32_t>(y);
}

bool is_traversable_cell(const std::string& cell_type) {
  return cell_type == "." || cell_type == "S" || cell_type == "x";
}

std::optional<std::string> cell_type_at(const LocalScan& scan, const int x, const int y) {
  for (const auto& cell : scan.cells) {
    if (cell.x == x && cell.y == y) {
      return cell.cell_type;
    }
  }
  return std::nullopt;
}

std::optional<std::uint8_t> direction_to(const GridPoint& from, const GridPoint& to) {
  const auto dx = to.x - from.x;
  const auto dy = to.y - from.y;

  if (dx == 0 && dy == -1) {
    return MoveCommand::UP;
  }
  if (dx == 0 && dy == 1) {
    return MoveCommand::DOWN;
  }
  if (dx == -1 && dy == 0) {
    return MoveCommand::LEFT;
  }
  if (dx == 1 && dy == 0) {
    return MoveCommand::RIGHT;
  }
  return std::nullopt;
}

class WorldExplorerNode final : public rclcpp::Node {
public:
  WorldExplorerNode(): Node("world_explorer_node"){
    const auto qos = rclcpp::QoS{10};

    scan_sub_ = create_subscription<LocalScan>(SCAN_TOPIC, qos, [this](const LocalScan::SharedPtr msg) { on_scan(*msg); });
    metrics_sub_ = create_subscription<RobotMetrics>(METRICS_TOPIC, qos, [this](const RobotMetrics::SharedPtr msg) { on_metrics(*msg); });

    move_pub_ = create_publisher<MoveCommand>(MOVE_TOPIC, qos);
    status_pub_ = create_publisher<StudentStatus>(STUDENT_STATUS_TOPIC, qos);
    trigger_client_ = create_client<PayloadTrigger>(PAYLOAD_TRIGGER_SERVICE);

    RCLCPP_INFO(get_logger(), "World Explorer node is running; trigger_service=%s", PAYLOAD_TRIGGER_SERVICE);
  }

private:
  void publish_status(const std::uint8_t state) {
    StudentStatus status;
    status.state = state;
    status_pub_->publish(status);
  }

  void fail_with_reason(const std::string& reason) {
    if (failed_) {
      return;
    }
    failed_ = true;
    publish_status(StudentStatus::FAILED);
    RCLCPP_ERROR(get_logger(), "Algorithm failed: %s", reason.c_str());
  }

  void request_trigger_for_contact(const int contact_id, const int x, const int y) {
    if (!trigger_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(),
                  "/payload/trigger service is not available yet; retrying contact_id=%d at (%d,%d)",
                  contact_id,
                  x,
                  y);
      return;
    }

    auto request = std::make_shared<PayloadTrigger::Request>();
    request->contact_id = contact_id;
    request->x = x;
    request->y = y;

    pending_triggers_.insert(contact_id);

    trigger_client_->async_send_request(
      request,
      [this, contact_id, x, y](rclcpp::Client<PayloadTrigger>::SharedFuture response_future) {
        pending_triggers_.erase(contact_id);

        const auto response = response_future.get();
        if (response->accepted) {
          processed_contacts_.insert(contact_id);
          RCLCPP_INFO(get_logger(), "Trigger accepted for contact_id=%d at (%d,%d)", contact_id, x, y);
          return;
        }

        RCLCPP_WARN(get_logger(),
                    "Trigger rejected for contact_id=%d at (%d,%d): %s",
                    contact_id,
                    x,
                    y,
                    response->reason.c_str());
      });
  }

  bool mission_complete() const {
    if (!latest_metrics_.has_value()) {
      return false;
    }

    const auto& metrics = *latest_metrics_;
    return metrics.map_coverage_percent >= 100.0F && metrics.contacts_seen == metrics.contacts_down;
  }

  void on_scan(const LocalScan& msg) {
    if (failed_) {
      publish_status(StudentStatus::FAILED);
      return;
    }

    if (mission_complete()) {
      exploration_done_ = true;
      publish_status(StudentStatus::DONE);
      return;
    }

    if (active_scenario_.empty()) {
      active_scenario_ = msg.scenario_name;
      start_cell_ = GridPoint{msg.robot_x, msg.robot_y};
    }

    std::vector<GridPoint> visible_contacts;
    visible_contacts.reserve(msg.cells.size());
    for (const auto& cell : msg.cells) {
      if (cell.cell_type == "C" &&
          processed_contacts_.find(cell.contact_id) == processed_contacts_.end() &&
          pending_triggers_.find(cell.contact_id) == pending_triggers_.end()) {
        visible_contacts.push_back(GridPoint{cell.x, cell.y});
        request_trigger_for_contact(cell.contact_id, cell.x, cell.y);
        if (failed_) {
          return;
        }
      }
    }

    if (!visible_contacts.empty() || !pending_triggers_.empty()) {
      publish_status(StudentStatus::ENGAGING);
      return;
    }

    const GridPoint current{msg.robot_x, msg.robot_y};
    visited_cells_.insert(point_key(current.x, current.y));

    struct DirectionCandidate {
      std::uint8_t direction = MoveCommand::UP;
      GridPoint target;
    };

    const std::vector<DirectionCandidate> candidates{
      {MoveCommand::UP, GridPoint{current.x, current.y - 1}},
      {MoveCommand::RIGHT, GridPoint{current.x + 1, current.y}},
      {MoveCommand::DOWN, GridPoint{current.x, current.y + 1}},
      {MoveCommand::LEFT, GridPoint{current.x - 1, current.y}},
    };

    for (const auto& candidate : candidates) {
      const auto type = cell_type_at(msg, candidate.target.x, candidate.target.y);
      if (!type.has_value() || !is_traversable_cell(*type)) {
        continue;
      }

      if (visited_cells_.find(point_key(candidate.target.x, candidate.target.y)) != visited_cells_.end()) {
        continue;
      }

      dfs_path_.push_back(current);

      MoveCommand move;
      move.direction = candidate.direction;
      move_pub_->publish(move);
      publish_status(StudentStatus::EXPLORING);

      RCLCPP_INFO(get_logger(), "Exploration move direction=%u from (%d,%d) to (%d,%d)",
                  static_cast<unsigned>(candidate.direction),
                  current.x,
                  current.y,
                  candidate.target.x,
                  candidate.target.y);
      return;
    }

    while (!dfs_path_.empty() && dfs_path_.back().x == current.x && dfs_path_.back().y == current.y) {
      dfs_path_.pop_back();
    }

    if (!dfs_path_.empty()) {
      const auto backtrack_target = dfs_path_.back();
      dfs_path_.pop_back();
      const auto direction = direction_to(current, backtrack_target);
      if (direction.has_value()) {
        MoveCommand move;
        move.direction = *direction;
        move_pub_->publish(move);
        publish_status(StudentStatus::RETURNING);
        RCLCPP_INFO(get_logger(), "Backtrack move direction=%u from (%d,%d) to (%d,%d)",
                    static_cast<unsigned>(*direction),
                    current.x,
                    current.y,
                    backtrack_target.x,
                    backtrack_target.y);
        return;
      }

      fail_with_reason("Backtrack target is not adjacent to current robot position");
      return;
    }

    if (!exploration_done_) {
      exploration_done_ = true;
      publish_status(StudentStatus::DONE);
      RCLCPP_INFO(get_logger(), "Exploration completed for scenario=%s", msg.scenario_name.c_str());
      return;
    }

    publish_status(StudentStatus::DONE);
  }

  void on_metrics(const RobotMetrics& msg) {
    latest_metrics_ = msg;
    RCLCPP_INFO(get_logger(), "Received metrics: scenario=%s steps_taken=%u invalid_moves=%u contacts_seen=%u contacts_down=%u",
                msg.scenario_name.c_str(), msg.steps_taken, msg.invalid_moves, msg.contacts_seen, msg.contacts_down);

    if (!failed_ && !exploration_done_ && mission_complete()) {
      exploration_done_ = true;
      publish_status(StudentStatus::DONE);
      RCLCPP_INFO(get_logger(), "Exploration completed for scenario=%s", msg.scenario_name.c_str());
    }
  }


  rclcpp::Subscription<LocalScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<RobotMetrics>::SharedPtr metrics_sub_;
  rclcpp::Publisher<MoveCommand>::SharedPtr move_pub_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_pub_;

  std::string active_scenario_;
  std::optional<GridPoint> start_cell_;
  std::unordered_set<std::int64_t> visited_cells_;
  std::vector<GridPoint> dfs_path_;
  std::unordered_set<int> processed_contacts_;
  std::unordered_set<int> pending_triggers_;
  std::optional<RobotMetrics> latest_metrics_;
  bool exploration_done_ = false;
  bool failed_ = false;

  rclcpp::Client<PayloadTrigger>::SharedPtr trigger_client_;
};
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
