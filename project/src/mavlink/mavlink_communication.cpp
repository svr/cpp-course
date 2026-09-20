#include <chrono>
#include <cstdint>
#include <cstring>

#include "mavlink_communication.hpp"

namespace mavlink {

const mavlink_msg_entry_t *mavlink_get_msg_entry(uint32_t msgid) {
  for (const auto &entry : standard::MESSAGE_ENTRIES) {
    if (entry.msgid == msgid) {
      return &entry;
    }
  }
  return nullptr;
}

} // namespace mavlink

bool MavlinkCommunication::wait_result_accepted() const {
  if (!transport) {
    return false;
  }

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const int resp = transport->receive(buf, sizeof(buf));
  if (resp <= 0) {
    return false;
  }

  mavlink::mavlink_message_t message{};
  mavlink::mavlink_status_t status{};

  for (int i = 0; i < resp; ++i) {
    if (mavlink::mavlink_parse_char(mavlink::MAVLINK_COMM_0, buf[i], &message,
                                    &status) == 1) {
      if (message.msgid == mavlink::common::msg::COMMAND_ACK::MSG_ID) {
        mavlink::common::msg::COMMAND_ACK command_ack{};
        mavlink::MsgMap map(&message);
        command_ack.deserialize(map);

        return command_ack.result == static_cast<uint8_t>(mavlink::common::MAV_RESULT::ACCEPTED);
      }
    }
  }
  return false;
}

void MavlinkCommunication::send_heartbeat() const {
  mavlink::minimal::msg::HEARTBEAT msg{};
  msg.type = static_cast<uint8_t>(mavlink::minimal::MAV_TYPE::ONBOARD_CONTROLLER);
  msg.autopilot = static_cast<uint8_t>(mavlink::minimal::MAV_AUTOPILOT::GENERIC);
  msg.base_mode = 0;
  msg.custom_mode = 0;
  msg.system_status = static_cast<uint8_t>(mavlink::minimal::MAV_STATE::ACTIVE);

  send_msg(msg);
}

void MavlinkCommunication::heartbeat() {
  using namespace std::chrono_literals;

  while (!shouldStop) {
    try {
      send_heartbeat();
    } catch (const std::exception &e) {
      std::cerr << "[WARN] heartbeat send failed: " << e.what() << "\n";
    }
    std::this_thread::sleep_for(1s);
  }
}

void MavlinkCommunication::start() {
  shouldStop = false;
  heartbeat_thread = std::thread([this]() { heartbeat(); });
}

void MavlinkCommunication::stop() {
  shouldStop = true;
  if (heartbeat_thread.joinable()) {
    heartbeat_thread.join();
  }
}

void MavlinkCommunication::send_attitude(float roll, float pitch, float yaw,
                                         float rollspeed, float pitchspeed,
                                         float yawspeed,
                                         uint32_t time_boot_ms) const {
  mavlink::common::msg::ATTITUDE msg{};
  msg.time_boot_ms = time_boot_ms;
  msg.roll = roll;
  msg.pitch = pitch;
  msg.yaw = yaw;
  msg.rollspeed = rollspeed;
  msg.pitchspeed = pitchspeed;
  msg.yawspeed = yawspeed;

  send_msg(msg);
}

void MavlinkCommunication::send_position(int32_t lat, int32_t lon, int32_t alt,
                                         int16_t vx, int16_t vy, uint16_t hdg,
                                         uint32_t time_boot_ms) const {
  mavlink::standard::msg::GLOBAL_POSITION_INT msg{};
  msg.time_boot_ms = time_boot_ms;
  msg.lat = lat;
  msg.lon = lon;
  msg.alt = alt;
  msg.relative_alt = 0;
  msg.vx = vx;
  msg.vy = vy;
  msg.vz = 0;
  msg.hdg = hdg;

  send_msg(msg);
}

void MavlinkCommunication::send_status_text(uint8_t severity,
                                            const char *text) const {
  mavlink::common::msg::STATUSTEXT msg{};
  msg.severity = severity;

  std::memset(msg.text.data(), 0, msg.text.size());
  std::strncpy(reinterpret_cast<char *>(msg.text.data()), text,
               msg.text.size() - 1);

  send_msg(msg);
}
void MavlinkCommunication::send_info(const char *text) const {
  send_status_text(static_cast<uint8_t>(mavlink::common::MAV_SEVERITY::INFO), text);
}

void MavlinkCommunication::send_warning(const char *text) const {
  send_status_text(static_cast<uint8_t>(mavlink::common::MAV_SEVERITY::WARNING), text);
}

void MavlinkCommunication::send_error(const char *text) const {
  send_status_text(static_cast<uint8_t>(mavlink::common::MAV_SEVERITY::ERROR),text);
}

void MavlinkCommunication::send_sys_status(
    uint32_t sensors_present, uint32_t sensors_enabled, uint32_t sensors_health,
    uint16_t cpu_load, uint32_t sensors_present_extended,
    uint32_t sensors_enabled_extended, uint32_t sensors_health_extended) const {
  mavlink::common::msg::SYS_STATUS msg{};

  msg.onboard_control_sensors_present = sensors_present;
  msg.onboard_control_sensors_enabled = sensors_enabled;
  msg.onboard_control_sensors_health = sensors_health;
  msg.load = cpu_load;
  msg.onboard_control_sensors_present_extended = sensors_present_extended;
  msg.onboard_control_sensors_enabled_extended = sensors_enabled_extended;
  msg.onboard_control_sensors_health_extended = sensors_health_extended;

  send_msg(msg);
}

void MavlinkCommunication::send_imu_status(const std::vector<bool> &healthy) const {
  using mavlink::common::MAV_SYS_STATUS_SENSOR;
  using mavlink::common::MAV_SYS_STATUS_SENSOR_EXTENDED;

  if (healthy.empty()) {
    send_sys_status(0, 0, 0, 0, 0, 0, 0);

    return;
  }

  if (healthy.size() > 4) {
    throw std::invalid_argument("MAVLink IMU status supports at most 4 IMUs");
  }

  uint32_t present = 0;
  uint32_t enabled = 0;
  uint32_t health = 0;

  uint32_t present_ext = 0;
  uint32_t enabled_ext = 0;
  uint32_t health_ext = 0;

  if (healthy.size() >= 1) {
    const uint32_t bits = static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_GYRO) | static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_ACCEL);

    present |= bits;
    enabled |= bits;

    if (healthy[0]) {
      health |= bits;
    }
  }

  if (healthy.size() >= 2) {
    const uint32_t bits = static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_GYRO2) | static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR::SENSOR_3D_ACCEL2);

    present |= bits;
    enabled |= bits;

    if (healthy[1]) {
      health |= bits;
    }
  }

  if (healthy.size() >= 3) {
    const uint32_t bits = static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR_EXTENDED::EXTENDED_3D_GYRO3) | static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR_EXTENDED::EXTENDED_3D_ACCEL3);

    present_ext |= bits;
    enabled_ext |= bits;

    if (healthy[2]) {
      health_ext |= bits;
    }
  }

  if (healthy.size() >= 4) {
    const uint32_t bits = static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR_EXTENDED::EXTENDED_3D_GYRO4) | static_cast<uint32_t>(MAV_SYS_STATUS_SENSOR_EXTENDED::EXTENDED_3D_ACCEL4);

    present_ext |= bits;
    enabled_ext |= bits;

    if (healthy[3]) {
      health_ext |= bits;
    }
  }

  send_sys_status(present, enabled, health, 0, present_ext, enabled_ext, health_ext);
}