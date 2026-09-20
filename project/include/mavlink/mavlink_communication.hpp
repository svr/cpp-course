#pragma once

#include <vector>
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <mavlink/common/common.hpp>
#include <mavlink/standard/standard.hpp>

#include "mavlink_transport.hpp"

class MavlinkCommunication {
public:
  explicit MavlinkCommunication(std::unique_ptr<MavlinkTransport> transport)
      : transport(std::move(transport)) {}

  ~MavlinkCommunication() = default;

  void start();
  void stop();
  void send_attitude(float roll, float pitch, float yaw, float rollspeed,
                     float pitchspeed, float yawspeed,
                     uint32_t time_boot_ms) const;

  void send_position(int32_t lat, int32_t lon, int32_t alt, int16_t vx,
                     int16_t vy, uint16_t hdg, uint32_t time_boot_ms) const;

  void send_info(const char *text) const;
  void send_warning(const char *text) const;
  void send_error(const char *text) const;

  void send_sys_status(uint32_t sensors_present, uint32_t sensors_enabled,
                       uint32_t sensors_health, uint16_t cpu_load = 0,
                       uint32_t sensors_present_extended = 0,
                       uint32_t sensors_enabled_extended = 0,
                       uint32_t sensors_health_extended = 0) const;
  void send_imu_status(const std::vector<bool> &healthy) const;

private:
  static constexpr uint8_t SYS_ID = 1;
  static constexpr uint8_t COMP_ID = static_cast<uint8_t>(mavlink::minimal::MAV_COMPONENT::COMP_ID_AUTOPILOT1);

  std::atomic<bool> shouldStop{false};
  std::thread heartbeat_thread;
  std::unique_ptr<MavlinkTransport> transport;

  void heartbeat();
  void send_status_text(uint8_t severity, const char *text) const;

  template <typename T>
  void send_msg(const T &msg) const {
    if (!transport) {
      return;
    }

    mavlink::mavlink_message_t packet{};
    mavlink::MsgMap map(packet);
    msg.serialize(map);

    mavlink::mavlink_finalize_message_chan(
        &packet, SYS_ID, COMP_ID, mavlink::MAVLINK_COMM_0, T::MIN_LENGTH,
        T::MIN_LENGTH, T::CRC_EXTRA);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink::mavlink_msg_to_send_buffer(buf, &packet);
    transport->send(buf, len);
  }

  template <typename T>
  void send_msg_ack(const T &msg) const {
    constexpr int MAX_ATTEMPTS = 5;
    constexpr auto INITIAL_BACKOFF = std::chrono::milliseconds(10);

    auto backoff = INITIAL_BACKOFF;
    for (int i = 0; i < MAX_ATTEMPTS; ++i) {
      send_msg(msg);

      if (wait_result_accepted()) {
        return;
      }
      std::this_thread::sleep_for(backoff);
      backoff *= 2;
    }
    throw std::runtime_error("Command not accepted within timeout");
  }

  bool wait_result_accepted() const;
  void send_heartbeat() const;
};