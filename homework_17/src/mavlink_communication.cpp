#include <chrono>
#include <stdexcept>
#include <thread>
#include "mavlink_communication.hpp"

namespace mavlink {
    const mavlink_msg_entry_t* mavlink_get_msg_entry(uint32_t msgid) {
        for (const auto& entry : common::MESSAGE_ENTRIES) {
            if (entry.msgid == msgid) {
                return &entry;
            }
        }
        return nullptr;
    }
} // namespace mavlink


bool MavlinkCommunication::wait_result_accepted() const {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int resp = transport->receive(buf, sizeof(buf));
    if (resp <= 0) {
        return false;
    }

    mavlink::mavlink_message_t message;
    mavlink::mavlink_status_t status;
    for (int i = 0; i < resp; ++i) {
        if (mavlink::mavlink_parse_char(mavlink::MAVLINK_COMM_0, buf[i], &message, &status) == 1) {
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

    msg.type = static_cast<uint8_t>(mavlink::minimal::MAV_TYPE::QUADROTOR);
    msg.autopilot = static_cast<uint8_t>(mavlink::minimal::MAV_AUTOPILOT::GENERIC);
    msg.base_mode = 0;
    msg.custom_mode = 0;
    msg.system_status = static_cast<uint8_t>(mavlink::minimal::MAV_STATE::ACTIVE);

    send_msg(msg);
}

void MavlinkCommunication::run() {
    while (!shouldStop) {
        send_heartbeat();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MavlinkCommunication::send_attitude(float roll, float pitch, float yaw, uint32_t time_boot_ms) const {
    mavlink::common::msg::ATTITUDE msg{};

    msg.time_boot_ms = time_boot_ms;
    msg.roll = roll;
    msg.pitch = pitch;
    msg.yaw = yaw;
    msg.rollspeed = 0.0f;
    msg.pitchspeed = 0.0f;
    msg.yawspeed = 0.0f;

    send_msg(msg);
}

void MavlinkCommunication::send_position(int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, uint16_t hdg, uint32_t time_boot_ms) const {
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

void MavlinkCommunication::send_drop(float lat, float lon, float alt) const {
    mavlink::common::msg::COMMAND_LONG msg{};
    msg.target_system = 1;
    msg.target_component = static_cast<uint8_t>(mavlink::minimal::MAV_COMPONENT::COMP_ID_AUTOPILOT1);
    msg.command = static_cast<uint16_t>(mavlink::common::MAV_CMD::USER_1);
    msg.confirmation = 0;
    msg.param5 = lat;
    msg.param6 = lon;
    msg.param7 = alt;

    send_msg_ack(msg);
}