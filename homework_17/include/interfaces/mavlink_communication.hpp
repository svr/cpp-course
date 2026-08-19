#pragma once
#include <memory>
#include <mavlink/common/common.hpp>

#include "runnable_thread.hpp"
#include "interfaces/mavlink_transport.hpp"

namespace {
    constexpr uint8_t SYS_ID = 1;
    constexpr uint8_t COMP_ID = static_cast<uint8_t>(mavlink::minimal::MAV_COMPONENT::COMP_ID_AUTOPILOT1);
}

class MavlinkCommunication : public RunnableThread {
    private:
        std::unique_ptr<MavlinkTransport> transport;

        template <typename T>
        void send_msg(const T& msg) const {
            mavlink::mavlink_message_t packet{};
            mavlink::MsgMap map(packet);
            msg.serialize(map);

            mavlink::mavlink_finalize_message_chan(
                &packet,
                SYS_ID,
                COMP_ID,
                mavlink::MAVLINK_COMM_0,
                T::MIN_LENGTH,
                T::LENGTH,
                T::CRC_EXTRA
            );

            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            transport->send(buf, mavlink::mavlink_msg_to_send_buffer(buf, &packet));
        }

        template <typename T>
        void send_msg_ack(const T& msg) const {
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
                std::cout << "Retrying to send message, attempt " << (i + 1) << std::endl;
            }
            throw std::runtime_error("Command not accepted within timeout");
        }

        bool wait_result_accepted() const;
        void send_heartbeat() const;
    public:
        MavlinkCommunication(std::unique_ptr<MavlinkTransport> transport)
            : RunnableThread(),
            transport(std::move(transport)) {}

        virtual ~MavlinkCommunication() = default;
        
        void run() override;
        void send_attitude(float roll, float pitch, float yaw, uint32_t time_boot_ms) const;
        void send_position(int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, uint16_t hdg, uint32_t time_boot_ms) const;
        void send_drop(float lat, float lon, float alt) const;
};