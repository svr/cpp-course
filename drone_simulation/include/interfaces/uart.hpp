#pragma once
#include <optional>
#include <variant>
#include <mutex>
#include <deque>

#include "common.hpp"
#include "drone_link.hpp"

class UART {
    int fd{-1};
    dlink::Parser parser{};
    std::mutex mtx;
    std::deque<std::variant<
        dlink::Telemetry,
        dlink::TargetPos,
        dlink::AmmoCfg,
        dlink::Result,
        dlink::Control,
        dlink::DroneCfg
    >> pendingPackets;
    static constexpr std::size_t kPendingPacketCapacity = 128;

    public:
        using Packet = std::variant<
            dlink::Telemetry,
            dlink::TargetPos,
            dlink::AmmoCfg,
            dlink::Result,
            dlink::Control,
            dlink::DroneCfg
        >;

        UART(const char* dev);
        ~UART();

        bool isOpen() const;
        std::optional<Packet> readPacket();
        int sendControl(float accel, float turnRate);
};