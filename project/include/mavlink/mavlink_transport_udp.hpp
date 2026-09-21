#pragma once

#include <arpa/inet.h>

#include "mavlink_transport.hpp"

class MavlinkTransportUDP : public MavlinkTransport {
public:
    MavlinkTransportUDP(const char* ip, uint16_t port);

    ~MavlinkTransportUDP() override;

    MavlinkTransportUDP(const MavlinkTransportUDP&) = delete;
    MavlinkTransportUDP& operator=(const MavlinkTransportUDP&) = delete;

    MavlinkTransportUDP(MavlinkTransportUDP&& other) noexcept;
    MavlinkTransportUDP& operator=(MavlinkTransportUDP&& other) noexcept;

    int send(const uint8_t* buf, size_t len) override;
    int receive(uint8_t* buf, size_t len) override;

private:
    int sockfd{-1};
    sockaddr_in addr{};
};
