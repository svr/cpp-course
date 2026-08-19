#pragma once

#include <string>
#include <arpa/inet.h>
#include "mavlink_transport.hpp"

class MavlinkTransportUDP : public MavlinkTransport {
public:
    MavlinkTransportUDP(const std::string& ip, uint16_t port);
    virtual int send(const uint8_t* buf, size_t len) override;
    virtual int receive(uint8_t* buf, size_t len) override;
    virtual ~MavlinkTransportUDP() = default;
private:
    int sockfd;
    struct sockaddr_in addr;
};