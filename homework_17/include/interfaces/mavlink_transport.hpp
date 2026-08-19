#pragma once
#include <cstdint>

class MavlinkTransport {
public:
    virtual int send(const uint8_t* buf, size_t len) = 0;
    virtual int receive(uint8_t* buf, size_t len) = 0;
    virtual ~MavlinkTransport() = default;
};