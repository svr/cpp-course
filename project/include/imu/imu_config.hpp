#pragma once

#include <cstdint>
#include <string>

struct ImuConfig {
    uint8_t address;
    std::string device;
};