#pragma once

#include <cstdint>

struct ImuSample {
    uint64_t timestamp_us = 0;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};
