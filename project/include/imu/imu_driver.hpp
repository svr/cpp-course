#pragma once
#include "imu_sample.hpp"
#include <optional>

class IImuDriver {
public:
    virtual ~IImuDriver() = default;
    virtual bool init() = 0;
    virtual std::optional<ImuSample> read() = 0;
    virtual bool is_healthy() const = 0;
};

