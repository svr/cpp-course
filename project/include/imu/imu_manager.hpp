#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include "imu_config.hpp"
#include "mpu6050_driver.hpp"
#include "sensor_fusion.hpp"

class MavlinkCommunication;

class ImuManager {
public:
explicit ImuManager(
const std::vector<ImuConfig>& configs);

~ImuManager() = default;

ImuManager(const ImuManager&) = delete;
ImuManager& operator=(const ImuManager&) = delete;

void update(
    float dt,
    uint32_t time_boot_ms,
    MavlinkCommunication& mavlink);


private:
struct ImuState {
std::unique_ptr<Mpu6050Driver> driver;

    bool initialized{false};

    bool reported_online{false};

    bool ever_online{false};

    std::chrono::steady_clock::time_point
        next_init_attempt{};
};

std::vector<ImuState> sensors_;

SensorFusion fusion_;

static constexpr std::size_t MAX_IMUS = 4;

static constexpr auto INIT_RETRY_INTERVAL =
    std::chrono::seconds(1);

};
