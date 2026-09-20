#include <algorithm>
#include <cstdio>
#include <string>
#include <utility>

#include "imu_manager.hpp"
#include "mavlink_communication.hpp"

ImuManager::ImuManager(const std::vector<ImuConfig> &configs) {
  const std::size_t count = std::min(configs.size(), MAX_IMUS);

  sensors_.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    ImuState state;

    state.driver =std::make_unique<Mpu6050Driver>(configs[i].address, configs[i].device);
    state.initialized = false;
    state.reported_online = false;
    state.ever_online = false;

    state.next_init_attempt = std::chrono::steady_clock::now();

    sensors_.push_back(std::move(state));
  }
}

void ImuManager::update(float dt, uint32_t time_boot_ms, MavlinkCommunication &mavlink) {
  const auto now = std::chrono::steady_clock::now();

  ImuSample fused_imu{};
  std::size_t valid_count = 0;
  for (std::size_t i = 0; i < sensors_.size(); ++i) {

    auto &sensor = sensors_[i];

    if (!sensor.initialized && now >= sensor.next_init_attempt) {
      sensor.initialized = sensor.driver->init();
      if (!sensor.initialized) {
        sensor.next_init_attempt = now + INIT_RETRY_INTERVAL;
        continue;
      }
    }

    if (!sensor.initialized) {
      continue;
    }

    const auto sample = sensor.driver->read();
    if (!sample.has_value()) {
      if (!sensor.driver->is_healthy()) {
        sensor.initialized = false;
        sensor.next_init_attempt = now + INIT_RETRY_INTERVAL;

        if (sensor.ever_online && sensor.reported_online) {
          sensor.reported_online = false;
          const std::string message = "IMU " + std::to_string(i) + " offline";
          mavlink.send_error(message.c_str());
          std::printf("%s\n", message.c_str());
        }
      }

      continue;
    }

    if (!sensor.reported_online) {
      if (sensor.ever_online) {
        const std::string message = "IMU " + std::to_string(i) + " restored";
        mavlink.send_warning(message.c_str());
        std::printf("%s\n", message.c_str());
      } else {
        const std::string message = "IMU " + std::to_string(i) + " online";
        std::printf("%s\n", message.c_str());
      }

      sensor.reported_online = true;
      sensor.ever_online = true;
    }

    fused_imu.accel_x += sample->accel_x;
    fused_imu.accel_y += sample->accel_y;
    fused_imu.accel_z += sample->accel_z;

    fused_imu.gyro_x += sample->gyro_x;
    fused_imu.gyro_y += sample->gyro_y;
    fused_imu.gyro_z += sample->gyro_z;

    ++valid_count;
  }

  if (valid_count == 0) {
    return;
  }

  const float scale = 1.0f / static_cast<float>(valid_count);

  fused_imu.accel_x *= scale;
  fused_imu.accel_y *= scale;
  fused_imu.accel_z *= scale;

  fused_imu.gyro_x *= scale;
  fused_imu.gyro_y *= scale;
  fused_imu.gyro_z *= scale;


  const AttitudeState attitude = fusion_.update(fused_imu, dt);

  mavlink.send_attitude(attitude.roll, attitude.pitch, attitude.yaw,
                        attitude.rollspeed, attitude.pitchspeed,
                        attitude.yawspeed, time_boot_ms);
}