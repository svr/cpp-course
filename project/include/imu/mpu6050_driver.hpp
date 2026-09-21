#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>


#include "imu_sample.hpp"

class Mpu6050Driver {
public:
  Mpu6050Driver(uint8_t address, std::string device_path);

  ~Mpu6050Driver();

  Mpu6050Driver(const Mpu6050Driver &) = delete;
  Mpu6050Driver &operator=(const Mpu6050Driver &) = delete;
  bool init();
  std::optional<ImuSample> read();
  bool is_healthy() const;

private:
  bool open_device();
  void close_device();

  bool transfer(struct i2c_msg *messages, std::size_t count);
  bool read_bytes(uint8_t reg, uint8_t *buffer, std::size_t length);
  bool read_register(uint8_t reg, uint8_t &value);
  bool write_register(uint8_t reg, uint8_t value);
  static int16_t make_int16(uint8_t high, uint8_t low);

  uint8_t address_;
  std::string device_path_;

  int i2c_fd_{-1};

  std::uint32_t consecutive_failures_{0};

  static constexpr std::uint32_t MAX_CONSECUTIVE_FAILURES = 5;

  static constexpr uint8_t WHO_AM_I_REG = 0x75;
  static constexpr uint8_t EXPECTED_WHO_AM_I = 0x68;

  static constexpr uint8_t PWR_MGMT_1_REG = 0x6B;
  static constexpr uint8_t CONFIG_REG = 0x1A;
  static constexpr uint8_t GYRO_CONFIG_REG = 0x1B;
  static constexpr uint8_t ACCEL_CONFIG_REG = 0x1C;
  static constexpr uint8_t ACCEL_XOUT_H_REG = 0x3B;

  static constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;
  static constexpr float GYRO_SCALE = 3.14159265358979323846f / 180.0f / 131.0f;
};