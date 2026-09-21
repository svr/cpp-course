#include "mpu6050_driver.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>
#include <utility>

Mpu6050Driver::Mpu6050Driver(uint8_t address, std::string device_path)
    : address_(address), device_path_(std::move(device_path)) {}

Mpu6050Driver::~Mpu6050Driver() { close_device(); }

bool Mpu6050Driver::open_device() {
  if (i2c_fd_ >= 0) {
    return true;
  }

  i2c_fd_ = ::open(device_path_.c_str(), O_RDWR | O_CLOEXEC);

  return i2c_fd_ >= 0;
}

void Mpu6050Driver::close_device() {
  if (i2c_fd_ >= 0) {
    ::close(i2c_fd_);
    i2c_fd_ = -1;
  }

  consecutive_failures_ = 0;
}

bool Mpu6050Driver::transfer(struct i2c_msg *messages, std::size_t count) {
  if (i2c_fd_ < 0 || messages == nullptr || count == 0) {
    return false;
  }

  struct i2c_rdwr_ioctl_data data{};

  data.msgs = messages;
  data.nmsgs = static_cast<__u32>(count);

  const int result = ::ioctl(i2c_fd_, I2C_RDWR, &data);

  return result == static_cast<int>(count);
}

bool Mpu6050Driver::read_bytes(uint8_t reg, uint8_t *buffer,
                               std::size_t length) {
  if (i2c_fd_ < 0 || buffer == nullptr || length == 0 || length > UINT16_MAX) {
    return false;
  }

  uint8_t register_address = reg;

  struct i2c_msg messages[2]{};

  messages[0].addr = address_;
  messages[0].flags = 0;
  messages[0].len = 1;
  messages[0].buf = &register_address;

  messages[1].addr = address_;
  messages[1].flags = I2C_M_RD;
  messages[1].len = static_cast<__u16>(length);
  messages[1].buf = buffer;

  return transfer(messages, 2);
}

bool Mpu6050Driver::read_register(uint8_t reg, uint8_t &value) {
  return read_bytes(reg, &value, 1);
}

bool Mpu6050Driver::write_register(uint8_t reg, uint8_t value) {
  if (i2c_fd_ < 0) {
    return false;
  }

  uint8_t buffer[2] = {reg, value};

  struct i2c_msg message{};

  message.addr = address_;
  message.flags = 0;
  message.len = 2;
  message.buf = buffer;

  return transfer(&message, 1);
}

int16_t Mpu6050Driver::make_int16(uint8_t high, uint8_t low) {
  const uint16_t value =
      (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);

  return static_cast<int16_t>(value);
}

bool Mpu6050Driver::init() {
  close_device();

  if (!open_device()) {
    return false;
  }

  uint8_t who_am_i = 0;

  if (!read_register(WHO_AM_I_REG, who_am_i)) {
    close_device();
    return false;
  }

  if (who_am_i != EXPECTED_WHO_AM_I) {
    close_device();
    return false;
  }

  if (!write_register(PWR_MGMT_1_REG, 0x01)) {
    close_device();
    return false;
  }

  if (!write_register(CONFIG_REG, 0x00)) {
    close_device();
    return false;
  }

  if (!write_register(GYRO_CONFIG_REG, 0x00)) {
    close_device();
    return false;
  }

  if (!write_register(ACCEL_CONFIG_REG, 0x00)) {
    close_device();
    return false;
  }

  uint8_t accel_config = 0;

  if (!read_register(ACCEL_CONFIG_REG, accel_config)) {
    close_device();
    return false;
  }

  if (accel_config != 0x00) {
    close_device();
    return false;
  }

  consecutive_failures_ = 0;

  std::cout << "[INFO] IMU initialized on " << device_path_ << " address 0x"
            << std::hex << static_cast<int>(address_) << std::dec << '\n';

  return true;
}

std::optional<ImuSample> Mpu6050Driver::read() {
  if (i2c_fd_ < 0) {
    return std::nullopt;
  }

  uint8_t data[14]{};

  if (!read_bytes(ACCEL_XOUT_H_REG, data, sizeof(data))) {
    ++consecutive_failures_;
    if (consecutive_failures_ >= MAX_CONSECUTIVE_FAILURES) {
      close_device();
    }

    return std::nullopt;
  }

  consecutive_failures_ = 0;

  const int16_t raw_ax = make_int16(data[0], data[1]);
  const int16_t raw_ay = make_int16(data[2], data[3]);
  const int16_t raw_az = make_int16(data[4], data[5]);
  const int16_t raw_gx = make_int16(data[8], data[9]);
  const int16_t raw_gy = make_int16(data[10], data[11]);
  const int16_t raw_gz = make_int16(data[12], data[13]);

  ImuSample sample{};

  sample.accel_x = raw_ax * ACCEL_SCALE;
  sample.accel_y = -raw_ay * ACCEL_SCALE;
  sample.accel_z = raw_az * ACCEL_SCALE;

  sample.gyro_x = -raw_gx * GYRO_SCALE;
  sample.gyro_y = raw_gy * GYRO_SCALE;
  sample.gyro_z = raw_gz * GYRO_SCALE;

  return sample;
}

bool Mpu6050Driver::is_healthy() const {
  return i2c_fd_ >= 0 && consecutive_failures_ < MAX_CONSECUTIVE_FAILURES;
}