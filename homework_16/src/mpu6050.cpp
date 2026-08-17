#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <chrono>

#include "mpu6050.hpp"


namespace {
    constexpr uint8_t MPU6050_DEVICE_ID = 0x68;
    constexpr uint8_t MPU6050_DEVICE_ID_2 = 0x72;
    constexpr uint8_t MPU6050_WHO_AM_I = 0x75;

    constexpr uint8_t MPU6050_ACCEL_X_H = 0x3B;
    constexpr uint8_t MPU6050_ACCEL_X_L = 0x3C;
    constexpr uint8_t MPU6050_ACCEL_Y_H = 0x3D;
    constexpr uint8_t MPU6050_ACCEL_Y_L = 0x3E;
    constexpr uint8_t MPU6050_ACCEL_Z_H = 0x3F;
    constexpr uint8_t MPU6050_ACCEL_Z_L = 0x40;

    constexpr uint8_t MPU6050_GYRO_X_H = 0x43;
    constexpr uint8_t MPU6050_GYRO_X_L = 0x44;
    constexpr uint8_t MPU6050_GYRO_Y_H = 0x45;
    constexpr uint8_t MPU6050_GYRO_Y_L = 0x46;
    constexpr uint8_t MPU6050_GYRO_Z_H = 0x47;
    constexpr uint8_t MPU6050_GYRO_Z_L = 0x48;

    constexpr uint8_t MPU6050_TEMP_H = 0x41;
    constexpr uint8_t MPU6050_TEMP_L = 0x42;

    constexpr uint8_t MPU6050_PWR_MGMT_1 = 0x6B;

    double convert_raw_accel_to_g(int16_t raw) {
        return raw / 16384.0;
    }

    double convert_raw_gyro_to_dps(int16_t raw) {
        return raw / 131.0;
    }

    double convert_raw_temp_to_celsius(int16_t raw) {
        return (raw / 340.0) + 36.53;
    }

    std::string uint8_to_hex(uint8_t val) {
        static constexpr char hex_chars[] = "0123456789abcdef";
        return { '0', 'x', hex_chars[val >> 4], hex_chars[val & 0x0F] };
    }

    int16_t join_bytes(uint8_t high, uint8_t low) {
        const uint16_t value = (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
        return static_cast<int16_t>(value);
    }
}

MPU6050::MPU6050(const char* i2c_device, int i2c_address) {
    i2c_fd_ = open(i2c_device, O_RDWR);
    if (i2c_fd_ < 0) {
        throw std::runtime_error("Failed to open I2C device " + std::string(i2c_device));
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address) < 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
        throw std::runtime_error("Failed to set I2C address " + std::to_string(i2c_address));
    }

    const auto id = read_register(MPU6050_WHO_AM_I);
    if (!id) {
        close(i2c_fd_);
        i2c_fd_ = -1;
        throw std::runtime_error("Failed to read WHO_AM_I register");
    }

    if (*id != MPU6050_DEVICE_ID && *id != MPU6050_DEVICE_ID_2) {
        close(i2c_fd_);
        i2c_fd_ = -1;
        throw std::runtime_error("Invalid device ID " + uint8_to_hex(*id));
    }

    uint8_t data[] = {MPU6050_PWR_MGMT_1, 0x00};
    if (write(i2c_fd_, data, sizeof(data)) != sizeof(data)) {
        close(i2c_fd_);
        i2c_fd_ = -1;
        throw std::runtime_error("Failed to wake up MPU6050");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

MPU6050::~MPU6050() {
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
    }
}

std::optional<uint8_t> MPU6050::read_register(uint8_t reg) const {
    if (i2c_fd_ < 0) {
        return std::nullopt;
    }
    if (write(i2c_fd_, &reg, 1) != 1) {
        return std::nullopt;
    }

    uint8_t raw;
    if (read(i2c_fd_, &raw, 1) != 1) {
        return std::nullopt;
    }

    return raw;
}

std::optional<int16_t> MPU6050::read_word(uint8_t reg_high, uint8_t reg_low) const {
    auto raw_high = read_register(reg_high);
    auto raw_low = read_register(reg_low);
    if (!raw_high || !raw_low) {
        return std::nullopt;
    }
    return join_bytes(*raw_high, *raw_low);
}

std::optional<double> MPU6050::get_accel(uint8_t reg_high, uint8_t reg_low) const {
    auto raw = read_word(reg_high, reg_low);
    if (!raw) {
        return std::nullopt;
    }
    return convert_raw_accel_to_g(*raw);
}

std::optional<double> MPU6050::get_gyro(uint8_t reg_high, uint8_t reg_low) const {
    auto raw = read_word(reg_high, reg_low);
    if (!raw) {
        return std::nullopt;
    }
    return convert_raw_gyro_to_dps(*raw);
}

std::optional<double> MPU6050::get_temperature() const {
    auto raw = read_word(MPU6050_TEMP_H, MPU6050_TEMP_L);
    if (!raw) {
        return std::nullopt;
    }
    return convert_raw_temp_to_celsius(*raw);
}

std::optional<double> MPU6050::get_accelX() const {
    return get_accel(MPU6050_ACCEL_X_H, MPU6050_ACCEL_X_L);
}

std::optional<double> MPU6050::get_accelY() const {
    return get_accel(MPU6050_ACCEL_Y_H, MPU6050_ACCEL_Y_L);
}

std::optional<double> MPU6050::get_accelZ() const {
    return get_accel(MPU6050_ACCEL_Z_H, MPU6050_ACCEL_Z_L);
}

std::optional<double> MPU6050::get_gyroX() const {
    return get_gyro(MPU6050_GYRO_X_H, MPU6050_GYRO_X_L);
}

std::optional<double> MPU6050::get_gyroY() const {
    return get_gyro(MPU6050_GYRO_Y_H, MPU6050_GYRO_Y_L);
}
std::optional<double> MPU6050::get_gyroZ() const {
    return get_gyro(MPU6050_GYRO_Z_H, MPU6050_GYRO_Z_L);
}
