#pragma once
#include <cstdint>
#include <optional>

class MPU6050 {
    private:
    int i2c_fd_ = -1;

    std::optional<uint8_t> read_register(uint8_t reg) const;
    std::optional<int16_t> read_word(uint8_t reg_high, uint8_t reg_low) const;

    std::optional<double> get_accel(uint8_t reg_high, uint8_t reg_low) const;
    std::optional<double> get_gyro(uint8_t reg_high, uint8_t reg_low) const;

public:
    MPU6050(const char* i2c_device, int i2c_address);
    ~MPU6050();

    std::optional<double> get_accelX() const;
    std::optional<double> get_accelY() const;
    std::optional<double> get_accelZ() const;

    std::optional<double> get_gyroX() const;
    std::optional<double> get_gyroY() const;
    std::optional<double> get_gyroZ() const;

    std::optional<double> get_temperature() const;
};