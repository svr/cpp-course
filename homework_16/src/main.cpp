#include <iostream>
#include <thread>
#include <chrono>

#include "mpu6050.hpp"

std::string format_value(const std::optional<double>& value, const std::string& unit) {
    auto string_value = value ? std::to_string(*value) : "--";
    return string_value + unit;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <i2c_device> <i2c_address>" << std::endl;
        return 1;
    }

    const char* i2c_device = argv[1];
    int i2c_address = std::stoi(argv[2], nullptr, 16);
    try {
        MPU6050 mpu(i2c_device, i2c_address);
        while(true) {
            std::optional<double> accelX = mpu.get_accelX();
            std::optional<double> accelY = mpu.get_accelY();
            std::optional<double> accelZ = mpu.get_accelZ();
            std::optional<double> gyroX = mpu.get_gyroX();
            std::optional<double> gyroY = mpu.get_gyroY();
            std::optional<double> gyroZ = mpu.get_gyroZ();
            std::optional<double> temperature = mpu.get_temperature();

            std::cout << "AccelX: " << format_value(accelX, "g") << " ";
            std::cout << "AccelY: " << format_value(accelY, "g") << " ";
            std::cout << "AccelZ: " << format_value(accelZ, "g") << "\n";

            std::cout << "GyroX: " << format_value(gyroX, "°/s") << " ";
            std::cout << "GyroY: " << format_value(gyroY, "°/s") << " ";
            std::cout << "GyroZ: " << format_value(gyroZ, "°/s") << "\n";

            std::cout << "Temperature: " << format_value(temperature, "°C") << "\n\n";

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    } catch (const std::runtime_error& e) {
        std::cerr << "Error initializing MPU6050: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Unexpected error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
