#include "uart.h"
#include <string.h>
#include <math.h>

void UART_SendString(UART_HandleTypeDef *huart, const char *str) {
    HAL_UART_Transmit(
        huart,
        (uint8_t *)str,
        strlen(str),
        HAL_MAX_DELAY
    );
}

void IMU_SendData(
    const MPU6050_Data_t *imu,
    IMU_Frequency_t frequency,
    UART_HandleTypeDef *huart2,
    UART_HandleTypeDef *huart4) {
    if (imu == nullptr) {
        return;
    }

    char buffer[256];
    const char *mode;

    switch (frequency) {
        case IMU_FREQ_1_HZ:
            mode = "1Hz";
            break;

        case IMU_FREQ_10_HZ:
            mode = "10Hz";
            break;

        case IMU_FREQ_50_HZ:
            mode = "50Hz";
            break;

        case IMU_FREQ_100_HZ:
        default:
            mode = "100Hz";
            break;
    }

    const int32_t ax = static_cast<int32_t>(imu->accel_x_g * 1000.0f);
    const int32_t ay = static_cast<int32_t>(imu->accel_y_g * 1000.0f);
    const int32_t az = static_cast<int32_t>(imu->accel_z_g * 1000.0f);

    const int32_t gx = static_cast<int32_t>(imu->gyro_x_dps * 100.0f);
    const int32_t gy = static_cast<int32_t>(imu->gyro_y_dps * 100.0f);
    const int32_t gz = static_cast<int32_t>(imu->gyro_z_dps * 100.0f);

    const int32_t temp = static_cast<int32_t>(imu->temperature_c * 100.0f);

    const uint32_t timestamp = HAL_GetTick();

    snprintf(
        buffer,
        sizeof(buffer),
        "t=%lu ms "
        "ax=%s%ld.%03ld "
        "ay=%s%ld.%03ld "
        "az=%s%ld.%03ld "
        "gx=%s%ld.%02ld "
        "gy=%s%ld.%02ld "
        "gz=%s%ld.%02ld "
        "temp=%s%ld.%02ld "
        "mode=%s\r\n",

        timestamp,

        (ax < 0) ? "-" : "", labs(ax) / 1000, labs(ax) % 1000,
        (ay < 0) ? "-" : "", labs(ay) / 1000, labs(ay) % 1000,
        (az < 0) ? "-" : "", labs(az) / 1000, labs(az) % 1000,

        (gx < 0) ? "-" : "", labs(gx) / 100, labs(gx) % 100,
        (gy < 0) ? "-" : "", labs(gy) / 100, labs(gy) % 100,
        (gz < 0) ? "-" : "", labs(gz) / 100, labs(gz) % 100,

        (temp < 0) ? "-" : "", labs(temp) / 100, labs(temp) % 100,

        mode
    );

    UART_SendString(huart2, buffer);
    UART_SendString(huart4, buffer);
}