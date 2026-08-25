#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stdio.h>
#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

void UART_SendString(UART_HandleTypeDef *huart, const char *str);
void IMU_SendData(const MPU6050_Data_t *data,
                 IMU_Frequency_t imu_frequency,
                 UART_HandleTypeDef *huart2,
                 UART_HandleTypeDef *huart4);

#ifdef __cplusplus
}
#endif

#endif // UART_H