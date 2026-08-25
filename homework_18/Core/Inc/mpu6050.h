#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include <stdint.h>


typedef enum {
    IMU_FREQ_1_HZ,
    IMU_FREQ_10_HZ,
    IMU_FREQ_50_HZ,
    IMU_FREQ_100_HZ
} IMU_Frequency_t;

/*
 * MPU6050 I2C address.
 *
 * AD0 = LOW  -> 0x68
 * AD0 = HIGH -> 0x69
 */
#define MPU6050_ADDR        (0x68 << 1)


#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75

#define MPU6050_WHO_AM_I_VALUE      0x68
#define MPU6050_WHO_AM_I_VALUE_2    0x72


#define MPU6050_ACCEL_SCALE         16384.0f
#define MPU6050_GYRO_SCALE          131.0f

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float temperature_c;

} MPU6050_Data_t;

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read(
    I2C_HandleTypeDef *hi2c,
    MPU6050_Data_t *data
);

HAL_StatusTypeDef MPU6050_ReadRaw(
    I2C_HandleTypeDef *hi2c,
    int16_t *accel_x,
    int16_t *accel_y,
    int16_t *accel_z,
    int16_t *gyro_x,
    int16_t *gyro_y,
    int16_t *gyro_z
);
#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */