#include "mpu6050.h"


HAL_StatusTypeDef MPU6050_WriteRegister(
    I2C_HandleTypeDef *hi2c,
    uint8_t reg,
    uint8_t value) {
    return HAL_I2C_Mem_Write(
        hi2c,
        MPU6050_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        HAL_MAX_DELAY
    );
}

HAL_StatusTypeDef MPU6050_ReadRegisters(
    I2C_HandleTypeDef *hi2c,
    uint8_t reg,
    uint8_t *data,
    uint16_t size) {
    return HAL_I2C_Mem_Read(
        hi2c,
        MPU6050_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        size,
        HAL_MAX_DELAY
    );
}


HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i;

    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = MPU6050_ReadRegisters(
        hi2c,
        MPU6050_REG_WHO_AM_I,
        &who_am_i,
        1
    );

    if (status != HAL_OK)
    {
        return status;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VALUE &&
        who_am_i != MPU6050_WHO_AM_I_VALUE_2)
    {
        return HAL_ERROR;
    }

    status = MPU6050_WriteRegister(
        hi2c,
        MPU6050_REG_PWR_MGMT_1,
        0x00
    );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(100);

    status = MPU6050_WriteRegister(
        hi2c,
        MPU6050_REG_SMPLRT_DIV,
        9
    );

    if (status != HAL_OK)
    {
        return status;
    }

    status = MPU6050_WriteRegister(
        hi2c,
        MPU6050_REG_CONFIG,
        0x03
    );

    if (status != HAL_OK)
    {
        return status;
    }

    status = MPU6050_WriteRegister(
        hi2c,
        MPU6050_REG_GYRO_CONFIG,
        0x00
    );

    if (status != HAL_OK)
    {
        return status;
    }

    status = MPU6050_WriteRegister(
        hi2c,
        MPU6050_REG_ACCEL_CONFIG,
        0x00
    );

    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadRaw(
    I2C_HandleTypeDef *hi2c,
    int16_t *accel_x,
    int16_t *accel_y,
    int16_t *accel_z,
    int16_t *gyro_x,
    int16_t *gyro_y,
    int16_t *gyro_z)
{
    uint8_t buffer[14];

    HAL_StatusTypeDef status = MPU6050_ReadRegisters(
            hi2c,
            MPU6050_REG_ACCEL_XOUT_H,
            buffer,
            sizeof(buffer)
        );

    if (status != HAL_OK) {
        return status;
    }

    *accel_x = (int16_t)(((uint16_t)buffer[0] << 8) |buffer[1]);

    *accel_y = (int16_t)(
        ((uint16_t)buffer[2] << 8) |
        buffer[3]
    );

    *accel_z = (int16_t)(
        ((uint16_t)buffer[4] << 8) |
        buffer[5]
    );

    *gyro_x = (int16_t)(
        ((uint16_t)buffer[8] << 8) |
        buffer[9]
    );

    *gyro_y = (int16_t)(
        ((uint16_t)buffer[10] << 8) |
        buffer[11]
    );

    *gyro_z = (int16_t)(
        ((uint16_t)buffer[12] << 8) |
        buffer[13]
    );

    return HAL_OK;
}


HAL_StatusTypeDef MPU6050_Read(
    I2C_HandleTypeDef *hi2c,
    MPU6050_Data_t *data) {
    uint8_t buffer[14];

    if (data == NULL) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status =
        MPU6050_ReadRegisters(
            hi2c,
            MPU6050_REG_ACCEL_XOUT_H,
            buffer,
            sizeof(buffer)
        );

    if (status != HAL_OK) {
        return status;
    }


    int16_t accel_x = (int16_t)(
        ((uint16_t)buffer[0] << 8) |
        buffer[1]
    );

    int16_t accel_y = (int16_t)(
        ((uint16_t)buffer[2] << 8) |
        buffer[3]
    );

    int16_t accel_z = (int16_t)(
        ((uint16_t)buffer[4] << 8) |
        buffer[5]
    );


    int16_t temperature_raw = (int16_t)(
        ((uint16_t)buffer[6] << 8) |
        buffer[7]
    );


    int16_t gyro_x = (int16_t)(
        ((uint16_t)buffer[8] << 8) |
        buffer[9]
    );

    int16_t gyro_y = (int16_t)(
        ((uint16_t)buffer[10] << 8) |
        buffer[11]
    );

    int16_t gyro_z = (int16_t)(
        ((uint16_t)buffer[12] << 8) |
        buffer[13]
    );

    data->accel_x_g = (float)accel_x / MPU6050_ACCEL_SCALE;
    data->accel_y_g = (float)accel_y / MPU6050_ACCEL_SCALE;
    data->accel_z_g = (float)accel_z / MPU6050_ACCEL_SCALE;

    data->gyro_x_dps = (float)gyro_x / MPU6050_GYRO_SCALE;
    data->gyro_y_dps = (float)gyro_y / MPU6050_GYRO_SCALE;
    data->gyro_z_dps = (float)gyro_z / MPU6050_GYRO_SCALE;

    data->temperature_c = ((float)temperature_raw / 340.0f) + 36.53f;

    return HAL_OK;
}