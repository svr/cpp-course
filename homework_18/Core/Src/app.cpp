#include <cstring>
#include "stm32f4xx_hal.h"
#include "app.h"
#include "mpu6050.h"
#include "uart.h"

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;

volatile bool imu_read_request = false;
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        imu_read_request = true;
    }
}

volatile bool user_button_pressed = false;
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint32_t last_press_time = 0;
    if (GPIO_Pin == B1_Pin) {

        uint32_t now = HAL_GetTick();

        if ((now - last_press_time) >= 50U) {
            last_press_time = now;
            user_button_pressed = true;
        }
    }
}

static uint8_t uart_rx_byte;
constexpr size_t UART_COMMAND_BUFFER_SIZE = 32;
static char uart_command[UART_COMMAND_BUFFER_SIZE];
static volatile uint32_t uart_command_index = 0;
static volatile bool uart_command_ready = false;
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART2) {
        // Echo received character
        HAL_UART_Transmit(
            &huart2,
            &uart_rx_byte,
            1,
            10
        );
        if (uart_rx_byte == '\r' || uart_rx_byte == '\n') {
            if (uart_command_index > 0 && uart_command_index < sizeof(uart_command)) {
                uart_command[uart_command_index] = '\0';
                uart_command_ready = true;
            } else {
                uart_command_index = 0;
            }
        } else {
            if (uart_command_index < sizeof(uart_command) - 1) {
                uart_command[uart_command_index++] = (char)uart_rx_byte;
            } else {
                uart_command_index = 0;
            }
        }
    }
    HAL_UART_Receive_IT(
        &huart2,
        &uart_rx_byte,
        1
    );
}

volatile IMU_Frequency_t imu_frequency = IMU_FREQ_1_HZ;
constexpr uint32_t TIM2_CLK_FREQ = 90'000'000; 
constexpr uint32_t TIM2_PRESCALER = 71; 
constexpr uint32_t TIM2_COUNTER_FREQUENCY = TIM2_CLK_FREQ / (TIM2_PRESCALER + 1);

static void IMU_SetFrequency(IMU_Frequency_t frequency) {
    uint32_t frequency_hz;

    switch (frequency)
    {
        case IMU_FREQ_1_HZ:
            frequency_hz = 1U;
            break;

        case IMU_FREQ_10_HZ:
            frequency_hz = 10U;
            break;

        case IMU_FREQ_50_HZ:
            frequency_hz = 50U;
            break;

        case IMU_FREQ_100_HZ:
        default:
            frequency_hz = 100U;
            break;
    }

    const uint32_t period = TIM2_COUNTER_FREQUENCY / frequency_hz - 1U;

    HAL_TIM_Base_Stop_IT(&htim2);

    __HAL_TIM_SET_AUTORELOAD(&htim2, period);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

    HAL_TIM_Base_Start_IT(&htim2);
}

static void IMU_NextFrequency(void) {
    switch (imu_frequency)
    {
        case IMU_FREQ_1_HZ:
            imu_frequency = IMU_FREQ_10_HZ;
            break;
        case IMU_FREQ_10_HZ:
            imu_frequency = IMU_FREQ_50_HZ;
            break;

        case IMU_FREQ_50_HZ:
            imu_frequency = IMU_FREQ_100_HZ;
            break;

        case IMU_FREQ_100_HZ:
        default:
            imu_frequency = IMU_FREQ_1_HZ;
            break;
    }

    IMU_SetFrequency(imu_frequency);
}

static void ProcessUartCommand(void) {
    char command[UART_COMMAND_BUFFER_SIZE];

    __disable_irq();
    strncpy(
        command,
        uart_command,
        sizeof(command)
    );
    __enable_irq();

    uart_command_ready = false;
    uart_command_index = 0;
    command[sizeof(command) - 1] = '\0';

    if (strncmp(command, "F1", UART_COMMAND_BUFFER_SIZE) == 0) {
        imu_frequency = IMU_FREQ_1_HZ;
        IMU_SetFrequency(imu_frequency);

        UART_SendString(
            &huart2,
            "OK: frequency=1Hz\r\n"
        );
    } else if (strncmp(command, "F10", UART_COMMAND_BUFFER_SIZE) == 0) {
        imu_frequency = IMU_FREQ_10_HZ;
        IMU_SetFrequency(imu_frequency);

        UART_SendString(
            &huart2,
            "OK: frequency=10Hz\r\n"
        );
    } else if (strncmp(command, "F50", UART_COMMAND_BUFFER_SIZE) == 0) {
        imu_frequency = IMU_FREQ_50_HZ;
        IMU_SetFrequency(imu_frequency);

        UART_SendString(
            &huart2,
            "OK: frequency=50Hz\r\n"
        );
    } else if (strncmp(command, "F100", UART_COMMAND_BUFFER_SIZE) == 0){
        imu_frequency = IMU_FREQ_100_HZ;
        IMU_SetFrequency(imu_frequency);

        UART_SendString(
            &huart2,
            "OK: frequency=100Hz\r\n"
        );
    } else if (strncmp(command, "STAT", UART_COMMAND_BUFFER_SIZE) == 0) {
        UART_SendString(
            &huart2,
            "OK: status\r\n"
        );
    } else {
        UART_SendString(
            &huart2,
            "ERROR: unknown command\r\n Supported commands: F1, F10, F50, F100, STAT\r\n"
        );
    }
}

void setup() {
    if (MPU6050_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
        Error_Handler();
    }
    HAL_UART_Receive_IT(
        &huart2,
        &uart_rx_byte,
        1
    );
    IMU_SetFrequency(imu_frequency);
}

extern "C" void app_start(void) {
    MPU6050_Data_t imu;

    setup();
    
    while (1) {
        if(uart_command_ready) {
            ProcessUartCommand();
        }

        if (user_button_pressed) {
            user_button_pressed = false;

            IMU_NextFrequency();

            UART_SendString(
                &huart2,
                "IMU frequency changed\r\n"
            );
        }

        if (imu_read_request) {
            imu_read_request = false;

            if (MPU6050_Read(&hi2c1, &imu) == HAL_OK) {
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
                IMU_SendData(&imu, imu_frequency, &huart2, &huart4);
            } else {
                UART_SendString(
                    &huart2,
                    "ERROR: MPU6050 I2C read failed\r\n"
                );
            }
        }
        __WFI();
    }
}
