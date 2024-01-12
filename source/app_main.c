/// \file app_main.c
/// \author Иван Краев
/// \date 26.12.2023

#include <stdbool.h>
#include <app_main.h>
#include <motor.h>

static motor motor_1 = {0};
motor_gpio motor_1_step = {
        .GPIOx = GPIOA,
        .GPIO_Pin = GPIO_PIN_10
};
motor_gpio motor_1_dir = {
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_PIN_3
};
static motor motor_2 = {0};
motor_gpio motor_2_step = {
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_PIN_5
};
motor_gpio motor_2_dir = {
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_PIN_4
};
static motor motor_3 = {0};
motor_gpio motor_3_step = {
        .GPIOx = GPIOB,
        .GPIO_Pin = GPIO_PIN_10
};
motor_gpio motor_3_dir = {
        .GPIOx = GPIOA,
        .GPIO_Pin = GPIO_PIN_8
};
static motor motor_4 = {0};
motor_gpio motor_4_step = {
        .GPIOx = GPIOA,
        .GPIO_Pin = GPIO_PIN_9
};
motor_gpio motor_4_dir = {
        .GPIOx = GPIOC,
        .GPIO_Pin = GPIO_PIN_7
};

extern UART_HandleTypeDef huart2;
/// \brief Буффер для даных, полученых с USB-UART
uint8_t uart_data_buffer[8] = {0};
const char* rx_confirm = "recieved.\n";


///// \brief Функция - callback по приёму сообщения по UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        // Шлём подтверждение

        HAL_UART_Transmit_IT(&huart2, (uint8_t*)rx_confirm ,sizeof(rx_confirm));
        // USART2 завершил прием данных, снова начинаем приём
        HAL_UART_Receive_IT(&huart2, uart_data_buffer, sizeof(uart_data_buffer));
    }
}

/// \brief Основная функция рабочей программы,содержит в себе бесконечный цикл
void app_main(){

    if( motor_init(motor_1_dir, motor_1_step, 0, true, 0, 0, &motor_1)) {
//        Error_Handler();
    }
    if( motor_init(motor_2_dir, motor_2_step, 0, true, 5000, 300,  &motor_2)) {
//        Error_Handler();
    }
    if( motor_init(motor_3_dir, motor_3_step, 0, true, 5000, 300,  &motor_3)) {
//        Error_Handler();
    }
    if( motor_init(motor_4_dir, motor_4_step, 0, true, 5000, 300,  &motor_4)) {
//        Error_Handler();
    }

//    /// \brief Первично запускаем приём по прерыванию
    HAL_UART_Receive_IT(&huart2, uart_data_buffer, sizeof(uart_data_buffer));

    while (1) {
        move_motor(&motor_1, uart_data_buffer[0], uart_data_buffer[4]);// Слева скорость (0-255), справа направление (0-1)
        move_motor(&motor_2, uart_data_buffer[1], uart_data_buffer[5]);// Слева скорость (0-255), справа направление (0-1)
        move_motor(&motor_3, uart_data_buffer[2], uart_data_buffer[6]);// Слева скорость (0-255), справа направление (0-1)
        move_motor(&motor_4, uart_data_buffer[3], uart_data_buffer[7]);// Слева скорость (0-255), справа направление (0-1)


    }
}