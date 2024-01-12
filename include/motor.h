//
// Created by drafty on 26.12.2023.
//



#ifndef MECANUM_STEPPER_CONTROL_MOTOR_H
#define MECANUM_STEPPER_CONTROL_MOTOR_H

///TODO: Все зависимости от HAL прокидывать в виде инъекций!
#include <stm32f4xx_hal.h>
#include <stdbool.h>


/// \struct motor_gpio
/// \struct Структура для объединения структуры порта и номера пина
typedef struct {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
} motor_gpio;

/// \struct motor
/// \brief Структура описывающая необходимые для управления мотором параметры
typedef struct {
    motor_gpio step_pin;
    motor_gpio direction_pin;
    uint8_t speed;
    bool direction;
    bool control_signal;
    uint32_t previous_time;
    uint32_t current_time;
    uint32_t delay_time;
} motor;

/// \brief - Функция инициализации мотора (ШД)
///
/// \param[in] dir_pin - Пин направления
/// \param[in] step_pin - Пин управления шагом
/// \param[in] init_speed - Начальная скорость [0-255]
/// \param[in] init_dir - Начальное направление [true - вперёдб false -назад]
/// \param[out] motor
/// \return
int32_t motor_init(motor_gpio dir_pin, motor_gpio step_pin, uint8_t init_speed, bool init_dir, motor* motor);

/// \brief Функция для запуска двигателя с определённой скоростью и направлением
///
/// \param[in] motor - Структура двигателя
/// \param[in] speed - Скорость [0-255]
/// \param[in] direction - Направление (1 - Вперёд, 0 - Назад)
///
void move_motor(motor* motor, uint8_t speed, bool direction);
#endif //MECANUM_STEPPER_CONTROL_MOTOR_H
