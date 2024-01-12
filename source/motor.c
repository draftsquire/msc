/// \file motor.c
/// \author Иван Краев
/// \date 26.12.2023


#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "motor.h"

//static struct {
//    void gpio_write_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
//} motor_dependencies;

/// \brief - Функция инициализации мотора (ШД)
///
/// \param[in] dir_pin - Пин направления
/// \param[in] step_pin - Пин управления шагом
/// \param[in] init_speed - Начальная скорость [0-255]
/// \param[in] init_dir - Начальное направление [true - вперёдб false -назад]
/// \param[in] min_t -
/// \param[in] max_t -
/// \param[out] motor
/// \return
int32_t motor_init(motor_gpio dir_pin, motor_gpio step_pin, uint8_t init_speed, bool init_dir, int32_t min_t, int32_t max_t, motor* motor) {
    if ((dir_pin.GPIOx == NULL) || (step_pin.GPIOx == NULL) || motor == NULL) {
        return -1;
    }
    motor->step_pin = step_pin;
    motor->direction_pin = dir_pin;
    HAL_GPIO_WritePin(step_pin.GPIOx, step_pin.GPIO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dir_pin.GPIOx, dir_pin.GPIO_Pin, GPIO_PIN_SET);
    motor->speed = init_speed;
    motor->direction = init_dir;
    motor->max_t = max_t;
    motor->min_t = min_t;
    return 0;
}

static uint32_t speed_map(uint8_t speed, int32_t max_t, int32_t min_t) {
    uint32_t delayTime = round((255. - speed) * ((min_t - max_t)/ 255.) + max_t);
    return delayTime;
}

/// \brief Функция для запуска двигателя с определённой скоростью и направлением
///
/// \param[in] motor - Структура двигателя
/// \param[in] speed - Скорость [0-255]
/// \param[in] direction - Направление (1 - Вперёд, 0 - Назад)
///
void move_motor(motor* motor, uint8_t speed, bool direction) {


    HAL_GPIO_WritePin(motor->direction_pin.GPIOx, motor->direction_pin.GPIO_Pin, (direction) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (speed!=0) {
        motor->delay_time = speed_map(speed, motor->max_t, motor->min_t);

        /// \warning Хз, как реализована функция micros() в Arduino, предположительно можно заменить на ниженаписанное.
        motor->current_time = HAL_GetTick() / 1000;
        if (motor->current_time - motor->previous_time >= motor->delay_time) {
            motor->control_signal = (motor->control_signal == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
            HAL_GPIO_WritePin(motor->step_pin.GPIOx, motor->step_pin.GPIO_Pin, motor->control_signal);
            motor->previous_time = motor->current_time;
        }
    } else {
        HAL_GPIO_WritePin(motor->step_pin.GPIOx, motor->step_pin.GPIO_Pin, GPIO_PIN_RESET);
    }
}


