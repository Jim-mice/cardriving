#include "bsp_motor_l9110.h"
#include "bsp_motor_pwm.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define MOTOR_SPEED_MAX 1000

#define LEFT_DIRECTION_PORT GPIOB
#define LEFT_DIRECTION_PIN GPIO_Pin_14
#define RIGHT_DIRECTION_PORT GPIOB
#define RIGHT_DIRECTION_PIN GPIO_Pin_13

static uint16_t motor_speed_to_compare(int speed)
{
    uint16_t period;
    uint16_t magnitude;

    if (speed > MOTOR_SPEED_MAX) {
        speed = MOTOR_SPEED_MAX;
    } else if (speed < -MOTOR_SPEED_MAX) {
        speed = -MOTOR_SPEED_MAX;
    }

    magnitude = (uint16_t)((speed < 0) ? -speed : speed);
    period = motor_pwm_get_period();
    return (uint16_t)(((uint32_t)magnitude * period) / MOTOR_SPEED_MAX);
}

void motor_stop(void)
{
    motor_pwm_set_left_compare(0);
    motor_pwm_set_right_compare(0);
    GPIO_ResetBits(LEFT_DIRECTION_PORT, LEFT_DIRECTION_PIN);
    GPIO_ResetBits(RIGHT_DIRECTION_PORT, RIGHT_DIRECTION_PIN);
}

void motor_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = LEFT_DIRECTION_PIN | RIGHT_DIRECTION_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    GPIO_ResetBits(LEFT_DIRECTION_PORT, LEFT_DIRECTION_PIN);
    GPIO_ResetBits(RIGHT_DIRECTION_PORT, RIGHT_DIRECTION_PIN);
    motor_pwm_init();
    motor_stop();
}

void motor_left_set(int speed)
{
    uint16_t compare;
    uint16_t period;

    if (speed == 0) {
        motor_pwm_set_left_compare(0);
        GPIO_ResetBits(LEFT_DIRECTION_PORT, LEFT_DIRECTION_PIN);
        return;
    }

    compare = motor_speed_to_compare(speed);
    if (speed > 0) {
        GPIO_ResetBits(LEFT_DIRECTION_PORT, LEFT_DIRECTION_PIN);
        motor_pwm_set_left_compare(compare);
    } else {
        period = motor_pwm_get_period();
        GPIO_SetBits(LEFT_DIRECTION_PORT, LEFT_DIRECTION_PIN);
        motor_pwm_set_left_compare((uint16_t)(period - compare));
    }
}

void motor_right_set(int speed)
{
    uint16_t compare;
    uint16_t period;

    if (speed == 0) {
        motor_pwm_set_right_compare(0);
        GPIO_ResetBits(RIGHT_DIRECTION_PORT, RIGHT_DIRECTION_PIN);
        return;
    }

    compare = motor_speed_to_compare(speed);
    if (speed > 0) {
        GPIO_ResetBits(RIGHT_DIRECTION_PORT, RIGHT_DIRECTION_PIN);
        motor_pwm_set_right_compare(compare);
    } else {
        period = motor_pwm_get_period();
        GPIO_SetBits(RIGHT_DIRECTION_PORT, RIGHT_DIRECTION_PIN);
        motor_pwm_set_right_compare((uint16_t)(period - compare));
    }
}
