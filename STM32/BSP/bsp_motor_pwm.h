#ifndef __BSP_MOTOR_PWM_H
#define __BSP_MOTOR_PWM_H

#include "stm32f10x.h"

void motor_pwm_init(void);
void motor_pwm_set_left_compare(uint16_t compare);
void motor_pwm_set_right_compare(uint16_t compare);
uint16_t motor_pwm_get_period(void);

#endif
