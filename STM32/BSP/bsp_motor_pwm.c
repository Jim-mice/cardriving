#include "bsp_motor_pwm.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#define MOTOR_PWM_FREQUENCY_HZ 20000UL

static uint16_t g_motorPwmPeriod;

void motor_pwm_init(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef timeBase;
    TIM_OCInitTypeDef outputCompare;
    RCC_ClocksTypeDef clocks;
    uint32_t timerClock;
    uint32_t period;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    RCC_GetClocksFreq(&clocks);
    timerClock = clocks.PCLK1_Frequency;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) {
        timerClock *= 2U;
    }

    period = timerClock / MOTOR_PWM_FREQUENCY_HZ;
    if (period == 0U) {
        period = 1U;
    }
    if (period > 0x10000UL) {
        period = 0x10000UL;
    }
    g_motorPwmPeriod = (uint16_t)period;

    TIM_TimeBaseStructInit(&timeBase);
    timeBase.TIM_Prescaler = 0;
    timeBase.TIM_CounterMode = TIM_CounterMode_Up;
    timeBase.TIM_Period = g_motorPwmPeriod - 1U;
    timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &timeBase);

    TIM_OCStructInit(&outputCompare);
    outputCompare.TIM_OCMode = TIM_OCMode_PWM1;
    outputCompare.TIM_OutputState = TIM_OutputState_Enable;
    outputCompare.TIM_Pulse = 0;
    outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &outputCompare);
    TIM_OC2Init(TIM4, &outputCompare);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_SetCompare1(TIM4, 0);
    TIM_SetCompare2(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

void motor_pwm_set_left_compare(uint16_t compare)
{
    if (compare > g_motorPwmPeriod) {
        compare = g_motorPwmPeriod;
    }
    TIM_SetCompare2(TIM4, compare);
}

void motor_pwm_set_right_compare(uint16_t compare)
{
    if (compare > g_motorPwmPeriod) {
        compare = g_motorPwmPeriod;
    }
    TIM_SetCompare1(TIM4, compare);
}

uint16_t motor_pwm_get_period(void)
{
    return g_motorPwmPeriod;
}
