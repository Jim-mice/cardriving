#include "bsp_encoder.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#define ENCODER_INPUT_FILTER 6U

static uint16_t g_leftPrevious;
static uint16_t g_rightPrevious;

static void EncoderTimerInit(TIM_TypeDef *timer)
{
    TIM_TimeBaseInitTypeDef timeBase;
    TIM_ICInitTypeDef inputCapture;

    TIM_TimeBaseStructInit(&timeBase);
    timeBase.TIM_Prescaler = 0U;
    timeBase.TIM_CounterMode = TIM_CounterMode_Up;
    timeBase.TIM_Period = 0xFFFFU;
    timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(timer, &timeBase);

    TIM_EncoderInterfaceConfig(timer, TIM_EncoderMode_TI12,
        TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICStructInit(&inputCapture);
    inputCapture.TIM_ICPolarity = TIM_ICPolarity_Rising;
    inputCapture.TIM_ICSelection = TIM_ICSelection_DirectTI;
    inputCapture.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    inputCapture.TIM_ICFilter = ENCODER_INPUT_FILTER;
    inputCapture.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(timer, &inputCapture);
    inputCapture.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(timer, &inputCapture);

    TIM_SetCounter(timer, 0U);
    TIM_Cmd(timer, ENABLE);
}

void encoder_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    EncoderTimerInit(TIM2);
    EncoderTimerInit(TIM3);

    g_leftPrevious = 0U;
    g_rightPrevious = 0U;
}

int16_t encoder_left_get_delta(void)
{
    uint16_t current = (uint16_t)TIM_GetCounter(TIM2);
    int16_t delta = (int16_t)(uint16_t)(current - g_leftPrevious);

    g_leftPrevious = current;
    return delta;
}

int16_t encoder_right_get_delta(void)
{
    uint16_t current = (uint16_t)TIM_GetCounter(TIM3);
    int16_t delta = (int16_t)(uint16_t)(current - g_rightPrevious);

    g_rightPrevious = current;
    return delta;
}
