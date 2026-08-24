#ifndef __STM32F10X_CONF_H
#define __STM32F10X_CONF_H


/*
 * STM32F10x Standard Peripheral Library configuration file
 *
 * 当前阶段只建立工程框架
 * 后续加入 SPL 后再打开对应模块
 */


// #include "stm32f10x_gpio.h"
// #include "stm32f10x_rcc.h"
// #include "stm32f10x_usart.h"
// #include "stm32f10x_tim.h"
// #include "misc.h"


#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line);

#endif


#endif
