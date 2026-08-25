#ifndef __DRV_USART1_H
#define __DRV_USART1_H

#include "stm32f10x.h"

void USART1_Hi3861_Init(void);

void USART1_SendByte(uint8_t data);

void USART1_SendString(char *str);

#endif