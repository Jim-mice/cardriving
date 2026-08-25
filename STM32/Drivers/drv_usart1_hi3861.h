#ifndef __DRV_USART1_H
#define __DRV_USART1_H

#include "stm32f10x.h"


void USART1_Hi3861_Init(void);


void USART1_SendByte(uint8_t data);


void USART1_SendString(char *str);

/* Hi3861 <-> STM32 binary UART frame: FC b1 b2 b3 b4 FD. */
#define USART1_HI3861_FRAME_HEAD 0xFC
#define USART1_HI3861_FRAME_TAIL 0xFD
#define USART1_HI3861_FRAME_LEN  6

/* Copies one complete frame received by the RX interrupt. Returns 1 on success. */
uint8_t USART1_GetReceivedFrame(uint8_t *frame);



extern volatile uint8_t USART1_RX_Buffer[64];

extern volatile uint8_t USART1_RX_Flag;


#endif
