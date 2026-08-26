#ifndef __DRV_USART2_NFC_H
#define __DRV_USART2_NFC_H

#include "stm32f10x.h"

#define USART2_NFC_RX_DEBUG_CAPACITY 128U

void USART2_NFC_Init(void);
void USART2_NFC_SendByte(uint8_t data);
void USART2_NFC_SendBuffer(const uint8_t *data, uint16_t length);
void USART2_NFC_ClearRx(void);
uint8_t USART2_NFC_ReadByte(uint8_t *data, uint32_t timeout);
uint16_t USART2_NFC_GetRawRxCount(void);
uint16_t USART2_NFC_CopyRawRx(uint8_t *data, uint16_t maxLength);

extern volatile uint32_t usart2_irq_count;
extern volatile uint8_t usart2_last_rx;
extern volatile uint8_t usart2_irq_rx[4];
void USART2_NFC_ResetIrqDiagnostic(void);

#endif
