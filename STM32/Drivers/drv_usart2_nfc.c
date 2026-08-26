#include "drv_usart2_nfc.h"

#include "misc.h"

#define USART2_NFC_RX_BUFFER_SIZE USART2_NFC_RX_DEBUG_CAPACITY

static volatile uint8_t g_usart2RxBuffer[USART2_NFC_RX_BUFFER_SIZE];
static volatile uint16_t g_usart2RxHead;
static volatile uint16_t g_usart2RxTail;
static volatile uint8_t g_usart2RawRxBuffer[USART2_NFC_RX_BUFFER_SIZE];
static volatile uint16_t g_usart2RawRxCount;
volatile uint32_t usart2_irq_count;
volatile uint8_t usart2_last_rx;
volatile uint8_t usart2_irq_rx[4];

void USART2_NFC_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef usart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    USART_StructInit(&usart);
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART2, &usart);

    g_usart2RxHead = 0;
    g_usart2RxTail = 0;
    g_usart2RawRxCount = 0;
    usart2_irq_count = 0;
    usart2_last_rx = 0;
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    USART_Cmd(USART2, ENABLE);
}

void USART2_NFC_SendByte(uint8_t data)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
    }
    USART_SendData(USART2, data);
}

void USART2_NFC_SendBuffer(const uint8_t *data, uint16_t length)
{
    uint16_t index;

    for (index = 0; index < length; index++) {
        USART2_NFC_SendByte(data[index]);
    }
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET) {
    }
}

void USART2_NFC_ClearRx(void)
{
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    g_usart2RxHead = 0;
    g_usart2RxTail = 0;
    g_usart2RawRxCount = 0;
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

uint8_t USART2_NFC_ReadByte(uint8_t *data, uint32_t timeout)
{
    while (timeout > 0U) {
        if (g_usart2RxTail != g_usart2RxHead) {
            *data = g_usart2RxBuffer[g_usart2RxTail];
            g_usart2RxTail = (uint16_t)((g_usart2RxTail + 1U) % USART2_NFC_RX_BUFFER_SIZE);
            return 1;
        }
        timeout--;
    }

    return 0;
}

uint16_t USART2_NFC_GetRawRxCount(void)
{
    uint16_t count;

    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    count = g_usart2RawRxCount;
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    return count;
}

uint16_t USART2_NFC_CopyRawRx(uint8_t *data, uint16_t maxLength)
{
    uint16_t count;
    uint16_t index;

    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
    count = g_usart2RawRxCount;
    if (count > maxLength) {
        count = maxLength;
    }
    for (index = 0; index < count; index++) {
        data[index] = g_usart2RawRxBuffer[index];
    }
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    return count;
}

void USART2_NFC_ResetIrqDiagnostic(void)
{
    uint8_t index;

    usart2_irq_count = 0;
    usart2_last_rx = 0;
    for (index = 0U; index < sizeof(usart2_irq_rx); index++) {
        usart2_irq_rx[index] = 0U;
    }
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART_ReceiveData(USART2);

        usart2_last_rx = data;
        if (usart2_irq_count < sizeof(usart2_irq_rx)) {
            usart2_irq_rx[usart2_irq_count] = data;
        }
        usart2_irq_count++;
    }
}
