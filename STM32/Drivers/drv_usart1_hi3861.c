#include "drv_usart1_hi3861.h"
#include "misc.h"

uint8_t USART1_RX_Buffer[64];

uint8_t USART1_RX_Index=0;

uint8_t USART1_RX_Flag=0;



void USART1_Hi3861_Init(void)
{

GPIO_InitTypeDef GPIO_InitStructure;

USART_InitTypeDef USART_InitStructure;

NVIC_InitTypeDef NVIC_InitStructure;



RCC_APB2PeriphClockCmd(
    RCC_APB2Periph_GPIOA |
    RCC_APB2Periph_USART1,
    ENABLE
);



/*
 PA9 TX
*/

GPIO_InitStructure.GPIO_Pin =
    GPIO_Pin_9;

GPIO_InitStructure.GPIO_Mode =
    GPIO_Mode_AF_PP;

GPIO_InitStructure.GPIO_Speed =
    GPIO_Speed_50MHz;


GPIO_Init(
    GPIOA,
    &GPIO_InitStructure
);



/*
 PA10 RX
*/

GPIO_InitStructure.GPIO_Pin =
    GPIO_Pin_10;

GPIO_InitStructure.GPIO_Mode =
    GPIO_Mode_IN_FLOATING;


GPIO_Init(
    GPIOA,
    &GPIO_InitStructure
);



USART_InitStructure.USART_BaudRate=115200;

USART_InitStructure.USART_WordLength=
USART_WordLength_8b;

USART_InitStructure.USART_StopBits=
USART_StopBits_1;

USART_InitStructure.USART_Parity=
USART_Parity_No;

USART_InitStructure.USART_HardwareFlowControl=
USART_HardwareFlowControl_None;

USART_InitStructure.USART_Mode=
USART_Mode_Tx | USART_Mode_Rx;



USART_Init(
    USART1,
    &USART_InitStructure
);



/*
 开启接收中断
*/

USART_ITConfig(
    USART1,
    USART_IT_RXNE,
    ENABLE
);



NVIC_InitStructure.NVIC_IRQChannel=
USART1_IRQn;

NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=
1;

NVIC_InitStructure.NVIC_IRQChannelSubPriority=
1;

NVIC_InitStructure.NVIC_IRQChannelCmd=
ENABLE;


NVIC_Init(
    &NVIC_InitStructure
);



USART_Cmd(
    USART1,
    ENABLE
);

}




void USART1_SendByte(uint8_t data)
{

USART_SendData(
    USART1,
    data
);


while(
USART_GetFlagStatus(
USART1,
USART_FLAG_TXE)==RESET
);

}



void USART1_SendString(char *str)
{

while(*str)
{

USART1_SendByte(
    *str++
);

}

}





void USART1_IRQHandler(void)
{

if(
USART_GetITStatus(
USART1,
USART_IT_RXNE)
!=RESET)
{


uint8_t data;


data =
USART_ReceiveData(
USART1
);



if(data=='\r' || data=='\n')
{

USART1_RX_Buffer[
USART1_RX_Index
]=0;


USART1_RX_Flag=1;


USART1_RX_Index=0;

}
else
{

if(
USART1_RX_Index<63)
{

USART1_RX_Buffer[
USART1_RX_Index++
]
=data;

}

}


USART_ClearITPendingBit(
USART1,
USART_IT_RXNE
);

}

}

