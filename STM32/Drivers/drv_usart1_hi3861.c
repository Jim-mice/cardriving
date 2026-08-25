#include "drv_usart1_hi3861.h"
#include "misc.h"

volatile uint8_t USART1_RX_Buffer[64];

uint8_t USART1_RX_Index=0;

volatile uint8_t USART1_RX_Flag=0;

static uint8_t g_rxFrame[USART1_HI3861_FRAME_LEN];
static uint8_t g_rxFrameIndex=0;



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





uint8_t USART1_GetReceivedFrame(uint8_t *frame)
{
uint8_t index;

if(USART1_RX_Flag==0)
{
return 0;
}

USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);

for(index=0;index<USART1_HI3861_FRAME_LEN;index++)
{
frame[index]=USART1_RX_Buffer[index];
}

USART1_RX_Flag=0;

USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

return 1;
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



/* ISR only frames bytes. Printing and application behavior stay in main(). */
if(g_rxFrameIndex==0)
{
if(data==USART1_HI3861_FRAME_HEAD)
{
g_rxFrame[g_rxFrameIndex++]=data;
}
}
else if(g_rxFrameIndex<(USART1_HI3861_FRAME_LEN-1))
{
g_rxFrame[g_rxFrameIndex++]=data;
}
else if(data==USART1_HI3861_FRAME_TAIL)
{
uint8_t index;

g_rxFrame[g_rxFrameIndex]=data;

/* Keep the most recently completed frame for the foreground task. */
for(index=0;index<USART1_HI3861_FRAME_LEN;index++)
{
USART1_RX_Buffer[index]=g_rxFrame[index];
}

USART1_RX_Flag=1;
g_rxFrameIndex=0;
}
else if(data==USART1_HI3861_FRAME_HEAD)
{
g_rxFrame[0]=data;
g_rxFrameIndex=1;
}
else
{
g_rxFrameIndex=0;
}


USART_ClearITPendingBit(
USART1,
USART_IT_RXNE
);

}

}
