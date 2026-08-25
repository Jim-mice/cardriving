#include "drv_usart1_hi3861.h"


void USART1_Hi3861_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;


    /*
        开启时钟

        GPIOA
        USART1
    */

    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA |
        RCC_APB2Periph_USART1,
        ENABLE
    );


    /*
        PA9 USART1_TX
        复用推挽输出
    */

    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_9;

    GPIO_InitStructure.GPIO_Mode =
        GPIO_Mode_AF_PP;

    GPIO_InitStructure.GPIO_Speed =
        GPIO_Speed_50MHz;

    GPIO_Init(GPIOA,&GPIO_InitStructure);



    /*
        PA10 USART1_RX
        浮空输入
    */

    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_10;

    GPIO_InitStructure.GPIO_Mode =
        GPIO_Mode_IN_FLOATING;

    GPIO_Init(GPIOA,&GPIO_InitStructure);



    /*
        USART参数

        这里先按照 Hi3861 常用：
        115200 8N1
    */

    USART_InitStructure.USART_BaudRate =
        115200;

    USART_InitStructure.USART_WordLength =
        USART_WordLength_8b;

    USART_InitStructure.USART_StopBits =
        USART_StopBits_1;

    USART_InitStructure.USART_Parity =
        USART_Parity_No;

    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;

    USART_InitStructure.USART_Mode =
        USART_Mode_Tx | USART_Mode_Rx;


    USART_Init(
        USART1,
        &USART_InitStructure
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
            USART_FLAG_TXE
        ) == RESET
    );

}



void USART1_SendString(char *str)
{
    while(*str)
    {
        USART1_SendByte(*str++);
    }
}
