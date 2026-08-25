#include "stm32f10x.h"
#include "drv_usart1_hi3861.h"


int main(void)
{

    USART1_Hi3861_Init();


    while(1)
    {

        USART1_SendString(
            "STM32 USART1 OK\r\n"
        );


        for(
            volatile int i=0;
            i<500000;
            i++
        );

    }

}
