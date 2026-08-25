#include "stm32f10x.h"

#include "bsp_light.h"
#include "drv_usart1_hi3861.h"


void delay(uint32_t t)
{
    while(t--);
}


int main(void)
{

    Light_Init();

    USART1_Hi3861_Init();


    USART1_SendString(
        "STM32 READY\r\n"
    );


    while(1)
    {

        /*
            ?????
        */
        Light_Run();


        /*
            ??????
        */
        if(USART1_RX_Flag)
        {

            USART1_RX_Flag=0;


            USART1_SendString(
                "RECV:"
            );


            USART1_SendString(
                (char*)USART1_RX_Buffer
            );


            USART1_SendString(
                "\r\n"
            );

        }


        delay(3000000);

    }

}
