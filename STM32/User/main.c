#include "stm32f10x.h"

#include "bsp_light.h"
#include "bsp_gpio.h"
#include "drv_usart1_hi3861.h"


void delay(uint32_t t)
{
    while(t--);
}

static void USART1_PrintFrame(uint8_t *frame)
{
static const char hex[]="0123456789ABCDEF";
uint8_t index;

USART1_SendString("STM32 RX:");

for(index=0;index<USART1_HI3861_FRAME_LEN;index++)
{
USART1_SendByte(hex[(frame[index]>>4)&0x0F]);
USART1_SendByte(hex[frame[index]&0x0F]);
USART1_SendByte(' ');
}

USART1_SendString("\r\n");
}


int main(void)
{

    uint8_t frame[USART1_HI3861_FRAME_LEN];
    uint8_t ledState=0;

    BSP_GPIO_Init();
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
        if(USART1_GetReceivedFrame(frame))
        {
            USART1_PrintFrame(frame);

            /* PC13 is active-low; toggle it for each valid received frame. */
            if(ledState==0)
            {
                LED1_ON();
                ledState=1;
            }
            else
            {
                LED1_OFF();
                ledState=0;
            }

        }


        delay(3000000);

    }

}
