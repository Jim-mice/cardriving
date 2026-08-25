#include "stm32f10x.h"
#include "bsp_gpio.h"


void delay(void)
{
    volatile int i;

    for(i=0;i<500000;i++);
}


int main(void)
{

    BSP_GPIO_Init();


    while(1)
    {

        GPIO_SetBits(GPIOC,GPIO_Pin_13);

        delay();


        GPIO_ResetBits(GPIOC,GPIO_Pin_13);

        delay();

    }

}
