#include "stm32f10x.h"
#include "bsp_ws2812.h"


void delay(uint32_t t)
{
    while(t--);
}



int main(void)
{

    WS2812_FR_Init();

    WS2812_BA_Init();


    while(1)
    {


        int i;


        for(i=0;i<6;i++)
        {

            WS2812_FR_Show(
                i,
                255,
                0,
                0
            );


            WS2812_BA_Show(
                5-i,
                0,
                0,
                255
            );


            delay(3000000);

        }


    }

}
