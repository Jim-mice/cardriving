#include "bsp_light.h"
#include "bsp_ws2812.h"


void Light_Init(void)
{
    WS2812_FR_Init();

    WS2812_BA_Init();
}



void Light_Run(void)
{

    WS2812_BA_SetLED(
        0,
        255,
        0,
        0
    );

}


