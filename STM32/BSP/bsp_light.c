#include "bsp_light.h"
#include "bsp_ws2812.h"


static uint8_t pos = 0;


void Light_Init(void)
{
    WS2812_FR_Init();

    WS2812_BA_Init();
}


void Light_Run(void)
{

    WS2812_FR_Show(
        pos,
        255,
        0,
        0
    );


    WS2812_BA_Show(
        5-pos,
        0,
        0,
        255
    );


    pos++;


    if(pos>=6)
        pos=0;

}
