#include "bsp_light.h"
#include "bsp_ws2812.h"

static const uint8_t g_lightColors[][3] = {
    {255, 0, 0}, {0, 255, 0}, {0, 0, 255},
    {255, 255, 0}, {0, 255, 255}, {255, 0, 255}
};


static uint8_t pos = 0;


void Light_Init(void)
{
    WS2812_FR_Init();

    WS2812_BA_Init();
}


void Light_Run(void)
{
    return;

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

void Light_SetEvent(uint8_t event, uint8_t color)
{
    const uint8_t *c = g_lightColors[color % 6U];
    uint8_t i;
    if (event == 1U) {
        for (i = 0U; i < 6U; i++) {
            WS2812_FR_SetLED(i, c[0], c[1], c[2]);
            WS2812_BA_SetLED(i, c[0], c[1], c[2]);
        }
    } else if (event == 2U) {
        WS2812_FR_SetLED(0U, c[0], c[1], c[2]);
        WS2812_BA_SetLED(5U, c[0], c[1], c[2]);
    } else if (event == 3U) {
        WS2812_FR_SetLED(5U, c[0], c[1], c[2]);
        WS2812_BA_SetLED(0U, c[0], c[1], c[2]);
    }
}
