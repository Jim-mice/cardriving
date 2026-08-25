#ifndef __BSP_WS2812_H
#define __BSP_WS2812_H


#include "stdint.h"


void WS2812_FR_Init(void);

void WS2812_BA_Init(void);


void WS2812_FR_Show(
uint8_t index,
uint8_t r,
uint8_t g,
uint8_t b);


void WS2812_BA_Show(
uint8_t index,
uint8_t r,
uint8_t g,
uint8_t b);



#endif


void WS2812_FR_SetLED(
uint8_t index,
uint8_t r,
uint8_t g,
uint8_t b);


void WS2812_BA_SetLED(
uint8_t index,
uint8_t r,
uint8_t g,
uint8_t b);

