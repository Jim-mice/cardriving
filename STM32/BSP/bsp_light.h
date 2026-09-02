#ifndef __BSP_LIGHT_H
#define __BSP_LIGHT_H

#include <stdint.h>


void Light_Init(void);

void Light_Run(void);
void Light_SetEvent(uint8_t event, uint8_t color);


#endif
