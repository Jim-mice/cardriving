#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include <stdint.h>

void encoder_init(void);
int16_t encoder_left_get_delta(void);
int16_t encoder_right_get_delta(void);

#endif
