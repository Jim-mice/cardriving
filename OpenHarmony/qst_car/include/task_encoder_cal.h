#ifndef TASK_ENCODER_CAL_H
#define TASK_ENCODER_CAL_H
#include <stdint.h>
void EncoderCalInit(void);
int EncoderCalIsActive(void);
void EncoderCalStep(uint32_t now,int *left,int *right);
#endif
