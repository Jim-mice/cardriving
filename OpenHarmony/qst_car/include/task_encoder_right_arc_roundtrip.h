#ifndef TASK_ENCODER_RIGHT_ARC_ROUNDTRIP_H
#define TASK_ENCODER_RIGHT_ARC_ROUNDTRIP_H

#include <stdint.h>

/* This module only chooses requested wheel commands. TaskCarControl owns motor I/O. */
void RightArcRoundtripInit(void);
void RightArcRoundtripStep(uint32_t now, int *leftCommand, int *rightCommand);

#endif
