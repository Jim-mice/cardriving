#ifndef TASK_ENCODER_ARC_ROUNDTRIP_H
#define TASK_ENCODER_ARC_ROUNDTRIP_H

#include <stdint.h>

/* This module only chooses requested wheel commands. TaskCarControl owns motor I/O. */
void ArcRoundtripInit(void);
void ArcRoundtripStep(uint32_t now, int *leftCommand, int *rightCommand);

#endif
