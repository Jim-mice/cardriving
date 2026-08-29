#ifndef TASK_ENCODER_SEGMENT_B_PROGRESS_H
#define TASK_ENCODER_SEGMENT_B_PROGRESS_H

#include <stdint.h>

/* This module only selects requested wheel commands. TaskCarControl owns motor I/O. */
void BProgressInit(void);
void BProgressStep(uint32_t now, int *leftCommand, int *rightCommand);

#endif
