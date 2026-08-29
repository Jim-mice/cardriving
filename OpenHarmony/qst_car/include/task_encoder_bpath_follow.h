#ifndef TASK_ENCODER_BPATH_FOLLOW_H
#define TASK_ENCODER_BPATH_FOLLOW_H

#include <stdint.h>

/* This module only selects requested wheel commands. TaskCarControl owns motor I/O. */
void BPathFollowInit(void);
void BPathFollowStep(uint32_t now, int *leftCommand, int *rightCommand);
/* TaskCarControl alone consumes this one-shot request, sends the selected
 * command, waits briefly, then sends the coordinated terminal stop. */
int BPathFollowTakeTerminalExecutionRequest(uint32_t *delayMs);
void BPathFollowNotifyTerminalStopExecuted(uint32_t actualStopMs,
                                           uint32_t actualWaitUs);

#endif
