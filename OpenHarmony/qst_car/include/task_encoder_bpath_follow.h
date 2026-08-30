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
/* Lifecycle ownership remains in TaskCarControl; this only reports run end. */
int BPathFollowIsFinished(void);

/* TRACE owns forward motion; these APIs only record encoder travel and then
 * enter the existing reverse follower. */
int BPathExternalRecordStart(uint32_t now);
int BPathExternalRecordStep(uint32_t now);
int BPathExternalRecordFinish(uint32_t now);
int BPathExternalReturnStart(uint32_t now);
int BPathExternalReturnSettleComplete(uint32_t now);
void BPathExternalRecordStop(uint32_t now);
/* Lifecycle guards only; they do not select or send motor commands. */
int BPathExternalHasForwardMovement(void);
void BPathExternalAbort(const char *reason);

#endif
