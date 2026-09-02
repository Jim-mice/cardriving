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
/* A rolling-idle encoder loss can discard only the optional recent-history
 * recorder.  These helpers intentionally do not drive motors or relax any
 * reverse/recovery failure checks. */
int BPathExternalEncoderFresh(uint32_t now);
int BPathExternalLastAbortWasEncoderRxTimeout(void);
void BPathExternalInvalidateIdleHistory(void);
/* Normal TRACE may keep only a compact rolling tail while no fork semantics
 * need a reversible path.  Disabling it freezes source indices for E1/E2. */
void BPathExternalSetIdleRolling(uint8_t enabled, const char *reason);
int BPathExternalRecordFinish(uint32_t now);
int BPathExternalReturnStart(uint32_t now);
int BPathExternalReturnSettleComplete(uint32_t now);
/* Read-only completion reason for safety owners that must not retry after a
 * reverse follower abort. */
int BPathExternalReturnAborted(void);
void BPathExternalRecordStop(uint32_t now);
/* Lifecycle guards only; they do not select or send motor commands. */
int BPathExternalHasForwardMovement(void);
void BPathExternalAbort(const char *reason);
/* Read-only recorder fill level for always-on telemetry. */
void BPathExternalGetForwardRecordProgress(uint16_t *count, uint16_t *capacity);

/* A forward marker has a stable serial identity within its recording epoch.
 * physicalIndex is only a current-array diagnostic; callers must not retain it
 * across an idle rolling compaction or a recorder restart. */
typedef struct {
    uint16_t physicalIndex;
    uint32_t sourceSerial;
    uint32_t epoch;
    int32_t encoderLeft;
    int32_t encoderRight;
} BPathForwardMarker;
int BPathExternalGetForwardRecordMarker(BPathForwardMarker *marker);
int BPathExternalGetForwardBaseMarker(BPathForwardMarker *marker);
int BPathExternalGetForwardMarkerBySerial(uint32_t sourceSerial,
                                          BPathForwardMarker *marker);

/* Test-only read-only path progress helpers; TaskCarControl still owns motors. */
int BPathExternalGetForwardRecordIndex(uint16_t *index);
/* Read-only cumulative encoder travel stored at one forward source point. */
int BPathExternalGetForwardPointTravel(uint16_t index, int32_t *left, int32_t *right);
/* Read-only source/reference bracketing used to diagnose safe reverse marks.
 * A source index of 0xffff means that side of the bracket is unavailable. */
typedef struct {
    uint16_t forwardPoints;
    uint16_t referencePoints;
    uint16_t firstReferenceSourceIndex;
    uint16_t lastReferenceSourceIndex;
    uint16_t nearestBeforeSourceIndex;
    uint16_t nearestBeforeReferenceIndex;
    uint16_t nearestAfterSourceIndex;
    uint16_t nearestAfterReferenceIndex;
} BPathReferenceMapDiagnostics;
int BPathExternalGetReferenceMapDiagnostics(uint16_t sourceIndex,
                                             BPathReferenceMapDiagnostics *diagnostics);
int BPathExternalMapForwardIndexToReference(uint16_t forwardIndex, uint16_t *referenceIndex);
int BPathExternalMapForwardSourceSerialToReference(uint32_t sourceSerial,
                                                    uint16_t *referenceIndex,
                                                    BPathForwardMarker *mappedMarker);
int BPathExternalGetReferenceMarker(uint16_t referenceIndex,
                                    BPathForwardMarker *marker);
int BPathExternalGetReturnReferenceCursor(uint16_t *referenceIndex);

#endif
