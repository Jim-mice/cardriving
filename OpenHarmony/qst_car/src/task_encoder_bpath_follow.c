#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_time.h"
#include "task_encoder_bpath_follow.h"
#include "udp_telemetry.h"

/* Forward is intentionally identical to the validated A->B experiment. */
#define BPF_A_LEFT 60
#define BPF_A_RIGHT 120
#define BPF_B_LEFT 120
#define BPF_B_RIGHT 60
#define BPF_SEGMENT_MS 300U
#define BPF_FORWARD_SETTLE_MS 800U
#define BPF_REVERSE_PAUSE_MS 300U
#define BPF_REVERSE_SETTLE_MS 800U
#define BPF_ARM_MS 5000U
#define BPF_READY_FRAMES 20U
#define BPF_READY_AGE_MS 100U
#define BPF_RX_TIMEOUT_MS 200U
/* Diagnostic safety ceiling only: normal terminal/FOLLOW completion exits earlier. */
#define BPF_REVERSE_TIMEOUT_MS 30000U
#define BPF_DIRECTION_WRONG 50
#define BPF_FINAL_STOP_LEAD 60
#define BPF_PATH_ERROR_ENTER 20
#define BPF_PATH_ERROR_RELEASE 10
#define BPF_TERMINAL_ARM_S 250
#define BPF_TERMINAL_FALLBACK_S 120
#define BPF_VS30_MAX 200
#define BPF_PREDICTED_COAST_MAX 300
#define BPF_WHEEL_COAST_NUM 2
#define BPF_WHEEL_COAST_DEN 1
#define BPF_WHEEL_COAST_MAX 200
#define BPF_TERMINAL_TIME_DIFF_TOLERANCE_MS 10
#define BPF_TERMINAL_OVERSHOOT_SAFETY 100
#define BPF_POST_TRIM_TRIGGER_TICKS 15
#define BPF_POST_TRIM_PULSE_MS 30U
#define BPF_POST_TRIM_SETTLE_MS 500U
#define BPF_POST_TRIM_MAX_AGE_MS 200U
#define BPF_MANUAL_MOVE_START_TICKS 5
#define BPF_MANUAL_IDLE_MS 1000U
#define BPF_MANUAL_TIMEOUT_MS 15000U
#define BPF_MANUAL_LOG_MS 200U
#define BPF_PHYSICAL_DIFF_TARGET (-520)
#define BPF_PDIFF_COAST_MAX 200
#define BPF_PDIFF_MAX_MS 1500U
#define BPF_PDIFF_SETTLE_MS 800U
#define BPF_PDIFF_AGE_MS 200U
#define BPF_PDIFF_WRONG_DIR 20
#define BPF_RESALIGN_IDLE_MS 1000U
#define BPF_RESALIGN_TIMEOUT_MS 15000U
#define BPF_MAX_POINTS 640U
/* With no unresolved fork, preserve only enough source history for the
 * pre-E1 extractor (16 lookback + 3 edge skip + margin). */
#define BPF_IDLE_TAIL_POINTS 32U
#define BPF_IDLE_COMPACT_TRIGGER_POINTS 64U

typedef enum { BPF_WAIT = 0, BPF_ARM, BPF_FORWARD_A, BPF_FORWARD_B,
               BPF_FORWARD_SETTLE, BPF_REVERSE_PAUSE, BPF_REVERSE,
               BPF_REVERSE_SETTLE, BPF_POSTTRIM_PREP, BPF_POSTTRIM_DRIVE,
               BPF_POSTTRIM_SETTLE, BPF_MALIGN_PREP, BPF_MALIGN_RUN,
               BPF_PDIFF_PREP, BPF_PDIFF_DRIVE, BPF_PDIFF_SETTLE,
               BPF_RESALIGN_PREP, BPF_RESALIGN_RUN,
               BPF_DONE, BPF_ABORT } BpfState;
typedef enum { BPF_SYNC = 0, BPF_RIGHT_AHEAD, BPF_RIGHT_BEHIND } BpfSyncState;
typedef enum { BPF_FWD_ACTIVE = 0, BPF_FWD_COAST, BPF_FWD_FINAL,
               BPF_FOLLOW_ACTIVE, BPF_FOLLOW_COAST, BPF_FOLLOW_FINAL } BpfPhase;
typedef enum { BPF_DUMP_IDLE = 0, BPF_DUMP_INFO, BPF_DUMP_FWD,
               BPF_DUMP_FOLLOW, BPF_DUMP_RESULT, BPF_DUMP_TEXEC, BPF_DUMP_DONE,
               BPF_DUMP_COMPLETE } BpfDumpState;
typedef enum { BPF_TVEC_NONE = 0, BPF_TVEC_LEFT_ONLY, BPF_TVEC_RIGHT_ONLY,
               BPF_TVEC_BOTH } BpfTerminalVector;

typedef struct {
    uint8_t phase, seq;
    uint16_t index;
    uint32_t timeMs;
    int32_t left, right;
    int32_t desiredLeft, desiredRight, pathError;
    uint16_t refIndex;
    int16_t commandLeft, commandRight;
} BpfPathPoint;
typedef struct { int32_t left, right, s; } BpfReferencePoint;
typedef struct {
    uint8_t valid, aborted, referenceInvalid;
    const char *abortReason;
    int32_t fStartL, fStartR, boundaryL, boundaryR, fEndL, fEndR;
    int32_t bLeft, bRight;
    int32_t rStartL, rStartR, rEndL, rEndR, reverseLeft, reverseRight;
    int32_t errorLeft, errorRight, errorLeftPermille, errorRightPermille;
    int32_t maxAbsPathErrorTicks;
    int32_t terminalRemainingSAtSchedule, terminalVs30, terminalPredictedCoastS;
    int32_t terminalStopLeft, terminalStopRight;
    int32_t terminalStopRemainingLeft, terminalStopRemainingRight, terminalStopRemainingS;
    uint32_t fStartFrames, fEndFrames, rStartFrames, rEndFrames;
    uint32_t fFrames, rFrames, fDurationMs, reverseDurationMs, followElapsedMs;
    uint32_t terminalArmMs, terminalScheduleMs, terminalDelayMs;
    uint32_t terminalActualWaitUs, terminalActualStopMs;
    int32_t scheduleRemainingL, scheduleRemainingR;
    int32_t scheduleVL30, scheduleVR30;
    int32_t schedulePredictedCoastL, schedulePredictedCoastR;
    int32_t scheduleResidualL, scheduleResidualR;
    int32_t scheduleTimeL, scheduleTimeR;
    uint32_t leftReachedMs, rightReachedMs;
    uint32_t rightAheadEnterCount, rightBehindEnterCount;
    uint8_t fStartSeq, boundarySeq, fEndSeq, rStartSeq, rEndSeq;
    uint8_t terminalFallback;
    int32_t autoErrorLeft, autoErrorRight;
    int32_t manualStartLeft, manualStartRight, manualFinalLeft, manualFinalRight;
    int32_t manualDeltaL, manualDeltaR, manualRawDeltaL, manualRawDeltaR;
    int32_t manualDifferential;
    int32_t pdiffStartL, pdiffStartR, pdiffStartDiff, pdiffStopDiff, pdiffFinalDiff;
    int32_t pdiffActualGain, pdiffCoastGain;
    int32_t pdiffFinalErrorL, pdiffFinalErrorR;
    int32_t pdiffV30, pdiffPredictedCoast;
    uint8_t pdiffValid;
    int32_t resStartL, resStartR, resFinalDiff, resDeltaL, resDeltaR, resGain;
    uint8_t resClean;
    int32_t trimPreLeft, trimPreRight, trimPreLeftExcess, trimPreRightExcess, trimPreError;
    int32_t trimPostLeft, trimPostRight, trimDeltaL, trimDeltaR, trimPostError, trimCorrection, trimRelativeGain;
    uint32_t trimActualPulseMs;
    uint8_t trimSide;
} BpfResult;

static BpfState g_state;
static BpfSyncState g_sync;
static BpfDumpState g_dumpState;
static BpfResult g_result;
static BpfPathPoint g_forwardPath[BPF_MAX_POINTS];
static BpfPathPoint g_followPath[BPF_MAX_POINTS];
static BpfReferencePoint g_reference[BPF_MAX_POINTS];
/* Source forward index for each reverse-reference point, descending by design. */
static uint16_t g_referenceForwardIndex[BPF_MAX_POINTS];
static uint16_t g_forwardCount, g_followCount, g_referenceCount;
/* Serial identity is derived, never duplicated in every point.  The forward
 * buffer stays serial-contiguous; idle compaction advances its base. */
static uint32_t g_forwardSourceSerialNext;
static uint32_t g_forwardBaseSourceSerial;
static uint16_t g_forwardSerialAssignedCount;
/* Epoch changes only when a wholly new external recorder is started. */
static uint32_t g_forwardEpoch;
static uint16_t g_forwardDumpIndex, g_followDumpIndex, g_referenceIndex;
static uint32_t g_stateMs, g_followStartMs, g_readyBase, g_lastArm, g_lastSummary;
static uint32_t g_forwardPathStartMs, g_reversePathStartMs;
static uint32_t g_forwardLastValid, g_reverseLastValid;
static uint8_t g_forwardOverflow, g_followOverflow;
static uint8_t g_leftDone, g_rightDone, g_leftPublished, g_rightPublished;
static uint8_t g_terminalArmed, g_terminalScheduled, g_terminalStopped;
static uint8_t g_terminalVelocityValid, g_terminalVelocitySamples;
static uint32_t g_terminalScheduledAtMs;
static uint8_t g_terminalExecutionRequestPending;
static uint32_t g_terminalPreviousValid;
static uint8_t g_terminalPreviousSeq;
static int32_t g_terminalPreviousLeftTravel, g_terminalPreviousRightTravel;
static int32_t g_terminalVs30Now, g_terminalVs30Previous;
static int g_terminalFrozenLeft, g_terminalFrozenRight;
static BpfTerminalVector g_terminalVector;
static int32_t g_terminalVL30, g_terminalVR30;
static int32_t g_terminalVL30Previous, g_terminalVR30Previous;
static int32_t g_terminalEvalVL30, g_terminalEvalVR30;
static int32_t g_terminalPredictedCoastL, g_terminalPredictedCoastR;
static int32_t g_terminalResidualL, g_terminalResidualR;
static int32_t g_terminalTimeL, g_terminalTimeR;
static uint32_t g_trimPulseStartMs;
static uint8_t g_trimSide;
static int32_t g_trimPreLeft, g_trimPreRight, g_trimPreError;
static uint32_t g_manualStartMs, g_manualLastMoveMs, g_manualLastLogMs;
static int32_t g_manualStartLeft, g_manualStartRight;
static int32_t g_manualStartRawLeft, g_manualStartRawRight;
static int32_t g_manualLastRawLeft, g_manualLastRawRight;
static int32_t g_manualLastLogDL, g_manualLastLogDR;
static uint8_t g_manualMovementStarted;
static int32_t g_pdiffStartL, g_pdiffStartR, g_pdiffStartDiff, g_pdiffLastGain;
static int32_t g_pdiffVNow, g_pdiffVPrev;
static uint8_t g_pdiffVSamples, g_pdiffVelocityValid, g_pdiffValid;
static uint32_t g_pdiffStartMs, g_pdiffStopMs, g_pdiffLastLogMs;
static uint32_t g_pdiffPrevValid;
static uint8_t g_pdiffPrevSeq;
static uint32_t g_resStartMs, g_resLastMoveMs, g_resLastLogMs;
static int32_t g_resStartL, g_resStartR, g_resStartRawL, g_resStartRawR;
static int32_t g_resLastRawL, g_resLastRawR, g_resLastLogDl, g_resLastLogDr;
static uint8_t g_resMovementStarted;
static uint32_t g_externalRecordStopMs;
static uint8_t g_externalReturnNoPdiff;
static uint8_t g_externalIdleRolling;
static uint8_t g_externalIdleRollingEver;
static char g_text[768];

static int32_t BpfAbs(int32_t value) { return value < 0 ? -value : value; }

static const char *BpfTerminalVectorName(BpfTerminalVector state)
{
    switch (state) {
        case BPF_TVEC_LEFT_ONLY: return "L_ONLY";
        case BPF_TVEC_RIGHT_ONLY: return "R_ONLY";
        case BPF_TVEC_BOTH: return "BOTH";
        default: return "NONE";
    }
}

static void BpfPublishTerminalVector(BpfTerminalVector state)
{
    char text[220];
    if (state == g_terminalVector) return;
    g_terminalVector = state;
    (void)snprintf(text, sizeof(text),
        "BPATH event=TVEC state=%s rem_l=%ld rem_r=%ld vl=%ld vr=%ld pc_l=%ld pc_r=%ld res_l=%ld res_r=%ld tl=%ld tr=%ld",
        BpfTerminalVectorName(state),
        (long)(g_terminalResidualL + g_terminalPredictedCoastL),
        (long)(g_terminalResidualR + g_terminalPredictedCoastR),
        (long)g_terminalEvalVL30, (long)g_terminalEvalVR30,
        (long)g_terminalPredictedCoastL, (long)g_terminalPredictedCoastR,
        (long)g_terminalResidualL, (long)g_terminalResidualR,
        (long)g_terminalTimeL, (long)g_terminalTimeR);
    (void)UdpTelemetryQueueExperimentText(text);
}

static int BpfFresh(const UdpEncoderTelemetryState *encoder, uint32_t nowMs, uint32_t ageMs)
{
    return encoder->validCount != 0U && (uint32_t)(nowMs - encoder->lastRxMs) <= ageMs;
}

static const char *BpfPhaseName(uint8_t phase)
{
    switch ((BpfPhase)phase) {
        case BPF_FWD_ACTIVE: case BPF_FOLLOW_ACTIVE: return "ACTIVE";
        case BPF_FWD_COAST: case BPF_FOLLOW_COAST: return "COAST";
        default: return "FINAL";
    }
}

static void BpfPublish(const char *event)
{
    (void)snprintf(g_text, sizeof(g_text),
        "BPATH event=%s mode=FOLLOW target_l=%ld target_r=%ld reverse_l=%ld reverse_r=%ld "
        "error_l=%ld error_r=%ld follow_elapsed_ms=%u timeout_ms=%u max_path_err=%ld terminal_arm_ms=%u terminal_schedule_ms=%u "
        "terminal_delay_ms=%u terminal_vs30=%ld terminal_predicted_coast_s=%ld "
        "terminal_fallback=%u valid=%u aborted=%u reason=%s",
        event, (long)g_result.bLeft, (long)g_result.bRight,
        (long)g_result.reverseLeft, (long)g_result.reverseRight,
        (long)g_result.errorLeft, (long)g_result.errorRight,
        (unsigned int)g_result.followElapsedMs, (unsigned int)BPF_REVERSE_TIMEOUT_MS,
        (long)g_result.maxAbsPathErrorTicks, (unsigned int)g_result.terminalArmMs,
        (unsigned int)g_result.terminalScheduleMs, (unsigned int)g_result.terminalDelayMs,
        (long)g_result.terminalVs30, (long)g_result.terminalPredictedCoastS,
        (unsigned int)g_result.terminalFallback, (unsigned int)g_result.valid,
        (unsigned int)g_result.aborted,
        g_result.abortReason != NULL ? g_result.abortReason : "NONE");
    UdpTelemetryPublishCal(g_text);
}

static void BpfAbort(const char *reason)
{
    if (g_state == BPF_ABORT) return;
    g_result.aborted = 1U; g_result.abortReason = reason; g_state = BPF_ABORT;
    BpfPublish("ABORT");
}

static void BpfUpdateTerminalVelocity(const UdpEncoderTelemetryState *encoder,
                                      int32_t leftTravel, int32_t rightTravel)
{
    int32_t deltaLeft, deltaRight, perFrameLeft, perFrameRight;
    uint8_t sequenceSteps;

    if (encoder->validCount == g_terminalPreviousValid) return;
    if (g_terminalPreviousValid != 0U) {
        sequenceSteps = (uint8_t)(encoder->sequence - g_terminalPreviousSeq);
        deltaLeft = leftTravel - g_terminalPreviousLeftTravel;
        deltaRight = rightTravel - g_terminalPreviousRightTravel;
        if (sequenceSteps >= 1U && sequenceSteps <= 3U && deltaLeft >= 0 &&
            deltaRight >= 0) {
            perFrameLeft = deltaLeft / (int32_t)sequenceSteps;
            perFrameRight = deltaRight / (int32_t)sequenceSteps;
            if (perFrameLeft <= BPF_VS30_MAX && perFrameRight <= BPF_VS30_MAX) {
                g_terminalVs30Previous = g_terminalVs30Now;
                g_terminalVs30Now = perFrameLeft + perFrameRight;
                g_terminalVL30Previous = g_terminalVL30;
                g_terminalVR30Previous = g_terminalVR30;
                g_terminalVL30 = perFrameLeft;
                g_terminalVR30 = perFrameRight;
                if (g_terminalVelocitySamples < 2U) g_terminalVelocitySamples++;
                g_terminalVelocityValid = 1U;
            }
        }
    }
    g_terminalPreviousValid = encoder->validCount;
    g_terminalPreviousSeq = encoder->sequence;
    g_terminalPreviousLeftTravel = leftTravel;
    g_terminalPreviousRightTravel = rightTravel;
}

static int32_t BpfTerminalVs30(void)
{
    if (g_terminalVelocitySamples >= 2U) {
        return (g_terminalVs30Now + g_terminalVs30Previous) / 2;
    }
    return g_terminalVs30Now;
}

static int32_t BpfWheelVelocity(int32_t current, int32_t previous, uint8_t samples)
{
    return samples >= 2U ? (current + previous) / 2 : current;
}

static void BpfEnterTerminalStop(uint32_t ms, int32_t leftTravel, int32_t rightTravel,
                                 int32_t leftRemaining, int32_t rightRemaining,
                                 int32_t remainingS, const char *event)
{
    g_result.terminalStopLeft = leftTravel;
    g_result.terminalStopRight = rightTravel;
    g_result.terminalStopRemainingLeft = leftRemaining;
    g_result.terminalStopRemainingRight = rightRemaining;
    g_result.terminalStopRemainingS = remainingS;
    g_result.terminalActualWaitUs = 0U;
    g_result.terminalActualStopMs = ms - g_stateMs;
    g_terminalStopped = 1U;
    g_result.reverseDurationMs = ms - g_stateMs;
    g_state = BPF_REVERSE_SETTLE;
    g_stateMs = ms;
    BpfPublish(event);
}

int BPathFollowTakeTerminalExecutionRequest(uint32_t *delayMs)
{
    if (delayMs == NULL || !g_terminalExecutionRequestPending ||
        !g_terminalScheduled || g_state != BPF_REVERSE) {
        return 0;
    }
    *delayMs = g_result.terminalDelayMs;
    g_terminalExecutionRequestPending = 0U;
    return 1;
}

void BPathFollowNotifyTerminalStopExecuted(uint32_t actualStopMs,
                                           uint32_t actualWaitUs)
{
    if (!g_terminalScheduled || g_state != BPF_REVERSE) {
        return;
    }
    /* Encoder totals are intentionally not sampled here: TaskCarControl has
     * just waited, so the most recent encoder snapshot is not a fabricated
     * representation of the motor-stop instant. */
    g_terminalScheduled = 0U;
    g_terminalExecutionRequestPending = 0U;
    g_terminalStopped = 1U;
    g_result.terminalActualWaitUs = actualWaitUs;
    g_result.terminalActualStopMs = actualStopMs - g_stateMs;
    g_result.reverseDurationMs = actualStopMs - g_stateMs;
    g_state = BPF_REVERSE_SETTLE;
    g_stateMs = actualStopMs;
    BpfPublish("TERMINAL_STOP_EXECUTED");
}

static void BpfPathAdd(BpfPathPoint *path, uint16_t *count, uint8_t *overflow,
                       uint8_t phase, uint8_t seq, uint32_t timeMs, int32_t left,
                       int32_t right, int32_t desiredLeft, int32_t desiredRight,
                       int32_t pathError, uint16_t referenceIndex,
                       int leftCmd, int rightCmd)
{
    BpfPathPoint *point;
    if (*count >= BPF_MAX_POINTS) { *overflow = 1U; return; }
    point = &path[*count]; point->phase = phase; point->seq = seq; point->index = *count;
    point->timeMs = timeMs; point->left = left; point->right = right;
    point->desiredLeft = desiredLeft; point->desiredRight = desiredRight;
    point->pathError = pathError; point->refIndex = referenceIndex;
    point->commandLeft = (int16_t)leftCmd; point->commandRight = (int16_t)rightCmd;
    (*count)++;
}

static void BpfAssignForwardSourceSerial(void)
{
    if (g_forwardCount > g_forwardSerialAssignedCount) {
        if (g_forwardSerialAssignedCount == 0U) {
            g_forwardBaseSourceSerial = ++g_forwardSourceSerialNext;
        } else {
            g_forwardSourceSerialNext +=
                (uint32_t)(g_forwardCount - g_forwardSerialAssignedCount);
        }
        g_forwardSerialAssignedCount = g_forwardCount;
    }
}

/* Rebase the existing single forward array around its newest retained tail.
 * The stored travel remains real encoder-relative travel, but its new zero is
 * the first retained source point so later appends and BPATH reference build
 * remain internally consistent.  This is called only while no E1/candidate
 * can refer to a forward source index. */
static void BpfCompactIdleForwardTail(void)
{
    uint16_t keep = BPF_IDLE_TAIL_POINTS;
    uint16_t start;
    uint16_t index;
    int32_t baseLeft;
    int32_t baseRight;
    uint32_t baseTime;
    uint16_t before = g_forwardCount;

    if (g_forwardCount <= BPF_IDLE_COMPACT_TRIGGER_POINTS) {
        return;
    }
    if (keep > g_forwardCount) {
        keep = g_forwardCount;
    }
    start = (uint16_t)(g_forwardCount - keep);
    baseLeft = g_forwardPath[start].left;
    baseRight = g_forwardPath[start].right;
    baseTime = g_forwardPath[start].timeMs;
    for (index = 0U; index < keep; index++) {
        g_forwardPath[index] = g_forwardPath[(uint16_t)(start + index)];
        g_forwardPath[index].index = index;
        g_forwardPath[index].timeMs -= baseTime;
        g_forwardPath[index].left -= baseLeft;
        g_forwardPath[index].right -= baseRight;
    }
    g_forwardCount = keep;
    g_forwardBaseSourceSerial += start;
    g_forwardSerialAssignedCount = keep;
    g_result.fStartL += baseLeft;
    g_result.fStartR += baseRight;
    g_result.boundaryL = g_result.fStartL;
    g_result.boundaryR = g_result.fStartR;
    g_forwardPathStartMs += baseTime;
    g_forwardOverflow = 0U;
    (void)snprintf(g_text, sizeof(g_text),
                   "BPATH event=IDLE_COMPACT before=%u after=%u",
                   (unsigned int)before, (unsigned int)keep);
    BpfPublish(g_text);
}

static void BpfStartForwardPath(const UdpEncoderTelemetryState *encoder, uint32_t ms)
{
    g_forwardCount = 0U; g_forwardOverflow = 0U; g_forwardLastValid = encoder->validCount;
    g_forwardSerialAssignedCount = 0U;
    g_forwardPathStartMs = ms;
    BpfPathAdd(g_forwardPath, &g_forwardCount, &g_forwardOverflow, BPF_FWD_ACTIVE,
               encoder->sequence, 0U, 0, 0, 0, 0, 0, 0U, BPF_B_LEFT, BPF_B_RIGHT);
    BpfAssignForwardSourceSerial();
}

static void BpfRecordForward(const UdpEncoderTelemetryState *encoder, uint32_t ms,
                             uint8_t phase, int leftCmd, int rightCmd)
{
    if (encoder->validCount == g_forwardLastValid) return;
    g_forwardLastValid = encoder->validCount;
    BpfPathAdd(g_forwardPath, &g_forwardCount, &g_forwardOverflow, phase, encoder->sequence,
               ms - g_forwardPathStartMs, encoder->totalLeft - g_result.boundaryL,
               encoder->totalRight - g_result.boundaryR, 0, 0, 0, 0U, leftCmd, rightCmd);
    BpfAssignForwardSourceSerial();
}

static void BpfFinalForward(uint32_t ms)
{
    BpfPathAdd(g_forwardPath, &g_forwardCount, &g_forwardOverflow, BPF_FWD_FINAL,
               g_result.fEndSeq, ms - g_forwardPathStartMs, g_result.bLeft,
               g_result.bRight, 0, 0, 0, 0U, 0, 0);
    BpfAssignForwardSourceSerial();
}

static int BpfBuildReference(void)
{
    int32_t previousLeft = -1, previousRight = -1;
    uint16_t inputIndex;
    g_referenceCount = 0U;
    for (inputIndex = g_forwardCount; inputIndex != 0U; inputIndex--) {
        const BpfPathPoint *forward = &g_forwardPath[inputIndex - 1U];
        int32_t left = g_result.bLeft - forward->left;
        int32_t right = g_result.bRight - forward->right;
        if (previousLeft >= 0) {
            if (left < previousLeft) {
                if (previousLeft - left <= 3) left = previousLeft; else return 0;
            }
            if (right < previousRight) {
                if (previousRight - right <= 3) right = previousRight; else return 0;
            }
            if (left == previousLeft && right == previousRight) continue;
        }
        if (g_referenceCount >= BPF_MAX_POINTS) return 0;
        g_reference[g_referenceCount].left = left;
        g_reference[g_referenceCount].right = right;
        g_reference[g_referenceCount].s = left + right;
        g_referenceForwardIndex[g_referenceCount] = inputIndex - 1U;
        g_referenceCount++;
        previousLeft = left; previousRight = right;
    }
    return g_referenceCount >= 2U && g_reference[0].left == 0 &&
           g_reference[0].right == 0 &&
           g_reference[g_referenceCount - 1U].left == g_result.bLeft &&
           g_reference[g_referenceCount - 1U].right == g_result.bRight;
}

static void BpfStartFollowPath(const UdpEncoderTelemetryState *encoder, uint32_t ms)
{
    g_followCount = 0U; g_followOverflow = 0U; g_reverseLastValid = encoder->validCount;
    g_reversePathStartMs = g_followStartMs = ms;
    g_result.followElapsedMs = 0U;
    BpfPathAdd(g_followPath, &g_followCount, &g_followOverflow, BPF_FOLLOW_ACTIVE,
               encoder->sequence, 0U, 0, 0, 0, 0, 0, 0U, 0, 0);
}

static void BpfRecordFollow(const UdpEncoderTelemetryState *encoder, uint32_t ms,
                            uint8_t phase, int32_t desiredLeft, int32_t desiredRight,
                            int32_t pathError, uint16_t referenceIndex,
                            int leftCmd, int rightCmd)
{
    if (encoder->validCount == g_reverseLastValid) return;
    g_reverseLastValid = encoder->validCount;
    BpfPathAdd(g_followPath, &g_followCount, &g_followOverflow, phase, encoder->sequence,
               ms - g_reversePathStartMs, -(encoder->totalLeft - g_result.rStartL),
               -(encoder->totalRight - g_result.rStartR), desiredLeft, desiredRight,
               pathError, referenceIndex, leftCmd, rightCmd);
}

static void BpfFinalFollow(uint32_t ms)
{
    BpfPathAdd(g_followPath, &g_followCount, &g_followOverflow, BPF_FOLLOW_FINAL,
               g_result.rEndSeq, ms - g_reversePathStartMs, -g_result.reverseLeft,
               -g_result.reverseRight, g_result.bLeft, g_result.bRight, 0,
               g_referenceCount - 1U, 0, 0);
}

static void BpfFinalizeResult(const UdpEncoderTelemetryState *encoder, uint32_t ms)
{
    g_result.rEndL = encoder->totalLeft; g_result.rEndR = encoder->totalRight;
    g_result.rEndSeq = encoder->sequence; g_result.rEndFrames = encoder->validCount;
    g_result.reverseLeft = encoder->totalLeft - g_result.rStartL;
    g_result.reverseRight = encoder->totalRight - g_result.rStartR;
    g_result.rFrames = encoder->validCount - g_result.rStartFrames;
    g_result.followElapsedMs = ms - g_followStartMs;
    g_result.errorLeft = g_result.bLeft + g_result.reverseLeft;
    g_result.errorRight = g_result.bRight + g_result.reverseRight;
    g_result.errorLeftPermille = g_result.errorLeft * 1000 / g_result.bLeft;
    g_result.errorRightPermille = g_result.errorRight * 1000 / g_result.bRight;
    g_result.valid = 1U;
    BpfFinalFollow(ms); g_forwardDumpIndex = g_followDumpIndex = 0U;
    g_dumpState = BPF_DUMP_INFO; g_state = BPF_DONE; g_lastSummary = ms;
    BpfPublish("RESULT");
}

static void BpfPublishTrimSkip(const char *reason)
{
    char text[120];
    (void)snprintf(text, sizeof(text), "BPATH event=PTRIM_SKIP err=%ld reason=%s",
                   (long)g_trimPreError, reason);
    (void)UdpTelemetryQueueExperimentText(text);
}

static void BpfPublishPdiff(const char *event, int32_t diff, int32_t rem, int32_t gain)
{
    char text[210];
    (void)snprintf(text, sizeof(text),
        "BPATH event=%s diff=%ld rem=%ld gain=%ld vd=%ld pc=%ld",
        event, (long)diff, (long)rem, (long)gain, (long)g_pdiffVNow,
        (long)(g_pdiffVNow > 0 ? (2 * g_pdiffVNow > BPF_PDIFF_COAST_MAX ?
                                  BPF_PDIFF_COAST_MAX : 2 * g_pdiffVNow) : 0));
    (void)UdpTelemetryQueueExperimentText(text);
}

static void BpfUpdatePdiffVelocity(const UdpEncoderTelemetryState *encoder,
                                   int32_t gain, uint32_t *prevValid, uint8_t *prevSeq,
                                   int32_t *prevGain)
{
    uint8_t steps;
    int32_t delta;
    if (encoder->validCount == *prevValid) return;
    if (*prevValid != 0U) {
        steps = (uint8_t)(encoder->sequence - *prevSeq);
        delta = gain - *prevGain;
        if (steps >= 1U && steps <= 3U && delta > 0 && delta <= 200) {
            g_pdiffVPrev = g_pdiffVNow; g_pdiffVNow = delta / (int32_t)steps;
            if (g_pdiffVSamples < 2U) g_pdiffVSamples++;
            g_pdiffVelocityValid = 1U;
        }
    }
    *prevValid = encoder->validCount; *prevSeq = encoder->sequence; *prevGain = gain;
}

static void BpfDesiredAt(int32_t currentS, int32_t *desiredLeft, int32_t *desiredRight,
                         uint16_t *referenceIndex)
{
    BpfReferencePoint *start;
    BpfReferencePoint *end;
    int64_t offset;
    if (g_referenceIndex + 1U < g_referenceCount) {
        while (g_referenceIndex + 1U < g_referenceCount &&
               currentS >= g_reference[g_referenceIndex + 1U].s) {
            g_referenceIndex++;
        }
    }
    if (g_referenceIndex + 1U >= g_referenceCount) {
        *desiredLeft = g_result.bLeft; *desiredRight = g_result.bRight;
        *referenceIndex = g_referenceCount - 1U; return;
    }
    start = &g_reference[g_referenceIndex]; end = &g_reference[g_referenceIndex + 1U];
    if (end->s <= start->s) { *desiredLeft = end->left; *desiredRight = end->right; }
    else {
        offset = currentS - start->s;
        if (offset < 0) offset = 0;
        *desiredLeft = start->left + (int32_t)(((int64_t)(end->left - start->left) * offset) /
                                                (end->s - start->s));
        *desiredRight = start->right + (int32_t)(((int64_t)(end->right - start->right) * offset) /
                                                  (end->s - start->s));
    }
    *referenceIndex = g_referenceIndex;
}

static void BpfQueuePoint(const char *pathName, const BpfPathPoint *point)
{
    (void)snprintf(g_text, sizeof(g_text),
        "BPATH event=POINT mode=FOLLOW path=%s phase=%s idx=%u seq=%u t_ms=%u "
        "l=%ld r=%ld desired_l=%ld desired_r=%ld path_err=%ld ref_idx=%u cmd_l=%d cmd_r=%d",
        pathName, BpfPhaseName(point->phase), (unsigned int)point->index,
        (unsigned int)point->seq, (unsigned int)point->timeMs, (long)point->left,
        (long)point->right, (long)point->desiredLeft, (long)point->desiredRight,
        (long)point->pathError, (unsigned int)point->refIndex,
        (int)point->commandLeft, (int)point->commandRight);
}

static void BpfPumpDump(void)
{
    if (g_dumpState == BPF_DUMP_INFO) {
        (void)snprintf(g_text, sizeof(g_text),
            "BPATH event=INFO mode=FOLLOW b_l=%ld b_r=%ld fwd_count=%u follow_count=%u "
            "ref_count=%u fwd_overflow=%u follow_overflow=%u",
            (long)g_result.bLeft, (long)g_result.bRight, (unsigned int)g_forwardCount,
            (unsigned int)g_followCount, (unsigned int)g_referenceCount,
            (unsigned int)g_forwardOverflow, (unsigned int)g_followOverflow);
        if (UdpTelemetryQueueExperimentText(g_text) != 0) g_dumpState = BPF_DUMP_FWD;
    } else if (g_dumpState == BPF_DUMP_FWD) {
        if (g_forwardDumpIndex >= g_forwardCount) { g_dumpState = BPF_DUMP_FOLLOW; return; }
        BpfQueuePoint("FWD", &g_forwardPath[g_forwardDumpIndex]);
        if (UdpTelemetryQueueExperimentText(g_text) != 0) g_forwardDumpIndex++;
    } else if (g_dumpState == BPF_DUMP_FOLLOW) {
        if (g_followDumpIndex >= g_followCount) { g_dumpState = BPF_DUMP_RESULT; return; }
        BpfQueuePoint("FOLLOW", &g_followPath[g_followDumpIndex]);
        if (UdpTelemetryQueueExperimentText(g_text) != 0) g_followDumpIndex++;
    } else if (g_dumpState == BPF_DUMP_RESULT) {
        (void)snprintf(g_text, sizeof(g_text),
            "BPATH event=RESULT mode=FOLLOW target_l=%ld target_r=%ld reverse_l=%ld reverse_r=%ld "
            "error_l=%ld error_r=%ld error_s=%ld max_path_err=%ld follow_elapsed_ms=%u timeout_ms=%u "
            "terminal_arm_ms=%u terminal_schedule_ms=%u terminal_delay_ms=%u "
            "terminal_remaining_s_at_schedule=%ld terminal_vs30=%ld terminal_predicted_coast_s=%ld "
            "terminal_stop_l=%ld terminal_stop_r=%ld terminal_stop_remaining_l=%ld "
            "terminal_stop_remaining_r=%ld terminal_stop_remaining_s=%ld final_l=%ld final_r=%ld "
            "coast_after_stop_l=%ld coast_after_stop_r=%ld coast_after_stop_s=%ld terminal_fallback=%u "
            "valid=%u aborted=%u reason=%s",
            (long)g_result.bLeft, (long)g_result.bRight, (long)g_result.reverseLeft,
            (long)g_result.reverseRight, (long)g_result.errorLeft, (long)g_result.errorRight,
            (long)(g_result.errorLeft + g_result.errorRight), (long)g_result.maxAbsPathErrorTicks,
            (unsigned int)g_result.followElapsedMs, (unsigned int)BPF_REVERSE_TIMEOUT_MS,
            (unsigned int)g_result.terminalArmMs,
            (unsigned int)g_result.terminalScheduleMs, (unsigned int)g_result.terminalDelayMs,
            (long)g_result.terminalRemainingSAtSchedule, (long)g_result.terminalVs30,
            (long)g_result.terminalPredictedCoastS, (long)g_result.terminalStopLeft,
            (long)g_result.terminalStopRight, (long)g_result.terminalStopRemainingLeft,
            (long)g_result.terminalStopRemainingRight, (long)g_result.terminalStopRemainingS,
            (long)-g_result.reverseLeft, (long)-g_result.reverseRight,
            (long)(-g_result.reverseLeft - g_result.terminalStopLeft),
            (long)(-g_result.reverseRight - g_result.terminalStopRight),
            (long)(-g_result.reverseLeft - g_result.terminalStopLeft -
                   g_result.reverseRight - g_result.terminalStopRight),
            (unsigned int)g_result.terminalFallback,
            (unsigned int)g_result.valid, (unsigned int)g_result.aborted,
            g_result.abortReason != NULL ? g_result.abortReason : "NONE");
        if (UdpTelemetryQueueExperimentText(g_text) != 0) g_dumpState = BPF_DUMP_TEXEC;
    } else if (g_dumpState == BPF_DUMP_TEXEC) {
        (void)snprintf(g_text, sizeof(g_text),
            "BPATH event=TEXEC mode=FOLLOW req_ms=%u wait_us=%u stop_ms=%u sched_ms=%u "
            "rem_s=%ld vs30=%ld pred_s=%ld fallback=%u",
            (unsigned int)g_result.terminalDelayMs,
            (unsigned int)g_result.terminalActualWaitUs,
            (unsigned int)g_result.terminalActualStopMs,
            (unsigned int)g_result.terminalScheduleMs,
            (long)g_result.terminalRemainingSAtSchedule,
            (long)g_result.terminalVs30,
            (long)g_result.terminalPredictedCoastS,
            (unsigned int)g_result.terminalFallback);
        if (UdpTelemetryQueueExperimentText(g_text) != 0) g_dumpState = BPF_DUMP_DONE;
    } else if (g_dumpState == BPF_DUMP_DONE) {
        (void)snprintf(g_text, sizeof(g_text),
            "BPATH event=DONE mode=FOLLOW fwd_count=%u follow_count=%u ref_count=%u "
            "fwd_overflow=%u follow_overflow=%u",
            (unsigned int)g_forwardCount, (unsigned int)g_followCount,
            (unsigned int)g_referenceCount, (unsigned int)g_forwardOverflow,
            (unsigned int)g_followOverflow);
        if (UdpTelemetryQueueExperimentText(g_text) != 0) g_dumpState = BPF_DUMP_COMPLETE;
    }
}

void BPathFollowInit(void)
{
    g_state = BPF_WAIT; g_sync = BPF_SYNC; g_dumpState = BPF_DUMP_IDLE;
    g_result = (BpfResult){0}; g_forwardCount = g_followCount = g_referenceCount = 0U;
    g_forwardSerialAssignedCount = 0U;
    g_forwardDumpIndex = g_followDumpIndex = g_referenceIndex = 0U;
    g_forwardOverflow = g_followOverflow = 0U;
    g_stateMs = g_followStartMs = g_readyBase = g_lastArm = g_lastSummary = 0U;
    g_forwardPathStartMs = g_reversePathStartMs = 0U;
    g_forwardLastValid = g_reverseLastValid = 0U;
    g_leftDone = g_rightDone = g_leftPublished = g_rightPublished = 0U;
    g_terminalArmed = g_terminalScheduled = g_terminalStopped = 0U;
    g_terminalExecutionRequestPending = 0U;
    g_terminalVelocityValid = g_terminalVelocitySamples = 0U;
    g_terminalScheduledAtMs = 0U;
    g_terminalPreviousValid = 0U; g_terminalPreviousSeq = 0U;
    g_terminalPreviousLeftTravel = g_terminalPreviousRightTravel = 0;
    g_terminalVs30Now = g_terminalVs30Previous = 0;
    g_terminalVL30 = g_terminalVR30 = 0;
    g_terminalVL30Previous = g_terminalVR30Previous = 0;
    g_terminalEvalVL30 = g_terminalEvalVR30 = 0;
    g_terminalPredictedCoastL = g_terminalPredictedCoastR = 0;
    g_terminalResidualL = g_terminalResidualR = 0;
    g_terminalTimeL = g_terminalTimeR = 0;
    g_terminalVector = BPF_TVEC_NONE;
    g_trimPulseStartMs = 0U; g_trimSide = 0U;
    g_trimPreLeft = g_trimPreRight = g_trimPreError = 0;
    g_manualStartMs = g_manualLastMoveMs = g_manualLastLogMs = 0U;
    g_manualStartLeft = g_manualStartRight = 0;
    g_manualStartRawLeft = g_manualStartRawRight = 0;
    g_manualLastRawLeft = g_manualLastRawRight = 0;
    g_manualLastLogDL = g_manualLastLogDR = 0;
    g_manualMovementStarted = 0U;
    g_pdiffStartL = g_pdiffStartR = g_pdiffStartDiff = g_pdiffLastGain = 0;
    g_pdiffVNow = g_pdiffVPrev = 0;
    g_pdiffVSamples = g_pdiffVelocityValid = g_pdiffValid = 0U;
    g_pdiffStartMs = g_pdiffStopMs = g_pdiffLastLogMs = 0U;
    g_pdiffPrevValid = 0U; g_pdiffPrevSeq = 0U;
    g_resStartMs = g_resLastMoveMs = g_resLastLogMs = 0U;
    g_resStartL = g_resStartR = g_resStartRawL = g_resStartRawR = 0;
    g_resLastRawL = g_resLastRawR = g_resLastLogDl = g_resLastLogDr = 0;
    g_resMovementStarted = 0U;
    g_externalRecordStopMs = 0U;
    g_externalReturnNoPdiff = 0U;
    g_externalIdleRolling = 0U;
    g_externalIdleRollingEver = 0U;
    g_terminalFrozenLeft = g_terminalFrozenRight = 0;
    BpfPublish("BOOT"); BpfPublish("WAIT_ENCODER");
}

int BPathFollowIsFinished(void)
{
    return g_state == BPF_DONE || g_state == BPF_ABORT;
}

int BPathExternalReturnAborted(void)
{
    return g_state == BPF_ABORT;
}

int BPathExternalRecordStart(uint32_t now)
{
    UdpEncoderTelemetryState encoder;
    uint32_t ms = AppTicksToMs(now);

    UdpTelemetryReadEncoder(&encoder);
    if (!BpfFresh(&encoder, ms, BPF_RX_TIMEOUT_MS)) {
        BpfAbort("ENCODER_RX_TIMEOUT");
        return -1;
    }
    g_result.fStartL = g_result.boundaryL = encoder.totalLeft;
    g_result.fStartR = g_result.boundaryR = encoder.totalRight;
    g_result.fStartSeq = g_result.boundarySeq = encoder.sequence;
    g_result.fStartFrames = encoder.validCount;
    g_forwardCount = 0U;
    g_forwardSerialAssignedCount = 0U;
    g_forwardEpoch++;
    if (g_forwardEpoch == 0U) g_forwardEpoch = 1U;
    g_forwardOverflow = 0U;
    g_externalIdleRolling = 0U;
    g_externalIdleRollingEver = 0U;
    g_forwardLastValid = encoder.validCount;
    g_forwardPathStartMs = ms;
    BpfPathAdd(g_forwardPath, &g_forwardCount, &g_forwardOverflow, BPF_FWD_ACTIVE,
               encoder.sequence, 0U, 0, 0, 0, 0, 0, 0U, 0, 0);
    BpfAssignForwardSourceSerial();
    BpfPublish("TRACE_RECORD_START");
    return 0;
}

int BPathExternalEncoderFresh(uint32_t now)
{
    UdpEncoderTelemetryState encoder;

    UdpTelemetryReadEncoder(&encoder);
    return BpfFresh(&encoder, AppTicksToMs(now), BPF_RX_TIMEOUT_MS);
}

int BPathExternalLastAbortWasEncoderRxTimeout(void)
{
    return g_state == BPF_ABORT && g_result.abortReason != NULL &&
        strcmp(g_result.abortReason, "ENCODER_RX_TIMEOUT") == 0;
}

void BPathExternalInvalidateIdleHistory(void)
{
    /* The rolling tail is optional until an Eleven candidate freezes it.
     * Discard invalid source indices rather than allowing a later fork return
     * to consume encoder data across a receive gap. */
    BPathFollowInit();
}

void BPathExternalSetIdleRolling(uint8_t enabled, const char *reason)
{
    uint8_t next = enabled != 0U ? 1U : 0U;

    if (next == g_externalIdleRolling) {
        return;
    }
    g_externalIdleRolling = next;
    if (next != 0U) {
        (void)snprintf(g_text, sizeof(g_text),
                       "BPATH event=IDLE_ROLL_%s keep_points=%u reason=%s",
                       g_externalIdleRollingEver == 0U ? "START" : "RESUME",
                       (unsigned int)BPF_IDLE_TAIL_POINTS,
                       reason == NULL ? "NONE" : reason);
        g_externalIdleRollingEver = 1U;
    } else {
        (void)snprintf(g_text, sizeof(g_text),
                       "BPATH event=FORK_PRESERVE_START reason=%s points=%u",
                       reason == NULL ? "NONE" : reason,
                       (unsigned int)g_forwardCount);
    }
    BpfPublish(g_text);
}

int BPathExternalRecordStep(uint32_t now)
{
    UdpEncoderTelemetryState encoder;
    uint32_t ms = AppTicksToMs(now);

    UdpTelemetryReadEncoder(&encoder);
    if (!BpfFresh(&encoder, ms, BPF_RX_TIMEOUT_MS)) {
        BpfAbort("ENCODER_RX_TIMEOUT");
        return -1;
    }
    if (encoder.validCount == g_forwardLastValid) {
        return 0;
    }
    if (g_externalIdleRolling != 0U) {
        BpfCompactIdleForwardTail();
    }
    g_forwardLastValid = encoder.validCount;
    BpfPathAdd(g_forwardPath, &g_forwardCount, &g_forwardOverflow,
               g_externalRecordStopMs != 0U ? BPF_FWD_COAST : BPF_FWD_ACTIVE,
               encoder.sequence, ms - g_forwardPathStartMs,
               encoder.totalLeft - g_result.fStartL,
               encoder.totalRight - g_result.fStartR, 0, 0, 0, 0U, 0, 0);
    BpfAssignForwardSourceSerial();
    if (g_forwardOverflow != 0U) {
        BpfAbort("FORWARD_OVERFLOW");
        return -1;
    }
    return 0;
}

void BPathExternalRecordStop(uint32_t now)
{
    g_externalIdleRolling = 0U;
    g_externalRecordStopMs = AppTicksToMs(now);
    BpfPublish("TRACE_RECORD_STOP");
}

int BPathExternalHasForwardMovement(void)
{
    uint16_t index;

    for (index = 0U; index < g_forwardCount; index++) {
        if (g_forwardPath[index].left != 0 || g_forwardPath[index].right != 0) {
            return 1;
        }
    }
    return 0;
}

void BPathExternalGetForwardRecordProgress(uint16_t *count, uint16_t *capacity)
{
    if (count != NULL) {
        *count = g_forwardCount;
    }
    if (capacity != NULL) {
        *capacity = BPF_MAX_POINTS;
    }
}

static void BpfForwardMarkerAt(uint16_t index, BPathForwardMarker *marker)
{
    marker->physicalIndex = index;
    marker->sourceSerial = g_forwardBaseSourceSerial + index;
    marker->epoch = g_forwardEpoch;
    marker->encoderLeft = g_result.fStartL + g_forwardPath[index].left;
    marker->encoderRight = g_result.fStartR + g_forwardPath[index].right;
}

int BPathExternalGetForwardRecordMarker(BPathForwardMarker *marker)
{
    if (marker == NULL || g_forwardCount == 0U) return -1;
    BpfForwardMarkerAt((uint16_t)(g_forwardCount - 1U), marker);
    return 0;
}

int BPathExternalGetForwardBaseMarker(BPathForwardMarker *marker)
{
    if (marker == NULL || g_forwardCount == 0U) return -1;
    BpfForwardMarkerAt(0U, marker);
    return 0;
}

int BPathExternalGetForwardMarkerBySerial(uint32_t sourceSerial,
                                          BPathForwardMarker *marker)
{
    uint16_t index;

    if (marker == NULL || sourceSerial < g_forwardBaseSourceSerial) return -1;
    index = (uint16_t)(sourceSerial - g_forwardBaseSourceSerial);
    if ((uint32_t)index != sourceSerial - g_forwardBaseSourceSerial ||
        index >= g_forwardCount) return -1;
    BpfForwardMarkerAt(index, marker);
    return 0;
}

int BPathExternalGetForwardRecordIndex(uint16_t *index)
{
    if (index == NULL || g_forwardCount == 0U) return -1;
    *index = g_forwardCount - 1U;
    return 0;
}

int BPathExternalGetForwardPointTravel(uint16_t index, int32_t *left, int32_t *right)
{
    if (left == NULL || right == NULL || index >= g_forwardCount) {
        return -1;
    }
    *left = g_forwardPath[index].left;
    *right = g_forwardPath[index].right;
    return 0;
}

int BPathExternalGetReferenceMapDiagnostics(uint16_t sourceIndex,
                                             BPathReferenceMapDiagnostics *diagnostics)
{
    uint16_t index;

    if (diagnostics == NULL) {
        return -1;
    }
    diagnostics->forwardPoints = g_forwardCount;
    diagnostics->referencePoints = g_referenceCount;
    diagnostics->firstReferenceSourceIndex = 0xffffU;
    diagnostics->lastReferenceSourceIndex = 0xffffU;
    diagnostics->nearestBeforeSourceIndex = 0xffffU;
    diagnostics->nearestBeforeReferenceIndex = 0xffffU;
    diagnostics->nearestAfterSourceIndex = 0xffffU;
    diagnostics->nearestAfterReferenceIndex = 0xffffU;
    if (sourceIndex >= g_forwardCount || g_referenceCount == 0U) {
        return -1;
    }
    diagnostics->firstReferenceSourceIndex = g_referenceForwardIndex[0U];
    diagnostics->lastReferenceSourceIndex =
        g_referenceForwardIndex[g_referenceCount - 1U];
    /* Sources descend as reverse reference indexes ascend.  "Before" is the
     * safe, already-passed side of a forward spatial mark. */
    for (index = 0U; index < g_referenceCount; index++) {
        uint16_t referenceSource = g_referenceForwardIndex[index];
        if (referenceSource <= sourceIndex &&
            diagnostics->nearestBeforeReferenceIndex == 0xffffU) {
            diagnostics->nearestBeforeSourceIndex = referenceSource;
            diagnostics->nearestBeforeReferenceIndex = index;
        }
        if (referenceSource > sourceIndex) {
            diagnostics->nearestAfterSourceIndex = referenceSource;
            diagnostics->nearestAfterReferenceIndex = index;
        }
    }
    return 0;
}

int BPathExternalMapForwardIndexToReference(uint16_t forwardIndex, uint16_t *referenceIndex)
{
    uint16_t index;

    if (referenceIndex == NULL || forwardIndex >= g_forwardCount || g_referenceCount == 0U) {
        return -1;
    }
    /* Reference index starts at the forward endpoint and increases while reversing. */
    for (index = 0U; index < g_referenceCount; index++) {
        if (g_referenceForwardIndex[index] <= forwardIndex) {
            *referenceIndex = index;
            return 0;
        }
    }
    /* BpfBuildReference intentionally coalesces repeated zero-travel source
     * samples.  If that removes source 0, the final reference is still the
     * physical path origin and is the only safe clamp for sample backoff. */
    if (forwardIndex == 0U &&
        g_forwardPath[g_referenceForwardIndex[g_referenceCount - 1U]].left ==
            g_forwardPath[0U].left &&
        g_forwardPath[g_referenceForwardIndex[g_referenceCount - 1U]].right ==
            g_forwardPath[0U].right) {
        *referenceIndex = g_referenceCount - 1U;
        return 0;
    }
    return -1;
}

int BPathExternalMapForwardSourceSerialToReference(uint32_t sourceSerial,
                                                    uint16_t *referenceIndex,
                                                    BPathForwardMarker *mappedMarker)
{
    BPathForwardMarker exactMarker;
    uint16_t reference;
    uint16_t mappedSource;

    if (referenceIndex == NULL || mappedMarker == NULL ||
        BPathExternalGetForwardMarkerBySerial(sourceSerial, &exactMarker) != 0 ||
        BPathExternalMapForwardIndexToReference(exactMarker.physicalIndex, &reference) != 0 ||
        reference >= g_referenceCount) {
        return -1;
    }
    mappedSource = g_referenceForwardIndex[reference];
    if (mappedSource >= g_forwardCount) return -1;
    BpfForwardMarkerAt(mappedSource, mappedMarker);
    *referenceIndex = reference;
    return 0;
}

int BPathExternalGetReferenceMarker(uint16_t referenceIndex,
                                    BPathForwardMarker *marker)
{
    uint16_t sourceIndex;

    if (marker == NULL || referenceIndex >= g_referenceCount) return -1;
    sourceIndex = g_referenceForwardIndex[referenceIndex];
    if (sourceIndex >= g_forwardCount) return -1;
    BpfForwardMarkerAt(sourceIndex, marker);
    return 0;
}

int BPathExternalGetReturnReferenceCursor(uint16_t *referenceIndex)
{
    if (referenceIndex == NULL || g_referenceCount == 0U) return -1;
    *referenceIndex = g_referenceIndex;
    return 0;
}

void BPathExternalAbort(const char *reason)
{
    BpfAbort(reason);
}

int BPathExternalReturnSettleComplete(uint32_t now)
{
    return g_externalRecordStopMs != 0U &&
           (uint32_t)(AppTicksToMs(now) - g_externalRecordStopMs) >=
           BPF_FORWARD_SETTLE_MS;
}

int BPathExternalRecordFinish(uint32_t now)
{
    UdpEncoderTelemetryState encoder;
    uint32_t ms = AppTicksToMs(now);

    if (BPathExternalRecordStep(now) != 0) {
        return -1;
    }
    UdpTelemetryReadEncoder(&encoder);
    g_result.fEndL = encoder.totalLeft;
    g_result.fEndR = encoder.totalRight;
    g_result.fEndSeq = encoder.sequence;
    g_result.fEndFrames = encoder.validCount;
    g_result.bLeft = encoder.totalLeft - g_result.fStartL;
    g_result.bRight = encoder.totalRight - g_result.fStartR;
    g_result.fFrames = encoder.validCount - g_result.fStartFrames;
    g_result.fDurationMs = ms - g_forwardPathStartMs;
    if (g_result.bLeft <= 0 || g_result.bRight <= 0) {
        BpfAbort("FORWARD_DIRECTION_INVALID");
        return -1;
    }
    BpfFinalForward(ms);
    if (g_forwardOverflow != 0U || BpfBuildReference() == 0) {
        g_result.referenceInvalid = 1U;
        BpfAbort(g_forwardOverflow != 0U ? "FORWARD_OVERFLOW" : "REF_NON_MONOTONIC");
        return -1;
    }
    (void)snprintf(g_text, sizeof(g_text),
        "BPATH event=TRACE_REF_READY points=%u ref=%u tgt_l=%ld tgt_r=%ld",
        (unsigned int)g_forwardCount, (unsigned int)g_referenceCount,
        (long)g_result.bLeft, (long)g_result.bRight);
    (void)UdpTelemetryQueueExperimentText(g_text);
    return 0;
}

int BPathExternalReturnStart(uint32_t now)
{
    if (g_referenceCount < 2U || g_state == BPF_ABORT) {
        return -1;
    }
    g_externalReturnNoPdiff = 1U;
    g_state = BPF_REVERSE_PAUSE;
    g_stateMs = AppTicksToMs(now);
    return 0;
}

void BPathFollowStep(uint32_t now, int *leftCmd, int *rightCmd)
{
    UdpEncoderTelemetryState encoder;
    uint32_t ms = AppTicksToMs(now);
    int32_t revLeft, revRight, leftTravel, rightTravel, currentS;
    int32_t desiredLeft, desiredRight, pathError, leftRemaining, rightRemaining;
    int32_t tangentLeft, tangentRight, remainingS;
    uint16_t referenceIndex;
    int commandLeft, commandRight;

    if (leftCmd == NULL || rightCmd == NULL) return;
    *leftCmd = 0; *rightCmd = 0; UdpTelemetryReadEncoder(&encoder);
    if ((g_state == BPF_FORWARD_A || g_state == BPF_FORWARD_B || g_state == BPF_REVERSE) &&
        !BpfFresh(&encoder, ms, BPF_RX_TIMEOUT_MS)) { BpfAbort("ENCODER_RX_TIMEOUT"); return; }

    switch (g_state) {
        case BPF_WAIT:
            if (g_readyBase == 0U) g_readyBase = encoder.validCount;
            if (encoder.validCount - g_readyBase >= BPF_READY_FRAMES &&
                BpfFresh(&encoder, ms, BPF_READY_AGE_MS)) {
                g_state = BPF_ARM; g_stateMs = g_lastArm = ms; BpfPublish("ENCODER_READY");
            }
            break;
        case BPF_ARM:
            if ((uint32_t)(ms - g_stateMs) >= BPF_ARM_MS) {
                g_state = BPF_FORWARD_A; g_stateMs = ms;
                g_result.fStartL = encoder.totalLeft; g_result.fStartR = encoder.totalRight;
                g_result.fStartSeq = encoder.sequence; g_result.fStartFrames = encoder.validCount;
                g_result.fDurationMs = 2U * BPF_SEGMENT_MS; BpfPublish("FORWARD_A_START");
                *leftCmd = BPF_A_LEFT; *rightCmd = BPF_A_RIGHT;
            }
            break;
        case BPF_FORWARD_A:
            if ((uint32_t)(ms - g_stateMs) >= BPF_SEGMENT_MS) {
                g_result.boundaryL = encoder.totalLeft; g_result.boundaryR = encoder.totalRight;
                g_result.boundarySeq = encoder.sequence; g_state = BPF_FORWARD_B; g_stateMs = ms;
                BpfStartForwardPath(&encoder, ms); BpfPublish("FORWARD_AB_SWITCH");
                *leftCmd = BPF_B_LEFT; *rightCmd = BPF_B_RIGHT;
            } else { *leftCmd = BPF_A_LEFT; *rightCmd = BPF_A_RIGHT; }
            break;
        case BPF_FORWARD_B:
            if ((uint32_t)(ms - g_stateMs) >= BPF_SEGMENT_MS) {
                g_state = BPF_FORWARD_SETTLE; g_stateMs = ms; BpfPublish("FORWARD_B_STOP");
            } else {
                BpfRecordForward(&encoder, ms, BPF_FWD_ACTIVE, BPF_B_LEFT, BPF_B_RIGHT);
                *leftCmd = BPF_B_LEFT; *rightCmd = BPF_B_RIGHT;
            }
            break;
        case BPF_FORWARD_SETTLE:
            BpfRecordForward(&encoder, ms, BPF_FWD_COAST, 0, 0);
            if ((uint32_t)(ms - g_stateMs) >= BPF_FORWARD_SETTLE_MS) {
                g_result.fEndL = encoder.totalLeft; g_result.fEndR = encoder.totalRight;
                g_result.fEndSeq = encoder.sequence; g_result.fEndFrames = encoder.validCount;
                g_result.bLeft = encoder.totalLeft - g_result.boundaryL;
                g_result.bRight = encoder.totalRight - g_result.boundaryR;
                g_result.fFrames = encoder.validCount - g_result.fStartFrames;
                if (g_result.bLeft <= 0 || g_result.bRight <= 0) {
                    BpfAbort("FORWARD_DIRECTION_INVALID"); break;
                }
                BpfFinalForward(ms);
                if (BpfBuildReference() == 0) {
                    g_result.referenceInvalid = 1U; BpfAbort("REF_NON_MONOTONIC"); break;
                }
                g_state = BPF_REVERSE_PAUSE; g_stateMs = ms; BpfPublish("FORWARD_DONE");
            }
            break;
        case BPF_REVERSE_PAUSE:
            if ((uint32_t)(ms - g_stateMs) >= BPF_REVERSE_PAUSE_MS) {
                g_state = BPF_REVERSE; g_stateMs = ms;
                g_result.rStartL = encoder.totalLeft; g_result.rStartR = encoder.totalRight;
                g_result.rStartSeq = encoder.sequence; g_result.rStartFrames = encoder.validCount;
                g_sync = BPF_SYNC; g_referenceIndex = 0U;
                g_leftDone = g_rightDone = g_leftPublished = g_rightPublished = 0U;
                g_terminalArmed = g_terminalScheduled = g_terminalStopped = 0U;
                g_terminalExecutionRequestPending = 0U;
                g_terminalVelocityValid = g_terminalVelocitySamples = 0U;
                g_terminalScheduledAtMs = 0U;
                g_terminalPreviousValid = encoder.validCount;
                g_terminalPreviousSeq = encoder.sequence;
                g_terminalPreviousLeftTravel = g_terminalPreviousRightTravel = 0;
                g_terminalVs30Now = g_terminalVs30Previous = 0;
                g_terminalVL30 = g_terminalVR30 = 0;
                g_terminalVL30Previous = g_terminalVR30Previous = 0;
                g_terminalEvalVL30 = g_terminalEvalVR30 = 0;
                g_terminalPredictedCoastL = g_terminalPredictedCoastR = 0;
                g_terminalResidualL = g_terminalResidualR = 0;
                g_terminalTimeL = g_terminalTimeR = 0;
                g_terminalVector = BPF_TVEC_NONE;
                g_terminalFrozenLeft = g_terminalFrozenRight = 0;
                BpfStartFollowPath(&encoder, ms); BpfPublish("REVERSE_FOLLOW_START");
            }
            break;
        case BPF_REVERSE:
            revLeft = encoder.totalLeft - g_result.rStartL;
            revRight = encoder.totalRight - g_result.rStartR;
            leftTravel = -revLeft; rightTravel = -revRight;
            /* Signed deltas are telemetry only: backward travel is negative. */
            g_result.reverseLeft = revLeft;
            g_result.reverseRight = revRight;
            g_result.errorLeft = g_result.bLeft + revLeft;
            g_result.errorRight = g_result.bRight + revRight;
            g_result.followElapsedMs = ms - g_followStartMs;
            if (revLeft > BPF_DIRECTION_WRONG || revRight > BPF_DIRECTION_WRONG) {
                BpfAbort("ENCODER_DIRECTION_WRONG"); break;
            }
            if ((uint32_t)(ms - g_stateMs) >= BPF_REVERSE_TIMEOUT_MS &&
                !g_terminalStopped) { BpfAbort("REVERSE_TIMEOUT"); break; }
            leftRemaining = g_result.bLeft - leftTravel;
            rightRemaining = g_result.bRight - rightTravel;
            currentS = leftTravel + rightTravel;
            BpfUpdateTerminalVelocity(&encoder, leftTravel, rightTravel);
            BpfDesiredAt(currentS, &desiredLeft, &desiredRight, &referenceIndex);
            pathError = rightTravel - desiredRight;
            if (BpfAbs(pathError) > g_result.maxAbsPathErrorTicks) {
                g_result.maxAbsPathErrorTicks = BpfAbs(pathError);
            }
            if (g_terminalScheduled) {
                if ((uint32_t)(ms - g_terminalScheduledAtMs) >=
                    g_result.terminalDelayMs) {
                    BpfEnterTerminalStop(ms, leftTravel, rightTravel, leftRemaining,
                                         rightRemaining, g_result.bLeft + g_result.bRight - currentS,
                                         "TERMINAL_STOP");
                    break;
                }
                *leftCmd = g_terminalFrozenLeft; *rightCmd = g_terminalFrozenRight;
                BpfRecordFollow(&encoder, ms, BPF_FOLLOW_ACTIVE, desiredLeft, desiredRight,
                                pathError, referenceIndex, g_terminalFrozenLeft,
                                g_terminalFrozenRight);
                break;
            }
            {
                remainingS = g_result.bLeft + g_result.bRight - currentS;
                int32_t vs30 = BpfTerminalVs30();
                int32_t predictedCoastS = 0;
                if (!g_terminalArmed && remainingS <= BPF_TERMINAL_ARM_S) {
                    g_terminalArmed = 1U; g_result.terminalArmMs = ms - g_stateMs;
                    BpfPublish("TERMINAL_ARMED");
                }
                if (remainingS <= 0) {
                    BpfEnterTerminalStop(ms, leftTravel, rightTravel, leftRemaining,
                                         rightRemaining, remainingS, "TERMINAL_STOP");
                    break;
                }
                if (g_terminalArmed && !g_terminalVelocityValid &&
                    remainingS <= BPF_TERMINAL_FALLBACK_S) {
                    g_result.terminalFallback = 1U;
                    BpfEnterTerminalStop(ms, leftTravel, rightTravel, leftRemaining,
                                         rightRemaining, remainingS, "TERMINAL_FALLBACK_STOP");
                    break;
                }
                if (g_terminalArmed && g_terminalVelocityValid && vs30 > 0) {
                    predictedCoastS = (int32_t)(((int64_t)5 * vs30) / 4);
                    if (predictedCoastS > BPF_PREDICTED_COAST_MAX) {
                        predictedCoastS = BPF_PREDICTED_COAST_MAX;
                    }
                    g_result.terminalVs30 = vs30;
                    g_result.terminalPredictedCoastS = predictedCoastS;
                }
            }
            if (g_sync == BPF_SYNC) {
                if (pathError > BPF_PATH_ERROR_ENTER) {
                    g_sync = BPF_RIGHT_AHEAD; g_result.rightAheadEnterCount++; BpfPublish("RIGHT_AHEAD");
                } else if (pathError < -BPF_PATH_ERROR_ENTER) {
                    g_sync = BPF_RIGHT_BEHIND; g_result.rightBehindEnterCount++; BpfPublish("RIGHT_BEHIND");
                }
            } else if (g_sync == BPF_RIGHT_AHEAD && pathError <= BPF_PATH_ERROR_RELEASE) {
                g_sync = BPF_SYNC; BpfPublish("PATH_SYNC");
            } else if (g_sync == BPF_RIGHT_BEHIND && pathError >= -BPF_PATH_ERROR_RELEASE) {
                g_sync = BPF_SYNC; BpfPublish("PATH_SYNC");
            }
            tangentLeft = (referenceIndex + 1U < g_referenceCount) ?
                          g_reference[referenceIndex + 1U].left - g_reference[referenceIndex].left : 0;
            tangentRight = (referenceIndex + 1U < g_referenceCount) ?
                           g_reference[referenceIndex + 1U].right - g_reference[referenceIndex].right : 0;
            if (tangentRight == 0 && tangentLeft > 0) { commandLeft = -100; commandRight = 0; }
            else if (tangentLeft == 0 && tangentRight > 0) { commandLeft = 0; commandRight = -100; }
            else { commandLeft = -100; commandRight = -100; }
            if (g_sync == BPF_RIGHT_AHEAD) { commandLeft = -100; commandRight = 0; }
            else if (g_sync == BPF_RIGHT_BEHIND) { commandLeft = 0; commandRight = -100; }

            if (g_terminalArmed && g_terminalVelocityValid) {
                int32_t vL = BpfWheelVelocity(g_terminalVL30, g_terminalVL30Previous,
                                              g_terminalVelocitySamples);
                int32_t vR = BpfWheelVelocity(g_terminalVR30, g_terminalVR30Previous,
                                              g_terminalVelocitySamples);
                int32_t coastL = (BPF_WHEEL_COAST_NUM * vL) / BPF_WHEEL_COAST_DEN;
                int32_t coastR = (BPF_WHEEL_COAST_NUM * vR) / BPF_WHEEL_COAST_DEN;
                int64_t timeL, timeR;
                BpfTerminalVector vector = BPF_TVEC_BOTH;
                g_terminalEvalVL30 = vL; g_terminalEvalVR30 = vR;
                if (coastL > BPF_WHEEL_COAST_MAX) coastL = BPF_WHEEL_COAST_MAX;
                if (coastR > BPF_WHEEL_COAST_MAX) coastR = BPF_WHEEL_COAST_MAX;
                g_terminalPredictedCoastL = coastL; g_terminalPredictedCoastR = coastR;
                g_terminalResidualL = leftRemaining - coastL;
                g_terminalResidualR = rightRemaining - coastR;
                if (leftRemaining <= -BPF_TERMINAL_OVERSHOOT_SAFETY ||
                    rightRemaining <= -BPF_TERMINAL_OVERSHOOT_SAFETY ||
                    (g_terminalResidualL <= 0 && g_terminalResidualR <= 0)) {
                    BpfPublishTerminalVector(BPF_TVEC_BOTH);
                    BpfEnterTerminalStop(ms, leftTravel, rightTravel, leftRemaining,
                                         rightRemaining, leftRemaining + rightRemaining,
                                         "TERMINAL_STOP");
                    break;
                }
                timeL = g_terminalResidualL > 0 && vL > 0 ?
                    ((int64_t)g_terminalResidualL * 30) / vL :
                    (g_terminalResidualL > 0 ? INT32_MAX / 4 : 0);
                timeR = g_terminalResidualR > 0 && vR > 0 ?
                    ((int64_t)g_terminalResidualR * 30) / vR :
                    (g_terminalResidualR > 0 ? INT32_MAX / 4 : 0);
                g_terminalTimeL = (int32_t)(timeL > INT32_MAX ? INT32_MAX : timeL);
                g_terminalTimeR = (int32_t)(timeR > INT32_MAX ? INT32_MAX : timeR);
                if (g_terminalResidualL <= 0 && g_terminalResidualR > 0) {
                    vector = BPF_TVEC_RIGHT_ONLY; commandLeft = 0; commandRight = -100;
                } else if (g_terminalResidualR <= 0 && g_terminalResidualL > 0) {
                    vector = BPF_TVEC_LEFT_ONLY; commandLeft = -100; commandRight = 0;
                } else if (timeL > timeR + BPF_TERMINAL_TIME_DIFF_TOLERANCE_MS) {
                    vector = BPF_TVEC_LEFT_ONLY; commandLeft = -100; commandRight = 0;
                } else if (timeR > timeL + BPF_TERMINAL_TIME_DIFF_TOLERANCE_MS) {
                    vector = BPF_TVEC_RIGHT_ONLY; commandLeft = 0; commandRight = -100;
                }
                BpfPublishTerminalVector(vector);
                if (vector == BPF_TVEC_BOTH && commandLeft == -100 && commandRight == -100 &&
                    timeL <= 30 && timeR <= 30) {
                    uint32_t delayMs = (uint32_t)((timeL + timeR + 1) / 2);
                    if (delayMs < 1U) delayMs = 1U;
                    if (delayMs > 30U) delayMs = 30U;
                    g_terminalScheduled = 1U; g_terminalScheduledAtMs = ms;
                    g_result.terminalScheduleMs = ms - g_stateMs;
                    g_result.terminalDelayMs = delayMs;
                    g_result.terminalRemainingSAtSchedule = remainingS;
                    g_result.scheduleRemainingL = leftRemaining; g_result.scheduleRemainingR = rightRemaining;
                    g_result.scheduleVL30 = g_terminalEvalVL30; g_result.scheduleVR30 = g_terminalEvalVR30;
                    g_result.schedulePredictedCoastL = coastL; g_result.schedulePredictedCoastR = coastR;
                    g_result.scheduleResidualL = g_terminalResidualL; g_result.scheduleResidualR = g_terminalResidualR;
                    g_result.scheduleTimeL = g_terminalTimeL; g_result.scheduleTimeR = g_terminalTimeR;
                    {
                        char text[220];
                        (void)snprintf(text, sizeof(text),
                            "BPATH event=VSTOP rem_l=%ld rem_r=%ld vl=%ld vr=%ld pc_l=%ld pc_r=%ld res_l=%ld res_r=%ld tl=%ld tr=%ld delay=%u",
                            (long)leftRemaining, (long)rightRemaining, (long)g_terminalEvalVL30,
                            (long)g_terminalEvalVR30, (long)coastL, (long)coastR,
                            (long)g_terminalResidualL, (long)g_terminalResidualR,
                            (long)g_terminalTimeL, (long)g_terminalTimeR, (unsigned int)delayMs);
                        (void)UdpTelemetryQueueExperimentText(text);
                    }
                }
            }
            *leftCmd = commandLeft; *rightCmd = commandRight;
            if (g_terminalScheduled) {
                g_terminalFrozenLeft = commandLeft; g_terminalFrozenRight = commandRight;
                g_terminalExecutionRequestPending = 1U;
                BpfPublish("TERMINAL_SCHEDULED");
            }
            BpfRecordFollow(&encoder, ms, BPF_FOLLOW_ACTIVE, desiredLeft, desiredRight,
                            pathError, referenceIndex, commandLeft, commandRight);
            break;
        case BPF_REVERSE_SETTLE:
            BpfRecordFollow(&encoder, ms, BPF_FOLLOW_COAST, g_result.bLeft, g_result.bRight,
                            0, g_referenceCount - 1U, 0, 0);
            if ((uint32_t)(ms - g_stateMs) >= BPF_REVERSE_SETTLE_MS) {
                g_result.rEndL = encoder.totalLeft; g_result.rEndR = encoder.totalRight;
                g_result.rEndSeq = encoder.sequence; g_result.rEndFrames = encoder.validCount;
                g_result.reverseLeft = encoder.totalLeft - g_result.rStartL;
                g_result.reverseRight = encoder.totalRight - g_result.rStartR;
                g_result.rFrames = encoder.validCount - g_result.rStartFrames;
                g_result.autoErrorLeft = g_result.bLeft + g_result.reverseLeft;
                g_result.autoErrorRight = g_result.bRight + g_result.reverseRight;
                {
                    char text[140];
                    (void)snprintf(text, sizeof(text),
                        "BPATH event=AUTO_RESULT l=%ld r=%ld errL=%ld errR=%ld diff=%ld",
                        (long)(-g_result.reverseLeft), (long)(-g_result.reverseRight),
                        (long)g_result.autoErrorLeft, (long)g_result.autoErrorRight,
                        (long)(g_result.autoErrorRight - g_result.autoErrorLeft));
                    (void)UdpTelemetryQueueExperimentText(text);
                }
                if (g_externalReturnNoPdiff != 0U) {
                    BpfFinalizeResult(&encoder, ms);
                } else {
                    g_state = BPF_PDIFF_PREP; g_stateMs = ms;
                }
            }
            break;
        case BPF_PDIFF_PREP:
            if (!BpfFresh(&encoder, ms, BPF_PDIFF_AGE_MS)) {
                g_pdiffValid = 0U; BpfPublishPdiff("PDIFF_FINAL", 0, 0, 0);
                BpfFinalizeResult(&encoder, ms); break;
            }
            g_pdiffStartL = -(encoder.totalLeft - g_result.rStartL);
            g_pdiffStartR = -(encoder.totalRight - g_result.rStartR);
            g_pdiffStartDiff = (g_pdiffStartL - g_result.bLeft) -
                               (g_pdiffStartR - g_result.bRight);
            g_result.pdiffStartL = g_pdiffStartL; g_result.pdiffStartR = g_pdiffStartR;
            g_result.pdiffStartDiff = g_pdiffStartDiff;
            {
                char text[180];
                (void)snprintf(text, sizeof(text),
                    "BPATH event=PDIFF_START l=%ld r=%ld exl=%ld exr=%ld diff=%ld target=%d",
                    (long)g_pdiffStartL, (long)g_pdiffStartR,
                    (long)(g_pdiffStartL - g_result.bLeft), (long)(g_pdiffStartR - g_result.bRight),
                    (long)g_pdiffStartDiff, BPF_PHYSICAL_DIFF_TARGET);
                (void)UdpTelemetryQueueExperimentText(text);
            }
            if (g_pdiffStartDiff <= BPF_PHYSICAL_DIFF_TARGET) {
                g_pdiffValid = 1U; BpfPublishPdiff("PDIFF_SKIP", g_pdiffStartDiff, 0, 0);
                BpfFinalizeResult(&encoder, ms); break;
            }
            g_pdiffStartMs = ms; g_pdiffLastLogMs = ms; g_pdiffLastGain = 0;
            g_pdiffPrevValid = encoder.validCount; g_pdiffPrevSeq = encoder.sequence;
            g_pdiffVNow = g_pdiffVPrev = 0; g_pdiffVSamples = 0; g_pdiffVelocityValid = 0;
            g_pdiffValid = 1U; g_state = BPF_PDIFF_DRIVE;
            *leftCmd = 0; *rightCmd = -100;
            break;
        case BPF_PDIFF_DRIVE:
            {
                int32_t currentL = -(encoder.totalLeft - g_result.rStartL);
                int32_t currentR = -(encoder.totalRight - g_result.rStartR);
                int32_t diff = (currentL - g_result.bLeft) - (currentR - g_result.bRight);
                int32_t gain = g_pdiffStartDiff - diff;
                int32_t previousGain = g_pdiffLastGain;
                int32_t rem = diff - BPF_PHYSICAL_DIFF_TARGET;
                int32_t vd = g_pdiffVelocityValid ?
                    (g_pdiffVSamples >= 2U ? (g_pdiffVNow + g_pdiffVPrev) / 2 : g_pdiffVNow) : 0;
                int32_t pc = vd > 0 ? (2 * vd > BPF_PDIFF_COAST_MAX ? BPF_PDIFF_COAST_MAX : 2 * vd) : 0;
                BpfUpdatePdiffVelocity(&encoder, gain, &g_pdiffPrevValid, &g_pdiffPrevSeq,
                                       &g_pdiffLastGain);
                if (previousGain - gain > BPF_PDIFF_WRONG_DIR ||
                    !BpfFresh(&encoder, ms, BPF_PDIFF_AGE_MS) ||
                    (uint32_t)(ms - g_pdiffStartMs) >= BPF_PDIFF_MAX_MS ||
                    rem <= pc) {
                    g_pdiffStopMs = ms; g_result.pdiffStopDiff = diff;
                    if (!BpfFresh(&encoder, ms, BPF_PDIFF_AGE_MS) ||
                        (uint32_t)(ms - g_pdiffStartMs) >= BPF_PDIFF_MAX_MS ||
                        previousGain - gain > BPF_PDIFF_WRONG_DIR) g_pdiffValid = 0U;
                    BpfPublishPdiff("PDIFF_STOP", diff, rem, gain);
                    g_state = BPF_PDIFF_SETTLE; g_stateMs = ms;
                    *leftCmd = 0; *rightCmd = 0;
                } else {
                    *leftCmd = 0; *rightCmd = -100;
                    if ((uint32_t)(ms - g_pdiffLastLogMs) >= 200U) {
                        BpfPublishPdiff("PDIFF", diff, rem, gain); g_pdiffLastLogMs = ms;
                    }
                }
            }
            break;
        case BPF_PDIFF_SETTLE:
            *leftCmd = 0; *rightCmd = 0;
            if ((uint32_t)(ms - g_stateMs) >= BPF_PDIFF_SETTLE_MS) {
                int32_t finalL = -(encoder.totalLeft - g_result.rStartL);
                int32_t finalR = -(encoder.totalRight - g_result.rStartR);
                int32_t finalDiff = (finalL - g_result.bLeft) - (finalR - g_result.bRight);
                int32_t gain = g_pdiffStartDiff - finalDiff;
                int32_t coast = g_result.pdiffStopDiff - finalDiff;
                g_result.pdiffFinalDiff = finalDiff; g_result.pdiffActualGain = gain;
                g_result.pdiffCoastGain = coast; g_result.pdiffV30 = g_pdiffVNow;
                g_result.pdiffPredictedCoast = g_pdiffVNow > 0 ?
                    (2 * g_pdiffVNow > BPF_PDIFF_COAST_MAX ? BPF_PDIFF_COAST_MAX : 2 * g_pdiffVNow) : 0;
                {
                    char text[210];
                    (void)snprintf(text, sizeof(text),
                        "BPATH event=PDIFF_FINAL start=%ld stop=%ld final=%ld gain=%ld coast=%ld target=%d valid=%u reason=%s",
                        (long)g_pdiffStartDiff, (long)g_result.pdiffStopDiff, (long)finalDiff,
                        (long)gain, (long)coast, BPF_PHYSICAL_DIFF_TARGET,
                        (unsigned int)g_pdiffValid, g_pdiffValid ? "DONE" : "INVALID");
                    (void)UdpTelemetryQueueExperimentText(text);
                }
                BpfFinalizeResult(&encoder, ms);
            }
            break;
        case BPF_RESALIGN_PREP:
            if (!BpfFresh(&encoder, ms, BPF_PDIFF_AGE_MS)) {
                g_state = BPF_RESALIGN_RUN; g_resStartMs = ms; g_resLastMoveMs = ms;
                break;
            }
            g_resStartL = -(encoder.totalLeft - g_result.rStartL);
            g_resStartR = -(encoder.totalRight - g_result.rStartR);
            g_resStartRawL = encoder.totalLeft; g_resStartRawR = encoder.totalRight;
            g_resLastRawL = g_resStartRawL; g_resLastRawR = g_resStartRawR;
            g_resStartMs = ms; g_resLastMoveMs = ms; g_resLastLogMs = ms;
            g_resLastLogDl = g_resLastLogDr = 0; g_resMovementStarted = 0U;
            (void)snprintf(g_text, sizeof(g_text),
                "BPATH event=RESALIGN_START l=%ld r=%ld diff=%ld target=%d",
                (long)g_resStartL, (long)g_resStartR,
                (long)((g_resStartL-g_result.bLeft)-(g_resStartR-g_result.bRight)),
                BPF_PHYSICAL_DIFF_TARGET);
            (void)UdpTelemetryQueueExperimentText(g_text);
            g_state = BPF_RESALIGN_RUN;
            break;
        case BPF_RESALIGN_RUN:
            {
                int32_t rawDl = encoder.totalLeft - g_resStartRawL;
                int32_t rawDr = encoder.totalRight - g_resStartRawR;
                int32_t dl = -(rawDl), dr = -(rawDr);
                int32_t diff = (g_resStartL + dl - g_result.bLeft) -
                               (g_resStartR + dr - g_result.bRight);
                *leftCmd = 0; *rightCmd = 0;
                if (!g_resMovementStarted && BpfAbs(rawDl)+BpfAbs(rawDr) >= 5) {
                    g_resMovementStarted = 1U; g_resLastMoveMs = ms;
                }
                if (rawDl != g_resLastRawL-g_resStartRawL || rawDr != g_resLastRawR-g_resStartRawR) g_resLastMoveMs = ms;
                if ((uint32_t)(ms-g_resLastLogMs) >= 200U || BpfAbs(dl-g_resLastLogDl)+BpfAbs(dr-g_resLastLogDr) >= 5) {
                    (void)snprintf(g_text, sizeof(g_text), "BPATH event=RESALIGN dl=%ld dr=%ld gain=%ld diff=%ld",
                        (long)dl, (long)dr, (long)(dr-dl), (long)diff);
                    (void)UdpTelemetryQueueExperimentText(g_text); g_resLastLogMs=ms; g_resLastLogDl=dl; g_resLastLogDr=dr;
                }
                g_resLastRawL=encoder.totalLeft; g_resLastRawR=encoder.totalRight;
                if ((g_resMovementStarted && (uint32_t)(ms-g_resLastMoveMs)>=BPF_RESALIGN_IDLE_MS) ||
                    (uint32_t)(ms-g_resStartMs)>=BPF_RESALIGN_TIMEOUT_MS) {
                    g_result.resStartL=g_resStartL; g_result.resStartR=g_resStartR;
                    g_result.resFinalDiff=diff; g_result.resDeltaL=dl; g_result.resDeltaR=dr; g_result.resGain=dr-dl;
                    g_result.resClean=(BpfAbs(dl)<=5)?1U:0U;
                    (void)snprintf(g_text,sizeof(g_text),"BPATH event=RESALIGN_FINAL start=%ld final=%ld dl=%ld dr=%ld gain=%ld physical_target=%ld clean=%u",
                        (long)((g_resStartL-g_result.bLeft)-(g_resStartR-g_result.bRight)),(long)diff,(long)dl,(long)dr,(long)(dr-dl),(long)diff,(unsigned int)g_result.resClean);
                    (void)UdpTelemetryQueueExperimentText(g_text); BpfFinalizeResult(&encoder,ms);
                }
            }
            break;
        case BPF_MALIGN_PREP:
            if (!BpfFresh(&encoder, ms, BPF_POST_TRIM_MAX_AGE_MS)) {
                BpfPublishTrimSkip("MALIGN_AGE");
                BpfFinalizeResult(&encoder, ms); break;
            }
            g_manualStartLeft = -(encoder.totalLeft - g_result.rStartL);
            g_manualStartRight = -(encoder.totalRight - g_result.rStartR);
            g_manualStartMs = ms; g_manualLastMoveMs = ms; g_manualLastLogMs = ms;
            g_manualLastRawLeft = encoder.totalLeft; g_manualLastRawRight = encoder.totalRight;
            g_manualStartRawLeft = encoder.totalLeft; g_manualStartRawRight = encoder.totalRight;
            g_manualLastLogDL = g_manualLastLogDR = 0;
            g_manualMovementStarted = 0U;
            g_result.manualStartLeft = g_manualStartLeft; g_result.manualStartRight = g_manualStartRight;
            {
                int32_t exL = g_manualStartLeft - g_result.bLeft;
                int32_t exR = g_manualStartRight - g_result.bRight;
                int32_t err = exL - exR;
                char text[200];
                (void)snprintf(text, sizeof(text),
                    "BPATH event=MALIGN_START l=%ld r=%ld ex_l=%ld ex_r=%ld err=%ld raw_l=%ld raw_r=%ld",
                    (long)g_manualStartLeft, (long)g_manualStartRight, (long)exL, (long)exR,
                    (long)err, (long)encoder.totalLeft, (long)encoder.totalRight);
                (void)UdpTelemetryQueueExperimentText(text);
            }
            g_state = BPF_MALIGN_RUN;
            break;
        case BPF_MALIGN_RUN:
            {
                int32_t rawDl = encoder.totalLeft - g_manualLastRawLeft;
                int32_t rawDr = encoder.totalRight - g_manualLastRawRight;
                int32_t dl = -(encoder.totalLeft - g_manualStartRawLeft);
                int32_t dr = -(encoder.totalRight - g_manualStartRawRight);
                int32_t absMove = BpfAbs(encoder.totalLeft - g_manualStartRawLeft) +
                                  BpfAbs(encoder.totalRight - g_manualStartRawRight);
                if (!g_manualMovementStarted && absMove >= BPF_MANUAL_MOVE_START_TICKS) {
                    g_manualMovementStarted = 1U; g_manualLastMoveMs = ms;
                }
                if (rawDl != 0 || rawDr != 0) g_manualLastMoveMs = ms;
                if ((uint32_t)(ms - g_manualLastLogMs) >= BPF_MANUAL_LOG_MS ||
                    BpfAbs(dl - g_manualLastLogDL) + BpfAbs(dr - g_manualLastLogDR) >= 5) {
                    char text[180];
                    (void)snprintf(text, sizeof(text),
                        "BPATH event=MALIGN t=%u dl=%ld dr=%ld raw_dl=%ld raw_dr=%ld",
                        (unsigned int)(ms - g_manualStartMs), (long)dl, (long)dr,
                        (long)(encoder.totalLeft - g_manualStartRawLeft),
                        (long)(encoder.totalRight - g_manualStartRawRight));
                    (void)UdpTelemetryQueueExperimentText(text);
                    g_manualLastLogMs = ms; g_manualLastLogDL = dl; g_manualLastLogDR = dr;
                }
                g_manualLastRawLeft = encoder.totalLeft; g_manualLastRawRight = encoder.totalRight;
                *leftCmd = 0; *rightCmd = 0;
                if ((g_manualMovementStarted && (uint32_t)(ms - g_manualLastMoveMs) >= BPF_MANUAL_IDLE_MS) ||
                    (uint32_t)(ms - g_manualStartMs) >= BPF_MANUAL_TIMEOUT_MS) {
                    g_result.manualFinalLeft = -(encoder.totalLeft - g_result.rStartL);
                    g_result.manualFinalRight = -(encoder.totalRight - g_result.rStartR);
                    g_result.manualDeltaL = g_result.manualFinalLeft - g_manualStartLeft;
                    g_result.manualDeltaR = g_result.manualFinalRight - g_manualStartRight;
                    g_result.manualRawDeltaL = encoder.totalLeft - g_manualStartRawLeft;
                    g_result.manualRawDeltaR = encoder.totalRight - g_manualStartRawRight;
                    g_result.manualDifferential = g_result.manualDeltaR - g_result.manualDeltaL;
                    {
                        char text[210];
                        (void)snprintf(text, sizeof(text),
                            "BPATH event=MALIGN_FINAL startL=%ld startR=%ld finalL=%ld finalR=%ld dl=%ld dr=%ld raw_dl=%ld raw_dr=%ld tgtL=%ld tgtR=%ld autoL=%ld autoR=%ld mdiff=%ld reason=%s",
                            (long)g_manualStartLeft, (long)g_manualStartRight,
                            (long)g_result.manualFinalLeft, (long)g_result.manualFinalRight,
                            (long)g_result.manualDeltaL, (long)g_result.manualDeltaR,
                            (long)g_result.manualRawDeltaL, (long)g_result.manualRawDeltaR,
                            (long)g_result.bLeft, (long)g_result.bRight,
                            (long)g_result.autoErrorLeft, (long)g_result.autoErrorRight,
                            (long)g_result.manualDifferential,
                            (uint32_t)(ms - g_manualStartMs) >= BPF_MANUAL_TIMEOUT_MS ? "TIMEOUT" : "IDLE");
                        (void)UdpTelemetryQueueExperimentText(text);
                    }
                    BpfFinalizeResult(&encoder, ms);
                }
            }
            break;
        case BPF_POSTTRIM_PREP:
            if (!BpfFresh(&encoder, ms, BPF_POST_TRIM_MAX_AGE_MS)) {
                g_trimPreError = 0; BpfPublishTrimSkip("AGE");
                BpfFinalizeResult(&encoder, ms); break;
            }
            g_trimPreLeft = -(encoder.totalLeft - g_result.rStartL);
            g_trimPreRight = -(encoder.totalRight - g_result.rStartR);
            g_trimPreError = (g_result.bLeft + g_trimPreLeft) -
                             (g_result.bRight + g_trimPreRight);
            g_result.trimPreLeft = g_trimPreLeft; g_result.trimPreRight = g_trimPreRight;
            g_result.trimPreLeftExcess = g_trimPreLeft - g_result.bLeft;
            g_result.trimPreRightExcess = g_trimPreRight - g_result.bRight;
            g_result.trimPreError = g_trimPreError;
            if (g_trimPreError > BPF_POST_TRIM_TRIGGER_TICKS) {
                g_trimSide = 1U;
            } else if (g_trimPreError < -BPF_POST_TRIM_TRIGGER_TICKS) {
                g_trimSide = 2U;
            } else {
                g_trimSide = 0U; BpfPublishTrimSkip("TOL");
                BpfFinalizeResult(&encoder, ms); break;
            }
            g_result.trimSide = g_trimSide;
            g_trimPulseStartMs = ms;
            {
                char text[180];
                (void)snprintf(text, sizeof(text),
                    "BPATH event=PTRIM_PRE side=%c l=%ld r=%ld ex_l=%ld ex_r=%ld err=%ld",
                    g_trimSide == 1U ? 'R' : 'L', (long)g_trimPreLeft, (long)g_trimPreRight,
                    (long)g_result.trimPreLeftExcess, (long)g_result.trimPreRightExcess,
                    (long)g_trimPreError);
                (void)UdpTelemetryQueueExperimentText(text);
            }
            g_state = BPF_POSTTRIM_DRIVE;
            *leftCmd = g_trimSide == 1U ? 0 : -100;
            *rightCmd = g_trimSide == 1U ? -100 : 0;
            break;
        case BPF_POSTTRIM_DRIVE:
            if ((uint32_t)(ms - g_trimPulseStartMs) >= BPF_POST_TRIM_PULSE_MS) {
                g_result.trimActualPulseMs = ms - g_trimPulseStartMs;
                g_state = BPF_POSTTRIM_SETTLE; g_stateMs = ms;
                *leftCmd = 0; *rightCmd = 0;
            } else {
                *leftCmd = g_trimSide == 1U ? 0 : -100;
                *rightCmd = g_trimSide == 1U ? -100 : 0;
            }
            break;
        case BPF_POSTTRIM_SETTLE:
            if ((uint32_t)(ms - g_stateMs) >= BPF_POST_TRIM_SETTLE_MS) {
                g_result.trimPostLeft = -(encoder.totalLeft - g_result.rStartL);
                g_result.trimPostRight = -(encoder.totalRight - g_result.rStartR);
                g_result.trimDeltaL = g_result.trimPostLeft - g_trimPreLeft;
                g_result.trimDeltaR = g_result.trimPostRight - g_trimPreRight;
                g_result.trimPostError = (g_result.bLeft + g_result.trimPostLeft) -
                                         (g_result.bRight + g_result.trimPostRight);
                g_result.trimCorrection = BpfAbs(g_trimPreError) - BpfAbs(g_result.trimPostError);
                g_result.trimRelativeGain = g_trimSide == 1U ?
                    g_result.trimDeltaR - g_result.trimDeltaL :
                    g_result.trimDeltaL - g_result.trimDeltaR;
                {
                    char text[200];
                    (void)snprintf(text, sizeof(text),
                        "BPATH event=PTRIM_POST side=%c req_ms=%u act_ms=%u dl=%ld dr=%ld gain=%ld pre=%ld post=%ld",
                        g_trimSide == 1U ? 'R' : 'L', (unsigned int)BPF_POST_TRIM_PULSE_MS,
                        (unsigned int)g_result.trimActualPulseMs, (long)g_result.trimDeltaL,
                        (long)g_result.trimDeltaR, (long)g_result.trimRelativeGain,
                        (long)g_trimPreError, (long)g_result.trimPostError);
                    (void)UdpTelemetryQueueExperimentText(text);
                }
                BpfFinalizeResult(&encoder, ms);
            }
            break;
        case BPF_DONE:
            BpfPumpDump();
            if ((uint32_t)(ms - g_lastSummary) >= 1000U) { g_lastSummary = ms; BpfPublish("SUMMARY"); }
            break;
        case BPF_ABORT:
        default: break;
    }
}
