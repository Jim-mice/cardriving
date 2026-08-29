#include <stdint.h>
#include <stdio.h>

#include "app_time.h"
#include "task_encoder_segment_b_progress.h"
#include "udp_telemetry.h"

#define BPROG_A_LEFT 60
#define BPROG_A_RIGHT 120
#define BPROG_B_LEFT 120
#define BPROG_B_RIGHT 60
#define BPROG_SEGMENT_MS 300U
#define BPROG_FORWARD_SETTLE_MS 800U
#define BPROG_REVERSE_PAUSE_MS 300U
#define BPROG_LEFT_FAST -120
#define BPROG_LEFT_SLOW -100
#define BPROG_RIGHT_RUN -100
#define BPROG_REVERSE_SETTLE_MS 800U
#define BPROG_REVERSE_TIMEOUT_MS 3000U
#define BPROG_READY_FRAMES 20U
#define BPROG_READY_AGE_MS 100U
#define BPROG_RX_TIMEOUT_MS 200U
#define BPROG_LEFT_SLOW_WINDOW 500
#define BPROG_STOP_LEAD 60
#define BPROG_ENTER_BAND 50
#define BPROG_RELEASE_BAND 25
#define BPROG_HARD_AHEAD 150
#define BPROG_DIRECTION_WRONG 50
#define BPROG_ARM_MS 5000U
#define BPROG_ARMING_EVENT_MS 1000U
#define BPROG_PROGRESS_EVENT_MS 180U
#define BPROG_SUMMARY_MS 1000U
#define BPATH_MAX_POINTS 128U

typedef enum { BPATH_FWD_ACTIVE = 0, BPATH_FWD_COAST, BPATH_FWD_FINAL,
               BPATH_REV_ACTIVE, BPATH_REV_COAST, BPATH_REV_FINAL } BPathPhase;
typedef struct {
    uint8_t phase, seq;
    uint16_t index;
    uint32_t timeMs;
    int32_t left, right;
    int16_t commandLeft, commandRight;
} BPathPoint;
typedef enum { BPATH_DUMP_IDLE = 0, BPATH_DUMP_INFO, BPATH_DUMP_FWD,
               BPATH_DUMP_REV, BPATH_DUMP_DONE, BPATH_DUMP_COMPLETE } BPathDumpState;

typedef enum { BP_WAIT = 0, BP_ARM, BP_FORWARD_A, BP_FORWARD_B, BP_FORWARD_SETTLE,
               BP_REVERSE_PAUSE, BP_REVERSE_B, BP_REVERSE_SETTLE, BP_DONE, BP_ABORT } BpState;
typedef enum { BP_SYNC = 0, BP_LEFT_AHEAD, BP_RIGHT_AHEAD } BpSyncState;
typedef struct {
    uint8_t valid, aborted; const char *abortReason;
    int32_t fStartL, fStartR, boundaryL, boundaryR, fEndL, fEndR;
    int32_t aL, aR, bL, bR;
    int32_t rStartL, rStartR, rEndL, rEndR, reverseBL, reverseBR;
    int32_t errorL, errorR, errorLPermille, errorRPermille;
    int32_t maxAbsProgressErrorPermille, maxProgressErrorLeftTravel, maxProgressErrorRightTravel;
    uint32_t fStartMs, rStartMs, rStopMs, fDurationMs, rDurationMs;
    uint32_t fStartFrames, fEndFrames, rStartFrames, rEndFrames, fFrames, rFrames;
    uint32_t leftReachedMs, rightReachedMs;
    uint32_t leftAheadCount, rightAheadCount;
    uint8_t fStartSeq, boundarySeq, fEndSeq, rStartSeq, rEndSeq;
} BpResult;

static BpState g_state;
static BpSyncState g_sync;
static BpResult g_result;
static uint32_t g_stateMs, g_readyBase, g_lastArm, g_lastProgress, g_lastSummary;
static uint8_t g_leftDone, g_rightDone, g_leftPublished, g_rightPublished;
static char g_text[768];
/* Static trajectory storage only.  No BPATH operation performs motor I/O. */
static BPathPoint g_forwardPath[BPATH_MAX_POINTS];
static BPathPoint g_reversePath[BPATH_MAX_POINTS];
static uint16_t g_forwardPathCount, g_reversePathCount, g_forwardPathDumpIndex,
                g_reversePathDumpIndex;
static uint32_t g_forwardPathLastValid, g_reversePathLastValid, g_forwardPathStartMs,
                g_reversePathStartMs;
static uint8_t g_forwardPathOverflow, g_reversePathOverflow;
static BPathDumpState g_pathDumpState;

static void BpPublish(const char *event, uint32_t remaining)
{
    (void)snprintf(g_text, sizeof(g_text),
        "BPROG event=%s a_l=%ld a_r=%ld b_l=%ld b_r=%ld reverse_l=%ld reverse_r=%ld "
        "error_l=%ld error_r=%ld error_l_permille=%ld error_r_permille=%ld "
        "max_p_err=%ld max_l_travel=%ld max_r_travel=%ld left_reached_ms=%u right_reached_ms=%u "
        "left_ahead_count=%u right_ahead_count=%u forward_ms=%u reverse_ms=%u "
        "valid=%u aborted=%u reason=%s left_done=%u right_done=%u remaining_ms=%u",
        event, (long)g_result.aL, (long)g_result.aR, (long)g_result.bL, (long)g_result.bR,
        (long)g_result.reverseBL, (long)g_result.reverseBR, (long)g_result.errorL,
        (long)g_result.errorR, (long)g_result.errorLPermille, (long)g_result.errorRPermille,
        (long)g_result.maxAbsProgressErrorPermille, (long)g_result.maxProgressErrorLeftTravel,
        (long)g_result.maxProgressErrorRightTravel, (unsigned int)g_result.leftReachedMs,
        (unsigned int)g_result.rightReachedMs, (unsigned int)g_result.leftAheadCount,
        (unsigned int)g_result.rightAheadCount, (unsigned int)g_result.fDurationMs,
        (unsigned int)g_result.rDurationMs, (unsigned int)g_result.valid,
        (unsigned int)g_result.aborted, g_result.abortReason ? g_result.abortReason : "NONE",
        (unsigned int)g_leftDone, (unsigned int)g_rightDone, (unsigned int)remaining);
    UdpTelemetryPublishCal(g_text);
}

static void BpProgressPublish(int32_t leftTravel, int32_t rightTravel, int32_t pLeft,
                              int32_t pRight, int32_t pError, int leftCmd, int rightCmd)
{
    const char *state = (g_sync == BP_LEFT_AHEAD) ? "LEFT_AHEAD" :
                        (g_sync == BP_RIGHT_AHEAD) ? "RIGHT_AHEAD" : "SYNC";
    (void)snprintf(g_text, sizeof(g_text),
        "BPROG event=PROGRESS travel_l=%ld travel_r=%ld target_l=%ld target_r=%ld "
        "p_l=%ld p_r=%ld p_err=%ld cmd_l=%d cmd_r=%d sync_state=%s",
        (long)leftTravel, (long)rightTravel, (long)g_result.bL, (long)g_result.bR,
        (long)pLeft, (long)pRight, (long)pError, leftCmd, rightCmd, state);
    UdpTelemetryPublishCal(g_text);
}

static void BpAbort(const char *reason)
{
    if (g_state == BP_ABORT) return;
    g_result.aborted = 1U; g_result.abortReason = reason; g_state = BP_ABORT;
    BpPublish("ABORT", 0U);
}

static int BpFresh(const UdpEncoderTelemetryState *e, uint32_t nowMs, uint32_t age)
{
    return e->validCount != 0U && (uint32_t)(nowMs - e->lastRxMs) <= age;
}

static int32_t BpAbs(int32_t value) { return value < 0 ? -value : value; }

static const char *BpathPhaseName(uint8_t phase)
{
    switch ((BPathPhase)phase) {
        case BPATH_FWD_ACTIVE: case BPATH_REV_ACTIVE: return "ACTIVE";
        case BPATH_FWD_COAST: case BPATH_REV_COAST: return "COAST";
        default: return "FINAL";
    }
}

static void BpathAdd(BPathPoint *path, uint16_t *count, uint8_t *overflow,
                     uint8_t phase, uint8_t seq, uint32_t timeMs, int32_t left,
                     int32_t right, int leftCmd, int rightCmd)
{
    BPathPoint *point;
    if (*count >= BPATH_MAX_POINTS) { *overflow = 1U; return; }
    point = &path[*count]; point->phase = phase; point->seq = seq; point->index = *count;
    point->timeMs = timeMs; point->left = left; point->right = right;
    point->commandLeft = (int16_t)leftCmd; point->commandRight = (int16_t)rightCmd;
    (*count)++;
}

static void BpathStartForward(const UdpEncoderTelemetryState *e, uint32_t ms)
{
    g_forwardPathCount = 0U; g_forwardPathOverflow = 0U;
    g_forwardPathLastValid = e->validCount; g_forwardPathStartMs = ms;
    BpathAdd(g_forwardPath, &g_forwardPathCount, &g_forwardPathOverflow,
             BPATH_FWD_ACTIVE, e->sequence, 0U, 0, 0, BPROG_B_LEFT, BPROG_B_RIGHT);
}

static void BpathRecordForward(const UdpEncoderTelemetryState *e, uint32_t ms,
                               uint8_t phase, int leftCmd, int rightCmd)
{
    if (e->validCount == g_forwardPathLastValid) return;
    g_forwardPathLastValid = e->validCount;
    BpathAdd(g_forwardPath, &g_forwardPathCount, &g_forwardPathOverflow, phase,
             e->sequence, ms - g_forwardPathStartMs,
             e->totalLeft - g_result.boundaryL, e->totalRight - g_result.boundaryR,
             leftCmd, rightCmd);
}

static void BpathFinalForward(uint32_t ms)
{
    BpathAdd(g_forwardPath, &g_forwardPathCount, &g_forwardPathOverflow,
             BPATH_FWD_FINAL, g_result.fEndSeq, ms - g_forwardPathStartMs,
             g_result.bL, g_result.bR, 0, 0);
}

static void BpathStartReverse(const UdpEncoderTelemetryState *e, uint32_t ms)
{
    g_reversePathCount = 0U; g_reversePathOverflow = 0U;
    g_reversePathLastValid = e->validCount; g_reversePathStartMs = ms;
    /* The transition cycle still requests 0/0; the first ACTIVE frame records
     * the actual BPROG command selected on the following control cycle. */
    BpathAdd(g_reversePath, &g_reversePathCount, &g_reversePathOverflow,
             BPATH_REV_ACTIVE, e->sequence, 0U, 0, 0, 0, 0);
}

static void BpathRecordReverse(const UdpEncoderTelemetryState *e, uint32_t ms,
                               uint8_t phase, int leftCmd, int rightCmd)
{
    if (e->validCount == g_reversePathLastValid) return;
    g_reversePathLastValid = e->validCount;
    BpathAdd(g_reversePath, &g_reversePathCount, &g_reversePathOverflow, phase,
             e->sequence, ms - g_reversePathStartMs,
             -(e->totalLeft - g_result.rStartL), -(e->totalRight - g_result.rStartR),
             leftCmd, rightCmd);
}

static void BpathFinalReverse(uint32_t ms)
{
    BpathAdd(g_reversePath, &g_reversePathCount, &g_reversePathOverflow,
             BPATH_REV_FINAL, g_result.rEndSeq, ms - g_reversePathStartMs,
             -g_result.reverseBL, -g_result.reverseBR, 0, 0);
}

static void BpathQueuePoint(const char *pathName, const BPathPoint *point)
{
    (void)snprintf(g_text, sizeof(g_text),
        "BPATH event=POINT path=%s phase=%s idx=%u seq=%u t_ms=%u l=%ld r=%ld cmd_l=%d cmd_r=%d",
        pathName, BpathPhaseName(point->phase), (unsigned int)point->index,
        (unsigned int)point->seq, (unsigned int)point->timeMs, (long)point->left,
        (long)point->right, (int)point->commandLeft, (int)point->commandRight);
}

static void BpathPump(void)
{
    if (g_pathDumpState == BPATH_DUMP_INFO) {
        (void)snprintf(g_text, sizeof(g_text),
            "BPATH event=INFO b_l=%ld b_r=%ld fwd_count=%u rev_count=%u fwd_overflow=%u rev_overflow=%u",
            (long)g_result.bL, (long)g_result.bR, (unsigned int)g_forwardPathCount,
            (unsigned int)g_reversePathCount, (unsigned int)g_forwardPathOverflow,
            (unsigned int)g_reversePathOverflow);
        if (UdpTelemetryQueueExperimentText(g_text) != 0) g_pathDumpState = BPATH_DUMP_FWD;
    } else if (g_pathDumpState == BPATH_DUMP_FWD) {
        if (g_forwardPathDumpIndex >= g_forwardPathCount) { g_pathDumpState = BPATH_DUMP_REV; return; }
        BpathQueuePoint("FWD", &g_forwardPath[g_forwardPathDumpIndex]);
        if (UdpTelemetryQueueExperimentText(g_text) != 0) g_forwardPathDumpIndex++;
    } else if (g_pathDumpState == BPATH_DUMP_REV) {
        if (g_reversePathDumpIndex >= g_reversePathCount) { g_pathDumpState = BPATH_DUMP_DONE; return; }
        BpathQueuePoint("REV", &g_reversePath[g_reversePathDumpIndex]);
        if (UdpTelemetryQueueExperimentText(g_text) != 0) g_reversePathDumpIndex++;
    } else if (g_pathDumpState == BPATH_DUMP_DONE) {
        (void)snprintf(g_text, sizeof(g_text),
            "BPATH event=DONE fwd_count=%u rev_count=%u fwd_overflow=%u rev_overflow=%u",
            (unsigned int)g_forwardPathCount, (unsigned int)g_reversePathCount,
            (unsigned int)g_forwardPathOverflow, (unsigned int)g_reversePathOverflow);
        if (UdpTelemetryQueueExperimentText(g_text) != 0) g_pathDumpState = BPATH_DUMP_COMPLETE;
    }
}

void BProgressInit(void)
{
    g_state = BP_WAIT; g_sync = BP_SYNC; g_result = (BpResult){0};
    g_stateMs = g_readyBase = g_lastArm = g_lastProgress = g_lastSummary = 0U;
    g_leftDone = g_rightDone = g_leftPublished = g_rightPublished = 0U;
    g_forwardPathCount = g_reversePathCount = g_forwardPathDumpIndex = g_reversePathDumpIndex = 0U;
    g_forwardPathOverflow = g_reversePathOverflow = 0U; g_pathDumpState = BPATH_DUMP_IDLE;
    BpPublish("BOOT", 0U); BpPublish("WAIT_ENCODER", 0U);
}

void BProgressStep(uint32_t now, int *leftCmd, int *rightCmd)
{
    UdpEncoderTelemetryState e; uint32_t ms = AppTicksToMs(now);
    int32_t revL, revR, travelL, travelR, pL, pR, pErr, leftRemain, rightRemain;
    int commandL, commandR;
    if (leftCmd == NULL || rightCmd == NULL) return;
    *leftCmd = 0; *rightCmd = 0; UdpTelemetryReadEncoder(&e);
    if ((g_state == BP_FORWARD_A || g_state == BP_FORWARD_B || g_state == BP_REVERSE_B) &&
        !BpFresh(&e, ms, BPROG_RX_TIMEOUT_MS)) { BpAbort("ENCODER_RX_TIMEOUT"); return; }
    switch (g_state) {
        case BP_WAIT:
            if (g_readyBase == 0U) g_readyBase = e.validCount;
            if (e.validCount - g_readyBase >= BPROG_READY_FRAMES && BpFresh(&e, ms, BPROG_READY_AGE_MS)) {
                g_state = BP_ARM; g_stateMs = g_lastArm = ms; BpPublish("ENCODER_READY", BPROG_ARM_MS);
            }
            break;
        case BP_ARM:
            if ((uint32_t)(ms - g_stateMs) >= BPROG_ARM_MS) {
                g_state = BP_FORWARD_A; g_stateMs = ms; g_result.fStartMs = ms;
                g_result.fStartL = e.totalLeft; g_result.fStartR = e.totalRight;
                g_result.fStartSeq = e.sequence; g_result.fStartFrames = e.validCount;
                g_result.fDurationMs = 2U * BPROG_SEGMENT_MS; BpPublish("FORWARD_A_START", 0U);
                *leftCmd = BPROG_A_LEFT; *rightCmd = BPROG_A_RIGHT;
            } else if ((uint32_t)(ms - g_lastArm) >= BPROG_ARMING_EVENT_MS) {
                uint32_t elapsed = ms - g_stateMs; g_lastArm = ms; BpPublish("ARMING", BPROG_ARM_MS - elapsed);
            }
            break;
        case BP_FORWARD_A:
            if ((uint32_t)(ms - g_stateMs) >= BPROG_SEGMENT_MS) {
                g_result.boundaryL = e.totalLeft; g_result.boundaryR = e.totalRight; g_result.boundarySeq = e.sequence;
                g_state = BP_FORWARD_B; g_stateMs = ms; BpPublish("FORWARD_AB_SWITCH", 0U);
                BpathStartForward(&e, ms);
                *leftCmd = BPROG_B_LEFT; *rightCmd = BPROG_B_RIGHT;
            } else { *leftCmd = BPROG_A_LEFT; *rightCmd = BPROG_A_RIGHT; }
            break;
        case BP_FORWARD_B:
            if ((uint32_t)(ms - g_stateMs) >= BPROG_SEGMENT_MS) {
                g_state = BP_FORWARD_SETTLE; g_stateMs = ms; BpPublish("FORWARD_B_STOP", 0U);
            } else {
                BpathRecordForward(&e, ms, BPATH_FWD_ACTIVE, BPROG_B_LEFT, BPROG_B_RIGHT);
                *leftCmd = BPROG_B_LEFT; *rightCmd = BPROG_B_RIGHT;
            }
            break;
        case BP_FORWARD_SETTLE:
            BpathRecordForward(&e, ms, BPATH_FWD_COAST, 0, 0);
            if ((uint32_t)(ms - g_stateMs) >= BPROG_FORWARD_SETTLE_MS) {
                g_result.fEndL = e.totalLeft; g_result.fEndR = e.totalRight; g_result.fEndSeq = e.sequence; g_result.fEndFrames = e.validCount;
                g_result.aL = g_result.boundaryL - g_result.fStartL; g_result.aR = g_result.boundaryR - g_result.fStartR;
                g_result.bL = e.totalLeft - g_result.boundaryL; g_result.bR = e.totalRight - g_result.boundaryR;
                g_result.fFrames = e.validCount - g_result.fStartFrames;
                if (g_result.bL <= 0 || g_result.bR <= 0) { BpAbort("FORWARD_DIRECTION_INVALID"); break; }
                BpathFinalForward(ms);
                g_state = BP_REVERSE_PAUSE; g_stateMs = ms; BpPublish("FORWARD_DONE", 0U);
            }
            break;
        case BP_REVERSE_PAUSE:
            if ((uint32_t)(ms - g_stateMs) >= BPROG_REVERSE_PAUSE_MS) {
                g_state = BP_REVERSE_B; g_stateMs = ms; g_result.rStartMs = ms;
                g_result.rStartL = e.totalLeft; g_result.rStartR = e.totalRight;
                g_result.rStartSeq = e.sequence; g_result.rStartFrames = e.validCount;
                g_sync = BP_SYNC; g_leftDone = g_rightDone = g_leftPublished = g_rightPublished = 0U;
                BpathStartReverse(&e, ms);
                BpPublish("REVERSE_B_START", 0U);
            }
            break;
        case BP_REVERSE_B:
            revL = e.totalLeft - g_result.rStartL; revR = e.totalRight - g_result.rStartR;
            travelL = -revL; travelR = -revR;
            if (revL > BPROG_DIRECTION_WRONG || revR > BPROG_DIRECTION_WRONG) { BpAbort("ENCODER_DIRECTION_WRONG"); break; }
            if ((uint32_t)(ms - g_result.rStartMs) >= BPROG_REVERSE_TIMEOUT_MS && (!g_leftDone || !g_rightDone)) { BpAbort("REVERSE_TIMEOUT"); break; }
            leftRemain = g_result.bL - travelL; rightRemain = g_result.bR - travelR;
            pL = travelL * 1000 / g_result.bL; pR = travelR * 1000 / g_result.bR; pErr = pL - pR;
            if (BpAbs(pErr) > g_result.maxAbsProgressErrorPermille) {
                g_result.maxAbsProgressErrorPermille = BpAbs(pErr);
                g_result.maxProgressErrorLeftTravel = travelL; g_result.maxProgressErrorRightTravel = travelR;
            }
            if (!g_leftDone && leftRemain <= BPROG_STOP_LEAD) {
                g_leftDone = 1U; g_result.leftReachedMs = ms - g_result.rStartMs;
                if (!g_leftPublished) { g_leftPublished = 1U; BpPublish("FINAL_LEFT_REACHED", 0U); }
            }
            if (!g_rightDone && rightRemain <= BPROG_STOP_LEAD) {
                g_rightDone = 1U; g_result.rightReachedMs = ms - g_result.rStartMs;
                if (!g_rightPublished) { g_rightPublished = 1U; BpPublish("FINAL_RIGHT_STOP", 0U); }
            }
            if (g_leftDone && g_rightDone) {
                g_result.rStopMs = ms; g_result.rDurationMs = ms - g_result.rStartMs;
                g_state = BP_REVERSE_SETTLE; g_stateMs = ms; BpPublish("REVERSE_B_STOP", 0U); break;
            }
            if (g_sync == BP_SYNC) {
                if (pErr > BPROG_ENTER_BAND) { g_sync = BP_LEFT_AHEAD; g_result.leftAheadCount++; BpPublish("LEFT_AHEAD", 0U); }
                else if (pErr < -BPROG_ENTER_BAND) { g_sync = BP_RIGHT_AHEAD; g_result.rightAheadCount++; BpPublish("RIGHT_AHEAD", 0U); }
            } else if (BpAbs(pErr) <= BPROG_RELEASE_BAND) {
                g_sync = BP_SYNC; BpPublish("SYNC_RESTORED", 0U);
            } else if (g_sync == BP_LEFT_AHEAD && pErr < -BPROG_ENTER_BAND) {
                g_sync = BP_RIGHT_AHEAD; g_result.rightAheadCount++; BpPublish("RIGHT_AHEAD", 0U);
            } else if (g_sync == BP_RIGHT_AHEAD && pErr > BPROG_ENTER_BAND) {
                g_sync = BP_LEFT_AHEAD; g_result.leftAheadCount++; BpPublish("LEFT_AHEAD", 0U);
            }
            commandL = (leftRemain > BPROG_LEFT_SLOW_WINDOW) ? BPROG_LEFT_FAST : BPROG_LEFT_SLOW;
            commandR = BPROG_RIGHT_RUN;
            if (g_sync == BP_LEFT_AHEAD) commandL = (pErr > BPROG_HARD_AHEAD) ? 0 : BPROG_LEFT_SLOW;
            else if (g_sync == BP_RIGHT_AHEAD) { commandL = (leftRemain > BPROG_LEFT_SLOW_WINDOW) ? BPROG_LEFT_FAST : BPROG_LEFT_SLOW; commandR = 0; }
            if (g_leftDone) {
                commandL = 0;
            }
            if (g_rightDone) {
                commandR = 0;
            }
            *leftCmd = commandL; *rightCmd = commandR;
            BpathRecordReverse(&e, ms, BPATH_REV_ACTIVE, commandL, commandR);
            if ((uint32_t)(ms - g_lastProgress) >= BPROG_PROGRESS_EVENT_MS) {
                g_lastProgress = ms; BpProgressPublish(travelL, travelR, pL, pR, pErr, commandL, commandR);
            }
            break;
        case BP_REVERSE_SETTLE:
            BpathRecordReverse(&e, ms, BPATH_REV_COAST, 0, 0);
            if ((uint32_t)(ms - g_stateMs) >= BPROG_REVERSE_SETTLE_MS) {
                g_result.rEndL = e.totalLeft; g_result.rEndR = e.totalRight; g_result.rEndSeq = e.sequence; g_result.rEndFrames = e.validCount;
                g_result.reverseBL = e.totalLeft - g_result.rStartL; g_result.reverseBR = e.totalRight - g_result.rStartR;
                g_result.rFrames = e.validCount - g_result.rStartFrames;
                g_result.errorL = g_result.bL + g_result.reverseBL; g_result.errorR = g_result.bR + g_result.reverseBR;
                g_result.errorLPermille = g_result.errorL * 1000 / g_result.bL; g_result.errorRPermille = g_result.errorR * 1000 / g_result.bR;
                g_result.valid = 1U; g_state = BP_DONE; g_lastSummary = ms; BpPublish("RESULT", 0U);
                BpathFinalReverse(ms); g_forwardPathDumpIndex = g_reversePathDumpIndex = 0U;
                g_pathDumpState = BPATH_DUMP_INFO;
            }
            break;
        case BP_DONE:
            BpathPump();
            if ((uint32_t)(ms - g_lastSummary) >= BPROG_SUMMARY_MS) { g_lastSummary = ms; BpPublish("SUMMARY", 0U); }
            break;
        case BP_ABORT:
        default: break;
    }
}
