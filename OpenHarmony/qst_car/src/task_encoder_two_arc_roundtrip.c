#include <stdint.h>
#include <stdio.h>

#include "app_time.h"
#include "task_encoder_two_arc_roundtrip.h"
#include "udp_telemetry.h"

#define SARC_A_FORWARD_LEFT_COMMAND      60
#define SARC_A_FORWARD_RIGHT_COMMAND    120
#define SARC_B_FORWARD_LEFT_COMMAND     120
#define SARC_B_FORWARD_RIGHT_COMMAND     60
#define SARC_SEGMENT_MS                 300U
#define SARC_FORWARD_SETTLE_MS          800U
#define SARC_REVERSE_PAUSE_MS           300U
#define SARC_B_REVERSE_LEFT_COMMAND    -120
#define SARC_B_REVERSE_RIGHT_COMMAND    -60
#define SARC_A_REVERSE_LEFT_COMMAND     -60
#define SARC_A_REVERSE_RIGHT_FAST      -120
#define SARC_A_REVERSE_RIGHT_SLOW      -100
#define SARC_REVERSE_SETTLE_MS          800U
#define SARC_REVERSE_TIMEOUT_MS        3000U
#define SARC_ENCODER_READY_FRAMES        20U
#define SARC_ENCODER_READY_MAX_AGE_MS   100U
#define SARC_ENCODER_RX_TIMEOUT_MS      200U
#define SARC_FINAL_LEFT_TOLERANCE         5
#define SARC_FINAL_RIGHT_SLOW_WINDOW    500
#define SARC_FINAL_RIGHT_STOP_LEAD       60
#define SARC_DIRECTION_WRONG_TICKS       50
#define SARC_ARM_MS                     5000U
#define SARC_ARMING_EVENT_MS            1000U
#define SARC_SUMMARY_MS                 1000U

typedef enum {
    SARC_WAIT_ENCODER = 0,
    SARC_ARM,
    SARC_FORWARD_A,
    SARC_FORWARD_B,
    SARC_FORWARD_SETTLE,
    SARC_REVERSE_PAUSE,
    SARC_REVERSE_B,
    SARC_REVERSE_A,
    SARC_REVERSE_SETTLE,
    SARC_DONE,
    SARC_ABORT
} TwoArcState;

typedef enum {
    SARC_FINAL_RIGHT_FAST = 0,
    SARC_FINAL_RIGHT_SLOW,
    SARC_FINAL_RIGHT_DONE
} TwoArcRightBrakeState;

typedef struct {
    uint8_t valid;
    uint8_t aborted;
    const char *abortReason;
    int32_t forwardStartLeft;
    int32_t forwardStartRight;
    int32_t forwardBoundaryLeft;
    int32_t forwardBoundaryRight;
    int32_t forwardEndLeft;
    int32_t forwardEndRight;
    int32_t segmentALeft;
    int32_t segmentARight;
    int32_t segmentBLeft;
    int32_t segmentBRight;
    int32_t totalTargetLeft;
    int32_t totalTargetRight;
    int32_t reverseStartLeft;
    int32_t reverseStartRight;
    int32_t reverseBoundaryLeft;
    int32_t reverseBoundaryRight;
    int32_t reverseEndLeft;
    int32_t reverseEndRight;
    int32_t bReverseLeft;
    int32_t bReverseRight;
    int32_t bBoundaryErrorLeft;
    int32_t bBoundaryErrorRight;
    int32_t reverseTotalLeft;
    int32_t reverseTotalRight;
    int32_t errorLeft;
    int32_t errorRight;
    int32_t errorLeftPermille;
    int32_t errorRightPermille;
    uint32_t forwardStartMs;
    uint32_t reverseStartMs;
    uint32_t reverseBoundaryMs;
    uint32_t reverseMotionStopMs;
    uint32_t forwardDurationMs;
    uint32_t reverseDurationMs;
    uint32_t forwardStartRxValid;
    uint32_t forwardEndRxValid;
    uint32_t reverseStartRxValid;
    uint32_t reverseEndRxValid;
    uint32_t forwardEncoderFrames;
    uint32_t reverseEncoderFrames;
    uint32_t leftReachedMs;
    uint32_t rightReachedMs;
    uint32_t rightSlowStartMs;
    int32_t rightSlowStartTravel;
    int32_t rightSlowStartRemaining;
    int32_t rightStopTravel;
    int32_t rightStopRemaining;
    uint8_t forwardStartSeq;
    uint8_t forwardBoundarySeq;
    uint8_t forwardEndSeq;
    uint8_t reverseStartSeq;
    uint8_t reverseBoundarySeq;
    uint8_t reverseEndSeq;
} TwoArcRoundtripResult;

static TwoArcState g_twoArcState;
static TwoArcRoundtripResult g_twoArcRoundtripResult;
static uint32_t g_twoArcStateStartMs;
static uint32_t g_twoArcReadyBaselineValidCount;
static uint32_t g_twoArcLastArmingEventMs;
static uint32_t g_twoArcLastSummaryMs;
static uint8_t g_twoArcFinalLeftDone;
static uint8_t g_twoArcFinalRightDone;
static uint8_t g_twoArcLeftReachedPublished;
static uint8_t g_twoArcRightReachedPublished;
static TwoArcRightBrakeState g_twoArcRightBrakeState;
static char g_twoArcTelemetryText[768];

static void TwoArcPublish(const char *event, uint32_t remainingMs)
{
    (void)snprintf(g_twoArcTelemetryText, sizeof(g_twoArcTelemetryText),
                   "SARC event=%s a_l=%ld a_r=%ld b_l=%ld b_r=%ld total_l=%ld total_r=%ld "
                   "b_err_l=%ld b_err_r=%ld reverse_l=%ld reverse_r=%ld "
                   "error_l=%ld error_r=%ld error_l_permille=%ld error_r_permille=%ld "
                   "forward_ms=%u reverse_ms=%u forward_frames=%u reverse_frames=%u "
                   "b_boundary_ms=%u left_reached_ms=%u right_reached_ms=%u "
                   "right_slow_start_ms=%u right_slow_start_travel=%ld "
                   "right_slow_start_remaining=%ld right_stop_travel=%ld right_stop_remaining=%ld "
                   "valid=%u aborted=%u reason=%s left_done=%u right_done=%u remaining_ms=%u",
                   event,
                   (long)g_twoArcRoundtripResult.segmentALeft,
                   (long)g_twoArcRoundtripResult.segmentARight,
                   (long)g_twoArcRoundtripResult.segmentBLeft,
                   (long)g_twoArcRoundtripResult.segmentBRight,
                   (long)g_twoArcRoundtripResult.totalTargetLeft,
                   (long)g_twoArcRoundtripResult.totalTargetRight,
                   (long)g_twoArcRoundtripResult.bBoundaryErrorLeft,
                   (long)g_twoArcRoundtripResult.bBoundaryErrorRight,
                   (long)g_twoArcRoundtripResult.reverseTotalLeft,
                   (long)g_twoArcRoundtripResult.reverseTotalRight,
                   (long)g_twoArcRoundtripResult.errorLeft,
                   (long)g_twoArcRoundtripResult.errorRight,
                   (long)g_twoArcRoundtripResult.errorLeftPermille,
                   (long)g_twoArcRoundtripResult.errorRightPermille,
                   (unsigned int)g_twoArcRoundtripResult.forwardDurationMs,
                   (unsigned int)g_twoArcRoundtripResult.reverseDurationMs,
                   (unsigned int)g_twoArcRoundtripResult.forwardEncoderFrames,
                   (unsigned int)g_twoArcRoundtripResult.reverseEncoderFrames,
                   (unsigned int)g_twoArcRoundtripResult.reverseBoundaryMs,
                   (unsigned int)g_twoArcRoundtripResult.leftReachedMs,
                   (unsigned int)g_twoArcRoundtripResult.rightReachedMs,
                   (unsigned int)g_twoArcRoundtripResult.rightSlowStartMs,
                   (long)g_twoArcRoundtripResult.rightSlowStartTravel,
                   (long)g_twoArcRoundtripResult.rightSlowStartRemaining,
                   (long)g_twoArcRoundtripResult.rightStopTravel,
                   (long)g_twoArcRoundtripResult.rightStopRemaining,
                   (unsigned int)g_twoArcRoundtripResult.valid,
                   (unsigned int)g_twoArcRoundtripResult.aborted,
                   (g_twoArcRoundtripResult.abortReason != NULL) ?
                       g_twoArcRoundtripResult.abortReason : "NONE",
                   (unsigned int)g_twoArcFinalLeftDone,
                   (unsigned int)g_twoArcFinalRightDone,
                   (unsigned int)remainingMs);
    UdpTelemetryPublishCal(g_twoArcTelemetryText);
}

static void TwoArcAbort(const char *reason)
{
    if (g_twoArcState == SARC_ABORT) {
        return;
    }
    g_twoArcRoundtripResult.aborted = 1U;
    g_twoArcRoundtripResult.abortReason = reason;
    g_twoArcState = SARC_ABORT;
    TwoArcPublish("ABORT", 0U);
}

static int TwoArcEncoderFresh(const UdpEncoderTelemetryState *encoder,
                              uint32_t nowMs, uint32_t maximumAgeMs)
{
    return (encoder->validCount != 0U &&
            (uint32_t)(nowMs - encoder->lastRxMs) <= maximumAgeMs) ? 1 : 0;
}

static void TwoArcInitializeFinalRightBrake(uint32_t nowMs, int32_t totalReverseRight)
{
    int32_t travel = -totalReverseRight;
    int32_t remaining = g_twoArcRoundtripResult.totalTargetRight - travel;

    if (remaining <= SARC_FINAL_RIGHT_STOP_LEAD) {
        g_twoArcFinalRightDone = 1U;
        g_twoArcRightBrakeState = SARC_FINAL_RIGHT_DONE;
        g_twoArcRoundtripResult.rightReachedMs =
            nowMs - g_twoArcRoundtripResult.reverseStartMs;
        g_twoArcRoundtripResult.rightStopTravel = travel;
        g_twoArcRoundtripResult.rightStopRemaining = remaining;
        g_twoArcRightReachedPublished = 1U;
        TwoArcPublish("FINAL_RIGHT_STOP", 0U);
    } else if (remaining <= SARC_FINAL_RIGHT_SLOW_WINDOW) {
        g_twoArcRightBrakeState = SARC_FINAL_RIGHT_SLOW;
        g_twoArcRoundtripResult.rightSlowStartMs =
            nowMs - g_twoArcRoundtripResult.reverseStartMs;
        g_twoArcRoundtripResult.rightSlowStartTravel = travel;
        g_twoArcRoundtripResult.rightSlowStartRemaining = remaining;
        TwoArcPublish("FINAL_RIGHT_SLOW", 0U);
    } else {
        g_twoArcRightBrakeState = SARC_FINAL_RIGHT_FAST;
    }
}

void TwoArcRoundtripInit(void)
{
    g_twoArcState = SARC_WAIT_ENCODER;
    g_twoArcRoundtripResult = (TwoArcRoundtripResult){0};
    g_twoArcStateStartMs = 0U;
    g_twoArcReadyBaselineValidCount = 0U;
    g_twoArcLastArmingEventMs = 0U;
    g_twoArcLastSummaryMs = 0U;
    g_twoArcFinalLeftDone = 0U;
    g_twoArcFinalRightDone = 0U;
    g_twoArcLeftReachedPublished = 0U;
    g_twoArcRightReachedPublished = 0U;
    g_twoArcRightBrakeState = SARC_FINAL_RIGHT_FAST;
    TwoArcPublish("BOOT", 0U);
    TwoArcPublish("WAIT_ENCODER", 0U);
}

void TwoArcRoundtripStep(uint32_t now, int *leftCommand, int *rightCommand)
{
    UdpEncoderTelemetryState encoder;
    uint32_t nowMs = AppTicksToMs(now);
    int32_t reverseLeft;
    int32_t reverseRight;
    int32_t totalRightTravel;
    int32_t totalRightRemaining;

    if (leftCommand == NULL || rightCommand == NULL) {
        return;
    }
    *leftCommand = 0;
    *rightCommand = 0;
    UdpTelemetryReadEncoder(&encoder);

    if ((g_twoArcState == SARC_FORWARD_A || g_twoArcState == SARC_FORWARD_B ||
         g_twoArcState == SARC_REVERSE_B || g_twoArcState == SARC_REVERSE_A) &&
        TwoArcEncoderFresh(&encoder, nowMs, SARC_ENCODER_RX_TIMEOUT_MS) == 0) {
        TwoArcAbort("ENCODER_RX_TIMEOUT");
        return;
    }

    switch (g_twoArcState) {
        case SARC_WAIT_ENCODER:
            if (g_twoArcReadyBaselineValidCount == 0U) {
                g_twoArcReadyBaselineValidCount = encoder.validCount;
            }
            if (encoder.validCount - g_twoArcReadyBaselineValidCount >=
                    SARC_ENCODER_READY_FRAMES &&
                TwoArcEncoderFresh(&encoder, nowMs,
                                   SARC_ENCODER_READY_MAX_AGE_MS) != 0) {
                g_twoArcState = SARC_ARM;
                g_twoArcStateStartMs = nowMs;
                g_twoArcLastArmingEventMs = nowMs;
                TwoArcPublish("ENCODER_READY", SARC_ARM_MS);
            }
            break;

        case SARC_ARM:
            if ((uint32_t)(nowMs - g_twoArcStateStartMs) >= SARC_ARM_MS) {
                g_twoArcState = SARC_FORWARD_A;
                g_twoArcStateStartMs = nowMs;
                g_twoArcRoundtripResult.forwardStartMs = nowMs;
                g_twoArcRoundtripResult.forwardStartLeft = encoder.totalLeft;
                g_twoArcRoundtripResult.forwardStartRight = encoder.totalRight;
                g_twoArcRoundtripResult.forwardStartSeq = encoder.sequence;
                g_twoArcRoundtripResult.forwardStartRxValid = encoder.validCount;
                g_twoArcRoundtripResult.forwardDurationMs = 2U * SARC_SEGMENT_MS;
                TwoArcPublish("FORWARD_A_START", 0U);
                *leftCommand = SARC_A_FORWARD_LEFT_COMMAND;
                *rightCommand = SARC_A_FORWARD_RIGHT_COMMAND;
            } else if ((uint32_t)(nowMs - g_twoArcLastArmingEventMs) >=
                       SARC_ARMING_EVENT_MS) {
                uint32_t elapsedMs = nowMs - g_twoArcStateStartMs;
                g_twoArcLastArmingEventMs = nowMs;
                TwoArcPublish("ARMING", SARC_ARM_MS - elapsedMs);
            }
            break;

        case SARC_FORWARD_A:
            if ((uint32_t)(nowMs - g_twoArcStateStartMs) >= SARC_SEGMENT_MS) {
                g_twoArcRoundtripResult.forwardBoundaryLeft = encoder.totalLeft;
                g_twoArcRoundtripResult.forwardBoundaryRight = encoder.totalRight;
                g_twoArcRoundtripResult.forwardBoundarySeq = encoder.sequence;
                g_twoArcState = SARC_FORWARD_B;
                g_twoArcStateStartMs = nowMs;
                TwoArcPublish("FORWARD_AB_SWITCH", 0U);
                *leftCommand = SARC_B_FORWARD_LEFT_COMMAND;
                *rightCommand = SARC_B_FORWARD_RIGHT_COMMAND;
            } else {
                *leftCommand = SARC_A_FORWARD_LEFT_COMMAND;
                *rightCommand = SARC_A_FORWARD_RIGHT_COMMAND;
            }
            break;

        case SARC_FORWARD_B:
            if ((uint32_t)(nowMs - g_twoArcStateStartMs) >= SARC_SEGMENT_MS) {
                g_twoArcState = SARC_FORWARD_SETTLE;
                g_twoArcStateStartMs = nowMs;
                TwoArcPublish("FORWARD_B_STOP", 0U);
            } else {
                *leftCommand = SARC_B_FORWARD_LEFT_COMMAND;
                *rightCommand = SARC_B_FORWARD_RIGHT_COMMAND;
            }
            break;

        case SARC_FORWARD_SETTLE:
            if ((uint32_t)(nowMs - g_twoArcStateStartMs) >= SARC_FORWARD_SETTLE_MS) {
                g_twoArcRoundtripResult.forwardEndLeft = encoder.totalLeft;
                g_twoArcRoundtripResult.forwardEndRight = encoder.totalRight;
                g_twoArcRoundtripResult.forwardEndSeq = encoder.sequence;
                g_twoArcRoundtripResult.forwardEndRxValid = encoder.validCount;
                g_twoArcRoundtripResult.segmentALeft =
                    g_twoArcRoundtripResult.forwardBoundaryLeft -
                    g_twoArcRoundtripResult.forwardStartLeft;
                g_twoArcRoundtripResult.segmentARight =
                    g_twoArcRoundtripResult.forwardBoundaryRight -
                    g_twoArcRoundtripResult.forwardStartRight;
                g_twoArcRoundtripResult.segmentBLeft =
                    encoder.totalLeft - g_twoArcRoundtripResult.forwardBoundaryLeft;
                g_twoArcRoundtripResult.segmentBRight =
                    encoder.totalRight - g_twoArcRoundtripResult.forwardBoundaryRight;
                g_twoArcRoundtripResult.totalTargetLeft =
                    encoder.totalLeft - g_twoArcRoundtripResult.forwardStartLeft;
                g_twoArcRoundtripResult.totalTargetRight =
                    encoder.totalRight - g_twoArcRoundtripResult.forwardStartRight;
                g_twoArcRoundtripResult.forwardEncoderFrames =
                    encoder.validCount - g_twoArcRoundtripResult.forwardStartRxValid;
                if (g_twoArcRoundtripResult.totalTargetLeft <= 0 ||
                    g_twoArcRoundtripResult.totalTargetRight <= 0 ||
                    g_twoArcRoundtripResult.segmentBLeft <= 0) {
                    TwoArcAbort("FORWARD_DIRECTION_INVALID");
                    break;
                }
                g_twoArcState = SARC_REVERSE_PAUSE;
                g_twoArcStateStartMs = nowMs;
                TwoArcPublish("FORWARD_DONE", 0U);
            }
            break;

        case SARC_REVERSE_PAUSE:
            if ((uint32_t)(nowMs - g_twoArcStateStartMs) >= SARC_REVERSE_PAUSE_MS) {
                g_twoArcState = SARC_REVERSE_B;
                g_twoArcStateStartMs = nowMs;
                g_twoArcRoundtripResult.reverseStartMs = nowMs;
                g_twoArcRoundtripResult.reverseStartLeft = encoder.totalLeft;
                g_twoArcRoundtripResult.reverseStartRight = encoder.totalRight;
                g_twoArcRoundtripResult.reverseStartSeq = encoder.sequence;
                g_twoArcRoundtripResult.reverseStartRxValid = encoder.validCount;
                g_twoArcFinalLeftDone = 0U;
                g_twoArcFinalRightDone = 0U;
                g_twoArcLeftReachedPublished = 0U;
                g_twoArcRightReachedPublished = 0U;
                g_twoArcRightBrakeState = SARC_FINAL_RIGHT_FAST;
                TwoArcPublish("REVERSE_B_START", 0U);
                *leftCommand = SARC_B_REVERSE_LEFT_COMMAND;
                *rightCommand = SARC_B_REVERSE_RIGHT_COMMAND;
            }
            break;

        case SARC_REVERSE_B:
            reverseLeft = encoder.totalLeft - g_twoArcRoundtripResult.reverseStartLeft;
            reverseRight = encoder.totalRight - g_twoArcRoundtripResult.reverseStartRight;
            if (reverseLeft > SARC_DIRECTION_WRONG_TICKS ||
                reverseRight > SARC_DIRECTION_WRONG_TICKS) {
                TwoArcAbort("ENCODER_DIRECTION_WRONG");
                break;
            }
            if ((uint32_t)(nowMs - g_twoArcRoundtripResult.reverseStartMs) >=
                    SARC_REVERSE_TIMEOUT_MS) {
                TwoArcAbort("REVERSE_TIMEOUT");
                break;
            }
            if (-reverseLeft >= g_twoArcRoundtripResult.segmentBLeft) {
                g_twoArcRoundtripResult.reverseBoundaryLeft = encoder.totalLeft;
                g_twoArcRoundtripResult.reverseBoundaryRight = encoder.totalRight;
                g_twoArcRoundtripResult.reverseBoundarySeq = encoder.sequence;
                g_twoArcRoundtripResult.reverseBoundaryMs =
                    nowMs - g_twoArcRoundtripResult.reverseStartMs;
                g_twoArcRoundtripResult.bReverseLeft = reverseLeft;
                g_twoArcRoundtripResult.bReverseRight = reverseRight;
                g_twoArcRoundtripResult.bBoundaryErrorLeft =
                    g_twoArcRoundtripResult.segmentBLeft + reverseLeft;
                g_twoArcRoundtripResult.bBoundaryErrorRight =
                    g_twoArcRoundtripResult.segmentBRight + reverseRight;
                g_twoArcState = SARC_REVERSE_A;
                g_twoArcStateStartMs = nowMs;
                TwoArcInitializeFinalRightBrake(nowMs, reverseRight);
                TwoArcPublish("REVERSE_BA_SWITCH", 0U);
                *leftCommand = SARC_A_REVERSE_LEFT_COMMAND;
                *rightCommand = (g_twoArcFinalRightDone != 0U) ? 0 :
                    ((g_twoArcRightBrakeState == SARC_FINAL_RIGHT_SLOW) ?
                     SARC_A_REVERSE_RIGHT_SLOW : SARC_A_REVERSE_RIGHT_FAST);
            } else {
                *leftCommand = SARC_B_REVERSE_LEFT_COMMAND;
                *rightCommand = SARC_B_REVERSE_RIGHT_COMMAND;
            }
            break;

        case SARC_REVERSE_A:
            reverseLeft = encoder.totalLeft - g_twoArcRoundtripResult.reverseStartLeft;
            reverseRight = encoder.totalRight - g_twoArcRoundtripResult.reverseStartRight;
            totalRightTravel = -reverseRight;
            totalRightRemaining =
                g_twoArcRoundtripResult.totalTargetRight - totalRightTravel;
            if (reverseLeft > SARC_DIRECTION_WRONG_TICKS ||
                reverseRight > SARC_DIRECTION_WRONG_TICKS) {
                TwoArcAbort("ENCODER_DIRECTION_WRONG");
                break;
            }
            if ((uint32_t)(nowMs - g_twoArcRoundtripResult.reverseStartMs) >=
                    SARC_REVERSE_TIMEOUT_MS &&
                (g_twoArcFinalLeftDone == 0U || g_twoArcFinalRightDone == 0U)) {
                TwoArcAbort("REVERSE_TIMEOUT");
                break;
            }
            if (g_twoArcFinalLeftDone == 0U &&
                (g_twoArcRoundtripResult.totalTargetLeft <= SARC_FINAL_LEFT_TOLERANCE ||
                 reverseLeft <= -(g_twoArcRoundtripResult.totalTargetLeft -
                                  SARC_FINAL_LEFT_TOLERANCE))) {
                g_twoArcFinalLeftDone = 1U;
                g_twoArcRoundtripResult.leftReachedMs =
                    (g_twoArcRoundtripResult.totalTargetLeft <=
                     SARC_FINAL_LEFT_TOLERANCE) ? 0U :
                    nowMs - g_twoArcRoundtripResult.reverseStartMs;
                if (g_twoArcLeftReachedPublished == 0U) {
                    g_twoArcLeftReachedPublished = 1U;
                    TwoArcPublish("FINAL_LEFT_REACHED", 0U);
                }
            }
            if (g_twoArcFinalRightDone == 0U &&
                g_twoArcRightBrakeState == SARC_FINAL_RIGHT_FAST &&
                totalRightRemaining <= SARC_FINAL_RIGHT_SLOW_WINDOW) {
                g_twoArcRightBrakeState = SARC_FINAL_RIGHT_SLOW;
                g_twoArcRoundtripResult.rightSlowStartMs =
                    nowMs - g_twoArcRoundtripResult.reverseStartMs;
                g_twoArcRoundtripResult.rightSlowStartTravel = totalRightTravel;
                g_twoArcRoundtripResult.rightSlowStartRemaining = totalRightRemaining;
                TwoArcPublish("FINAL_RIGHT_SLOW", 0U);
            } else if (g_twoArcFinalRightDone == 0U &&
                       g_twoArcRightBrakeState == SARC_FINAL_RIGHT_SLOW &&
                       totalRightRemaining <= SARC_FINAL_RIGHT_STOP_LEAD) {
                g_twoArcFinalRightDone = 1U;
                g_twoArcRightBrakeState = SARC_FINAL_RIGHT_DONE;
                g_twoArcRoundtripResult.rightReachedMs =
                    nowMs - g_twoArcRoundtripResult.reverseStartMs;
                g_twoArcRoundtripResult.rightStopTravel = totalRightTravel;
                g_twoArcRoundtripResult.rightStopRemaining = totalRightRemaining;
                if (g_twoArcRightReachedPublished == 0U) {
                    g_twoArcRightReachedPublished = 1U;
                    TwoArcPublish("FINAL_RIGHT_STOP", 0U);
                }
            }
            if (g_twoArcFinalLeftDone != 0U && g_twoArcFinalRightDone != 0U) {
                g_twoArcRoundtripResult.reverseMotionStopMs = nowMs;
                g_twoArcRoundtripResult.reverseDurationMs =
                    nowMs - g_twoArcRoundtripResult.reverseStartMs;
                g_twoArcState = SARC_REVERSE_SETTLE;
                g_twoArcStateStartMs = nowMs;
                TwoArcPublish("REVERSE_STOP", 0U);
            } else {
                *leftCommand = (g_twoArcFinalLeftDone != 0U) ? 0 :
                    SARC_A_REVERSE_LEFT_COMMAND;
                *rightCommand = (g_twoArcFinalRightDone != 0U) ? 0 :
                    ((g_twoArcRightBrakeState == SARC_FINAL_RIGHT_SLOW) ?
                     SARC_A_REVERSE_RIGHT_SLOW : SARC_A_REVERSE_RIGHT_FAST);
            }
            break;

        case SARC_REVERSE_SETTLE:
            if ((uint32_t)(nowMs - g_twoArcStateStartMs) >= SARC_REVERSE_SETTLE_MS) {
                g_twoArcRoundtripResult.reverseEndLeft = encoder.totalLeft;
                g_twoArcRoundtripResult.reverseEndRight = encoder.totalRight;
                g_twoArcRoundtripResult.reverseEndSeq = encoder.sequence;
                g_twoArcRoundtripResult.reverseEndRxValid = encoder.validCount;
                g_twoArcRoundtripResult.reverseTotalLeft =
                    encoder.totalLeft - g_twoArcRoundtripResult.reverseStartLeft;
                g_twoArcRoundtripResult.reverseTotalRight =
                    encoder.totalRight - g_twoArcRoundtripResult.reverseStartRight;
                g_twoArcRoundtripResult.reverseEncoderFrames =
                    encoder.validCount - g_twoArcRoundtripResult.reverseStartRxValid;
                g_twoArcRoundtripResult.errorLeft =
                    g_twoArcRoundtripResult.totalTargetLeft +
                    g_twoArcRoundtripResult.reverseTotalLeft;
                g_twoArcRoundtripResult.errorRight =
                    g_twoArcRoundtripResult.totalTargetRight +
                    g_twoArcRoundtripResult.reverseTotalRight;
                g_twoArcRoundtripResult.errorLeftPermille =
                    (g_twoArcRoundtripResult.totalTargetLeft != 0) ?
                    (g_twoArcRoundtripResult.errorLeft * 1000 /
                     g_twoArcRoundtripResult.totalTargetLeft) : 0;
                g_twoArcRoundtripResult.errorRightPermille =
                    (g_twoArcRoundtripResult.totalTargetRight != 0) ?
                    (g_twoArcRoundtripResult.errorRight * 1000 /
                     g_twoArcRoundtripResult.totalTargetRight) : 0;
                g_twoArcRoundtripResult.valid = 1U;
                g_twoArcState = SARC_DONE;
                g_twoArcLastSummaryMs = nowMs;
                TwoArcPublish("RESULT", 0U);
            }
            break;

        case SARC_DONE:
            if ((uint32_t)(nowMs - g_twoArcLastSummaryMs) >= SARC_SUMMARY_MS) {
                g_twoArcLastSummaryMs = nowMs;
                TwoArcPublish("SUMMARY", 0U);
            }
            break;

        case SARC_ABORT:
        default:
            break;
    }
}
