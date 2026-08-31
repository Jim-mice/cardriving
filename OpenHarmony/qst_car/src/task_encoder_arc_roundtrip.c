#include <stdint.h>
#include <stdio.h>

#include "app_time.h"
#include "task_encoder_arc_roundtrip.h"
#include "udp_telemetry.h"

#define ARC_FORWARD_LEFT_COMMAND       60
#define ARC_FORWARD_RIGHT_COMMAND     120
#define ARC_FORWARD_MS                300U
#define ARC_FORWARD_SETTLE_MS         800U
#define ARC_REVERSE_PAUSE_MS          300U
#define ARC_REVERSE_LEFT_COMMAND      -60
#define ARC_REVERSE_RIGHT_COMMAND    -120
#define ARC_RIGHT_SLOW_COMMAND       -100
#define ARC_REVERSE_SETTLE_MS         800U
#define ARC_REVERSE_TIMEOUT_MS       2500U
#define ARC_ENCODER_READY_FRAMES       20U
#define ARC_ENCODER_READY_MAX_AGE_MS  100U
#define ARC_ENCODER_RX_TIMEOUT_MS     200U
#define ARC_LEFT_TOLERANCE_TICKS        5
#define ARC_RIGHT_TOLERANCE_TICKS      15
#define ARC_RIGHT_SLOW_WINDOW_TICKS   500
#define ARC_RIGHT_STOP_LEAD_TICKS      60
#define ARC_DIRECTION_WRONG_TICKS      50
#define ARC_ARM_MS                    5000U
#define ARC_ARMING_EVENT_MS           1000U
#define ARC_SUMMARY_MS                1000U

typedef enum {
    ARC_WAIT_ENCODER = 0,
    ARC_ARM,
    ARC_FORWARD,
    ARC_FORWARD_SETTLE,
    ARC_REVERSE_PAUSE,
    ARC_REVERSE,
    ARC_REVERSE_SETTLE,
    ARC_DONE,
    ARC_ABORT
} ArcRoundtripState;

typedef enum {
    ARC_RIGHT_FAST = 0,
    ARC_RIGHT_SLOW,
    ARC_RIGHT_DONE
} ArcRightBrakeState;

typedef struct {
    uint8_t valid;
    uint8_t aborted;
    const char *abortReason;
    int32_t forwardStartLeft;
    int32_t forwardStartRight;
    int32_t forwardEndLeft;
    int32_t forwardEndRight;
    int32_t targetLeft;
    int32_t targetRight;
    int32_t reverseStartLeft;
    int32_t reverseStartRight;
    int32_t reverseEndLeft;
    int32_t reverseEndRight;
    int32_t reverseTotalLeft;
    int32_t reverseTotalRight;
    int32_t errorLeft;
    int32_t errorRight;
    int32_t errorLeftPermille;
    int32_t errorRightPermille;
    uint32_t forwardStartMs;
    uint32_t reverseStartMs;
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
    uint8_t forwardEndSeq;
    uint8_t reverseStartSeq;
    uint8_t reverseEndSeq;
} ArcRoundtripResult;

static ArcRoundtripState g_arcState;
static ArcRoundtripResult g_arcRoundtripResult;
static uint32_t g_arcStateStartMs;
static uint32_t g_arcReadyBaselineValidCount;
static uint32_t g_arcLastArmingEventMs;
static uint32_t g_arcLastSummaryMs;
static uint8_t g_arcLeftDone;
static uint8_t g_arcRightDone;
static uint8_t g_arcLeftReachedPublished;
static uint8_t g_arcRightReachedPublished;
static uint8_t g_arcLowLeftTargetPending;
static ArcRightBrakeState g_arcRightBrakeState;
static char g_arcTelemetryText[768];

static void ArcPublish(const char *event, uint32_t remainingMs)
{
    (void)snprintf(g_arcTelemetryText, sizeof(g_arcTelemetryText),
                   "ARC event=%s target_l=%ld target_r=%ld reverse_l=%ld reverse_r=%ld "
                   "error_l=%ld error_r=%ld error_l_permille=%ld error_r_permille=%ld "
                   "forward_ms=%u reverse_ms=%u forward_frames=%u reverse_frames=%u "
                   "left_reached_ms=%u right_reached_ms=%u f_start_seq=%u f_end_seq=%u "
                   "r_start_seq=%u r_end_seq=%u valid=%u aborted=%u reason=%s "
                   "left_done=%u right_done=%u remaining_ms=%u "
                   "right_slow_start_ms=%u right_slow_start_travel=%ld "
                   "right_slow_start_remaining=%ld right_stop_travel=%ld "
                   "right_stop_remaining=%ld",
                   event,
                   (long)g_arcRoundtripResult.targetLeft,
                   (long)g_arcRoundtripResult.targetRight,
                   (long)g_arcRoundtripResult.reverseTotalLeft,
                   (long)g_arcRoundtripResult.reverseTotalRight,
                   (long)g_arcRoundtripResult.errorLeft,
                   (long)g_arcRoundtripResult.errorRight,
                   (long)g_arcRoundtripResult.errorLeftPermille,
                   (long)g_arcRoundtripResult.errorRightPermille,
                   (unsigned int)g_arcRoundtripResult.forwardDurationMs,
                   (unsigned int)g_arcRoundtripResult.reverseDurationMs,
                   (unsigned int)g_arcRoundtripResult.forwardEncoderFrames,
                   (unsigned int)g_arcRoundtripResult.reverseEncoderFrames,
                   (unsigned int)g_arcRoundtripResult.leftReachedMs,
                   (unsigned int)g_arcRoundtripResult.rightReachedMs,
                   (unsigned int)g_arcRoundtripResult.forwardStartSeq,
                   (unsigned int)g_arcRoundtripResult.forwardEndSeq,
                   (unsigned int)g_arcRoundtripResult.reverseStartSeq,
                   (unsigned int)g_arcRoundtripResult.reverseEndSeq,
                   (unsigned int)g_arcRoundtripResult.valid,
                   (unsigned int)g_arcRoundtripResult.aborted,
                   (g_arcRoundtripResult.abortReason != NULL) ?
                       g_arcRoundtripResult.abortReason : "NONE",
                   (unsigned int)g_arcLeftDone,
                   (unsigned int)g_arcRightDone,
                   (unsigned int)remainingMs,
                   (unsigned int)g_arcRoundtripResult.rightSlowStartMs,
                   (long)g_arcRoundtripResult.rightSlowStartTravel,
                   (long)g_arcRoundtripResult.rightSlowStartRemaining,
                   (long)g_arcRoundtripResult.rightStopTravel,
                   (long)g_arcRoundtripResult.rightStopRemaining);
    UdpTelemetryPublishCal(g_arcTelemetryText);
}

static void ArcAbort(const char *reason)
{
    if (g_arcState == ARC_ABORT) {
        return;
    }

    g_arcRoundtripResult.aborted = 1U;
    g_arcRoundtripResult.abortReason = reason;
    g_arcState = ARC_ABORT;
    ArcPublish("ABORT", 0U);
}

static int ArcEncoderFresh(const UdpEncoderTelemetryState *encoder,
                           uint32_t nowMs, uint32_t maximumAgeMs)
{
    return (encoder->validCount != 0U &&
            (uint32_t)(nowMs - encoder->lastRxMs) <= maximumAgeMs) ? 1 : 0;
}

void ArcRoundtripInit(void)
{
    g_arcState = ARC_WAIT_ENCODER;
    g_arcRoundtripResult = (ArcRoundtripResult){0};
    g_arcStateStartMs = 0U;
    g_arcReadyBaselineValidCount = 0U;
    g_arcLastArmingEventMs = 0U;
    g_arcLastSummaryMs = 0U;
    g_arcLeftDone = 0U;
    g_arcRightDone = 0U;
    g_arcLeftReachedPublished = 0U;
    g_arcRightReachedPublished = 0U;
    g_arcLowLeftTargetPending = 0U;
    g_arcRightBrakeState = ARC_RIGHT_FAST;
    ArcPublish("BOOT", 0U);
    ArcPublish("WAIT_ENCODER", 0U);
}

void ArcRoundtripStep(uint32_t now, int *leftCommand, int *rightCommand)
{
    UdpEncoderTelemetryState encoder;
    uint32_t nowMs = AppTicksToMs(now);
    int32_t reverseLeft;
    int32_t reverseRight;
    int32_t rightTravel;
    int32_t rightRemaining;

    if (leftCommand == NULL || rightCommand == NULL) {
        return;
    }

    *leftCommand = 0;
    *rightCommand = 0;
    UdpTelemetryReadEncoder(&encoder);

    if ((g_arcState == ARC_FORWARD || g_arcState == ARC_REVERSE) &&
        ArcEncoderFresh(&encoder, nowMs, ARC_ENCODER_RX_TIMEOUT_MS) == 0) {
        ArcAbort("ENCODER_RX_TIMEOUT");
        return;
    }

    switch (g_arcState) {
        case ARC_WAIT_ENCODER:
            if (g_arcReadyBaselineValidCount == 0U) {
                g_arcReadyBaselineValidCount = encoder.validCount;
            }
            if (encoder.validCount - g_arcReadyBaselineValidCount >=
                    ARC_ENCODER_READY_FRAMES &&
                ArcEncoderFresh(&encoder, nowMs,
                                ARC_ENCODER_READY_MAX_AGE_MS) != 0) {
                g_arcState = ARC_ARM;
                g_arcStateStartMs = nowMs;
                g_arcLastArmingEventMs = nowMs;
                ArcPublish("ENCODER_READY", ARC_ARM_MS);
            }
            break;

        case ARC_ARM:
            if ((uint32_t)(nowMs - g_arcStateStartMs) >= ARC_ARM_MS) {
                g_arcState = ARC_FORWARD;
                g_arcStateStartMs = nowMs;
                g_arcRoundtripResult.forwardStartMs = nowMs;
                g_arcRoundtripResult.forwardStartLeft = encoder.totalLeft;
                g_arcRoundtripResult.forwardStartRight = encoder.totalRight;
                g_arcRoundtripResult.forwardStartSeq = encoder.sequence;
                g_arcRoundtripResult.forwardStartRxValid = encoder.validCount;
                g_arcRoundtripResult.forwardDurationMs = ARC_FORWARD_MS;
                ArcPublish("FORWARD_START", 0U);
                *leftCommand = ARC_FORWARD_LEFT_COMMAND;
                *rightCommand = ARC_FORWARD_RIGHT_COMMAND;
            } else if ((uint32_t)(nowMs - g_arcLastArmingEventMs) >=
                       ARC_ARMING_EVENT_MS) {
                uint32_t elapsedMs = nowMs - g_arcStateStartMs;
                g_arcLastArmingEventMs = nowMs;
                ArcPublish("ARMING", ARC_ARM_MS - elapsedMs);
            }
            break;

        case ARC_FORWARD:
            if ((uint32_t)(nowMs - g_arcStateStartMs) >= ARC_FORWARD_MS) {
                g_arcState = ARC_FORWARD_SETTLE;
                g_arcStateStartMs = nowMs;
                ArcPublish("FORWARD_STOP", 0U);
            } else {
                *leftCommand = ARC_FORWARD_LEFT_COMMAND;
                *rightCommand = ARC_FORWARD_RIGHT_COMMAND;
            }
            break;

        case ARC_FORWARD_SETTLE:
            if ((uint32_t)(nowMs - g_arcStateStartMs) >= ARC_FORWARD_SETTLE_MS) {
                g_arcRoundtripResult.forwardEndLeft = encoder.totalLeft;
                g_arcRoundtripResult.forwardEndRight = encoder.totalRight;
                g_arcRoundtripResult.forwardEndSeq = encoder.sequence;
                g_arcRoundtripResult.forwardEndRxValid = encoder.validCount;
                g_arcRoundtripResult.targetLeft =
                    encoder.totalLeft - g_arcRoundtripResult.forwardStartLeft;
                g_arcRoundtripResult.targetRight =
                    encoder.totalRight - g_arcRoundtripResult.forwardStartRight;
                g_arcRoundtripResult.forwardEncoderFrames =
                    encoder.validCount - g_arcRoundtripResult.forwardStartRxValid;
                if (g_arcRoundtripResult.targetLeft < 0 ||
                    g_arcRoundtripResult.targetRight <= 0) {
                    ArcAbort("FORWARD_DIRECTION_INVALID");
                    break;
                }
                g_arcState = ARC_REVERSE_PAUSE;
                g_arcStateStartMs = nowMs;
                ArcPublish("FORWARD_DONE", 0U);
                if (g_arcRoundtripResult.targetLeft < 10) {
                    g_arcLowLeftTargetPending = 1U;
                }
            }
            break;

        case ARC_REVERSE_PAUSE:
            if (g_arcLowLeftTargetPending != 0U) {
                g_arcLowLeftTargetPending = 0U;
                ArcPublish("LOW_LEFT_TARGET", 0U);
            }
            if ((uint32_t)(nowMs - g_arcStateStartMs) >= ARC_REVERSE_PAUSE_MS) {
                g_arcState = ARC_REVERSE;
                g_arcStateStartMs = nowMs;
                g_arcRoundtripResult.reverseStartMs = nowMs;
                g_arcRoundtripResult.reverseStartLeft = encoder.totalLeft;
                g_arcRoundtripResult.reverseStartRight = encoder.totalRight;
                g_arcRoundtripResult.reverseStartSeq = encoder.sequence;
                g_arcRoundtripResult.reverseStartRxValid = encoder.validCount;
                g_arcLeftDone =
                    (g_arcRoundtripResult.targetLeft <= ARC_LEFT_TOLERANCE_TICKS) ?
                    1U : 0U;
                g_arcRightDone = 0U;
                g_arcRightBrakeState = ARC_RIGHT_FAST;
                g_arcLeftReachedPublished = 0U;
                g_arcRightReachedPublished = 0U;
                if (g_arcLeftDone != 0U) {
                    g_arcRoundtripResult.leftReachedMs = 0U;
                    g_arcLeftReachedPublished = 1U;
                }
                ArcPublish("REVERSE_START", 0U);
                if (g_arcLeftDone != 0U) {
                    ArcPublish("LEFT_TARGET_REACHED", 0U);
                }
            }
            break;

        case ARC_REVERSE:
            reverseLeft = encoder.totalLeft - g_arcRoundtripResult.reverseStartLeft;
            reverseRight = encoder.totalRight - g_arcRoundtripResult.reverseStartRight;
            rightTravel = -reverseRight;
            rightRemaining = g_arcRoundtripResult.targetRight - rightTravel;
            if (reverseLeft > ARC_DIRECTION_WRONG_TICKS ||
                reverseRight > ARC_DIRECTION_WRONG_TICKS) {
                ArcAbort("ENCODER_DIRECTION_WRONG");
                break;
            }
            if ((uint32_t)(nowMs - g_arcRoundtripResult.reverseStartMs) >=
                ARC_REVERSE_TIMEOUT_MS &&
                (g_arcLeftDone == 0U || g_arcRightDone == 0U)) {
                ArcAbort("REVERSE_TIMEOUT");
                break;
            }
            if (g_arcLeftDone == 0U &&
                reverseLeft <= -(g_arcRoundtripResult.targetLeft -
                                 ARC_LEFT_TOLERANCE_TICKS)) {
                g_arcLeftDone = 1U;
                g_arcRoundtripResult.leftReachedMs =
                    nowMs - g_arcRoundtripResult.reverseStartMs;
                if (g_arcLeftReachedPublished == 0U) {
                    g_arcLeftReachedPublished = 1U;
                    ArcPublish("LEFT_TARGET_REACHED", 0U);
                }
            }
            if (g_arcRightDone == 0U &&
                g_arcRightBrakeState == ARC_RIGHT_FAST &&
                rightRemaining <= ARC_RIGHT_SLOW_WINDOW_TICKS) {
                g_arcRightBrakeState = ARC_RIGHT_SLOW;
                g_arcRoundtripResult.rightSlowStartMs =
                    nowMs - g_arcRoundtripResult.reverseStartMs;
                g_arcRoundtripResult.rightSlowStartTravel = rightTravel;
                g_arcRoundtripResult.rightSlowStartRemaining = rightRemaining;
                ArcPublish("RIGHT_SLOW_START", 0U);
            } else if (g_arcRightDone == 0U &&
                       g_arcRightBrakeState == ARC_RIGHT_SLOW &&
                       rightRemaining <= ARC_RIGHT_STOP_LEAD_TICKS) {
                g_arcRightDone = 1U;
                g_arcRightBrakeState = ARC_RIGHT_DONE;
                g_arcRoundtripResult.rightReachedMs =
                    nowMs - g_arcRoundtripResult.reverseStartMs;
                g_arcRoundtripResult.rightStopTravel = rightTravel;
                g_arcRoundtripResult.rightStopRemaining = rightRemaining;
                if (g_arcRightReachedPublished == 0U) {
                    g_arcRightReachedPublished = 1U;
                    ArcPublish("RIGHT_BRAKE_STOP", 0U);
                }
            }
            if (g_arcLeftDone != 0U && g_arcRightDone != 0U) {
                g_arcRoundtripResult.reverseMotionStopMs = nowMs;
                g_arcRoundtripResult.reverseDurationMs =
                    nowMs - g_arcRoundtripResult.reverseStartMs;
                g_arcState = ARC_REVERSE_SETTLE;
                g_arcStateStartMs = nowMs;
                ArcPublish("REVERSE_STOP", 0U);
            } else {
                *leftCommand = (g_arcLeftDone != 0U) ? 0 :
                    ARC_REVERSE_LEFT_COMMAND;
                *rightCommand = (g_arcRightDone != 0U) ? 0 :
                    ((g_arcRightBrakeState == ARC_RIGHT_SLOW) ?
                     ARC_RIGHT_SLOW_COMMAND : ARC_REVERSE_RIGHT_COMMAND);
            }
            break;

        case ARC_REVERSE_SETTLE:
            if ((uint32_t)(nowMs - g_arcStateStartMs) >= ARC_REVERSE_SETTLE_MS) {
                g_arcRoundtripResult.reverseEndLeft = encoder.totalLeft;
                g_arcRoundtripResult.reverseEndRight = encoder.totalRight;
                g_arcRoundtripResult.reverseEndSeq = encoder.sequence;
                g_arcRoundtripResult.reverseEndRxValid = encoder.validCount;
                g_arcRoundtripResult.reverseTotalLeft =
                    encoder.totalLeft - g_arcRoundtripResult.reverseStartLeft;
                g_arcRoundtripResult.reverseTotalRight =
                    encoder.totalRight - g_arcRoundtripResult.reverseStartRight;
                g_arcRoundtripResult.reverseEncoderFrames =
                    encoder.validCount - g_arcRoundtripResult.reverseStartRxValid;
                g_arcRoundtripResult.errorLeft =
                    g_arcRoundtripResult.targetLeft +
                    g_arcRoundtripResult.reverseTotalLeft;
                g_arcRoundtripResult.errorRight =
                    g_arcRoundtripResult.targetRight +
                    g_arcRoundtripResult.reverseTotalRight;
                g_arcRoundtripResult.errorLeftPermille =
                    (g_arcRoundtripResult.targetLeft != 0) ?
                    (g_arcRoundtripResult.errorLeft * 1000 /
                     g_arcRoundtripResult.targetLeft) : 0;
                g_arcRoundtripResult.errorRightPermille =
                    (g_arcRoundtripResult.targetRight != 0) ?
                    (g_arcRoundtripResult.errorRight * 1000 /
                     g_arcRoundtripResult.targetRight) : 0;
                g_arcRoundtripResult.valid = 1U;
                g_arcState = ARC_DONE;
                g_arcLastSummaryMs = nowMs;
                ArcPublish("RESULT", 0U);
            }
            break;

        case ARC_DONE:
            if ((uint32_t)(nowMs - g_arcLastSummaryMs) >= ARC_SUMMARY_MS) {
                g_arcLastSummaryMs = nowMs;
                ArcPublish("SUMMARY", 0U);
            }
            break;

        case ARC_ABORT:
        default:
            break;
    }
}
