#include <stdint.h>
#include <stdio.h>

#include "app_time.h"
#include "task_encoder_right_arc_roundtrip.h"
#include "udp_telemetry.h"

#define RARC_FORWARD_LEFT_COMMAND       120
#define RARC_FORWARD_RIGHT_COMMAND       60
#define RARC_FORWARD_MS                 300U
#define RARC_FORWARD_SETTLE_MS          800U
#define RARC_REVERSE_PAUSE_MS           300U
#define RARC_REVERSE_LEFT_FAST_COMMAND -120
#define RARC_REVERSE_LEFT_SLOW_COMMAND -100
#define RARC_REVERSE_RIGHT_COMMAND      -60
#define RARC_REVERSE_SETTLE_MS          800U
#define RARC_REVERSE_TIMEOUT_MS        2500U
#define RARC_ENCODER_READY_FRAMES        20U
#define RARC_ENCODER_READY_MAX_AGE_MS   100U
#define RARC_ENCODER_RX_TIMEOUT_MS      200U
#define RARC_RIGHT_TOLERANCE_TICKS        5
#define RARC_LEFT_SLOW_WINDOW_TICKS     500
#define RARC_LEFT_STOP_LEAD_TICKS        60
#define RARC_DIRECTION_WRONG_TICKS       50
#define RARC_ARM_MS                     5000U
#define RARC_ARMING_EVENT_MS            1000U
#define RARC_SUMMARY_MS                 1000U

typedef enum {
    RARC_WAIT_ENCODER = 0,
    RARC_ARM,
    RARC_FORWARD,
    RARC_FORWARD_SETTLE,
    RARC_REVERSE_PAUSE,
    RARC_REVERSE,
    RARC_REVERSE_SETTLE,
    RARC_DONE,
    RARC_ABORT
} RightArcRoundtripState;

typedef enum {
    RARC_LEFT_FAST = 0,
    RARC_LEFT_SLOW,
    RARC_LEFT_DONE
} RightArcLeftBrakeState;

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
    uint32_t leftSlowStartMs;
    int32_t leftSlowStartTravel;
    int32_t leftSlowStartRemaining;
    int32_t leftStopTravel;
    int32_t leftStopRemaining;
    uint8_t forwardStartSeq;
    uint8_t forwardEndSeq;
    uint8_t reverseStartSeq;
    uint8_t reverseEndSeq;
} RightArcRoundtripResult;

static RightArcRoundtripState g_rightArcState;
static RightArcRoundtripResult g_rightArcRoundtripResult;
static uint32_t g_rightArcStateStartMs;
static uint32_t g_rightArcReadyBaselineValidCount;
static uint32_t g_rightArcLastArmingEventMs;
static uint32_t g_rightArcLastSummaryMs;
static uint8_t g_rightArcLeftDone;
static uint8_t g_rightArcRightDone;
static uint8_t g_rightArcLeftReachedPublished;
static uint8_t g_rightArcRightReachedPublished;
static uint8_t g_rightArcLowRightTargetPending;
static RightArcLeftBrakeState g_rightArcLeftBrakeState;
static char g_rightArcTelemetryText[768];

static void RightArcPublish(const char *event, uint32_t remainingMs)
{
    (void)snprintf(g_rightArcTelemetryText, sizeof(g_rightArcTelemetryText),
                   "RARC event=%s target_l=%ld target_r=%ld reverse_l=%ld reverse_r=%ld "
                   "error_l=%ld error_r=%ld error_l_permille=%ld error_r_permille=%ld "
                   "forward_ms=%u reverse_ms=%u forward_frames=%u reverse_frames=%u "
                   "left_reached_ms=%u right_reached_ms=%u f_start_seq=%u f_end_seq=%u "
                   "r_start_seq=%u r_end_seq=%u valid=%u aborted=%u reason=%s "
                   "left_done=%u right_done=%u remaining_ms=%u "
                   "left_slow_start_ms=%u left_slow_start_travel=%ld "
                   "left_slow_start_remaining=%ld left_stop_travel=%ld "
                   "left_stop_remaining=%ld",
                   event,
                   (long)g_rightArcRoundtripResult.targetLeft,
                   (long)g_rightArcRoundtripResult.targetRight,
                   (long)g_rightArcRoundtripResult.reverseTotalLeft,
                   (long)g_rightArcRoundtripResult.reverseTotalRight,
                   (long)g_rightArcRoundtripResult.errorLeft,
                   (long)g_rightArcRoundtripResult.errorRight,
                   (long)g_rightArcRoundtripResult.errorLeftPermille,
                   (long)g_rightArcRoundtripResult.errorRightPermille,
                   (unsigned int)g_rightArcRoundtripResult.forwardDurationMs,
                   (unsigned int)g_rightArcRoundtripResult.reverseDurationMs,
                   (unsigned int)g_rightArcRoundtripResult.forwardEncoderFrames,
                   (unsigned int)g_rightArcRoundtripResult.reverseEncoderFrames,
                   (unsigned int)g_rightArcRoundtripResult.leftReachedMs,
                   (unsigned int)g_rightArcRoundtripResult.rightReachedMs,
                   (unsigned int)g_rightArcRoundtripResult.forwardStartSeq,
                   (unsigned int)g_rightArcRoundtripResult.forwardEndSeq,
                   (unsigned int)g_rightArcRoundtripResult.reverseStartSeq,
                   (unsigned int)g_rightArcRoundtripResult.reverseEndSeq,
                   (unsigned int)g_rightArcRoundtripResult.valid,
                   (unsigned int)g_rightArcRoundtripResult.aborted,
                   (g_rightArcRoundtripResult.abortReason != NULL) ?
                       g_rightArcRoundtripResult.abortReason : "NONE",
                   (unsigned int)g_rightArcLeftDone,
                   (unsigned int)g_rightArcRightDone,
                   (unsigned int)remainingMs,
                   (unsigned int)g_rightArcRoundtripResult.leftSlowStartMs,
                   (long)g_rightArcRoundtripResult.leftSlowStartTravel,
                   (long)g_rightArcRoundtripResult.leftSlowStartRemaining,
                   (long)g_rightArcRoundtripResult.leftStopTravel,
                   (long)g_rightArcRoundtripResult.leftStopRemaining);
    UdpTelemetryPublishCal(g_rightArcTelemetryText);
}

static void RightArcAbort(const char *reason)
{
    if (g_rightArcState == RARC_ABORT) {
        return;
    }
    g_rightArcRoundtripResult.aborted = 1U;
    g_rightArcRoundtripResult.abortReason = reason;
    g_rightArcState = RARC_ABORT;
    RightArcPublish("ABORT", 0U);
}

static int RightArcEncoderFresh(const UdpEncoderTelemetryState *encoder,
                                uint32_t nowMs, uint32_t maximumAgeMs)
{
    return (encoder->validCount != 0U &&
            (uint32_t)(nowMs - encoder->lastRxMs) <= maximumAgeMs) ? 1 : 0;
}

void RightArcRoundtripInit(void)
{
    g_rightArcState = RARC_WAIT_ENCODER;
    g_rightArcRoundtripResult = (RightArcRoundtripResult){0};
    g_rightArcStateStartMs = 0U;
    g_rightArcReadyBaselineValidCount = 0U;
    g_rightArcLastArmingEventMs = 0U;
    g_rightArcLastSummaryMs = 0U;
    g_rightArcLeftDone = 0U;
    g_rightArcRightDone = 0U;
    g_rightArcLeftReachedPublished = 0U;
    g_rightArcRightReachedPublished = 0U;
    g_rightArcLowRightTargetPending = 0U;
    g_rightArcLeftBrakeState = RARC_LEFT_FAST;
    RightArcPublish("BOOT", 0U);
    RightArcPublish("WAIT_ENCODER", 0U);
}

void RightArcRoundtripStep(uint32_t now, int *leftCommand, int *rightCommand)
{
    UdpEncoderTelemetryState encoder;
    uint32_t nowMs = AppTicksToMs(now);
    int32_t reverseLeft;
    int32_t reverseRight;
    int32_t leftTravel;
    int32_t leftRemaining;

    if (leftCommand == NULL || rightCommand == NULL) {
        return;
    }
    *leftCommand = 0;
    *rightCommand = 0;
    UdpTelemetryReadEncoder(&encoder);

    if ((g_rightArcState == RARC_FORWARD || g_rightArcState == RARC_REVERSE) &&
        RightArcEncoderFresh(&encoder, nowMs, RARC_ENCODER_RX_TIMEOUT_MS) == 0) {
        RightArcAbort("ENCODER_RX_TIMEOUT");
        return;
    }

    switch (g_rightArcState) {
        case RARC_WAIT_ENCODER:
            if (g_rightArcReadyBaselineValidCount == 0U) {
                g_rightArcReadyBaselineValidCount = encoder.validCount;
            }
            if (encoder.validCount - g_rightArcReadyBaselineValidCount >=
                    RARC_ENCODER_READY_FRAMES &&
                RightArcEncoderFresh(&encoder, nowMs,
                                     RARC_ENCODER_READY_MAX_AGE_MS) != 0) {
                g_rightArcState = RARC_ARM;
                g_rightArcStateStartMs = nowMs;
                g_rightArcLastArmingEventMs = nowMs;
                RightArcPublish("ENCODER_READY", RARC_ARM_MS);
            }
            break;

        case RARC_ARM:
            if ((uint32_t)(nowMs - g_rightArcStateStartMs) >= RARC_ARM_MS) {
                g_rightArcState = RARC_FORWARD;
                g_rightArcStateStartMs = nowMs;
                g_rightArcRoundtripResult.forwardStartMs = nowMs;
                g_rightArcRoundtripResult.forwardStartLeft = encoder.totalLeft;
                g_rightArcRoundtripResult.forwardStartRight = encoder.totalRight;
                g_rightArcRoundtripResult.forwardStartSeq = encoder.sequence;
                g_rightArcRoundtripResult.forwardStartRxValid = encoder.validCount;
                g_rightArcRoundtripResult.forwardDurationMs = RARC_FORWARD_MS;
                RightArcPublish("FORWARD_START", 0U);
                *leftCommand = RARC_FORWARD_LEFT_COMMAND;
                *rightCommand = RARC_FORWARD_RIGHT_COMMAND;
            } else if ((uint32_t)(nowMs - g_rightArcLastArmingEventMs) >=
                       RARC_ARMING_EVENT_MS) {
                uint32_t elapsedMs = nowMs - g_rightArcStateStartMs;
                g_rightArcLastArmingEventMs = nowMs;
                RightArcPublish("ARMING", RARC_ARM_MS - elapsedMs);
            }
            break;

        case RARC_FORWARD:
            if ((uint32_t)(nowMs - g_rightArcStateStartMs) >= RARC_FORWARD_MS) {
                g_rightArcState = RARC_FORWARD_SETTLE;
                g_rightArcStateStartMs = nowMs;
                RightArcPublish("FORWARD_STOP", 0U);
            } else {
                *leftCommand = RARC_FORWARD_LEFT_COMMAND;
                *rightCommand = RARC_FORWARD_RIGHT_COMMAND;
            }
            break;

        case RARC_FORWARD_SETTLE:
            if ((uint32_t)(nowMs - g_rightArcStateStartMs) >=
                RARC_FORWARD_SETTLE_MS) {
                g_rightArcRoundtripResult.forwardEndLeft = encoder.totalLeft;
                g_rightArcRoundtripResult.forwardEndRight = encoder.totalRight;
                g_rightArcRoundtripResult.forwardEndSeq = encoder.sequence;
                g_rightArcRoundtripResult.forwardEndRxValid = encoder.validCount;
                g_rightArcRoundtripResult.targetLeft =
                    encoder.totalLeft - g_rightArcRoundtripResult.forwardStartLeft;
                g_rightArcRoundtripResult.targetRight =
                    encoder.totalRight - g_rightArcRoundtripResult.forwardStartRight;
                g_rightArcRoundtripResult.forwardEncoderFrames =
                    encoder.validCount - g_rightArcRoundtripResult.forwardStartRxValid;
                if (g_rightArcRoundtripResult.targetLeft <= 0 ||
                    g_rightArcRoundtripResult.targetRight < 0) {
                    RightArcAbort("FORWARD_DIRECTION_INVALID");
                    break;
                }
                g_rightArcState = RARC_REVERSE_PAUSE;
                g_rightArcStateStartMs = nowMs;
                RightArcPublish("FORWARD_DONE", 0U);
                if (g_rightArcRoundtripResult.targetRight < 10) {
                    g_rightArcLowRightTargetPending = 1U;
                }
            }
            break;

        case RARC_REVERSE_PAUSE:
            if (g_rightArcLowRightTargetPending != 0U) {
                g_rightArcLowRightTargetPending = 0U;
                RightArcPublish("LOW_RIGHT_TARGET", 0U);
            }
            if ((uint32_t)(nowMs - g_rightArcStateStartMs) >=
                RARC_REVERSE_PAUSE_MS) {
                g_rightArcState = RARC_REVERSE;
                g_rightArcStateStartMs = nowMs;
                g_rightArcRoundtripResult.reverseStartMs = nowMs;
                g_rightArcRoundtripResult.reverseStartLeft = encoder.totalLeft;
                g_rightArcRoundtripResult.reverseStartRight = encoder.totalRight;
                g_rightArcRoundtripResult.reverseStartSeq = encoder.sequence;
                g_rightArcRoundtripResult.reverseStartRxValid = encoder.validCount;
                g_rightArcLeftDone = 0U;
                g_rightArcRightDone =
                    (g_rightArcRoundtripResult.targetRight <=
                     RARC_RIGHT_TOLERANCE_TICKS) ? 1U : 0U;
                g_rightArcLeftBrakeState = RARC_LEFT_FAST;
                g_rightArcLeftReachedPublished = 0U;
                g_rightArcRightReachedPublished = 0U;
                if (g_rightArcRightDone != 0U) {
                    g_rightArcRoundtripResult.rightReachedMs = 0U;
                    g_rightArcRightReachedPublished = 1U;
                }
                RightArcPublish("REVERSE_START", 0U);
                if (g_rightArcRightDone != 0U) {
                    RightArcPublish("RIGHT_TARGET_REACHED", 0U);
                }
            }
            break;

        case RARC_REVERSE:
            reverseLeft = encoder.totalLeft - g_rightArcRoundtripResult.reverseStartLeft;
            reverseRight = encoder.totalRight - g_rightArcRoundtripResult.reverseStartRight;
            leftTravel = -reverseLeft;
            leftRemaining = g_rightArcRoundtripResult.targetLeft - leftTravel;
            if (reverseLeft > RARC_DIRECTION_WRONG_TICKS ||
                reverseRight > RARC_DIRECTION_WRONG_TICKS) {
                RightArcAbort("ENCODER_DIRECTION_WRONG");
                break;
            }
            if ((uint32_t)(nowMs - g_rightArcRoundtripResult.reverseStartMs) >=
                    RARC_REVERSE_TIMEOUT_MS &&
                (g_rightArcLeftDone == 0U || g_rightArcRightDone == 0U)) {
                RightArcAbort("REVERSE_TIMEOUT");
                break;
            }
            if (g_rightArcRightDone == 0U &&
                reverseRight <= -(g_rightArcRoundtripResult.targetRight -
                                  RARC_RIGHT_TOLERANCE_TICKS)) {
                g_rightArcRightDone = 1U;
                g_rightArcRoundtripResult.rightReachedMs =
                    nowMs - g_rightArcRoundtripResult.reverseStartMs;
                if (g_rightArcRightReachedPublished == 0U) {
                    g_rightArcRightReachedPublished = 1U;
                    RightArcPublish("RIGHT_TARGET_REACHED", 0U);
                }
            }
            if (g_rightArcLeftDone == 0U &&
                g_rightArcLeftBrakeState == RARC_LEFT_FAST &&
                leftRemaining <= RARC_LEFT_SLOW_WINDOW_TICKS) {
                g_rightArcLeftBrakeState = RARC_LEFT_SLOW;
                g_rightArcRoundtripResult.leftSlowStartMs =
                    nowMs - g_rightArcRoundtripResult.reverseStartMs;
                g_rightArcRoundtripResult.leftSlowStartTravel = leftTravel;
                g_rightArcRoundtripResult.leftSlowStartRemaining = leftRemaining;
                RightArcPublish("LEFT_SLOW_START", 0U);
            } else if (g_rightArcLeftDone == 0U &&
                       g_rightArcLeftBrakeState == RARC_LEFT_SLOW &&
                       leftRemaining <= RARC_LEFT_STOP_LEAD_TICKS) {
                g_rightArcLeftDone = 1U;
                g_rightArcLeftBrakeState = RARC_LEFT_DONE;
                g_rightArcRoundtripResult.leftReachedMs =
                    nowMs - g_rightArcRoundtripResult.reverseStartMs;
                g_rightArcRoundtripResult.leftStopTravel = leftTravel;
                g_rightArcRoundtripResult.leftStopRemaining = leftRemaining;
                if (g_rightArcLeftReachedPublished == 0U) {
                    g_rightArcLeftReachedPublished = 1U;
                    RightArcPublish("LEFT_BRAKE_STOP", 0U);
                }
            }
            if (g_rightArcLeftDone != 0U && g_rightArcRightDone != 0U) {
                g_rightArcRoundtripResult.reverseMotionStopMs = nowMs;
                g_rightArcRoundtripResult.reverseDurationMs =
                    nowMs - g_rightArcRoundtripResult.reverseStartMs;
                g_rightArcState = RARC_REVERSE_SETTLE;
                g_rightArcStateStartMs = nowMs;
                RightArcPublish("REVERSE_STOP", 0U);
            } else {
                *leftCommand = (g_rightArcLeftDone != 0U) ? 0 :
                    ((g_rightArcLeftBrakeState == RARC_LEFT_SLOW) ?
                     RARC_REVERSE_LEFT_SLOW_COMMAND :
                     RARC_REVERSE_LEFT_FAST_COMMAND);
                *rightCommand = (g_rightArcRightDone != 0U) ? 0 :
                    RARC_REVERSE_RIGHT_COMMAND;
            }
            break;

        case RARC_REVERSE_SETTLE:
            if ((uint32_t)(nowMs - g_rightArcStateStartMs) >=
                RARC_REVERSE_SETTLE_MS) {
                g_rightArcRoundtripResult.reverseEndLeft = encoder.totalLeft;
                g_rightArcRoundtripResult.reverseEndRight = encoder.totalRight;
                g_rightArcRoundtripResult.reverseEndSeq = encoder.sequence;
                g_rightArcRoundtripResult.reverseEndRxValid = encoder.validCount;
                g_rightArcRoundtripResult.reverseTotalLeft =
                    encoder.totalLeft - g_rightArcRoundtripResult.reverseStartLeft;
                g_rightArcRoundtripResult.reverseTotalRight =
                    encoder.totalRight - g_rightArcRoundtripResult.reverseStartRight;
                g_rightArcRoundtripResult.reverseEncoderFrames =
                    encoder.validCount - g_rightArcRoundtripResult.reverseStartRxValid;
                g_rightArcRoundtripResult.errorLeft =
                    g_rightArcRoundtripResult.targetLeft +
                    g_rightArcRoundtripResult.reverseTotalLeft;
                g_rightArcRoundtripResult.errorRight =
                    g_rightArcRoundtripResult.targetRight +
                    g_rightArcRoundtripResult.reverseTotalRight;
                g_rightArcRoundtripResult.errorLeftPermille =
                    (g_rightArcRoundtripResult.targetLeft != 0) ?
                    (g_rightArcRoundtripResult.errorLeft * 1000 /
                     g_rightArcRoundtripResult.targetLeft) : 0;
                g_rightArcRoundtripResult.errorRightPermille =
                    (g_rightArcRoundtripResult.targetRight != 0) ?
                    (g_rightArcRoundtripResult.errorRight * 1000 /
                     g_rightArcRoundtripResult.targetRight) : 0;
                g_rightArcRoundtripResult.valid = 1U;
                g_rightArcState = RARC_DONE;
                g_rightArcLastSummaryMs = nowMs;
                RightArcPublish("RESULT", 0U);
            }
            break;

        case RARC_DONE:
            if ((uint32_t)(nowMs - g_rightArcLastSummaryMs) >= RARC_SUMMARY_MS) {
                g_rightArcLastSummaryMs = nowMs;
                RightArcPublish("SUMMARY", 0U);
            }
            break;

        case RARC_ABORT:
        default:
            break;
    }
}
