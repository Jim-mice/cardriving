#include <stdint.h>
#include <stdio.h>

#include "app_time.h"
#include "task_motor_response.h"
#include "udp_telemetry.h"

#define MOTOR_RESPONSE_ACTIVE_MS 500U
#define MOTOR_RESPONSE_SETTLE_MS 800U
#define MOTOR_RESPONSE_STALE_MS 200U
#define MOTOR_RESPONSE_STEADY_SAMPLES 5U

typedef struct {
    int left;
    int right;
} MotorResponsePhase;

typedef enum {
    MOTOR_RESPONSE_IDLE = 0,
    MOTOR_RESPONSE_PRE_SETTLE,
    MOTOR_RESPONSE_ACTIVE,
    MOTOR_RESPONSE_POST_SETTLE
} MotorResponseState;

static const MotorResponsePhase g_motorResponsePhases[] = {
    {85, 85}, {90, 90}, {95, 95}, {97, 97}, {99, 99}, {100, 100},
    {90, 110}, {110, 90}, {95, 110}, {110, 95},
    {90, 120}, {120, 90}, {95, 120}, {120, 95}
};

static MotorResponseState g_motorResponseState;
static uint32_t g_motorResponseStateStartMs;
static uint32_t g_motorResponsePhaseIndex;
static int32_t g_motorResponseStartLeft;
static int32_t g_motorResponseStartRight;
static int32_t g_motorResponseCommandEndLeft;
static int32_t g_motorResponseCommandEndRight;
static uint32_t g_motorResponseLastValidCount;
static uint32_t g_motorResponseSamples;
static int16_t g_motorResponseSteadyLeft[MOTOR_RESPONSE_STEADY_SAMPLES];
static int16_t g_motorResponseSteadyRight[MOTOR_RESPONSE_STEADY_SAMPLES];
static uint8_t g_motorResponseSteadyCount;
static uint8_t g_motorResponseSteadyNext;
static int32_t g_motorResponseFirstMoveMsLeft;
static int32_t g_motorResponseFirstMoveMsRight;

static int16_t MotorResponseAbs(int16_t value)
{
    return value < 0 ? (int16_t)-value : value;
}

static int MotorResponseEncoderFresh(const UdpEncoderTelemetryState *encoder,
                                     uint32_t nowMs)
{
    return encoder->validCount != 0U &&
           (uint32_t)(nowMs - encoder->lastRxMs) <= MOTOR_RESPONSE_STALE_MS;
}

static void MotorResponsePublish(const char *event)
{
    (void)UdpTelemetryQueueExperimentText(event);
}

static void MotorResponseAbort(const char *reason)
{
    char text[80];

    (void)snprintf(text, sizeof(text), "MOTORCAL event=ABORT reason=%s", reason);
    MotorResponsePublish(text);
    g_motorResponseState = MOTOR_RESPONSE_IDLE;
}

static void MotorResponseStartPhase(const UdpEncoderTelemetryState *encoder,
                                    uint32_t nowMs)
{
    char text[112];
    const MotorResponsePhase *phase = &g_motorResponsePhases[g_motorResponsePhaseIndex];

    g_motorResponseStartLeft = encoder->totalLeft;
    g_motorResponseStartRight = encoder->totalRight;
    g_motorResponseLastValidCount = encoder->validCount;
    g_motorResponseSamples = 0U;
    g_motorResponseSteadyCount = 0U;
    g_motorResponseSteadyNext = 0U;
    g_motorResponseFirstMoveMsLeft = -1;
    g_motorResponseFirstMoveMsRight = -1;
    g_motorResponseStateStartMs = nowMs;
    g_motorResponseState = MOTOR_RESPONSE_ACTIVE;
    (void)snprintf(text, sizeof(text),
                   "MOTORCAL event=PHASE_START idx=%u cmd_l=%d cmd_r=%d",
                   (unsigned int)(g_motorResponsePhaseIndex + 1U),
                   phase->left, phase->right);
    MotorResponsePublish(text);
}

static int32_t MotorResponseSteadyAverage(const int16_t *values, uint8_t count)
{
    int32_t sum = 0;
    uint8_t index;

    if (count == 0U) {
        return 0;
    }
    for (index = 0U; index < count; index++) {
        sum += values[index];
    }
    return sum / (int32_t)count;
}

static void MotorResponsePublishResult(const UdpEncoderTelemetryState *encoder)
{
    char text[220];
    const MotorResponsePhase *phase = &g_motorResponsePhases[g_motorResponsePhaseIndex];
    int32_t activeLeft = g_motorResponseCommandEndLeft - g_motorResponseStartLeft;
    int32_t activeRight = g_motorResponseCommandEndRight - g_motorResponseStartRight;
    int32_t coastLeft = encoder->totalLeft - g_motorResponseCommandEndLeft;
    int32_t coastRight = encoder->totalRight - g_motorResponseCommandEndRight;

    (void)snprintf(text, sizeof(text),
                   "MOTORCAL event=RESULT idx=%u cmd_l=%d cmd_r=%d samples=%u active_l=%ld active_r=%ld steady_l=%ld steady_r=%ld coast_l=%ld coast_r=%ld first_move_ms_l=%ld first_move_ms_r=%ld",
                   (unsigned int)(g_motorResponsePhaseIndex + 1U),
                   phase->left, phase->right,
                   (unsigned int)g_motorResponseSamples,
                   (long)activeLeft, (long)activeRight,
                   (long)MotorResponseSteadyAverage(g_motorResponseSteadyLeft,
                                                     g_motorResponseSteadyCount),
                   (long)MotorResponseSteadyAverage(g_motorResponseSteadyRight,
                                                     g_motorResponseSteadyCount),
                   (long)coastLeft, (long)coastRight,
                   (long)g_motorResponseFirstMoveMsLeft,
                   (long)g_motorResponseFirstMoveMsRight);
    MotorResponsePublish(text);
}

void MotorResponseInit(void)
{
    g_motorResponseState = MOTOR_RESPONSE_IDLE;
    g_motorResponseStateStartMs = 0U;
    g_motorResponsePhaseIndex = 0U;
    g_motorResponseStartLeft = 0;
    g_motorResponseStartRight = 0;
    g_motorResponseCommandEndLeft = 0;
    g_motorResponseCommandEndRight = 0;
    g_motorResponseLastValidCount = 0U;
    g_motorResponseSamples = 0U;
    g_motorResponseSteadyCount = 0U;
    g_motorResponseSteadyNext = 0U;
    g_motorResponseFirstMoveMsLeft = -1;
    g_motorResponseFirstMoveMsRight = -1;
}

void MotorResponseStep(uint32_t nowTicks, MotorResponseCommand command,
                       int *leftCommand, int *rightCommand)
{
    UdpEncoderTelemetryState encoder;
    uint32_t nowMs = AppTicksToMs(nowTicks);

    *leftCommand = 0;
    *rightCommand = 0;
    UdpTelemetryReadEncoder(&encoder);

    if (command == MOTOR_RESPONSE_COMMAND_STOP) {
        if (g_motorResponseState != MOTOR_RESPONSE_IDLE) {
            MotorResponseAbort("USER_STOP");
        }
        return;
    }
    if (command == MOTOR_RESPONSE_COMMAND_START) {
        if (g_motorResponseState == MOTOR_RESPONSE_IDLE) {
            if (MotorResponseEncoderFresh(&encoder, nowMs) == 0) {
                MotorResponseAbort("ENCODER_STALE");
                return;
            }
            g_motorResponsePhaseIndex = 0U;
            g_motorResponseStateStartMs = nowMs;
            g_motorResponseState = MOTOR_RESPONSE_PRE_SETTLE;
            MotorResponsePublish("MOTORCAL event=START");
        }
        return;
    }
    if (g_motorResponseState == MOTOR_RESPONSE_IDLE) {
        return;
    }
    if (MotorResponseEncoderFresh(&encoder, nowMs) == 0) {
        MotorResponseAbort("ENCODER_STALE");
        return;
    }

    if (g_motorResponseState == MOTOR_RESPONSE_PRE_SETTLE) {
        if ((uint32_t)(nowMs - g_motorResponseStateStartMs) >=
            MOTOR_RESPONSE_SETTLE_MS) {
            MotorResponseStartPhase(&encoder, nowMs);
            *leftCommand = g_motorResponsePhases[g_motorResponsePhaseIndex].left;
            *rightCommand = g_motorResponsePhases[g_motorResponsePhaseIndex].right;
        }
    } else if (g_motorResponseState == MOTOR_RESPONSE_ACTIVE) {
        const MotorResponsePhase *phase = &g_motorResponsePhases[g_motorResponsePhaseIndex];

        if (encoder.validCount != g_motorResponseLastValidCount) {
            g_motorResponseLastValidCount = encoder.validCount;
            g_motorResponseSamples++;
            g_motorResponseSteadyLeft[g_motorResponseSteadyNext] = encoder.leftDelta;
            g_motorResponseSteadyRight[g_motorResponseSteadyNext] = encoder.rightDelta;
            if (g_motorResponseFirstMoveMsLeft < 0 &&
                MotorResponseAbs(encoder.leftDelta) > 0) {
                g_motorResponseFirstMoveMsLeft =
                    (int32_t)(nowMs - g_motorResponseStateStartMs);
            }
            if (g_motorResponseFirstMoveMsRight < 0 &&
                MotorResponseAbs(encoder.rightDelta) > 0) {
                g_motorResponseFirstMoveMsRight =
                    (int32_t)(nowMs - g_motorResponseStateStartMs);
            }
            g_motorResponseSteadyNext =
                (uint8_t)((g_motorResponseSteadyNext + 1U) % MOTOR_RESPONSE_STEADY_SAMPLES);
            if (g_motorResponseSteadyCount < MOTOR_RESPONSE_STEADY_SAMPLES) {
                g_motorResponseSteadyCount++;
            }
        }
        if ((uint32_t)(nowMs - g_motorResponseStateStartMs) >=
            MOTOR_RESPONSE_ACTIVE_MS) {
            g_motorResponseCommandEndLeft = encoder.totalLeft;
            g_motorResponseCommandEndRight = encoder.totalRight;
            g_motorResponseStateStartMs = nowMs;
            g_motorResponseState = MOTOR_RESPONSE_POST_SETTLE;
        } else {
            *leftCommand = phase->left;
            *rightCommand = phase->right;
        }
    } else if (g_motorResponseState == MOTOR_RESPONSE_POST_SETTLE) {
        if ((uint32_t)(nowMs - g_motorResponseStateStartMs) >=
            MOTOR_RESPONSE_SETTLE_MS) {
            MotorResponsePublishResult(&encoder);
            g_motorResponsePhaseIndex++;
            if (g_motorResponsePhaseIndex >=
                (uint32_t)(sizeof(g_motorResponsePhases) / sizeof(g_motorResponsePhases[0]))) {
                MotorResponsePublish("MOTORCAL event=DONE");
                g_motorResponseState = MOTOR_RESPONSE_IDLE;
            } else {
                MotorResponseStartPhase(&encoder, nowMs);
                *leftCommand = g_motorResponsePhases[g_motorResponsePhaseIndex].left;
                *rightCommand = g_motorResponsePhases[g_motorResponsePhaseIndex].right;
            }
        }
    } else {
        MotorResponseAbort("INTERNAL_STATE");
        return;
    }
}
