#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "app_time.h"
#include "cmsis_os2.h"
#include "teach_follow.h"
#include "teach_route.h"
#include "udp_telemetry.h"

extern void stm32motor_control(int motorA, int motorB);

#define TEACH_FOLLOW_PERIOD_MS 20U
#define TEACH_FOLLOW_STACK_SIZE 2048U
#define TEACH_FOLLOW_ENCODER_TIMEOUT_MS 250U
#define TEACH_FOLLOW_TOLERANCE_TICKS 100
#define TEACH_FOLLOW_BASE_PWM 80
#define TEACH_FOLLOW_MIN_PWM 45
#define TEACH_FOLLOW_MAX_PWM 120
#define TEACH_FOLLOW_MAX_TURN_PWM 35
#define TEACH_FOLLOW_DISTANCE_DIVISOR 12
#define TEACH_FOLLOW_TURN_DIVISOR 10

static volatile TeachFollowState g_state;
static uint32_t g_waypointIndex;
static int32_t g_startLeft;
static int32_t g_startRight;

static int TeachFollowClamp(int value, int lower, int upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static int TeachFollowEncoderFresh(const UdpEncoderTelemetryState *encoder,
                                   uint32_t nowMs)
{
    return encoder->validCount != 0U &&
        (uint32_t)(nowMs - encoder->lastRxMs) <= TEACH_FOLLOW_ENCODER_TIMEOUT_MS;
}

static void TeachFollowStop(const char *reason)
{
    stm32motor_control(0, 0);
    g_state = TEACH_FOLLOW_DONE;
    printf("[TEACH_FOLLOW] %s index=%u/%u\r\n", reason,
           (unsigned int)g_waypointIndex, (unsigned int)TEACH_ROUTE_COUNT);
}

static void TeachFollowStep(uint32_t nowMs)
{
    UdpEncoderTelemetryState encoder;
    const TeachWaypoint *target;
    int32_t currentLeft;
    int32_t currentRight;
    int32_t errorLeft;
    int32_t errorRight;
    int commonError;
    int differenceError;
    int commonPwm;
    int turnPwm;
    int leftPwm;
    int rightPwm;

    if (g_state != TEACH_FOLLOW_RUNNING) {
        return;
    }

    UdpTelemetryReadEncoder(&encoder);
    if (!TeachFollowEncoderFresh(&encoder, nowMs)) {
        TeachFollowStop("encoder timeout");
        return;
    }

    if (g_waypointIndex >= TEACH_ROUTE_COUNT) {
        TeachFollowStop("finished");
        return;
    }

    target = &g_teach_route[g_waypointIndex];
    currentLeft = encoder.totalLeft - g_startLeft;
    currentRight = encoder.totalRight - g_startRight;
    errorLeft = target->targetLeft - currentLeft;
    errorRight = target->targetRight - currentRight;

    if (abs(errorLeft) <= TEACH_FOLLOW_TOLERANCE_TICKS &&
        abs(errorRight) <= TEACH_FOLLOW_TOLERANCE_TICKS) {
        g_waypointIndex++;
        return;
    }

    commonError = (int)((errorLeft + errorRight) / 2);
    differenceError = (int)(errorLeft - errorRight);
    commonPwm = TeachFollowClamp(commonError / TEACH_FOLLOW_DISTANCE_DIVISOR,
                                  TEACH_FOLLOW_MIN_PWM, TEACH_FOLLOW_BASE_PWM);
    turnPwm = TeachFollowClamp(differenceError / TEACH_FOLLOW_TURN_DIVISOR,
                                -TEACH_FOLLOW_MAX_TURN_PWM,
                                TEACH_FOLLOW_MAX_TURN_PWM);
    leftPwm = TeachFollowClamp(commonPwm + turnPwm, 0, TEACH_FOLLOW_MAX_PWM);
    rightPwm = TeachFollowClamp(commonPwm - turnPwm, 0, TEACH_FOLLOW_MAX_PWM);
    if (leftPwm > 0 && leftPwm < 80) {
        leftPwm = 80;
    } else if (leftPwm < 0 && leftPwm > -80) {
        leftPwm = -80;
    }
    if (rightPwm > 0 && rightPwm < 80) {
        rightPwm = 80;
    } else if (rightPwm < 0 && rightPwm > -80) {
        rightPwm = -80;
    }
    stm32motor_control(leftPwm, rightPwm);
}

static void TeachFollowTask(void *argument)
{
    (void)argument;
    for (;;) {
        TeachFollowStep(AppTicksToMs(osKernelGetTickCount()));
        osDelay(AppMsToTicks(TEACH_FOLLOW_PERIOD_MS));
    }
}

int TeachFollowStart(void)
{
    UdpEncoderTelemetryState encoder;
    uint32_t nowMs = AppTicksToMs(osKernelGetTickCount());

    UdpTelemetryReadEncoder(&encoder);
    if (!TeachFollowEncoderFresh(&encoder, nowMs)) {
        stm32motor_control(0, 0);
        printf("[TEACH_FOLLOW] start rejected: encoder unavailable\r\n");
        return -1;
    }

    stm32motor_control(0, 0);
    g_startLeft = encoder.totalLeft;
    g_startRight = encoder.totalRight;
    g_waypointIndex = 0U;
    g_state = TEACH_FOLLOW_RUNNING;
    printf("[TEACH_FOLLOW] start count=%u left=%ld right=%ld\r\n",
           (unsigned int)TEACH_ROUTE_COUNT, (long)g_startLeft, (long)g_startRight);
    return 0;
}

TeachFollowState TeachFollowGetState(void)
{
    return g_state;
}

int TeachFollowInit(void)
{
    static const osThreadAttr_t attr = {
        .name = "teach_follow",
        .stack_size = TEACH_FOLLOW_STACK_SIZE,
        .priority = osPriorityNormal,
    };

    g_state = TEACH_FOLLOW_IDLE;
    return osThreadNew(TeachFollowTask, NULL, &attr) == NULL ? -1 : 0;
}
