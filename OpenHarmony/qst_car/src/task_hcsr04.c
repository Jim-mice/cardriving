#include <stdio.h>

#include "cmsis_os2.h"
#include "app_time.h"
#include "task_hcsr04.h"
#include "task_car_control.h"

extern void engine_turn_left(void);
extern void engine_turn_right(void);
extern void regress_middle(void);
extern float GetDistance(void);

#define HCSR04_SERVO_SETTLE_DELAY_MS 500
#define HCSR04_SCAN_DELAY_MS         1000

static volatile Hcsr04Angle g_angle;
static volatile float g_distance;
static volatile int g_hcsr04DataValid;
static volatile uint32_t g_snapshotGeneration;
static volatile Hcsr04Snapshot g_snapshot;

static void Hcsr04UpdateSnapshot(Hcsr04Angle angle, float distance)
{
    uint32_t nowMs = AppTicksToMs(osKernelGetTickCount());
    uint8_t valid = (distance >= 0.0f) ? 1U : 0U;

    g_snapshotGeneration++;
    if (angle == HCSR04_ANGLE_LEFT) {
        g_snapshot.leftCm = distance;
        g_snapshot.leftValid = valid;
        g_snapshot.leftTimestampMs = nowMs;
    } else if (angle == HCSR04_ANGLE_MIDDLE) {
        g_snapshot.frontCm = distance;
        g_snapshot.frontValid = valid;
        g_snapshot.frontTimestampMs = nowMs;
    } else {
        g_snapshot.rightCm = distance;
        g_snapshot.rightValid = valid;
        g_snapshot.rightTimestampMs = nowMs;
    }
    g_snapshotGeneration++;
}

int Hcsr04GetSnapshot(Hcsr04Snapshot *snapshot)
{
    uint32_t before;
    uint32_t after;

    if (snapshot == NULL) {
        return 0;
    }

    for (;;) {
        before = g_snapshotGeneration;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *snapshot = g_snapshot;
        after = g_snapshotGeneration;
        if (before == after && (after & 0x01U) == 0U) {
            return 1;
        }
    }
}

int TaskHcsr04GetLatest(Hcsr04Angle *angle, float *distance)
{
    if (g_hcsr04DataValid == 0) {
        return 0;
    }

    *angle = g_angle;
    *distance = g_distance;
    return 1;
}

static void Hcsr04MeasureAt(Hcsr04Angle angle, const char *angleName)
{
    float distance = GetDistance();

    g_angle = angle;
    g_distance = distance;
    g_hcsr04DataValid = 1;
    Hcsr04UpdateSnapshot(angle, distance);

    printf("angle: %s\r\n", angleName);
    if (distance < 0.0f) {
        printf("distance: no echo\r\n");
        return;
    }
    printf("distance: %.2f cm\r\n", (double)distance);
}

static void Hcsr04Task(void *argument)
{
    (void)argument;

    regress_middle();
    osDelay(AppMsToTicks(HCSR04_SERVO_SETTLE_DELAY_MS));

    while (1) {
        engine_turn_left();
        osDelay(AppMsToTicks(HCSR04_SERVO_SETTLE_DELAY_MS));
        Hcsr04MeasureAt(HCSR04_ANGLE_LEFT, "left");

        regress_middle();
        osDelay(AppMsToTicks(HCSR04_SERVO_SETTLE_DELAY_MS));
        Hcsr04MeasureAt(HCSR04_ANGLE_MIDDLE, "middle");

        engine_turn_right();
        osDelay(AppMsToTicks(HCSR04_SERVO_SETTLE_DELAY_MS));
        Hcsr04MeasureAt(HCSR04_ANGLE_RIGHT, "right");

        osDelay(AppMsToTicks(HCSR04_SCAN_DELAY_MS));
    }
}

void TaskHcsr04Init(void)
{
    osThreadAttr_t attr;

    if (CarControlReverseV8TestModeEnabled() != 0 ||
        CarControlEncoderOnlyExperimentModeEnabled() != 0) {
        /* Reverse/replay and encoder-only experiments use no distance data:
         * center once, then do not start the periodic scan task. */
        regress_middle();
        printf("HCSR04 scan disabled for current experiment; servo centered once\r\n");
        return;
    }

    attr.name = "hcsr04_scan";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(Hcsr04Task, NULL, &attr) == NULL) {
        printf("HCSR04 thread create failed\r\n");
    }
}
