#include <stdio.h>

#include "cmsis_os2.h"
#include "task_hcsr04.h"

extern void engine_turn_left(void);
extern void engine_turn_right(void);
extern void regress_middle(void);
extern float GetDistance(void);

#define HCSR04_SERVO_SETTLE_DELAY_MS 500
#define HCSR04_SCAN_DELAY_MS         1000

static volatile Hcsr04Angle g_angle;
static volatile float g_distance;
static volatile int g_hcsr04DataValid;

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
    osDelay(HCSR04_SERVO_SETTLE_DELAY_MS);

    while (1) {
        engine_turn_left();
        osDelay(HCSR04_SERVO_SETTLE_DELAY_MS);
        Hcsr04MeasureAt(HCSR04_ANGLE_LEFT, "left");

        regress_middle();
        osDelay(HCSR04_SERVO_SETTLE_DELAY_MS);
        Hcsr04MeasureAt(HCSR04_ANGLE_MIDDLE, "middle");

        engine_turn_right();
        osDelay(HCSR04_SERVO_SETTLE_DELAY_MS);
        Hcsr04MeasureAt(HCSR04_ANGLE_RIGHT, "right");

        osDelay(HCSR04_SCAN_DELAY_MS);
    }
}

void TaskHcsr04Init(void)
{
    osThreadAttr_t attr;

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
