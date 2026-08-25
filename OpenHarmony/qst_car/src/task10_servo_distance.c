#include <stdio.h>

#include "cmsis_os2.h"
#include "task10_servo_distance.h"

extern void engine_turn_left(void);
extern void engine_turn_right(void);
extern void regress_middle(void);
extern float GetDistance(void);

#define SERVO_SETTLE_DELAY_MS 500
#define SERVO_SCAN_DELAY_MS   1000

static void PrintDistance(const char *angle)
{
    float distance = GetDistance();

    printf("angle: %s\r\n", angle);
    if (distance < 0.0f) {
        printf("distance: no echo\r\n");
        return;
    }
    printf("distance: %.2f cm\r\n", (double)distance);
}

static void ServoDistanceTask(void *argument)
{
    (void)argument;

    regress_middle();
    osDelay(SERVO_SETTLE_DELAY_MS);

    while (1) {
        engine_turn_left();
        osDelay(SERVO_SETTLE_DELAY_MS);
        PrintDistance("left");

        regress_middle();
        osDelay(SERVO_SETTLE_DELAY_MS);
        PrintDistance("middle");

        engine_turn_right();
        osDelay(SERVO_SETTLE_DELAY_MS);
        PrintDistance("right");

        osDelay(SERVO_SCAN_DELAY_MS);
    }
}

void Task10ServoDistanceInit(void)
{
    osThreadAttr_t attr;

    attr.name = "servo_distance";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(ServoDistanceTask, NULL, &attr) == NULL) {
        printf("servo distance thread create failed\r\n");
    }
}
