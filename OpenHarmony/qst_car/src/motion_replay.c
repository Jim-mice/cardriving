#include <stdio.h>
#include <unistd.h>

#include "cmsis_os2.h"
#include "app_time.h"
#include "motion_record.h"
#include "motion_replay.h"

extern void stm32motor_control(int motorA, int motorB);

#define MOTION_REPLAY_BOOT_WAIT_MS 1000U

static void MotionReplayTask(void *argument)
{
    uint32_t index;

    (void)argument;
    osDelay(AppMsToTicks(MOTION_REPLAY_BOOT_WAIT_MS));

    printf("[MOTION_REPLAY] start count=%u\r\n", (unsigned int)MOTION_RECORD_COUNT);
    for (index = 0U; index < MOTION_RECORD_COUNT; index++) {
        stm32motor_control(g_motion_record[index].left, g_motion_record[index].right);
        usleep((unsigned int)g_motion_record[index].delay_ms * 1000U);
    }

    stm32motor_control(0, 0);
    printf("[MOTION_REPLAY] finished\r\n");
}

int MotionReplayInit(void)
{
    static const osThreadAttr_t attr = {
        .name = "motion_replay",
        .attr_bits = 0,
        .cb_mem = NULL,
        .cb_size = 0,
        .stack_mem = NULL,
        .stack_size = 2048,
        .priority = osPriorityNormal,
    };

    return osThreadNew(MotionReplayTask, NULL, &attr) == NULL ? -1 : 0;
}
