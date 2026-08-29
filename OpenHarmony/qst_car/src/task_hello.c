#include <stdio.h>

#include "cmsis_os2.h"
#include "app_time.h"
#include "task_hello.h"

#define HELLO_PERIOD_MS 2000

static void HelloTask(void *argument)
{
    (void)argument;

    while (1) {
        printf("Hello OpenHarmony\r\n");
        osDelay(AppMsToTicks(HELLO_PERIOD_MS));
    }
}

void TaskHelloInit(void)
{
    osThreadAttr_t attr;

    attr.name = "hello_task";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(HelloTask, NULL, &attr) == NULL) {
        printf("hello thread create failed\r\n");
    }
}
