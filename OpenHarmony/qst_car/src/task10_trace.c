#include <stdio.h>

#include "cmsis_os2.h"
#include "task10_bluetooth.h"
#include "task10_trace.h"

#define TRACE_DURATION_MS 15000
#define TRACE_PERIOD_MS      30

extern void trace_module_once(void);
extern void car_stop(void);

static void TraceTask(void *argument)
{
    unsigned int elapsed;
    (void)argument;

    for (elapsed = 0; elapsed < TRACE_DURATION_MS; elapsed += TRACE_PERIOD_MS) {
        trace_module_once();
        osDelay(TRACE_PERIOD_MS);
    }

    car_stop();
    printf("trace task finished\r\n");
    Task10BluetoothInit();
}

void Task10TraceInit(void)
{
    osThreadAttr_t attr;

    attr.name = "task10_trace";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(TraceTask, NULL, &attr) == NULL) {
        printf("trace thread create failed\r\n");
    }
}
