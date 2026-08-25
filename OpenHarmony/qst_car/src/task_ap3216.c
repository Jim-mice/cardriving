#include <stdio.h>

#include "cmsis_os2.h"
#include "hal_bsp_ap3216c.h"
#include "task_ap3216.h"

#define AP3216_READ_PERIOD_MS 1000

static volatile uint16_t g_light;
static volatile uint16_t g_ir;
static volatile uint16_t g_proximity;
static volatile int g_ap3216DataValid;

int TaskAp3216GetLatest(uint16_t *light, uint16_t *ir, uint16_t *proximity)
{
    if (g_ap3216DataValid == 0) {
        return 0;
    }

    *light = g_light;
    *ir = g_ir;
    *proximity = g_proximity;
    return 1;
}

static void Ap3216Task(void *argument)
{
    uint16_t light;
    uint16_t ir;
    uint16_t proximity;
    (void)argument;

    if (AP3216C_Init() != 0) {
        printf("AP3216 init failed\r\n");
        return;
    }

    while (1) {
        if (AP3216C_ReadData(&ir, &light, &proximity) == 0) {
            g_light = light;
            g_ir = ir;
            g_proximity = proximity;
            g_ap3216DataValid = 1;
            printf("light: %u\r\n", (unsigned int)light);
            printf("ir: %u\r\n", (unsigned int)ir);
            printf("proximity: %u\r\n", (unsigned int)proximity);
        } else {
            printf("AP3216 read failed\r\n");
        }
        osDelay(AP3216_READ_PERIOD_MS);
    }
}

void TaskAp3216Init(void)
{
    osThreadAttr_t attr;

    attr.name = "ap3216_sensor";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(Ap3216Task, NULL, &attr) == NULL) {
        printf("AP3216 thread create failed\r\n");
    }
}
