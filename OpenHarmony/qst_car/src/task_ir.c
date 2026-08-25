#include <stdio.h>

#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "task_ir.h"

/* GPIO13/GPIO14 are configured as infrared inputs in Peripheral.c. */
#define IR_LEFT_GPIO  WIFI_IOT_GPIO_IDX_13
#define IR_RIGHT_GPIO WIFI_IOT_GPIO_IDX_14
#define IR_POLL_PERIOD_MS 200

static void IrTask(void *argument)
{
    WifiIotGpioValue left;
    WifiIotGpioValue right;
    (void)argument;

    while (1) {
        if (GpioGetInputVal(IR_LEFT_GPIO, &left) == 0 &&
            GpioGetInputVal(IR_RIGHT_GPIO, &right) == 0) {
            printf("IR left: %d\r\n", (int)left);
            printf("IR right: %d\r\n", (int)right);
        }
        osDelay(IR_POLL_PERIOD_MS);
    }
}

void TaskIrInit(void)
{
    osThreadAttr_t attr;

    attr.name = "ir_monitor";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(IrTask, NULL, &attr) == NULL) {
        printf("IR thread create failed\r\n");
    }
}
