#include <stdio.h>
#include "ohos_init.h"
#include "cmsis_os2.h"

#include "Peripheral.h"
#include "wifi_connect.h"
#include "uart_task9.h"
#include "task10_servo_distance.h"
#include "task10_trace.h"
#include "task_ir.h"
#include "task_oled.h"
#include "task_sht20.h"
#include "task_ap3216.h"


static void QstCarTask(void)
{
    printf("QST car start\r\n");

    Peripheral_Init();

    /* UART2 is initialized by Peripheral_Init before its RX tasks start. */
    UartTask9Init();
    Task10ServoDistanceInit();
    Task10TraceInit();
    TaskIrInit();
    TaskSht20Init();
    TaskAp3216Init();
    TaskOledInit();

    printf("Peripheral init done\r\n");


    while (1) {

        // 后续这里放：
        // 1. 电机控制
        // 2. MQTT通信
        // 3. 传感器读取

        osDelay(1000);
    }
}


static void QstCarEntry(void)
{
    osThreadAttr_t attr;

    attr.name = "qst_car_task";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityNormal;


    osThreadNew(
        (osThreadFunc_t)QstCarTask,
        NULL,
        &attr
    );
}


SYS_RUN(QstCarEntry);
