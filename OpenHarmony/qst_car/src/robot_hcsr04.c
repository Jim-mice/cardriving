#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_watchdog.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"

//HC-SR04 超声波测距模块通过GPIO7和8连接到3861
#define GPIO_8 8
#define GPIO_7 7
#define HCSR04_ECHO_TIMEOUT_US 30000

//测距功能实现
float GetDistance  (void) 
{
    unsigned long long start_time = 0;
    unsigned long long time = 0;
    unsigned long long wait_start = 0;
    float distance = 0.0;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;

    //GPIO_7输出一个脉冲触发信号到超声波测距模块
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE0);
   
    //等待超声波模块输出回响信号(高电平)，超时则认为未检测到回波
    wait_start = hi_get_us();
    while (1) {
        GpioGetInputVal(GPIO_8, &value);
        if (value == WIFI_IOT_GPIO_VALUE1) {
            start_time = hi_get_us();
            break;
        }
        if (hi_get_us() - wait_start >= HCSR04_ECHO_TIMEOUT_US) {
            return -1.0f;
        }
    }

    //测量回响信号高电平时间，超时则认为回波异常
    while (1) {
        GpioGetInputVal(GPIO_8, &value);
        if (value == WIFI_IOT_GPIO_VALUE0) {
            time = hi_get_us() - start_time;
            break;
        }
        if (hi_get_us() - start_time >= HCSR04_ECHO_TIMEOUT_US) {
            return -1.0f;
        }
    }
    //距离=高电平时间*0.034 / 2
    distance = time * 0.034 / 2;
    //printf("distance is %f\r\n", distance);
    return distance;
}
