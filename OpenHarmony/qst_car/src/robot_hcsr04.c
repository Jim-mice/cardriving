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

//测距功能实现
float GetDistance  (void) 
{
    static unsigned long long start_time = 0, time = 0;
    float distance = 0.0;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0;

    //GPIO_7输出一个脉冲触发信号到超声波测距模块
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE0);
   
    //超声波测距模块接收到GPIO_7输出的脉冲触发信号后,模块输出回响信号(高电平)到GPIO_8
    while (1) {
        GpioGetInputVal(GPIO_8, &value);
        //测量回响信号(高电平)时间
        if ( value == WIFI_IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
            //printf("start time is %d\r\n", start_time);
            flag = 1;
        }
        if (value == WIFI_IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() ;
            //printf("time is %d\r\n", time);
            time= time - start_time;
            //printf("time is %d\r\n", time);
            start_time = 0;
            break;
        }
    }
    //距离=高电平时间*0.034 / 2
    distance = time * 0.034 / 2;
    //printf("distance is %f\r\n", distance);
    return distance;
}