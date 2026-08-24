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
#define GPIO_FUNC 0

//测距功能实现
float GetDistance  (void) 
{
    static unsigned long long start_time = 0, time = 0;
    float distance = 0.0;
    int i;
    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;
    unsigned int flag = 0;
//    printf("start\r\n "); 
    i=0;
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7,WIFI_IOT_IO_FUNC_GPIO_7_GPIO);//LED：高电平触发
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8,WIFI_IOT_IO_FUNC_GPIO_8_GPIO);//LED：高电平触发
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_8,WIFI_IOT_GPIO_DIR_IN);//设置GPIO为输ru模式
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_7,WIFI_IOT_GPIO_DIR_OUT);//设置GPIO为输出模式
    //GPIO_7输出一个脉冲触发信号到超声波测距模块
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE0);
   
    //超声波测距模块接收到GPIO_7输出的脉冲触发信号后,模块输出回响信号(高电平)到GPIO_8
    while (1) {
        GpioGetInputVal(GPIO_8, &value);
//        printf("while\r\n ");
        //测量回响信号(高电平)时间
        if ( value == WIFI_IOT_GPIO_VALUE1 && flag == 0) {
            start_time = hi_get_us();
//            printf("start time is %d\r\n", start_time);
            flag = 1;
        }
        if (value == WIFI_IOT_GPIO_VALUE0 && flag == 1) {
            time = hi_get_us() ;
//            printf("time is %d\r\n", time);
            time=time- start_time;
//             printf("time is %d\r\n", time);
            start_time = 0;
            break;
        }
      i++;
 //     if(i>=20) break;
     

    }

    //距离=高电平时间*0.034 / 2
    distance = time * 0.034 / 2;
//     printf("distance is %f\r\n", distance);
    return distance;
}