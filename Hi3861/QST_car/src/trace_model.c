#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_watchdog.h"
#include "wifiiot_errno.h"
#include "hi_pwm.h"
#include "hi_timer.h"
#include "wifiiot_pwm.h"
#include <stdio.h>
#include "Peripheral.h"
#include "hi_task.h"

//查阅机器人板原理图可知
//左边的红外传感器通过GPIO13与3861芯片连接
//右边的红外传感器通过GPIO14与3861芯片连接
#define GPIOL 13
#define GPIOR 14
#define GPIO_FUNC 0

extern void stm32motor_control(uint8_t dir,uint8_t motorA,uint8_t motorB,int angle);   
extern void car_backward(void);
extern void car_forward(void);
extern void car_left_tra(void);
extern void car_right_tra(void);
extern void car_stop(void);

extern uint8_t sen_or_car_flag;   //0 传感器采集模式   1  小车模式
extern unsigned char g_car_status;  

WifiIotGpioValue io_status_left;
WifiIotGpioValue io_status_right;

//获取红外传感器的值，调整电机的状态
void timer1_callback(unsigned int arg)
{
    (void)arg;

    GpioGetInputVal(GPIOL,&io_status_left); //获取GPIO13引脚的输入电平值
    GpioGetInputVal(GPIOR,&io_status_right);//获取GPIO14引脚的输入电平值
    
    if(io_status_right != WIFI_IOT_GPIO_VALUE1 && io_status_left != WIFI_IOT_GPIO_VALUE1){
        car_forward();
    }
    else if(io_status_right == WIFI_IOT_GPIO_VALUE1 && io_status_left != WIFI_IOT_GPIO_VALUE1)
    {   
        car_right_tra();
    }
    else if(io_status_right != WIFI_IOT_GPIO_VALUE1 && io_status_left == WIFI_IOT_GPIO_VALUE1)
    {
        car_left_tra();
    }
    else if(io_status_right == WIFI_IOT_GPIO_VALUE1 && io_status_left == WIFI_IOT_GPIO_VALUE1){
        car_stop();
    }
}

void trace_module(void)
{  
    while (1) { 
        timer1_callback(0);   //扫描红外对管函数
        if (sen_or_car_flag !=1 || g_car_status != 1) {            //标志位变化，跳出任务
             printf("trace_module stop!\n");
             car_stop();
            break;
        }
        hi_sleep(30);
    }
}
