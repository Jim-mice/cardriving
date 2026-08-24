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
//左边的红外传感器通过GPIO11与3861芯片连接
//右边的红外传感器通过GPIO12与3861芯片连接
#define GPIOL 13
#define GPIOR 14
#define GPIO_FUNC 0
#define car_speed_left 0
#define car_speed_right 0
extern unsigned char g_car_status;  //小车运动模式
extern uint8_t sen_or_car_flag;   //0 传感器采集模式   1  小车模式

unsigned int g_car_speed_left = car_speed_left;
unsigned int g_car_speed_right = car_speed_right;
WifiIotGpioValue io_status_left;
WifiIotGpioValue io_status_right;

//获取红外传感器的值，调整电机的状态
void timer1_callback(unsigned int arg)
{
    (void)arg;
    WifiIotGpioValue io_status;

    //调整左轮状态 
    //如果左轮处于停止状态（GPIO1为高电平），则左边红外传感器没检测黑色的话把左轮处于正转状态（GPIO1为低电平）
    if(g_car_speed_left != car_speed_left)
    {
        GpioGetInputVal(GPIOL,&io_status);
        if(io_status != WIFI_IOT_GPIO_VALUE1){
            g_car_speed_left = car_speed_left;
           
        }
    }

    //调整右轮状态
    //如果右轮处于停止状态（GPIO10为高电平），则右边红外传感器没检测黑色的话把右轮处于正转状态（GPIO10为低电平）
    if(g_car_speed_right != car_speed_right)   
    {
        GpioGetInputVal(GPIOR,&io_status);
        if(io_status != WIFI_IOT_GPIO_VALUE1){
            g_car_speed_right = car_speed_right;
           
        }
    }
    
    //小车处于停止状态，则把小车变成前进状态（如果GPIO1和GPIO10输出高电平，则GPIO1和GPIO10输出低电平）
    if(g_car_speed_left != car_speed_left && g_car_speed_right != car_speed_right)
    {
        g_car_speed_left = car_speed_left;
        g_car_speed_right = car_speed_right;
    }

    GpioGetInputVal(GPIOL,&io_status_left); //获取GPIO11引脚的输入电平值
    GpioGetInputVal(GPIOR,&io_status_right);//获取GPIO12引脚的输入电平值
    
    //小车往左偏，则需要向右修正方向(右转)
    //如果GPIO12输入低电平（右边的红外传感器检测到黑色）并且GPIO11输入高电平（左边的红外传感器未检测到黑色）
    if(io_status_right == WIFI_IOT_GPIO_VALUE1 && io_status_left != WIFI_IOT_GPIO_VALUE1)
    {   
        //则GPIO1输出低电平,GPIO10输出高电平。小车右转（右轮不转，左轮正转）
        g_car_speed_left = car_speed_left;
        g_car_speed_right = 1;
    } 

    //小车往右偏，则需要向左修正方向（左转）
    //如果GPIO12输入高电平（右边的红外传感器未检测到黑色）并且GPIO11输入低电平（左边的红外传感器检测到黑色）
    if(io_status_right != WIFI_IOT_GPIO_VALUE1 && io_status_left == WIFI_IOT_GPIO_VALUE1)
    {
        //则GPIO1输出高电平,GPIO10输出低电平。小车左转（右轮正转，左轮不转）
        g_car_speed_left = 1;
        g_car_speed_right = car_speed_right;
    }

}

void trace_module(void)
{  
    while (1) 
    { 
        timer1_callback(0);
        printf("L:  %d,R:  %d\n\r",g_car_speed_left,g_car_speed_right);
        if (sen_or_car_flag!=1||g_car_status != 1) {
             printf("car_mode_control_func 0 module changed\n");
            break;
        }
        hi_sleep(250);
    }
}
