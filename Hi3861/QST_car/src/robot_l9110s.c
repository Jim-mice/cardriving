#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "wifiiot_uart.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_pwm.h"
#include "hi_pwm.h"
#include "hi_uart.h"

uint8_t uart_sendbuf[20];

/***通信协议***/
/*
发送至stm32的数据协议
参数1：左侧电机的速度rad/s的一百倍，例如：设置转速为1rad/s则传入100
*/
void stm32motor_control(int motorA, int motorB)
{                                             
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;
    //确认旋转方向 正转：0 反转 1
    if(motorA<0){
        A_dir=1;
        motorA = -motorA;
    }else{
        A_dir=0;
    }
    if(motorB<0){
        B_dir=1;
        motorB = -motorB;
    }else{
        B_dir=0;
    }
    //限制幅度 -150 ~150
    if (motorA > 150)
    {
        motorA = 150;
    }
    if (motorB > 150)
    {
        motorB = 150;
    }

    // 数据协议 
    uart_sendbuf[0] = 0xFC;   // 帧头
    uart_sendbuf[1] = A_dir;  // 左轮方向    0正转，1反转
    uart_sendbuf[2] = motorA; // 左轮速度
    uart_sendbuf[3] = B_dir;  // 右轮方向    0正转，1反转
    uart_sendbuf[4] = motorB; // 右轮速度
    uart_sendbuf[5] = 0xFD;   // 帧尾
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

// 小车后退
void car_backward(void)
{
    stm32motor_control(-150, -150);
}

// 小车前进
void car_forward(void)
{
    stm32motor_control(100, 100);
}

// 小车左转 循迹使用
void car_left_tra(void)
{
    stm32motor_control(65, 110);
}

// 小车右转 循迹使用
void car_right_tra(void)
{
    stm32motor_control(110, 65);
}

// 小车左转 避障使用
void car_left(void)
{
    stm32motor_control(-50, 150);
}

// 小车右转 避障使用
void car_right(void)
{
    stm32motor_control(150, -50);
}

// 小车停止
void car_stop(void)
{
    stm32motor_control(0, 0);
}
