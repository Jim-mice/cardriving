#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include "cmsis_os2.h"
#include "Peripheral.h"
#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_i2c.h"
#include "wifiiot_i2c_ex.h"
#include "hal_bsp_log.h"
#include "wifiiot_uart.h"
#include "hal_bsp_sht20.h"
#include "hal_bsp_ap3216c.h"
#include "hal_bsp_ssd1306.h"

//HC-SR04 超声波测距模块通过GPIO7和8连接到3861
#define GPIO_8 8
#define GPIO_7 7
#define GPIO_FUNC 0

/**********************串口参数******************/
WifiIotUartAttribute uart_attr2 = {
//波特率: 115200
.baudRate = 115200,
//数据位: 8bits
.dataBits = 8,
.stopBits = 1,
.parity = 0,
 };

/***************************************************************
* 函数名称: Peripheral_IO_Init
* 说    明: Peripheral_GPIO初始化
* 参    数: 无
* 返 回 值: 无
***************************************************************/
static void Peripheral_IO_Init(void)
{

GpioInit();//GPIO功能初始化
/**********************蓝牙初始化******************/
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);//GPIO_0复用为UART1_TXD
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);//GPIO_1复用为UART1_RX
//串口功能初始化
//UartInit(WIFI_IOT_UART_IDX_1, &uart_attr2, NULL);
/**********************通讯串口初始化******************/
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);//GPIO_11复用为UART2_TXD
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);//GPIO_12复用为UART2_RX
UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);
/**********************串口打印初始化******************/

/**********************红外循迹初始化******************/
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13,WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14,WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
/**********************IIC初始化******************/
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_10, WIFI_IOT_IO_FUNC_GPIO_10_I2C0_SDA);//GPIO_10复用为I2C1_SDA
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_9, WIFI_IOT_IO_FUNC_GPIO_9_I2C0_SCL);//GPIO_9复用为I2C1_SCL
I2cInit(WIFI_IOT_I2C_IDX_0, 400000);//模拟I2C初始化
I2cSetBaudrate(WIFI_IOT_I2C_IDX_0, 400000);//baudrate: 400kbps
/**********************超声波初始化******************/
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7,WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8,WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
GpioSetDir(WIFI_IOT_IO_NAME_GPIO_8,WIFI_IOT_GPIO_DIR_IN);//设置GPIO为输ru模式
GpioSetDir(WIFI_IOT_IO_NAME_GPIO_7,WIFI_IOT_GPIO_DIR_OUT);//设置GPIO为输出模式
}

/***************************************************************
* 函数名称: Peripheral_Init
* 说    明: 初始化Peripheral
* 参    数: 无
* 返 回 值: 无
***************************************************************/
void Peripheral_Init(void)
{
    Peripheral_IO_Init();//外设IO口初始化
    SSD1306_Init(); // OLED 显示屏初始化
    SHT20_Init(); // SHT20初始化
    AP3216C_Init();   // 三合一传感器初始化
}