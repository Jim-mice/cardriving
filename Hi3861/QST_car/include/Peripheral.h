
#ifndef __PERIPHERAL_H__
#define __PERIPHERAL_H__

/***************************************************************
* 名		称: GasStatus_ENUM
* 说    明：枚举状态结构体
***************************************************************/
typedef enum
{
	OFF = 0,
	ON
} Peripheral_Status_ENUM;

/* 传感器数据类型定义 ------------------------------------------------------------*/
typedef struct
{
	float    Lux;							//光照强度
	float    Humidity;        //湿度
	float    Temperature;     //温度
} Peripheral_Data_TypeDef;




/* 寄存器宏定义 --------------------------------------------------------------------*/
#define SHT30_Addr 0x44

#define BH1750_Addr 0x23
double GetGP2Y(void);
double GetVoltage(void);
void Peripheral_Init(void);
void Light_StatusSet(Peripheral_Status_ENUM status);
void Motor_StatusSet_Fan(Peripheral_Status_ENUM status);
void Motor_StatusSet_RGB_G(Peripheral_Status_ENUM status);
void Motor_StatusSet_RGB_R(Peripheral_Status_ENUM status);
void Motor_StatusSet_RGB_B(Peripheral_Status_ENUM status);
void Motor_StatusSet_Water(Peripheral_Status_ENUM status);















#endif /* __E53_IA1_H__ */

