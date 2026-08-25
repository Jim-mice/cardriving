#include "bsp_ws2812.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"


#define FR_PORT GPIOC
#define FR_PIN  GPIO_Pin_13


#define BA_PORT GPIOC
#define BA_PIN  GPIO_Pin_14


#define LED_NUM 6


static void delay_short(void)
{
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}


/*
 WS2812 timing
 */
static void send_bit(
GPIO_TypeDef* port,
uint16_t pin,
uint8_t bit)
{

if(bit)
{
    GPIO_SetBits(port,pin);

    delay_short();
    delay_short();
    delay_short();


    GPIO_ResetBits(port,pin);

    delay_short();

}
else
{
    GPIO_SetBits(port,pin);

    delay_short();


    GPIO_ResetBits(port,pin);

    delay_short();
    delay_short();
}

}



static void send_byte(
GPIO_TypeDef* port,
uint16_t pin,
uint8_t data)
{
int i;

for(i=7;i>=0;i--)
{
    send_bit(
        port,
        pin,
        (data>>i)&1
    );
}

}



static void send_led(
GPIO_TypeDef* port,
uint16_t pin,
uint8_t r,
uint8_t g,
uint8_t b)
{

/*
 WS2812:
 GRB
*/

send_byte(port,pin,g);
send_byte(port,pin,r);
send_byte(port,pin,b);

}



static void reset_latch(void)
{
volatile int i;

for(i=0;i<10000;i++);
}



void WS2812_FR_Init(void)
{

GPIO_InitTypeDef gpio;


RCC_APB2PeriphClockCmd(
RCC_APB2Periph_GPIOC,
ENABLE);


gpio.GPIO_Pin=FR_PIN;

gpio.GPIO_Mode=
GPIO_Mode_Out_PP;

gpio.GPIO_Speed=
GPIO_Speed_50MHz;


GPIO_Init(
FR_PORT,
&gpio);


GPIO_ResetBits(
FR_PORT,
FR_PIN);

}



void WS2812_BA_Init(void)
{

GPIO_InitTypeDef gpio;


RCC_APB2PeriphClockCmd(
RCC_APB2Periph_GPIOC,
ENABLE);


gpio.GPIO_Pin=BA_PIN;

gpio.GPIO_Mode=
GPIO_Mode_Out_PP;

gpio.GPIO_Speed=
GPIO_Speed_50MHz;


GPIO_Init(
BA_PORT,
&gpio);


GPIO_ResetBits(
BA_PORT,
BA_PIN);

}





void WS2812_FR_Show(
uint8_t index,
uint8_t r,
uint8_t g,
uint8_t b)
{

int i;

for(i=0;i<LED_NUM;i++)
{

if(i==index)
{
send_led(
FR_PORT,
FR_PIN,
r,g,b);
}
else
{
send_led(
FR_PORT,
FR_PIN,
0,0,0);
}

}

reset_latch();

}




void WS2812_BA_Show(
uint8_t index,
uint8_t r,
uint8_t g,
uint8_t b)
{

int i;


for(i=0;i<LED_NUM;i++)
{

if(i==index)
{
send_led(
BA_PORT,
BA_PIN,
r,g,b);
}
else
{
send_led(
BA_PORT,
BA_PIN,
0,0,0);
}

}


reset_latch();

}

/*
  ¼æÈÝ¾É½Ó¿Ú
*/

void WS2812_FR_SetLED(
uint8_t index,
uint8_t r,
uint8_t g,
uint8_t b)
{
    WS2812_FR_Show(
        index,
        r,
        g,
        b);
}



void WS2812_BA_SetLED(
uint8_t index,
uint8_t r,
uint8_t g,
uint8_t b)
{
    WS2812_BA_Show(
        index,
        r,
        g,
        b);
}

