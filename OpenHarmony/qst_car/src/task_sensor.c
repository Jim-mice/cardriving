#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "hal_bsp_ap3216c.h"
#include "hal_bsp_sht20.h"
#include "hal_bsp_ssd1306.h"
#include "task_hcsr04.h"
#include "task_sensor.h"
#include "wifiiot_gpio.h"

#define SENSOR_POLL_PERIOD_MS 100
#define SENSOR_IR_PERIOD_MS 200
#define SENSOR_READ_PERIOD_MS 1000
#define SENSOR_LOG_PERIOD_MS 5000
#define SENSOR_OLED_PERIOD_MS 500
#define SENSOR_IR_LEFT_GPIO WIFI_IOT_GPIO_IDX_13
#define SENSOR_IR_RIGHT_GPIO WIFI_IOT_GPIO_IDX_14

static volatile float g_temperature;
static volatile float g_humidity;
static volatile uint16_t g_light;
static volatile uint16_t g_ir;
static volatile uint16_t g_proximity;
static volatile int g_sht20DataValid;
static volatile int g_ap3216DataValid;
static int g_sensorTaskStarted;

static void OledShowStatus(WifiIotGpioValue left, WifiIotGpioValue right)
{
    uint8_t temperatureLine[22];
    uint8_t humidityLine[22];
    uint8_t lightLine[22];
    uint8_t irLine[22];

    (void)left;
    (void)right;

    (void)snprintf((char *)temperatureLine, sizeof(temperatureLine), "Temp:%.2f C", (double)g_temperature);
    (void)snprintf((char *)humidityLine, sizeof(humidityLine), "Hum:%.2f %%", (double)g_humidity);
    (void)snprintf((char *)lightLine, sizeof(lightLine), "Light:%u", (unsigned int)g_light);
    (void)snprintf((char *)irLine, sizeof(irLine), "IR:%u Pro:%u", (unsigned int)g_ir,
        (unsigned int)g_proximity);

    SSD1306_CLS();
    SSD1306_ShowStr(0, 0, temperatureLine, 8);
    SSD1306_ShowStr(0, 1, humidityLine, 8);
    SSD1306_ShowStr(0, 2, lightLine, 8);
    SSD1306_ShowStr(0, 3, irLine, 8);
}

static void SensorTask(void *argument)
{
    WifiIotGpioValue left = 0;
    WifiIotGpioValue right = 0;
    unsigned int elapsed = 0;
    (void)argument;

    if (SSD1306_Init() != 0) {
        printf("OLED init failed\r\n");
    }
    if (SHT20_Init() == 0) {
        g_sht20DataValid = 1;
    } else {
        printf("SHT20 init failed\r\n");
    }
    if (AP3216C_Init() == 0) {
        g_ap3216DataValid = 1;
    } else {
        printf("AP3216 init failed\r\n");
    }

    for (;;) {
        if ((elapsed % SENSOR_IR_PERIOD_MS) == 0U &&
            GpioGetInputVal(SENSOR_IR_LEFT_GPIO, &left) == 0 &&
            GpioGetInputVal(SENSOR_IR_RIGHT_GPIO, &right) == 0) {
            if ((elapsed % SENSOR_LOG_PERIOD_MS) == 0U) {
                printf("IR left: %d\r\n", (int)left);
                printf("IR right: %d\r\n", (int)right);
            }
        }
        if ((elapsed % SENSOR_READ_PERIOD_MS) == 0U) {
            if (g_sht20DataValid != 0 && SHT20_ReadData((float *)&g_temperature, (float *)&g_humidity) == 0) {
                if ((elapsed % SENSOR_LOG_PERIOD_MS) == 0U) {
                    printf("temperature: %.2f C\r\n", (double)g_temperature);
                    printf("humidity: %.2f %%\r\n", (double)g_humidity);
                }
            } else if (g_sht20DataValid != 0) {
                printf("SHT20 read failed\r\n");
            }
            if (g_ap3216DataValid != 0 && AP3216C_ReadData((uint16_t *)&g_ir, (uint16_t *)&g_light,
                (uint16_t *)&g_proximity) == 0) {
                if ((elapsed % SENSOR_LOG_PERIOD_MS) == 0U) {
                    printf("light: %u\r\n", (unsigned int)g_light);
                    printf("ir: %u\r\n", (unsigned int)g_ir);
                    printf("proximity: %u\r\n", (unsigned int)g_proximity);
                }
            } else if (g_ap3216DataValid != 0) {
                printf("AP3216 read failed\r\n");
            }
        }
        if ((elapsed % SENSOR_OLED_PERIOD_MS) == 0U) {
            OledShowStatus(left, right);
        }
        osDelay(SENSOR_POLL_PERIOD_MS);
        elapsed += SENSOR_POLL_PERIOD_MS;
    }
}

void TaskSensorInit(void)
{
    osThreadAttr_t attr;
    if (g_sensorTaskStarted != 0) {
        return;
    }
    attr.name = "sensor_task";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = osPriorityBelowNormal;
    if (osThreadNew(SensorTask, NULL, &attr) == NULL) {
        printf("sensor thread create failed\r\n");
        return;
    }
    g_sensorTaskStarted = 1;
}

int TaskSensorGetSht20Latest(float *temperature, float *humidity)
{
    if (temperature == NULL || humidity == NULL || g_sht20DataValid == 0) {
        return 0;
    }
    *temperature = g_temperature;
    *humidity = g_humidity;
    return 1;
}

int TaskSensorGetAp3216Latest(uint16_t *light, uint16_t *ir, uint16_t *proximity)
{
    if (light == NULL || ir == NULL || proximity == NULL || g_ap3216DataValid == 0) {
        return 0;
    }
    *light = g_light;
    *ir = g_ir;
    *proximity = g_proximity;
    return 1;
}
