#include <stdio.h>

#include "cmsis_os2.h"
#include "hal_bsp_sht20.h"
#include "task_sht20.h"

#define SHT20_READ_PERIOD_MS 1000

static volatile float g_temperature;
static volatile float g_humidity;
static volatile int g_sht20DataValid;

int TaskSht20GetLatest(float *temperature, float *humidity)
{
    if (g_sht20DataValid == 0) {
        return 0;
    }

    *temperature = g_temperature;
    *humidity = g_humidity;
    return 1;
}

static void Sht20Task(void *argument)
{
    float temperature;
    float humidity;
    (void)argument;

    if (SHT20_Init() != 0) {
        printf("SHT20 init failed\r\n");
        return;
    }

    while (1) {
        if (SHT20_ReadData(&temperature, &humidity) == 0) {
            g_temperature = temperature;
            g_humidity = humidity;
            g_sht20DataValid = 1;
            printf("temperature: %.2f C\r\n", (double)temperature);
            printf("humidity: %.2f %%\r\n", (double)humidity);
        } else {
            printf("SHT20 read failed\r\n");
        }
        osDelay(SHT20_READ_PERIOD_MS);
    }
}

void TaskSht20Init(void)
{
    osThreadAttr_t attr;

    attr.name = "sht20_sensor";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(Sht20Task, NULL, &attr) == NULL) {
        printf("SHT20 thread create failed\r\n");
    }
}
