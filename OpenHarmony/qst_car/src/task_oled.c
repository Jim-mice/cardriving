#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "hal_bsp_ssd1306.h"
#include "task_oled.h"
#include "task_sht20.h"
#include "wifiiot_gpio.h"

#define OLED_IR_LEFT_GPIO    WIFI_IOT_GPIO_IDX_13
#define OLED_IR_RIGHT_GPIO   WIFI_IOT_GPIO_IDX_14
#define OLED_REFRESH_PERIOD_MS 500

static void OledShowStatus(WifiIotGpioValue left, WifiIotGpioValue right,
    float temperature, float humidity, int envValid)
{
    uint8_t title[] = "QST CAR";
    uint8_t leftLine[] = "IR left: 0";
    uint8_t rightLine[] = "IR right: 0";
    uint8_t temperatureLine[20] = "T: --.-- C";
    uint8_t humidityLine[20] = "H: --.-- %";

    leftLine[sizeof(leftLine) - 2] = (uint8_t)('0' + left);
    rightLine[sizeof(rightLine) - 2] = (uint8_t)('0' + right);
    if (envValid != 0) {
        (void)snprintf((char *)temperatureLine, sizeof(temperatureLine),
            "T: %.2f C", (double)temperature);
        (void)snprintf((char *)humidityLine, sizeof(humidityLine),
            "H: %.2f %%", (double)humidity);
    }

    SSD1306_CLS();
    SSD1306_ShowStr(0, 0, title, 16);
    SSD1306_ShowStr(0, 2, leftLine, 8);
    SSD1306_ShowStr(0, 3, rightLine, 8);
    SSD1306_ShowStr(0, 4, temperatureLine, 8);
    SSD1306_ShowStr(0, 5, humidityLine, 8);
}

static void OledTask(void *argument)
{
    WifiIotGpioValue left;
    WifiIotGpioValue right;
    float temperature = 0.0f;
    float humidity = 0.0f;
    int envValid;
    (void)argument;

    if (SSD1306_Init() != 0) {
        printf("OLED init failed\r\n");
        return;
    }

    while (1) {
        if (GpioGetInputVal(OLED_IR_LEFT_GPIO, &left) == 0 &&
            GpioGetInputVal(OLED_IR_RIGHT_GPIO, &right) == 0) {
            envValid = TaskSht20GetLatest(&temperature, &humidity);
            OledShowStatus(left, right, temperature, humidity, envValid);
        }
        osDelay(OLED_REFRESH_PERIOD_MS);
    }
}

void TaskOledInit(void)
{
    osThreadAttr_t attr;

    attr.name = "oled_display";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(OledTask, NULL, &attr) == NULL) {
        printf("OLED thread create failed\r\n");
    }
}
