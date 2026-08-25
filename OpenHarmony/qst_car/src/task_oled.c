#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "hal_bsp_ssd1306.h"
#include "task_ap3216.h"
#include "task_hcsr04.h"
#include "task_oled.h"
#include "task_sht20.h"
#include "wifiiot_gpio.h"

#define OLED_IR_LEFT_GPIO    WIFI_IOT_GPIO_IDX_13
#define OLED_IR_RIGHT_GPIO   WIFI_IOT_GPIO_IDX_14
#define OLED_REFRESH_PERIOD_MS 500

static void OledShowStatus(WifiIotGpioValue left, WifiIotGpioValue right,
    float temperature, float humidity, int envValid, uint16_t light,
    uint16_t ir, uint16_t proximity, int ap3216Valid, Hcsr04Angle angle,
    float distance, int hcsr04Valid)
{
    uint8_t title[] = "QST CAR";
    uint8_t irGpioLine[] = "IR L:0 R:0";
    uint8_t temperatureLine[20] = "T: --.-- C";
    uint8_t humidityLine[20] = "H: --.-- %";
    uint8_t lightLine[16] = "L: ----";
    uint8_t irLine[16] = "IR: ----";
    uint8_t proximityLine[16] = "P: ----";
    uint8_t hcsr04Line[24] = "A:-- D:--";
    const char *angleName = "--";

    irGpioLine[5] = (uint8_t)('0' + left);
    irGpioLine[9] = (uint8_t)('0' + right);
    if (envValid != 0) {
        (void)snprintf((char *)temperatureLine, sizeof(temperatureLine),
            "T: %.2f C", (double)temperature);
        (void)snprintf((char *)humidityLine, sizeof(humidityLine),
            "H: %.2f %%", (double)humidity);
    }
    if (ap3216Valid != 0) {
        (void)snprintf((char *)lightLine, sizeof(lightLine), "L: %u",
            (unsigned int)light);
        (void)snprintf((char *)irLine, sizeof(irLine), "IR: %u",
            (unsigned int)ir);
        (void)snprintf((char *)proximityLine, sizeof(proximityLine), "P: %u",
            (unsigned int)proximity);
    }
    if (hcsr04Valid != 0) {
        if (angle == HCSR04_ANGLE_LEFT) {
            angleName = "left";
        } else if (angle == HCSR04_ANGLE_MIDDLE) {
            angleName = "middle";
        } else if (angle == HCSR04_ANGLE_RIGHT) {
            angleName = "right";
        }
        if (distance < 0.0f) {
            (void)snprintf((char *)hcsr04Line, sizeof(hcsr04Line),
                "A:%s D:no echo", angleName);
        } else {
            (void)snprintf((char *)hcsr04Line, sizeof(hcsr04Line),
                "A:%s D:%.2f", angleName, (double)distance);
        }
    }

    SSD1306_CLS();
    SSD1306_ShowStr(0, 0, title, 8);
    SSD1306_ShowStr(0, 1, irGpioLine, 8);
    SSD1306_ShowStr(0, 2, temperatureLine, 8);
    SSD1306_ShowStr(0, 3, humidityLine, 8);
    SSD1306_ShowStr(0, 4, lightLine, 8);
    SSD1306_ShowStr(0, 5, irLine, 8);
    SSD1306_ShowStr(0, 6, proximityLine, 8);
    SSD1306_ShowStr(0, 7, hcsr04Line, 8);
}

static void OledTask(void *argument)
{
    WifiIotGpioValue left;
    WifiIotGpioValue right;
    float temperature = 0.0f;
    float humidity = 0.0f;
    uint16_t light = 0;
    uint16_t ir = 0;
    uint16_t proximity = 0;
    Hcsr04Angle angle = HCSR04_ANGLE_MIDDLE;
    float distance = 0.0f;
    int envValid;
    int ap3216Valid;
    int hcsr04Valid;
    (void)argument;

    if (SSD1306_Init() != 0) {
        printf("OLED init failed\r\n");
        return;
    }

    while (1) {
        if (GpioGetInputVal(OLED_IR_LEFT_GPIO, &left) == 0 &&
            GpioGetInputVal(OLED_IR_RIGHT_GPIO, &right) == 0) {
            envValid = TaskSht20GetLatest(&temperature, &humidity);
            ap3216Valid = TaskAp3216GetLatest(&light, &ir, &proximity);
            hcsr04Valid = TaskHcsr04GetLatest(&angle, &distance);
            OledShowStatus(left, right, temperature, humidity, envValid,
                light, ir, proximity, ap3216Valid, angle, distance, hcsr04Valid);
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
