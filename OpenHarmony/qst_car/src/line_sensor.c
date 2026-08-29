#include <stddef.h>
#include "line_sensor.h"

/* Hardware calibration: physical left TCRT5000 is GPIO13; right is GPIO14. */
#define LINE_SENSOR_LEFT_GPIO  WIFI_IOT_GPIO_IDX_13
#define LINE_SENSOR_RIGHT_GPIO WIFI_IOT_GPIO_IDX_14

int LineSensorRead(WifiIotGpioValue *left, WifiIotGpioValue *right)
{
    if (left == NULL || right == NULL) {
        return 0;
    }

    if (GpioGetInputVal(LINE_SENSOR_LEFT_GPIO, left) != 0) {
        return 0;
    }
    if (GpioGetInputVal(LINE_SENSOR_RIGHT_GPIO, right) != 0) {
        return 0;
    }

    return 1;
}
