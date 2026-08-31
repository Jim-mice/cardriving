#ifndef QST_CAR_LINE_SENSOR_H
#define QST_CAR_LINE_SENSOR_H

#include "wifiiot_gpio.h"

/*
 * Read the two board-level line sensors only. Hardware verification shows
 * Latest hardware calibration confirms GPIO13 is the physical left sensor and
 * GPIO14 is the physical right sensor. Both pins are configured by Peripheral_Init().
 * Both pins are configured by Peripheral_Init().
 */
int LineSensorRead(WifiIotGpioValue *left, WifiIotGpioValue *right);

#endif
