#include "line_sensor.h"

/*
 * Legacy compatibility hook. It intentionally performs no car control:
 * future trace decisions belong exclusively to task_car_control.c.
 */
void trace_module_once(void)
{
    WifiIotGpioValue left;
    WifiIotGpioValue right;

    (void)LineSensorRead(&left, &right);
}
