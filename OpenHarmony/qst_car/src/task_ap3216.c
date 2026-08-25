#include "task_ap3216.h"
#include "task_sensor.h"

void TaskAp3216Init(void)
{
    TaskSensorInit();
}

int TaskAp3216GetLatest(uint16_t *light, uint16_t *ir, uint16_t *proximity)
{
    return TaskSensorGetAp3216Latest(light, ir, proximity);
}
