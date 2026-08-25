#include "task_sensor.h"
#include "task_sht20.h"

void TaskSht20Init(void)
{
    TaskSensorInit();
}

int TaskSht20GetLatest(float *temperature, float *humidity)
{
    return TaskSensorGetSht20Latest(temperature, humidity);
}
