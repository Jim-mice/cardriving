#ifndef TASK_SENSOR_H
#define TASK_SENSOR_H

#include <stdint.h>

void TaskSensorInit(void);
int TaskSensorGetSht20Latest(float *temperature, float *humidity);
int TaskSensorGetAp3216Latest(uint16_t *light, uint16_t *ir, uint16_t *proximity);

#endif
