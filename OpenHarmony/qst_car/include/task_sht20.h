#ifndef TASK_SHT20_H
#define TASK_SHT20_H

void TaskSht20Init(void);
int TaskSht20GetLatest(float *temperature, float *humidity);

#endif
