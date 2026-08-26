#ifndef TASK_WIFI_H
#define TASK_WIFI_H

#include <stdint.h>

void TaskWifiInit(void);
int TaskWifiWaitStaStarted(uint32_t timeoutMs);

#endif
