#ifndef TASK_AP3216_H
#define TASK_AP3216_H

#include <stdint.h>

void TaskAp3216Init(void);
int TaskAp3216GetLatest(uint16_t *light, uint16_t *ir, uint16_t *proximity);

#endif
