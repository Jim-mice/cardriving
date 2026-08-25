#ifndef TASK_HCSR04_H
#define TASK_HCSR04_H

typedef enum {
    HCSR04_ANGLE_LEFT,
    HCSR04_ANGLE_MIDDLE,
    HCSR04_ANGLE_RIGHT,
} Hcsr04Angle;

void TaskHcsr04Init(void);
int TaskHcsr04GetLatest(Hcsr04Angle *angle, float *distance);

#endif
