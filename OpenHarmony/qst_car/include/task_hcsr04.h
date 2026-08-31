#ifndef TASK_HCSR04_H
#define TASK_HCSR04_H

#include <stdint.h>

typedef enum {
    HCSR04_ANGLE_LEFT,
    HCSR04_ANGLE_MIDDLE,
    HCSR04_ANGLE_RIGHT,
} Hcsr04Angle;

/* Completed left/front/right measurements from the SG90 scan. */
typedef struct {
    float leftCm;
    float frontCm;
    float rightCm;
    uint8_t leftValid;
    uint8_t frontValid;
    uint8_t rightValid;
    uint32_t leftTimestampMs;
    uint32_t frontTimestampMs;
    uint32_t rightTimestampMs;
} Hcsr04Snapshot;

void TaskHcsr04Init(void);
int TaskHcsr04GetLatest(Hcsr04Angle *angle, float *distance);
int Hcsr04GetSnapshot(Hcsr04Snapshot *snapshot);

#endif
