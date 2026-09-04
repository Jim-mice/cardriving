#ifndef QST_CAR_MOTION_REPLAY_H
#define QST_CAR_MOTION_REPLAY_H

#include <stdint.h>

typedef struct {
    uint16_t delay_ms;
    int16_t left;
    int16_t right;
} MotionFrame;

int MotionReplayInit(void);

#endif
