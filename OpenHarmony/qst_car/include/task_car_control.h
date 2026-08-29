#ifndef QST_CAR_TASK_CAR_CONTROL_H
#define QST_CAR_TASK_CAR_CONTROL_H

typedef enum {
    CAR_MODE_IDLE = 0,
    CAR_MODE_TRACE,
    CAR_MODE_AVOID
} CarMode;

int TaskCarControlInit(void);
void CarControlSetMode(CarMode mode);
CarMode CarControlGetMode(void);
/* V8-only hardware gate for tasks that are irrelevant to reverse line tests. */
int CarControlReverseV8TestModeEnabled(void);
/* Encoder-only experiments do not use distance scanning or a moving servo. */
int CarControlEncoderOnlyExperimentModeEnabled(void);

#endif
