#ifndef QST_CAR_TASK_MOTOR_RESPONSE_H
#define QST_CAR_TASK_MOTOR_RESPONSE_H

#include <stdint.h>

/* Temporary attended characterization mode. Set to 0 to restore the
 * existing BPATH/TRACE lifecycle without accepting MOTORCAL UDP commands. */
#define MOTOR_RESPONSE_TEST_MODE 0

typedef enum {
    MOTOR_RESPONSE_COMMAND_NONE = 0,
    MOTOR_RESPONSE_COMMAND_START,
    MOTOR_RESPONSE_COMMAND_STOP
} MotorResponseCommand;

/* This module never sends motor commands. TaskCarControl supplies the only
 * motor I/O after consuming the requested command values returned here. */
void MotorResponseInit(void);
void MotorResponseStep(uint32_t nowTicks, MotorResponseCommand command,
                       int *leftCommand, int *rightCommand);

#endif
