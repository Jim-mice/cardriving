#ifndef QST_CAR_TASK_CAR_CONTROL_H
#define QST_CAR_TASK_CAR_CONTROL_H

#include "task_motor_response.h"

/* Attended one-shot actuator-direction check; keep disabled for normal running. */
#define MOTOR_TURN_DIRECTION_TEST_MODE 0

/* Static track/sensor geometry observation; keep disabled for normal running. */
#define TRACE_TRACK_GEOMETRY_TEST_MODE 0

/* Attended TRACE forward-output diagnostic; keep disabled for normal running. */
#define TRACE_OPEN_LOOP_STRAIGHT_TEST_MODE 0

/* Attended fixed-pulse sensor response diagnostic; keep disabled for normal running. */
#define TRACE_STEP_RESPONSE_TEST_MODE 0

/* Attended passive sensor/debounce observer; keep disabled for normal running. */
#define TRACE_OBSERVER_TEST_MODE 0

/* One automatic boot run starts forward TRACE plus recording. */
#define AUTO_TRACE_BOOT_TEST_MODE 1
/* Disabled during forward TRACE debugging: BPATH RETURN remains manual. */
#define AUTO_RETURN_ON_11_TEST_MODE 0
#define AUTO_RETURN_11_CONFIRM_MS 300U

typedef enum {
    CAR_MODE_IDLE = 0,
    CAR_MODE_TRACE,
    CAR_MODE_AVOID
} CarMode;

/* BLE RX only publishes one of these commands. TaskCarControl consumes it. */
typedef enum {
    BPATH_CONTROL_COMMAND_NONE = 0,
    BPATH_CONTROL_COMMAND_START,
    BPATH_CONTROL_COMMAND_RETURN,
    BPATH_CONTROL_COMMAND_RESET
} BpathControlCommand;

typedef enum {
    BPATH_COMMAND_SOURCE_NONE = 0,
    BPATH_COMMAND_SOURCE_UDP,
    BPATH_COMMAND_SOURCE_BLE
} BpathCommandSource;

typedef enum {
    MOTOR_TURN_DIRECTION_COMMAND_NONE = 0,
    MOTOR_TURN_DIRECTION_COMMAND_LEFT,
    MOTOR_TURN_DIRECTION_COMMAND_RIGHT,
    MOTOR_TURN_DIRECTION_COMMAND_FORWARD,
    MOTOR_TURN_DIRECTION_COMMAND_STOP
} MotorTurnDirectionCommand;

typedef enum {
    TRACE_STEP_RESPONSE_COMMAND_NONE = 0,
    TRACE_STEP_RESPONSE_COMMAND_LEFT_100,
    TRACE_STEP_RESPONSE_COMMAND_LEFT_200,
    TRACE_STEP_RESPONSE_COMMAND_LEFT_300,
    TRACE_STEP_RESPONSE_COMMAND_RIGHT_100,
    TRACE_STEP_RESPONSE_COMMAND_RIGHT_200,
    TRACE_STEP_RESPONSE_COMMAND_RIGHT_300,
    TRACE_STEP_RESPONSE_COMMAND_STOP
} TraceStepResponseCommand;

typedef enum {
    TRACE_OBSERVER_COMMAND_NONE = 0,
    TRACE_OBSERVER_COMMAND_START,
    TRACE_OBSERVER_COMMAND_STOP
} TraceObserverCommand;

int TaskCarControlInit(void);
void CarControlSetMode(CarMode mode);
CarMode CarControlGetMode(void);
void CarControlSubmitBpathCommand(BpathControlCommand command);
void CarControlSubmitBpathCommandFromSource(BpathControlCommand command,
                                            BpathCommandSource source);
void CarControlSubmitMotorResponseCommand(MotorResponseCommand command);
void CarControlSubmitMotorTurnDirectionCommand(MotorTurnDirectionCommand command);
void CarControlSubmitTraceStepResponseCommand(TraceStepResponseCommand command);
void CarControlSubmitTraceObserverCommand(TraceObserverCommand command);
/* V8-only hardware gate for tasks that are irrelevant to reverse line tests. */
int CarControlReverseV8TestModeEnabled(void);
/* Encoder-only experiments do not use distance scanning or a moving servo. */
int CarControlEncoderOnlyExperimentModeEnabled(void);

#endif
