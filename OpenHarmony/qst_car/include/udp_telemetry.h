#ifndef QST_CAR_UDP_TELEMETRY_H
#define QST_CAR_UDP_TELEMETRY_H

#include <stdint.h>

typedef enum {
    UDP_TELEMETRY_ACTION_STOP = 0,
    UDP_TELEMETRY_ACTION_FORWARD,
    UDP_TELEMETRY_ACTION_LEFT,
    UDP_TELEMETRY_ACTION_RIGHT,
    UDP_TELEMETRY_ACTION_RECOVER_LEFT,
    UDP_TELEMETRY_ACTION_RECOVER_RIGHT,
    UDP_TELEMETRY_ACTION_HOLD_LEFT,
    UDP_TELEMETRY_ACTION_HOLD_RIGHT,
    UDP_TELEMETRY_ACTION_HOLD_CENTER
} UdpTelemetryAction;

typedef struct {
    uint8_t left;
    uint8_t right;
    uint8_t stableState;
    UdpTelemetryAction action;
    int leftCommand;
    int rightCommand;
    uint32_t sequence;
} CarTelemetryState;

typedef struct {
    uint32_t left0;
    uint32_t left1;
    uint32_t leftTransitions;
    uint32_t right0;
    uint32_t right1;
    uint32_t rightTransitions;
    uint32_t sequence;
} UdpSensorStats;

typedef struct {
    uint8_t left;
    uint8_t right;
    uint32_t sequence;
} UdpLineCalibrationState;

typedef enum {
    UDP_AVOID_ACTION_STOP = 0,
    UDP_AVOID_ACTION_FORWARD,
    UDP_AVOID_ACTION_LEFT_TURN,
    UDP_AVOID_ACTION_RIGHT_TURN,
    UDP_AVOID_ACTION_BLOCKED
} UdpAvoidAction;

typedef struct {
    float leftCm;
    float frontCm;
    float rightCm;
    uint8_t leftValid;
    uint8_t frontValid;
    uint8_t rightValid;
    UdpAvoidAction action;
    int leftCommand;
    int rightCommand;
    uint32_t sequence;
} UdpAvoidTelemetryState;

typedef enum {
    UDP_RACE_EVENT_START_MARKER = 0,
    UDP_RACE_EVENT_START_CLEAR,
    UDP_RACE_EVENT_SENSOR,
    UDP_RACE_EVENT_MARKER1,
    UDP_RACE_EVENT_MARKER1_CLEAR,
    UDP_RACE_EVENT_FINISH_MARKER2,
    UDP_RACE_EVENT_FINISH_SLOW,
    UDP_RACE_EVENT_FINISH_STOP,
    UDP_RACE_EVENT_DEADEND,
    UDP_RACE_EVENT_DEADEND_SUMMARY,
    UDP_RACE_EVENT_HEARTBEAT
} UdpRaceEvent;

typedef struct {
    UdpRaceEvent event;
    uint8_t raceState;
    uint8_t sensorState;
    uint8_t previousSensorState;
    uint8_t lastNon00SensorState;
    uint8_t lastNon11SensorState;
    UdpTelemetryAction action;
    int leftCommand;
    int rightCommand;
    uint32_t elapsedMs;
    uint32_t lastActionChangeMs;
    uint32_t transitionCount;
    uint32_t leftCorrectionCount;
    uint32_t rightCorrectionCount;
    uint32_t leftCorrectionMs;
    uint32_t rightCorrectionMs;
    uint32_t markerGapMs;
    uint32_t marker1ElapsedMs;
    uint32_t probeElapsedMs;
    uint32_t last10AgeMs;
    uint32_t last01AgeMs;
    uint32_t sequence;
} UdpRaceTelemetryState;

typedef struct {
    uint8_t raceState;
    uint8_t rawLeft;
    uint8_t rawRight;
    uint8_t sensorState;
    uint8_t previousSensorState;
    uint8_t lastNon00SensorState;
    uint8_t lastNon11SensorState;
    uint8_t lastCorrection;
    UdpTelemetryAction action;
    int leftCommand;
    int rightCommand;
    uint32_t elapsedMs;
    uint32_t correctionAgeMs;
    uint32_t leftCorrectionCount;
    uint32_t rightCorrectionCount;
    uint32_t transitionCount;
    uint32_t sequence;
} UdpRaceDebugState;

typedef enum {
    UDP_REVERSE_EVENT_START = 0,
    UDP_REVERSE_EVENT_SENSOR,
    UDP_REVERSE_EVENT_SENSOR_11_STOP,
    UDP_REVERSE_EVENT_HEARTBEAT
} UdpReverseEvent;

typedef enum {
    UDP_REVERSE_ACTION_STOP = 0,
    UDP_REVERSE_ACTION_FORWARD,
    UDP_REVERSE_ACTION_LEFT,
    UDP_REVERSE_ACTION_RIGHT
} UdpReverseAction;

typedef struct {
    UdpReverseEvent event;
    uint8_t rawLeft;
    uint8_t rawRight;
    uint8_t sensorState;
    uint8_t previousSensorState;
    uint8_t lastCorrection;
    UdpReverseAction action;
    int leftCommand;
    int rightCommand;
    uint16_t consecutive00;
    uint16_t consecutive01;
    uint16_t consecutive10;
    uint16_t consecutive11;
    uint32_t sequence;
} UdpReverseTelemetryState;

typedef enum {
    UDP_TRACEDEBUG_EVENT_UPDATE = 0,
    UDP_TRACEDEBUG_EVENT_POSSIBLE_LOST_LINE,
    UDP_TRACEDEBUG_EVENT_HEARTBEAT
} UdpTraceDebugEvent;

typedef struct {
    UdpTraceDebugEvent event;
    uint8_t rawLeft;
    uint8_t rawRight;
    uint8_t sensorState;
    uint8_t previousSensorState;
    uint8_t lastNon00SensorState;
    uint8_t lastNon11SensorState;
    UdpTelemetryAction action;
    UdpTelemetryAction lastAction;
    uint8_t lastCorrection;
    int leftCommand;
    int rightCommand;
    uint32_t actionAgeMs;
    uint32_t sensorAgeMs;
    uint32_t correctionAgeMs;
    uint32_t last10AgeMs;
    uint32_t last01AgeMs;
    uint16_t consecutive00;
    uint16_t consecutive10;
    uint16_t consecutive01;
    uint16_t consecutive11;
    uint16_t history00;
    uint16_t history01;
    uint16_t history10;
    uint16_t history11;
    uint32_t sequence;
} UdpTraceDebugState;

/* Starts an outbound-only telemetry task. Failure is non-fatal to vehicle control. */
void UdpTelemetryInit(void);

/* Lightweight state update for CarControl; it performs no socket I/O. */
void UdpTelemetryUpdate(uint8_t left, uint8_t right, uint8_t stableState,
                        UdpTelemetryAction action, int leftCommand, int rightCommand);
/* Update only the actual motor command in the current telemetry snapshot. */
void UdpTelemetryUpdateMotorCommand(int leftCommand, int rightCommand);
typedef struct { int16_t leftDelta; int16_t rightDelta; uint8_t sequence; uint32_t validCount; uint32_t badChecksumCount; uint32_t badFrameCount; uint32_t lastRxMs; int32_t totalLeft; int32_t totalRight; } UdpEncoderTelemetryState;
void UdpTelemetryUpdateEncoder(const UdpEncoderTelemetryState *state);
void UdpTelemetryReadEncoder(UdpEncoderTelemetryState *state);
typedef struct { uint32_t timeMs; int32_t left; int32_t right; } UdpTeachPoint;
int UdpTelemetryQueueTeachPoint(const UdpTeachPoint *point);
int UdpTelemetryPopTeachPoint(UdpTeachPoint *point);
void UdpTelemetryPublishCal(const char *text);
/* Queued experiment text is drained by the low-priority UDP task.  It keeps
 * post-run trajectory dumps out of the 30 ms motor-control task. */
int UdpTelemetryQueueExperimentText(const char *text);

/* Lightweight raw GPIO statistics update; it performs no socket I/O. */
void UdpTelemetryUpdateSensorStats(uint32_t left0, uint32_t left1,
                                   uint32_t leftTransitions, uint32_t right0,
                                   uint32_t right1, uint32_t rightTransitions);

/* Lightweight raw calibration update; it performs no socket I/O. */
void UdpTelemetryUpdateLineCalibration(uint8_t left, uint8_t right);

/* Lightweight AVOID snapshot update; socket I/O remains in UdpTelemetryTask. */
void UdpTelemetryUpdateAvoid(const UdpAvoidTelemetryState *state);

/* Lightweight race-event snapshot update; socket I/O remains in UdpTelemetryTask. */
void UdpTelemetryUpdateRace(const UdpRaceTelemetryState *state);
void UdpTelemetryUpdateRaceDebug(const UdpRaceDebugState *state);

/* Lightweight reverse-trace diagnostics update; socket I/O remains in UdpTelemetryTask. */
void UdpTelemetryUpdateReverse(const UdpReverseTelemetryState *state);

typedef enum {
    UDP_REVERSE_V2_EVENT_START = 0,
    UDP_REVERSE_V2_EVENT_SENSOR,
    UDP_REVERSE_V2_EVENT_ALIGN_LEFT_START,
    UDP_REVERSE_V2_EVENT_ALIGN_RIGHT_START,
    UDP_REVERSE_V2_EVENT_ALIGN_CLEAR,
    UDP_REVERSE_V2_EVENT_BACK_RESUME,
    UDP_REVERSE_V2_EVENT_ALIGN_TIMEOUT,
    UDP_REVERSE_V2_EVENT_SENSOR_11_STOP,
    UDP_REVERSE_V2_EVENT_HEARTBEAT
} UdpReverseV2Event;

typedef enum {
    UDP_REVERSE_V2_STATE_BACK = 0,
    UDP_REVERSE_V2_STATE_ALIGN_LEFT,
    UDP_REVERSE_V2_STATE_ALIGN_RIGHT,
    UDP_REVERSE_V2_STATE_SETTLING,
    UDP_REVERSE_V2_STATE_STOPPED
} UdpReverseV2State;

typedef struct {
    UdpReverseV2Event event;
    UdpReverseV2State state;
    uint8_t rawLeft;
    uint8_t rawRight;
    uint8_t sensorState;
    int leftCommand;
    int rightCommand;
    uint16_t consecutive00;
    uint16_t consecutive01;
    uint16_t consecutive10;
    uint16_t consecutive11;
    uint32_t alignElapsedMs;
    uint32_t sequence;
} UdpReverseV2TelemetryState;

/* Lightweight Reverse V2 diagnostics update; socket I/O remains in UdpTelemetryTask. */
void UdpTelemetryUpdateReverseV2(const UdpReverseV2TelemetryState *state);

typedef enum {
    UDP_REVERSE_V3_EVENT_START = 0,
    UDP_REVERSE_V3_EVENT_SENSOR,
    UDP_REVERSE_V3_EVENT_SENSOR_11_STOP,
    UDP_REVERSE_V3_EVENT_HEARTBEAT
} UdpReverseV3Event;

typedef enum {
    UDP_REVERSE_V3_ACTION_STOP = 0,
    UDP_REVERSE_V3_ACTION_BACK,
    UDP_REVERSE_V3_ACTION_CORRECT_LEFT,
    UDP_REVERSE_V3_ACTION_CORRECT_RIGHT
} UdpReverseV3Action;

typedef struct {
    UdpReverseV3Event event;
    uint8_t rawLeft;
    uint8_t rawRight;
    uint8_t sensorState;
    uint8_t previousSensorState;
    uint8_t lastCorrection;
    UdpReverseV3Action action;
    int leftCommand;
    int rightCommand;
    uint16_t consecutive00;
    uint16_t consecutive01;
    uint16_t consecutive10;
    uint16_t consecutive11;
    uint32_t sequence;
} UdpReverseV3TelemetryState;

/* Lightweight Reverse V3 diagnostics update; socket I/O remains in UdpTelemetryTask. */
void UdpTelemetryUpdateReverseV3(const UdpReverseV3TelemetryState *state);

typedef enum {
    UDP_REVERSE_V4_EVENT_START = 0,
    UDP_REVERSE_V4_EVENT_SENSOR,
    UDP_REVERSE_V4_EVENT_PULSE_START,
    UDP_REVERSE_V4_EVENT_PULSE_END,
    UDP_REVERSE_V4_EVENT_PROBE_START,
    UDP_REVERSE_V4_EVENT_PROBE_RESULT,
    UDP_REVERSE_V4_EVENT_PULSE_LIMIT,
    UDP_REVERSE_V4_EVENT_SENSOR_11_STOP,
    UDP_REVERSE_V4_EVENT_WAIT_CLEAR,
    UDP_REVERSE_V4_EVENT_ARMED,
    UDP_REVERSE_V4_EVENT_REARM,
    UDP_REVERSE_V4_EVENT_HEARTBEAT
} UdpReverseV4Event;

typedef enum {
    UDP_REVERSE_V4_STATE_BACK = 0,
    UDP_REVERSE_V4_STATE_PULSE_LEFT,
    UDP_REVERSE_V4_STATE_PULSE_RIGHT,
    UDP_REVERSE_V4_STATE_PROBE_BACK,
    UDP_REVERSE_V4_STATE_STOPPED
} UdpReverseV4State;

typedef struct {
    UdpReverseV4Event event;
    UdpReverseV4State state;
    uint8_t rawLeft;
    uint8_t rawRight;
    uint8_t sensorState;
    uint8_t previousSensorState;
    uint8_t triggerSensorState;
    uint8_t edgeSide;
    uint8_t expectedSensorState;
    uint8_t correctionDirection;
    int leftCommand;
    int rightCommand;
    uint16_t sameSidePulses;
    uint32_t pulseElapsedMs;
    uint32_t targetMs;
    uint32_t probeElapsedMs;
    uint32_t transitElapsedMs;
    uint32_t searchElapsedMs;
    uint32_t nudgeElapsedMs;
    uint32_t shiftElapsedMs;
    uint32_t shiftDurationMs;
    uint32_t restoreElapsedMs;
    uint32_t restoreTargetMs;
    uint8_t centerConfidence;
    uint32_t sequence;
} UdpReverseV4TelemetryState;

/* Lightweight Reverse V4 diagnostics update; socket I/O remains in UdpTelemetryTask. */
void UdpTelemetryUpdateReverseV4(const UdpReverseV4TelemetryState *state);

/* Pure forward-trajectory reverse replay telemetry.  The reverse phase only
 * observes sensors; it never uses this snapshot to steer. */
typedef enum {
    UDP_REPLAY_EVENT_WAIT_START = 0,
    UDP_REPLAY_EVENT_RECORD_START,
    UDP_REPLAY_EVENT_COMMAND_CHANGE,
    UDP_REPLAY_EVENT_END_MARKER,
    UDP_REPLAY_EVENT_REVERSE_START,
    UDP_REPLAY_EVENT_REVERSE_COMMAND_CHANGE,
    UDP_REPLAY_EVENT_SENSOR_MISMATCH,
    UDP_REPLAY_EVENT_DONE,
    UDP_REPLAY_EVENT_BUFFER_FULL,
    UDP_REPLAY_EVENT_PRE_MARKER_HISTORY,
    UDP_REPLAY_EVENT_END_MARKER_CONTEXT,
    UDP_REPLAY_EVENT_MARKER_CANDIDATE,
    UDP_REPLAY_EVENT_MARKER_REJECTED,
    UDP_REPLAY_EVENT_HEARTBEAT,
    UDP_REPLAY_EVENT_MARKER_CONFIRM_START,
    UDP_REPLAY_EVENT_MARKER_CONFIRM_HEARTBEAT,
    UDP_REPLAY_EVENT_MARKER_CONFIRM_REJECT,
    UDP_REPLAY_EVENT_MARKER_CONFIRMED
} UdpReplayEvent;

typedef enum {
    UDP_REPLAY_MARKER_REASON_NONE = 0,
    UDP_REPLAY_MARKER_REASON_DIRECT_00_TO_11,
    UDP_REPLAY_MARKER_REASON_SHORT_10_PREAMBLE,
    UDP_REPLAY_MARKER_REASON_SHORT_01_PREAMBLE,
    UDP_REPLAY_MARKER_REASON_SIGNED_PREAMBLE_TOO_LONG,
    UDP_REPLAY_MARKER_REASON_NO_00_CONTEXT
} UdpReplayMarkerReason;

typedef struct {
    UdpReplayEvent event;
    uint8_t phase;
    uint8_t recordedSensor;
    uint8_t currentSensor;
    uint8_t lowMagnitude;
    int origLeftCommand;
    int origRightCommand;
    int replayLeftCommand;
    int replayRightCommand;
    uint32_t frameIndex;
    uint32_t frameCount;
    uint32_t forwardDurationMs;
    uint32_t elapsedMs;
    uint8_t previousStableSensor;
    uint8_t stableBefore11[5];
    uint32_t sinceLast00Ms;
    uint32_t sinceLast10Ms;
    uint32_t sinceLast01Ms;
    uint32_t stable00DurationMs;
    uint8_t stateBeforePrevious;
    uint8_t signedPreambleSensor;
    uint8_t markerDecision;
    uint8_t markerReason;
    uint32_t signedPreambleMs;
    uint32_t markerLimitMs;
    uint32_t confirmElapsedMs;
    uint32_t confirmTargetMs;
    uint32_t sequence;
} UdpReplayTelemetryState;

void UdpTelemetryUpdateReplay(const UdpReplayTelemetryState *state);

#define UDP_REPLAY_HISTORY_CAPACITY 100U

typedef struct {
    uint32_t timestampMs;
    uint8_t rawLeft;
    uint8_t rawRight;
    uint8_t stableSensor;
    uint8_t previousStableSensor;
    uint8_t traceAction;
    uint8_t reserved[3];
    int leftCommand;
    int rightCommand;
} UdpReplayHistoryFrame;

void UdpTelemetryRecordReplayHistory(const UdpReplayHistoryFrame *frame);
void UdpTelemetryRequestReplayHistoryDump(void);

typedef enum {
    UDP_REVERSE_V5_EVENT_START = 100, UDP_REVERSE_V5_EVENT_ARMED,
    UDP_REVERSE_V5_EVENT_HEADING_PULSE_START, UDP_REVERSE_V5_EVENT_HEADING_PROBE,
    UDP_REVERSE_V5_EVENT_LATERAL_SHIFT_START, UDP_REVERSE_V5_EVENT_SHIFT_YAW_OUT,
    UDP_REVERSE_V5_EVENT_SHIFT_BACK, UDP_REVERSE_V5_EVENT_SHIFT_YAW_BACK,
    UDP_REVERSE_V5_EVENT_RECHECK, UDP_REVERSE_V5_EVENT_LATERAL_RECENTER_SUCCESS,
    UDP_REVERSE_V5_EVENT_LATERAL_RECENTER_SAME_SIDE, UDP_REVERSE_V5_EVENT_LATERAL_OVERSHOOT,
    UDP_REVERSE_V5_EVENT_LATERAL_SHIFT_LIMIT, UDP_REVERSE_V5_EVENT_OPPOSITE_AFTER_HEADING_PROBE,
    UDP_REVERSE_V5_EVENT_SENSOR_11_STOP, UDP_REVERSE_V5_EVENT_REARM,
    UDP_REVERSE_V5_EVENT_WAIT_CLEAR, UDP_REVERSE_V5_EVENT_HEARTBEAT
} UdpReverseV5Event;
typedef struct {
    UdpReverseV5Event event; uint8_t state, rawLeft, rawRight, sensorState, previousSensorState;
    uint8_t triggerSensorState, shiftDirection; int leftCommand, rightCommand;
    uint32_t phaseElapsedMs; uint16_t headingTrigger, lateralShiftCount; uint32_t sequence;
} UdpReverseV5TelemetryState;
void UdpTelemetryUpdateReverseV5(const UdpReverseV5TelemetryState *state);
typedef enum {
    UDP_REVERSE_V6_EVENT_START = 200, UDP_REVERSE_V6_EVENT_ARMED,
    UDP_REVERSE_V6_EVENT_HEADING_START, UDP_REVERSE_V6_EVENT_HEADING_CAPTURE_00,
    UDP_REVERSE_V6_EVENT_HEADING_TIMEOUT_SAME_SIDE, UDP_REVERSE_V6_EVENT_HEADING_OVERSHOOT,
    UDP_REVERSE_V6_EVENT_LATERAL_START, UDP_REVERSE_V6_EVENT_LATERAL_CAPTURE_00,
    UDP_REVERSE_V6_EVENT_LATERAL_CAPTURE_TIMEOUT, UDP_REVERSE_V6_EVENT_LATERAL_OPPOSITE_WITHOUT_CAPTURE,
    UDP_REVERSE_V6_EVENT_YAW_RESTORE_START, UDP_REVERSE_V6_EVENT_YAW_RESTORE_DONE,
    UDP_REVERSE_V6_EVENT_RESTORE_RETURN_ORIGINAL_SIDE, UDP_REVERSE_V6_EVENT_RESTORE_CROSS_OPPOSITE_SIDE,
    UDP_REVERSE_V6_EVENT_VERIFY_START, UDP_REVERSE_V6_EVENT_TRACK_REACQUIRED,
    UDP_REVERSE_V6_EVENT_SENSOR_11_STOP, UDP_REVERSE_V6_EVENT_REARM, UDP_REVERSE_V6_EVENT_WAIT_CLEAR,
    UDP_REVERSE_V6_EVENT_HEARTBEAT
} UdpReverseV6Event;
typedef enum {
 UDP_REVERSE_V7_EVENT_START=300, UDP_REVERSE_V7_EVENT_ARMED, UDP_REVERSE_V7_EVENT_HEADING_START,
 UDP_REVERSE_V7_EVENT_SWEEP_ENTRY_YAW_START,
 UDP_REVERSE_V7_EVENT_SWEEP_START, UDP_REVERSE_V7_EVENT_CENTER_WINDOW_ENTER,
 UDP_REVERSE_V7_EVENT_OPPOSITE_EDGE_CONFIRMED, UDP_REVERSE_V7_EVENT_OPPOSITE_EDGE_TIMEOUT,
 UDP_REVERSE_V7_EVENT_RETURN_HEADING_START, UDP_REVERSE_V7_EVENT_RETURN_SWEEP_START,
 UDP_REVERSE_V7_EVENT_RETURN_CENTER_CAPTURE, UDP_REVERSE_V7_EVENT_YAW_RESTORE_START,
 UDP_REVERSE_V7_EVENT_VERIFY_START, UDP_REVERSE_V7_EVENT_TRACK_REACQUIRED,
 UDP_REVERSE_V7_EVENT_VERIFY_ORIGINAL_SIDE, UDP_REVERSE_V7_EVENT_VERIFY_OPPOSITE_SIDE,
 UDP_REVERSE_V7_EVENT_SENSOR_11_STOP, UDP_REVERSE_V7_EVENT_REARM, UDP_REVERSE_V7_EVENT_HEARTBEAT
} UdpReverseV7Event;

typedef enum {
    UDP_REVERSE_V8_EVENT_START = 400,
    UDP_REVERSE_V8_EVENT_ARMED,
    UDP_REVERSE_V8_EVENT_EDGE_ENTRY_START,
    UDP_REVERSE_V8_EVENT_EDGE_CANDIDATE_START,
    UDP_REVERSE_V8_EVENT_EDGE_LOCKED,
    UDP_REVERSE_V8_EVENT_EDGE_REACQUIRED,
    UDP_REVERSE_V8_EVENT_EDGE_GAP_START,
    UDP_REVERSE_V8_EVENT_EDGE_CORRECT_TOWARD,
    UDP_REVERSE_V8_EVENT_EDGE_CORRECT_AWAY,
    UDP_REVERSE_V8_EVENT_EDGE_LOST,
    UDP_REVERSE_V8_EVENT_EDGE_SIDE_SWITCH,
    UDP_REVERSE_V8_EVENT_SENSOR_11_STOP,
    UDP_REVERSE_V8_EVENT_HEARTBEAT,
    /* V8.2 simple stable-state reverse servo. */
    UDP_REVERSE_V8_EVENT_SIMPLE_STATE_CHANGE,
    UDP_REVERSE_V8_EVENT_SIMPLE_HEARTBEAT,
    UDP_REVERSE_V8_EVENT_SIMPLE_SEARCH_BACK_START,
    UDP_REVERSE_V8_EVENT_SIMPLE_EDGE_REACQUIRED,
    UDP_REVERSE_V8_EVENT_SIMPLE_LINE_LOST,
    UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_START,
    UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_END,
    /* V8.4 paired-turn recenter experiment. */
    UDP_REVERSE_V8_EVENT_PAIR_SHIFT_START,
    UDP_REVERSE_V8_EVENT_PAIR_TRUSTED_00,
    UDP_REVERSE_V8_EVENT_PAIR_RESTORE_START,
    UDP_REVERSE_V8_EVENT_PAIR_RESTORE_END,
    UDP_REVERSE_V8_EVENT_PAIR_HIT_11,
    UDP_REVERSE_V8_EVENT_PAIR_SHIFT_TIMEOUT,
    UDP_REVERSE_V8_EVENT_PAIR_HEARTBEAT,
    /* V8.5 micro paired reverse servo. */
    UDP_REVERSE_V8_EVENT_MICRO_SHIFT_START,
    UDP_REVERSE_V8_EVENT_MICRO_SHIFT_END,
    UDP_REVERSE_V8_EVENT_MICRO_RESTORE_START,
    UDP_REVERSE_V8_EVENT_MICRO_RESTORE_END,
    UDP_REVERSE_V8_EVENT_MICRO_SETTLE_START,
    UDP_REVERSE_V8_EVENT_MICRO_SETTLE_END,
    UDP_REVERSE_V8_EVENT_MICRO_PAIR_REPEAT,
    UDP_REVERSE_V8_EVENT_MICRO_HEARTBEAT,
    /* V8.6 biased micro reverse servo. */
    UDP_REVERSE_V8_EVENT_BIASED_SHIFT_START,
    UDP_REVERSE_V8_EVENT_BIASED_SHIFT_END,
    UDP_REVERSE_V8_EVENT_BIASED_PARTIAL_RESTORE,
    UDP_REVERSE_V8_EVENT_BIASED_SETTLE,
    UDP_REVERSE_V8_EVENT_BIASED_REPEAT,
    UDP_REVERSE_V8_EVENT_ESCAPE_11_START,
    UDP_REVERSE_V8_EVENT_ESCAPE_11_PULSE,
    UDP_REVERSE_V8_EVENT_ESCAPE_11_EXIT,
    UDP_REVERSE_V8_EVENT_BIASED_HEARTBEAT
    ,UDP_REVERSE_V8_EVENT_BIASED_10_SHIFT
    ,UDP_REVERSE_V8_EVENT_BIASED_10_RESTORE
    ,UDP_REVERSE_V8_EVENT_BIASED_01_SHIFT
    ,UDP_REVERSE_V8_EVENT_BIASED_01_RESTORE
    ,UDP_REVERSE_V8_EVENT_AMBIGUOUS_00
    ,UDP_REVERSE_V8_EVENT_AMBIGUOUS_11
    ,UDP_REVERSE_V8_EVENT_SIGNED_SENSOR_OVERRIDE
} UdpReverseV8Event;

/* Lightweight sharp-turn diagnostics update; socket I/O remains in UdpTelemetryTask. */
void UdpTelemetryUpdateTraceDebug(const UdpTraceDebugState *state);

#endif
