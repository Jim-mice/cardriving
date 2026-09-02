#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "app_time.h"
#include "cloud_config_private.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "task_car_control.h"
#include "task_wifi.h"
#include "udp_telemetry.h"
#define UDP_TELEMETRY_PORT 5005U
#define UDP_COMMAND_PORT 7788U
#define UDP_TELEMETRY_PERIOD_MS 100U
#define UDP_TELEMETRY_HEARTBEAT_MS 1000U
#define UDP_TELEMETRY_SOCKET_RETRY_MS 2000U
#define UDP_TELEMETRY_SEND_ERROR_LOG_MS 5000U
#define UDP_TELEMETRY_THREAD_STACK_SIZE 4096U
#define UDP_COMMAND_THREAD_STACK_SIZE 2048U
#define UDP_COMMAND_RX_BUFFER_SIZE 160U
#define UDP_COMMAND_RECV_TIMEOUT_MS 200U
#define UDP_TELEMETRY_TEXT_BUFFER_SIZE 1536U
#define UDP_REPLAY_EVENT_QUEUE_CAPACITY 16U
#define UDP_EXPERIMENT_EVENT_QUEUE_CAPACITY 8U
#define UDP_EXPERIMENT_EVENT_TEXT_SIZE 1536U

static volatile uint32_t g_snapshotGeneration;
static volatile CarTelemetryState g_snapshot;
static volatile uint32_t g_sensorStatsGeneration;
static volatile UdpSensorStats g_sensorStats;
static volatile uint32_t g_lineCalibrationGeneration;
static volatile UdpLineCalibrationState g_lineCalibration;
static volatile uint32_t g_avoidGeneration;
static volatile UdpAvoidTelemetryState g_avoid;
static volatile uint32_t g_raceGeneration;
static volatile UdpRaceTelemetryState g_race;
static volatile uint32_t g_raceDebugGeneration;
static volatile UdpRaceDebugState g_raceDebug;
static volatile uint32_t g_reverseGeneration;
static volatile UdpReverseTelemetryState g_reverse;
static volatile uint32_t g_reverseV2Generation;
static volatile UdpReverseV2TelemetryState g_reverseV2;
static volatile uint32_t g_reverseV3Generation;
static volatile UdpReverseV3TelemetryState g_reverseV3;
static volatile uint32_t g_reverseV4Generation;
static volatile UdpReverseV4TelemetryState g_reverseV4;
static volatile uint32_t g_replayGeneration;
static volatile UdpReplayTelemetryState g_replay;
static UdpReplayTelemetryState g_replayEventQueue[UDP_REPLAY_EVENT_QUEUE_CAPACITY];
static volatile uint32_t g_replayEventQueueHead;
static volatile uint32_t g_replayEventQueueTail;
static volatile uint32_t g_replayEventQueueCount;
static char g_experimentEventQueue[UDP_EXPERIMENT_EVENT_QUEUE_CAPACITY]
                                  [UDP_EXPERIMENT_EVENT_TEXT_SIZE];
static volatile uint32_t g_experimentEventQueueHead;
static volatile uint32_t g_experimentEventQueueTail;
static volatile uint32_t g_experimentEventQueueCount;
/* Forward pre-marker diagnostic history: static storage, never task stack. */
static UdpReplayHistoryFrame g_replayHistory[UDP_REPLAY_HISTORY_CAPACITY];
static volatile uint32_t g_replayHistoryWriteIndex;
static volatile uint32_t g_replayHistoryCount;
static volatile uint32_t g_replayHistoryDumpGeneration;
static volatile uint32_t g_traceDebugGeneration;
static volatile UdpTraceDebugState g_traceDebug;
static volatile uint32_t g_encoderGeneration;
static volatile UdpEncoderTelemetryState g_encoder;
void UdpTelemetryReadEncoder(UdpEncoderTelemetryState *state)
{
    uint32_t before = g_encoderGeneration;
    while ((before & 1U) != 0U) before = g_encoderGeneration;
    *state = g_encoder;
}
static int g_udpTelemetryTaskStarted;
static int g_udpCommandTaskStarted;
static volatile uint32_t g_calGeneration;
static char g_calText[768];

/* All UDP formatting is serialized by UdpTelemetryTask.  Keep these buffers
 * out of that task's stack so snprintf() call depth has adequate headroom. */
static char g_udpTelemetryText[UDP_TELEMETRY_TEXT_BUFFER_SIZE];
static char g_udpAvoidLeftText[16];
static char g_udpAvoidFrontText[16];
static char g_udpAvoidRightText[16];

static const char *UdpTelemetryActionName(UdpTelemetryAction action)
{
    switch (action) {
        case UDP_TELEMETRY_ACTION_FORWARD:
            return "FWD";
        case UDP_TELEMETRY_ACTION_LEFT:
            return "LEFT";
        case UDP_TELEMETRY_ACTION_RIGHT:
            return "RIGHT";
        case UDP_TELEMETRY_ACTION_RECOVER_LEFT:
            return "RECOVER_LEFT";
        case UDP_TELEMETRY_ACTION_RECOVER_RIGHT:
            return "RECOVER_RIGHT";
        case UDP_TELEMETRY_ACTION_HOLD_LEFT:
            return "HOLD_LEFT";
        case UDP_TELEMETRY_ACTION_HOLD_RIGHT:
            return "HOLD_RIGHT";
        case UDP_TELEMETRY_ACTION_HOLD_CENTER:
            return "HOLD_CENTER";
        default:
            return "STOP";
    }
}

static uint32_t UdpTelemetryUptimeMs(void)
{
    return AppTicksToMs(osKernelGetTickCount());
}

static void UdpTelemetryReadSnapshot(CarTelemetryState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_snapshotGeneration;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_snapshot;
        after = g_snapshotGeneration;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadSensorStats(UdpSensorStats *stats)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_sensorStatsGeneration;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *stats = g_sensorStats;
        after = g_sensorStatsGeneration;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadLineCalibration(UdpLineCalibrationState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_lineCalibrationGeneration;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_lineCalibration;
        after = g_lineCalibrationGeneration;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadAvoid(UdpAvoidTelemetryState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_avoidGeneration;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_avoid;
        after = g_avoidGeneration;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadRace(UdpRaceTelemetryState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_raceGeneration;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_race;
        after = g_raceGeneration;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadRaceDebug(UdpRaceDebugState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_raceDebugGeneration;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_raceDebug;
        after = g_raceDebugGeneration;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadReverse(UdpReverseTelemetryState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_reverseGeneration;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_reverse;
        after = g_reverseGeneration;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadReverseV2(UdpReverseV2TelemetryState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_reverseV2Generation;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_reverseV2;
        after = g_reverseV2Generation;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadReverseV3(UdpReverseV3TelemetryState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_reverseV3Generation;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_reverseV3;
        after = g_reverseV3Generation;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadReverseV4(UdpReverseV4TelemetryState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_reverseV4Generation;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_reverseV4;
        after = g_reverseV4Generation;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadReplay(UdpReplayTelemetryState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_replayGeneration;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_replay;
        after = g_replayGeneration;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

static void UdpTelemetryReadTraceDebug(UdpTraceDebugState *state)
{
    uint32_t before;
    uint32_t after;

    for (;;) {
        before = g_traceDebugGeneration;
        if ((before & 0x01U) != 0U) {
            continue;
        }
        *state = g_traceDebug;
        after = g_traceDebugGeneration;
        if (before == after && (after & 0x01U) == 0U) {
            return;
        }
    }
}

void UdpTelemetryUpdate(uint8_t left, uint8_t right, uint8_t stableState,
                        UdpTelemetryAction action, int leftCommand, int rightCommand)
{
    CarTelemetryState previous;

    UdpTelemetryReadSnapshot(&previous);

    g_snapshotGeneration++;
    g_snapshot.left = left;
    g_snapshot.right = right;
    g_snapshot.stableState = stableState;
    g_snapshot.action = action;
    g_snapshot.leftCommand = leftCommand;
    g_snapshot.rightCommand = rightCommand;
    if (previous.left != left || previous.right != right ||
        previous.stableState != stableState || previous.action != action ||
        previous.leftCommand != leftCommand || previous.rightCommand != rightCommand) {
        g_snapshot.sequence = previous.sequence + 1U;
    }
    g_snapshotGeneration++;
}

void UdpTelemetryUpdateMotorCommand(int leftCommand, int rightCommand)
{
    CarTelemetryState previous;

    UdpTelemetryReadSnapshot(&previous);
    UdpTelemetryUpdate(previous.left, previous.right, previous.stableState,
                       previous.action, leftCommand, rightCommand);
}

void UdpTelemetryUpdateSensorStats(uint32_t left0, uint32_t left1,
                                   uint32_t leftTransitions, uint32_t right0,
                                   uint32_t right1, uint32_t rightTransitions)
{
    UdpSensorStats previous;

    UdpTelemetryReadSensorStats(&previous);

    g_sensorStatsGeneration++;
    g_sensorStats.left0 = left0;
    g_sensorStats.left1 = left1;
    g_sensorStats.leftTransitions = leftTransitions;
    g_sensorStats.right0 = right0;
    g_sensorStats.right1 = right1;
    g_sensorStats.rightTransitions = rightTransitions;
    g_sensorStats.sequence = previous.sequence + 1U;
    g_sensorStatsGeneration++;
}

void UdpTelemetryUpdateLineCalibration(uint8_t left, uint8_t right)
{
    UdpLineCalibrationState previous;

    UdpTelemetryReadLineCalibration(&previous);

    g_lineCalibrationGeneration++;
    g_lineCalibration.left = left;
    g_lineCalibration.right = right;
    g_lineCalibration.sequence = previous.sequence + 1U;
    g_lineCalibrationGeneration++;
}

void UdpTelemetryPublishCal(const char *text) { if (text == NULL) return; g_calGeneration++; (void)snprintf(g_calText,sizeof(g_calText),"%s",text); g_calGeneration++; }

int UdpTelemetryQueueExperimentText(const char *text)
{
    uint32_t index;

    if (text == NULL || g_experimentEventQueueCount >=
                        UDP_EXPERIMENT_EVENT_QUEUE_CAPACITY) {
        return 0;
    }
    index = g_experimentEventQueueHead;
    (void)snprintf(g_experimentEventQueue[index],
                   sizeof(g_experimentEventQueue[index]), "%s", text);
    g_experimentEventQueueHead = (index + 1U) % UDP_EXPERIMENT_EVENT_QUEUE_CAPACITY;
    g_experimentEventQueueCount++;
    return 1;
}

static int UdpTelemetryPopExperimentText(char *text, uint32_t textSize)
{
    uint32_t index;

    if (text == NULL || textSize == 0U || g_experimentEventQueueCount == 0U) {
        return 0;
    }
    index = g_experimentEventQueueTail;
    (void)snprintf(text, textSize, "%s", g_experimentEventQueue[index]);
    g_experimentEventQueueTail = (index + 1U) % UDP_EXPERIMENT_EVENT_QUEUE_CAPACITY;
    g_experimentEventQueueCount--;
    return 1;
}

void UdpTelemetryUpdateEncoder(const UdpEncoderTelemetryState *state)
{
    if (state == NULL) return;
    g_encoderGeneration++;
    g_encoder = *state;
    g_encoderGeneration++;
}

void UdpTelemetryUpdateAvoid(const UdpAvoidTelemetryState *state)
{
    UdpAvoidTelemetryState previous;

    if (state == NULL) {
        return;
    }

    UdpTelemetryReadAvoid(&previous);
    g_avoidGeneration++;
    g_avoid = *state;
    if (previous.leftCm != state->leftCm || previous.frontCm != state->frontCm ||
        previous.rightCm != state->rightCm || previous.leftValid != state->leftValid ||
        previous.frontValid != state->frontValid || previous.rightValid != state->rightValid ||
        previous.action != state->action || previous.leftCommand != state->leftCommand ||
        previous.rightCommand != state->rightCommand) {
        g_avoid.sequence = previous.sequence + 1U;
    }
    g_avoidGeneration++;
}

void UdpTelemetryUpdateRace(const UdpRaceTelemetryState *state)
{
    UdpRaceTelemetryState previous;

    if (state == NULL) {
        return;
    }

    UdpTelemetryReadRace(&previous);
    g_raceGeneration++;
    g_race = *state;
    g_race.sequence = previous.sequence + 1U;
    g_raceGeneration++;
}

void UdpTelemetryUpdateRaceDebug(const UdpRaceDebugState *state)
{
    UdpRaceDebugState previous;

    if (state == NULL) {
        return;
    }

    UdpTelemetryReadRaceDebug(&previous);
    g_raceDebugGeneration++;
    g_raceDebug = *state;
    g_raceDebug.sequence = previous.sequence + 1U;
    g_raceDebugGeneration++;
}

void UdpTelemetryUpdateReverse(const UdpReverseTelemetryState *state)
{
    UdpReverseTelemetryState previous;

    if (state == NULL) {
        return;
    }

    UdpTelemetryReadReverse(&previous);
    g_reverseGeneration++;
    g_reverse = *state;
    g_reverse.sequence = previous.sequence + 1U;
    g_reverseGeneration++;
}

void UdpTelemetryUpdateReverseV2(const UdpReverseV2TelemetryState *state)
{
    UdpReverseV2TelemetryState previous;

    if (state == NULL) {
        return;
    }

    UdpTelemetryReadReverseV2(&previous);
    g_reverseV2Generation++;
    g_reverseV2 = *state;
    g_reverseV2.sequence = previous.sequence + 1U;
    g_reverseV2Generation++;
}

void UdpTelemetryUpdateReverseV3(const UdpReverseV3TelemetryState *state)
{
    UdpReverseV3TelemetryState previous;

    if (state == NULL) {
        return;
    }

    UdpTelemetryReadReverseV3(&previous);
    g_reverseV3Generation++;
    g_reverseV3 = *state;
    g_reverseV3.sequence = previous.sequence + 1U;
    g_reverseV3Generation++;
}

void UdpTelemetryUpdateReverseV4(const UdpReverseV4TelemetryState *state)
{
    UdpReverseV4TelemetryState previous;

    if (state == NULL) {
        return;
    }
    UdpTelemetryReadReverseV4(&previous);
    g_reverseV4Generation++;
    g_reverseV4 = *state;
    g_reverseV4.sequence = previous.sequence + 1U;
    g_reverseV4Generation++;
}

void UdpTelemetryUpdateReplay(const UdpReplayTelemetryState *state)
{
    UdpReplayTelemetryState previous;

    if (state == NULL) {
        return;
    }

    UdpTelemetryReadReplay(&previous);
    g_replayGeneration++;
    g_replay = *state;
    g_replay.sequence = previous.sequence + 1U;
    g_replayGeneration++;

    g_replayEventQueue[g_replayEventQueueHead] = g_replay;
    g_replayEventQueueHead = (g_replayEventQueueHead + 1U) %
                             UDP_REPLAY_EVENT_QUEUE_CAPACITY;
    if (g_replayEventQueueCount < UDP_REPLAY_EVENT_QUEUE_CAPACITY) {
        g_replayEventQueueCount++;
    } else {
        g_replayEventQueueTail = (g_replayEventQueueTail + 1U) %
                                  UDP_REPLAY_EVENT_QUEUE_CAPACITY;
    }
}

static int UdpTelemetryPopReplayEvent(UdpReplayTelemetryState *state)
{
    uint32_t index;

    if (g_replayEventQueueCount == 0U) {
        return 0;
    }
    index = g_replayEventQueueTail;
    *state = g_replayEventQueue[index];
    g_replayEventQueueTail = (index + 1U) % UDP_REPLAY_EVENT_QUEUE_CAPACITY;
    g_replayEventQueueCount--;
    return 1;
}

void UdpTelemetryRecordReplayHistory(const UdpReplayHistoryFrame *frame)
{
    uint32_t index;

    if (frame == NULL) {
        return;
    }
    index = g_replayHistoryWriteIndex;
    g_replayHistory[index] = *frame;
    index = (index + 1U) % UDP_REPLAY_HISTORY_CAPACITY;
    g_replayHistoryWriteIndex = index;
    if (g_replayHistoryCount < UDP_REPLAY_HISTORY_CAPACITY) {
        g_replayHistoryCount++;
    }
}

void UdpTelemetryRequestReplayHistoryDump(void)
{
    g_replayHistoryDumpGeneration++;
}

void UdpTelemetryUpdateTraceDebug(const UdpTraceDebugState *state)
{
    UdpTraceDebugState previous;

    if (state == NULL) {
        return;
    }

    UdpTelemetryReadTraceDebug(&previous);
    g_traceDebugGeneration++;
    g_traceDebug = *state;
    g_traceDebug.sequence = previous.sequence + 1U;
    g_traceDebugGeneration++;
}

static int UdpTelemetryCreateSocket(struct sockaddr_in *target)
{
    int socketFd;

    (void)memset(target, 0, sizeof(*target));
    target->sin_family = AF_INET;
    target->sin_port = htons(UDP_TELEMETRY_PORT);
    if (inet_aton(UDP_TELEMETRY_HOST, &target->sin_addr) == 0) {
        return -1;
    }

    socketFd = socket(AF_INET, SOCK_DGRAM, 0);
    return socketFd;
}

static int UdpCommandCreateSocket(void)
{
    struct sockaddr_in local;
    struct timeval receiveTimeout;
    int socketFd;

    socketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd < 0) {
        return -1;
    }
    (void)memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = htons(UDP_COMMAND_PORT);
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(socketFd, (const struct sockaddr *)&local, sizeof(local)) != 0) {
        (void)lwip_close(socketFd);
        return -1;
    }
    receiveTimeout.tv_sec = 0;
    receiveTimeout.tv_usec = UDP_COMMAND_RECV_TIMEOUT_MS * 1000U;
    if (setsockopt(socketFd, SOL_SOCKET, SO_RCVTIMEO, &receiveTimeout,
                   sizeof(receiveTimeout)) != 0) {
        (void)lwip_close(socketFd);
        return -1;
    }
    return socketFd;
}

static const char *UdpCommandHandlePayload(const char *payload, int length,
                                           const struct sockaddr_in *sender)
{
    char rcLine[UDP_COMMAND_RX_BUFFER_SIZE];
    RemoteControlState remote;
    unsigned int seq, admin, ge, le, re, ae;
    int gear, steer;
    if (length > 0 && payload[length - 1] == '\r') {
        length--;
    } else if (length > 0 && payload[length - 1] == '\n') {
        length--;
        if (length > 0 && payload[length - 1] == '\r') {
            length--;
        }
    }

    if (length > 0 && length < (int)sizeof(rcLine)) {
        (void)memcpy(rcLine, payload, (size_t)length);
        rcLine[length] = '\0';
    } else {
        rcLine[0] = '\0';
    }

    if (length > 4 && sscanf(rcLine, "RC1 seq=%u gear=%d steer=%d admin=%u ge=%u le=%u re=%u ae=%u",
                             &seq, &gear, &steer, &admin, &ge, &le, &re, &ae) == 8) {
        if (gear >= -3 && gear <= 3 && steer >= -1 && steer <= 1 && admin <= 1U) {
            remote.gear = (int8_t)gear;
            remote.steer = (int8_t)steer;
            remote.adminStop = (uint8_t)admin;
            remote.sequence = seq;
            remote.gearEvent = ge;
            remote.leftEvent = le;
            remote.rightEvent = re;
            remote.adminEvent = ae;
            CarControlSubmitRemoteState(&remote);
            return "OK RC1";
        }
    }

    if (length == (int)(sizeof("BPATH START") - 1U) &&
        memcmp(payload, "BPATH START", sizeof("BPATH START") - 1U) == 0) {
        CarControlSubmitBpathCommandFromSource(BPATH_CONTROL_COMMAND_START,
                                               BPATH_COMMAND_SOURCE_UDP);
        return "OK START";
    }
    if (length == (int)(sizeof("BPATH RETURN") - 1U) &&
        memcmp(payload, "BPATH RETURN", sizeof("BPATH RETURN") - 1U) == 0) {
        char text[160];

        (void)snprintf(text, sizeof(text),
            "BPATHCMD event=RX cmd=RETURN source_ip=%s source_port=%u",
            inet_ntoa(sender->sin_addr), (unsigned int)ntohs(sender->sin_port));
        (void)UdpTelemetryQueueExperimentText(text);
        CarControlSubmitBpathCommandFromSource(BPATH_CONTROL_COMMAND_RETURN,
                                               BPATH_COMMAND_SOURCE_UDP);
        return "OK RETURN";
    }
    if (length == (int)(sizeof("BPATH RESET") - 1U) &&
        memcmp(payload, "BPATH RESET", sizeof("BPATH RESET") - 1U) == 0) {
        CarControlSubmitBpathCommandFromSource(BPATH_CONTROL_COMMAND_RESET,
                                               BPATH_COMMAND_SOURCE_UDP);
        return "OK RESET";
    }
#if (FORK_BACKTRACK_PROOF_TEST_MODE == 1)
    if (length == (int)(sizeof("FORKTEST RESET") - 1U) &&
        memcmp(payload, "FORKTEST RESET", sizeof("FORKTEST RESET") - 1U) == 0) {
        CarControlSubmitForkTestCommand(FORK_TEST_COMMAND_RESET);
        return "OK FORKTEST RESET";
    }
    if (length == (int)(sizeof("FORKTEST MARK LEFT") - 1U) &&
        memcmp(payload, "FORKTEST MARK LEFT", sizeof("FORKTEST MARK LEFT") - 1U) == 0) {
        CarControlSubmitForkTestCommand(FORK_TEST_COMMAND_MARK_LEFT);
        return "OK FORKTEST MARK LEFT";
    }
    if (length == (int)(sizeof("FORKTEST MARK RIGHT") - 1U) &&
        memcmp(payload, "FORKTEST MARK RIGHT", sizeof("FORKTEST MARK RIGHT") - 1U) == 0) {
        CarControlSubmitForkTestCommand(FORK_TEST_COMMAND_MARK_RIGHT);
        return "OK FORKTEST MARK RIGHT";
    }
    if (length == (int)(sizeof("FORKTEST GO") - 1U) &&
        memcmp(payload, "FORKTEST GO", sizeof("FORKTEST GO") - 1U) == 0) {
        CarControlSubmitForkTestCommand(FORK_TEST_COMMAND_GO);
        return "OK FORKTEST GO";
    }
#endif
#if (MOTOR_RESPONSE_TEST_MODE == 1)
    if (length == (int)(sizeof("MOTORCAL START") - 1U) &&
        memcmp(payload, "MOTORCAL START", sizeof("MOTORCAL START") - 1U) == 0) {
        CarControlSubmitMotorResponseCommand(MOTOR_RESPONSE_COMMAND_START);
        return "OK MOTORCAL START";
    }
    if (length == (int)(sizeof("MOTORCAL STOP") - 1U) &&
        memcmp(payload, "MOTORCAL STOP", sizeof("MOTORCAL STOP") - 1U) == 0) {
        CarControlSubmitMotorResponseCommand(MOTOR_RESPONSE_COMMAND_STOP);
        return "OK MOTORCAL STOP";
    }
#endif
#if (MOTOR_TURN_DIRECTION_TEST_MODE == 1)
    if (length == (int)(sizeof("TURNTEST LEFT") - 1U) &&
        memcmp(payload, "TURNTEST LEFT", sizeof("TURNTEST LEFT") - 1U) == 0) {
        CarControlSubmitMotorTurnDirectionCommand(MOTOR_TURN_DIRECTION_COMMAND_LEFT);
        return "OK TURNTEST LEFT";
    }
    if (length == (int)(sizeof("TURNTEST RIGHT") - 1U) &&
        memcmp(payload, "TURNTEST RIGHT", sizeof("TURNTEST RIGHT") - 1U) == 0) {
        CarControlSubmitMotorTurnDirectionCommand(MOTOR_TURN_DIRECTION_COMMAND_RIGHT);
        return "OK TURNTEST RIGHT";
    }
    if (length == (int)(sizeof("TURNTEST FWD") - 1U) &&
        memcmp(payload, "TURNTEST FWD", sizeof("TURNTEST FWD") - 1U) == 0) {
        CarControlSubmitMotorTurnDirectionCommand(MOTOR_TURN_DIRECTION_COMMAND_FORWARD);
        return "OK TURNTEST FWD";
    }
    if (length == (int)(sizeof("TURNTEST STOP") - 1U) &&
        memcmp(payload, "TURNTEST STOP", sizeof("TURNTEST STOP") - 1U) == 0) {
        CarControlSubmitMotorTurnDirectionCommand(MOTOR_TURN_DIRECTION_COMMAND_STOP);
        return "OK TURNTEST STOP";
    }
#endif
#if (TRACE_OBSERVER_TEST_MODE == 1)
    if (length == (int)(sizeof("TRACEOBS START") - 1U) &&
        memcmp(payload, "TRACEOBS START", sizeof("TRACEOBS START") - 1U) == 0) {
        CarControlSubmitTraceObserverCommand(TRACE_OBSERVER_COMMAND_START);
        return "OK TRACEOBS START";
    }
    if (length == (int)(sizeof("TRACEOBS STOP") - 1U) &&
        memcmp(payload, "TRACEOBS STOP", sizeof("TRACEOBS STOP") - 1U) == 0) {
        CarControlSubmitTraceObserverCommand(TRACE_OBSERVER_COMMAND_STOP);
        return "OK TRACEOBS STOP";
    }
#endif
#if (TRACE_STEP_RESPONSE_TEST_MODE == 1)
    if (length == (int)(sizeof("TRACEPULSE LEFT 100") - 1U) &&
        memcmp(payload, "TRACEPULSE LEFT 100", sizeof("TRACEPULSE LEFT 100") - 1U) == 0) {
        CarControlSubmitTraceStepResponseCommand(TRACE_STEP_RESPONSE_COMMAND_LEFT_100);
        return "OK TRACEPULSE LEFT 100";
    }
    if (length == (int)(sizeof("TRACEPULSE LEFT 200") - 1U) &&
        memcmp(payload, "TRACEPULSE LEFT 200", sizeof("TRACEPULSE LEFT 200") - 1U) == 0) {
        CarControlSubmitTraceStepResponseCommand(TRACE_STEP_RESPONSE_COMMAND_LEFT_200);
        return "OK TRACEPULSE LEFT 200";
    }
    if (length == (int)(sizeof("TRACEPULSE LEFT 300") - 1U) &&
        memcmp(payload, "TRACEPULSE LEFT 300", sizeof("TRACEPULSE LEFT 300") - 1U) == 0) {
        CarControlSubmitTraceStepResponseCommand(TRACE_STEP_RESPONSE_COMMAND_LEFT_300);
        return "OK TRACEPULSE LEFT 300";
    }
    if (length == (int)(sizeof("TRACEPULSE RIGHT 100") - 1U) &&
        memcmp(payload, "TRACEPULSE RIGHT 100", sizeof("TRACEPULSE RIGHT 100") - 1U) == 0) {
        CarControlSubmitTraceStepResponseCommand(TRACE_STEP_RESPONSE_COMMAND_RIGHT_100);
        return "OK TRACEPULSE RIGHT 100";
    }
    if (length == (int)(sizeof("TRACEPULSE RIGHT 200") - 1U) &&
        memcmp(payload, "TRACEPULSE RIGHT 200", sizeof("TRACEPULSE RIGHT 200") - 1U) == 0) {
        CarControlSubmitTraceStepResponseCommand(TRACE_STEP_RESPONSE_COMMAND_RIGHT_200);
        return "OK TRACEPULSE RIGHT 200";
    }
    if (length == (int)(sizeof("TRACEPULSE RIGHT 300") - 1U) &&
        memcmp(payload, "TRACEPULSE RIGHT 300", sizeof("TRACEPULSE RIGHT 300") - 1U) == 0) {
        CarControlSubmitTraceStepResponseCommand(TRACE_STEP_RESPONSE_COMMAND_RIGHT_300);
        return "OK TRACEPULSE RIGHT 300";
    }
    if (length == (int)(sizeof("TRACEPULSE STOP") - 1U) &&
        memcmp(payload, "TRACEPULSE STOP", sizeof("TRACEPULSE STOP") - 1U) == 0) {
        CarControlSubmitTraceStepResponseCommand(TRACE_STEP_RESPONSE_COMMAND_STOP);
        return "OK TRACEPULSE STOP";
    }
#endif

#if (MOTOR_TURN_DIRECTION_TEST_MODE == 1)
    (void)UdpTelemetryQueueExperimentText(
        "TURNTEST event=IGNORE cmd=UDP reason=UNKNOWN");
#elif (MOTOR_RESPONSE_TEST_MODE == 1)
    (void)UdpTelemetryQueueExperimentText(
        "MOTORCAL event=IGNORE cmd=UDP reason=UNKNOWN");
#else
    (void)UdpTelemetryQueueExperimentText(
        "BPATHCTL event=IGNORE cmd=UDP reason=UNKNOWN");
#endif
    return "ERR UNKNOWN";
}

static void UdpCommandTask(void *argument)
{
    int socketFd = -1;

    (void)argument;
    printf("UDP command waiting network\r\n");
    for (;;) {
        if (TaskWifiIsNetworkReady() == 0) {
            if (socketFd >= 0) {
                (void)lwip_close(socketFd);
                socketFd = -1;
            }
            osDelay(AppMsToTicks(UDP_TELEMETRY_PERIOD_MS));
            continue;
        }
        if (socketFd < 0) {
            socketFd = UdpCommandCreateSocket();
            if (socketFd < 0) {
                printf("UDP command socket create failed\r\n");
                osDelay(AppMsToTicks(UDP_TELEMETRY_SOCKET_RETRY_MS));
                continue;
            }
            printf("UDP command listening on %u\r\n", (unsigned int)UDP_COMMAND_PORT);
        }
        {
            char payload[UDP_COMMAND_RX_BUFFER_SIZE];
            struct sockaddr_in sender;
            socklen_t senderLength = sizeof(sender);
            int received = recvfrom(socketFd, payload, sizeof(payload), 0,
                                    (struct sockaddr *)&sender, &senderLength);
            if (received >= 0) {
                const char *ack = UdpCommandHandlePayload(payload, received, &sender);
                (void)sendto(socketFd, ack, strlen(ack), 0,
                             (const struct sockaddr *)&sender, senderLength);
            }
        }
    }
}

static int UdpTelemetrySend(int socketFd, const struct sockaddr_in *target,
                            const char *kind, const CarTelemetryState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "%s ms=%u seq=%u sensor=%u%u action=%s L=%d R=%d\r\n",
                       kind, (unsigned int)UdpTelemetryUptimeMs(),
                       (unsigned int)state->sequence,
                       (unsigned int)((state->stableState >> 1) & 0x01U),
                       (unsigned int)(state->stableState & 0x01U),
                       UdpTelemetryActionName(state->action),
                       state->leftCommand, state->rightCommand);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }

    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static int UdpTelemetrySendEncoder(int socketFd, const struct sockaddr_in *target,
                                   const char *kind, const UdpEncoderTelemetryState *state)
{
    int textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
        "%s ms=%u seq=%u dl=%d dr=%d rx_valid=%u bad_checksum=%u bad_frame=%u last_rx_ms=%u\r\n",
        kind,
        (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
        state->leftDelta, state->rightDelta, (unsigned int)state->validCount,
        (unsigned int)state->badChecksumCount, (unsigned int)state->badFrameCount,
        (unsigned int)state->lastRxMs);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) return -1;
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static int UdpTelemetrySendSensorStats(int socketFd, const struct sockaddr_in *target,
                                       const UdpSensorStats *stats)
{
    uint32_t leftTotal = stats->left0 + stats->left1;
    uint32_t rightTotal = stats->right0 + stats->right1;
    uint32_t leftPctTenth = 0U;
    uint32_t rightPctTenth = 0U;
    int textLen;

    if (leftTotal != 0U) {
        leftPctTenth = (stats->left1 * 1000U + (leftTotal / 2U)) / leftTotal;
    }
    if (rightTotal != 0U) {
        rightPctTenth = (stats->right1 * 1000U + (rightTotal / 2U)) / rightTotal;
    }

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "SENSORSTAT ms=%u seq=%u L0=%u L1=%u Lt=%u Lpct1=%u.%u "
                       "R0=%u R1=%u Rt=%u Rpct1=%u.%u\r\n",
                       (unsigned int)UdpTelemetryUptimeMs(),
                       (unsigned int)stats->sequence,
                       (unsigned int)stats->left0, (unsigned int)stats->left1,
                       (unsigned int)stats->leftTransitions,
                       (unsigned int)(leftPctTenth / 10U),
                       (unsigned int)(leftPctTenth % 10U),
                       (unsigned int)stats->right0, (unsigned int)stats->right1,
                       (unsigned int)stats->rightTransitions,
                       (unsigned int)(rightPctTenth / 10U),
                       (unsigned int)(rightPctTenth % 10U));
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }

    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static int UdpTelemetrySendLineCalibration(int socketFd, const struct sockaddr_in *target,
                                           const UdpLineCalibrationState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText), "CAL L=%u R=%u\r\n",
                       (unsigned int)state->left, (unsigned int)state->right);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }

    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static const char *UdpAvoidActionName(UdpAvoidAction action)
{
    switch (action) {
        case UDP_AVOID_ACTION_FORWARD:
            return "FWD";
        case UDP_AVOID_ACTION_LEFT_TURN:
            return "LTURN";
        case UDP_AVOID_ACTION_RIGHT_TURN:
            return "RTURN";
        case UDP_AVOID_ACTION_BLOCKED:
            return "BLOCKED";
        default:
            return "STOP";
    }
}

static void UdpAvoidDistanceText(char *text, uint32_t size, uint8_t valid, float distance)
{
    if (valid != 0U) {
        (void)snprintf(text, size, "%.1f", (double)distance);
    } else {
        (void)snprintf(text, size, "NA");
    }
}

static int UdpTelemetrySendAvoid(int socketFd, const struct sockaddr_in *target,
                                 const UdpAvoidTelemetryState *state)
{
    int textLen;

    UdpAvoidDistanceText(g_udpAvoidLeftText, sizeof(g_udpAvoidLeftText), state->leftValid, state->leftCm);
    UdpAvoidDistanceText(g_udpAvoidFrontText, sizeof(g_udpAvoidFrontText), state->frontValid, state->frontCm);
    UdpAvoidDistanceText(g_udpAvoidRightText, sizeof(g_udpAvoidRightText), state->rightValid, state->rightCm);
    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "AVOID ms=%u seq=%u front=%s left=%s right=%s action=%s L=%d R=%d\r\n",
                       (unsigned int)UdpTelemetryUptimeMs(),
                       (unsigned int)state->sequence, g_udpAvoidFrontText,
                       g_udpAvoidLeftText, g_udpAvoidRightText,
                       UdpAvoidActionName(state->action), state->leftCommand,
                       state->rightCommand);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static const char *UdpRaceEventName(UdpRaceEvent event)
{
    switch (event) {
        case UDP_RACE_EVENT_START_MARKER:
            return "START_MARKER";
        case UDP_RACE_EVENT_START_CLEAR:
            return "START_CLEAR";
        case UDP_RACE_EVENT_SENSOR:
            return "SENSOR";
        case UDP_RACE_EVENT_MARKER1:
            return "MARKER1";
        case UDP_RACE_EVENT_MARKER1_CLEAR:
            return "MARKER1_CLEAR";
        case UDP_RACE_EVENT_FINISH_MARKER2:
            return "FINISH_MARKER2";
        case UDP_RACE_EVENT_FINISH_SLOW:
            return "FINISH_SLOW";
        case UDP_RACE_EVENT_FINISH_STOP:
            return "FINISH_STOP";
        case UDP_RACE_EVENT_DEADEND:
            return "DEADEND";
        case UDP_RACE_EVENT_DEADEND_SUMMARY:
            return "DEADEND_SUMMARY";
        default:
            return "HEARTBEAT";
    }
}

static int UdpTelemetrySendRace(int socketFd, const struct sockaddr_in *target,
                                const UdpRaceTelemetryState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "RACE event=%s ms=%u seq=%u state=%u sensor=%u%u action=%s L=%d R=%d "
                       "elapsed=%u prev_sensor=%u%u last_non00=%u%u last_non11=%u%u last_action_ms=%u "
                       "transitions=%u left_corr_count=%u right_corr_count=%u "
                       "left_corr_ms=%u right_corr_ms=%u marker_gap_ms=%u marker1_elapsed_ms=%u "
                       "probe_elapsed_ms=%u since_last_10_ms=%u since_last_01_ms=%u\r\n",
                       UdpRaceEventName(state->event),
                       (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
                       (unsigned int)state->raceState,
                       (unsigned int)((state->sensorState >> 1) & 0x01U),
                       (unsigned int)(state->sensorState & 0x01U),
                       UdpTelemetryActionName(state->action), state->leftCommand,
                       state->rightCommand, (unsigned int)state->elapsedMs,
                       (unsigned int)((state->previousSensorState >> 1) & 0x01U),
                       (unsigned int)(state->previousSensorState & 0x01U),
                       (unsigned int)((state->lastNon00SensorState >> 1) & 0x01U),
                       (unsigned int)(state->lastNon00SensorState & 0x01U),
                       (unsigned int)((state->lastNon11SensorState >> 1) & 0x01U),
                       (unsigned int)(state->lastNon11SensorState & 0x01U),
                       (unsigned int)state->lastActionChangeMs,
                       (unsigned int)state->transitionCount,
                       (unsigned int)state->leftCorrectionCount,
                       (unsigned int)state->rightCorrectionCount,
                       (unsigned int)state->leftCorrectionMs,
                       (unsigned int)state->rightCorrectionMs,
                       (unsigned int)state->markerGapMs,
                       (unsigned int)state->marker1ElapsedMs,
                       (unsigned int)state->probeElapsedMs,
                       (unsigned int)state->last10AgeMs,
                       (unsigned int)state->last01AgeMs);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static const char *UdpRaceCorrectionName(uint8_t correction)
{
    if (correction == 1U) {
        return "LEFT";
    }
    if (correction == 2U) {
        return "RIGHT";
    }
    return "NONE";
}

static int UdpTelemetrySendRaceDebug(int socketFd, const struct sockaddr_in *target,
                                     const UdpRaceDebugState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "RACEDEBUG ms=%u seq=%u race_state=%u sensor=%u%u physical_left=%u "
                       "physical_right=%u prev_sensor=%u%u last_non00=%u%u last_non11=%u%u "
                       "action=%s L=%d R=%d last_correction=%s correction_age_ms=%u "
                       "left_corr_count=%u right_corr_count=%u transition_count=%u elapsed=%u\r\n",
                       (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
                       (unsigned int)state->raceState,
                       (unsigned int)((state->sensorState >> 1) & 0x01U),
                       (unsigned int)(state->sensorState & 0x01U),
                       (unsigned int)state->rawLeft, (unsigned int)state->rawRight,
                       (unsigned int)((state->previousSensorState >> 1) & 0x01U),
                       (unsigned int)(state->previousSensorState & 0x01U),
                       (unsigned int)((state->lastNon00SensorState >> 1) & 0x01U),
                       (unsigned int)(state->lastNon00SensorState & 0x01U),
                       (unsigned int)((state->lastNon11SensorState >> 1) & 0x01U),
                       (unsigned int)(state->lastNon11SensorState & 0x01U),
                       UdpTelemetryActionName(state->action), state->leftCommand,
                       state->rightCommand,
                       UdpRaceCorrectionName(state->lastCorrection),
                       (unsigned int)state->correctionAgeMs,
                       (unsigned int)state->leftCorrectionCount,
                       (unsigned int)state->rightCorrectionCount,
                       (unsigned int)state->transitionCount,
                       (unsigned int)state->elapsedMs);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static const char *UdpReverseEventName(UdpReverseEvent event)
{
    switch (event) {
        case UDP_REVERSE_EVENT_START:
            return "START";
        case UDP_REVERSE_EVENT_SENSOR_11_STOP:
            return "SENSOR_11_STOP";
        case UDP_REVERSE_EVENT_HEARTBEAT:
            return "HEARTBEAT";
        default:
            return "SENSOR";
    }
}

static const char *UdpReverseActionName(UdpReverseAction action)
{
    switch (action) {
        case UDP_REVERSE_ACTION_FORWARD:
            return "REV_FWD";
        case UDP_REVERSE_ACTION_LEFT:
            return "REV_LEFT";
        case UDP_REVERSE_ACTION_RIGHT:
            return "REV_RIGHT";
        default:
            return "STOP";
    }
}

static int UdpTelemetrySendReverse(int socketFd, const struct sockaddr_in *target,
                                   const UdpReverseTelemetryState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "REVERSE event=%s ms=%u seq=%u sensor=%u%u action=%s L=%d R=%d\r\n",
                       UdpReverseEventName(state->event),
                       (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
                       (unsigned int)((state->sensorState >> 1) & 0x01U),
                       (unsigned int)(state->sensorState & 0x01U),
                       UdpReverseActionName(state->action), state->leftCommand,
                       state->rightCommand);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static int UdpTelemetrySendReverseDebug(int socketFd, const struct sockaddr_in *target,
                                        const UdpReverseTelemetryState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "REVERSEDEBUG ms=%u seq=%u sensor=%u%u physical_left=%u physical_right=%u "
                       "prev_sensor=%u%u action=%s L=%d R=%d last_correction=%s "
                       "consecutive_00=%u consecutive_01=%u consecutive_10=%u consecutive_11=%u\r\n",
                       (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
                       (unsigned int)((state->sensorState >> 1) & 0x01U),
                       (unsigned int)(state->sensorState & 0x01U),
                       (unsigned int)state->rawLeft, (unsigned int)state->rawRight,
                       (unsigned int)((state->previousSensorState >> 1) & 0x01U),
                       (unsigned int)(state->previousSensorState & 0x01U),
                       UdpReverseActionName(state->action), state->leftCommand,
                       state->rightCommand,
                       (state->lastCorrection == 1U) ? "LEFT" :
                       ((state->lastCorrection == 2U) ? "RIGHT" : "NONE"),
                       (unsigned int)state->consecutive00,
                       (unsigned int)state->consecutive01,
                       (unsigned int)state->consecutive10,
                       (unsigned int)state->consecutive11);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static const char *UdpReverseV2EventName(UdpReverseV2Event event)
{
    switch (event) {
        case UDP_REVERSE_V2_EVENT_START:
            return "START";
        case UDP_REVERSE_V2_EVENT_ALIGN_LEFT_START:
        case UDP_REVERSE_V2_EVENT_ALIGN_RIGHT_START:
            return "ALIGN_START";
        case UDP_REVERSE_V2_EVENT_ALIGN_CLEAR:
            return "ALIGN_CLEAR";
        case UDP_REVERSE_V2_EVENT_BACK_RESUME:
            return "BACK_RESUME";
        case UDP_REVERSE_V2_EVENT_ALIGN_TIMEOUT:
            return "ALIGN_TIMEOUT";
        case UDP_REVERSE_V2_EVENT_SENSOR_11_STOP:
            return "SENSOR_11_STOP";
        case UDP_REVERSE_V2_EVENT_HEARTBEAT:
            return "HEARTBEAT";
        default:
            return "SENSOR";
    }
}

static const char *UdpReverseV2TriggerName(uint8_t sensorState)
{
    if (sensorState == 0x02U) {
        return "LEFT";
    }
    if (sensorState == 0x01U) {
        return "RIGHT";
    }
    return "NONE";
}

static const char *UdpReverseV2YawName(UdpReverseV2State state)
{
    if (state == UDP_REVERSE_V2_STATE_ALIGN_LEFT) {
        return "LEFT";
    }
    if (state == UDP_REVERSE_V2_STATE_ALIGN_RIGHT) {
        return "RIGHT";
    }
    return "NONE";
}

static const char *UdpReverseV2StateName(UdpReverseV2State state)
{
    switch (state) {
        case UDP_REVERSE_V2_STATE_BACK:
            return "BACK";
        case UDP_REVERSE_V2_STATE_ALIGN_LEFT:
            return "ALIGN_L";
        case UDP_REVERSE_V2_STATE_ALIGN_RIGHT:
            return "ALIGN_R";
        case UDP_REVERSE_V2_STATE_SETTLING:
            return "SETTLE";
        default:
            return "STOP";
    }
}

static int UdpTelemetrySendReverseV2(int socketFd, const struct sockaddr_in *target,
                                     const UdpReverseV2TelemetryState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "REVERSEV2 event=%s trigger=%s yaw=%s ms=%u seq=%u sensor=%u%u "
                       "state=%s L=%d R=%d\r\n",
                       UdpReverseV2EventName(state->event),
                       UdpReverseV2TriggerName(state->sensorState),
                       UdpReverseV2YawName(state->state),
                       (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
                       (unsigned int)((state->sensorState >> 1) & 0x01U),
                       (unsigned int)(state->sensorState & 0x01U),
                       UdpReverseV2StateName(state->state), state->leftCommand,
                       state->rightCommand);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static int UdpTelemetrySendReverseV2Debug(int socketFd, const struct sockaddr_in *target,
                                          const UdpReverseV2TelemetryState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "REVERSEV2DEBUG ms=%u seq=%u state=%s trigger=%s yaw=%s sensor=%u%u physical_left=%u "
                       "physical_right=%u L=%d R=%d align_elapsed_ms=%u consecutive_00=%u "
                       "consecutive_01=%u consecutive_10=%u consecutive_11=%u\r\n",
                       (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
                       UdpReverseV2StateName(state->state),
                       UdpReverseV2TriggerName(state->sensorState),
                       UdpReverseV2YawName(state->state),
                       (unsigned int)((state->sensorState >> 1) & 0x01U),
                       (unsigned int)(state->sensorState & 0x01U),
                       (unsigned int)state->rawLeft, (unsigned int)state->rawRight,
                       state->leftCommand, state->rightCommand,
                       (unsigned int)state->alignElapsedMs,
                       (unsigned int)state->consecutive00,
                       (unsigned int)state->consecutive01,
                       (unsigned int)state->consecutive10,
                       (unsigned int)state->consecutive11);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static const char *UdpReverseV3EventName(UdpReverseV3Event event)
{
    switch (event) {
        case UDP_REVERSE_V3_EVENT_START:
            return "START";
        case UDP_REVERSE_V3_EVENT_SENSOR_11_STOP:
            return "SENSOR_11_STOP";
        case UDP_REVERSE_V3_EVENT_HEARTBEAT:
            return "HEARTBEAT";
        default:
            return "SENSOR";
    }
}

static const char *UdpReverseV3ActionName(UdpReverseV3Action action)
{
    switch (action) {
        case UDP_REVERSE_V3_ACTION_BACK:
            return "REV_BACK";
        case UDP_REVERSE_V3_ACTION_CORRECT_LEFT:
            return "REV_CORRECT_LEFT";
        case UDP_REVERSE_V3_ACTION_CORRECT_RIGHT:
            return "REV_CORRECT_RIGHT";
        default:
            return "STOP";
    }
}

static const char *UdpReverseV3TriggerName(uint8_t sensorState)
{
    if (sensorState == 0x02U) {
        return "LEFT";
    }
    if (sensorState == 0x01U) {
        return "RIGHT";
    }
    return "NONE";
}

static const char *UdpReverseV3YawName(UdpReverseV3Action action)
{
    if (action == UDP_REVERSE_V3_ACTION_CORRECT_LEFT) {
        return "RIGHT";
    }
    if (action == UDP_REVERSE_V3_ACTION_CORRECT_RIGHT) {
        return "LEFT";
    }
    return "NONE";
}

static int UdpTelemetrySendReverseV3(int socketFd, const struct sockaddr_in *target,
                                     const UdpReverseV3TelemetryState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "REVERSEV3 event=%s trigger=%s yaw=%s ms=%u seq=%u sensor=%u%u "
                       "action=%s L=%d R=%d\r\n",
                       UdpReverseV3EventName(state->event),
                       UdpReverseV3TriggerName(state->sensorState),
                       UdpReverseV3YawName(state->action),
                       (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
                       (unsigned int)((state->sensorState >> 1) & 0x01U),
                       (unsigned int)(state->sensorState & 0x01U),
                       UdpReverseV3ActionName(state->action), state->leftCommand,
                       state->rightCommand);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static int UdpTelemetrySendReverseV3Debug(int socketFd, const struct sockaddr_in *target,
                                          const UdpReverseV3TelemetryState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "REVERSEV3DEBUG ms=%u seq=%u sensor=%u%u physical_left=%u physical_right=%u "
                       "trigger=%s yaw=%s action=%s L=%d R=%d prev_sensor=%u%u "
                       "last_correction=%s consecutive_00=%u consecutive_01=%u "
                       "consecutive_10=%u consecutive_11=%u\r\n",
                       (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
                       (unsigned int)((state->sensorState >> 1) & 0x01U),
                       (unsigned int)(state->sensorState & 0x01U),
                       (unsigned int)state->rawLeft, (unsigned int)state->rawRight,
                       UdpReverseV3TriggerName(state->sensorState),
                       UdpReverseV3YawName(state->action),
                       UdpReverseV3ActionName(state->action), state->leftCommand,
                       state->rightCommand,
                       (unsigned int)((state->previousSensorState >> 1) & 0x01U),
                       (unsigned int)(state->previousSensorState & 0x01U),
                       (state->lastCorrection == 1U) ? "LEFT" :
                       ((state->lastCorrection == 2U) ? "RIGHT" : "NONE"),
                       (unsigned int)state->consecutive00,
                       (unsigned int)state->consecutive01,
                       (unsigned int)state->consecutive10,
                       (unsigned int)state->consecutive11);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static const char *UdpReverseV4EventName(UdpReverseV4Event event)
{
    switch ((int)event) {
        case UDP_REVERSE_V4_EVENT_START: return "START";
        case UDP_REVERSE_V4_EVENT_PULSE_START: return "PULSE_START";
        case UDP_REVERSE_V4_EVENT_PULSE_END: return "PULSE_END";
        case UDP_REVERSE_V4_EVENT_PROBE_START: return "PROBE_START";
        case UDP_REVERSE_V4_EVENT_PROBE_RESULT: return "PROBE_RESULT";
        case UDP_REVERSE_V4_EVENT_PULSE_LIMIT: return "PULSE_LIMIT";
        case UDP_REVERSE_V4_EVENT_SENSOR_11_STOP: return "SENSOR_11_STOP";
        case UDP_REVERSE_V4_EVENT_WAIT_CLEAR: return "WAIT_CLEAR";
        case UDP_REVERSE_V4_EVENT_ARMED: return "ARMED";
        case UDP_REVERSE_V4_EVENT_REARM: return "REARM";
        case UDP_REVERSE_V4_EVENT_HEARTBEAT: return "HEARTBEAT";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_START: return "START";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_ARMED: return "ARMED";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_HEADING_PULSE_START: return "HEADING_PULSE_START";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_HEADING_PROBE: return "HEADING_PROBE";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_LATERAL_SHIFT_START: return "LATERAL_SHIFT_START";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_SHIFT_YAW_OUT: return "SHIFT_YAW_OUT";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_SHIFT_BACK: return "SHIFT_BACK";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_SHIFT_YAW_BACK: return "SHIFT_YAW_BACK";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_RECHECK: return "RECHECK";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_LATERAL_RECENTER_SUCCESS: return "LATERAL_RECENTER_SUCCESS";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_LATERAL_RECENTER_SAME_SIDE: return "LATERAL_RECENTER_SAME_SIDE";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_LATERAL_OVERSHOOT: return "LATERAL_OVERSHOOT";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_LATERAL_SHIFT_LIMIT: return "LATERAL_SHIFT_LIMIT";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_OPPOSITE_AFTER_HEADING_PROBE: return "OPPOSITE_AFTER_HEADING_PROBE";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_REARM: return "REARM";
        case (UdpReverseV4Event)UDP_REVERSE_V5_EVENT_WAIT_CLEAR: return "WAIT_CLEAR";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_START: return "START";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_ARMED: return "ARMED";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_HEADING_START: return "HEADING_START";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_HEADING_CAPTURE_00: return "HEADING_CAPTURE_00";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_HEADING_TIMEOUT_SAME_SIDE: return "HEADING_TIMEOUT_SAME_SIDE";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_HEADING_OVERSHOOT: return "HEADING_OVERSHOOT";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_LATERAL_START: return "LATERAL_START";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_LATERAL_CAPTURE_00: return "LATERAL_CAPTURE_00";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_LATERAL_CAPTURE_TIMEOUT: return "LATERAL_CAPTURE_TIMEOUT";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_LATERAL_OPPOSITE_WITHOUT_CAPTURE: return "LATERAL_OPPOSITE_WITHOUT_CAPTURE";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_YAW_RESTORE_START: return "YAW_RESTORE_START";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_YAW_RESTORE_DONE: return "YAW_RESTORE_DONE";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_RESTORE_RETURN_ORIGINAL_SIDE: return "RESTORE_RETURN_ORIGINAL_SIDE";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_RESTORE_CROSS_OPPOSITE_SIDE: return "RESTORE_CROSS_OPPOSITE_SIDE";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_VERIFY_START: return "VERIFY_START";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_TRACK_REACQUIRED: return "TRACK_REACQUIRED";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_SENSOR_11_STOP: return "SENSOR_11_STOP";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_REARM: return "REARM";
        case (UdpReverseV4Event)UDP_REVERSE_V6_EVENT_HEARTBEAT: return "HEARTBEAT";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_START: return "START";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_ARMED: return "ARMED";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_HEADING_START: return "HEADING_START";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_SWEEP_ENTRY_YAW_START: return "SWEEP_ENTRY_YAW_START";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_SWEEP_START: return "SWEEP_START";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_CENTER_WINDOW_ENTER: return "CENTER_WINDOW_ENTER";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_OPPOSITE_EDGE_CONFIRMED: return "OPPOSITE_EDGE_CONFIRMED";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_OPPOSITE_EDGE_TIMEOUT: return "OPPOSITE_EDGE_TIMEOUT";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_RETURN_HEADING_START: return "RETURN_HEADING_START";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_RETURN_SWEEP_START: return "RETURN_SWEEP_START";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_RETURN_CENTER_CAPTURE: return "RETURN_CENTER_CAPTURE";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_YAW_RESTORE_START: return "YAW_RESTORE_START";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_VERIFY_START: return "VERIFY_START";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_TRACK_REACQUIRED: return "TRACK_REACQUIRED";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_VERIFY_ORIGINAL_SIDE: return "VERIFY_ORIGINAL_SIDE";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_VERIFY_OPPOSITE_SIDE: return "VERIFY_OPPOSITE_SIDE";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_SENSOR_11_STOP: return "SENSOR_11_STOP";
        case (UdpReverseV4Event)UDP_REVERSE_V7_EVENT_REARM: return "REARM";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_START: return "START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_ARMED: return "ARMED";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_EDGE_ENTRY_START: return "EDGE_ENTRY_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_EDGE_CANDIDATE_START: return "EDGE_CANDIDATE_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_EDGE_LOCKED: return "EDGE_LOCKED";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_EDGE_REACQUIRED: return "EDGE_REACQUIRED";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_EDGE_GAP_START: return "EDGE_GAP_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_EDGE_CORRECT_TOWARD: return "EDGE_CORRECT_TOWARD";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_EDGE_CORRECT_AWAY: return "EDGE_CORRECT_AWAY";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_EDGE_LOST: return "EDGE_LOST";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_EDGE_SIDE_SWITCH: return "EDGE_SIDE_SWITCH";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_SENSOR_11_STOP: return "SENSOR_11_STOP";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_HEARTBEAT: return "EDGE_HEARTBEAT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_SIMPLE_STATE_CHANGE: return "SIMPLE_STATE_CHANGE";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_SIMPLE_HEARTBEAT: return "SIMPLE_HEARTBEAT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_SIMPLE_SEARCH_BACK_START: return "SIMPLE_SEARCH_BACK_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_SIMPLE_EDGE_REACQUIRED: return "SIMPLE_EDGE_REACQUIRED";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_SIMPLE_LINE_LOST: return "SIMPLE_LINE_LOST";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_START: return "SIMPLE_NUDGE_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_SIMPLE_NUDGE_END: return "SIMPLE_NUDGE_END";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_PAIR_SHIFT_START: return "PAIR_SHIFT_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_PAIR_TRUSTED_00: return "PAIR_TRUSTED_00";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_PAIR_RESTORE_START: return "PAIR_RESTORE_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_PAIR_RESTORE_END: return "PAIR_RESTORE_END";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_PAIR_HIT_11: return "PAIR_HIT_11";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_PAIR_SHIFT_TIMEOUT: return "PAIR_SHIFT_TIMEOUT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_PAIR_HEARTBEAT: return "PAIR_HEARTBEAT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_MICRO_SHIFT_START: return "MICRO_SHIFT_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_MICRO_SHIFT_END: return "MICRO_SHIFT_END";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_MICRO_RESTORE_START: return "MICRO_RESTORE_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_MICRO_RESTORE_END: return "MICRO_RESTORE_END";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_MICRO_SETTLE_START: return "MICRO_SETTLE_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_MICRO_SETTLE_END: return "MICRO_SETTLE_END";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_MICRO_PAIR_REPEAT: return "MICRO_PAIR_REPEAT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_MICRO_HEARTBEAT: return "MICRO_HEARTBEAT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_BIASED_SHIFT_START: return "BIASED_SHIFT_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_BIASED_SHIFT_END: return "BIASED_SHIFT_END";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_BIASED_PARTIAL_RESTORE: return "BIASED_PARTIAL_RESTORE";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_BIASED_SETTLE: return "BIASED_SETTLE";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_BIASED_REPEAT: return "BIASED_REPEAT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_ESCAPE_11_START: return "ESCAPE_11_START";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_ESCAPE_11_PULSE: return "ESCAPE_11_PULSE";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_ESCAPE_11_EXIT: return "ESCAPE_11_EXIT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_BIASED_HEARTBEAT: return "BIASED_HEARTBEAT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_BIASED_10_SHIFT: return "BIASED_10_SHIFT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_BIASED_10_RESTORE: return "BIASED_10_RESTORE";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_BIASED_01_SHIFT: return "BIASED_01_SHIFT";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_BIASED_01_RESTORE: return "BIASED_01_RESTORE";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_AMBIGUOUS_00: return "AMBIGUOUS_00";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_AMBIGUOUS_11: return "AMBIGUOUS_11";
        case (UdpReverseV4Event)UDP_REVERSE_V8_EVENT_SIGNED_SENSOR_OVERRIDE: return "SIGNED_SENSOR_OVERRIDE";
        default: return "SENSOR";
    }
}

static const char *UdpReverseV4StateName(UdpReverseV4State state)
{
    switch (state) {
        case UDP_REVERSE_V4_STATE_BACK: return "REV4_BACK";
        case UDP_REVERSE_V4_STATE_PULSE_LEFT: return "REV4_PULSE_LEFT";
        case UDP_REVERSE_V4_STATE_PULSE_RIGHT: return "REV4_PULSE_RIGHT";
        case UDP_REVERSE_V4_STATE_PROBE_BACK: return "REV4_PROBE_BACK";
        default: return "REV4_STOPPED";
    }
}

static const char *UdpReverseV4TriggerName(uint8_t sensor)
{
    if (sensor == 0x02U) return "LEFT";
    if (sensor == 0x01U) return "RIGHT";
    return "NONE";
}

static const char *UdpReverseV4YawName(UdpReverseV4State state)
{
    if (state == UDP_REVERSE_V4_STATE_PULSE_RIGHT) return "RIGHT";
    if (state == UDP_REVERSE_V4_STATE_PULSE_LEFT) return "LEFT";
    return "NONE";
}

static const char *UdpReverseV4ActionName(UdpReverseV4State state)
{
    if (state == UDP_REVERSE_V4_STATE_BACK) return "BACK";
    if (state == UDP_REVERSE_V4_STATE_PROBE_BACK) return "PROBE_BACK";
    if (state == UDP_REVERSE_V4_STATE_PULSE_RIGHT) return "PULSE_RIGHT";
    if (state == UDP_REVERSE_V4_STATE_PULSE_LEFT) return "PULSE_LEFT";
    return "STOP";
}
static const char *UdpReverseV7ActionName(const UdpReverseV4TelemetryState *s)
{
    if (s->state == (UdpReverseV4State)301) return "BACK";
    if (s->state == (UdpReverseV4State)302 || s->state == (UdpReverseV4State)303)
        return (s->leftCommand > s->rightCommand) ? "HEADING_RIGHT" : "HEADING_LEFT";
    if (s->state == (UdpReverseV4State)304 || s->state == (UdpReverseV4State)305)
        return "SWEEP_ENTRY_YAW";
    if (s->state == (UdpReverseV4State)306 || s->state == (UdpReverseV4State)307)
        return "SWEEP_BACK";
    if (s->state == (UdpReverseV4State)309 || s->state == (UdpReverseV4State)310)
        return "RETURN_HEADING";
    if (s->state == (UdpReverseV4State)311) return "VERIFY_BACK";
    return "STOP";
}
static const char *UdpReverseV7YawName(const UdpReverseV4TelemetryState *s)
{
    if (s->state == (UdpReverseV4State)302 || s->state == (UdpReverseV4State)303 ||
        s->state == (UdpReverseV4State)304 || s->state == (UdpReverseV4State)305 ||
        s->state == (UdpReverseV4State)309 || s->state == (UdpReverseV4State)310)
        return (s->leftCommand > s->rightCommand) ? "RIGHT" : "LEFT";
    return "NONE";
}

static const char *UdpReverseV8ActionName(const UdpReverseV4TelemetryState *s)
{
    if ((int)s->state >= 480) {
        switch ((int)s->state - 480) {
            case 0: return "BIASED_WAIT_CLEAR";
            case 1: return "BIASED_STRAIGHT_00";
            case 2: return "BIASED_SHIFT_RIGHT";
            case 3: return "BIASED_PARTIAL_RESTORE_LEFT";
            case 4: return "BIASED_SHIFT_LEFT";
            case 5: return "BIASED_PARTIAL_RESTORE_RIGHT";
            case 6: return "BIASED_SETTLE";
            case 7: return "ESCAPE_11_RIGHT";
            case 8: return "ESCAPE_11_LEFT";
            case 9: return "ESCAPE_11_STRAIGHT";
            default: return "BIASED_STOPPED";
        }
    }
    if ((int)s->state >= 460) {
        switch ((int)s->state - 460) {
            case 0: return "MICRO_WAIT_CLEAR";
            case 1: return "MICRO_STRAIGHT_00";
            case 2: return "MICRO_SHIFT_RIGHT";
            case 3: return "MICRO_RESTORE_LEFT";
            case 4: return "MICRO_SHIFT_LEFT";
            case 5: return "MICRO_RESTORE_RIGHT";
            case 6: return "MICRO_SETTLE";
            default: return "MICRO_STOPPED";
        }
    }
    if ((int)s->state >= 440) {
        switch ((int)s->state - 440) {
            case 0: return "PAIR_WAIT_CLEAR";
            case 1: return "PAIR_INITIAL_STRAIGHT";
            case 2: return "PAIR_SHIFT_RIGHT";
            case 3: return "PAIR_SHIFT_LEFT";
            case 4: return "PAIR_RESTORE_LEFT";
            case 5: return "PAIR_RESTORE_RIGHT";
            case 6: return "PAIR_TRUSTED_STRAIGHT";
            case 7: return "PAIR_ABORT";
            default: return "PAIR_STOPPED";
        }
    }
    if ((int)s->state >= 420) {
        switch ((int)s->state - 420) {
            case 0: return "WAIT_CLEAR";
            case 1: return "REV_SIMPLE_STRAIGHT";
            case 2: return "REV_SIMPLE_NUDGE_RIGHT";
            case 3: return "REV_SIMPLE_EDGE_LEFT_HOLD";
            case 4: return "REV_SIMPLE_NUDGE_LEFT";
            case 5: return "REV_SIMPLE_EDGE_RIGHT_HOLD";
            case 6: return "REV_SIMPLE_TRANSIT_LEFT_TO_RIGHT";
            case 7: return "REV_SIMPLE_TRANSIT_RIGHT_TO_LEFT";
            case 8: return "REV_SIMPLE_RIDE_BACK_NUDGE_LEFT";
            case 9: return "REV_SIMPLE_RIDE_BACK_HOLD_LEFT";
            case 10: return "REV_SIMPLE_RIDE_BACK_NUDGE_RIGHT";
            case 11: return "REV_SIMPLE_RIDE_BACK_HOLD_RIGHT";
            case 12: return "REV_SIMPLE_SEARCH_BACK_LEFT";
            case 13: return "REV_SIMPLE_SEARCH_BACK_RIGHT";
            default: return "STOP";
        }
    }
    if (s->correctionDirection == 1U) return "EDGE_CORRECT_TOWARD";
    if (s->correctionDirection == 2U) return "EDGE_CORRECT_AWAY";
    switch ((int)s->state - 400) {
        case 1: return "BACK";
        case 2: case 3: return "EDGE_ENTRY_YAW";
        case 4: return "EDGE_CANDIDATE";
        case 5: return "EDGE_LOCKED";
        case 6: return "EDGE_GAP";
        default: return "STOP";
    }
}

static const char *UdpReverseV8YawName(const UdpReverseV4TelemetryState *s)
{
    if ((int)s->state >= 440) {
        return (s->leftCommand == s->rightCommand) ? "NONE" :
               ((s->leftCommand > s->rightCommand) ? "RIGHT" : "LEFT");
    }
    if (((int)s->state >= 422 && (int)s->state <= 422) ||
        ((int)s->state >= 424 && (int)s->state <= 424) ||
        ((int)s->state >= 426 && (int)s->state <= 428) ||
        ((int)s->state >= 430 && (int)s->state <= 433)) {
        return (s->leftCommand > s->rightCommand) ? "RIGHT" : "LEFT";
    }
    if (s->correctionDirection != 0U || (int)s->state - 400 == 2 ||
        (int)s->state - 400 == 3) {
        return (s->leftCommand > s->rightCommand) ? "RIGHT" : "LEFT";
    }
    return "NONE";
}

static const char *UdpReverseV8CorrectionName(const UdpReverseV4TelemetryState *s)
{
    if (s->correctionDirection == 1U) return "TOWARD";
    if (s->correctionDirection == 2U) return "AWAY";
    return "NONE";
}

static int UdpTelemetrySendReverseV4(int socketFd, const struct sockaddr_in *target,
                                     const UdpReverseV4TelemetryState *state)
{
    const char *prefix = (state->event >= 400) ? "REVERSEV8" : (state->event >= 300) ? "REVERSEV7" : (state->event >= 200) ? "REVERSEV6" : (state->event >= 100) ? "REVERSEV5" : "REVERSEV4";
    int textLen;
    if (state->event >= 400 && (int)state->state >= 480) {
        textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
            "%s event=%s origin_sensor=%s last_edge_side=%s yaw=%s ms=%u seq=%u sensor=%u%u physical_left=%u physical_right=%u action=%s L=%d R=%d shift_target_ms=60 shift_elapsed_ms=%u shift_actual_ms=%u restore_target_ms=%u settle_target_ms=60 repeat_count=%u escape_count=%u\r\n",
            prefix, UdpReverseV4EventName(state->event), UdpReverseV4TriggerName(state->triggerSensorState), UdpReverseV4TriggerName(state->edgeSide), UdpReverseV8YawName(state), (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence, (unsigned int)((state->sensorState >> 1)&1U), (unsigned int)(state->sensorState&1U), (unsigned int)state->rawLeft, (unsigned int)state->rawRight, UdpReverseV8ActionName(state), state->leftCommand, state->rightCommand, (unsigned int)state->shiftElapsedMs, (unsigned int)state->shiftDurationMs, (unsigned int)state->restoreTargetMs, (unsigned int)state->sameSidePulses, (unsigned int)state->probeElapsedMs);
    } else if (state->event >= 400 && (int)state->state >= 460) {
        textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
            "%s event=%s origin_sensor=%s last_edge_side=%s yaw=%s ms=%u seq=%u sensor=%u%u physical_left=%u physical_right=%u action=%s L=%d R=%d shift_elapsed_ms=%u shift_actual_ms=%u restore_elapsed_ms=%u restore_target_ms=%u pair_count=%u target_ms=%u\r\n",
            prefix, UdpReverseV4EventName(state->event),
            UdpReverseV4TriggerName(state->triggerSensorState),
            UdpReverseV4TriggerName(state->edgeSide), UdpReverseV8YawName(state),
            (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
            (unsigned int)((state->sensorState >> 1) & 1U),
            (unsigned int)(state->sensorState & 1U), (unsigned int)state->rawLeft,
            (unsigned int)state->rawRight, UdpReverseV8ActionName(state),
            state->leftCommand, state->rightCommand,
            (unsigned int)state->shiftElapsedMs, (unsigned int)state->shiftDurationMs,
            (unsigned int)state->restoreElapsedMs, (unsigned int)state->restoreTargetMs,
            (unsigned int)state->sameSidePulses, (unsigned int)state->targetMs);
    } else if (state->event >= 400 && (int)state->state >= 440) {
        textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
            "%s event=%s origin_edge=%s center_confidence=%s yaw=%s ms=%u seq=%u sensor=%u%u physical_left=%u physical_right=%u action=%s L=%d R=%d shift_elapsed_ms=%u shift_duration_ms=%u restore_elapsed_ms=%u restore_target_ms=%u target_ms=%u\r\n",
            prefix, UdpReverseV4EventName(state->event),
            UdpReverseV4TriggerName(state->edgeSide),
            (state->centerConfidence != 0U) ? "TRUSTED" : "INITIAL",
            UdpReverseV8YawName(state), (unsigned int)UdpTelemetryUptimeMs(),
            (unsigned int)state->sequence,
            (unsigned int)((state->sensorState >> 1) & 1U),
            (unsigned int)(state->sensorState & 1U), (unsigned int)state->rawLeft,
            (unsigned int)state->rawRight, UdpReverseV8ActionName(state),
            state->leftCommand, state->rightCommand,
            (unsigned int)state->shiftElapsedMs, (unsigned int)state->shiftDurationMs,
            (unsigned int)state->restoreElapsedMs, (unsigned int)state->restoreTargetMs,
            (unsigned int)state->targetMs);
    } else if (state->event >= 400) {
        textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
            "%s event=%s last_edge_side=%s expected_edge=%s trigger=%s correction=%s yaw=%s ms=%u seq=%u sensor=%u%u action=%s L=%d R=%d phase_elapsed_ms=%u nudge_elapsed_ms=%u transit_elapsed_ms=%u search_elapsed_ms=%u target_ms=%u\r\n",
            prefix, UdpReverseV4EventName(state->event), UdpReverseV4TriggerName(state->edgeSide),
            UdpReverseV4TriggerName(state->expectedSensorState), UdpReverseV4TriggerName(state->triggerSensorState), UdpReverseV8CorrectionName(state), UdpReverseV8YawName(state),
            (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
            (unsigned int)((state->sensorState >> 1) & 1U), (unsigned int)(state->sensorState & 1U),
            UdpReverseV8ActionName(state), state->leftCommand, state->rightCommand,
            (unsigned int)state->pulseElapsedMs, (unsigned int)state->nudgeElapsedMs, (unsigned int)state->transitElapsedMs,
            (unsigned int)state->searchElapsedMs, (unsigned int)state->targetMs);
    } else if (state->event >= 300) {
        textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
            "%s event=%s target_ms=%u trigger=%s yaw=%s ms=%u seq=%u sensor=%u%u action=%s L=%d R=%d\r\n",
            prefix, UdpReverseV4EventName(state->event), (unsigned int)state->targetMs,
            UdpReverseV4TriggerName(state->triggerSensorState),
            (state->leftCommand == state->rightCommand) ? "NONE" : (state->leftCommand > state->rightCommand ? "RIGHT" : "LEFT"),
            (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
            (unsigned int)((state->sensorState >> 1) & 1U), (unsigned int)(state->sensorState & 1U),
            UdpReverseV7ActionName(state), state->leftCommand, state->rightCommand);
    } else {
        textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
            "%s event=%s trigger=%s yaw=%s ms=%u seq=%u sensor=%u%u action=%s L=%d R=%d pulse_ms=100 probe_ms=150\r\n",
            prefix,
            UdpReverseV4EventName(state->event), UdpReverseV4TriggerName(state->triggerSensorState),
            UdpReverseV4YawName(state->state), (unsigned int)UdpTelemetryUptimeMs(),
            (unsigned int)state->sequence, (unsigned int)((state->sensorState >> 1) & 1U),
            (unsigned int)(state->sensorState & 1U), UdpReverseV4ActionName(state->state),
            state->leftCommand, state->rightCommand);
    }
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) return -1;
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static int UdpTelemetrySendReverseV4Debug(int socketFd, const struct sockaddr_in *target,
                                          const UdpReverseV4TelemetryState *state)
{
    const char *prefix = (state->event >= 400) ? "REVERSEV8DEBUG" : (state->event >= 300) ? "REVERSEV7DEBUG" : (state->event >= 200) ? "REVERSEV6DEBUG" : (state->event >= 100) ? "REVERSEV5DEBUG" : "REVERSEV4DEBUG";
    const char *stateName = UdpReverseV4StateName(state->state);
    if (state->event >= 400) {
        if ((int)state->state >= 480) {
            switch ((int)state->state - 480) {
                case 0: stateName = "REV8_BIASED_WAIT_CLEAR"; break;
                case 1: stateName = "REV8_BIASED_STRAIGHT"; break;
                case 2: stateName = "REV8_BIASED_SHIFT_RIGHT"; break;
                case 3: stateName = "REV8_BIASED_RESTORE_LEFT"; break;
                case 4: stateName = "REV8_BIASED_SHIFT_LEFT"; break;
                case 5: stateName = "REV8_BIASED_RESTORE_RIGHT"; break;
                case 6: stateName = "REV8_BIASED_SETTLE"; break;
                case 7: stateName = "REV8_ESCAPE_11_RIGHT"; break;
                case 8: stateName = "REV8_ESCAPE_11_LEFT"; break;
                case 9: stateName = "REV8_ESCAPE_11_STRAIGHT"; break;
                default: stateName = "REV8_BIASED_STOPPED"; break;
            }
        } else if ((int)state->state >= 460) {
            switch ((int)state->state - 460) {
                case 0: stateName = "REV8_MICRO_WAIT_CLEAR"; break;
                case 1: stateName = "REV8_MICRO_STRAIGHT"; break;
                case 2: stateName = "REV8_MICRO_SHIFT_RIGHT"; break;
                case 3: stateName = "REV8_MICRO_RESTORE_LEFT"; break;
                case 4: stateName = "REV8_MICRO_SHIFT_LEFT"; break;
                case 5: stateName = "REV8_MICRO_RESTORE_RIGHT"; break;
                case 6: stateName = "REV8_MICRO_SETTLE"; break;
                default: stateName = "REV8_MICRO_STOPPED"; break;
            }
        } else if ((int)state->state >= 440) {
            switch ((int)state->state - 440) {
                case 0: stateName = "REV8_PAIR_WAIT_CLEAR"; break;
                case 1: stateName = "REV8_PAIR_INITIAL_STRAIGHT"; break;
                case 2: stateName = "REV8_PAIR_SHIFT_FROM_LEFT"; break;
                case 3: stateName = "REV8_PAIR_SHIFT_FROM_RIGHT"; break;
                case 4: stateName = "REV8_PAIR_RESTORE_FROM_LEFT"; break;
                case 5: stateName = "REV8_PAIR_RESTORE_FROM_RIGHT"; break;
                case 6: stateName = "REV8_PAIR_TRUSTED_STRAIGHT"; break;
                case 7: stateName = "REV8_PAIR_ABORT"; break;
                default: stateName = "REV8_PAIR_STOPPED"; break;
            }
        } else if ((int)state->state >= 420) {
            switch ((int)state->state - 420) {
                case 0: stateName = "REV8_WAIT_CLEAR"; break;
                case 1: stateName = "REV8_SIMPLE_STRAIGHT"; break;
                case 2: stateName = "REV8_SIMPLE_NUDGE_RIGHT"; break;
                case 3: stateName = "REV8_SIMPLE_EDGE_LEFT_HOLD"; break;
                case 4: stateName = "REV8_SIMPLE_NUDGE_LEFT"; break;
                case 5: stateName = "REV8_SIMPLE_EDGE_RIGHT_HOLD"; break;
                case 6: stateName = "REV8_SIMPLE_TRANSIT_LEFT_TO_RIGHT"; break;
                case 7: stateName = "REV8_SIMPLE_TRANSIT_RIGHT_TO_LEFT"; break;
                case 8: stateName = "REV8_SIMPLE_RIDE_BACK_NUDGE_LEFT"; break;
                case 9: stateName = "REV8_SIMPLE_RIDE_BACK_HOLD_LEFT"; break;
                case 10: stateName = "REV8_SIMPLE_RIDE_BACK_NUDGE_RIGHT"; break;
                case 11: stateName = "REV8_SIMPLE_RIDE_BACK_HOLD_RIGHT"; break;
                case 12: stateName = "REV8_SIMPLE_SEARCH_BACK_LEFT"; break;
                case 13: stateName = "REV8_SIMPLE_SEARCH_BACK_RIGHT"; break;
                default: stateName = "REV8_SIMPLE_STOPPED"; break;
            }
        } else {
            switch ((int)state->state - 400) {
                case 1: stateName = "REV8_BACK"; break;
                case 2: case 3: stateName = "REV8_EDGE_ENTRY"; break;
                case 4: stateName = "REV8_EDGE_CANDIDATE"; break;
                case 5: stateName = "REV8_EDGE_LOCKED"; break;
                case 6: stateName = "REV8_EDGE_GAP"; break;
                default: stateName = "REV8_STOPPED"; break;
            }
        }
    } else if (state->event >= 300) {
        switch ((int)state->state - 300) {
            case 1: stateName = "REV7_BACK"; break;
            case 2: case 3: stateName = "REV7_HEADING_TO_CENTER"; break;
            case 4: case 5: stateName = "REV7_SWEEP_ENTRY_YAW"; break;
            case 6: case 7: stateName = "REV7_SWEEP_TO_OPPOSITE"; break;
            case 8: stateName = "REV7_OPPOSITE_CONFIRMED"; break;
            case 9: case 10: stateName = "REV7_RETURN_HEADING"; break;
            case 11: stateName = "REV7_VERIFY_CENTER"; break;
            default: stateName = "REV7_STOPPED"; break;
        }
    } else if (state->event >= 200) {
        switch ((int)state->state - 200) {
            case 1: stateName = "REV6_BACK"; break;
            case 2: stateName = "REV6_HEADING_LEFT"; break;
            case 3: stateName = "REV6_HEADING_RIGHT"; break;
            case 4: case 5: stateName = "REV6_LATERAL_YAW_OUT"; break;
            case 6: stateName = "REV6_SHIFT_BACK"; break;
            case 7: case 8: stateName = "REV6_YAW_RESTORE"; break;
            case 9: stateName = "REV6_VERIFY_BACK"; break;
            default: stateName = "REV6_STOPPED"; break;
        }
    }
    int textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
        "%s ms=%u seq=%u state=%s sensor=%u%u physical_left=%u physical_right=%u last_edge_side=%s expected_edge=%s trigger=%s correction=%s yaw=%s action=%s L=%d R=%d phase_elapsed_ms=%u nudge_elapsed_ms=%u transit_elapsed_ms=%u search_elapsed_ms=%u shift_elapsed_ms=%u shift_duration_ms=%u restore_elapsed_ms=%u restore_target_ms=%u center_confidence=%u target_ms=%u probe_elapsed_ms=%u same_side_pulses=%u prev_sensor=%u%u\r\n",
        prefix,
        (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
        stateName, (unsigned int)((state->sensorState >> 1) & 1U),
        (unsigned int)(state->sensorState & 1U), (unsigned int)state->rawLeft,
        (unsigned int)state->rawRight, UdpReverseV4TriggerName(state->edgeSide),
        UdpReverseV4TriggerName(state->expectedSensorState),
        UdpReverseV4TriggerName(state->triggerSensorState), UdpReverseV8CorrectionName(state),
        (state->event >= 400) ? UdpReverseV8YawName(state) :
        ((state->event >= 300) ? UdpReverseV7YawName(state) : UdpReverseV4YawName(state->state)),
        (state->event >= 400) ? UdpReverseV8ActionName(state) :
        ((state->event >= 300) ? UdpReverseV7ActionName(state) : UdpReverseV4ActionName(state->state)),
        state->leftCommand, state->rightCommand, (unsigned int)state->pulseElapsedMs,
        (unsigned int)state->nudgeElapsedMs, (unsigned int)state->transitElapsedMs,
        (unsigned int)state->searchElapsedMs, (unsigned int)state->shiftElapsedMs,
        (unsigned int)state->shiftDurationMs, (unsigned int)state->restoreElapsedMs,
        (unsigned int)state->restoreTargetMs, (unsigned int)state->centerConfidence,
        (unsigned int)state->targetMs, (unsigned int)state->probeElapsedMs, (unsigned int)state->sameSidePulses,
        (unsigned int)((state->previousSensorState >> 1) & 1U),
        (unsigned int)(state->previousSensorState & 1U));
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) return -1;
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static const char *UdpTraceDebugEventName(UdpTraceDebugEvent event)
{
    switch (event) {
        case UDP_TRACEDEBUG_EVENT_POSSIBLE_LOST_LINE:
            return "POSSIBLE_LOST_LINE";
        case UDP_TRACEDEBUG_EVENT_HEARTBEAT:
            return "HEARTBEAT";
        default:
            return "UPDATE";
    }
}

static const char *UdpTraceDebugCorrectionName(uint8_t correction)
{
    if (correction == 1U) {
        return "LEFT";
    }
    if (correction == 2U) {
        return "RIGHT";
    }
    return "NONE";
}

static int UdpTelemetrySendTraceDebug(int socketFd, const struct sockaddr_in *target,
                                      const UdpTraceDebugState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "TRACEDEBUG event=%s ms=%u seq=%u raw_left=%u raw_right=%u sensor=%u%u "
                       "prev_sensor=%u%u last_non00_sensor=%u%u last_non11_sensor=%u%u "
                       "action=%s L=%d R=%d last_action=%s action_age_ms=%u sensor_age_ms=%u "
                       "last_correction=%s correction_age_ms=%u since_last_10_ms=%u since_last_01_ms=%u "
                       "consecutive_00=%u consecutive_10=%u consecutive_01=%u consecutive_11=%u "
                       "history00=%u history01=%u history10=%u history11=%u\r\n",
                       UdpTraceDebugEventName(state->event),
                       (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
                       (unsigned int)state->rawLeft, (unsigned int)state->rawRight,
                       (unsigned int)((state->sensorState >> 1) & 0x01U),
                       (unsigned int)(state->sensorState & 0x01U),
                       (unsigned int)((state->previousSensorState >> 1) & 0x01U),
                       (unsigned int)(state->previousSensorState & 0x01U),
                       (unsigned int)((state->lastNon00SensorState >> 1) & 0x01U),
                       (unsigned int)(state->lastNon00SensorState & 0x01U),
                       (unsigned int)((state->lastNon11SensorState >> 1) & 0x01U),
                       (unsigned int)(state->lastNon11SensorState & 0x01U),
                       UdpTelemetryActionName(state->action), state->leftCommand,
                       state->rightCommand, UdpTelemetryActionName(state->lastAction),
                       (unsigned int)state->actionAgeMs, (unsigned int)state->sensorAgeMs,
                       UdpTraceDebugCorrectionName(state->lastCorrection),
                       (unsigned int)state->correctionAgeMs,
                       (unsigned int)state->last10AgeMs, (unsigned int)state->last01AgeMs,
                       (unsigned int)state->consecutive00, (unsigned int)state->consecutive10,
                       (unsigned int)state->consecutive01, (unsigned int)state->consecutive11,
                       (unsigned int)state->history00, (unsigned int)state->history01,
                       (unsigned int)state->history10, (unsigned int)state->history11);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static const char *UdpReplayEventName(UdpReplayEvent event)
{
    switch (event) {
        case UDP_REPLAY_EVENT_WAIT_START: return "WAIT_START";
        case UDP_REPLAY_EVENT_RECORD_START: return "RECORD_START";
        case UDP_REPLAY_EVENT_COMMAND_CHANGE: return "COMMAND_CHANGE";
        case UDP_REPLAY_EVENT_END_MARKER: return "END_MARKER";
        case UDP_REPLAY_EVENT_REVERSE_START: return "REVERSE_START";
        case UDP_REPLAY_EVENT_REVERSE_COMMAND_CHANGE: return "REVERSE_COMMAND_CHANGE";
        case UDP_REPLAY_EVENT_SENSOR_MISMATCH: return "SENSOR_MISMATCH";
        case UDP_REPLAY_EVENT_DONE: return "DONE";
        case UDP_REPLAY_EVENT_BUFFER_FULL: return "BUFFER_FULL";
        case UDP_REPLAY_EVENT_MARKER_CANDIDATE: return "MARKER_CANDIDATE";
        case UDP_REPLAY_EVENT_MARKER_REJECTED: return "MARKER_REJECTED";
        case UDP_REPLAY_EVENT_MARKER_CONFIRM_START: return "MARKER_CONFIRM_START";
        case UDP_REPLAY_EVENT_MARKER_CONFIRM_HEARTBEAT: return "MARKER_CONFIRM_HEARTBEAT";
        case UDP_REPLAY_EVENT_MARKER_CONFIRM_REJECT: return "MARKER_CONFIRM_REJECT";
        case UDP_REPLAY_EVENT_MARKER_CONFIRMED: return "MARKER_CONFIRMED";
        default: return "HEARTBEAT";
    }
}

static const char *UdpReplayMarkerReasonName(uint8_t reason)
{
    switch ((UdpReplayMarkerReason)reason) {
        case UDP_REPLAY_MARKER_REASON_DIRECT_00_TO_11: return "DIRECT_00_TO_11";
        case UDP_REPLAY_MARKER_REASON_SHORT_10_PREAMBLE: return "SHORT_10_PREAMBLE";
        case UDP_REPLAY_MARKER_REASON_SHORT_01_PREAMBLE: return "SHORT_01_PREAMBLE";
        case UDP_REPLAY_MARKER_REASON_SIGNED_PREAMBLE_TOO_LONG: return "SIGNED_PREAMBLE_TOO_LONG";
        default: return "NO_00_CONTEXT";
    }
}

static int UdpTelemetrySendReplay(int socketFd, const struct sockaddr_in *target,
                                  const UdpReplayTelemetryState *state)
{
    int textLen;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "REPLAY event=%s ms=%u seq=%u phase=%u frame_index=%u frame_count=%u forward_duration_ms=%u elapsed_ms=%u confirm_elapsed_ms=%u confirm_target_ms=%u recorded_sensor=%u%u current_sensor=%u%u prev_stable_sensor=%u%u state_before_prev=%u%u signed_preamble_sensor=%u%u signed_preamble_ms=%u marker_limit_ms=%u decision=%s reason=%s before11_1=%u before11_2=%u before11_3=%u before11_4=%u before11_5=%u since_last00_ms=%u since_last10_ms=%u since_last01_ms=%u stable00_duration_ms=%u orig_L=%d orig_R=%d replay_L=%d replay_R=%d low_magnitude=%u\r\n",
                       UdpReplayEventName(state->event),
                       (unsigned int)UdpTelemetryUptimeMs(), (unsigned int)state->sequence,
                       (unsigned int)state->phase, (unsigned int)state->frameIndex,
                       (unsigned int)state->frameCount, (unsigned int)state->forwardDurationMs,
                       (unsigned int)state->elapsedMs,
                       (unsigned int)state->confirmElapsedMs,
                       (unsigned int)state->confirmTargetMs,
                       (unsigned int)((state->recordedSensor >> 1) & 1U),
                       (unsigned int)(state->recordedSensor & 1U),
                       (unsigned int)((state->currentSensor >> 1) & 1U),
                       (unsigned int)(state->currentSensor & 1U),
                       (unsigned int)((state->previousStableSensor >> 1) & 1U),
                       (unsigned int)(state->previousStableSensor & 1U),
                       (unsigned int)((state->stateBeforePrevious >> 1) & 1U),
                       (unsigned int)(state->stateBeforePrevious & 1U),
                       (unsigned int)((state->signedPreambleSensor >> 1) & 1U),
                       (unsigned int)(state->signedPreambleSensor & 1U),
                       (unsigned int)state->signedPreambleMs,
                       (unsigned int)state->markerLimitMs,
                       (state->markerDecision != 0U) ? "ACCEPT" : "REJECT",
                       UdpReplayMarkerReasonName(state->markerReason),
                       (unsigned int)state->stableBefore11[0],
                       (unsigned int)state->stableBefore11[1],
                       (unsigned int)state->stableBefore11[2],
                       (unsigned int)state->stableBefore11[3],
                       (unsigned int)state->stableBefore11[4],
                       (unsigned int)state->sinceLast00Ms,
                       (unsigned int)state->sinceLast10Ms,
                       (unsigned int)state->sinceLast01Ms,
                       (unsigned int)state->stable00DurationMs,
                       state->origLeftCommand, state->origRightCommand,
                       state->replayLeftCommand, state->replayRightCommand,
                       (unsigned int)state->lowMagnitude);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static int UdpTelemetrySendReplayHistory(int socketFd, const struct sockaddr_in *target,
                                         const UdpReplayHistoryFrame *frame,
                                         uint32_t historyIndex, uint32_t newestMs)
{
    int textLen;
    uint32_t ageMs = (newestMs >= frame->timestampMs) ? newestMs - frame->timestampMs : 0U;

    textLen = snprintf(g_udpTelemetryText, sizeof(g_udpTelemetryText),
                       "REPLAY event=PRE_MARKER_HISTORY history_index=%u age_ms=%u control_ms=%u raw_left=%u raw_right=%u stable_sensor=%u%u prev_stable_sensor=%u%u L=%d R=%d trace_action=%u\r\n",
                       (unsigned int)historyIndex, (unsigned int)ageMs,
                       (unsigned int)frame->timestampMs, (unsigned int)frame->rawLeft,
                       (unsigned int)frame->rawRight,
                       (unsigned int)((frame->stableSensor >> 1) & 1U),
                       (unsigned int)(frame->stableSensor & 1U),
                       (unsigned int)((frame->previousStableSensor >> 1) & 1U),
                       (unsigned int)(frame->previousStableSensor & 1U),
                       frame->leftCommand, frame->rightCommand,
                       (unsigned int)frame->traceAction);
    if (textLen < 0 || (unsigned int)textLen >= sizeof(g_udpTelemetryText)) {
        return -1;
    }
    return (sendto(socketFd, g_udpTelemetryText, (size_t)textLen, 0,
                   (const struct sockaddr *)target, sizeof(*target)) == textLen) ? 0 : -1;
}

static void UdpTelemetryTask(void *argument)
{
    struct sockaddr_in target;
    CarTelemetryState state;
    UdpSensorStats sensorStats;
    UdpLineCalibrationState lineCalibration;
    UdpAvoidTelemetryState avoid;
    UdpRaceTelemetryState race;
    UdpRaceDebugState raceDebug;
    UdpReverseTelemetryState reverse;
    UdpReverseV2TelemetryState reverseV2;
    UdpReverseV3TelemetryState reverseV3;
    UdpReverseV4TelemetryState reverseV4;
    UdpReplayTelemetryState replay;
    UdpTraceDebugState traceDebug;
    UdpEncoderTelemetryState encoder;
    uint32_t lastSentSequence = 0U;
    uint32_t lastSensorStatsSequence = 0U;
    uint32_t lastLineCalibrationSequence = 0U;
    uint32_t lastAvoidSequence = 0U;
    uint32_t lastAvoidHeartbeatTick = 0U;
    uint32_t lastRaceSequence = 0U;
    uint32_t lastRaceDebugSequence = 0U;
    uint32_t lastReverseSequence = 0U;
    uint32_t lastReverseV2Sequence = 0U;
    uint32_t lastReverseV3Sequence = 0U;
    uint32_t lastReverseV4Sequence = 0U;
    uint32_t lastReplaySequence = 0U;
    uint32_t lastReplayHistoryDumpGeneration = 0U;
    uint32_t replayHistoryDumpIndex = 0U;
    uint32_t replayHistoryDumpRemaining = 0U;
    uint32_t replayHistoryDumpNewestMs = 0U;
    uint32_t lastTraceDebugSequence = 0U;
    uint32_t lastEncoderValidCount = 0U;
    uint32_t lastEncoderHeartbeatTick = 0U;
    uint32_t lastHeartbeatTick = 0U;
    uint32_t lastErrorLogTick = 0U;
    int socketFd = -1;

    (void)argument;
    printf("UDP telemetry waiting network\r\n");

    for (;;) {
        uint32_t now = osKernelGetTickCount();

        if (TaskWifiIsNetworkReady() == 0) {
            if (socketFd >= 0) {
                (void)lwip_close(socketFd);
                socketFd = -1;
            }
            osDelay(AppMsToTicks(UDP_TELEMETRY_PERIOD_MS));
            continue;
        }

        if (socketFd < 0) {
            printf("UDP telemetry network ready\r\n");
            socketFd = UdpTelemetryCreateSocket(&target);
            if (socketFd < 0) {
                if ((uint32_t)(now - lastErrorLogTick) >=
                    AppMsToTicks(UDP_TELEMETRY_SOCKET_RETRY_MS)) {
                    printf("UDP telemetry socket create failed\r\n");
                    lastErrorLogTick = now;
                }
                osDelay(AppMsToTicks(UDP_TELEMETRY_SOCKET_RETRY_MS));
                continue;
            }
            printf("UDP telemetry target %s:%u\r\n", UDP_TELEMETRY_HOST,
                   (unsigned int)UDP_TELEMETRY_PORT);
            printf("UDP telemetry socket ready\r\n");
            lastHeartbeatTick = 0U;
        }

        UdpTelemetryReadSnapshot(&state);
        if (state.sequence != lastSentSequence) {
            if (UdpTelemetrySend(socketFd, &target, "LINE", &state) == 0) {
                lastSentSequence = state.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        UdpTelemetryReadSensorStats(&sensorStats);
        if (sensorStats.sequence != lastSensorStatsSequence) {
            if (UdpTelemetrySendSensorStats(socketFd, &target, &sensorStats) == 0) {
                lastSensorStatsSequence = sensorStats.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        UdpTelemetryReadLineCalibration(&lineCalibration);
        if (lineCalibration.sequence != lastLineCalibrationSequence) {
            if (UdpTelemetrySendLineCalibration(socketFd, &target, &lineCalibration) == 0) {
                lastLineCalibrationSequence = lineCalibration.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        UdpTelemetryReadAvoid(&avoid);
        if (avoid.sequence != lastAvoidSequence) {
            if (UdpTelemetrySendAvoid(socketFd, &target, &avoid) == 0) {
                lastAvoidSequence = avoid.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }
        if ((uint32_t)(now - lastAvoidHeartbeatTick) >=
            AppMsToTicks(UDP_TELEMETRY_HEARTBEAT_MS)) {
            (void)UdpTelemetrySendAvoid(socketFd, &target, &avoid);
            lastAvoidHeartbeatTick = now;
        }

        UdpTelemetryReadRace(&race);
        if (race.sequence != lastRaceSequence) {
            if (UdpTelemetrySendRace(socketFd, &target, &race) == 0) {
                lastRaceSequence = race.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        UdpTelemetryReadRaceDebug(&raceDebug);
        if (raceDebug.sequence != lastRaceDebugSequence) {
            if (UdpTelemetrySendRaceDebug(socketFd, &target, &raceDebug) == 0) {
                lastRaceDebugSequence = raceDebug.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        UdpTelemetryReadReverse(&reverse);
        if (reverse.sequence != lastReverseSequence) {
            if (UdpTelemetrySendReverse(socketFd, &target, &reverse) == 0 &&
                UdpTelemetrySendReverseDebug(socketFd, &target, &reverse) == 0) {
                lastReverseSequence = reverse.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        UdpTelemetryReadReverseV2(&reverseV2);
        if (reverseV2.sequence != lastReverseV2Sequence) {
            if (UdpTelemetrySendReverseV2(socketFd, &target, &reverseV2) == 0 &&
                UdpTelemetrySendReverseV2Debug(socketFd, &target, &reverseV2) == 0) {
                lastReverseV2Sequence = reverseV2.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        UdpTelemetryReadReverseV3(&reverseV3);
        if (reverseV3.sequence != lastReverseV3Sequence) {
            if (UdpTelemetrySendReverseV3(socketFd, &target, &reverseV3) == 0 &&
                UdpTelemetrySendReverseV3Debug(socketFd, &target, &reverseV3) == 0) {
                lastReverseV3Sequence = reverseV3.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        UdpTelemetryReadReverseV4(&reverseV4);
        if (reverseV4.sequence != lastReverseV4Sequence) {
            if (UdpTelemetrySendReverseV4(socketFd, &target, &reverseV4) == 0 &&
                UdpTelemetrySendReverseV4Debug(socketFd, &target, &reverseV4) == 0) {
                lastReverseV4Sequence = reverseV4.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        {
            uint32_t replayEventsSent = 0U;
            while (replayEventsSent < 4U && UdpTelemetryPopReplayEvent(&replay) != 0) {
                if (UdpTelemetrySendReplay(socketFd, &target, &replay) != 0) {
                    break;
                }
                lastReplaySequence = replay.sequence;
                replayEventsSent++;
            }
        }
        UdpTelemetryReadReplay(&replay);
        if (replay.sequence != lastReplaySequence) {
            if (UdpTelemetrySendReplay(socketFd, &target, &replay) == 0) {
                lastReplaySequence = replay.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        if (g_replayHistoryDumpGeneration != lastReplayHistoryDumpGeneration) {
            uint32_t historyCount = g_replayHistoryCount;
            uint32_t writeIndex = g_replayHistoryWriteIndex;
            replayHistoryDumpIndex = (writeIndex + UDP_REPLAY_HISTORY_CAPACITY - historyCount) %
                                     UDP_REPLAY_HISTORY_CAPACITY;
            replayHistoryDumpRemaining = historyCount;
            replayHistoryDumpNewestMs = UdpTelemetryUptimeMs();
            lastReplayHistoryDumpGeneration = g_replayHistoryDumpGeneration;
        }
        /* History transport is owned by this low-priority telemetry task.
         * Batching eight short UDP lines cannot delay the 30 ms car task. */
        {
            uint32_t sent = 0U;
            while (replayHistoryDumpRemaining != 0U && sent < 8U) {
                UdpReplayHistoryFrame frame = g_replayHistory[replayHistoryDumpIndex];
                if (UdpTelemetrySendReplayHistory(socketFd, &target, &frame,
                                                   replayHistoryDumpIndex,
                                                   replayHistoryDumpNewestMs) != 0) {
                    break;
                }
                replayHistoryDumpIndex = (replayHistoryDumpIndex + 1U) %
                                         UDP_REPLAY_HISTORY_CAPACITY;
                replayHistoryDumpRemaining--;
                sent++;
            }
        }

        UdpTelemetryReadTraceDebug(&traceDebug);
        if (traceDebug.sequence != lastTraceDebugSequence) {
            if (UdpTelemetrySendTraceDebug(socketFd, &target, &traceDebug) == 0) {
                lastTraceDebugSequence = traceDebug.sequence;
            } else if ((uint32_t)(now - lastErrorLogTick) >=
                       AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
        }

        { static uint32_t calSeen; if (calSeen != g_calGeneration) { (void)sendto(socketFd, g_calText, strlen(g_calText), 0, (const struct sockaddr *)&target, sizeof(target)); calSeen = g_calGeneration; } }

        /* Experiment trajectory dumps are queued after motion has stopped.
         * Drain a small bounded batch here, never from TaskCarControl. */
        {
            uint32_t experimentEventsSent = 0U;
            while (experimentEventsSent < 4U &&
                   UdpTelemetryPopExperimentText(g_udpTelemetryText,
                                                  sizeof(g_udpTelemetryText)) != 0) {
                if (sendto(socketFd, g_udpTelemetryText, strlen(g_udpTelemetryText), 0,
                           (const struct sockaddr *)&target, sizeof(target)) < 0) {
                    break;
                }
                experimentEventsSent++;
            }
        }

        UdpTelemetryReadEncoder(&encoder);
        if (encoder.validCount != lastEncoderValidCount) {
            if (UdpTelemetrySendEncoder(socketFd, &target, "ENC", &encoder) == 0) {
                lastEncoderValidCount = encoder.validCount;
            }
        }
        if (encoder.validCount != 0U &&
            (uint32_t)(now - lastEncoderHeartbeatTick) >= AppMsToTicks(1000U)) {
            (void)UdpTelemetrySendEncoder(socketFd, &target, "ENC_HEARTBEAT", &encoder);
            lastEncoderHeartbeatTick = now;
        }

        if ((uint32_t)(now - lastHeartbeatTick) >=
            AppMsToTicks(UDP_TELEMETRY_HEARTBEAT_MS)) {
            if (UdpTelemetrySend(socketFd, &target, "STAT", &state) != 0 &&
                (uint32_t)(now - lastErrorLogTick) >=
                AppMsToTicks(UDP_TELEMETRY_SEND_ERROR_LOG_MS)) {
                printf("UDP telemetry send failed\r\n");
                lastErrorLogTick = now;
            }
            lastHeartbeatTick = now;
        }

        osDelay(AppMsToTicks(UDP_TELEMETRY_PERIOD_MS));
    }
}

void UdpTelemetryInit(void)
{
    osThreadAttr_t attr;

    if (g_udpTelemetryTaskStarted != 0) {
        return;
    }

    attr.name = "udp_telemetry";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = UDP_TELEMETRY_THREAD_STACK_SIZE;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(UdpTelemetryTask, NULL, &attr) == NULL) {
        printf("UDP telemetry thread create failed\r\n");
        return;
    }

    g_udpTelemetryTaskStarted = 1;
    printf("UDP telemetry stack size = %u\r\n",
           (unsigned int)UDP_TELEMETRY_THREAD_STACK_SIZE);

    if (g_udpCommandTaskStarted == 0) {
        attr.name = "udp_command";
        attr.stack_size = UDP_COMMAND_THREAD_STACK_SIZE;
        if (osThreadNew(UdpCommandTask, NULL, &attr) == NULL) {
            printf("UDP command thread create failed\r\n");
            return;
        }
        g_udpCommandTaskStarted = 1;
        printf("UDP command stack size = %u\r\n",
               (unsigned int)UDP_COMMAND_THREAD_STACK_SIZE);
    }
}
