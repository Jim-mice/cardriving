#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_time.h"
#include "cmsis_os2.h"
#include "teach_encoder.h"
#include "udp_telemetry.h"

#define TEACH_SAMPLE_PERIOD_MS 20U
#define TEACH_TASK_STACK_SIZE 2048U

static volatile TeachEncoderState g_teach;
static uint32_t g_last_sample_ms;

void TeachEncoderStart(void)
{
    UdpEncoderTelemetryState encoder;

    UdpTelemetryReadEncoder(&encoder);
    g_teach.start_left = encoder.totalLeft;
    g_teach.start_right = encoder.totalRight;
    g_teach.sample_count = 0U;
    g_teach.start_time = AppTicksToMs(osKernelGetTickCount());
    g_last_sample_ms = g_teach.start_time - TEACH_SAMPLE_PERIOD_MS;
    g_teach.recording = true;
    printf("EVENT | TEACH_START left=%ld right=%ld\r\n",
           (long)g_teach.start_left, (long)g_teach.start_right);
}

void TeachEncoderStop(void)
{
    g_teach.recording = false;
    printf("EVENT | TEACH_STOP samples=%u\r\n",
           (unsigned int)g_teach.sample_count);
}

bool TeachEncoderIsRecording(void)
{
    return g_teach.recording;
}

void TeachEncoderStep(uint32_t now_ms)
{
    UdpEncoderTelemetryState encoder;
    UdpTeachPoint point;

    if (!g_teach.recording || (uint32_t)(now_ms - g_last_sample_ms) <
        TEACH_SAMPLE_PERIOD_MS) {
        return;
    }

    g_last_sample_ms = now_ms;
    UdpTelemetryReadEncoder(&encoder);
    point.timeMs = now_ms - g_teach.start_time;
    point.left = encoder.totalLeft - g_teach.start_left;
    point.right = encoder.totalRight - g_teach.start_right;
    if (UdpTelemetryQueueTeachPoint(&point) == 0) {
        printf("EVENT | TEACH_DROP samples=%u\r\n",
               (unsigned int)g_teach.sample_count);
        return;
    }
    g_teach.sample_count++;
}

static void TeachEncoderTask(void *argument)
{
    (void)argument;
    for (;;) {
        TeachEncoderStep(AppTicksToMs(osKernelGetTickCount()));
        osDelay(AppMsToTicks(TEACH_SAMPLE_PERIOD_MS));
    }
}

void TeachEncoderInit(void)
{
    static const osThreadAttr_t attr = {
        .name = "teach_encoder",
        .stack_size = TEACH_TASK_STACK_SIZE,
        .priority = osPriorityBelowNormal,
    };

    g_teach.recording = false;
    if (osThreadNew(TeachEncoderTask, NULL, &attr) == NULL) {
        printf("teach encoder thread create failed\r\n");
    }
}
