#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "app_time.h"
#include "wifiiot_uart.h"
#include "uart_task9.h"
#include "app_time.h"
#include "udp_telemetry.h"

#define UART_TASK9_PORT        WIFI_IOT_UART_IDX_2
#define UART_FRAME_HEAD        0xFC
#define UART_FRAME_TAIL        0xFD
#define UART_FRAME_LEN         6
#define UART_TASK9_QUEUE_DEPTH 8
#define UART_TASK9_LOG_PERIOD_MS 5000U
#define ENCODER_FRAME_HEAD 0xEC
#define ENCODER_FRAME_TAIL 0xED
#define ENCODER_FRAME_LEN 8

typedef struct {
    uint8_t data[UART_FRAME_LEN];
} UartTask9Message;

static osMessageQueueId_t g_uartTask9Queue;
static uint32_t g_encoderValidCount;
static uint32_t g_encoderBadChecksumCount;
static uint32_t g_encoderBadFrameCount;

/* The RX task only receives, frames, and queues data; it has no business logic. */
static void UartTask9ParseByte(uint8_t byte)
{
    static UartTask9Message frame;
    static uint32_t frameIndex;
    static uint8_t encoderFrame[ENCODER_FRAME_LEN];
    static uint8_t encoderIndex;

    if (encoderIndex != 0U || byte == ENCODER_FRAME_HEAD) {
        if (encoderIndex == 0U) encoderFrame[encoderIndex++] = byte;
        else encoderFrame[encoderIndex++] = byte;
        if (encoderIndex == ENCODER_FRAME_LEN) {
            uint8_t checksum = (uint8_t)(encoderFrame[1] ^ encoderFrame[2] ^ encoderFrame[3] ^ encoderFrame[4] ^ encoderFrame[5]);
            if (encoderFrame[0] == ENCODER_FRAME_HEAD && encoderFrame[7] == ENCODER_FRAME_TAIL) {
                if (checksum == encoderFrame[6]) {
                    UdpEncoderTelemetryState state;
                    uint16_t ul = (uint16_t)encoderFrame[2] | ((uint16_t)encoderFrame[3] << 8);
                    uint16_t ur = (uint16_t)encoderFrame[4] | ((uint16_t)encoderFrame[5] << 8);
                    state.leftDelta = (int16_t)ul; state.rightDelta = (int16_t)ur;
                    state.sequence = encoderFrame[1]; state.validCount = ++g_encoderValidCount;
                    state.badChecksumCount = g_encoderBadChecksumCount;
                    state.badFrameCount = g_encoderBadFrameCount;
                    state.lastRxMs = AppTicksToMs(osKernelGetTickCount());
                    { UdpEncoderTelemetryState p; UdpTelemetryReadEncoder(&p); state.totalLeft=p.totalLeft+state.leftDelta; state.totalRight=p.totalRight+state.rightDelta; }
                    UdpTelemetryUpdateEncoder(&state);
                } else g_encoderBadChecksumCount++;
            } else g_encoderBadFrameCount++;
            encoderIndex = 0U;
        }
        return;
    }

    if (frameIndex == 0) {
        if (byte == UART_FRAME_HEAD) {
            frame.data[frameIndex++] = byte;
        }
        return;
    }

    if (frameIndex < UART_FRAME_LEN - 1) {
        frame.data[frameIndex++] = byte;
        return;
    }

    if (byte == UART_FRAME_TAIL) {
        frame.data[frameIndex] = byte;
        /* Do not wait here: a full queue must not stall UART reception. */
        (void)osMessageQueuePut(g_uartTask9Queue, &frame, 0, 0);
        frameIndex = 0;
    } else if (byte == UART_FRAME_HEAD) {
        /* The invalid tail can also be the head of the next frame. */
        frame.data[0] = byte;
        frameIndex = 1;
    } else {
        frameIndex = 0;
    }
}

static void UartRxThread(void *argument)
{
    uint8_t byte;
    (void)argument;

    while (1) {
        int readLen = UartRead(UART_TASK9_PORT, &byte, 1);
        if (readLen == 1) {
            UartTask9ParseByte(byte);
        } else {
            /* Avoid a busy loop if the UART API reports a temporary error. */
            osDelay(AppMsToTicks(1U));
        }
    }
}

static void UartMsgThread(void *argument)
{
    UartTask9Message message;
    uint32_t lastLogTick = 0;
    uint32_t logPeriodTicks;
    (void)argument;

    logPeriodTicks = AppMsToTicks(UART_TASK9_LOG_PERIOD_MS);

    while (1) {
        if (osMessageQueueGet(g_uartTask9Queue, &message, NULL,
            osWaitForever) == osOK) {
            uint32_t now = osKernelGetTickCount();
            if (lastLogTick != 0U && (now - lastLogTick) < logPeriodTicks) {
                continue;
            }
            lastLogTick = now;
            printf("UART RX:\r\n");
            printf("%02X %02X %02X %02X %02X %02X\r\n",
                message.data[0], message.data[1], message.data[2],
                message.data[3], message.data[4], message.data[5]);
        }
    }
}

void UartTask9Init(void)
{
    static const osThreadAttr_t rxThreadAttr = {
        .name = "uart2_rx",
        .stack_size = 2048,
        .priority = osPriorityNormal,
    };
    static const osThreadAttr_t msgThreadAttr = {
        .name = "uart2_msg",
        .stack_size = 2048,
        .priority = osPriorityBelowNormal,
    };

    g_uartTask9Queue = osMessageQueueNew(UART_TASK9_QUEUE_DEPTH,
        sizeof(UartTask9Message), NULL);
    if (g_uartTask9Queue == NULL) {
        printf("UART RX queue create failed\r\n");
        return;
    }

    if (osThreadNew(UartRxThread, NULL, &rxThreadAttr) == NULL) {
        printf("UART RX thread create failed\r\n");
        return;
    }

    if (osThreadNew(UartMsgThread, NULL, &msgThreadAttr) == NULL) {
        printf("UART message thread create failed\r\n");
    }
}
