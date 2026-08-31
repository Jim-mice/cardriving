#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "app_time.h"
#include "wifiiot_uart.h"
#include "task10_bluetooth.h"
#include "task_car_control.h"

#define BLUETOOTH_UART              WIFI_IOT_UART_IDX_1
#define BLE_PROBE_BAUD_9600         9600U
#define BLE_PROBE_BAUD_115200       115200U
#define BLE_READ_BUFFER_LEN         64U
#define BLE_AT_RESPONSE_WAIT_MS     200U
#define BLE_IDLE_READ_PERIOD_MS     20U

static volatile int g_bleUartReady;
static int g_bleTaskStarted;

static void BleHandleControlLine(const char *line)
{
    if (strcmp(line, "BPATH START") == 0) {
        CarControlSubmitBpathCommandFromSource(BPATH_CONTROL_COMMAND_START,
                                               BPATH_COMMAND_SOURCE_BLE);
    } else if (strcmp(line, "BPATH RETURN") == 0) {
        CarControlSubmitBpathCommandFromSource(BPATH_CONTROL_COMMAND_RETURN,
                                               BPATH_COMMAND_SOURCE_BLE);
    } else if (strcmp(line, "BPATH RESET") == 0) {
        CarControlSubmitBpathCommandFromSource(BPATH_CONTROL_COMMAND_RESET,
                                               BPATH_COMMAND_SOURCE_BLE);
    }
}

static void BleHandleControlBytes(const unsigned char *data, int len)
{
    static char line[24];
    static unsigned int lineLength;
    int index;

    for (index = 0; index < len; index++) {
        unsigned char byte = data[index];

        if (byte == '\r') {
            continue;
        }
        if (byte == '\n') {
            line[lineLength] = '\0';
            BleHandleControlLine(line);
            lineLength = 0U;
            continue;
        }
        if (byte >= 0x20U && byte <= 0x7eU && lineLength < sizeof(line) - 1U) {
            line[lineLength++] = (char)byte;
        } else {
            lineLength = 0U;
        }
    }
}

static int BleUartConfigure(uint32_t baudRate)
{
    WifiIotUartAttribute uartAttr = {
        .baudRate = baudRate,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };

    return UartInit(BLUETOOTH_UART, &uartAttr, NULL);
}

static void BlePrintResponse(const char *query, const unsigned char *data, int len)
{
    int index;

    printf("JDY16 %s: ", query);
    for (index = 0; index < len; index++) {
        unsigned char value = data[index];
        if (value >= 0x20U && value <= 0x7EU) {
            printf("%c", (char)value);
        } else {
            printf("\\x%02X", value);
        }
    }
    printf("\r\n");
}

static int BleAtQuery(const char *query)
{
    unsigned char response[BLE_READ_BUFFER_LEN];
    uint32_t start;
    int readLen;
    unsigned int length;

    length = (unsigned int)strlen(query);
    if (UartWrite(BLUETOOTH_UART, (unsigned char *)query, length) < 0) {
        return 0;
    }

    start = osKernelGetTickCount();
    while ((uint32_t)(osKernelGetTickCount() - start) <
           AppMsToTicks(BLE_AT_RESPONSE_WAIT_MS)) {
        readLen = UartRead(BLUETOOTH_UART, response, sizeof(response));
        if (readLen > 0) {
            BlePrintResponse(query, response, readLen);
            return 1;
        }
        osDelay(AppMsToTicks(10U));
    }

    return 0;
}

static int BleDetectUart(void)
{
    if (BleUartConfigure(BLE_PROBE_BAUD_9600) == 0 &&
        BleAtQuery("AT+VER\r\n") != 0) {
        printf("BLE UART detected baud=%u\r\n", (unsigned int)BLE_PROBE_BAUD_9600);
        return 1;
    }

    if (BleUartConfigure(BLE_PROBE_BAUD_115200) == 0 &&
        BleAtQuery("AT+VER\r\n") != 0) {
        printf("BLE UART detected baud=%u\r\n", (unsigned int)BLE_PROBE_BAUD_115200);
        return 1;
    }

    printf("BLE UART detect failed (9600/115200)\r\n");
    return 0;
}

int BleUartIsReady(void)
{
    return g_bleUartReady;
}

int BleUartSend(const unsigned char *data, unsigned int len)
{
    if (g_bleUartReady == 0 || data == NULL || len == 0U) {
        return -1;
    }

    return UartWrite(BLUETOOTH_UART, (unsigned char *)data, len);
}

int BleUartSendString(const char *text)
{
    if (text == NULL) {
        return -1;
    }

    return BleUartSend((const unsigned char *)text, (unsigned int)strlen(text));
}

static void BluetoothTask(void *argument)
{
    unsigned char data[BLE_READ_BUFFER_LEN];

    (void)argument;

    if (BleDetectUart() == 0) {
        return;
    }

    /* Query only: never alter JDY-16 persistent parameters. */
    (void)BleAtQuery("AT+NAME\r\n");
    (void)BleAtQuery("AT+BAUD\r\n");
    (void)BleAtQuery("AT+STARTEN\r\n");
    (void)BleAtQuery("AT+MASTEREN\r\n");
    (void)BleAtQuery("AT+SVRUUID\r\n");
    (void)BleAtQuery("AT+CHRUUID\r\n");

    g_bleUartReady = 1;
    printf("BLE diagnostic UART ready\r\n");

    for (;;) {
        int readLen = UartRead(BLUETOOTH_UART, data, sizeof(data));
        if (readLen > 0) {
            /* Only BPATH START, RETURN, and RESET publish owner-thread work. */
            BleHandleControlBytes(data, readLen);
        }
        osDelay(AppMsToTicks(BLE_IDLE_READ_PERIOD_MS));
    }
}

void Task10BluetoothInit(void)
{
    osThreadAttr_t attr;

    if (g_bleTaskStarted != 0) {
        return;
    }

    attr.name = "ble_diag";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(BluetoothTask, NULL, &attr) == NULL) {
        printf("BLE diagnostic thread create failed\r\n");
        return;
    }

    g_bleTaskStarted = 1;
}
