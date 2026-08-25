#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "wifiiot_uart.h"
#include "task10_bluetooth.h"

#define BLUETOOTH_UART           WIFI_IOT_UART_IDX_1
#define BLUETOOTH_BAUD_RATE      115200
#define BLUETOOTH_READ_BUFFER_LEN 32

static void BluetoothTask(void *argument)
{
    WifiIotUartAttribute uartAttr = {
        .baudRate = BLUETOOTH_BAUD_RATE,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    unsigned char data[BLUETOOTH_READ_BUFFER_LEN];
    int readLen;
    int index;
    (void)argument;

    if (UartInit(BLUETOOTH_UART, &uartAttr, NULL) != 0) {
        printf("bluetooth UART1 init failed\r\n");
        return;
    }

    printf("bluetooth UART1 ready\r\n");
    while (1) {
        readLen = UartRead(BLUETOOTH_UART, data, sizeof(data));
        if (readLen > 0) {
            printf("bluetooth RX:");
            for (index = 0; index < readLen; index++) {
                printf(" %02X", data[index]);
            }
            printf("\r\n");
        } else {
            osDelay(10);
        }
    }
}

void Task10BluetoothInit(void)
{
    osThreadAttr_t attr;

    attr.name = "task10_bluetooth";
    attr.attr_bits = 0;
    attr.cb_mem = NULL;
    attr.cb_size = 0;
    attr.stack_mem = NULL;
    attr.stack_size = 2048;
    attr.priority = osPriorityBelowNormal;

    if (osThreadNew(BluetoothTask, NULL, &attr) == NULL) {
        printf("bluetooth thread create failed\r\n");
    }
}
