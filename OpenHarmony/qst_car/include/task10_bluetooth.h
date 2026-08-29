#ifndef TASK10_BLUETOOTH_H
#define TASK10_BLUETOOTH_H

void Task10BluetoothInit(void);

/* JDY-16 UART diagnostics only; these APIs never control the vehicle. */
int BleUartIsReady(void);
int BleUartSend(const unsigned char *data, unsigned int len);
int BleUartSendString(const char *text);

#endif
