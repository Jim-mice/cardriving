#ifndef __BSP_PN532_H
#define __BSP_PN532_H

#include "stm32f10x.h"

#define PN532_UID_MAX_LENGTH 10U

typedef enum {
    PN532_ERROR_NONE = 0,
    PN532_ERROR_RX_EMPTY,
    PN532_ERROR_ACK_INVALID,
    PN532_ERROR_RESPONSE_TIMEOUT,
    PN532_ERROR_RESPONSE_INVALID
} PN532_Error;

void PN532_Init(void);
uint8_t PN532_GetFirmwareVersion(uint8_t version[4]);
PN532_Error PN532_GetLastError(void);
uint8_t PN532_SAMConfiguration(void);
uint8_t PN532_InListPassiveTarget(uint8_t *uid, uint8_t *uidLength);

#endif
