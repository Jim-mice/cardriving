#include "bsp_pn532.h"

#include "drv_usart2_nfc.h"

#define PN532_PREAMBLE              0x00U
#define PN532_STARTCODE_1           0x00U
#define PN532_STARTCODE_2           0xFFU
#define PN532_POSTAMBLE             0x00U
#define PN532_HOST_TO_PN532         0xD4U
#define PN532_PN532_TO_HOST         0xD5U
#define PN532_ACK_FRAME_LENGTH      6U
#define PN532_MAX_FRAME_DATA_LENGTH 32U
#define PN532_BYTE_TIMEOUT          10000000UL
#define PN532_RESPONSE_TIMEOUT      100000000UL

static const uint8_t g_pn532AckFrame[PN532_ACK_FRAME_LENGTH] = {
    0x00U, 0x00U, 0xFFU, 0x00U, 0xFFU, 0x00U
};
static PN532_Error g_pn532LastError = PN532_ERROR_NONE;

static PN532_Error PN532_ReadAck(void)
{
    uint8_t window[PN532_ACK_FRAME_LENGTH];
    uint8_t data;
    uint8_t index;

    for (index = 0; index < PN532_ACK_FRAME_LENGTH; index++) {
        window[index] = 0xFFU;
    }

    for (index = 0; index < 32U; index++) {
        uint8_t shift;

        if (USART2_NFC_ReadByte(&data, PN532_RESPONSE_TIMEOUT) == 0U) {
            return (USART2_NFC_GetRawRxCount() == 0U) ? PN532_ERROR_RX_EMPTY : PN532_ERROR_ACK_INVALID;
        }
        for (shift = 0; shift < (PN532_ACK_FRAME_LENGTH - 1U); shift++) {
            window[shift] = window[shift + 1U];
        }
        window[PN532_ACK_FRAME_LENGTH - 1U] = data;

        for (shift = 0; shift < PN532_ACK_FRAME_LENGTH; shift++) {
            if (window[shift] != g_pn532AckFrame[shift]) {
                break;
            }
        }
        if (shift == PN532_ACK_FRAME_LENGTH) {
            return PN532_ERROR_NONE;
        }
    }

    return PN532_ERROR_ACK_INVALID;
}

static PN532_Error PN532_ReadFrame(uint8_t *data, uint8_t *dataLength)
{
    uint8_t byte;
    uint8_t frameLength;
    uint8_t lengthChecksum;
    uint8_t dataChecksum;
    uint8_t checksum = 0U;
    uint8_t index;
    uint8_t preambleState = 0U;

    while (1) {
        if (USART2_NFC_ReadByte(&byte, PN532_RESPONSE_TIMEOUT) == 0U) {
            return (preambleState == 0U) ? PN532_ERROR_RESPONSE_TIMEOUT : PN532_ERROR_RESPONSE_INVALID;
        }

        if (preambleState == 0U) {
            preambleState = (byte == PN532_PREAMBLE) ? 1U : 0U;
        } else if (preambleState == 1U) {
            preambleState = (byte == PN532_STARTCODE_1) ? 2U : 0U;
        } else {
            if (byte == PN532_STARTCODE_2) {
                break;
            }
            preambleState = (byte == PN532_PREAMBLE) ? 1U : 0U;
        }
    }

    if (USART2_NFC_ReadByte(&frameLength, PN532_BYTE_TIMEOUT) == 0U ||
        USART2_NFC_ReadByte(&lengthChecksum, PN532_BYTE_TIMEOUT) == 0U) {
        return PN532_ERROR_RESPONSE_INVALID;
    }
    if ((uint8_t)(frameLength + lengthChecksum) != 0U ||
        frameLength == 0U || frameLength == 0xFFU ||
        frameLength > PN532_MAX_FRAME_DATA_LENGTH) {
        return PN532_ERROR_RESPONSE_INVALID;
    }

    for (index = 0; index < frameLength; index++) {
        if (USART2_NFC_ReadByte(&data[index], PN532_BYTE_TIMEOUT) == 0U) {
            return PN532_ERROR_RESPONSE_INVALID;
        }
        checksum = (uint8_t)(checksum + data[index]);
    }

    if (USART2_NFC_ReadByte(&dataChecksum, PN532_BYTE_TIMEOUT) == 0U ||
        USART2_NFC_ReadByte(&byte, PN532_BYTE_TIMEOUT) == 0U) {
        return PN532_ERROR_RESPONSE_INVALID;
    }
    if ((uint8_t)(checksum + dataChecksum) != 0U || byte != PN532_POSTAMBLE) {
        return PN532_ERROR_RESPONSE_INVALID;
    }

    *dataLength = frameLength;
    return PN532_ERROR_NONE;
}

static uint8_t PN532_Transceive(uint8_t command, const uint8_t *parameters,
    uint8_t parameterLength, uint8_t *response, uint8_t *responseLength)
{
    uint8_t frame[PN532_MAX_FRAME_DATA_LENGTH + 7U];
    uint8_t frameLength = (uint8_t)(parameterLength + 2U);
    uint8_t checksum = 0U;
    uint8_t index;

    if (frameLength > PN532_MAX_FRAME_DATA_LENGTH) {
        return 0U;
    }

    frame[0] = PN532_PREAMBLE;
    frame[1] = PN532_STARTCODE_1;
    frame[2] = PN532_STARTCODE_2;
    frame[3] = frameLength;
    frame[4] = (uint8_t)(0U - frameLength);
    frame[5] = PN532_HOST_TO_PN532;
    frame[6] = command;
    checksum = (uint8_t)(PN532_HOST_TO_PN532 + command);
    for (index = 0; index < parameterLength; index++) {
        frame[7U + index] = parameters[index];
        checksum = (uint8_t)(checksum + parameters[index]);
    }
    frame[7U + parameterLength] = (uint8_t)(0U - checksum);
    frame[8U + parameterLength] = PN532_POSTAMBLE;

    PN532_Error error;

    USART2_NFC_ClearRx();
    USART2_NFC_SendBuffer(frame, (uint16_t)(frameLength + 7U));
    error = PN532_ReadAck();
    if (error != PN532_ERROR_NONE) {
        g_pn532LastError = error;
        return 0U;
    }
    error = PN532_ReadFrame(response, responseLength);
    if (error != PN532_ERROR_NONE) {
        g_pn532LastError = error;
        return 0U;
    }
    if (*responseLength < 2U || response[0] != PN532_PN532_TO_HOST ||
        response[1] != (uint8_t)(command + 1U)) {
        g_pn532LastError = PN532_ERROR_RESPONSE_INVALID;
        return 0U;
    }

    g_pn532LastError = PN532_ERROR_NONE;
    return 1U;
}

void PN532_Init(void)
{
    static const uint8_t wakeFrame[] = {0x55U, 0x55U, 0x00U, 0x00U, 0x00U};

    /* Elechouse/Seeed HSU wake-up sequence. */
    USART2_NFC_ClearRx();
    USART2_NFC_SendBuffer(wakeFrame, sizeof(wakeFrame));
}

PN532_Error PN532_GetLastError(void)
{
    return g_pn532LastError;
}

uint8_t PN532_GetFirmwareVersion(uint8_t version[4])
{
    uint8_t response[PN532_MAX_FRAME_DATA_LENGTH];
    uint8_t responseLength;

    if (PN532_Transceive(0x02U, (const uint8_t *)0, 0U, response, &responseLength) == 0U) {
        return 0U;
    }
    if (responseLength != 6U) {
        g_pn532LastError = PN532_ERROR_RESPONSE_INVALID;
        return 0U;
    }

    version[0] = response[2];
    version[1] = response[3];
    version[2] = response[4];
    version[3] = response[5];
    return 1U;
}

uint8_t PN532_SAMConfiguration(void)
{
    static const uint8_t parameters[] = {0x01U, 0x14U, 0x01U};
    uint8_t response[PN532_MAX_FRAME_DATA_LENGTH];
    uint8_t responseLength;

    if (PN532_Transceive(0x14U, parameters, sizeof(parameters), response,
        &responseLength) == 0U) {
        return 0U;
    }

    return (responseLength == 2U) ? 1U : 0U;
}

uint8_t PN532_InListPassiveTarget(uint8_t *uid, uint8_t *uidLength)
{
    static const uint8_t parameters[] = {0x01U, 0x00U};
    uint8_t response[PN532_MAX_FRAME_DATA_LENGTH];
    uint8_t responseLength;
    uint8_t index;
    uint8_t length;

    *uidLength = 0U;
    if (PN532_Transceive(0x4AU, parameters, sizeof(parameters), response,
        &responseLength) == 0U) {
        return 0U;
    }
    if (responseLength < 3U || response[2] == 0U) {
        return 0U;
    }

    /* D5 4B NbTg Tg SENS_RES[2] SEL_RES NFCIDLength NFCID... */
    if (responseLength < 8U) {
        return 0U;
    }
    length = response[7];
    if (length == 0U || length > PN532_UID_MAX_LENGTH ||
        responseLength < (uint8_t)(8U + length)) {
        return 0U;
    }

    for (index = 0; index < length; index++) {
        uid[index] = response[8U + index];
    }
    *uidLength = length;
    return 1U;
}
