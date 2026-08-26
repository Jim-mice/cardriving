#include "stm32f10x.h"

#include "bsp_light.h"
#include "bsp_gpio.h"
#include "bsp_encoder.h"
#include "bsp_motor_l9110.h"
#include "drv_usart1_hi3861.h"
#include "drv_usart2_nfc.h"

#define PN532_WAKEUP_FRAME_LENGTH          5U
#define PN532_FIRMWARE_FRAME_LENGTH        9U
#define PN532_HOST_ACK_FRAME_LENGTH         6U
#define PN532_RAW_RX_CAPACITY              64U

static const uint8_t authorized_uid[] = {
    0xF3U, 0x97U, 0x47U, 0x06U
};

volatile uint32_t g_ms_ticks;

void SysTick_Handler(void)
{
    g_ms_ticks++;
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = g_ms_ticks;

    while ((uint32_t)(g_ms_ticks - start) < ms) {
    }
}

static void USART1_PrintFrame(uint8_t *frame)
{
static const char hex[]="0123456789ABCDEF";
uint8_t index;

USART1_SendString("STM32 RX:");

for(index=0;index<USART1_HI3861_FRAME_LEN;index++)
{
USART1_SendByte(hex[(frame[index]>>4)&0x0F]);
USART1_SendByte(hex[frame[index]&0x0F]);
USART1_SendByte(' ');
}

USART1_SendString("\r\n");
}

static void USART1_PrintHexByte(uint8_t data)
{
    static const char hex[] = "0123456789ABCDEF";

    USART1_SendByte(hex[(data >> 4) & 0x0FU]);
    USART1_SendByte(hex[data & 0x0FU]);
}

/* Read SR followed by DR clears a pending RXNE/ORE condition on STM32F1. */
static void USART2_ClearReceiveStatus(void)
{
    volatile uint32_t status;
    volatile uint32_t data;

    status = USART2->SR;
    data = USART2->DR;
    (void)status;
    (void)data;
}

/*
 * Collect every received byte during the complete timeout interval.  This is
 * deliberately polling-only: USART2 RXNE interrupt handling is not used by
 * this PN532 transport check.
 */
static uint16_t USART2_CollectPollingRx(uint8_t *raw, uint16_t capacity,
    uint32_t timeoutMs)
{
    uint16_t count = 0U;
    uint32_t start = g_ms_ticks;

    while ((uint32_t)(g_ms_ticks - start) < timeoutMs) {
        if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET) {
            uint8_t data = (uint8_t)USART_ReceiveData(USART2);

            if (count < capacity) {
                raw[count] = data;
                count++;
            }
        }
    }

    return count;
}

typedef enum {
    PN532_TRANSACTION_OK = 0,
    PN532_TRANSACTION_NO_ACK,
    PN532_TRANSACTION_ACK_INVALID,
    PN532_TRANSACTION_RESPONSE_MISSING,
    PN532_TRANSACTION_PREAMBLE_INVALID,
    PN532_TRANSACTION_LEN_INVALID,
    PN532_TRANSACTION_LCS_INVALID,
    PN532_TRANSACTION_TFI_INVALID,
    PN532_TRANSACTION_COMMAND_INVALID,
    PN532_TRANSACTION_DCS_INVALID,
    PN532_TRANSACTION_POSTAMBLE_INVALID
} PN532_TransactionResult;

typedef struct {
    uint8_t raw[PN532_RAW_RX_CAPACITY];
    uint16_t rawCount;
} PN532_RawTrace;

static uint16_t PN532_FindAckOffset(const uint8_t *raw, uint16_t count)
{
    static const uint8_t ack[] = {0x00U, 0x00U, 0xFFU, 0x00U, 0xFFU, 0x00U};
    uint16_t offset;
    uint8_t index;

    for (offset = 0U; (uint16_t)(offset + 6U) <= count; offset++) {
        for (index = 0U; index < 6U; index++) {
            if (raw[offset + index] != ack[index]) {
                break;
            }
        }
        if (index == 6U) {
            return offset;
        }
    }
    return 0xFFFFU;
}

static PN532_TransactionResult PN532_ParseResponse(const uint8_t *raw,
    uint16_t count, uint8_t expectedCommand, uint16_t ackOffset,
    const uint8_t **responseData, uint8_t *responseLength)
{
    uint16_t offset;
    uint8_t length;
    uint8_t checksum = 0U;
    uint8_t index;

    if ((uint16_t)(ackOffset + 6U) >= count) {
        return PN532_TRANSACTION_RESPONSE_MISSING;
    }
    for (offset = (uint16_t)(ackOffset + 6U);
        (uint16_t)(offset + 3U) <= count; offset++) {
        if (raw[offset] == 0x00U && raw[offset + 1U] == 0x00U &&
            raw[offset + 2U] == 0xFFU) {
            break;
        }
    }
    if ((uint16_t)(offset + 3U) > count) {
        return PN532_TRANSACTION_PREAMBLE_INVALID;
    }
    if ((uint16_t)(offset + 5U) > count) {
        return PN532_TRANSACTION_RESPONSE_MISSING;
    }

    length = raw[offset + 3U];
    if (length == 0U || length == 0xFFU) {
        return PN532_TRANSACTION_LEN_INVALID;
    }
    if ((uint8_t)(length + raw[offset + 4U]) != 0U) {
        return PN532_TRANSACTION_LCS_INVALID;
    }
    if ((uint16_t)(offset + 7U + length) > count) {
        return PN532_TRANSACTION_RESPONSE_MISSING;
    }
    if (raw[offset + 5U] != 0xD5U) {
        return PN532_TRANSACTION_TFI_INVALID;
    }
    if (length < 2U || raw[offset + 6U] != expectedCommand) {
        return PN532_TRANSACTION_COMMAND_INVALID;
    }
    for (index = 0U; index < length; index++) {
        checksum = (uint8_t)(checksum + raw[offset + 5U + index]);
    }
    if ((uint8_t)(checksum + raw[offset + 5U + length]) != 0U) {
        return PN532_TRANSACTION_DCS_INVALID;
    }
    if (raw[offset + 6U + length] != 0x00U) {
        return PN532_TRANSACTION_POSTAMBLE_INVALID;
    }

    *responseData = &raw[offset + 5U];
    *responseLength = length;
    return PN532_TRANSACTION_OK;
}

static PN532_TransactionResult PN532_Transceive(const uint8_t *command,
    uint16_t commandLength, uint8_t *response, uint8_t responseMax,
    uint8_t *responseLength, PN532_RawTrace *trace)
{
    static const uint8_t hostAck[PN532_HOST_ACK_FRAME_LENGTH] = {
        0x00U, 0x00U, 0xFFU, 0x00U, 0xFFU, 0x00U
    };
    uint8_t localRaw[PN532_RAW_RX_CAPACITY];
    uint8_t *raw = (trace == 0) ? localRaw : trace->raw;
    const uint8_t *frameData;
    uint8_t frameLength;
    uint16_t rawCount;
    uint16_t ackOffset;
    uint8_t index;
    PN532_TransactionResult result;

    *responseLength = 0U;
    if (trace != 0) {
        trace->rawCount = 0U;
    }
    if (commandLength < 7U) {
        return PN532_TRANSACTION_COMMAND_INVALID;
    }

    USART2_ClearReceiveStatus();
    USART2_NFC_SendBuffer(command, commandLength);
    rawCount = USART2_CollectPollingRx(raw, PN532_RAW_RX_CAPACITY, 500U);
    if (trace != 0) {
        trace->rawCount = rawCount;
    }
    if (rawCount == 0U) {
        return PN532_TRANSACTION_NO_ACK;
    }
    ackOffset = PN532_FindAckOffset(raw, rawCount);
    if (ackOffset == 0xFFFFU) {
        return PN532_TRANSACTION_ACK_INVALID;
    }

    result = PN532_ParseResponse(raw, rawCount, (uint8_t)(command[6] + 1U),
        ackOffset, &frameData, &frameLength);
    if (result != PN532_TRANSACTION_OK) {
        return result;
    }
    if (frameLength > responseMax) {
        return PN532_TRANSACTION_LEN_INVALID;
    }
    for (index = 0U; index < frameLength; index++) {
        response[index] = frameData[index];
    }
    *responseLength = frameLength;

    /* Host ACK follows only after the full response has passed all checks. */
    USART2_NFC_SendBuffer(hostAck, PN532_HOST_ACK_FRAME_LENGTH);
    return PN532_TRANSACTION_OK;
}

static uint8_t PN532_ParseUid(const uint8_t *response, uint8_t responseLength,
    const uint8_t **uid, uint8_t *uidLength)
{
    uint8_t length;

    if (responseLength < 8U || response[2] == 0U) {
        return 0U;
    }
    length = response[7];
    if (length == 0U || length > 10U ||
        responseLength < (uint8_t)(8U + length)) {
        return 0U;
    }
    *uid = &response[8];
    *uidLength = length;
    return 1U;
}

static uint8_t PN532_IsSameUid(const uint8_t *first, uint8_t firstLength,
    const uint8_t *second, uint8_t secondLength)
{
    uint8_t index;

    if (firstLength != secondLength) {
        return 0U;
    }
    for (index = 0U; index < firstLength; index++) {
        if (first[index] != second[index]) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t PN532_IsAuthorizedUid(const uint8_t *uid, uint8_t uidLength)
{
    uint8_t index;

    if (uidLength != sizeof(authorized_uid)) {
        return 0U;
    }
    for (index = 0U; index < sizeof(authorized_uid); index++) {
        if (uid[index] != authorized_uid[index]) {
            return 0U;
        }
    }
    return 1U;
}

static void PN532_PrintUid(const uint8_t *uid, uint8_t uidLength)
{
    uint8_t index;

    USART1_SendString("NFC card found\r\nUID: ");
    for (index = 0U; index < uidLength; index++) {
        USART1_PrintHexByte(uid[index]);
        if (index + 1U < uidLength) {
            USART1_SendByte(' ');
        }
    }
    USART1_SendString("\r\n");
    if (PN532_IsAuthorizedUid(uid, uidLength) != 0U) {
        USART1_SendString("CARD OK\r\n");
    } else {
        USART1_SendString("CARD DENIED\r\n");
    }
}

int main(void)
{

    uint8_t frame[USART1_HI3861_FRAME_LEN];
    uint8_t ledState=0;
    static const uint8_t pn532Wakeup[PN532_WAKEUP_FRAME_LENGTH] = {
        0x55U, 0x55U, 0x00U, 0x00U, 0x00U
    };
    static const uint8_t pn532GetFirmwareVersion[PN532_FIRMWARE_FRAME_LENGTH] = {
        0x00U, 0x00U, 0xFFU, 0x02U, 0xFEU, 0xD4U, 0x02U, 0x2AU, 0x00U
    };
    static const uint8_t sam_wake_frame[26] = {
        0x55U, 0x55U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0xFFU, 0x05U, 0xFBU, 0xD4U,
        0x14U, 0x01U, 0x14U, 0x01U, 0x02U, 0x00U
    };
    static const uint8_t pn532RfConfiguration[] = {
        0x00U, 0x00U, 0xFFU, 0x06U, 0xFAU, 0xD4U, 0x32U,
        0x05U, 0xFFU, 0x01U, 0x01U, 0xF4U, 0x00U
    };
    static const uint8_t pn532InListPassiveTarget[] = {
        0x00U, 0x00U, 0xFFU, 0x04U, 0xFCU, 0xD4U, 0x4AU,
        0x01U, 0x00U, 0xE1U, 0x00U
    };
    uint8_t pn532Response[32];
    uint8_t pn532ResponseLength;
    PN532_RawTrace samTrace;
    const uint8_t *samResponse;
    uint8_t samResponseLength;
    uint16_t samAckOffset;
    PN532_TransactionResult samResult;
    uint8_t nfcReady = 0U;
    uint8_t lastUid[10];
    uint8_t lastUidLength = 0U;
    const uint8_t *uid;
    uint8_t uidLength;
    uint8_t index;

    BSP_GPIO_Init();
    Light_Init();

    USART1_Hi3861_Init();
    motor_init();
    motor_stop();
    encoder_init();

    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000U) != 0U) {
        while (1) {
        }
    }

    USART2_NFC_Init();
    USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);

    delay_ms(100U);

    /* HSU wakeup is issued exactly once before all subsequent commands. */
    USART2_ClearReceiveStatus();
    USART2_NFC_SendBuffer(pn532Wakeup, PN532_WAKEUP_FRAME_LENGTH);
    delay_ms(5U);

    if (PN532_Transceive(pn532GetFirmwareVersion,
        PN532_FIRMWARE_FRAME_LENGTH, pn532Response, sizeof(pn532Response),
        &pn532ResponseLength, 0) == PN532_TRANSACTION_OK &&
        pn532ResponseLength >= 6U) {
        USART1_SendString("PN532 firmware verified\r\n");
        delay_ms(5U);

        /* Elechouse HSU tester pattern: wake preamble and SAM in one write. */
        USART2_NFC_SendBuffer(sam_wake_frame, sizeof(sam_wake_frame));
        samTrace.rawCount = USART2_CollectPollingRx(samTrace.raw,
            PN532_RAW_RX_CAPACITY, 2000U);

        samAckOffset = PN532_FindAckOffset(samTrace.raw, samTrace.rawCount);
        if (samAckOffset == 0xFFFFU) {
            samResult = (samTrace.rawCount == 0U) ? PN532_TRANSACTION_NO_ACK :
                PN532_TRANSACTION_ACK_INVALID;
        } else {
            samResult = PN532_ParseResponse(samTrace.raw, samTrace.rawCount,
                0x15U, samAckOffset, &samResponse, &samResponseLength);
        }
        if (samResult == PN532_TRANSACTION_OK && samResponseLength == 2U) {
            static const uint8_t hostAck[PN532_HOST_ACK_FRAME_LENGTH] = {
                0x00U, 0x00U, 0xFFU, 0x00U, 0xFFU, 0x00U
            };

            USART2_NFC_SendBuffer(hostAck, PN532_HOST_ACK_FRAME_LENGTH);
            USART1_SendString("PN532 SAM configured\r\n");
            delay_ms(5U);

            if (PN532_Transceive(pn532RfConfiguration,
                sizeof(pn532RfConfiguration), pn532Response,
                sizeof(pn532Response), &pn532ResponseLength,
                0) != PN532_TRANSACTION_OK) {
                USART1_SendString("PN532 RF configuration failed\r\n");
            }
            nfcReady = 1U;
        }
    } else {
        USART1_SendString("PN532 firmware failed\r\n");
    }

    while(1)
    {
        Light_Run();

        /*
            ??????
        */
        if(USART1_GetReceivedFrame(frame))
        {
            USART1_PrintFrame(frame);

            /* PC13 is active-low; toggle it for each valid received frame. */
            if(ledState==0)
            {
                LED1_ON();
                ledState=1;
            }
            else
            {
                LED1_OFF();
                ledState=0;
            }

        }

        if (nfcReady != 0U) {
            if (PN532_Transceive(pn532InListPassiveTarget,
                sizeof(pn532InListPassiveTarget), pn532Response,
                sizeof(pn532Response), &pn532ResponseLength,
                0) == PN532_TRANSACTION_OK) {
                if (pn532ResponseLength >= 3U && pn532Response[2] == 0U) {
                    lastUidLength = 0U;
                } else if (PN532_ParseUid(pn532Response, pn532ResponseLength,
                    &uid, &uidLength) != 0U) {
                    if (PN532_IsSameUid(lastUid, lastUidLength, uid,
                        uidLength) == 0U) {
                        PN532_PrintUid(uid, uidLength);
                        for (index = 0U; index < uidLength; index++) {
                            lastUid[index] = uid[index];
                        }
                        lastUidLength = uidLength;
                    }
                } else {
                    lastUidLength = 0U;
                }
            } else {
                lastUidLength = 0U;
            }
            delay_ms(300U);
        }
    }

}
