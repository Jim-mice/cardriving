#include "stm32f10x.h"

#include "bsp_light.h"
#include "bsp_gpio.h"
#include "bsp_encoder.h"
#include "bsp_motor_l9110.h"
#include "drv_usart1_hi3861.h"
#include "drv_usart2_nfc.h"

/* =========================================================
 * PN532
 * ========================================================= */

#define PN532_WAKEUP_FRAME_LENGTH        5U
#define PN532_FIRMWARE_FRAME_LENGTH      9U
#define PN532_HOST_ACK_FRAME_LENGTH      6U
#define PN532_RAW_RX_CAPACITY            64U


/* =========================================================
 * ?????????
 *
 * 1U:???? car_actions[] ????
 * 0U:???? NFC ????
 * ========================================================= */

#define CAR_ACTION_TEST_MODE             0U

/*
 * 1U: keep PN532 initialization, but do not run the blocking foreground
 *     card polling loop while validating Hi3861 real-time motor control.
 * 0U: restore normal foreground NFC polling.
 */
#define CAR_REMOTE_CONTROL_TEST_MODE     1U

/* 1U: suspended AIR baseline; 2U: 800 ms ground baseline; 0U: disabled. */
#define MOTOR_BASELINE_TEST_MODE          0U
#define MOTOR_BASELINE_SAMPLE_PERIOD_MS   100U

/* Local-only left/right motor mapping verification. */
#define MOTOR_SIDE_MAPPING_TEST_MODE      0U
#define ENCODER_TELEMETRY_TEST_MODE       1U
#define ENCODER_TELEMETRY_PERIOD_MS       30U

/* Fixed left/right compensation is deliberately disabled. */
#define LEFT_FORWARD_GAIN_NUM             800
#define LEFT_FORWARD_GAIN_DEN             800


#if (MOTOR_BASELINE_TEST_MODE == 1U)
#define MOTOR_BASELINE_SAMPLE_COUNT       10U
#define MOTOR_BASELINE_START_LABEL        "AIR"
#define MOTOR_BASELINE_WHEEL_LABEL        "AIR"
#define MOTOR_BASELINE_LEFT_PWM           800
#define MOTOR_BASELINE_RIGHT_PWM          800
#elif (MOTOR_BASELINE_TEST_MODE == 2U)
#define MOTOR_BASELINE_SAMPLE_COUNT       5U
#define MOTOR_BASELINE_START_LABEL        "GND COMP3"
#define MOTOR_BASELINE_WHEEL_LABEL        "COMP3"
#define MOTOR_BASELINE_LEFT_PWM           865
#define MOTOR_BASELINE_RIGHT_PWM          800
#endif

#if (MOTOR_BASELINE_TEST_MODE == 0U) && \
    (MOTOR_SIDE_MAPPING_TEST_MODE == 0U)
static void ApplyHi3861MotorFrame(const uint8_t *frame);
#endif
#if (MOTOR_SIDE_MAPPING_TEST_MODE == 0U) || \
    (MOTOR_BASELINE_TEST_MODE != 0U) || CAR_ACTION_TEST_MODE
static void USART1_PrintSignedInt(int value);
#endif
#if (MOTOR_SIDE_MAPPING_TEST_MODE == 1U)
static void MotorSideMappingTest(void);
#endif
#if (MOTOR_BASELINE_TEST_MODE != 0U)
static void MotorBaselineStart(void);
static void WheelStraightDiagnosticPoll(void);
#endif

/* ???????????? */
#define CAR_ACTION_LOG_INTERVAL_MS       500U


/*
 * ????:
 *
 * left_speed  :????,???? -1000 ~ 1000
 * right_speed :????,???? -1000 ~ 1000
 * duration_ms :??????????
 *
 * ??:??
 * ??:??
 * 0:????
 */
#if CAR_ACTION_TEST_MODE

typedef struct
{
    int left_speed;
    int right_speed;
    uint32_t duration_ms;
} CarAction;


/* =========================================================
 * ????? ????????? ?????
 *
 * ??:
 * {????, ????, ??ms}
 *
 * ??????????,?????????
 * ========================================================= */

static const CarAction car_actions[] =
{
    /*
     * ??:???? 8 ?
     *
     * ??:
     * ???,???
     */
    {800, 800, 500U},
										{0, 0, 1000U},
    {-500, 800, 1200U},
										{0, 0, 1000U},
		{800, 800, 1000U},
										{0, 0, 1000U},
									
		{-800, -800, 1000U},
										{0, 0, 1000U},
    {	500, -800, 1200U},	
										{0, 0, 1000U},
    {-800, -800, 500U},		
										{0, 0, 1000U},
    /*
     * ?????????,??:
     *
     * {650, 650, 2000U},     // ??2?
     * {500, 700, 3000U},     // ??3?
     * {650, 650, 1500U},     // ??1.5?
     * {700, 500, 3000U},     // ??3?
     *
     * {-500, -500, 2000U},   // ??2?
     *
     * {-500, 500, 1500U},    // ?????????
     * {500, -500, 1500U},    // ??????????
     *
     * {0, 0, 1000U},         // ?1?
     */
    {800, 800, 500U},
{0, 0, 1000U},
    {800, -500, 1200U},
{0, 0, 1000U},
		{800, 800, 1000U},
{0, 0, 1000U},
		{-800, -800, 1000U},
{0, 0, 1000U},
    {-800, 500, 1200U},	
{0, 0, 1000U},
    {-800, -800, 500U},
{0, 0, 1000U},
};

#define CAR_ACTION_COUNT \
    ((uint32_t)(sizeof(car_actions) / sizeof(car_actions[0])))

#endif


/* =========================================================
 * NFC ???
 * ========================================================= */

#if (CAR_REMOTE_CONTROL_TEST_MODE == 0U)
static const uint8_t authorized_uid[] =
{
    0xF3U, 0x97U, 0x47U, 0x06U
};
#endif


/* =========================================================
 * SysTick
 * ========================================================= */

volatile uint32_t g_ms_ticks;


void SysTick_Handler(void)
{
    g_ms_ticks++;
}


static void delay_ms(uint32_t ms)
{
    uint32_t start = g_ms_ticks;

    while ((uint32_t)(g_ms_ticks - start) < ms)
    {
    }
}

#if (MOTOR_SIDE_MAPPING_TEST_MODE == 1U)
static void MotorSideMappingTest(void)
{
    motor_stop();
    delay_ms(1000U);

    USART1_SendString("MOTOR SIDE TEST START\r\n");

    USART1_SendString("TEST LEFT COMMAND: L=800 R=0\r\n");
    motor_left_set(800);
    motor_right_set(0);
    delay_ms(1000U);

    motor_stop();
    delay_ms(1500U);

    USART1_SendString("TEST RIGHT COMMAND: L=0 R=800\r\n");
    motor_left_set(0);
    motor_right_set(800);
    delay_ms(1000U);

    motor_stop();
    USART1_SendString("MOTOR SIDE TEST STOP\r\n");
}
#endif

#if (MOTOR_BASELINE_TEST_MODE != 0U)

typedef enum
{
    MOTOR_BASELINE_IDLE = 0,
    MOTOR_BASELINE_RUNNING,
    MOTOR_BASELINE_COMPLETE
} MotorBaselineState;

static MotorBaselineState g_motorBaselineState;
static uint8_t g_motorBaselineSampleCount;
static uint32_t g_motorBaselineLastSampleTick;

static void MotorBaselineStart(void)
{
#if (MOTOR_BASELINE_TEST_MODE == 1U) || (MOTOR_BASELINE_TEST_MODE == 2U)
    USART1_SendString("BASELINE ");
    USART1_SendString(MOTOR_BASELINE_START_LABEL);
    USART1_SendString(" START\r\n");

    motor_stop();
    delay_ms(500U);

    /* Establish the previous-count baseline without clearing TIM2/TIM3 CNT. */
    (void)encoder_left_get_delta();
    (void)encoder_right_get_delta();

    g_motorBaselineSampleCount = 0U;
    g_motorBaselineLastSampleTick = g_ms_ticks;
    g_motorBaselineState = MOTOR_BASELINE_RUNNING;

    USART1_SendString("BASELINE PWM L=");
    USART1_PrintSignedInt(MOTOR_BASELINE_LEFT_PWM);
    USART1_SendString(" R=");
    USART1_PrintSignedInt(MOTOR_BASELINE_RIGHT_PWM);
    USART1_SendString("\r\n");

    motor_left_set(MOTOR_BASELINE_LEFT_PWM);
    motor_right_set(MOTOR_BASELINE_RIGHT_PWM);
#else
    g_motorBaselineState = MOTOR_BASELINE_COMPLETE;
#endif
}

static void WheelStraightDiagnosticPoll(void)
{
#if (MOTOR_BASELINE_TEST_MODE == 1U) || (MOTOR_BASELINE_TEST_MODE == 2U)
    uint32_t now;
    int16_t leftDelta;
    int16_t rightDelta;
    int diff;
    if (g_motorBaselineState != MOTOR_BASELINE_RUNNING)
    {
        return;
    }

    now = g_ms_ticks;
    if ((uint32_t)(now - g_motorBaselineLastSampleTick) <
        MOTOR_BASELINE_SAMPLE_PERIOD_MS)
    {
        return;
    }

    g_motorBaselineLastSampleTick = now;
    leftDelta = encoder_left_get_delta();
    rightDelta = encoder_right_get_delta();
    diff = (int)leftDelta - (int)rightDelta;

    g_motorBaselineSampleCount++;
    USART1_SendString("WHEEL ");
    USART1_SendString(MOTOR_BASELINE_WHEEL_LABEL);
    USART1_SendByte(' ');
    USART1_PrintSignedInt((int)g_motorBaselineSampleCount);
    USART1_SendString(" L=");
    USART1_PrintSignedInt((int)leftDelta);
    USART1_SendString(" R=");
    USART1_PrintSignedInt((int)rightDelta);
    USART1_SendString(" diff=");
    USART1_PrintSignedInt(diff);
    USART1_SendString("\r\n");

    if (g_motorBaselineSampleCount >= MOTOR_BASELINE_SAMPLE_COUNT)
    {
        motor_stop();
        g_motorBaselineState = MOTOR_BASELINE_COMPLETE;
        USART1_SendString("BASELINE ");
        USART1_SendString(MOTOR_BASELINE_START_LABEL);
        USART1_SendString(" STOP\r\n");
    }
#endif
}

#endif


/* =========================================================
 * USART1 ??
 * ========================================================= */

#if (CAR_REMOTE_CONTROL_TEST_MODE == 0U)
static void USART1_PrintHexByte(uint8_t data)
{
    static const char hex[] = "0123456789ABCDEF";

    USART1_SendByte(hex[(data >> 4) & 0x0FU]);
    USART1_SendByte(hex[data & 0x0FU]);
}
#endif


/*
 * ??????????
 * ???? printf,?????????????
 */
#if (MOTOR_SIDE_MAPPING_TEST_MODE == 0U) || \
    (MOTOR_BASELINE_TEST_MODE != 0U) || CAR_ACTION_TEST_MODE
static void USART1_PrintSignedInt(int value)
{
    char buffer[16];
    uint8_t index = 0U;
    uint8_t i;
    unsigned int number;

    if (value < 0)
    {
        USART1_SendByte('-');
        number = (unsigned int)(-value);
    }
    else
    {
        number = (unsigned int)value;
    }

    if (number == 0U)
    {
        USART1_SendByte('0');
        return;
    }

    while (number > 0U && index < sizeof(buffer))
    {
        buffer[index++] = (char)('0' + (number % 10U));
        number /= 10U;
    }

    for (i = index; i > 0U; i--)
    {
        USART1_SendByte((uint8_t)buffer[i - 1U]);
    }
}
#endif


/* =========================================================
 * USART2 / PN532
 * ========================================================= */

/*
 * STM32F1:
 * ? SR ??? DR,?? RXNE / ORE ??????
 */
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
 * PN532 ???????????????????
 */
static uint16_t USART2_CollectPollingRx(
    uint8_t *raw,
    uint16_t capacity,
    uint32_t timeoutMs)
{
    uint16_t count = 0U;
    uint32_t start = g_ms_ticks;

    while ((uint32_t)(g_ms_ticks - start) < timeoutMs)
    {
        if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
        {
            uint8_t data;

            data = (uint8_t)USART_ReceiveData(USART2);

            if (count < capacity)
            {
                raw[count] = data;
                count++;
            }
        }
    }

    return count;
}


typedef enum
{
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


typedef struct
{
    uint8_t raw[PN532_RAW_RX_CAPACITY];
    uint16_t rawCount;

} PN532_RawTrace;


static uint16_t PN532_FindAckOffset(
    const uint8_t *raw,
    uint16_t count)
{
    static const uint8_t ack[] =
    {
        0x00U, 0x00U, 0xFFU,
        0x00U, 0xFFU, 0x00U
    };

    uint16_t offset;
    uint8_t index;

    for (offset = 0U;
         (uint16_t)(offset + 6U) <= count;
         offset++)
    {
        for (index = 0U; index < 6U; index++)
        {
            if (raw[offset + index] != ack[index])
            {
                break;
            }
        }

        if (index == 6U)
        {
            return offset;
        }
    }

    return 0xFFFFU;
}


static PN532_TransactionResult PN532_ParseResponse(
    const uint8_t *raw,
    uint16_t count,
    uint8_t expectedCommand,
    uint16_t ackOffset,
    const uint8_t **responseData,
    uint8_t *responseLength)
{
    uint16_t offset;
    uint8_t length;
    uint8_t checksum = 0U;
    uint8_t index;

    if ((uint16_t)(ackOffset + 6U) >= count)
    {
        return PN532_TRANSACTION_RESPONSE_MISSING;
    }


    /*
     * ? ACK ???:
     *
     * 00 00 FF
     */
    for (offset = (uint16_t)(ackOffset + 6U);
         (uint16_t)(offset + 3U) <= count;
         offset++)
    {
        if (raw[offset] == 0x00U &&
            raw[offset + 1U] == 0x00U &&
            raw[offset + 2U] == 0xFFU)
        {
            break;
        }
    }


    if ((uint16_t)(offset + 3U) > count)
    {
        return PN532_TRANSACTION_PREAMBLE_INVALID;
    }


    if ((uint16_t)(offset + 5U) > count)
    {
        return PN532_TRANSACTION_RESPONSE_MISSING;
    }


    length = raw[offset + 3U];


    if (length == 0U || length == 0xFFU)
    {
        return PN532_TRANSACTION_LEN_INVALID;
    }


    if ((uint8_t)(length + raw[offset + 4U]) != 0U)
    {
        return PN532_TRANSACTION_LCS_INVALID;
    }


    /*
     * ??????:
     *
     * 00 00 FF
     * LEN
     * LCS
     * DATA...
     * DCS
     * 00
     *
     * ? LEN + 7 ??
     */
    if ((uint16_t)(offset + 7U + length) > count)
    {
        return PN532_TRANSACTION_RESPONSE_MISSING;
    }


    if (raw[offset + 5U] != 0xD5U)
    {
        return PN532_TRANSACTION_TFI_INVALID;
    }


    if (length < 2U ||
        raw[offset + 6U] != expectedCommand)
    {
        return PN532_TRANSACTION_COMMAND_INVALID;
    }


    for (index = 0U; index < length; index++)
    {
        checksum =
            (uint8_t)(checksum + raw[offset + 5U + index]);
    }


    if ((uint8_t)(
        checksum +
        raw[offset + 5U + length]) != 0U)
    {
        return PN532_TRANSACTION_DCS_INVALID;
    }


    if (raw[offset + 6U + length] != 0x00U)
    {
        return PN532_TRANSACTION_POSTAMBLE_INVALID;
    }


    *responseData = &raw[offset + 5U];
    *responseLength = length;

    return PN532_TRANSACTION_OK;
}


static PN532_TransactionResult PN532_Transceive(
    const uint8_t *command,
    uint16_t commandLength,
    uint8_t *response,
    uint8_t responseMax,
    uint8_t *responseLength,
    PN532_RawTrace *trace)
{
    static const uint8_t hostAck[
        PN532_HOST_ACK_FRAME_LENGTH] =
    {
        0x00U, 0x00U, 0xFFU,
        0x00U, 0xFFU, 0x00U
    };

    uint8_t localRaw[PN532_RAW_RX_CAPACITY];

    uint8_t *raw =
        (trace == 0) ?
        localRaw :
        trace->raw;

    const uint8_t *frameData;
    uint8_t frameLength;

    uint16_t rawCount;
    uint16_t ackOffset;

    uint8_t index;

    PN532_TransactionResult result;


    *responseLength = 0U;


    if (trace != 0)
    {
        trace->rawCount = 0U;
    }


    if (commandLength < 7U)
    {
        return PN532_TRANSACTION_COMMAND_INVALID;
    }


    USART2_ClearReceiveStatus();

    USART2_NFC_SendBuffer(
        command,
        commandLength);


    rawCount =
        USART2_CollectPollingRx(
            raw,
            PN532_RAW_RX_CAPACITY,
            500U);


    if (trace != 0)
    {
        trace->rawCount = rawCount;
    }


    if (rawCount == 0U)
    {
        return PN532_TRANSACTION_NO_ACK;
    }


    ackOffset =
        PN532_FindAckOffset(
            raw,
            rawCount);


    if (ackOffset == 0xFFFFU)
    {
        return PN532_TRANSACTION_ACK_INVALID;
    }


    result =
        PN532_ParseResponse(
            raw,
            rawCount,
            (uint8_t)(command[6] + 1U),
            ackOffset,
            &frameData,
            &frameLength);


    if (result != PN532_TRANSACTION_OK)
    {
        return result;
    }


    if (frameLength > responseMax)
    {
        return PN532_TRANSACTION_LEN_INVALID;
    }


    for (index = 0U;
         index < frameLength;
         index++)
    {
        response[index] = frameData[index];
    }


    *responseLength = frameLength;


    /*
     * ???? PN532 ??????:
     * ?? Response ????? Host ACK?
     */
    USART2_NFC_SendBuffer(
        hostAck,
        PN532_HOST_ACK_FRAME_LENGTH);


    return PN532_TRANSACTION_OK;
}


/* =========================================================
 * PN532 UID
 * ========================================================= */

#if (CAR_REMOTE_CONTROL_TEST_MODE == 0U)

static uint8_t PN532_ParseUid(
    const uint8_t *response,
    uint8_t responseLength,
    const uint8_t **uid,
    uint8_t *uidLength)
{
    uint8_t length;


    if (responseLength < 8U ||
        response[2] == 0U)
    {
        return 0U;
    }


    length = response[7];


    if (length == 0U ||
        length > 10U ||
        responseLength <
            (uint8_t)(8U + length))
    {
        return 0U;
    }


    *uid = &response[8];
    *uidLength = length;


    return 1U;
}


static uint8_t PN532_IsSameUid(
    const uint8_t *first,
    uint8_t firstLength,
    const uint8_t *second,
    uint8_t secondLength)
{
    uint8_t index;


    if (firstLength != secondLength)
    {
        return 0U;
    }


    for (index = 0U;
         index < firstLength;
         index++)
    {
        if (first[index] != second[index])
        {
            return 0U;
        }
    }


    return 1U;
}


static uint8_t PN532_IsAuthorizedUid(
    const uint8_t *uid,
    uint8_t uidLength)
{
    uint8_t index;


    if (uidLength != sizeof(authorized_uid))
    {
        return 0U;
    }


    for (index = 0U;
         index < sizeof(authorized_uid);
         index++)
    {
        if (uid[index] != authorized_uid[index])
        {
            return 0U;
        }
    }


    return 1U;
}


static void PN532_PrintUid(
    const uint8_t *uid,
    uint8_t uidLength)
{
    uint8_t index;


    USART1_SendString(
        "NFC card found\r\nUID: ");


    for (index = 0U;
         index < uidLength;
         index++)
    {
        USART1_PrintHexByte(uid[index]);

        if (index + 1U < uidLength)
        {
            USART1_SendByte(' ');
        }
    }


    USART1_SendString("\r\n");


    if (PN532_IsAuthorizedUid(
        uid,
        uidLength) != 0U)
    {
        USART1_SendString(
            "CARD OK\r\n");
    }
    else
    {
        USART1_SendString(
            "CARD DENIED\r\n");
    }
}
#endif


/* =========================================================
 * ???????
 * ========================================================= */

/*
 * ?????????????
 */
#if CAR_ACTION_TEST_MODE

static void Car_ApplyAction(uint32_t actionIndex)
{
    motor_left_set(
        car_actions[actionIndex].left_speed);

    motor_right_set(
        car_actions[actionIndex].right_speed);
}


/*
 * ???????
 */
static void Car_PrintAction(uint32_t actionIndex)
{
    USART1_SendString("ACTION ");

    USART1_PrintSignedInt(
        (int)actionIndex);

    USART1_SendString("  L=");

    USART1_PrintSignedInt(
        car_actions[actionIndex].left_speed);

    USART1_SendString(" R=");

    USART1_PrintSignedInt(
        car_actions[actionIndex].right_speed);

    USART1_SendString("\r\n");
}

#endif

/* =========================================================
 * main
 * ========================================================= */

int main(void)
{
#if (MOTOR_BASELINE_TEST_MODE == 0U) && \
    (MOTOR_SIDE_MAPPING_TEST_MODE == 0U)
    uint8_t frame[
        USART1_HI3861_FRAME_LEN];

#endif


    /* -----------------------------------------------------
     * PN532 ???
     * ----------------------------------------------------- */

    static const uint8_t pn532Wakeup[
        PN532_WAKEUP_FRAME_LENGTH] =
    {
        0x55U,
        0x55U,
        0x00U,
        0x00U,
        0x00U
    };


    static const uint8_t pn532GetFirmwareVersion[
        PN532_FIRMWARE_FRAME_LENGTH] =
    {
        0x00U, 0x00U, 0xFFU,
        0x02U, 0xFEU,
        0xD4U, 0x02U,
        0x2AU,
        0x00U
    };


    /*
     * ???????:
     *
     * Wakeup + SAM ???? buffer ???
     */
    static const uint8_t sam_wake_frame[26] =
    {
        0x55U, 0x55U,

        0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U,

        0x00U, 0x00U, 0xFFU,
        0x05U, 0xFBU,
        0xD4U, 0x14U,
        0x01U, 0x14U, 0x01U,
        0x02U,
        0x00U
    };


    static const uint8_t pn532RfConfiguration[] =
    {
        0x00U, 0x00U, 0xFFU,
        0x06U, 0xFAU,
        0xD4U, 0x32U,
        0x05U, 0xFFU,
        0x01U, 0x01U,
        0xF4U,
        0x00U
    };


#if (CAR_REMOTE_CONTROL_TEST_MODE == 0U)
    static const uint8_t pn532InListPassiveTarget[] =
    {
        0x00U, 0x00U, 0xFFU,
        0x04U, 0xFCU,
        0xD4U, 0x4AU,
        0x01U, 0x00U,
        0xE1U,
        0x00U
    };
#endif


    uint8_t pn532Response[32];
    uint8_t pn532ResponseLength;

    PN532_RawTrace samTrace;

    const uint8_t *samResponse;
    uint8_t samResponseLength;

    uint16_t samAckOffset;

    PN532_TransactionResult samResult;

#if (CAR_REMOTE_CONTROL_TEST_MODE == 0U)
    uint8_t nfcReady = 0U;
    uint8_t lastUid[10];
    uint8_t lastUidLength = 0U;
    const uint8_t *uid;
    uint8_t uidLength;
    uint8_t index;
#endif


#if CAR_ACTION_TEST_MODE

    uint32_t actionIndex = 0U;

    uint32_t actionStart;

    uint32_t actionLastLog;

    uint32_t now;

#endif


    /* =====================================================
     * ????
     * ===================================================== */

    BSP_GPIO_Init();

    Light_Init();

    USART1_Hi3861_Init();

    motor_init();

    motor_stop();

    encoder_init();
#if (ENCODER_TELEMETRY_TEST_MODE == 1U)
    (void)encoder_left_get_delta();
    (void)encoder_right_get_delta();
#endif


    /* =====================================================
     * SysTick 1ms
     * ===================================================== */

    SystemCoreClockUpdate();


    if (SysTick_Config(
        SystemCoreClock / 1000U) != 0U)
    {
        while (1)
        {
        }
    }

    USART1_SendString("MCU READY\r\n");


    /* =====================================================
     * PN532 ???
     * ===================================================== */

    USART2_NFC_Init();

    USART_ITConfig(
        USART2,
        USART_IT_RXNE,
        DISABLE);


    delay_ms(100U);


    /*
     * Firmware ? Wakeup?
     */
    USART2_ClearReceiveStatus();

    USART2_NFC_SendBuffer(
        pn532Wakeup,
        PN532_WAKEUP_FRAME_LENGTH);


    delay_ms(5U);


    if (
        PN532_Transceive(
            pn532GetFirmwareVersion,
            PN532_FIRMWARE_FRAME_LENGTH,
            pn532Response,
            sizeof(pn532Response),
            &pn532ResponseLength,
            0)
            == PN532_TRANSACTION_OK
        &&
        pn532ResponseLength >= 6U
       )
    {
        USART1_SendString(
            "PN532 firmware verified\r\n");


        delay_ms(5U);


        /*
         * ?????:
         *
         * SAM ??? Wakeup,
         * Wakeup + SAM ?????
         */
        USART2_NFC_SendBuffer(
            sam_wake_frame,
            sizeof(sam_wake_frame));


        samTrace.rawCount =
            USART2_CollectPollingRx(
                samTrace.raw,
                PN532_RAW_RX_CAPACITY,
                2000U);


        samAckOffset =
            PN532_FindAckOffset(
                samTrace.raw,
                samTrace.rawCount);


        if (samAckOffset == 0xFFFFU)
        {
            samResult =
                (samTrace.rawCount == 0U)
                ?
                PN532_TRANSACTION_NO_ACK
                :
                PN532_TRANSACTION_ACK_INVALID;
        }
        else
        {
            samResult =
                PN532_ParseResponse(
                    samTrace.raw,
                    samTrace.rawCount,
                    0x15U,
                    samAckOffset,
                    &samResponse,
                    &samResponseLength);
        }


        if (
            samResult == PN532_TRANSACTION_OK
            &&
            samResponseLength == 2U
           )
        {
            static const uint8_t hostAck[
                PN532_HOST_ACK_FRAME_LENGTH] =
            {
                0x00U, 0x00U, 0xFFU,
                0x00U, 0xFFU, 0x00U
            };


            USART2_NFC_SendBuffer(
                hostAck,
                PN532_HOST_ACK_FRAME_LENGTH);


            USART1_SendString(
                "PN532 SAM configured\r\n");


            delay_ms(5U);


            if (
                PN532_Transceive(
                    pn532RfConfiguration,
                    sizeof(pn532RfConfiguration),
                    pn532Response,
                    sizeof(pn532Response),
                    &pn532ResponseLength,
                    0)
                    != PN532_TRANSACTION_OK
               )
            {
                USART1_SendString(
                    "PN532 RF configuration failed\r\n");
            }


#if (CAR_REMOTE_CONTROL_TEST_MODE == 0U)
            nfcReady = 1U;
#endif
        }
    }
    else
    {
        USART1_SendString(
            "PN532 firmware failed\r\n");
    }

#if (MOTOR_BASELINE_TEST_MODE == 1U) || (MOTOR_BASELINE_TEST_MODE == 2U)
    MotorBaselineStart();
#endif

#if (MOTOR_SIDE_MAPPING_TEST_MODE == 1U)
    MotorSideMappingTest();
#endif

#if (MOTOR_BASELINE_TEST_MODE == 0U) && \
    (MOTOR_SIDE_MAPPING_TEST_MODE == 0U)
    USART1_SendString("NORMAL UART MOTOR MODE\r\n");
    USART1_SendString("UART MOTOR CONTROL READY\r\n");
#endif


    /* =====================================================
     * ?????????
     * ===================================================== */

#if CAR_ACTION_TEST_MODE

    /*
     * ???0???
     */
    actionIndex = 0U;

    actionStart = g_ms_ticks;

    actionLastLog = g_ms_ticks;


    Car_ApplyAction(actionIndex);

    Car_PrintAction(actionIndex);

#endif


    /* =====================================================
     * ???
     * ===================================================== */

    while (1)
    {
        Light_Run();
#if (ENCODER_TELEMETRY_TEST_MODE == 1U)
        {
            static uint32_t lastEncoderTxMs;
            static uint8_t encoderSeq;
            if ((uint32_t)(g_ms_ticks - lastEncoderTxMs) >= ENCODER_TELEMETRY_PERIOD_MS) {
                int16_t leftDelta = encoder_left_get_delta();
                int16_t rightDelta = encoder_right_get_delta();
                uint8_t frame[8];
                lastEncoderTxMs = g_ms_ticks;
                frame[0] = 0xECU; frame[1] = encoderSeq++;
                frame[2] = (uint8_t)leftDelta; frame[3] = (uint8_t)((uint16_t)leftDelta >> 8);
                frame[4] = (uint8_t)rightDelta; frame[5] = (uint8_t)((uint16_t)rightDelta >> 8);
                frame[6] = (uint8_t)(frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
                frame[7] = 0xEDU;
                for (uint8_t i = 0U; i < 8U; i++) USART1_SendByte(frame[i]);
            }
        }
#endif

#if (MOTOR_BASELINE_TEST_MODE == 0U) && \
    (MOTOR_SIDE_MAPPING_TEST_MODE == 0U)
        /* -------------------------------------------------
         * USART1 ????
         * ------------------------------------------------- */
        if (USART1_GetReceivedFrame(frame))
        {
            ApplyHi3861MotorFrame(frame);
        }
#endif

#if (MOTOR_BASELINE_TEST_MODE != 0U)
        WheelStraightDiagnosticPoll();
#endif


        /* =================================================
         * ???????
         * ================================================= */

#if CAR_ACTION_TEST_MODE

        now = g_ms_ticks;


        /*
         * ?????????
         */
        if ((uint32_t)(
                now - actionStart)
            >=
            car_actions[actionIndex].duration_ms)
        {
            /*
             * ?????????
             */
            actionIndex++;


            /*
             * ???????????0??
             */
            if (actionIndex >= CAR_ACTION_COUNT)
            {
                actionIndex = 0U;
            }


            actionStart = now;


            /*
             * ???????PWM?
             *
             * ??? motor_stop(),
             * ???????????
             */
            Car_ApplyAction(actionIndex);


            Car_PrintAction(actionIndex);
        }


        /*
         * ?????????
         */
        if ((uint32_t)(
                now - actionLastLog)
            >=
            CAR_ACTION_LOG_INTERVAL_MS)
        {
            actionLastLog = now;

            Car_PrintAction(actionIndex);
        }


#elif ((CAR_REMOTE_CONTROL_TEST_MODE == 0U) && \
       (MOTOR_BASELINE_TEST_MODE == 0U))

        /* =================================================
         * ?? NFC ??
         * ================================================= */

        if (nfcReady != 0U)
        {
            if (
                PN532_Transceive(
                    pn532InListPassiveTarget,
                    sizeof(pn532InListPassiveTarget),
                    pn532Response,
                    sizeof(pn532Response),
                    &pn532ResponseLength,
                    0)
                    == PN532_TRANSACTION_OK
               )
            {
                /*
                 * NbTg == 0:
                 * ????
                 */
                if (
                    pn532ResponseLength >= 3U
                    &&
                    pn532Response[2] == 0U
                   )
                {
                    lastUidLength = 0U;
                }
                else if (
                    PN532_ParseUid(
                        pn532Response,
                        pn532ResponseLength,
                        &uid,
                        &uidLength)
                    != 0U
                        )
                {
                    if (
                        PN532_IsSameUid(
                            lastUid,
                            lastUidLength,
                            uid,
                            uidLength)
                        == 0U
                       )
                    {
                        PN532_PrintUid(
                            uid,
                            uidLength);


                        for (
                            index = 0U;
                            index < uidLength;
                            index++)
                        {
                            lastUid[index] =
                                uid[index];
                        }


                        lastUidLength =
                            uidLength;
                    }
                }
                else
                {
                    lastUidLength = 0U;
                }
            }
            else
            {
                lastUidLength = 0U;
            }


            delay_ms(300U);
        }

#endif
    }
}


/*
 * Apply the already validated Hi3861 frame protocol in the foreground task.
 * The USART1 IRQ only assembles the frame; PWM changes must not run in ISR.
 */
#if (MOTOR_BASELINE_TEST_MODE == 0U) && \
    (MOTOR_SIDE_MAPPING_TEST_MODE == 0U)
static void ApplyHi3861MotorFrame(const uint8_t *frame)
{
    static uint8_t lastLeftDir;
    static uint8_t lastLeftCmd;
    static uint8_t lastRightDir;
    static uint8_t lastRightCmd;
    static uint8_t lastCommandValid = 0U;
    uint8_t leftDir;
    uint8_t leftCmd;
    uint8_t rightDir;
    uint8_t rightCmd;
    uint8_t commandChanged;
    int baseLeft;
    int baseRight;
    int leftSpeed;
    int rightSpeed;

    if (frame == 0 ||
        frame[0] != USART1_HI3861_FRAME_HEAD ||
        frame[5] != USART1_HI3861_FRAME_TAIL)
    {
        return;
    }

    leftDir = frame[1];
    leftCmd = frame[2];
    rightDir = frame[3];
    rightCmd = frame[4];

    if (leftCmd > 150U)
    {
        leftCmd = 150U;
    }
    if (rightCmd > 150U)
    {
        rightCmd = 150U;
    }

    baseLeft = ((int)leftCmd * 1000) / 150;
    baseRight = ((int)rightCmd * 1000) / 150;

    commandChanged = (lastCommandValid == 0U ||
                      leftDir != lastLeftDir ||
                      leftCmd != lastLeftCmd ||
                      rightDir != lastRightDir ||
                      rightCmd != lastRightCmd) ? 1U : 0U;
    lastLeftDir = leftDir;
    lastLeftCmd = leftCmd;
    lastRightDir = rightDir;
    lastRightCmd = rightCmd;
    lastCommandValid = 1U;

    /* No fixed left/right gain and no encoder feedback in this version. */
    if (leftDir == 0U)
    {
        leftSpeed = baseLeft;
    }
    else
    {
        leftSpeed = -baseLeft;
    }

    if (rightDir == 0U)
    {
        rightSpeed = baseRight;
    }
    else
    {
        rightSpeed = -baseRight;
    }

    if (leftCmd == 0U && rightCmd == 0U)
    {
        motor_stop();
    }
    else
    {
        motor_left_set(leftSpeed);
        motor_right_set(rightSpeed);
    }

    if (commandChanged != 0U)
    {
        USART1_SendString("CTRL Lcmd=");
        USART1_PrintSignedInt((int)leftCmd);
        USART1_SendString(" Rcmd=");
        USART1_PrintSignedInt((int)rightCmd);
        USART1_SendString(" pwm L=");
        USART1_PrintSignedInt(leftSpeed);
        USART1_SendString(" R=");
        USART1_PrintSignedInt(rightSpeed);
        USART1_SendString("\r\n");
    }
}
#endif
