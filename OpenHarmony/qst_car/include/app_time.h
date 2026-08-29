#ifndef QST_CAR_APP_TIME_H
#define QST_CAR_APP_TIME_H

#include <stdint.h>
#include "cmsis_os2.h"

static inline uint32_t AppMsToTicks(uint32_t milliseconds)
{
    uint32_t frequency;
    uint64_t ticks;

    if (milliseconds == 0U) {
        return 0U;
    }

    frequency = osKernelGetTickFreq();
    if (frequency == 0U) {
        return 1U;
    }

    ticks = ((uint64_t)milliseconds * frequency + 999U) / 1000U;
    if (ticks == 0U) {
        return 1U;
    }
    if (ticks > 0xFFFFFFFFULL) {
        return 0xFFFFFFFFU;
    }
    return (uint32_t)ticks;
}

/* Convert an RTOS tick value to milliseconds without assuming 1 tick == 1 ms. */
static inline uint32_t AppTicksToMs(uint32_t ticks)
{
    uint32_t frequency = osKernelGetTickFreq();
    uint64_t milliseconds;

    if (frequency == 0U) {
        return 0U;
    }

    milliseconds = ((uint64_t)ticks * 1000U) / frequency;
    if (milliseconds > 0xFFFFFFFFULL) {
        return 0xFFFFFFFFU;
    }
    return (uint32_t)milliseconds;
}

#endif
