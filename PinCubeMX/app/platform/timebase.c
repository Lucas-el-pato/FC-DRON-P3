/**
 ******************************************************************************
 * @file    timebase.c
 * @brief   Implementacion del timebase DWT CYCCNT.
 ******************************************************************************
 */

#include "timebase.h"
#include "stm32f4xx_hal.h"

#define TIMEBASE_FALLBACK_HZ  168000000u

static bool     s_ready = false;
static uint32_t s_cycles_per_us = 168u;
static uint32_t s_last_cyc = 0u;
static uint64_t s_cycles64 = 0u;

bool timebase_init(void)
{
    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0u) {
        s_ready = false;
        return false;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t hz = SystemCoreClock;
    if (hz < 1000000u) {
        hz = TIMEBASE_FALLBACK_HZ;
    }
    s_cycles_per_us = hz / 1000000u;
    if (s_cycles_per_us == 0u) {
        s_cycles_per_us = 168u;
    }

    const uint32_t a = DWT->CYCCNT;
    const uint32_t b = DWT->CYCCNT;
    s_ready = (b != a);

    s_last_cyc = DWT->CYCCNT;
    s_cycles64 = 0u;

    return s_ready;
}

bool timebase_ready(void)
{
    return s_ready;
}

uint32_t timebase_cycles_per_us(void)
{
    return s_cycles_per_us;
}

uint32_t timebase_cycles_to_us(uint32_t cycles)
{
    if (s_cycles_per_us == 0u) {
        return 0u;
    }
    return cycles / s_cycles_per_us;
}

uint32_t timebase_cycles_to_us_x100(uint32_t cycles)
{
    if (s_cycles_per_us == 0u) {
        return 0u;
    }
    return (cycles * 100u) / s_cycles_per_us;
}

uint32_t timebase_delta_us(uint32_t t0, uint32_t t1)
{
    return timebase_cycles_to_us(t1 - t0);
}

uint32_t timebase_elapsed_us(uint32_t t0)
{
    return timebase_delta_us(t0, timebase_now());
}

static void timebase_update_cycles64(void)
{
    const uint32_t now = DWT->CYCCNT;
    s_cycles64 += (uint64_t)(now - s_last_cyc);
    s_last_cyc = now;
}

uint64_t timebase_cycles64(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    timebase_update_cycles64();
    const uint64_t out = s_cycles64;
    __set_PRIMASK(primask);
    return out;
}

uint64_t timebase_us64(void)
{
    if (s_cycles_per_us == 0u) {
        return 0u;
    }
    return timebase_cycles64() / (uint64_t)s_cycles_per_us;
}

void timebase_delay_us(uint32_t us)
{
    if (s_cycles_per_us == 0u) {
        return;
    }

    const uint32_t start = timebase_now();
    const uint32_t target = us * s_cycles_per_us;
    while ((timebase_now() - start) < target) {
        /* busy-wait */
    }
}
