/**
 ******************************************************************************
 * @file    timebase.h
 * @brief   Timebase de alta resolucion con DWT CYCCNT (Cortex-M4).
 *
 *          SYSCLK = 168 MHz -> 1 ciclo = 5.952 ns, CYCCNT desborda a ~25.6 s.
 *          Seguro llamar timebase_now() desde ISR.
 ******************************************************************************
 */

#ifndef TESTS_INC_TIMEBASE_H_
#define TESTS_INC_TIMEBASE_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Habilita DWT CYCCNT. Idempotente. false si el core no tiene ciclo-contador. */
bool timebase_init(void);
bool timebase_ready(void);

/* Ciclos crudos. Inline, ~2 ciclos. Seguro desde ISR. */
static inline uint32_t timebase_now(void)
{
    return DWT->CYCCNT;
}

uint32_t timebase_cycles_per_us(void);
uint32_t timebase_cycles_to_us(uint32_t cycles);
uint32_t timebase_cycles_to_us_x100(uint32_t cycles);
uint32_t timebase_delta_us(uint32_t t0, uint32_t t1);
uint32_t timebase_elapsed_us(uint32_t t0);

/* Tiempo absoluto sin limite de 25 s. Llamar al menos una vez cada ~20 s. */
uint64_t timebase_cycles64(void);
uint64_t timebase_us64(void);

/* Busy-wait, para bring-up / autotest. */
void timebase_delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_TIMEBASE_H_ */
