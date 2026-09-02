/**
 ******************************************************************************
 * @file    failsafe.h
 * @brief   Maquina de estados de failsafe (equivalente a flight/failsafe.c de
 *          Betaflight, version minima).
 *
 *          Se evalua cada FAILSAFE_PERIOD_MS (10 ms) desde el lazo realtime,
 *          igual que el bloque "failsafeCheckDataFailurePeriod" del diagrama.
 *
 *            IDLE ---(sin frames RC por FAILSAFE_RXLOSS_MS)---> RX_LOSS_DETECTED
 *            RX_LOSS_DETECTED --(guard FAILSAFE_GUARD_MS)--> LANDING
 *            LANDING --(etapa 1: desarme inmediato)--> LANDED
 *            LANDED --(vuelve el link y el switch pasa por OFF)--> IDLE
 *
 *          Etapa 1 de este proyecto: LANDING desarma en el acto (no hay
 *          auto-landing con baro). La transicion queda hecha para engancharle
 *          un descenso controlado mas adelante.
 ******************************************************************************
 */

#ifndef CONTROL_FAILSAFE_H_
#define CONTROL_FAILSAFE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Periodo de evaluacion de la FSM. */
#define FAILSAFE_PERIOD_MS    10u

/* Sin frames RC validos por mas de esto -> se considera link caido. */
#define FAILSAFE_RXLOSS_MS    500u

/* Espera antes de actuar (filtra micro-cortes del enlace). */
#define FAILSAFE_GUARD_MS     100u

typedef enum {
    FAILSAFE_IDLE = 0,
    FAILSAFE_RX_LOSS_DETECTED,
    FAILSAFE_LANDING,
    FAILSAFE_LANDED
} failsafe_state_t;

void failsafe_init(void);

/* ------------------------------------------------------------------------- */
/* Un paso de la FSM. Llamar cada FAILSAFE_PERIOD_MS.                         */
/*   link_ok : hay frames RC frescos (< FAILSAFE_RXLOSS_MS)                   */
/*   armed   : estado actual de armado                                        */
/* Devuelve true si el failsafe pide desarmar en esta pasada.                 */
/* ------------------------------------------------------------------------- */
bool failsafe_update(bool link_ok, bool armed);

/* true mientras el failsafe no permita volar (bloquea el armado). */
bool failsafe_active(void);

failsafe_state_t failsafe_get_state(void);
const char *failsafe_state_name(failsafe_state_t st);

/* Cuantas veces se disparo el failsafe desde el arranque. */
uint32_t failsafe_event_count(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_FAILSAFE_H_ */
