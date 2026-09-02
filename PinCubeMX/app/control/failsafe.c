/**
 ******************************************************************************
 * @file    failsafe.c
 * @brief   Implementacion de la FSM de failsafe (ver failsafe.h).
 ******************************************************************************
 */

#include "failsafe.h"
#include "stm32f4xx_hal.h"

static failsafe_state_t s_state = FAILSAFE_IDLE;
static uint32_t s_state_since_ms = 0u;
static uint32_t s_events = 0u;

static void failsafe_set_state(failsafe_state_t st)
{
    if (s_state != st) {
        s_state = st;
        s_state_since_ms = HAL_GetTick();
    }
}

void failsafe_init(void)
{
    s_state = FAILSAFE_IDLE;
    s_state_since_ms = HAL_GetTick();
    s_events = 0u;
}

bool failsafe_update(bool link_ok, bool armed)
{
    const uint32_t now = HAL_GetTick();
    const uint32_t in_state_ms = now - s_state_since_ms;
    bool request_disarm = false;

    switch (s_state) {
    case FAILSAFE_IDLE:
        if (!link_ok) {
            failsafe_set_state(FAILSAFE_RX_LOSS_DETECTED);
            s_events++;
        }
        break;

    case FAILSAFE_RX_LOSS_DETECTED:
        if (link_ok) {
            /* Micro-corte: el enlace volvio antes del guard. */
            failsafe_set_state(FAILSAFE_IDLE);
        } else if (in_state_ms >= FAILSAFE_GUARD_MS) {
            failsafe_set_state(armed ? FAILSAFE_LANDING : FAILSAFE_LANDED);
        }
        break;

    case FAILSAFE_LANDING:
        /* Etapa 1: sin descenso controlado, se corta el motor y se desarma. */
        request_disarm = true;
        failsafe_set_state(FAILSAFE_LANDED);
        break;

    case FAILSAFE_LANDED:
    default:
        request_disarm = armed;   /* no dejar que vuelva a arrancar solo */
        if (link_ok) {
            failsafe_set_state(FAILSAFE_IDLE);
        }
        break;
    }

    return request_disarm;
}

bool failsafe_active(void)
{
    return (s_state != FAILSAFE_IDLE);
}

failsafe_state_t failsafe_get_state(void)
{
    return s_state;
}

const char *failsafe_state_name(failsafe_state_t st)
{
    switch (st) {
    case FAILSAFE_IDLE:             return "IDLE";
    case FAILSAFE_RX_LOSS_DETECTED: return "RX_LOSS";
    case FAILSAFE_LANDING:          return "LANDING";
    case FAILSAFE_LANDED:           return "LANDED";
    default:                        return "?";
    }
}

uint32_t failsafe_event_count(void)
{
    return s_events;
}
