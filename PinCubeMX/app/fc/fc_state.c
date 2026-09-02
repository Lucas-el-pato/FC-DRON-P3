/**
 ******************************************************************************
 * @file    fc_state.c
 * @brief   Estado global del FC (ver fc_state.h).
 ******************************************************************************
 */

#include "fc_state.h"
#include <string.h>

fc_state_t g_fcState;

void fc_state_init(void)
{
    memset(&g_fcState, 0, sizeof(g_fcState));
    g_fcState.mode = FC_MODE_ACRO;
}

fc_state_t *fc_state(void)
{
    return &g_fcState;
}
