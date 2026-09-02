/**
 ******************************************************************************
 * @file    fc_rc.h
 * @brief   Canales CRSF -> setpoints de vuelo (equivalente a fc/rc.c +
 *          fc/rc_controls.c de Betaflight).
 *
 *          Radio: PocketMaster / ELRS por UART4 @ 420000 (driver_crsf).
 *          Mapeo AETR (default de EdgeTX/ELRS):
 *            CH1 = roll (A)   CH2 = pitch (E)
 *            CH3 = throttle (T) CH4 = yaw (R)
 *            CH5 = AUX1 -> switch de armado
 *
 *          Rango CRSF: 172 (min) .. 992 (centro) .. 1811 (max).
 *
 *          Salidas:
 *            throttle 0..1
 *            setpoint por eje en dps (roll, pitch, yaw), con deadband y expo
 *            estado del enlace (edad del ultimo frame RC valido)
 ******************************************************************************
 */

#ifndef FC_FC_RC_H_
#define FC_FC_RC_H_

#include "driver_crsf.h"
#include "pid.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Indices de canal (base 0). */
#define FC_RC_CH_ROLL       0u
#define FC_RC_CH_PITCH      1u
#define FC_RC_CH_THROTTLE   2u
#define FC_RC_CH_YAW        3u
#define FC_RC_CH_ARM        4u   /* AUX1 */

/* Escala CRSF. */
#define FC_RC_RAW_MIN       172u
#define FC_RC_RAW_MID       992u
#define FC_RC_RAW_MAX       1811u

/* Umbral del switch de armado (por encima = armado pedido). */
#define FC_RC_ARM_THRESHOLD 1500u

/* Rate maximo por eje con el stick al tope. */
#define FC_RC_RATE_DPS      400.0f
#define FC_RC_YAW_RATE_DPS  300.0f

/* Zona muerta alrededor del centro, en cuentas CRSF. */
#define FC_RC_DEADBAND      8u

/* Expo (0 = lineal, 0.5 = suave en el centro). */
#define FC_RC_EXPO          0.30f

/* Sin frame RC valido por mas de esto -> link caido. */
#define FC_RC_LINK_TIMEOUT_MS  500u

typedef struct {
    float    throttle;                        /* 0..1                        */
    float    setpoint_dps[PID_AXIS_COUNT];    /* roll, pitch, yaw            */
    bool     arm_switch;                      /* AUX1 en alto                */
    bool     link_ok;                         /* frames frescos              */
    uint32_t age_ms;                          /* edad del ultimo frame RC    */
    uint16_t raw[CRSF_CHANNELS_COUNT];        /* canales crudos              */
    uint32_t frames;                          /* frames 0x16 decodificados   */
} fc_rc_t;

/* Estado global (visible en Live Expressions de CubeIDE). */
extern fc_rc_t g_fcRc;

void fc_rc_init(void);

/* ------------------------------------------------------------------------- */
/* Consume el ring DMA del CRSF y actualiza g_fcRc. No bloquea.               */
/* Devuelve true si en esta pasada llego al menos un frame de canales nuevo.  */
/* ------------------------------------------------------------------------- */
bool fc_rc_poll(void);

/* Recalcula edad / link_ok sin tocar el UART (se llama solo desde fc_rc_poll  */
/* y desde el chequeo de failsafe).                                           */
void fc_rc_update_link(void);

const fc_rc_t *fc_rc_get(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_FC_RC_H_ */
