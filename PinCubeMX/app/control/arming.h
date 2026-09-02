/**
 ******************************************************************************
 * @file    arming.h
 * @brief   Armado / desarmado y sus condiciones de bloqueo (equivalente a la
 *          parte de arming de fc/core.c + runtime_config.c de Betaflight).
 *
 *          Reglas para armar (todas a la vez):
 *            - switch de armado (AUX1) en alto
 *            - throttle por debajo de ARMING_THROTTLE_MAX
 *            - link RC vivo
 *            - failsafe inactivo
 *            - sin flags de bloqueo (gyro ausente, motores sin init, etc.)
 *            - el switch tiene que haber pasado por OFF desde el arranque
 *              (evita armar solo si la radio quedo prendida con el switch on)
 *
 *          Al desarmar: motores a 0 (frame DShot inmediato) + PID reseteado.
 ******************************************************************************
 */

#ifndef CONTROL_ARMING_H_
#define CONTROL_ARMING_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Throttle maximo (0..1) para permitir armar. */
#define ARMING_THROTTLE_MAX     0.05f

/* Ventana muerta tras el boot: no se arma antes de esto. */
#define ARMING_BOOT_GRACE_MS    3000u

/* Flags de bloqueo (bitmask, como ARMING_DISABLED_xxx de Betaflight). */
typedef enum {
    ARMING_DISABLED_NONE        = 0u,
    ARMING_DISABLED_NO_GYRO     = (1u << 0),
    ARMING_DISABLED_NO_MOTORS   = (1u << 1),
    ARMING_DISABLED_RX_LOSS     = (1u << 2),
    ARMING_DISABLED_THROTTLE    = (1u << 3),
    ARMING_DISABLED_FAILSAFE    = (1u << 4),
    ARMING_DISABLED_BOOT_GRACE  = (1u << 5),
    ARMING_DISABLED_ARM_SWITCH  = (1u << 6)   /* switch on desde el boot */
} arming_disable_flag_t;

void arming_init(void);

/* Flags "duros" que setea el init del FC (gyro/motores ausentes). */
void arming_disable_set(uint32_t flags);
void arming_disable_clear(uint32_t flags);
uint32_t arming_disable_flags(void);

/* ------------------------------------------------------------------------- */
/* Evalua las condiciones y arma/desarma. Llamar desde el lazo (cada pasada   */
/* de PID o de RX).                                                            */
/*   arm_switch       : AUX1 en alto                                          */
/*   throttle         : 0..1 del stick                                        */
/*   link_ok          : frames RC frescos                                     */
/*   failsafe_disarm  : el failsafe pide desarmar ya                          */
/*   failsafe_blocked : el failsafe no esta en IDLE (bloquea armar)           */
/* ------------------------------------------------------------------------- */
void arming_update(bool arm_switch,
                   float throttle,
                   bool link_ok,
                   bool failsafe_disarm,
                   bool failsafe_blocked);

bool arming_is_armed(void);

/* Desarme inmediato (motores a 0 + PID reset). Seguro llamarlo repetido. */
void arming_disarm(void);

uint32_t arming_arm_count(void);
uint32_t arming_disarm_count(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_ARMING_H_ */
