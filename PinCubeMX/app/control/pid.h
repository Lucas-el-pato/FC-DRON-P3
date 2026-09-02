/**
 ******************************************************************************
 * @file    pid.h
 * @brief   PID de rate (equivalente a flight/pid.c de Betaflight, un solo
 *          perfil, modo ACRO).
 *
 *          Entrada:  setpoint en dps (del stick) y gyro en dps (filtrado).
 *          Salida:   correccion normalizada por eje, -1..+1, que consume el
 *                    mixer junto con el throttle.
 *
 *          Convenciones (mismas que Betaflight):
 *            eje 0 = roll, eje 1 = pitch, eje 2 = yaw
 *            error = setpoint - gyro
 *            D se calcula sobre el gyro filtrado (no sobre el error), asi un
 *            escalon de stick no genera un pico de D.
 *
 *          dt es fijo (1 / PID_RATE_HZ): el lazo lo dispara el DRDY del gyro
 *          con un divisor entero, no se mide tiempo por iteracion.
 ******************************************************************************
 */

#ifndef CONTROL_PID_H_
#define CONTROL_PID_H_

#include "filter.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PID_AXIS_COUNT      3u
#define PID_AXIS_ROLL       0u
#define PID_AXIS_PITCH      1u
#define PID_AXIS_YAW        2u

/* Ganancias en unidades normalizadas: salida por dps de error.
 * Con KP = 0.0020, un error de 500 dps satura el eje (salida 1.0).
 * Punto de partida conservador: afinar recien con el drone atado. */
#define PID_ROLL_KP         0.0020f
#define PID_ROLL_KI         0.0025f
#define PID_ROLL_KD         0.000020f

#define PID_PITCH_KP        0.0020f
#define PID_PITCH_KI        0.0025f
#define PID_PITCH_KD        0.000020f

#define PID_YAW_KP          0.0030f
#define PID_YAW_KI          0.0035f
#define PID_YAW_KD          0.0f

/* Limites de saturacion. */
#define PID_ITERM_LIMIT     0.30f   /* anti-windup por eje */
#define PID_OUTPUT_LIMIT    1.00f   /* salida por eje      */

/* Debajo de este throttle el I-term se congela y decae (evita windup en el
 * suelo con los motores al ralenti). */
#define PID_ITERM_RELAX_THROTTLE  0.05f

/* Cutoff del PT1 sobre el gyro que alimenta al D-term. */
#define PID_DTERM_LPF_HZ    100.0f

/* Inicializa ganancias, filtros y estado. dt_s = 1 / PID_RATE_HZ. */
void pid_init(float dt_s);

/* Borra I-term y derivadas (al armar/desarmar o al perder el link). */
void pid_reset(void);

/* ------------------------------------------------------------------------- */
/* Un paso del PID.                                                           */
/*   sp_dps[3]   : setpoint por eje (roll, pitch, yaw) en dps                  */
/*   gyro_dps[3] : gyro filtrado por eje, mismo orden y signo                  */
/*   throttle    : 0..1, solo para el I-term relax                             */
/*   out[3]      : correccion normalizada -1..+1 por eje                       */
/* ------------------------------------------------------------------------- */
void pid_update(const float sp_dps[PID_AXIS_COUNT],
                const float gyro_dps[PID_AXIS_COUNT],
                float throttle,
                float out[PID_AXIS_COUNT]);

/* Ultimo I-term por eje (diagnostico en consola / Live Expressions). */
float pid_iterm(uint8_t axis);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_PID_H_ */
