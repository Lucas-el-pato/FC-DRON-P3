/**
 ******************************************************************************
 * @file    mixer.h
 * @brief   Mezcla quad-X (equivalente a flight/mixer.c de Betaflight).
 *
 *          Toma throttle (0..1) + correcciones del PID (-1..+1 por eje) y
 *          produce los 4 valores DShot que consume motors_write4().
 *
 *          Orden de salida = canal TIM2 (indice 0..3 = CH1..CH4). Las
 *          etiquetas M1..M4 son las de la PCB:
 *            CH1 = M4 delantero derecho (PA15)
 *            CH2 = M3 trasero derecho   (PB3 )
 *            CH3 = M2 delantero izq.    (PA2 )
 *            CH4 = M1 trasero izquierdo (PA3 )
 *
 *          IMPORTANTE: verificar contra el cableado real de la PCB antes de
 *          poner helices. Si el orden fisico difiere, ajustar MIXER_QUAD_X.
 *
 *          Tabla (throttle, roll, pitch, yaw):
 *            CH1 (M4):  1, -1, -1,  1
 *            CH2 (M3):  1, -1,  1, -1
 *            CH3 (M2):  1,  1, -1, -1
 *            CH4 (M1):  1,  1,  1,  1
 *
 *          Sentido de giro (props-in, visto desde arriba):
 *            M4 antihorario (6030), M3 horario (6030R),
 *            M2 horario (6030R),    M1 antihorario (6030)
 ******************************************************************************
 */

#ifndef CONTROL_MIXER_H_
#define CONTROL_MIXER_H_

#include "pid.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MIXER_MOTOR_COUNT   4u

/* Motores que giran invertidos respecto del sentido normal del ESC
 * (bit0 = CH1 .. bit3 = CH4). Con los 4 ESC en normal = antihorario, invertir
 * CH2 (M3) y CH3 (M2) deja el sentido de giro de la tabla de arriba. Se
 * aplica por DShot (comandos 7/8) en cada armado, ver arming.c.             */
#ifndef MIXER_MOTOR_REVERSED_MASK
#define MIXER_MOTOR_REVERSED_MASK   0x06u
#endif

/* Props-in (default de BF): el PID de yaw se invierte antes de la mezcla,
 * igual que mixer.c de BF con yaw_motors_reversed = OFF. Poner 1 si las
 * helices se montan props-out (sentidos de giro opuestos a la tabla).      */
#ifndef MIXER_YAW_MOTORS_REVERSED
#define MIXER_YAW_MOTORS_REVERSED   0
#endif

/* Rango DShot util: 0 = motor parado, 48 = minimo comandable, 2047 = maximo. */
#define MIXER_DSHOT_MIN     48u
#define MIXER_DSHOT_MAX     2047u

/* Ralenti con el drone armado (fraccion del rango). Betaflight: motor_idle. */
#define MIXER_IDLE_THROTTLE 0.055f

/* Techo de throttle en banco/primeros vuelos. 1.0f = sin limite. */
#ifndef MIXER_THROTTLE_LIMIT
#define MIXER_THROTTLE_LIMIT 0.6f
#endif

/* Deja el estado interno listo (llamar una vez en el init del FC). */
void mixer_init(void);

/* ------------------------------------------------------------------------- */
/* Calcula la salida de los 4 motores.                                        */
/*   throttle : 0..1 del stick                                                */
/*   pid_axis : correccion por eje (roll, pitch, yaw); NULL = passthrough      */
/*   armed    : false -> los 4 motores salen en 0 (parados)                   */
/*   out      : valores DShot 0 / 48..2047                                     */
/* ------------------------------------------------------------------------- */
void mixer_run(float throttle,
               const float pid_axis[PID_AXIS_COUNT],
               bool armed,
               uint16_t out[MIXER_MOTOR_COUNT]);

/* Ultima salida calculada (para el log de consola). */
const uint16_t *mixer_last_output(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_MIXER_H_ */
