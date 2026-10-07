/**
 ******************************************************************************
 * @file    pid.h
 * @brief   PID en modo ANGLE (equivalente a pidLevel() + pidController() de
 *          flight/pid.c de Betaflight, un solo perfil).
 *
 *          Dos lazos en cascada:
 *            nivel (roll/pitch): stick -> angulo objetivo (+/-60 grados)
 *                                error de angulo * PID_LEVEL_GAIN -> setpoint dps
 *            rate  (3 ejes)    : setpoint dps - gyro -> P + I + D
 *          Yaw no se autonivela: su setpoint es directo del stick en dps.
 *
 *          Ganancias en las mismas unidades que Betaflight (P45 I80 D40...),
 *          con PTERM/ITERM/DTERM_SCALE de flight/pid.h. La suma se limita a
 *          pidSumLimit y se divide por PID_MIXER_SCALING (1000) para entregar
 *          la correccion normalizada -1..+1 que consume el mixer.
 *
 *          Convenciones (mismas que Betaflight):
 *            eje 0 = roll, eje 1 = pitch, eje 2 = yaw
 *            error = setpoint - gyro
 *            D sobre el gyro (no sobre el error): un escalon de stick no
 *            genera un pico de D.
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

/* Escalas de Betaflight (flight/pid.h): convierten las ganancias "de
 * configurador" a unidades internas.                                        */
#define PID_PTERM_SCALE     0.032029f
#define PID_ITERM_SCALE     0.244381f
#define PID_DTERM_SCALE     0.000529f
#define PID_MIXER_SCALING   1000.0f

/* Ganancias en unidades Betaflight (defaults de BF 4.4).
 * Primer vuelo: atado y con ~70 % de estos valores.                         */
#define PID_ROLL_KP         45.0f
#define PID_ROLL_KI         80.0f
#define PID_ROLL_KD         40.0f

#define PID_PITCH_KP        47.0f
#define PID_PITCH_KI        84.0f
#define PID_PITCH_KD        46.0f

#define PID_YAW_KP          45.0f
#define PID_YAW_KI          80.0f
#define PID_YAW_KD          0.0f

/* Limites (iterm_windup / pidsum_limit / pidsum_limit_yaw de BF). */
#define PID_ITERM_LIMIT     400.0f
#define PID_SUM_LIMIT       500.0f
#define PID_SUM_LIMIT_YAW   400.0f

/* Modo ANGLE: angle_limit = 60, angle_p_gain = 50 -> levelGain = 50 / 10.
 * Un error de 10 grados pide 50 dps de correccion.                          */
#define PID_ANGLE_LIMIT_DEG     60.0f
#define PID_LEVEL_GAIN          5.0f
#define PID_LEVEL_MAX_RATE_DPS  400.0f

/* Debajo de este throttle (sin airmode) el I-term se mantiene en 0, como
 * pidResetIterm() de BF con el throttle abajo: evita windup en el suelo.   */
#define PID_ITERM_RELAX_THROTTLE  0.05f

/* Cutoff del PT1 sobre el gyro que alimenta al D-term. */
#define PID_DTERM_LPF_HZ    100.0f

/* Inicializa ganancias, filtros y estado. dt_s = 1 / PID_RATE_HZ. */
void pid_init(float dt_s);

/* Borra I-term y derivadas (al armar/desarmar o al perder el link). */
void pid_reset(void);

/* ------------------------------------------------------------------------- */
/* Un paso del PID en modo ANGLE.                                             */
/*   stick[3]     : deflexion -1..+1 (roll, pitch) -> angulo objetivo         */
/*   yaw_sp_dps   : setpoint de yaw en dps (rate, sin autonivelar)            */
/*   angle_deg[2] : actitud actual roll, pitch en grados (mismo signo que el  */
/*                  gyro: rotar con gyro positivo hace crecer el angulo)      */
/*   gyro_dps[3]  : gyro filtrado por eje                                     */
/*   throttle     : 0..1, solo para el reset del I-term                       */
/*   out[3]       : correccion normalizada -1..+1 por eje                     */
/* ------------------------------------------------------------------------- */
void pid_update(const float stick[PID_AXIS_COUNT],
                float yaw_sp_dps,
                const float angle_deg[2],
                const float gyro_dps[PID_AXIS_COUNT],
                float throttle,
                float out[PID_AXIS_COUNT]);

/* Diagnostico (consola / Live Expressions). */
float pid_iterm(uint8_t axis);
float pid_setpoint_dps(uint8_t axis);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_PID_H_ */
