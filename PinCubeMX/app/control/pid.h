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

/* Ganancias en unidades Betaflight (defaults de BF 4.4), escaladas por
 * PID_GAIN_SCALE.                                                           */
#define PID_ROLL_KP         45.0f
#define PID_ROLL_KI         80.0f
#define PID_ROLL_KD         40.0f

#define PID_PITCH_KP        47.0f
#define PID_PITCH_KI        84.0f
#define PID_PITCH_KD        46.0f

#define PID_YAW_KP          45.0f
#define PID_YAW_KI          80.0f
#define PID_YAW_KD          0.0f

/* Multiplicador maestro sobre P, I y D (simplified_master_multiplier de BF).
 * 0.5 para las primeras pruebas; subir de a 0.1 una vez que vuela estable. */
#ifndef PID_GAIN_SCALE
#define PID_GAIN_SCALE      0.5f
#endif

/* Limites (iterm_windup / pidsum_limit / pidsum_limit_yaw de BF).
 * Valores de primeras pruebas: la suma por eje mueve como mucho el 25 % del
 * rango del motor (BF usa 500 = 50 %).                                     */
#ifndef PID_SUM_LIMIT
#define PID_SUM_LIMIT       250.0f
#endif
#ifndef PID_SUM_LIMIT_YAW
#define PID_SUM_LIMIT_YAW   200.0f
#endif
#ifndef PID_ITERM_LIMIT
#define PID_ITERM_LIMIT     150.0f
#endif

/* Modo ANGLE: angulo maximo con el stick al tope y lazo de nivel
 * (angle_p_gain = 50 de BF -> levelGain = 50 / 10 = 5 dps por grado de
 * error, constante de tiempo 1/5 = 0.2 s). La velocidad de correccion se
 * recorta a PID_LEVEL_MAX_RATE_DPS.                                       */
#ifndef PID_ANGLE_LIMIT_DEG
#define PID_ANGLE_LIMIT_DEG     30.0f
#endif
#ifndef PID_LEVEL_GAIN
#define PID_LEVEL_GAIN          5.0f
#endif
#ifndef PID_LEVEL_MAX_RATE_DPS
#define PID_LEVEL_MAX_RATE_DPS  150.0f
#endif

/* Debajo de este throttle el I-term se mantiene en 0 (como pidResetIterm()
 * de BF sin airmode): apoyado en el piso no se carga contra el suelo.     */
#ifndef PID_ITERM_RELAX_THROTTLE
#define PID_ITERM_RELAX_THROTTLE  0.20f
#endif

/* Multiplicador extra solo para D (simplified_d_gain de BF), encima de
 * PID_GAIN_SCALE. El D amplifica el ruido de los motores: arrancar bajo y
 * subir en vuelo atado mientras los motores no se calienten.              */
#ifndef PID_D_SCALE
#define PID_D_SCALE         0.5f
#endif

/* Dos PT1 en cascada sobre el gyro que alimenta al D-term (dterm_lpf1 +
 * dterm_lpf2 de BF). Sin filtro RPM ni notch dinamico, el D necesita este
 * filtrado para no convertir la vibracion en zumbido.                      */
#ifndef PID_DTERM_LPF1_HZ
#define PID_DTERM_LPF1_HZ   75.0f
#endif
#ifndef PID_DTERM_LPF2_HZ
#define PID_DTERM_LPF2_HZ   150.0f
#endif

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
float pid_dterm(uint8_t axis);   /* ultimo D, unidades BF (/1000 = motor) */

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_PID_H_ */
