/**
 ******************************************************************************
 * @file    bal_config.h
 * @brief   Parametros del balancin de 2 motores (M1 izquierda, M3 derecha).
 *
 *          Angulo del balancin: respecto de la horizontal real (gravedad),
 *          positivo = antihorario visto desde ATRAS del FC (mirando hacia el
 *          frente) = baja el lado izquierdo (M1) y sube el derecho (M3).
 *
 *          Todo es #ifndef: se puede pisar con -D desde CMake.
 ******************************************************************************
 */

#ifndef BALANCER_BAL_CONFIG_H_
#define BALANCER_BAL_CONFIG_H_

#include "driver_imu.h"

/* ------------------------------------------------------------------------- */
/* Lazo: gyro 8 kHz, control 2 kHz (igual que el FC del dron).               */
/* ------------------------------------------------------------------------- */
#define BAL_GYRO_RATE_HZ        IMU_ODR_HZ
#define BAL_PID_DENOM           4u
#define BAL_PID_RATE_HZ         (BAL_GYRO_RATE_HZ / BAL_PID_DENOM)
#define BAL_GYRO_LPF_HZ         150.0f
#define BAL_GYRO_LPF2_HZ        300.0f

/* ------------------------------------------------------------------------- */
/* Motores. Indice = canal TIM2 (0 = CH1 .. 3 = CH4), ver driver_motors.h:   */
/*   CH4 = M1 (izquierda, PA3)   CH2 = M3 (derecha, PB3)                     */
/*   CH1 = M4 y CH3 = M2 deshabilitados: siempre DShot 0.                    */
/* ------------------------------------------------------------------------- */
#define BAL_MOTOR_LEFT_CH       3u
#define BAL_MOTOR_RIGHT_CH      1u
#define BAL_MOTOR_MASK          ((1u << BAL_MOTOR_LEFT_CH) | (1u << BAL_MOTOR_RIGHT_CH))

/* Motores con sentido invertido (bit = canal). M3 (CH2) gira horario, como
 * MIXER_MOTOR_REVERSED_MASK del dron. Se aplica por DShot en cada armado.  */
#ifndef BAL_MOTOR_REVERSED_MASK
#define BAL_MOTOR_REVERSED_MASK 0x02u
#endif

/* Techo duro de cada motor (fraccion del rango DShot 48..2047). */
#ifndef BAL_MOTOR_LIMIT
#define BAL_MOTOR_LIMIT         0.20f
#endif

/* Minimo con el balancin armado (los motores nunca se paran en el aire). */
#ifndef BAL_IDLE
#define BAL_IDLE                0.04f
#endif

/* Throttle base comun al arrancar (se cambia por serial con "T <pct>"). */
#ifndef BAL_BASE_THROTTLE
#define BAL_BASE_THROTTLE       0.10f
#endif

/* Diferencial maximo que el PID puede sumar/restar a cada motor (la salida
 * del PID ya es fraccion de motor, como en el mixer del dron).            */
#ifndef BAL_CORR_LIMIT
#define BAL_CORR_LIMIT          0.08f
#endif

#define BAL_DSHOT_MIN           48u
#define BAL_DSHOT_MAX           2047u

/* ------------------------------------------------------------------------- */
/* Seguridad.                                                                 */
/* ------------------------------------------------------------------------- */
#ifndef BAL_REF_LIMIT_DEG
#define BAL_REF_LIMIT_DEG       30.0f     /* referencia maxima aceptada   */
#endif
#ifndef BAL_ARM_MAX_ANGLE_DEG
#define BAL_ARM_MAX_ANGLE_DEG   70.0f     /* arma apoyado en el tope fisico (~58 grados) */
#endif
#ifndef BAL_TILT_CUTOFF_DEG
#define BAL_TILT_CUTOFF_DEG     80.0f     /* por encima del tope: solo si algo se suelta */
#endif

/* ------------------------------------------------------------------------- */
/* Signos y montaje.                                                          */
/*   BAL_ROLL_SIGN: lleva el roll de la IMU al marco Betaflight (roll+ = lado */
/*                  derecho abajo). Mismo valor que FC_GYRO_ROLL_SIGN,        */
/*                  verificado en banco.                                      */
/*   BAL_CCW_SIGN : roll BF -> angulo antihorario visto desde atras.          */
/*                  Derecha abajo es horario, por eso -1.                     */
/*   BAL_ANGLE_TRIM_DEG: lo que marca "?" con el balancin nivelado con un     */
/*                  nivel de burbuja (IMU no montada perfectamente plana).    */
/* ------------------------------------------------------------------------- */
#ifndef BAL_ROLL_SIGN
#define BAL_ROLL_SIGN           (-1.0f)
#endif
#ifndef BAL_CCW_SIGN
#define BAL_CCW_SIGN            (-1.0f)
#endif
#ifndef BAL_ANGLE_TRIM_DEG
#define BAL_ANGLE_TRIM_DEG      0.0f
#endif

/* ------------------------------------------------------------------------- */
/* PID (misma ley y unidades que control/pid.h: ganancias "de configurador"  */
/* Betaflight escaladas por PID_PTERM/ITERM/DTERM_SCALE).                     */
/*   nivel: error de angulo * BAL_LEVEL_GAIN -> setpoint de rate (dps)        */
/*   rate : setpoint - gyro -> P + I + D -> suma / 1000 = fraccion de motor   */
/* Arranque conservador; afinar con el balancin atado.                       */
/* ------------------------------------------------------------------------- */
#ifndef BAL_KP
#define BAL_KP                  40.0f
#endif
#ifndef BAL_KI
#define BAL_KI                  60.0f
#endif
#ifndef BAL_KD
#define BAL_KD                  20.0f
#endif
#ifndef BAL_LEVEL_GAIN
#define BAL_LEVEL_GAIN          4.0f      /* dps por grado de error */
#endif
#ifndef BAL_LEVEL_MAX_RATE_DPS
#define BAL_LEVEL_MAX_RATE_DPS  120.0f
#endif
#ifndef BAL_ITERM_LIMIT
#define BAL_ITERM_LIMIT         50.0f     /* unidades BF (/1000 = motor) */
#endif
#ifndef BAL_SUM_LIMIT
#define BAL_SUM_LIMIT           (BAL_CORR_LIMIT * 1000.0f)
#endif
#ifndef BAL_DTERM_LPF1_HZ
#define BAL_DTERM_LPF1_HZ       75.0f
#endif
#ifndef BAL_DTERM_LPF2_HZ
#define BAL_DTERM_LPF2_HZ       150.0f
#endif

#endif /* BALANCER_BAL_CONFIG_H_ */
