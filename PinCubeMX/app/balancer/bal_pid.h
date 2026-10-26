/**
 ******************************************************************************
 * @file    bal_pid.h
 * @brief   PID de un eje para el balancin: misma cascada que control/pid.c
 *          (pidLevel + pidController de Betaflight) reducida al eje roll.
 *
 *            nivel: (ref - angulo) * BAL_LEVEL_GAIN -> setpoint dps
 *            rate : setpoint - gyro -> P + I + D (D sobre el gyro filtrado)
 *
 *          A diferencia del dron, el I-term integra siempre que se llame
 *          (el dron lo apaga con throttle < 20 %, y aca el techo es 20 %).
 *          Llamar solo armado; bal_pid_reset() al armar/desarmar.
 ******************************************************************************
 */

#ifndef BALANCER_BAL_PID_H_
#define BALANCER_BAL_PID_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bal_pid_init(float dt_s);
void bal_pid_reset(void);

/* ------------------------------------------------------------------------- */
/* Un paso del lazo.                                                          */
/*   ref_deg   : referencia de angulo (antihorario +)                         */
/*   angle_deg : angulo actual, mismo signo                                   */
/*   rate_dps  : gyro filtrado, mismo signo (antihorario +)                   */
/* Devuelve la correccion en fraccion de motor, +/-BAL_CORR_LIMIT.            */
/* Positivo = hay que girar antihorario = mas empuje a la derecha (M3).       */
/* ------------------------------------------------------------------------- */
float bal_pid_update(float ref_deg, float angle_deg, float rate_dps);

/* Diagnostico (unidades BF, /1000 = motor). */
float bal_pid_setpoint_dps(void);
float bal_pid_pterm(void);
float bal_pid_iterm(void);
float bal_pid_dterm(void);

#ifdef __cplusplus
}
#endif

#endif /* BALANCER_BAL_PID_H_ */
