/**
 ******************************************************************************
 * @file    balancer.h
 * @brief   Balancin de 2 motores (M1 izquierda, M3 derecha) controlado desde
 *          la PC por USB CDC. Se compila con la preset CMake "Balancin"
 *          (APP_BALANCER=1); main.c llama bal_run() en lugar de fc_run().
 *
 *          Reusa el stack del dron: IMU + calibracion de gyro, scheduler del
 *          lazo de 8 kHz, filtros PT1, DShot300. Sin RC ni telemetria.
 *
 *          Boot: calibra el gyro (~1.25 s, balancin quieto) y queda
 *          DESARMADO con referencia 0 grados (horizonte real). La PC arma
 *          con "A"; ver bal_link.h para los comandos.
 *
 *          Desarma solo si: comando "D", se cierra el puerto COM, el angulo
 *          pasa BAL_TILT_CUTOFF_DEG o la IMU deja de responder.
 ******************************************************************************
 */

#ifndef BALANCER_BALANCER_H_
#define BALANCER_BALANCER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Init + scheduler. No retorna. */
void bal_run(void);

#ifdef __cplusplus
}
#endif

#endif /* BALANCER_BALANCER_H_ */
