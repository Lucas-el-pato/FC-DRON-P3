/**
 ******************************************************************************
 * @file    fc_tasks.h
 * @brief   Tabla de tareas del FC y etapas del lazo realtime (equivalente a
 *          fc/tasks.c de Betaflight).
 *
 *          Etapas realtime (disparadas por el DRDY del gyro, 8 kHz):
 *            gyro   -> imu_read_gyro + escalado
 *            filter -> PT1 sobre los 3 ejes (cada FC_FILTER_DENOM muestras)
 *            pid    -> RC -> arming -> PID -> mixer -> motors_write4
 *                      (cada FC_PID_DENOM muestras: 8 kHz / 4 = 2 kHz)
 *
 *          Cola lenta (una por pasada, elegida por el scheduler):
 *            RX CRSF, actitud, baro, telemetria ESC, telemetria CRSF, log.
 *
 *          Tareas que bloquean demasiado para el lazo quedan apagadas por
 *          defecto (ver FC_ENABLE_MAG / FC_ENABLE_GPS abajo).
 ******************************************************************************
 */

#ifndef FC_FC_TASKS_H_
#define FC_FC_TASKS_H_

#include "scheduler.h"
#include "driver_imu.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Configuracion del lazo                                                     */
/* ------------------------------------------------------------------------- */

/* ODR del gyro (driver_imu lo configura en 8 kHz). */
#define FC_GYRO_RATE_HZ     IMU_ODR_HZ

/* Divisores del lazo: filtro a 8 kHz, PID/motores a 2 kHz. */
#define FC_FILTER_DENOM     1u
#define FC_PID_DENOM        4u
#define FC_PID_RATE_HZ      (FC_GYRO_RATE_HZ / FC_PID_DENOM)

/* Cutoff del pasabajos del gyro que entra al PID. */
#define FC_GYRO_LPF_HZ      150.0f

/* Etapa 1 = passthrough: el stick de throttle va directo a los 4 motores.
 * Poner en 1 recien despues de verificar sentido de giro y orden de motores. */
#ifndef FC_ENABLE_PID
#define FC_ENABLE_PID       0
#endif

/* Estas dos bloquean el lazo (polling I2C de ~8 ms y UART con timeout), asi
 * que no se agendan en vuelo. Sirven en banco con el drone quieto.          */
#ifndef FC_ENABLE_MAG
#define FC_ENABLE_MAG       0
#endif
#ifndef FC_ENABLE_GPS
#define FC_ENABLE_GPS       0
#endif

/* Telemetria CRSF hacia la radio (attitude / altitud / vario). */
#ifndef FC_ENABLE_TELEM_TX
#define FC_ENABLE_TELEM_TX  1
#endif

/* ------------------------------------------------------------------------- */
/* Mapeo de ejes IMU -> ejes de vuelo.                                        */
/* VERIFICAR con el montaje real: mover el drone en roll y confirmar el signo */
/* en el log (gx debe crecer al rolar a la derecha).                          */
/* ------------------------------------------------------------------------- */
#define FC_GYRO_ROLL_SIGN    (+1.0f)
#define FC_GYRO_PITCH_SIGN   (+1.0f)
#define FC_GYRO_YAW_SIGN     (+1.0f)

/* Prepara filtros, PID, mixer, arming, failsafe y la tabla de tareas.
 * has_gyro/has_baro/has_mag vienen del init de fc.c.                        */
void fc_tasks_init(void);

/* Config realtime + tabla, para pasarle a scheduler_init(). */
const sched_realtime_t *fc_tasks_realtime(void);
sched_task_t *fc_tasks_table(uint8_t *count);

/* Etapas expuestas para tests de banco. */
void fc_task_gyro(void);
void fc_task_filter(void);
void fc_task_pid(void);
void fc_task_rx(void);
void fc_task_failsafe(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_FC_TASKS_H_ */
