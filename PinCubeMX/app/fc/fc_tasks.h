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

/* Segundo PT1 en cascada (gyro_lpf2 de BF): atenua mas la vibracion de los
 * motores (no hay filtro RPM ni notch dinamico).                           */
#ifndef FC_GYRO_LPF2_HZ
#define FC_GYRO_LPF2_HZ     300.0f
#endif

/* 1 = modo ANGLE (stick -> angulo -> PID -> mixer). 0 = passthrough: solo
 * throttle a los 4 motores; los sticks de roll/pitch/yaw no hacen nada.     */
#ifndef FC_ENABLE_PID
#define FC_ENABLE_PID       1
#endif

/* Estas dos bloquean el lazo (polling I2C de ~8 ms y UART con timeout), asi
 * que no se agendan en vuelo. Sirven en banco con el drone quieto.          */
#ifndef FC_ENABLE_MAG
#define FC_ENABLE_MAG       0
#endif
#ifndef FC_ENABLE_GPS
#define FC_ENABLE_GPS       0
#endif

/* 1 = el log USB sigue imprimiendo con el dron armado. SOLO en banco, sin
 * helices y con el host leyendo el CDC (si no, console_print puede bloquear
 * el lazo hasta 160 ms).                                                    */
#ifndef FC_LOG_WHEN_ARMED
#define FC_LOG_WHEN_ARMED   0
#endif

/* Telemetria CRSF hacia la radio (attitude / altitud / vario). */
#ifndef FC_ENABLE_TELEM_TX
#define FC_ENABLE_TELEM_TX  1
#endif

/* ------------------------------------------------------------------------- */
/* Mapeo de ejes IMU -> ejes de vuelo.                                        */
/* Marco de Betaflight (el que asume el mixer):                              */
/*   roll+  = ala derecha abajo                                               */
/*   pitch+ = nariz abajo                                                     */
/*   yaw+   = antihorario visto desde arriba                                  */
/* VERIFICAR SIN HELICES con el log: gyro y angulo deben dar positivo en esos */
/* sentidos. Si un eje sale invertido, cambiar su signo aca (se aplica al     */
/* gyro y al angulo). Un signo mal = realimentacion positiva = motores locos. */
/* ------------------------------------------------------------------------- */
#define FC_GYRO_ROLL_SIGN    (-1.0f)   /* verificado en banco: roll invertido */
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
