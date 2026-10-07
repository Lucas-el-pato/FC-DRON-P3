/**
 ******************************************************************************
 * @file    sensors_calib.h
 * @brief   Calibracion de la IMU en el arranque (equivalente a
 *          performGyroCalibration de sensors/gyro.c y
 *          performAccelerationCalibration de sensors/acceleration_init.c de
 *          Betaflight).
 *
 *          Se ejecuta en cada boot, sin guardar nada en flash:
 *            - gyro: promedio de N muestras = bias por eje (gyroZero).
 *            - accel: la posicion de encendido es el nivel cero. X/Y se
 *              llevan a 0 y Z a 1 g (accZero).
 *
 *          Igual que Betaflight, si la desviacion estandar del gyro supera
 *          SENSORS_CALIB_GYRO_NOISE_DPS (el dron se movio) la calibracion se
 *          descarta y vuelve a empezar.
 *
 *          No bloquea: se alimenta muestra a muestra desde el lazo del gyro.
 *          Mientras no termina, el FC mantiene ARMING_DISABLED_CALIBRATING.
 *
 *          Los zeros quedan en raw (LSB) y se restan antes de
 *          sensors_imu_scale(), como gyro.ADC - gyroZero en Betaflight.
 ******************************************************************************
 */

#ifndef SENSORS_SENSORS_CALIB_H_
#define SENSORS_SENSORS_CALIB_H_

#include "driver_imu.h"
#include "sensors_scale.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Muestras de la ventana: 1.25 s a 8 kHz (gyro_calib_duration = 125 de BF). */
#define SENSORS_CALIB_SAMPLES        10000u

/* Desvio estandar maximo del gyro en reposo. BF: gyro_calib_noise_limit = 48
 * LSB a 16.4 LSB/dps (~2.9 dps). Por encima se reinicia la calibracion.   */
#define SENSORS_CALIB_GYRO_NOISE_DPS 3.0f

/* 1 g en LSB del accel (+/-4 g -> 0.122 mg/LSB). */
#define SENSORS_CALIB_ACC_1G_LSB     (1000.0f / SENSORS_IMU_ACC_MG_PER_LSB)

typedef enum {
    SENSORS_CALIB_IDLE = 0,
    SENSORS_CALIB_RUNNING,
    SENSORS_CALIB_DONE
} sensors_calib_state_t;

/* Arranca (o reinicia) la calibracion con n muestras (0 = default). */
void sensors_calib_start(uint32_t n_samples);

/* ------------------------------------------------------------------------- */
/* Agrega una muestra completa (gyro + accel, sin corregir).                  */
/* Devuelve true solo en la llamada en que la calibracion termina bien.      */
/* ------------------------------------------------------------------------- */
bool sensors_calib_feed(const imu_sample_t *raw);

sensors_calib_state_t sensors_calib_state(void);
bool sensors_calib_done(void);

/* Veces que se descarto la ventana por movimiento. */
uint32_t sensors_calib_restarts(void);

/* Ultimo desvio estandar medido por eje, en dps (diagnostico). */
float sensors_calib_gyro_stddev_dps(uint8_t axis);

/* Zeros en LSB (diagnostico / Live Expressions). */
const int16_t *sensors_calib_gyro_zero(void);
const int16_t *sensors_calib_acc_zero(void);

/* Resta los zeros a una muestra raw. apply_acc = false para lecturas de
 * solo gyro (imu_read_gyro no actualiza ax/ay/az).                        */
void sensors_calib_apply(imu_sample_t *raw, bool apply_acc);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_SENSORS_CALIB_H_ */
