/**
 ******************************************************************************
 * @file    rpm_filter.h
 * @brief   Filtro RPM (como rpm_filter.c de Betaflight, escrito de cero):
 *          un notch por motor y por armonico, centrado en la velocidad de
 *          giro medida de cada motor (eRPM del DShot bidireccional).
 *
 *          Por eje: 4 motores x RPM_FILTER_HARMONICS notches en serie
 *          (36 en total con 3 armonicos). Los coeficientes de cada
 *          (motor, armonico) son comunes a los 3 ejes; cada eje tiene su
 *          estado. Notch biquad en forma directa I: los coeficientes se
 *          pueden cambiar en cada paso sin transitorios grandes.
 *
 *          Frecuencias (Hz del rotor -> armonicos):
 *            - PT1 de RPM_FILTER_LPF_HZ sobre la velocidad de cada motor
 *              (la eRPM llega a la tasa del PID).
 *            - Debajo de RPM_FILTER_MIN_HZ el notch no filtra; entre MIN y
 *              MIN + RPM_FILTER_FADE_HZ entra de a poco (peso 0..1), asi
 *              no aparece de golpe al subir el throttle ni filtra el ruido
 *              de ralenti (con el dron apoyado no importa y agrega retardo).
 *            - Arriba de 0.48 x tasa de muestreo, sin filtrar.
 *
 *          Costo: 36 biquads por muestra de gyro + RPM_FILTER_UPDATES_PER_STEP
 *          recalculos de coeficientes (sinf/cosf) por muestra, repartidos.
 ******************************************************************************
 */

#ifndef CONTROL_RPM_FILTER_H_
#define CONTROL_RPM_FILTER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RPM_FILTER_MOTORS        4u
#define RPM_FILTER_AXES          3u

/* 1 = fundamental, 2 = +2x, 3 = +3x (paso de palas de la tribala). */
#ifndef RPM_FILTER_HARMONICS
#define RPM_FILTER_HARMONICS     3u
#endif
/* Q = centro / ancho de banda. BF: rpm_filter_q = 500 -> 5.0. */
#ifndef RPM_FILTER_Q
#define RPM_FILTER_Q             5.0f
#endif
/* BF trae 100 Hz + 50 de fundido (pensado para 5"). Este 6" gira en hover
 * a ~130 Hz (vuelo libre BB011): con 100 + 50 la fundamental quedaba a
 * medio filtrar. 80 + 30 = notch completo desde 110 Hz.                 */
#ifndef RPM_FILTER_MIN_HZ
#define RPM_FILTER_MIN_HZ        80.0f
#endif
#ifndef RPM_FILTER_FADE_HZ
#define RPM_FILTER_FADE_HZ       30.0f
#endif
/* BF: rpm_filter_lpf_hz = 150. */
#ifndef RPM_FILTER_LPF_HZ
#define RPM_FILTER_LPF_HZ        150.0f
#endif
/* Juegos de coeficientes recalculados por muestra de gyro. Con 12 juegos
 * (4 motores x 3 armonicos) y 2 por muestra, cada uno se renueva cada 6
 * muestras = 1.33 kHz a 8 kHz de gyro (la eRPM llega a 2 kHz y pasa por un
 * PT1 de 150 Hz: sobra). Con 3 la cadena llego a 118 de 125 us.        */
#ifndef RPM_FILTER_UPDATES_PER_STEP
#define RPM_FILTER_UPDATES_PER_STEP 2u
#endif

/* sample_hz: tasa a la que corre rpm_filter_apply (gyro, 8 kHz).
 * update_hz: tasa a la que llega rpm_filter_set_motor_hz (PID, 2 kHz). */
void rpm_filter_init(float sample_hz, float update_hz);

/* Velocidad de giro de cada rotor en Hz (0 = sin dato / parado). */
void rpm_filter_set_motor_hz(const float hz[RPM_FILTER_MOTORS]);

/* Una vez por muestra de gyro, antes de rpm_filter_apply: recalcula unos
 * pocos juegos de coeficientes (round robin).                          */
void rpm_filter_step(void);

/* Pasa una muestra de un eje por los notches de los 4 motores. */
float rpm_filter_apply(uint8_t axis, float input);

/* Fundamental promedio de los motores (Hz, ya filtrada) para el log. */
float rpm_filter_mean_hz(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_RPM_FILTER_H_ */
