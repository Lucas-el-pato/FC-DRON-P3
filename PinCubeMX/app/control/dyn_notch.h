/**
 ******************************************************************************
 * @file    dyn_notch.h
 * @brief   Notch dinamico (como dyn_notch_filter.c de Betaflight 4.3+,
 *          escrito de cero): busca en el gyro el pico de ruido que queda
 *          despues del filtro RPM (resonancias del frame o del banco, que no
 *          siguen a los motores) y le pone un notch, uno por eje.
 *
 *          Analisis: SDFT (transformada de Fourier deslizante) del gyro
 *          promediado de a DYN_NOTCH_DECIM muestras (8 kHz / 4 = 2 kHz), con
 *          DYN_NOTCH_N puntos: resolucion 2000 / 128 = 15.6 Hz. Solo se
 *          calculan los bins entre DYN_NOTCH_MIN_HZ y DYN_NOTCH_MAX_HZ.
 *          Ventana de Hann aplicada en frecuencia (bins vecinos).
 *
 *          Pico: el bin local mas alto del rango, interpolado (parabola)
 *          entre vecinos. Se usa solo si su potencia supera
 *          DYN_NOTCH_THRESHOLD veces la potencia media del rango; si no hay
 *          pico valido durante DYN_NOTCH_HOLD_S el notch se apaga. Para
 *          encenderlo, el pico tiene que sostenerse DYN_NOTCH_CONFIRM_S en
 *          la misma frecuencia (asi el ruido blanco no lo dispara).
 *          Encendido, sigue al pico mientras se mueva menos de 2 bins por
 *          analisis (PT1 de DYN_NOTCH_SMOOTH_HZ sobre el centro); un pico
 *          en otra frecuencia tiene que sostenerse DYN_NOTCH_CONFIRM_S para
 *          que el notch salte ahi.
 *
 *          El analisis recibe el gyro ANTES de este notch (si no, al bajar el
 *          pico lo perderia y el notch oscilaria).
 *
 *          Costo: 3 biquads por muestra de gyro; una vez por periodo de PID
 *          (dyn_notch_update) la SDFT de los 3 ejes y el analisis de 1 eje.
 ******************************************************************************
 */

#ifndef CONTROL_DYN_NOTCH_H_
#define CONTROL_DYN_NOTCH_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DYN_NOTCH_AXES           3u

#ifndef DYN_NOTCH_N
#define DYN_NOTCH_N              128u
#endif
/* BF: dyn_notch_min_hz 100 / max 600. 80 por el 6" (resonancia de pitch
 * del banco en ~115 Hz, ver blackbox del filtro RPM).                   */
#ifndef DYN_NOTCH_MIN_HZ
#define DYN_NOTCH_MIN_HZ         80.0f
#endif
#ifndef DYN_NOTCH_MAX_HZ
#define DYN_NOTCH_MAX_HZ         600.0f
#endif
/* BF: dyn_notch_q = 300 -> 3.0. */
#ifndef DYN_NOTCH_Q
#define DYN_NOTCH_Q              3.0f
#endif
/* Potencia del pico / potencia media del rango para aceptarlo. */
#ifndef DYN_NOTCH_THRESHOLD
#define DYN_NOTCH_THRESHOLD      8.0f
#endif
/* Para ENCENDER el notch el pico tiene que mantenerse en la misma
 * frecuencia (+-2 bins) durante este tiempo: un pico de ruido dura lo que
 * la ventana de la SDFT (64 ms) y se mueve; una resonancia se queda.     */
#ifndef DYN_NOTCH_CONFIRM_S
#define DYN_NOTCH_CONFIRM_S      0.1f
#endif
#ifndef DYN_NOTCH_SMOOTH_HZ
#define DYN_NOTCH_SMOOTH_HZ      15.0f
#endif
/* Sin pico valido durante este tiempo -> notch apagado. */
#ifndef DYN_NOTCH_HOLD_S
#define DYN_NOTCH_HOLD_S         1.0f
#endif

/* sample_hz: tasa del gyro (8 kHz). decim: muestras promediadas por punto
 * de la SDFT (= cada cuantas muestras se llama dyn_notch_update).        */
void dyn_notch_init(float sample_hz, uint8_t decim);

/* Cada muestra de gyro, con el gyro de los 3 ejes ANTES de este notch. */
void dyn_notch_accumulate(const float x[DYN_NOTCH_AXES]);

/* Cada decim muestras: empuja el promedio a la SDFT, analiza un eje (round
 * robin) y recalcula su notch.                                           */
void dyn_notch_update(void);

/* Aplica el notch del eje (pasa directo si no hay pico activo). */
float dyn_notch_apply(uint8_t axis, float input);

/* Centro del notch del eje en Hz; 0 = apagado. */
float dyn_notch_center_hz(uint8_t axis);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_DYN_NOTCH_H_ */
