/**
 ******************************************************************************
 * @file    filter.h
 * @brief   Filtros del lazo de control (equivalente a common/filter.c de
 *          Betaflight, recortado a lo que usa este FC).
 *
 *          PT1 = pasabajos de primer orden discreto:
 *            y[n] = y[n-1] + k * (x[n] - y[n-1])
 *            k = dt / (dt + 1/(2*pi*fc))
 *
 *          Se usa en dos lugares:
 *            - gyro filtrado antes del PID (ruido de motores/frame)
 *            - D-term (la derivada amplifica todo el ruido de arriba)
 ******************************************************************************
 */

#ifndef CONTROL_FILTER_H_
#define CONTROL_FILTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float state;
    float k;      /* ganancia del paso, 0..1 */
} pt1_filter_t;

/* Inicializa el filtro: cutoff_hz > 0, dt_s = periodo de muestreo. */
void pt1_init(pt1_filter_t *f, float cutoff_hz, float dt_s);

/* Reinicia el estado interno (al armar / desarmar). */
void pt1_reset(pt1_filter_t *f, float value);

/* Aplica un paso del filtro y devuelve la salida. */
static inline float pt1_apply(pt1_filter_t *f, float input)
{
    f->state += f->k * (input - f->state);
    return f->state;
}

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_FILTER_H_ */
