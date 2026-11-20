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

/* ------------------------------------------------------------------------- */
/* Notch biquad (RBJ), forma directa I como el notch dinamico de BF: los     */
/* coeficientes se pueden cambiar en cada paso sin transitorios grandes.     */
/* Coeficientes y estado separados: un juego de coeficientes sirve a los 3   */
/* ejes (misma frecuencia), cada eje tiene su propio estado.                 */
/* ------------------------------------------------------------------------- */
typedef struct {
    float b0, b1, b2;   /* normalizados por a0 */
    float a1, a2;
} biquad_coeff_t;

typedef struct {
    float x1, x2;
    float y1, y2;
} biquad_state_t;

/* Coeficientes de notch: centro center_hz, factor de calidad q (ancho de
 * banda = center_hz / q), periodo de muestreo dt_s.                       */
void biquad_notch_coeff(biquad_coeff_t *c, float center_hz, float q, float dt_s);

/* Coeficientes de paso directo (salida = entrada). */
void biquad_passthrough_coeff(biquad_coeff_t *c);

void biquad_reset(biquad_state_t *s, float value);

static inline float biquad_apply(const biquad_coeff_t *c, biquad_state_t *s,
                                 float input)
{
    const float y = c->b0 * input + c->b1 * s->x1 + c->b2 * s->x2
                  - c->a1 * s->y1 - c->a2 * s->y2;
    s->x2 = s->x1;
    s->x1 = input;
    s->y2 = s->y1;
    s->y1 = y;
    return y;
}

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_FILTER_H_ */
