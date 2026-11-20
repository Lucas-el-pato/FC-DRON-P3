/**
 ******************************************************************************
 * @file    filter.c
 * @brief   Implementacion del PT1 (ver filter.h).
 ******************************************************************************
 */

#include "filter.h"

#include <math.h>

#define FILTER_2PI  6.28318530718f

void pt1_init(pt1_filter_t *f, float cutoff_hz, float dt_s)
{
    if (f == 0) {
        return;
    }

    f->state = 0.0f;

    if (cutoff_hz <= 0.0f || dt_s <= 0.0f) {
        f->k = 1.0f;   /* sin filtrado */
        return;
    }

    const float rc = 1.0f / (FILTER_2PI * cutoff_hz);
    f->k = dt_s / (rc + dt_s);
}

void pt1_reset(pt1_filter_t *f, float value)
{
    if (f != 0) {
        f->state = value;
    }
}

void biquad_passthrough_coeff(biquad_coeff_t *c)
{
    if (c == 0) {
        return;
    }
    c->b0 = 1.0f;
    c->b1 = 0.0f;
    c->b2 = 0.0f;
    c->a1 = 0.0f;
    c->a2 = 0.0f;
}

void biquad_notch_coeff(biquad_coeff_t *c, float center_hz, float q, float dt_s)
{
    if (c == 0) {
        return;
    }
    /* Fuera de rango (o por encima de ~Nyquist) -> sin filtrar. */
    if (center_hz <= 0.0f || q <= 0.0f || dt_s <= 0.0f ||
        center_hz >= 0.48f / dt_s) {
        biquad_passthrough_coeff(c);
        return;
    }

    const float w0 = FILTER_2PI * center_hz * dt_s;
    const float cs = cosf(w0);
    const float alpha = sinf(w0) / (2.0f * q);
    const float inv_a0 = 1.0f / (1.0f + alpha);

    c->b0 = inv_a0;
    c->b1 = -2.0f * cs * inv_a0;
    c->b2 = inv_a0;
    c->a1 = c->b1;
    c->a2 = (1.0f - alpha) * inv_a0;
}

void biquad_reset(biquad_state_t *s, float value)
{
    if (s == 0) {
        return;
    }
    /* Estado estacionario con entrada constante = value (ganancia DC 1). */
    s->x1 = value;
    s->x2 = value;
    s->y1 = value;
    s->y2 = value;
}
