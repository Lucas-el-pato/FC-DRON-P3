/**
 ******************************************************************************
 * @file    filter.c
 * @brief   Implementacion del PT1 (ver filter.h).
 ******************************************************************************
 */

#include "filter.h"

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
