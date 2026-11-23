/**
 ******************************************************************************
 * @file    dyn_notch.c
 * @brief   Notch dinamico por eje con SDFT (ver dyn_notch.h).
 *
 *          SDFT con amortiguacion r (estable con float):
 *            X_k[n] = r * e^{j 2 pi k / N} * (X_k[n-1] + x[n] - r^N x[n-N])
 *          Equivale a la DFT de las ultimas N muestras con la ventana
 *          empezando en la mas vieja, asi que la ventana de Hann se aplica
 *          con los vecinos: Xw[k] = 0.5 X[k] - 0.25 (X[k-1] + X[k+1]).
 ******************************************************************************
 */

#include "dyn_notch.h"
#include "filter.h"

#include <math.h>

#define DN_R          0.9999f
#define DN_MAX_BINS   ((DYN_NOTCH_N / 2u) + 1u)
#define DN_2PI        6.28318530718f

static float    s_dt = 0.000125f;       /* periodo del gyro               */
static float    s_bin_hz = 15.625f;
static float    s_r_pow_n = 1.0f;
static uint16_t s_kmin = 0u, s_kmax = 0u;   /* rango de busqueda          */
static uint16_t s_klo = 0u, s_nbins = 0u;   /* bins calculados (con vecinos) */

static float    s_tw_re[DN_MAX_BINS], s_tw_im[DN_MAX_BINS];
static float    s_re[DYN_NOTCH_AXES][DN_MAX_BINS];
static float    s_im[DYN_NOTCH_AXES][DN_MAX_BINS];
static float    s_ring[DYN_NOTCH_AXES][DYN_NOTCH_N];
static uint16_t s_idx = 0u;
static uint16_t s_filled = 0u;

static float    s_acc[DYN_NOTCH_AXES];
static uint8_t  s_acc_n = 0u;
static uint8_t  s_axis_next = 0u;

static biquad_coeff_t s_coeff[DYN_NOTCH_AXES];
static biquad_state_t s_state[DYN_NOTCH_AXES];
static pt1_filter_t   s_center_lpf[DYN_NOTCH_AXES];
static float          s_center[DYN_NOTCH_AXES];
static bool           s_active[DYN_NOTCH_AXES];
static uint32_t       s_miss[DYN_NOTCH_AXES];
static uint32_t       s_hits[DYN_NOTCH_AXES];   /* detecciones seguidas (apagado) */
static float          s_cand[DYN_NOTCH_AXES];   /* frecuencia candidata        */
static uint32_t       s_hold_updates = 1u;
static uint32_t       s_confirm_updates = 1u;
static bool           s_inited = false;

void dyn_notch_init(float sample_hz, uint8_t decim)
{
    s_inited = false;
    if ((sample_hz <= 0.0f) || (decim == 0u)) {
        return;
    }

    s_dt = 1.0f / sample_hz;
    const float fs = sample_hz / (float)decim;           /* tasa de la SDFT */
    s_bin_hz = fs / (float)DYN_NOTCH_N;

    uint32_t kmin = (uint32_t)(DYN_NOTCH_MIN_HZ / s_bin_hz);
    uint32_t kmax = (uint32_t)((DYN_NOTCH_MAX_HZ / s_bin_hz) + 0.999f);
    if (kmin < 2u) {
        kmin = 2u;
    }
    if (kmax > ((DYN_NOTCH_N / 2u) - 2u)) {
        kmax = (DYN_NOTCH_N / 2u) - 2u;
    }
    if (kmax <= kmin) {
        return;
    }
    s_kmin = (uint16_t)kmin;
    s_kmax = (uint16_t)kmax;
    s_klo = (uint16_t)(kmin - 1u);
    s_nbins = (uint16_t)((kmax + 1u) - s_klo + 1u);

    for (uint16_t i = 0u; i < s_nbins; ++i) {
        const float w = DN_2PI * (float)(s_klo + i) / (float)DYN_NOTCH_N;
        s_tw_re[i] = DN_R * cosf(w);
        s_tw_im[i] = DN_R * sinf(w);
    }
    s_r_pow_n = powf(DN_R, (float)DYN_NOTCH_N);

    for (uint8_t a = 0u; a < DYN_NOTCH_AXES; ++a) {
        for (uint16_t i = 0u; i < DN_MAX_BINS; ++i) {
            s_re[a][i] = 0.0f;
            s_im[a][i] = 0.0f;
        }
        for (uint16_t n = 0u; n < DYN_NOTCH_N; ++n) {
            s_ring[a][n] = 0.0f;
        }
        s_acc[a] = 0.0f;
        biquad_passthrough_coeff(&s_coeff[a]);
        biquad_reset(&s_state[a], 0.0f);
        /* Cada eje se analiza 1 de cada 3 actualizaciones de la SDFT. */
        pt1_init(&s_center_lpf[a], DYN_NOTCH_SMOOTH_HZ, (float)DYN_NOTCH_AXES / fs);
        s_center[a] = 0.0f;
        s_active[a] = false;
        s_miss[a] = 0u;
        s_hits[a] = 0u;
        s_cand[a] = 0.0f;
    }
    s_hold_updates = (uint32_t)(DYN_NOTCH_HOLD_S * fs / (float)DYN_NOTCH_AXES);
    if (s_hold_updates == 0u) {
        s_hold_updates = 1u;
    }
    s_confirm_updates = (uint32_t)(DYN_NOTCH_CONFIRM_S * fs / (float)DYN_NOTCH_AXES);
    if (s_confirm_updates == 0u) {
        s_confirm_updates = 1u;
    }
    s_idx = 0u;
    s_filled = 0u;
    s_acc_n = 0u;
    s_axis_next = 0u;
    s_inited = true;
}

void dyn_notch_accumulate(const float x[DYN_NOTCH_AXES])
{
    if (!s_inited) {
        return;
    }
    for (uint8_t a = 0u; a < DYN_NOTCH_AXES; ++a) {
        s_acc[a] += x[a];
    }
    s_acc_n++;
}

static void dyn_notch_off(uint8_t a)
{
    s_active[a] = false;
    s_hits[a] = 0u;
    s_center[a] = 0.0f;
    biquad_passthrough_coeff(&s_coeff[a]);
}

/* Busca el pico del eje y mueve su notch. */
static void dyn_notch_analyse(uint8_t a)
{
    float p[DN_MAX_BINS];
    float sum = 0.0f;
    const float *re = s_re[a];
    const float *im = s_im[a];

    /* Potencia con Hann para kmin..kmax (indices i = k - klo). */
    for (uint16_t k = s_kmin; k <= s_kmax; ++k) {
        const uint16_t i = (uint16_t)(k - s_klo);
        const float wr = (0.5f * re[i]) - (0.25f * (re[i - 1u] + re[i + 1u]));
        const float wi = (0.5f * im[i]) - (0.25f * (im[i - 1u] + im[i + 1u]));
        p[i] = (wr * wr) + (wi * wi);
        sum += p[i];
    }
    const float mean = sum / (float)((s_kmax - s_kmin) + 1u);

    /* Maximo local mas alto del rango (los bordes cuentan como maximo). */
    int32_t best = -1;
    float best_p = 0.0f;
    for (uint16_t k = s_kmin; k <= s_kmax; ++k) {
        const uint16_t i = (uint16_t)(k - s_klo);
        const bool above_left = (k == s_kmin) || (p[i] > p[i - 1u]);
        const bool above_right = (k == s_kmax) || (p[i] >= p[i + 1u]);
        if (above_left && above_right && (p[i] > best_p)) {
            best_p = p[i];
            best = (int32_t)k;
        }
    }

    if ((best < 0) || (best_p < (DYN_NOTCH_THRESHOLD * mean))) {
        if (!s_active[a]) {
            s_hits[a] = 0u;
        } else if (++s_miss[a] > s_hold_updates) {
            dyn_notch_off(a);
        }
        return;
    }

    /* Interpolacion parabolica sobre las magnitudes vecinas. */
    const uint16_t k = (uint16_t)best;
    const uint16_t i = (uint16_t)(k - s_klo);
    const float mb = sqrtf(p[i]);
    const float ml = (k > s_kmin) ? sqrtf(p[i - 1u]) : mb;
    const float mr = (k < s_kmax) ? sqrtf(p[i + 1u]) : mb;
    const float den = ml - (2.0f * mb) + mr;
    float delta = 0.0f;
    if (den < -1e-12f) {
        delta = 0.5f * (ml - mr) / den;
        if (delta > 0.5f) {
            delta = 0.5f;
        } else if (delta < -0.5f) {
            delta = -0.5f;
        }
    }
    float hz = ((float)k + delta) * s_bin_hz;
    if (hz < DYN_NOTCH_MIN_HZ) {
        hz = DYN_NOTCH_MIN_HZ;
    } else if (hz > DYN_NOTCH_MAX_HZ) {
        hz = DYN_NOTCH_MAX_HZ;
    }

    if (!s_active[a]) {
        /* Apagado: el pico tiene que quedarse en la misma frecuencia
         * DYN_NOTCH_CONFIRM_S antes de encender el notch.              */
        const float tol = 2.0f * s_bin_hz;
        if ((s_hits[a] == 0u) || (fabsf(hz - s_cand[a]) > tol)) {
            pt1_reset(&s_center_lpf[a], hz);
            s_cand[a] = hz;
            s_hits[a] = 1u;
            return;
        }
        s_cand[a] = pt1_apply(&s_center_lpf[a], hz);
        if (++s_hits[a] < s_confirm_updates) {
            return;
        }
        s_active[a] = true;
        s_miss[a] = 0u;
        s_hits[a] = 0u;
        s_center[a] = s_cand[a];
    } else {
        s_miss[a] = 0u;
        const float tol = 2.0f * s_bin_hz;
        if (fabsf(hz - s_center[a]) <= tol) {
            /* Mismo pico: lo sigue con el PT1. */
            s_hits[a] = 0u;
            s_center[a] = pt1_apply(&s_center_lpf[a], hz);
        } else {
            /* Pico en otra frecuencia: el notch no se mueve hasta que el
             * nuevo pico se sostenga DYN_NOTCH_CONFIRM_S (en vuelo libre el
             * de roll saltaba 150-500 Hz, ~8 veces por segundo, siguiendo
             * ruido ancho que no es una resonancia).                     */
            if ((s_hits[a] == 0u) || (fabsf(hz - s_cand[a]) > tol)) {
                s_cand[a] = hz;
                s_hits[a] = 1u;
                return;
            }
            s_cand[a] = 0.5f * (s_cand[a] + hz);
            if (++s_hits[a] < s_confirm_updates) {
                return;
            }
            s_hits[a] = 0u;
            pt1_reset(&s_center_lpf[a], s_cand[a]);
            s_center[a] = s_cand[a];
        }
    }
    biquad_notch_coeff(&s_coeff[a], s_center[a], DYN_NOTCH_Q, s_dt);
}

void dyn_notch_update(void)
{
    if (!s_inited || (s_acc_n == 0u)) {
        return;
    }

    const float inv = 1.0f / (float)s_acc_n;
    for (uint8_t a = 0u; a < DYN_NOTCH_AXES; ++a) {
        const float x = s_acc[a] * inv;
        s_acc[a] = 0.0f;
        const float delta = x - (s_r_pow_n * s_ring[a][s_idx]);
        s_ring[a][s_idx] = x;

        float *re = s_re[a];
        float *im = s_im[a];
        for (uint16_t i = 0u; i < s_nbins; ++i) {
            const float r0 = re[i] + delta;
            const float i0 = im[i];
            re[i] = (s_tw_re[i] * r0) - (s_tw_im[i] * i0);
            im[i] = (s_tw_re[i] * i0) + (s_tw_im[i] * r0);
        }
    }
    s_acc_n = 0u;
    s_idx++;
    if (s_idx >= DYN_NOTCH_N) {
        s_idx = 0u;
    }
    if (s_filled < DYN_NOTCH_N) {
        s_filled++;
        return;   /* ventana todavia incompleta */
    }

    dyn_notch_analyse(s_axis_next);
    s_axis_next++;
    if (s_axis_next >= DYN_NOTCH_AXES) {
        s_axis_next = 0u;
    }
}

float dyn_notch_apply(uint8_t axis, float input)
{
    if (!s_inited || (axis >= DYN_NOTCH_AXES)) {
        return input;
    }
    /* Apagado = coeficientes de paso directo; el estado sigue al dia. */
    return biquad_apply(&s_coeff[axis], &s_state[axis], input);
}

float dyn_notch_center_hz(uint8_t axis)
{
    return (axis < DYN_NOTCH_AXES) ? s_center[axis] : 0.0f;
}
