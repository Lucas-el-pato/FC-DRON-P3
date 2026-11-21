/**
 ******************************************************************************
 * @file    rpm_filter.c
 * @brief   Filtro RPM por motor y armonico (ver rpm_filter.h).
 ******************************************************************************
 */

#include "rpm_filter.h"
#include "filter.h"

#define RPM_FILTER_SETS  (RPM_FILTER_MOTORS * RPM_FILTER_HARMONICS)

typedef struct {
    biquad_coeff_t coeff;
    float          weight;     /* 0 = no filtra, 1 = notch completo */
} rpm_notch_t;

/* Indice de juego: motor * RPM_FILTER_HARMONICS + (armonico - 1). */
static rpm_notch_t    s_notch[RPM_FILTER_SETS];
static biquad_state_t s_state[RPM_FILTER_AXES][RPM_FILTER_SETS];
static pt1_filter_t   s_motor_lpf[RPM_FILTER_MOTORS];
static float          s_motor_hz[RPM_FILTER_MOTORS];   /* ya filtrada */
static float          s_dt = 0.000125f;
static float          s_max_hz = 3840.0f;
static uint8_t        s_next = 0u;                      /* round robin */
static bool           s_inited = false;

static void rpm_filter_update_set(uint8_t set)
{
    const uint8_t motor = (uint8_t)(set / RPM_FILTER_HARMONICS);
    const float harmonic = (float)((set % RPM_FILTER_HARMONICS) + 1u);
    float hz = s_motor_hz[motor] * harmonic;

    /* Peso: 0 debajo del minimo, rampa en la zona de fundido. */
    float weight = 1.0f;
    if (hz < (RPM_FILTER_MIN_HZ + RPM_FILTER_FADE_HZ)) {
        weight = (hz - RPM_FILTER_MIN_HZ) / RPM_FILTER_FADE_HZ;
        if (weight < 0.0f) {
            weight = 0.0f;
        }
    }

    /* El centro no baja del minimo: con peso 0 igual sigue corriendo y
     * asi el notch no salta de 0 Hz a la frecuencia real al entrar.    */
    if (hz < RPM_FILTER_MIN_HZ) {
        hz = RPM_FILTER_MIN_HZ;
    }
    if (hz > s_max_hz) {
        hz = s_max_hz;
        weight = 0.0f;
    }

    biquad_notch_coeff(&s_notch[set].coeff, hz, RPM_FILTER_Q, s_dt);
    s_notch[set].weight = weight;
}

void rpm_filter_init(float sample_hz, float update_hz)
{
    if ((sample_hz <= 0.0f) || (update_hz <= 0.0f)) {
        s_inited = false;
        return;
    }
    s_dt = 1.0f / sample_hz;
    s_max_hz = 0.48f * sample_hz;
    s_next = 0u;

    for (uint8_t m = 0u; m < RPM_FILTER_MOTORS; ++m) {
        pt1_init(&s_motor_lpf[m], RPM_FILTER_LPF_HZ, 1.0f / update_hz);
        pt1_reset(&s_motor_lpf[m], 0.0f);
        s_motor_hz[m] = 0.0f;
    }
    for (uint8_t set = 0u; set < RPM_FILTER_SETS; ++set) {
        rpm_filter_update_set(set);   /* minimo y peso 0 */
        for (uint8_t ax = 0u; ax < RPM_FILTER_AXES; ++ax) {
            biquad_reset(&s_state[ax][set], 0.0f);
        }
    }
    s_inited = true;
}

void rpm_filter_set_motor_hz(const float hz[RPM_FILTER_MOTORS])
{
    if (!s_inited || (hz == 0)) {
        return;
    }
    for (uint8_t m = 0u; m < RPM_FILTER_MOTORS; ++m) {
        const float in = (hz[m] > 0.0f) ? hz[m] : 0.0f;
        s_motor_hz[m] = pt1_apply(&s_motor_lpf[m], in);
    }
}

void rpm_filter_step(void)
{
    if (!s_inited) {
        return;
    }
    for (uint8_t i = 0u; i < RPM_FILTER_UPDATES_PER_STEP; ++i) {
        rpm_filter_update_set(s_next);
        s_next++;
        if (s_next >= RPM_FILTER_SETS) {
            s_next = 0u;
        }
    }
}

float rpm_filter_apply(uint8_t axis, float input)
{
    if (!s_inited || (axis >= RPM_FILTER_AXES)) {
        return input;
    }

    float x = input;
    biquad_state_t *st = s_state[axis];
    for (uint8_t set = 0u; set < RPM_FILTER_SETS; ++set) {
        /* Siempre se calcula (mantiene el estado al dia aunque pese 0). */
        const float y = biquad_apply(&s_notch[set].coeff, &st[set], x);
        x += s_notch[set].weight * (y - x);
    }
    return x;
}

float rpm_filter_mean_hz(void)
{
    float sum = 0.0f;
    for (uint8_t m = 0u; m < RPM_FILTER_MOTORS; ++m) {
        sum += s_motor_hz[m];
    }
    return sum / (float)RPM_FILTER_MOTORS;
}
