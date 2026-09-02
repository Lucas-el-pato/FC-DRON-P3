/**
 ******************************************************************************
 * @file    mixer.c
 * @brief   Implementacion de la mezcla quad-X (ver mixer.h).
 ******************************************************************************
 */

#include "mixer.h"

/* Filas: motor. Columnas: roll, pitch, yaw (el throttle es comun). */
static const float MIXER_QUAD_X[MIXER_MOTOR_COUNT][PID_AXIS_COUNT] = {
    { -1.0f,  1.0f, -1.0f },   /* M1 trasero derecho   */
    { -1.0f, -1.0f,  1.0f },   /* M2 delantero derecho */
    {  1.0f,  1.0f,  1.0f },   /* M3 trasero izquierdo */
    {  1.0f, -1.0f, -1.0f },   /* M4 delantero izq.    */
};

static uint16_t s_last_out[MIXER_MOTOR_COUNT];

static float clamp01(float v)
{
    if (v < 0.0f) {
        return 0.0f;
    }
    if (v > 1.0f) {
        return 1.0f;
    }
    return v;
}

void mixer_init(void)
{
    for (uint8_t i = 0u; i < MIXER_MOTOR_COUNT; ++i) {
        s_last_out[i] = 0u;
    }
}

void mixer_run(float throttle,
               const float pid_axis[PID_AXIS_COUNT],
               bool armed,
               uint16_t out[MIXER_MOTOR_COUNT])
{
    if (out == 0) {
        return;
    }

    if (!armed) {
        for (uint8_t i = 0u; i < MIXER_MOTOR_COUNT; ++i) {
            out[i] = 0u;
            s_last_out[i] = 0u;
        }
        return;
    }

    throttle = clamp01(throttle) * MIXER_THROTTLE_LIMIT;

    /* 1. Mezcla de los ejes (sin throttle todavia). */
    float mix[MIXER_MOTOR_COUNT];
    float mix_max = 0.0f;
    float mix_min = 0.0f;

    for (uint8_t i = 0u; i < MIXER_MOTOR_COUNT; ++i) {
        float m = 0.0f;
        if (pid_axis != 0) {
            for (uint8_t ax = 0u; ax < PID_AXIS_COUNT; ++ax) {
                m += MIXER_QUAD_X[i][ax] * pid_axis[ax];
            }
        }
        mix[i] = m;
        if (m > mix_max) {
            mix_max = m;
        }
        if (m < mix_min) {
            mix_min = m;
        }
    }

    /* 2. Si la mezcla no entra en el rango disponible, se comprime en vez de
     * recortar un motor solo (recortar rompe la relacion entre ejes).      */
    const float range = mix_max - mix_min;
    if (range > 1.0f) {
        const float scale = 1.0f / range;
        for (uint8_t i = 0u; i < MIXER_MOTOR_COUNT; ++i) {
            mix[i] *= scale;
        }
        mix_max *= scale;
        mix_min *= scale;
    }

    /* 3. Throttle desplazado para que ningun motor caiga por debajo del
     * ralenti ni supere el maximo (airmode off: prioridad al throttle).   */
    const float idle = MIXER_IDLE_THROTTLE;
    float thr = idle + throttle * (1.0f - idle);

    if (thr + mix_min < idle) {
        thr = idle - mix_min;
    }
    if (thr + mix_max > 1.0f) {
        thr = 1.0f - mix_max;
    }
    thr = clamp01(thr);

    /* 4. Escalado final a DShot. */
    for (uint8_t i = 0u; i < MIXER_MOTOR_COUNT; ++i) {
        const float value = clamp01(thr + mix[i]);
        uint32_t dshot = (uint32_t)MIXER_DSHOT_MIN +
                         (uint32_t)(value * (float)(MIXER_DSHOT_MAX - MIXER_DSHOT_MIN) + 0.5f);
        if (dshot > MIXER_DSHOT_MAX) {
            dshot = MIXER_DSHOT_MAX;
        }
        out[i] = (uint16_t)dshot;
        s_last_out[i] = out[i];
    }
}

const uint16_t *mixer_last_output(void)
{
    return s_last_out;
}
