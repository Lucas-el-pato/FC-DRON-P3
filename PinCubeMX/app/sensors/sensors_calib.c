/**
 ******************************************************************************
 * @file    sensors_calib.c
 * @brief   Calibracion de gyro y nivel del accel en el boot (ver
 *          sensors_calib.h).
 ******************************************************************************
 */

#include "sensors_calib.h"

#include <math.h>

#define CALIB_AXES  3u

/* Varianza incremental (Welford), equivalente a devPush() de Betaflight. */
typedef struct {
    uint32_t n;
    float    mean;
    float    m2;
} calib_dev_t;

static sensors_calib_state_t s_state = SENSORS_CALIB_IDLE;
static uint32_t s_target = SENSORS_CALIB_SAMPLES;
static uint32_t s_count = 0u;
static uint32_t s_restarts = 0u;

static int32_t     s_gyro_sum[CALIB_AXES];
static int32_t     s_acc_sum[CALIB_AXES];
static calib_dev_t s_gyro_dev[CALIB_AXES];
static float       s_gyro_stddev_dps[CALIB_AXES];

static int16_t s_gyro_zero[CALIB_AXES];
static int16_t s_acc_zero[CALIB_AXES];

static void dev_clear(calib_dev_t *d)
{
    d->n = 0u;
    d->mean = 0.0f;
    d->m2 = 0.0f;
}

static void dev_push(calib_dev_t *d, float x)
{
    d->n++;
    const float delta = x - d->mean;
    d->mean += delta / (float)d->n;
    d->m2 += delta * (x - d->mean);
}

static float dev_stddev(const calib_dev_t *d)
{
    return (d->n > 1u) ? sqrtf(d->m2 / (float)(d->n - 1u)) : 0.0f;
}

static int16_t round_div(int32_t sum, uint32_t n)
{
    const float v = (float)sum / (float)n;
    return (int16_t)lroundf(v);
}

static int16_t sat_s16(int32_t v)
{
    if (v > INT16_MAX) {
        return INT16_MAX;
    }
    if (v < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)v;
}

static void calib_reset_window(void)
{
    s_count = 0u;
    for (uint8_t i = 0u; i < CALIB_AXES; ++i) {
        s_gyro_sum[i] = 0;
        s_acc_sum[i] = 0;
        dev_clear(&s_gyro_dev[i]);
    }
}

void sensors_calib_start(uint32_t n_samples)
{
    s_target = (n_samples == 0u) ? SENSORS_CALIB_SAMPLES : n_samples;
    s_restarts = 0u;
    for (uint8_t i = 0u; i < CALIB_AXES; ++i) {
        s_gyro_zero[i] = 0;
        s_acc_zero[i] = 0;
        s_gyro_stddev_dps[i] = 0.0f;
    }
    calib_reset_window();
    s_state = SENSORS_CALIB_RUNNING;
}

bool sensors_calib_feed(const imu_sample_t *raw)
{
    if (raw == 0 || s_state != SENSORS_CALIB_RUNNING) {
        return false;
    }

    const int16_t g[CALIB_AXES] = { raw->gx, raw->gy, raw->gz };
    const int16_t a[CALIB_AXES] = { raw->ax, raw->ay, raw->az };

    for (uint8_t i = 0u; i < CALIB_AXES; ++i) {
        s_gyro_sum[i] += g[i];
        s_acc_sum[i] += a[i];
        dev_push(&s_gyro_dev[i], (float)g[i]);
    }

    if (++s_count < s_target) {
        return false;
    }

    /* Fin de ventana: si el gyro se movio demasiado, se descarta todo. */
    const float lsb_to_dps = SENSORS_IMU_GYRO_MDPS_PER_LSB * 0.001f;
    bool moved = false;
    for (uint8_t i = 0u; i < CALIB_AXES; ++i) {
        s_gyro_stddev_dps[i] = dev_stddev(&s_gyro_dev[i]) * lsb_to_dps;
        if (s_gyro_stddev_dps[i] > SENSORS_CALIB_GYRO_NOISE_DPS) {
            moved = true;
        }
    }
    if (moved) {
        s_restarts++;
        calib_reset_window();
        return false;
    }

    for (uint8_t i = 0u; i < CALIB_AXES; ++i) {
        s_gyro_zero[i] = round_div(s_gyro_sum[i], s_count);
        s_acc_zero[i] = round_div(s_acc_sum[i], s_count);
    }

    /* Z conserva 1 g con el signo que tenga el montaje. */
    const int16_t one_g = (int16_t)lroundf(SENSORS_CALIB_ACC_1G_LSB);
    s_acc_zero[2] = (s_acc_zero[2] >= 0) ? (int16_t)(s_acc_zero[2] - one_g)
                                         : (int16_t)(s_acc_zero[2] + one_g);

    s_state = SENSORS_CALIB_DONE;
    return true;
}

sensors_calib_state_t sensors_calib_state(void)
{
    return s_state;
}

bool sensors_calib_done(void)
{
    return s_state == SENSORS_CALIB_DONE;
}

uint32_t sensors_calib_restarts(void)
{
    return s_restarts;
}

float sensors_calib_gyro_stddev_dps(uint8_t axis)
{
    return (axis < CALIB_AXES) ? s_gyro_stddev_dps[axis] : 0.0f;
}

const int16_t *sensors_calib_gyro_zero(void)
{
    return s_gyro_zero;
}

const int16_t *sensors_calib_acc_zero(void)
{
    return s_acc_zero;
}

void sensors_calib_apply(imu_sample_t *raw, bool apply_acc)
{
    if (raw == 0) {
        return;
    }

    raw->gx = sat_s16((int32_t)raw->gx - s_gyro_zero[0]);
    raw->gy = sat_s16((int32_t)raw->gy - s_gyro_zero[1]);
    raw->gz = sat_s16((int32_t)raw->gz - s_gyro_zero[2]);

    if (apply_acc) {
        raw->ax = sat_s16((int32_t)raw->ax - s_acc_zero[0]);
        raw->ay = sat_s16((int32_t)raw->ay - s_acc_zero[1]);
        raw->az = sat_s16((int32_t)raw->az - s_acc_zero[2]);
    }
}
