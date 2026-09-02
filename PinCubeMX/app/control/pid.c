/**
 ******************************************************************************
 * @file    pid.c
 * @brief   Implementacion del PID de rate (ver pid.h).
 ******************************************************************************
 */

#include "pid.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float iterm;
    float prev_gyro;      /* gyro filtrado de la iteracion anterior */
    pt1_filter_t dterm_lpf;
} pid_axis_t;

static pid_axis_t s_axis[PID_AXIS_COUNT];
static float s_dt = 1.0f / 2000.0f;
static float s_inv_dt = 2000.0f;

static float clampf(float v, float lim)
{
    if (v > lim) {
        return lim;
    }
    if (v < -lim) {
        return -lim;
    }
    return v;
}

void pid_init(float dt_s)
{
    if (dt_s <= 0.0f) {
        dt_s = 1.0f / 2000.0f;
    }
    s_dt = dt_s;
    s_inv_dt = 1.0f / dt_s;

    s_axis[PID_AXIS_ROLL].kp  = PID_ROLL_KP;
    s_axis[PID_AXIS_ROLL].ki  = PID_ROLL_KI;
    s_axis[PID_AXIS_ROLL].kd  = PID_ROLL_KD;

    s_axis[PID_AXIS_PITCH].kp = PID_PITCH_KP;
    s_axis[PID_AXIS_PITCH].ki = PID_PITCH_KI;
    s_axis[PID_AXIS_PITCH].kd = PID_PITCH_KD;

    s_axis[PID_AXIS_YAW].kp   = PID_YAW_KP;
    s_axis[PID_AXIS_YAW].ki   = PID_YAW_KI;
    s_axis[PID_AXIS_YAW].kd   = PID_YAW_KD;

    for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
        pt1_init(&s_axis[i].dterm_lpf, PID_DTERM_LPF_HZ, s_dt);
    }

    pid_reset();
}

void pid_reset(void)
{
    for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
        s_axis[i].iterm = 0.0f;
        s_axis[i].prev_gyro = 0.0f;
        pt1_reset(&s_axis[i].dterm_lpf, 0.0f);
    }
}

void pid_update(const float sp_dps[PID_AXIS_COUNT],
                const float gyro_dps[PID_AXIS_COUNT],
                float throttle,
                float out[PID_AXIS_COUNT])
{
    if (sp_dps == 0 || gyro_dps == 0 || out == 0) {
        return;
    }

    for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
        pid_axis_t *ax = &s_axis[i];

        const float error = sp_dps[i] - gyro_dps[i];

        /* P */
        const float p = ax->kp * error;

        /* I: congelado y con decaimiento si el throttle esta abajo. */
        if (throttle > PID_ITERM_RELAX_THROTTLE) {
            ax->iterm += ax->ki * error * s_dt;
            ax->iterm = clampf(ax->iterm, PID_ITERM_LIMIT);
        } else {
            ax->iterm *= 0.98f;
        }

        /* D sobre el gyro filtrado, con signo invertido (d(error)/dt con
         * setpoint constante = -d(gyro)/dt). */
        const float gyro_f = pt1_apply(&ax->dterm_lpf, gyro_dps[i]);
        const float d = (ax->kd != 0.0f)
                      ? (-ax->kd * (gyro_f - ax->prev_gyro) * s_inv_dt)
                      : 0.0f;
        ax->prev_gyro = gyro_f;

        out[i] = clampf(p + ax->iterm + d, PID_OUTPUT_LIMIT);
    }
}

float pid_iterm(uint8_t axis)
{
    return (axis < PID_AXIS_COUNT) ? s_axis[axis].iterm : 0.0f;
}
