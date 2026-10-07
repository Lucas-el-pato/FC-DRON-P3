/**
 ******************************************************************************
 * @file    pid.c
 * @brief   Implementacion del PID ANGLE + rate (ver pid.h).
 ******************************************************************************
 */

#include "pid.h"

typedef struct {
    float kp;             /* ya multiplicadas por las escalas de BF */
    float ki;
    float kd;
    float sum_limit;
    float iterm;
    float setpoint;       /* ultimo setpoint de rate (diagnostico)  */
    float dterm;          /* ultimo D (diagnostico)                 */
    float prev_gyro;      /* gyro del D-term de la iteracion previa */
    pt1_filter_t dterm_lpf1;
    pt1_filter_t dterm_lpf2;
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

static void pid_axis_set(pid_axis_t *ax, float p, float i, float d, float sum_limit)
{
    ax->kp = PID_GAIN_SCALE * PID_PTERM_SCALE * p;
    ax->ki = PID_GAIN_SCALE * PID_ITERM_SCALE * i;
    ax->kd = PID_GAIN_SCALE * PID_D_SCALE * PID_DTERM_SCALE * d;
    ax->sum_limit = sum_limit;
}

void pid_init(float dt_s)
{
    if (dt_s <= 0.0f) {
        dt_s = 1.0f / 2000.0f;
    }
    s_dt = dt_s;
    s_inv_dt = 1.0f / dt_s;

    pid_axis_set(&s_axis[PID_AXIS_ROLL],
                 PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, PID_SUM_LIMIT);
    pid_axis_set(&s_axis[PID_AXIS_PITCH],
                 PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, PID_SUM_LIMIT);
    pid_axis_set(&s_axis[PID_AXIS_YAW],
                 PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, PID_SUM_LIMIT_YAW);

    for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
        pt1_init(&s_axis[i].dterm_lpf1, PID_DTERM_LPF1_HZ, s_dt);
        pt1_init(&s_axis[i].dterm_lpf2, PID_DTERM_LPF2_HZ, s_dt);
    }

    pid_reset();
}

void pid_reset(void)
{
    for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
        s_axis[i].iterm = 0.0f;
        s_axis[i].setpoint = 0.0f;
        s_axis[i].dterm = 0.0f;
        s_axis[i].prev_gyro = 0.0f;
        pt1_reset(&s_axis[i].dterm_lpf1, 0.0f);
        pt1_reset(&s_axis[i].dterm_lpf2, 0.0f);
    }
}

/* pidLevel() de BF: stick -> angulo objetivo -> setpoint de rate. */
static float pid_level(float stick, float angle_deg)
{
    const float target_deg = clampf(stick, 1.0f) * PID_ANGLE_LIMIT_DEG;
    const float error_deg = target_deg - angle_deg;
    return clampf(error_deg * PID_LEVEL_GAIN, PID_LEVEL_MAX_RATE_DPS);
}

void pid_update(const float stick[PID_AXIS_COUNT],
                float yaw_sp_dps,
                const float angle_deg[2],
                const float gyro_dps[PID_AXIS_COUNT],
                float throttle,
                float out[PID_AXIS_COUNT])
{
    if (stick == 0 || angle_deg == 0 || gyro_dps == 0 || out == 0) {
        return;
    }

    float setpoint[PID_AXIS_COUNT];
    setpoint[PID_AXIS_ROLL] = pid_level(stick[PID_AXIS_ROLL], angle_deg[PID_AXIS_ROLL]);
    setpoint[PID_AXIS_PITCH] = pid_level(stick[PID_AXIS_PITCH], angle_deg[PID_AXIS_PITCH]);
    setpoint[PID_AXIS_YAW] = yaw_sp_dps;

    const bool iterm_active = (throttle > PID_ITERM_RELAX_THROTTLE);

    for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
        pid_axis_t *ax = &s_axis[i];

        ax->setpoint = setpoint[i];
        const float error = setpoint[i] - gyro_dps[i];

        /* P */
        const float p = ax->kp * error;

        /* I: con throttle abajo (sin airmode) se mantiene en 0. */
        if (iterm_active) {
            ax->iterm = clampf(ax->iterm + ax->ki * s_dt * error, PID_ITERM_LIMIT);
        } else {
            ax->iterm = 0.0f;
        }

        /* D sobre el gyro filtrado, con signo invertido (d(error)/dt con
         * setpoint constante = -d(gyro)/dt). */
        const float gyro_d = pt1_apply(&ax->dterm_lpf2,
                                       pt1_apply(&ax->dterm_lpf1, gyro_dps[i]));
        const float d = (ax->kd != 0.0f)
                      ? (-ax->kd * (gyro_d - ax->prev_gyro) * s_inv_dt)
                      : 0.0f;
        ax->prev_gyro = gyro_d;
        ax->dterm = d;

        const float sum = clampf(p + ax->iterm + d, ax->sum_limit);
        out[i] = sum / PID_MIXER_SCALING;
    }
}

float pid_iterm(uint8_t axis)
{
    return (axis < PID_AXIS_COUNT) ? s_axis[axis].iterm : 0.0f;
}

float pid_setpoint_dps(uint8_t axis)
{
    return (axis < PID_AXIS_COUNT) ? s_axis[axis].setpoint : 0.0f;
}

float pid_dterm(uint8_t axis)
{
    return (axis < PID_AXIS_COUNT) ? s_axis[axis].dterm : 0.0f;
}
