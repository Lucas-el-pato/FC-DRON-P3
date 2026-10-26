/**
 ******************************************************************************
 * @file    bal_pid.c
 * @brief   Implementacion del PID de un eje del balancin (ver bal_pid.h).
 ******************************************************************************
 */

#include "bal_pid.h"
#include "bal_config.h"
#include "filter.h"
#include "pid.h"     /* PID_PTERM/ITERM/DTERM_SCALE, PID_MIXER_SCALING */

static float s_kp;
static float s_ki;
static float s_kd;
static float s_dt = 1.0f / 2000.0f;
static float s_inv_dt = 2000.0f;

static float s_iterm;
static float s_setpoint;
static float s_pterm;
static float s_dterm;
static float s_prev_gyro;
static pt1_filter_t s_dterm_lpf1;
static pt1_filter_t s_dterm_lpf2;

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

void bal_pid_init(float dt_s)
{
    if (dt_s <= 0.0f) {
        dt_s = 1.0f / 2000.0f;
    }
    s_dt = dt_s;
    s_inv_dt = 1.0f / dt_s;

    s_kp = PID_PTERM_SCALE * BAL_KP;
    s_ki = PID_ITERM_SCALE * BAL_KI;
    s_kd = PID_DTERM_SCALE * BAL_KD;

    pt1_init(&s_dterm_lpf1, BAL_DTERM_LPF1_HZ, s_dt);
    pt1_init(&s_dterm_lpf2, BAL_DTERM_LPF2_HZ, s_dt);

    bal_pid_reset();
}

void bal_pid_reset(void)
{
    s_iterm = 0.0f;
    s_setpoint = 0.0f;
    s_pterm = 0.0f;
    s_dterm = 0.0f;
    s_prev_gyro = 0.0f;
    pt1_reset(&s_dterm_lpf1, 0.0f);
    pt1_reset(&s_dterm_lpf2, 0.0f);
}

float bal_pid_update(float ref_deg, float angle_deg, float rate_dps)
{
    /* Nivel: error de angulo -> setpoint de rate. */
    const float error_deg = clampf(ref_deg, BAL_REF_LIMIT_DEG) - angle_deg;
    s_setpoint = clampf(error_deg * BAL_LEVEL_GAIN, BAL_LEVEL_MAX_RATE_DPS);

    /* Rate. */
    const float error = s_setpoint - rate_dps;

    s_pterm = s_kp * error;
    s_iterm = clampf(s_iterm + s_ki * s_dt * error, BAL_ITERM_LIMIT);

    /* D sobre el gyro filtrado, signo invertido (setpoint constante). */
    const float gyro_d = pt1_apply(&s_dterm_lpf2, pt1_apply(&s_dterm_lpf1, rate_dps));
    s_dterm = (s_kd != 0.0f) ? (-s_kd * (gyro_d - s_prev_gyro) * s_inv_dt) : 0.0f;
    s_prev_gyro = gyro_d;

    const float sum = clampf(s_pterm + s_iterm + s_dterm, BAL_SUM_LIMIT);
    return sum / PID_MIXER_SCALING;
}

float bal_pid_setpoint_dps(void)
{
    return s_setpoint;
}

float bal_pid_pterm(void)
{
    return s_pterm;
}

float bal_pid_iterm(void)
{
    return s_iterm;
}

float bal_pid_dterm(void)
{
    return s_dterm;
}
