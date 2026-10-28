/**
 ******************************************************************************
 * @file    fc_telem.c
 * @brief   Tareas de telemetria CRSF (ver fc_telem.h).
 ******************************************************************************
 */

#include "fc_telem.h"
#include "fc_rc.h"
#include "fc_state.h"
#include "fc_tasks.h"

#include "arming.h"
#include "failsafe.h"
#include "pid.h"

#include "driver_crsf.h"
#include "sensors_battery.h"
#include "sensors_calib.h"
#include "telemtry.h"

/* ------------------------------------------------------------------------- */
/* Helpers de empaquetado                                                     */
/* ------------------------------------------------------------------------- */

static int16_t fc_telem_i16(float v, float scale)
{
    float s = v * scale;
    if (s > 32767.0f) {
        s = 32767.0f;
    } else if (s < -32768.0f) {
        s = -32768.0f;
    }
    /* Redondeo simetrico sin libm. */
    return (int16_t)((s >= 0.0f) ? (s + 0.5f) : (s - 0.5f));
}

static uint16_t fc_telem_u16_sat(uint32_t v)
{
    return (v > 0xFFFFu) ? 0xFFFFu : (uint16_t)v;
}

static uint8_t fc_telem_put16(uint8_t *buf, uint8_t idx, uint16_t v)
{
    buf[idx] = (uint8_t)((v >> 8) & 0xFFu);
    buf[idx + 1u] = (uint8_t)(v & 0xFFu);
    return (uint8_t)(idx + 2u);
}

/* ------------------------------------------------------------------------- */
/* Frames estandar                                                            */
/* ------------------------------------------------------------------------- */

void fc_telem_attitude(void)
{
    const fc_state_t *st = fc_state();
    /* Roll y pitch con el mismo signo que usa el lazo de nivel, para que el
     * horizonte muestre lo que ve el PID. */
    (void)crsf_send_attitude(FC_GYRO_PITCH_SIGN * st->attitude.pitch_rad,
                             FC_GYRO_ROLL_SIGN * st->attitude.roll_rad,
                             st->attitude.yaw_rad);
}

void fc_telem_baro(void)
{
    const fc_state_t *st = fc_state();
    if (!st->baro_si.valid) {
        return;
    }
    (void)crsf_send_baro_altitude(st->baro_si.alt_dm);
    (void)crsf_send_vario(st->baro_si.vspeed_cm_s);
}

/* Voltaje y % desde el ADC del FC (divisor en PB0). Corriente y mAh desde
 * la telemetria KISS del ESC, solo si hay frame valido.                  */
void fc_telem_battery(void)
{
    const sensors_battery_t *bat = sensors_battery_get();
    if (!bat->valid) {
        return;
    }

    const bool esc = g_telemLast.valid;
    (void)crsf_send_battery((uint16_t)(bat->voltage_cv / 10u),
                            esc ? (uint16_t)(g_telemLast.current_ca / 10u) : 0u,
                            esc ? g_telemLast.consumption_mah : 0u,
                            bat->percent);
}

/* Convencion Betaflight: '*' al final = desarmado. */
void fc_telem_flight_mode(void)
{
    const char *mode;

    if (!sensors_calib_done()) {
        mode = "CAL*";
    } else if (failsafe_active()) {
        mode = "!FS!";
    } else if (arming_is_armed()) {
        mode = FC_ENABLE_PID ? "ANGL" : "PASS";
    } else {
        mode = FC_ENABLE_PID ? "ANGL*" : "PASS*";
    }
    (void)crsf_send_flight_mode(mode);
}

/* Un solo cable de telemetria KISS: un valor de RPM y temperatura del ESC.
 * La segunda temperatura es la del barometro (placa).                   */
void fc_telem_esc(void)
{
    const fc_state_t *st = fc_state();

    if (g_telemLast.valid) {
        const int32_t rpm = (int32_t)g_telemLast.rpm;
        (void)crsf_send_rpm(0u, &rpm, 1u);
    }

    int16_t temps[2];
    uint8_t n = 0u;
    if (g_telemLast.valid) {
        temps[n++] = (int16_t)(g_telemLast.temperature_c * 10);
    }
    if (st->baro_si.valid) {
        temps[n++] = fc_telem_i16(st->baro_si.temp_c, 10.0f);
    }
    if (n > 0u) {
        (void)crsf_send_temp(0u, temps, n);
    }
}

/* ------------------------------------------------------------------------- */
/* Frame propio de debug                                                      */
/* ------------------------------------------------------------------------- */

/* Pagina 0: estado de vuelo (el contenido de la version 1). */
static uint8_t fc_telem_debug_page0(uint8_t *p)
{
    const fc_state_t *st = fc_state();
    const fc_rc_t *rc = fc_rc_get();
    const bool armed = arming_is_armed();

    uint8_t flags = 0u;
    flags |= armed                ? (1u << 0) : 0u;
    flags |= failsafe_active()    ? (1u << 1) : 0u;
    flags |= rc->link_ok          ? (1u << 2) : 0u;
    flags |= sensors_calib_done() ? (1u << 3) : 0u;
    flags |= FC_ENABLE_PID        ? (1u << 4) : 0u;
    flags |= st->has_baro         ? (1u << 5) : 0u;
    flags |= st->has_mag          ? (1u << 6) : 0u;
    flags |= g_telemLast.valid    ? (1u << 7) : 0u;

    uint8_t idx = 0u;

    p[idx++] = (uint8_t)FC_TELEM_DEBUG_VERSION;
    p[idx++] = 0u;
    p[idx++] = flags;
    p[idx++] = (uint8_t)(arming_disable_flags() & 0xFFu);
    p[idx++] = (uint8_t)failsafe_get_state();

    for (uint8_t ax = 0u; ax < PID_AXIS_COUNT; ++ax) {
        idx = fc_telem_put16(p, idx, (uint16_t)fc_telem_i16(rc->stick[ax], 1000.0f));
    }
    idx = fc_telem_put16(p, idx, (uint16_t)fc_telem_i16(rc->throttle, 1000.0f));

    for (uint8_t ax = 0u; ax < PID_AXIS_COUNT; ++ax) {
        idx = fc_telem_put16(p, idx, (uint16_t)fc_telem_i16(pid_setpoint_dps(ax), 10.0f));
    }
    for (uint8_t ax = 0u; ax < PID_AXIS_COUNT; ++ax) {
        idx = fc_telem_put16(p, idx, (uint16_t)fc_telem_i16(st->gyro_filt_dps[ax], 10.0f));
    }
    for (uint8_t ax = 0u; ax < PID_AXIS_COUNT; ++ax) {
        idx = fc_telem_put16(p, idx, (uint16_t)fc_telem_i16(st->pid_out[ax], 1000.0f));
    }
    for (uint8_t m = 0u; m < 4u; ++m) {
        idx = fc_telem_put16(p, idx, st->motor[m]);
    }

    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(st->loop.sched.pid_max_us));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(rc->age_ms));
    idx = fc_telem_put16(p, idx, (uint16_t)(st->gyro_errors & 0xFFFFu));
    idx = fc_telem_put16(p, idx, (uint16_t)(st->motor_drops & 0xFFFFu));

    return idx;
}

/* Pagina 1: cierra la ventana de estadisticas del lazo y la manda. */
static uint8_t fc_telem_debug_page1(uint8_t *p)
{
    fc_tasks_loop_stats_take();

    const fc_loop_stats_t *lp = &fc_state()->loop;
    const sched_stats_t *sc = &lp->sched;
    uint8_t idx = 0u;

    p[idx++] = (uint8_t)FC_TELEM_DEBUG_VERSION;
    p[idx++] = 1u;

    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(sc->window_us / 1000u));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(sc->gyro_samples));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(sc->pid_runs));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(sc->realtime_max_us));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(sc->pid_max_us));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(sc->pid_interval_max_us));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(sc->slow_task_max_us));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(sc->slow_runs));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(sc->slow_forced));
    p[idx++] = sc->slow_task_index;

    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(lp->drdy_edges));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(lp->drdy_late_events));
    idx = fc_telem_put16(p, idx, fc_telem_u16_sat(lp->drdy_missed));
    idx = fc_telem_put16(p, idx, (uint16_t)(lp->drdy_minus_samples & 0xFFFFu));

    /* Reservados para la Fase 2 (IMU por DMA). */
    for (uint8_t i = 0u; i < 4u; ++i) {
        idx = fc_telem_put16(p, idx, 0u);
    }

    return idx;
}

void fc_telem_debug(void)
{
    static uint8_t s_page = 0u;

    uint8_t p[FC_TELEM_DEBUG_LEN_P0];   /* la pagina mas larga */
    const uint8_t len = (s_page == 0u) ? fc_telem_debug_page0(p)
                                       : fc_telem_debug_page1(p);
    s_page ^= 1u;

    (void)crsf_send_ext_frame(CRSF_TYPE_FC_DEBUG, CRSF_ADDR_RADIO, p, len);
}
